#include "ros2_linkattacher/gz_link_attacher.hpp"

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/PoseCmd.hh>

#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gz::sim::systems {

// ---------------------------------------------------------------------------
// Static entity-lookup helpers
// ---------------------------------------------------------------------------

static Entity findTopLevelModel(
    EntityComponentManager & ecm, const std::string & name)
{
    Entity result = kNullEntity;
    ecm.Each<components::Model, components::Name, components::ParentEntity>(
        [&](const Entity & e,
            const components::Model *,
            const components::Name * n,
            const components::ParentEntity * p) -> bool
        {
            if (n->Data() == name && ecm.Component<components::World>(p->Data())) {
                result = e;
                return false;
            }
            return true;
        });
    return result;
}

static bool isDescendant(
    EntityComponentManager & ecm, Entity e, Entity ancestor)
{
    auto * par = ecm.Component<components::ParentEntity>(e);
    while (par) {
        if (par->Data() == ancestor) { return true; }
        par = ecm.Component<components::ParentEntity>(par->Data());
    }
    return false;
}

static Entity findLinkUnder(
    EntityComponentManager & ecm,
    Entity modelEntity,
    const std::string & linkName)
{
    Entity result = kNullEntity;
    ecm.Each<components::Link, components::Name>(
        [&](const Entity & e,
            const components::Link *,
            const components::Name * n) -> bool
        {
            if (n->Data() == linkName && isDescendant(ecm, e, modelEntity)) {
                result = e;
                return false;
            }
            return true;
        });
    return result;
}

static void upsertWorldPoseCmd(
    EntityComponentManager & ecm, Entity e, const gz::math::Pose3d & pose)
{
    auto * cmd = ecm.Component<components::WorldPoseCmd>(e);
    if (!cmd) {
        ecm.CreateComponent(e, components::WorldPoseCmd(pose));
    } else {
        cmd->Data() = pose;
    }
}

// ---------------------------------------------------------------------------
// Pending operation
// ---------------------------------------------------------------------------

struct PendingOp {
    std::string model1, link1, model2, link2;
    bool isAttach{true};
    std::promise<std::pair<bool, std::string>> promise;
};

// ---------------------------------------------------------------------------
// Private implementation
// ---------------------------------------------------------------------------

struct GzLinkAttacherPrivate {
    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_srv;
    rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_srv;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread spin_thread;

    std::mutex opMtx;
    std::unique_ptr<PendingOp> pendingOp;

    // All-box lock state — populated on first PreUpdate; entity → locked world pose.
    // Every non-attached box is held at its entry here via WorldPoseCmd each step,
    // so physics settings (gravity, kp) cannot drift them. On detach the entry is
    // updated to the final placed pose so the box stays where it was put.
    bool initialized{false};
    std::unordered_map<Entity, gz::math::Pose3d> boxLockPoses;

    // Active attachments — each entry drives one box to follow one link every
    // PreUpdate. Multiple attachments are supported simultaneously (e.g. a box
    // on the gripper plus several boxes glued to the robot's cargo bin).
    struct Attachment {
        Entity linkEntity{kNullEntity};
        Entity boxEntity{kNullEntity};
        gz::math::Pose3d relativeTransform;
    };
    std::vector<Attachment> attachments;

    ~GzLinkAttacherPrivate()
    {
        if (executor) { executor->cancel(); }
        if (spin_thread.joinable()) { spin_thread.join(); }
    }
};

// ---------------------------------------------------------------------------
// GzLinkAttacher
// ---------------------------------------------------------------------------

GzLinkAttacher::GzLinkAttacher()
    : impl_(std::make_unique<GzLinkAttacherPrivate>()) {}

GzLinkAttacher::~GzLinkAttacher() = default;

void GzLinkAttacher::Configure(
    const Entity &,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &,
    EventManager &)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    impl_->ros_node = std::make_shared<rclcpp::Node>("gz_link_attacher");

    impl_->attach_srv =
        impl_->ros_node->create_service<linkattacher_msgs::srv::AttachLink>(
            "ATTACHLINK",
            [this](
                linkattacher_msgs::srv::AttachLink::Request::SharedPtr req,
                linkattacher_msgs::srv::AttachLink::Response::SharedPtr res)
            {
                auto op = std::make_unique<PendingOp>();
                op->model1   = req->model1_name;
                op->link1    = req->link1_name;
                op->model2   = req->model2_name;
                op->link2    = req->link2_name;
                op->isAttach = true;
                auto fut = op->promise.get_future();
                {
                    std::lock_guard<std::mutex> lk(impl_->opMtx);
                    impl_->pendingOp = std::move(op);
                }
                auto [ok, msg] = fut.get();
                res->success = ok;
                res->message = msg;
            });

    impl_->detach_srv =
        impl_->ros_node->create_service<linkattacher_msgs::srv::DetachLink>(
            "DETACHLINK",
            [this](
                linkattacher_msgs::srv::DetachLink::Request::SharedPtr req,
                linkattacher_msgs::srv::DetachLink::Response::SharedPtr res)
            {
                auto op = std::make_unique<PendingOp>();
                op->model1   = req->model1_name;
                op->link1    = req->link1_name;
                op->model2   = req->model2_name;
                op->link2    = req->link2_name;
                op->isAttach = false;
                auto fut = op->promise.get_future();
                {
                    std::lock_guard<std::mutex> lk(impl_->opMtx);
                    impl_->pendingOp = std::move(op);
                }
                auto [ok, msg] = fut.get();
                res->success = ok;
                res->message = msg;
            });

    impl_->executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    impl_->executor->add_node(impl_->ros_node);
    impl_->spin_thread = std::thread(
        [this] { impl_->executor->spin(); });

    RCLCPP_INFO(impl_->ros_node->get_logger(),
                "GzLinkAttacher ready (/ATTACHLINK, /DETACHLINK)");
}

// Process an attach request inside PreUpdate (has ECM access).
void GzLinkAttacher::processAttach(
    EntityComponentManager & ecm, std::unique_ptr<PendingOp> op)
{
    Entity m1 = findTopLevelModel(ecm, op->model1);
    if (m1 == kNullEntity) {
        op->promise.set_value({false, "Model not found: " + op->model1});
        return;
    }
    Entity l1 = findLinkUnder(ecm, m1, op->link1);
    if (l1 == kNullEntity) {
        op->promise.set_value({false, "Link not found: " + op->link1});
        return;
    }
    Entity m2 = findTopLevelModel(ecm, op->model2);
    if (m2 == kNullEntity) {
        op->promise.set_value({false, "Model not found: " + op->model2});
        return;
    }

    // gz::sim::worldPose() traverses the parent chain using Pose components,
    // which are updated for all entities including arm links after each physics
    // step — unlike WorldPose which is not reliably populated for articulated
    // robot links.
    gz::math::Pose3d l1WorldPose = gz::sim::worldPose(l1, ecm);
    gz::math::Pose3d m2WorldPose = gz::sim::worldPose(m2, ecm);

    GzLinkAttacherPrivate::Attachment a;
    a.linkEntity        = l1;
    a.boxEntity         = m2;
    a.relativeTransform = l1WorldPose.Inverse() * m2WorldPose;

    // If the same box is already attached somewhere, replace the existing entry
    // (a box can only follow one link at a time).
    bool replaced = false;
    for (auto & existing : impl_->attachments) {
        if (existing.boxEntity == m2) {
            existing = a;
            replaced = true;
            break;
        }
    }
    if (!replaced) {
        impl_->attachments.push_back(a);
    }

    op->promise.set_value({true,
        "Attached " + op->model2 + "::" + op->link2 +
        " to " + op->model1 + "::" + op->link1});
}

// Process a detach request inside PreUpdate.
void GzLinkAttacher::processDetach(
    EntityComponentManager & ecm, std::unique_ptr<PendingOp> op)
{
    Entity m2 = findTopLevelModel(ecm, op->model2);
    for (auto it = impl_->attachments.begin(); it != impl_->attachments.end(); ++it) {
        if (it->boxEntity == m2 && m2 != kNullEntity) {
            // Record the final placed pose so the box stays where the arm put it.
            gz::math::Pose3d linkPose = gz::sim::worldPose(it->linkEntity, ecm);
            gz::math::Pose3d finalPose = linkPose * it->relativeTransform;
            impl_->boxLockPoses[it->boxEntity] = finalPose;
            impl_->attachments.erase(it);
            op->promise.set_value({true,
                "Detached " + op->model2 + "::" + op->link2});
            return;
        }
    }
    op->promise.set_value({false,
        "Detach: no active attachment for " + op->model2});
}

void GzLinkAttacher::PreUpdate(
    const UpdateInfo &,
    EntityComponentManager & ecm)
{
    // On the first simulation step, lock every top-level model whose name
    // starts with "box_" at its current world pose via WorldPoseCmd.
    // This prevents any physics drift regardless of gravity/contact settings.
    if (!impl_->initialized) {
        ecm.Each<components::Model, components::Name, components::ParentEntity>(
            [&](const Entity & e,
                const components::Model *,
                const components::Name * n,
                const components::ParentEntity * p) -> bool
            {
                if (!ecm.Component<components::World>(p->Data())) { return true; }
                if (n->Data().rfind("box_", 0) != 0) { return true; }
                gz::math::Pose3d pose = gz::sim::worldPose(e, ecm);
                impl_->boxLockPoses[e] = pose;
                upsertWorldPoseCmd(ecm, e, pose);
                return true;
            });
        impl_->initialized = true;
    }

    // Drain any pending service request
    std::unique_ptr<PendingOp> op;
    {
        std::lock_guard<std::mutex> lk(impl_->opMtx);
        op = std::move(impl_->pendingOp);
    }

    if (op) {
        if (op->isAttach) {
            processAttach(ecm, std::move(op));
        } else {
            processDetach(ecm, std::move(op));
        }
    }

    // Build the set of currently-attached boxes so we can skip them in the
    // static-lock loop (their pose is driven by the attachment loop below).
    std::unordered_set<Entity> attachedEntities;
    attachedEntities.reserve(impl_->attachments.size());
    for (const auto & a : impl_->attachments) {
        attachedEntities.insert(a.boxEntity);
    }

    // Keep every non-attached box locked at its recorded pose.
    for (auto & [entity, pose] : impl_->boxLockPoses) {
        if (attachedEntities.count(entity)) { continue; }
        upsertWorldPoseCmd(ecm, entity, pose);
    }

    // Drive each attached box to follow its parent link.
    for (const auto & a : impl_->attachments) {
        gz::math::Pose3d linkPose = gz::sim::worldPose(a.linkEntity, ecm);
        gz::math::Pose3d target = linkPose * a.relativeTransform;
        upsertWorldPoseCmd(ecm, a.boxEntity, target);
    }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(
    gz::sim::systems::GzLinkAttacher,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::GzLinkAttacher,
    "gz::sim::systems::GzLinkAttacher")
