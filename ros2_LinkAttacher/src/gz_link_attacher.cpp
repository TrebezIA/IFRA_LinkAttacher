#include "ros2_linkattacher/gz_link_attacher.hpp"

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

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

    // Active attachment state — only accessed in PreUpdate (single-threaded)
    bool attached{false};
    Entity gripperLinkEntity{kNullEntity};
    Entity boxModelEntity{kNullEntity};
    gz::math::Pose3d relativeTransform;

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

    impl_->gripperLinkEntity = l1;
    impl_->boxModelEntity    = m2;
    impl_->relativeTransform = l1WorldPose.Inverse() * m2WorldPose;
    impl_->attached          = true;

    op->promise.set_value({true,
        "Attached " + op->model2 + "::" + op->link2 +
        " to " + op->model1 + "::" + op->link1});
}

// Process a detach request inside PreUpdate.
void GzLinkAttacher::processDetach(
    EntityComponentManager & ecm, std::unique_ptr<PendingOp> op)
{
    if (impl_->attached && impl_->boxModelEntity != kNullEntity) {
        // Record the final placed pose so the box stays where the arm put it.
        gz::math::Pose3d gripperPose = gz::sim::worldPose(impl_->gripperLinkEntity, ecm);
        gz::math::Pose3d finalPose = gripperPose * impl_->relativeTransform;
        impl_->boxLockPoses[impl_->boxModelEntity] = finalPose;
    }
    impl_->attached          = false;
    impl_->gripperLinkEntity = kNullEntity;
    impl_->boxModelEntity    = kNullEntity;
    op->promise.set_value({true,
        "Detached " + op->model2 + "::" + op->link2});
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

    // Keep every non-attached box locked at its recorded pose.
    for (auto & [entity, pose] : impl_->boxLockPoses) {
        if (impl_->attached && entity == impl_->boxModelEntity) { continue; }
        upsertWorldPoseCmd(ecm, entity, pose);
    }

    // Drive the attached box to follow the gripper link.
    if (!impl_->attached) { return; }

    gz::math::Pose3d gripperPose = gz::sim::worldPose(impl_->gripperLinkEntity, ecm);
    gz::math::Pose3d target = gripperPose * impl_->relativeTransform;
    upsertWorldPoseCmd(ecm, impl_->boxModelEntity, target);
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
