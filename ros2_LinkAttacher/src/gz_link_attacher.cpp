#include "ros2_linkattacher/gz_link_attacher.hpp"

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Pose.hh>
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

static void ensureWorldPose(EntityComponentManager & ecm, Entity e)
{
    if (!ecm.Component<components::WorldPose>(e)) {
        ecm.CreateComponent(e, components::WorldPose());
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

    ensureWorldPose(ecm, l1);
    ensureWorldPose(ecm, m2);

    auto * l1Pose = ecm.Component<components::WorldPose>(l1);
    auto * m2Pose = ecm.Component<components::WorldPose>(m2);
    if (!l1Pose || !m2Pose) {
        // Poses not yet computed — re-queue and retry next step
        std::lock_guard<std::mutex> lk(impl_->opMtx);
        impl_->pendingOp = std::move(op);
        return;
    }

    impl_->gripperLinkEntity = l1;
    impl_->boxModelEntity    = m2;
    impl_->relativeTransform = l1Pose->Data().Inverse() * m2Pose->Data();
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
        ecm.RemoveComponent<components::WorldPoseCmd>(impl_->boxModelEntity);
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

    // Apply kinematic constraint every step while attached
    if (!impl_->attached) { return; }

    auto * gripperPose =
        ecm.Component<components::WorldPose>(impl_->gripperLinkEntity);
    if (!gripperPose) { return; }

    gz::math::Pose3d target = gripperPose->Data() * impl_->relativeTransform;

    auto * cmd = ecm.Component<components::WorldPoseCmd>(impl_->boxModelEntity);
    if (!cmd) {
        ecm.CreateComponent(impl_->boxModelEntity,
                            components::WorldPoseCmd(target));
    } else {
        cmd->Data() = target;
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
