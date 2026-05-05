#pragma once

#include <memory>
#include <gz/sim/System.hh>

namespace gz::sim::systems {

class GzLinkAttacherPrivate;

class GzLinkAttacher
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
    GzLinkAttacher();
    ~GzLinkAttacher() override;

    void Configure(
        const gz::sim::Entity & _entity,
        const std::shared_ptr<const sdf::Element> & _sdf,
        gz::sim::EntityComponentManager & _ecm,
        gz::sim::EventManager & _eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo & _info,
        gz::sim::EntityComponentManager & _ecm) override;

private:
    void processAttach(gz::sim::EntityComponentManager & ecm,
                       std::unique_ptr<struct PendingOp> op);
    void processDetach(gz::sim::EntityComponentManager & ecm,
                       std::unique_ptr<struct PendingOp> op);

    std::unique_ptr<GzLinkAttacherPrivate> impl_;
};

}  // namespace gz::sim::systems
