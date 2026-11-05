#include "Trajectory.hh"
#include <chrono>
#include <fstream>
#include <memory>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace replay
{
class Replay : public gz::sim::System,
               public gz::sim::ISystemConfigure,
               public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model = gz::sim::Model(entity);
    const auto path = sdf->Get<std::string>("trajectory_file");
    std::ifstream file(path);
    trajectory = std::make_unique<Trajectory>(file);
    speed = sdf->Get<double>("speed", 1.).first;
    floor = sdf->Get<gz::math::Vector3d>("floor_plane", gz::math::Vector3d::Zero).first;
    hold = sdf->Get<double>("hold_seconds", 2.).first;
    if (!std::isfinite(speed) || speed <= 0 || hold < 0)
      throw std::runtime_error("Invalid replay speed/hold");
    gzmsg << "Trajectory REPLAY: " << path << ", duration "
          << trajectory->Duration() << " s; vehicle physics is not evaluated.\n";
  }
  public: void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (!trajectory || info.paused) return;
    double elapsed = std::chrono::duration<double>(info.simTime).count()*speed;
    auto pose = trajectory->At(std::fmod(std::max(0., elapsed), trajectory->Duration()+hold));
    model.SetWorldPoseCmd(ecm, gz::math::Pose3d(pose.x, pose.y, floor.X()*pose.x + floor.Y()*pose.y + floor.Z() + .035, 0., 0., pose.yaw));
  }
  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: std::unique_ptr<Trajectory> trajectory;
  private: gz::math::Vector3d floor{0,0,0};
  private: double speed{1.}, hold{2.};
};
}

GZ_ADD_PLUGIN(replay::Replay, gz::sim::System,
              replay::Replay::ISystemConfigure, replay::Replay::ISystemPreUpdate)
