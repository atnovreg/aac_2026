#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace localization_display {
struct Sample { double t; std::array<double,6> pose; };
class Pose3DReplay : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    std::ifstream file(sdf->Get<std::string>("trajectory_file"));
    std::string line;
    if (!std::getline(file,line)) throw std::runtime_error("Empty3DposeCSV");
    while (std::getline(file,line)) {
      std::istringstream row(line); std::string field; std::vector<double> values;
      while(std::getline(row,field,',')) values.push_back(std::stod(field));
      if(values.size()!=7) throw std::runtime_error("3DposeCSVneeds7fields");
      for(double v:values) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite3Dpose");
      Sample sample{values[0],{values[1],values[2],values[3],values[4],values[5],values[6]}};
      if(!samples.empty() && sample.t<=samples.back().t) throw std::runtime_error("3Dpose times must increase");
      samples.push_back(sample);
    }
    if(samples.size()<2) throw std::runtime_error("Two3Dposes required");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const double elapsed=std::chrono::duration<double>(info.simTime).count();
    const double duration=samples.back().t-samples.front().t;
    const double t=std::fmod(std::max(0.,elapsed),duration)+samples.front().t;
    auto next=std::upper_bound(samples.begin(),samples.end(),t,
        [](double v,const Sample&s){return v<s.t;});
    if(next==samples.begin())++next;
    if(next==samples.end())--next;
    const auto&a=*(next-1);const auto&b=*next;
    const double w=std::clamp((t-a.t)/(b.t-a.t),0.,1.);
    std::array<double,6> pose;
    for(int i=0;i<6;++i)pose[i]=a.pose[i]+w*(b.pose[i]-a.pose[i]);
    model.SetWorldPoseCmd(ecm,gz::math::Pose3d(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]));
  }
 private:
  gz::sim::Model model{gz::sim::kNullEntity};std::vector<Sample>samples;
};
}
GZ_ADD_PLUGIN(localization_display::Pose3DReplay,gz::sim::System,
              localization_display::Pose3DReplay::ISystemConfigure,
              localization_display::Pose3DReplay::ISystemPreUpdate)
