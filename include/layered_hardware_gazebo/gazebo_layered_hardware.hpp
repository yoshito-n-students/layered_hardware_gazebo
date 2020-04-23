#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class GazeboLayeredHardware : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {}

private:
  void OnUpdate() {}

private:
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;
};
} // namespace gazebo

#endif