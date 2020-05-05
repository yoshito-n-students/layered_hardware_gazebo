#ifndef LAYERED_HARDWARE_GAZEBO_PASSIVE_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_PASSIVE_MODE_HPP

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

#include <gazebo/physics/physics.hh>

namespace layered_hardware_gazebo {

class PassiveMode : public OperationModeBase {
public:
  PassiveMode(const gzp::JointPtr joint) : OperationModeBase("passive", joint) {}

  virtual ~PassiveMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) {}

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    joint_->SetForce(0, 0.);
  }

  virtual void stopping() {}
};
} // namespace layered_hardware_gazebo

#endif