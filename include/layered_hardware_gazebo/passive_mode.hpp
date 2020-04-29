#ifndef LAYERED_HARDWARE_GAZEBO_PASSIVE_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_PASSIVE_MODE_HPP

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

namespace layered_hardware_gazebo {

class PassiveMode : public OperationModeBase {
public:
  PassiveMode(ti::RawJointData *const data) : OperationModeBase("passive", data) {}

  virtual ~PassiveMode() {}

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    joint_->SetForce(0, 0.);
  }

  virtual void stopping() {}
};
} // namespace layered_hardware_gazebo

#endif