#ifndef LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

namespace layered_hardware_gazebo {

class EffortMode : public OperationModeBase {
public:
  EffortMode(ti::RawJointData *const data) : OperationModeBase("effort", data) {}

  virtual ~EffortMode() {}

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);
    
    data_->effort_cmd = 0.;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    joint_->SetForce(0, data_->effort_cmd);
  }

  virtual void stopping() {
    // nothing to do or zero effort ??
  }
};
} // namespace layered_hardware_gazebo

#endif