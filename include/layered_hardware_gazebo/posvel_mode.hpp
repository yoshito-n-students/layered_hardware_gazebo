#ifndef LAYERED_HARDWARE_GAZEBO_POSVEL_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_POSVEL_MODE_HPP

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

namespace layered_hardware_gazebo {

class PosVelMode : public OperationModeBase {
public:
  PosVelMode(ti::RawJointData *const data) : OperationModeBase("posvel", data) {}

  virtual ~PosVelMode() {}

  virtual void starting() {
    // enable ODE's joint motor function for effort-based position control
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 1e10);

    data_->position_cmd = joint_->Position(0);
    data_->velocity_cmd = 0.;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // TODO: saturate the position command based on profile velocity
    joint_->SetPosition(0, data_->position_cmd, true);
  }

  virtual void stopping() {
    // disable ODE's joint motor function and zero effort
    joint_->SetParam("fmax", 0, 0.);
    joint_->SetForce(0, 0.);
  }
};
} // namespace layered_hardware_gazebo

#endif