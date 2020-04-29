#ifndef LAYERED_HARDWARE_GAZEBO_POSITION_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_POSITION_MODE_HPP

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_gazebo {

class PositionMode : public OperationModeBase {
public:
  PositionMode(ti::RawJointData *const data) : OperationModeBase("position", data) {}

  virtual ~PositionMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting() {
    // enable ODE's joint motor function for effort-based position control
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 1e10);

    data_->position = joint_->Position(0);
    data_->position_cmd = data_->position;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    const double vel_cmd((data_->position_cmd - joint_->Position(0)) / period.toSec());
    if (!boost::math::isnan(vel_cmd)) {
      // use SetParam("vel") instead of SetVelocity()
      // to notify the desired velocity to the joint motor
      joint_->SetParam("vel", 0, vel_cmd);
    }
  }

  virtual void stopping() {
    // disable ODE's joint motor function and zero effort
    joint_->SetParam("fmax", 0, 0.);
    joint_->SetForce(0, 0.);
  }
};
} // namespace layered_hardware_gazebo

#endif