#ifndef LAYERED_HARDWARE_GAZEBO_POSVEL_PID_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_POSVEL_PID_MODE_HPP

#include <cmath>

#include <control_toolbox/pid.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

#include <boost/algorithm/clamp.hpp>

namespace layered_hardware_gazebo {

class PosVelPIDMode : public OperationModeBase {
public:
  PosVelPIDMode(ti::RawJointData *const data) : OperationModeBase("posvel_pid", data) {}

  virtual ~PosVelPIDMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    return pid_.initParam(param_nh.resolveName("posvel_pid"));
  }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    data_->position = joint_->Position(0);
    data_->position_cmd = data_->position;
    data_->velocity_cmd = 0.;

    pid_.reset();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    namespace ba = boost::algorithm;

    // velocity required to realize the desired position in the next simulation step
    const double max_vel((data_->position_cmd - joint_->Position(0)) / period.toSec());
    // velocity limit
    const double vel_lim(std::abs(data_->velocity_cmd));
    // clamp the required velocity with the limits
    const double vel_cmd(ba::clamp(max_vel, -vel_lim, vel_lim));

    const double vel_err(vel_cmd - joint_->GetVelocity(0));
    const double eff_cmd(pid_.computeCommand(vel_err, period));
    joint_->SetForce(0, eff_cmd);
  }

  virtual void stopping() {
    // disable ODE's joint motor function and zero effort
    joint_->SetParam("fmax", 0, 0.);
    joint_->SetForce(0, 0.);
  }

private:
  control_toolbox::Pid pid_;
};
} // namespace layered_hardware_gazebo

#endif