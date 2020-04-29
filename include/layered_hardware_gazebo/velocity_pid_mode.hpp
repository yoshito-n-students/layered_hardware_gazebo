#ifndef LAYERED_HARDWARE_GAZEBO_VELOCITY_PID_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_VELOCITY_PID_MODE_HPP

#include <control_toolbox/pid.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

namespace layered_hardware_gazebo {

class VelocityPIDMode : public OperationModeBase {
public:
  VelocityPIDMode(ti::RawJointData *const data) : OperationModeBase("velocity_pid", data) {}

  virtual ~VelocityPIDMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    return pid_.initParam(param_nh.resolveName("velocity_pid"));
  }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    data_->velocity_cmd = 0.;

    pid_.reset();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    const double vel_err(data_->velocity_cmd - joint_->GetVelocity(0));
    const double eff_cmd(pid_.computeCommand(vel_err, period));
    joint_->SetForce(0, eff_cmd);
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  control_toolbox::Pid pid_;
};
} // namespace layered_hardware_gazebo

#endif