#ifndef LAYERED_HARDWARE_GAZEBO_POSITION_PID_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_POSITION_PID_MODE_HPP

#include <control_toolbox/pid.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData

namespace layered_hardware_gazebo {

class PositionPIDMode : public OperationModeBase {
public:
  PositionPIDMode(ti::RawJointData *const data) : OperationModeBase("position_pid", data) {}

  virtual ~PositionPIDMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    return pid_.initParam(param_nh.resolveName("position_pid"));
  }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    data_->position = joint_->Position(0);
    data_->position_cmd = data_->position;

    pid_.reset();
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = joint_->Position(0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    const double pos_err(data_->position_cmd - joint_->Position(0));
    const double eff_cmd(pid_.computeCommand(pos_err, period));
    joint_->SetForce(0, eff_cmd);
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  control_toolbox::Pid pid_;
};
} // namespace layered_hardware_gazebo

#endif