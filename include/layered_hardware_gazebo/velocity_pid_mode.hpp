#ifndef LAYERED_HARDWARE_GAZEBO_VELOCITY_PID_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_VELOCITY_PID_MODE_HPP

#include <cmath>

#include <control_toolbox/pid.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData
#include <urdf/model.h>

#include <gazebo/physics/physics.hh>

#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_gazebo {

class VelocityPIDMode : public OperationModeBase {
public:
  VelocityPIDMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("velocity_pid", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10) {}

  virtual ~VelocityPIDMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    return pid_.initParam(param_nh.resolveName("velocity_pid"));
  }

  virtual void starting(ti::RawJointData *const data) {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    vel_cmd_ = 0.;

    pid_.reset();
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) { vel_cmd_ = data.velocity_cmd; }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    const double vel_err(vel_cmd_ - joint_->GetVelocity(0));
    const double eff_cmd(
        boost::algorithm::clamp(pid_.computeCommand(vel_err, period), -eff_lim_, eff_lim_));
    if (!boost::math::isnan(eff_cmd)) {
      joint_->SetForce(0, eff_cmd);
    }
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  const double eff_lim_;
  double vel_cmd_;
  control_toolbox::Pid pid_;
};
} // namespace layered_hardware_gazebo

#endif