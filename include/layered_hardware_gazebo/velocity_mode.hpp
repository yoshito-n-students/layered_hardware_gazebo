#ifndef LAYERED_HARDWARE_GAZEBO_VELOCITY_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_VELOCITY_MODE_HPP

#include <cmath>

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <layered_hardware_gazebo/wrap.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData
#include <urdf/model.h>

#include <gazebo/physics/physics.hh>

#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_gazebo {

class VelocityMode : public OperationModeBase {
public:
  VelocityMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("velocity", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10) {}

  virtual ~VelocityMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting(ti::RawJointData *const data) {
    // enable ODE's joint motor function for effort-based velocity control
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, eff_lim_);

    vel_cmd_ = 0.;
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) { vel_cmd_ = data.velocity_cmd; }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    if (!boost::math::isnan(vel_cmd_)) {
      // use SetParam("vel") instead of SetVelocity()
      // to notify the desired velocity to the joint motor
      joint_->SetParam("vel", 0, vel_cmd_);
    }
  }

  virtual void stopping() {
    // disable ODE's joint motor function and zero effort
    joint_->SetParam("fmax", 0, 0.);
    joint_->SetForce(0, 0.);
  }

private:
  const double eff_lim_;
  double vel_cmd_;
};
} // namespace layered_hardware_gazebo

#endif