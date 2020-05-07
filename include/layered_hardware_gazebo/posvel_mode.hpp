#ifndef LAYERED_HARDWARE_GAZEBO_POSVEL_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_POSVEL_MODE_HPP

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

#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_gazebo {

class PosVelMode : public OperationModeBase {
public:
  PosVelMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("posvel", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10) {}

  virtual ~PosVelMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting(ti::RawJointData *const data) {
    // enable ODE's joint motor function for effort-based position control
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, eff_lim_);

    const double pos(Position(*joint_, 0));
    // the latest position may be useful in the starting procedure of a pos-based controller
    data->position = pos;
    pos_goal_ = pos;
    pos_sp_ = pos;
    vel_prof_ = 0.;
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) {
    pos_goal_ = data.position_cmd;
    vel_prof_ = std::abs(data.velocity_cmd);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // update the position setpoint
    const double dpos_sp_lim(vel_prof_ * period.toSec());
    pos_sp_ += boost::algorithm::clamp(pos_goal_ - pos_sp_, -dpos_sp_lim, dpos_sp_lim);

    // track the position setpoint
    const double vel_cmd((pos_sp_ - Position(*joint_, 0)) / period.toSec());
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

private:
  const double eff_lim_;
  double pos_goal_, pos_sp_, vel_prof_;
};
} // namespace layered_hardware_gazebo

#endif