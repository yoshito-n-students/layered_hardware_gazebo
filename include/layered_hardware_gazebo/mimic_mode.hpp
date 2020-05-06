#ifndef LAYERED_HARDWARE_GAZEBO_MIMIC_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_MIMIC_MODE_HPP

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

class MimicMode : public OperationModeBase {
public:
  MimicMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("mimic", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10),
        offset_(desc.mimic ? desc.mimic->offset : 0.),
        multiplier_(desc.mimic ? desc.mimic->multiplier : 1.),
        mimic_joint_name_(desc.mimic ? desc.mimic->joint_name : "") {}

  virtual ~MimicMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    // update from params
    offset_ = param(param_nh, "mimic/offset", offset_);
    multiplier_ = param(param_nh, "mimic/multiplier", multiplier_);
    mimic_joint_name_ = param(param_nh, "mimic/joint", mimic_joint_name_);

    // find the mimic joint
    mimic_joint_ = joint_->GetParent()->GetModel()->GetJoint(mimic_joint_name_);
    if (!mimic_joint_) {
      ROS_ERROR_STREAM("MimicMode::init(): Failed to get the joint '"
                       << mimic_joint_name_ << "' to mimic the joint '" << joint_->GetName()
                       << "'");
      return false;
    }

    return true;
  }

  virtual void starting(ti::RawJointData *const data) {
    // enable ODE's joint motor function for effort-based position control
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, eff_lim_);
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) {}

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    const double pos_cmd(multiplier_ * Position(*mimic_joint_, 0) + offset_);
    const double vel_cmd((pos_cmd - Position(*joint_, 0)) / period.toSec());
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
  double offset_, multiplier_;
  std::string mimic_joint_name_;
  gzp::JointPtr mimic_joint_;
};
} // namespace layered_hardware_gazebo

#endif