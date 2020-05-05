#ifndef LAYERED_HARDWARE_GAZEBO_MIMIC_PID_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_MIMIC_PID_MODE_HPP

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

class MimicPIDMode : public OperationModeBase {
public:
  MimicPIDMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("mimic_pid", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10),
        offset_(desc.mimic ? desc.mimic->offset : 0.),
        multiplier_(desc.mimic ? desc.mimic->multiplier : 1.),
        mimic_joint_name_(desc.mimic ? desc.mimic->joint_name : "") {}

  virtual ~MimicPIDMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) {
    // update from params
    if (!pid_.initParam(param_nh.resolveName("mimic_pid"))) {
      ROS_ERROR_STREAM("MimicPIDMode::init(): Failed to init PID params for the joint '"
                       << joint_->GetName() << "'");
      return false;
    }
    offset_ = param_nh.param("mimic_pid/offset", offset_);
    multiplier_ = param_nh.param("mimic_pid/multiplier", multiplier_);
    mimic_joint_name_ = param_nh.param("mimic_pid/joint", mimic_joint_name_);

    // find the mimic joint
    mimic_joint_ = joint_->GetParent()->GetModel()->GetJoint(mimic_joint_name_);
    if (!mimic_joint_) {
      ROS_ERROR_STREAM("MimicPIDMode::init(): Failed to get the joint '"
                       << mimic_joint_name_ << "' to mimic the joint '" << joint_->GetName()
                       << "'");
      return false;
    }

    return true;
  }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    pid_.reset();
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) {}

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    const double pos_cmd(multiplier_ * Position(mimic_joint_, 0) + offset_);
    const double pos_err(pos_cmd - Position(joint_, 0));
    const double eff_cmd(
        boost::algorithm::clamp(pid_.computeCommand(pos_err, period), -eff_lim_, eff_lim_));
    if (!boost::math::isnan(eff_cmd)) {
      joint_->SetForce(0, eff_cmd);
    }
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  const double eff_lim_;
  double offset_, multiplier_;
  std::string mimic_joint_name_;
  gzp::JointPtr mimic_joint_;
  control_toolbox::Pid pid_;
};
} // namespace layered_hardware_gazebo

#endif