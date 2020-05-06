#ifndef LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP

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

class EffortMode : public OperationModeBase {
public:
  EffortMode(const urdf::Joint &desc, const gzp::JointPtr joint)
      : OperationModeBase("effort", joint),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10) {}

  virtual ~EffortMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting(ti::RawJointData *const data) {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    eff_cmd_ = 0.;
  }

  virtual void read(ti::RawJointData *const data) {
    data->position = Position(*joint_, 0);
    data->velocity = joint_->GetVelocity(0);
    data->effort = joint_->GetForce(0);
  }

  virtual void write(const ti::RawJointData &data) {
    eff_cmd_ = boost::algorithm::clamp(data.effort_cmd, -eff_lim_, eff_lim_);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    if (!boost::math::isnan(eff_cmd_)) {
      joint_->SetForce(0, eff_cmd_);
    }
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  const double eff_lim_;
  double eff_cmd_;
};
} // namespace layered_hardware_gazebo

#endif