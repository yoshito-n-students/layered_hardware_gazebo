#ifndef LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP
#define LAYERED_HARDWARE_GAZEBO_EFFORT_MODE_HPP

#include <cmath>

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> //for RawJointData
#include <urdf/model.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/fpclassify.hpp> // for isnan()

namespace layered_hardware_gazebo {

class EffortMode : public OperationModeBase {
public:
  EffortMode(ti::RawJointData *const data, const urdf::Joint &desc)
      : OperationModeBase("effort", data),
        eff_lim_(desc.limits ? std::abs(desc.limits->effort) : 1e10) {}

  virtual ~EffortMode() {}

  virtual bool init(const ros::NodeHandle &param_nh) { return true; }

  virtual void starting() {
    // disable ODE's joint motor function or effort control does not work
    // (TODO: specialization for other physics engines)
    joint_->SetParam("fmax", 0, 0.);

    data_->effort_cmd = 0.;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    data_->position = Position(joint_, 0);
    data_->velocity = joint_->GetVelocity(0);
    data_->effort = joint_->GetForce(0);
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    namespace ba = boost::algorithm;
    namespace bm = boost::math;

    const double eff_cmd(ba::clamp(data_->effort_cmd, -eff_lim_, eff_lim_));
    if (!bm::isnan(eff_cmd)) {
      joint_->SetForce(0, eff_cmd);
    }
  }

  virtual void stopping() { joint_->SetForce(0, 0.); }

private:
  const double eff_lim_;
};
} // namespace layered_hardware_gazebo

#endif