#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <controller_manager/controller_manager.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/layered_hardware_gazebo.hpp>
#include <layered_hardware_gazebo/wrap.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/bind/bind.hpp>
#include <boost/scoped_ptr.hpp>

namespace gazebo {

class GazeboLayeredHardware : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) {
    // ROS namespaces
    const std::string model_name(model->GetName());
    ros::NodeHandle nh(model_name), pnh(ros::names::append(model_name, "layered_hardware_gazebo"));

    // activate ros-control layers
    hw_.reset(new lhg::LayeredHardwareGazebo());
    if (!hw_->init(pnh, model)) {
      ROS_ERROR("GazeboLayeredHardware::Load(): Failed to init LayeredHardware");
      return;
    }

    // bind ros-controller manager to layers
    cm_.reset(new controller_manager::ControllerManager(hw_.get(), nh));

    // schedule controllers' & layers' update
    const double control_frequency(pnh.param("control_frequency", 10.));
    update_period_ = ros::Rate(control_frequency).expectedCycleTime();
    const ros::Time now(lhg::SimTime(model->GetWorld()));
    last_update_time_ = (now.toNSec() >= update_period_.toNSec())
                            ? now - update_period_
                            : ros::Time(0, 0); // ros::Time cannot be negative
    next_update_time_ = now;
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboLayeredHardware::update, this, _1));

    ROS_INFO_STREAM(
        "GazeboLayeredHardware::Load(): Started updating LayeredHardware for the model '"
        << model_name << "' at " << control_frequency << " Hz");
  }

private:
  void update(const common::UpdateInfo &info) {
    // check the scheduled time has come
    const ros::Time time(info.simTime.sec, info.simTime.nsec);
    if (time < next_update_time_) {
      return;
    }

    // update the hardware and controllers
    const ros::Duration period(time - last_update_time_);
    hw_->read(time, period);
    cm_->update(time, period);
    hw_->write(time, period);
    last_update_time_ = time;

    // schedule the next update
    next_update_time_ += update_period_;
  }

private:
  boost::scoped_ptr< lhg::LayeredHardwareGazebo > hw_;
  boost::scoped_ptr< controller_manager::ControllerManager > cm_;
  ros::Time last_update_time_, next_update_time_;
  ros::Duration update_period_;
  event::ConnectionPtr update_connection_;
};
} // namespace gazebo

#endif