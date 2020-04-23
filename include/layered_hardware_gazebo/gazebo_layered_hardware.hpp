#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYERED_HARDWARE_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <controller_manager/controller_manager.h>
#include <layered_hardware/layered_hardware.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/gazebo_joint_layer.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <boost/bind/bind.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/scoped_ptr.hpp>

namespace gazebo {

class GazeboLayeredHardware : public ModelPlugin {
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {
    // ROS namespaces
    const std::string model_name(_model->GetName());
    ros::NodeHandle nh(model_name), pnh(ros::names::append("/layered_hardware_gazebo", model_name));

    // activate ros-control layers
    hw_.reset(new lh::LayeredHardware());
    if (!hw_->init(pnh)) {
      ROS_ERROR("Failed to init LayeredHardware");
      return;
    }

    // notify the gazebo model to gazebo-related layers
    for (std::size_t i = 0; i < hw_->size(); ++i) {
      // skip non-gazebo layers
      const lhg::GazeboJointLayerPtr layer(
          boost::dynamic_pointer_cast< lhg::GazeboJointLayer >(hw_->layer(i)));
      if (!layer) {
        continue;
      }

      // set the model to a layer found
      if (!layer->setGazeboModel(_model)) {
        ROS_ERROR_STREAM("Failed to set the Gazebo model '" << model_name
                                                            << "' to the layer [" << i << "]");
        return;
      }
    }

    // bind ros-controller manager to layers
    cm_.reset(new controller_manager::ControllerManager(hw_.get(), nh));

    // init timestamp
    prev_time_ = ros::Time::now();

    // schedule controllers' & layers' update on every simulation step
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboLayeredHardware::OnWorldUpdateBegin, this));
  }

private:
  void OnWorldUpdateBegin() {
    // TODO: set control frequency from ros-param
    const ros::Time time(ros::Time::now());
    const ros::Duration period(time - prev_time_);
    hw_->read(time, period);
    cm_->update(time, period);
    hw_->write(time, period);
    prev_time_ = time;
  }

private:
  boost::scoped_ptr< lh::LayeredHardware > hw_;
  boost::scoped_ptr< controller_manager::ControllerManager > cm_;
  ros::Time prev_time_;
  event::ConnectionPtr update_connection_;
};
} // namespace gazebo

#endif