#ifndef LAYERED_HARDWARE_GAZEBO_LAYERED_HARDWARE_GAZEBO_HPP
#define LAYERED_HARDWARE_GAZEBO_LAYERED_HARDWARE_GAZEBO_HPP

#include <list>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/gazebo_layer_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <gazebo/physics/physics.hh>

#include <boost/foreach.hpp>

namespace layered_hardware_gazebo {

class LayeredHardwareGazebo : public hi::RobotHW {
public:
  LayeredHardwareGazebo()
      : layer_loader_("layered_hardware", "layered_hardware::LayerBase"),
        gazebo_layer_loader_("layered_hardware_gazebo",
                             "layered_hardware_gazebo::GazeboLayerBase") {}

  virtual ~LayeredHardwareGazebo() {}

  bool init(const ros::NodeHandle &param_nh, const gzp::ModelPtr model) {
    namespace rn = ros::names;
    namespace rp = ros::param;

    // get URDF description from param
    std::string urdf_str;
    if (!param_nh.getParam("robot_description", urdf_str) &&
        !rp::get("robot_description", urdf_str)) {
      ROS_WARN_STREAM(
          "LayeredHardwareGazebo::init(): Failed to get URDF description from params neither '"
          << param_nh.resolveName("robot_description") << "' nor '"
          << rn::resolve("robot_description")
          << "'. Every layers will be initialized with empty string.");
      // continue because layers may not require robot description
    }

    // get layer names from param
    std::vector< std::string > layer_names;
    if (!param_nh.getParam("layers", layer_names)) {
      ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to get param '"
                       << param_nh.resolveName("layers") << "'");
      return false;
    }
    if (layer_names.empty()) {
      ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Param '" << param_nh.resolveName("layers")
                                                                << "' must be a string array");
      return false;
    }

    // load & init layer instances from bottom (actuator-side) to upper (controller-side)
    layers_.resize(layer_names.size());
    for (int i = layers_.size() - 1; i >= 0; --i) {
      lh::LayerPtr &layer(layers_[i]);
      const std::string &layer_name(layer_names[i]);
      const ros::NodeHandle layer_param_nh(param_nh, layer_name);

      // get layer's typename from param
      std::string lookup_name;
      if (!layer_param_nh.getParam("type", lookup_name)) {
        ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to get param '"
                         << layer_param_nh.resolveName("type") << "'");
        return false;
      }

      // load as a gazebo layer
      if (gazebo_layer_loader_.isClassAvailable(lookup_name)) {
        GazeboLayerPtr gazebo_layer;
        // create a gazebo layer instance by typename
        try {
          gazebo_layer = gazebo_layer_loader_.createInstance(lookup_name);
        } catch (const pluginlib::PluginlibException &ex) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to create a gazebo layer "
                           "instance by the lookup name '"
                           << lookup_name << "' for the gazebo layer '" << layer_name
                           << "': " << ex.what());
          return false;
        }
        // init the layer
        if (!gazebo_layer->init(this, layer_param_nh, urdf_str, model)) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to initialize the gazebo layer '"
                           << layer_name << "'");
          return false;
        }
        ROS_INFO_STREAM("LayeredHardwareGazebo::init(): Initialized the gazebo layer '"
                        << layer_name << "'");
        layer = gazebo_layer;
      }
      // load as a non-gazebo layer
      else if (layer_loader_.isClassAvailable(lookup_name)) {
        // create a layer instance by typename
        try {
          layer = layer_loader_.createInstance(lookup_name);
        } catch (const pluginlib::PluginlibException &ex) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to create a layer instance by "
                           "the lookup name '"
                           << lookup_name << "' for the layer '" << layer_name
                           << "': " << ex.what());
          return false;
        }
        // init the layer
        if (!layer->init(this, layer_param_nh, urdf_str)) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to initialize the layer '"
                           << layer_name << "'");
          return false;
        }
        ROS_INFO_STREAM("LayeredHardwareGazebo::init(): Initialized the layer '" << layer_name
                                                                                 << "'");
      }
      // no layer to load
      else {
        ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Unavailable lookup name '"
                         << lookup_name << "' for the layer '" << layer_name << "'");
        return false;
      }
    }

    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // ask each layers if stopping/starting given controllers is possible
    BOOST_REVERSE_FOREACH(const lh::LayerPtr &layer, layers_) {
      if (!layer->prepareSwitch(start_list, stop_list)) {
        return false;
      }
    }
    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // do something required on just before switching controllers
    BOOST_REVERSE_FOREACH(const lh::LayerPtr &layer, layers_) {
      layer->doSwitch(start_list, stop_list);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read states from actuators
    BOOST_REVERSE_FOREACH(const lh::LayerPtr &layer, layers_) { layer->read(time, period); }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write commands to actuators
    BOOST_FOREACH (const lh::LayerPtr &layer, layers_) { layer->write(time, period); }
  }

  std::size_t size() const { return layers_.size(); }

  lh::LayerPtr layer(const std::size_t i) { return layers_[i]; }

  lh::LayerConstPtr layer(const std::size_t i) const { return layers_[i]; }

private:
  pluginlib::ClassLoader< lh::LayerBase > layer_loader_;
  pluginlib::ClassLoader< GazeboLayerBase > gazebo_layer_loader_;
  std::vector< lh::LayerPtr > layers_;
};
} // namespace layered_hardware_gazebo

#endif