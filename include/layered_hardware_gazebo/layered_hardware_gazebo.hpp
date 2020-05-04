#ifndef LAYERED_HARDWARE_GAZEBO_LAYERED_HARDWARE_GAZEBO_HPP
#define LAYERED_HARDWARE_GAZEBO_LAYERED_HARDWARE_GAZEBO_HPP

#include <string>
#include <vector>

#include <layered_hardware/layer_base.hpp>
#include <layered_hardware/layered_hardware.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/gazebo_layer_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>

#include <gazebo/physics/physics.hh>

namespace layered_hardware_gazebo {

class LayeredHardwareGazebo : public lh::LayeredHardware {
public:
  LayeredHardwareGazebo()
      : lh::LayeredHardware(), gazebo_layer_loader_("layered_hardware_gazebo",
                                                    "layered_hardware_gazebo::GazeboLayerBase") {}

  virtual ~LayeredHardwareGazebo() {}

  bool init(const ros::NodeHandle &param_nh, const gzp::ModelPtr model) {
    // get URDF description from param
    const std::string urdf_str(getURDFStr(param_nh));
    if (urdf_str.empty()) {
      ROS_WARN("LayeredHardwareGazebo::init(): Failed to get URDF description. "
               "Every layers will be initialized with empty string.");
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
        layer = loadGazeboLayer(lookup_name, layer_param_nh, urdf_str, model);
        if (!layer) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to load the gazebo layer '"
                           << layer_name << "'");
          return false;
        }
        ROS_INFO_STREAM("LayeredHardwareGazebo::init(): Initialized the gazebo layer '"
                        << layer_name << "'");
      }
      // load as a non-gazebo layer
      else if (layer_loader_.isClassAvailable(lookup_name)) {
        layer = loadLayer(lookup_name, layer_param_nh, urdf_str);
        if (!layer) {
          ROS_ERROR_STREAM("LayeredHardwareGazebo::init(): Failed to load the layer '" << layer_name
                                                                                       << "'");
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

protected:
  // override init() from the parent class to prevent calling
  bool init(const ros::NodeHandle &param_nh) {
    ROS_ERROR("LayeredHardwareGazebo::init(): Disabled version of init(). Call another version.");
    return false;
  }

  GazeboLayerPtr loadGazeboLayer(const std::string &lookup_name, const ros::NodeHandle &param_nh,
                                 const std::string &urdf_str, const gzp::ModelPtr &model) {
    GazeboLayerPtr layer;

    // create a gazebo layer instance by typename
    try {
      layer = gazebo_layer_loader_.createInstance(lookup_name);
    } catch (const pluginlib::PluginlibException &ex) {
      ROS_ERROR_STREAM("LayeredHardwareGazebo::loadGazeboLayer(): Failed to create a gazebo layer  "
                       "by the lookup name '"
                       << lookup_name << "': " << ex.what());
      return GazeboLayerPtr();
    }

    // init the layer
    if (!layer->init(this, param_nh, urdf_str, model)) {
      ROS_ERROR_STREAM(
          "LayeredHardwareGazebo::loadGazeboLayer(): Failed to initialize a gazebo layer");
      return GazeboLayerPtr();
    }

    return layer;
  }

protected:
  pluginlib::ClassLoader< GazeboLayerBase > gazebo_layer_loader_;
};
} // namespace layered_hardware_gazebo

#endif