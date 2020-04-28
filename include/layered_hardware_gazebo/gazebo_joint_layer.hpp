#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP

#include <list>
#include <set>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/gazebo_joint_driver.hpp>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> // for RequisiteProvider
#include <transmission_interface/transmission_parser.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <gazebo/physics/physics.hh>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_gazebo {

class GazeboJointLayer : public lh::LayerBase {
public:
  GazeboJointLayer()
      : joint_iface_provider_loader_("transmission_interface",
                                     "transmission_interface::RequisiteProvider") {}

  virtual ~GazeboJointLayer() {}

  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    ////////////////////////////////////////////////
    // 1. register joint interfaces to the hardware

    // get transmission info, which contains info of joints' hardware interface, from URDF
    std::vector< ti::TransmissionInfo > trans_infos;
    if (!ti::TransmissionParser::parse(urdf_str, trans_infos)) {
      ROS_ERROR("GazeboJointLayer::init(): Failed to parse transmissions from URDF");
      return false;
    }

    // map transmissions by joints' hardware interface types
    typedef std::map< std::string, std::set< const ti::TransmissionInfo * > > TransmissionMap;
    TransmissionMap trans_map;
    BOOST_FOREACH (const ti::TransmissionInfo &trans_info, trans_infos) {
      BOOST_FOREACH (const ti::JointInfo &joint_info, trans_info.joints_) {
        BOOST_FOREACH (const std::string &joint_iface_type, joint_info.hardware_interfaces_) {
          trans_map[joint_iface_type].insert(&trans_info);
        }
      }
    }

    // make joint hardware interfaces by using provider plugins to support any types of interfaces
    BOOST_FOREACH (const TransmissionMap::value_type &trans_map_kv, trans_map) {
      // find a provider class for the joint hardware interface type
      const std::string &joint_iface_type(trans_map_kv.first);
      const boost::shared_ptr< ti::RequisiteProvider > joint_iface_provider(
          joint_iface_provider_loader_.createInstance(joint_iface_type));
      if (!joint_iface_provider) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to find a provider for the joint "
                         "hardware interface type '"
                         << joint_iface_type << "'");
        return false;
      }

      // let the provider update hardware interfaces
      const std::set< const ti::TransmissionInfo * > &trans_set(trans_map_kv.second);
      BOOST_FOREACH (const ti::TransmissionInfo *trans_info, trans_set) {
        if (!joint_iface_provider->updateJointInterfaces(*trans_info, hw, joint_ifaces_,
                                                         joint_data_map_)) {
          ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to provide the hardware interface of '"
                           << joint_iface_type << "' to joints in the transmission '"
                           << trans_info->name_ << "'");
          return false;
        }
        ROS_INFO_STREAM("GazeboJointLayer::init(): Updated the joint hardware interface '"
                        << joint_iface_type << "' in the transmission '" << trans_info->name_
                        << "'");
      }
    }

    /////////////////////////
    // 2. init joint drivers

    // get list of joints to be handled by this layer from params
    XmlRpc::XmlRpcValue joints_param;
    if (!param_nh.getParam("joints", joints_param)) {
      ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to get param '"
                       << param_nh.resolveName("joints") << "'");
      return false;
    }
    if (joints_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM("GazeboJointLayer::init(): Param '" << param_nh.resolveName("joints")
                                                           << "' must be a struct");
      return false;
    }

    // init joint drivers
    // (could not use BOOST_FOREACH here to avoid a bug in the library in Kinetic)
    for (XmlRpc::XmlRpcValue::iterator joint_param = joints_param.begin();
         joint_param != joints_param.end(); ++joint_param) {
      // find joint data associated with hardware interfaces
      const std::string &joint_name(joint_param->first);
      ti::RawJointDataMap::iterator joint_data_kv(joint_data_map_.find(joint_name));
      if (joint_data_kv == joint_data_map_.end()) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to find data for the joint '"
                         << joint_name << "'");
        return false;
      }

      // init joint drivers with external data
      const GazeboJointDriverPtr joint_driver(new GazeboJointDriver());
      ti::RawJointData &joint_data(joint_data_kv->second);
      ros::NodeHandle joint_param_nh(param_nh, ros::names::append("joints", joint_name));
      if (!joint_driver->init(joint_name, &joint_data, joint_param_nh)) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to init a driver for the joint '"
                         << joint_name << "'");
        return false;
      }
      ROS_INFO_STREAM("GazeboJointLayer::init(): Initialized the gazebo joint '" << joint_name
                                                                                 << "'");
      joint_drivers_.push_back(joint_driver);
    }

    return true;
  }

  bool setGazeboModel(const gzp::ModelPtr model) {
    BOOST_FOREACH (const GazeboJointDriverPtr &driver, joint_drivers_) {
      if (!driver->setGazeboModel(model)) {
        return false;
      }
    }
    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // ask to all joint drivers if controller switching is possible
    BOOST_FOREACH (const GazeboJointDriverPtr &driver, joint_drivers_) {
      if (!driver->prepareSwitch(start_list, stop_list)) {
        return false;
      }
    }
    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // notify controller switching to all joint drivers
    BOOST_FOREACH (const GazeboJointDriverPtr &driver, joint_drivers_) {
      driver->doSwitch(start_list, stop_list);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read from all joint drivers
    BOOST_FOREACH (const GazeboJointDriverPtr &driver, joint_drivers_) {
      driver->read(time, period);
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all joint drivers
    BOOST_FOREACH (const GazeboJointDriverPtr &driver, joint_drivers_) {
      driver->write(time, period);
    }
  }

private:
  pluginlib::ClassLoader< ti::RequisiteProvider > joint_iface_provider_loader_;
  ti::JointInterfaces joint_ifaces_;
  ti::RawJointDataMap joint_data_map_;
  std::vector< GazeboJointDriverPtr > joint_drivers_;
};

typedef boost::shared_ptr< GazeboJointLayer > GazeboJointLayerPtr;
typedef boost::shared_ptr< const GazeboJointLayer > GazeboJointLayerConstPtr;
} // namespace layered_hardware_gazebo

#endif