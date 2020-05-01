#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP

#include <list>
#include <map>
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
#include <urdf/model.h>
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
    /////////////////////////////////////////////////////////////////
    // 1. get list of joints to be handled by this layer from params

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

    ////////////////////////////////////////////////
    // 2. register joint interfaces to the hardware

    // populate joint info from URDF
    typedef std::map< std::string, ti::JointInfo > JointNameToInfo;
    JointNameToInfo joint_name_to_info;
    {
      // get transmission info, which contains joint info, from URDF
      std::vector< ti::TransmissionInfo > trans_infos;
      if (!ti::TransmissionParser::parse(urdf_str, trans_infos)) {
        ROS_ERROR("GazeboJointLayer::init(): Failed to parse transmissions from URDF");
        return false;
      }

      // organize joint info in transmission info by joint names
      BOOST_FOREACH (const ti::TransmissionInfo &trans_info, trans_infos) {
        BOOST_FOREACH (const ti::JointInfo &joint_info_src, trans_info.joints_) {
          // skip non-target joints
          if (!joints_param.hasMember(joint_info_src.name_)) {
            continue;
          }

          // update organized joint info
          ti::JointInfo &joint_info_dst(joint_name_to_info[joint_info_src.name_]);
          if (joint_info_dst.name_.empty()) {
            joint_info_dst.name_ = joint_info_src.name_;
          }
          std::vector< std::string > &iface_types_dst(joint_info_dst.hardware_interfaces_);
          BOOST_FOREACH (const std::string &iface_type_src, joint_info_src.hardware_interfaces_) {
            if (std::find(iface_types_dst.begin(), iface_types_dst.end(), iface_type_src) !=
                iface_types_dst.end()) {
              continue;
            }
            iface_types_dst.push_back(iface_type_src);
          }
          if (joint_info_dst.xml_element_.empty()) {
            joint_info_dst.xml_element_ = joint_info_src.xml_element_;
          }
        }
      }
    }

    // organize joint infos by interface types
    typedef std::map< std::string, std::vector< ti::JointInfo > > IfaceTypeToJointInfos;
    IfaceTypeToJointInfos iface_type_to_joint_infos;
    BOOST_FOREACH (const JointNameToInfo::value_type &kv, joint_name_to_info) {
      const ti::JointInfo &joint_info(kv.second);
      BOOST_FOREACH (const std::string &iface_type, joint_info.hardware_interfaces_) {
        iface_type_to_joint_infos[iface_type].push_back(joint_info);
      }
    }

    // make joint hardware interfaces by using provider plugins to support any types of interfaces
    BOOST_FOREACH (const IfaceTypeToJointInfos::value_type &kv, iface_type_to_joint_infos) {
      const std::string &iface_type(kv.first);
      const std::vector< ti::JointInfo > &joint_infos(kv.second);

      // find a provider class for the joint hardware interface type
      const boost::shared_ptr< ti::RequisiteProvider > iface_provider(
          joint_iface_provider_loader_.createInstance(iface_type));
      if (!iface_provider) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to find a provider for the joint "
                         "hardware interface type '"
                         << iface_type << "'");
        return false;
      }

      // let the provider update hardware interfaces
      BOOST_FOREACH (const ti::JointInfo &joint_info, joint_infos) {
        ti::TransmissionInfo trans_info;
        trans_info.joints_.push_back(joint_info);
        if (!iface_provider->updateJointInterfaces(trans_info, hw, joint_ifaces_,
                                                   joint_name_to_data_)) {
          ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to provide the hardware interface of '"
                           << iface_type << "' for the joint '" << joint_info.name_ << "'");
          return false;
        }
        ROS_INFO_STREAM("GazeboJointLayer::init(): Updated the joint hardware interface '"
                        << iface_type << "' for the joint '" << joint_info.name_ << "'");
      }
    }

    /////////////////////////////////////
    // 3. build robot skeleton from URDF

    urdf::Model robot_desc;
    if (!robot_desc.initString(urdf_str)) {
      ROS_ERROR("GazeboJointLayer::init(): Failed to build a robot skeleton from URDF");
      return false;
    }

    /////////////////////////
    // 4. init joint drivers

    // (could not use BOOST_FOREACH here to avoid a bug in the library in Kinetic)
    for (XmlRpc::XmlRpcValue::iterator joint_param = joints_param.begin();
         joint_param != joints_param.end(); ++joint_param) {
      const std::string &joint_name(joint_param->first);

      // find joint data associated with hardware interfaces
      if (joint_name_to_data_.count(joint_name) == 0) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to find data for the joint '"
                         << joint_name << "'");
        return false;
      }
      ti::RawJointData *const joint_data(&joint_name_to_data_[joint_name]);

      // find joint description
      const urdf::JointConstSharedPtr joint_desc(robot_desc.getJoint(joint_name));
      if (!joint_desc) {
        ROS_ERROR_STREAM(
            "GazeboJointLayer::init(): Failed to find description in URDF for the joint '"
            << joint_name << "'");
        return false;
      }

      // init joint drivers with external data
      const GazeboJointDriverPtr joint_driver(new GazeboJointDriver());
      ros::NodeHandle joint_param_nh(param_nh, ros::names::append("joints", joint_name));
      if (!joint_driver->init(joint_name, joint_param_nh, *joint_desc)) {
        ROS_ERROR_STREAM("GazeboJointLayer::init(): Failed to init a driver for the joint '"
                         << joint_name << "'");
        return false;
      }
      ROS_INFO_STREAM("GazeboJointLayer::init(): Initialized the gazebo joint '" << joint_name
                                                                                 << "'");
      joint_data_to_driver_[joint_data] = joint_driver;
    }

    return true;
  }

  bool setGazeboModel(const gzp::ModelPtr model) {
    BOOST_FOREACH (const JointDataToDriver::value_type &kv, joint_data_to_driver_) {
      const GazeboJointDriverPtr &driver(kv.second);
      if (!driver->setGazeboModel(model)) {
        return false;
      }
    }
    return true;
  }

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {
    // ask to all joint drivers if controller switching is possible
    BOOST_FOREACH (const JointDataToDriver::value_type &kv, joint_data_to_driver_) {
      const GazeboJointDriverPtr &driver(kv.second);
      if (!driver->prepareSwitch(start_list, stop_list)) {
        return false;
      }
    }
    return true;
  }

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {
    // notify controller switching to all joint drivers
    BOOST_FOREACH (const JointDataToDriver::value_type &kv, joint_data_to_driver_) {
      const GazeboJointDriverPtr &driver(kv.second);
      driver->doSwitch(start_list, stop_list);
    }
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read from all joint drivers
    BOOST_FOREACH (const JointDataToDriver::value_type &kv, joint_data_to_driver_) {
      ti::RawJointData *const data(kv.first);
      const GazeboJointDriverPtr &driver(kv.second);
      driver->read(data);
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // write to all joint drivers
    BOOST_FOREACH (const JointDataToDriver::value_type &kv, joint_data_to_driver_) {
      ti::RawJointData *const data(kv.first);
      const GazeboJointDriverPtr &driver(kv.second);
      driver->write(*data);
    }
  }

private:
  typedef std::map< ti::RawJointData *, GazeboJointDriverPtr > JointDataToDriver;

  pluginlib::ClassLoader< ti::RequisiteProvider > joint_iface_provider_loader_;
  ti::JointInterfaces joint_ifaces_;
  ti::RawJointDataMap joint_name_to_data_;
  JointDataToDriver joint_data_to_driver_;
};

typedef boost::shared_ptr< GazeboJointLayer > GazeboJointLayerPtr;
typedef boost::shared_ptr< const GazeboJointLayer > GazeboJointLayerConstPtr;
} // namespace layered_hardware_gazebo

#endif