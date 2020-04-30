#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_DRIVER_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_DRIVER_HPP

#include <list>
#include <map>
#include <string>

#include <hardware_interface/controller_info.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/effort_mode.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <layered_hardware_gazebo/passive_mode.hpp>
#include <layered_hardware_gazebo/position_mode.hpp>
#include <layered_hardware_gazebo/position_pid_mode.hpp>
#include <layered_hardware_gazebo/posvel_mode.hpp>
#include <layered_hardware_gazebo/posvel_pid_mode.hpp>
#include <layered_hardware_gazebo/velocity_mode.hpp>
#include <layered_hardware_gazebo/velocity_pid_mode.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> // for RawJointData
#include <urdf/model.h>

#include <gazebo/physics/physics.hh>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_gazebo {

class GazeboJointDriver {
public:
  GazeboJointDriver() {}

  virtual ~GazeboJointDriver() {}

  bool init(const std::string &name, ti::RawJointData *const data, const ros::NodeHandle &param_nh,
            const urdf::Joint &desc) {
    name_ = name;

    typedef std::map< std::string, std::string > ModeNameMap;
    ModeNameMap mode_name_map;
    if (!param_nh.getParam("operation_mode_map", mode_name_map)) {
      ROS_ERROR_STREAM("GazeboJointDriver::init(): Failed to get param '"
                       << param_nh.resolveName("operation_mode_map") << "'");
      return false;
    }
    BOOST_FOREACH (const ModeNameMap::value_type &mode_name_kv, mode_name_map) {
      const std::string &controller_name(mode_name_kv.first);
      const std::string &mode_name(mode_name_kv.second);
      const OperationModePtr mode(makeOperationMode(mode_name, data, param_nh, desc));
      if (!mode) {
        ROS_ERROR_STREAM("GazeboJointDriver::init(): Failed to make operation mode '"
                         << mode_name << "' for the joint '" << name << "'");
        return false;
      }
      mode_map_[controller_name] = mode;
    }

    return true;
  }

  bool setGazeboModel(const gzp::ModelPtr model) {
    const gzp::JointPtr joint(model->GetJoint(name_));
    if (!joint) {
      ROS_ERROR_STREAM("GazeboJointDriver::setGazeboModel(): Failed to find the joint '"
                       << name_ << "' in the model '" << model->GetName() << "'");
      return false;
    }

    BOOST_FOREACH (ModeMap::value_type &mode_kv, mode_map_) {
      const OperationModePtr &mode(mode_kv.second);
      mode->setGazeboJoint(joint);
    }

    return true;
  }

  bool prepareSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                     const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // check if switching is possible by counting number of operation modes after switching

    // number of modes before switching
    std::size_t n_modes(present_mode_ ? 1 : 0);

    // number of modes after stopping controllers
    if (n_modes != 0) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const ModeMap::const_iterator mode_to_stop(mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          n_modes = 0;
          break;
        }
      }
    }

    // number of modes after starting controllers
    BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
      const ModeMap::const_iterator mode_to_start(mode_map_.find(starting_controller.name));
      if (mode_to_start != mode_map_.end() && mode_to_start->second) {
        ++n_modes;
      }
    }

    // assert 0 or 1 operation modes. multiple modes are impossible.
    if (n_modes != 0 && n_modes != 1) {
      ROS_ERROR_STREAM("GazeboJointDriver::prepareSwitch(): Rejected unfeasible controller "
                       "switching for the joint '"
                       << name_ << "'");
      return false;
    }

    return true;
  }

  void doSwitch(const std::list< hi::ControllerInfo > &starting_controller_list,
                const std::list< hi::ControllerInfo > &stopping_controller_list) {
    // stop joint's operation mode according to stopping controller list
    if (present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const ModeMap::const_iterator mode_to_stop(mode_map_.find(stopping_controller.name));
        if (mode_to_stop != mode_map_.end() && mode_to_stop->second == present_mode_) {
          ROS_INFO_STREAM("GazeboJointDriver::doSwitch(): Stopping the operation mode '"
                          << present_mode_->getName() << "' for the joint '" << name_ << "'");
          present_mode_->stopping();
          present_mode_ = OperationModePtr();
          break;
        }
      }
    }

    // start joint's operation mode according to starting controllers
    if (!present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
        const ModeMap::const_iterator mode_to_start(mode_map_.find(starting_controller.name));
        if (mode_to_start != mode_map_.end() && mode_to_start->second) {
          ROS_INFO_STREAM("GazeboJointDriver::doSwitch(): Starting the operating mode '"
                          << mode_to_start->second->getName() << "' for the joint '" << name_
                          << "'");
          present_mode_ = mode_to_start->second;
          present_mode_->starting();
          break;
        }
      }
    }
  }

  void read(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->read(time, period);
    }
  }

  void write(const ros::Time &time, const ros::Duration &period) {
    if (present_mode_) {
      present_mode_->write(time, period);
    }
  }

private:
  OperationModePtr makeOperationMode(const std::string &mode_str, ti::RawJointData *const data,
                                     const ros::NodeHandle &param_nh, const urdf::Joint &desc) {
    OperationModePtr mode;
    if (mode_str == "effort") {
      mode.reset(new EffortMode(data, desc));
    } else if (mode_str == "passive") {
      mode.reset(new PassiveMode(data));
    } else if (mode_str == "position") {
      mode.reset(new PositionMode(data, desc));
    } else if (mode_str == "position_pid") {
      mode.reset(new PositionPIDMode(data, desc));
    } else if (mode_str == "posvel") {
      mode.reset(new PosVelMode(data, desc));
    } else if (mode_str == "posvel_pid") {
      mode.reset(new PosVelPIDMode(data, desc));
    } else if (mode_str == "velocity") {
      mode.reset(new VelocityMode(data, desc));
    } else if (mode_str == "velocity_pid") {
      mode.reset(new VelocityPIDMode(data, desc));
    }
    if (!mode) {
      ROS_ERROR_STREAM("GazeboJointDriver::makeOperationMode(): Unknown operation mode name '"
                       << mode_str << " for the joint '" << name_ << "'");
      return OperationModePtr();
    }

    if (!mode->init(param_nh)) {
      ROS_ERROR_STREAM(
          "GazeboJointDriver::makeOperationMode(): Failed to initialize the operation mode '"
          << mode_str << " for the joint '" << name_ << "'");
      return OperationModePtr();
    }

    return mode;
  }

private:
  typedef std::map< std::string, OperationModePtr > ModeMap;

  std::string name_;
  ModeMap mode_map_;
  OperationModePtr present_mode_;
};

typedef boost::shared_ptr< GazeboJointDriver > GazeboJointDriverPtr;
typedef boost::shared_ptr< const GazeboJointDriver > GazeboJointDriverConstPtr;
} // namespace layered_hardware_gazebo

#endif