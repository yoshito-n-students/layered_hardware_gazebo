#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_DRIVER_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_DRIVER_HPP

#include <list>
#include <map>
#include <string>

#include <hardware_interface/controller_info.h>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <layered_hardware_gazebo/effort_mode.hpp>
#include <layered_hardware_gazebo/fixed_mode.hpp>
#include <layered_hardware_gazebo/mimic_mode.hpp>
#include <layered_hardware_gazebo/mimic_pid_mode.hpp>
#include <layered_hardware_gazebo/operation_mode_base.hpp>
#include <layered_hardware_gazebo/passive_mode.hpp>
#include <layered_hardware_gazebo/position_mode.hpp>
#include <layered_hardware_gazebo/position_pid_mode.hpp>
#include <layered_hardware_gazebo/posvel_mode.hpp>
#include <layered_hardware_gazebo/posvel_pid_mode.hpp>
#include <layered_hardware_gazebo/velocity_mode.hpp>
#include <layered_hardware_gazebo/velocity_pid_mode.hpp>
#include <layered_hardware_gazebo/wrap.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> // for RawJointData
#include <urdf/model.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace layered_hardware_gazebo {

class GazeboJointDriver {
public:
  GazeboJointDriver() {}

  virtual ~GazeboJointDriver() {}

  bool init(const std::string &name, const ros::NodeHandle &param_nh, const urdf::Joint &desc,
            const gzp::JointPtr joint) {
    name_ = name;

    // make operation modes based on param
    typedef std::map< std::string, std::string > ControllerToModeName;
    ControllerToModeName controller_to_mode_name;
    if (!param_nh.getParam("operation_mode_map", controller_to_mode_name)) {
      ROS_ERROR_STREAM("GazeboJointDriver::init(): Failed to get param '"
                       << param_nh.resolveName("operation_mode_map") << "'");
      return false;
    }
    BOOST_FOREACH (const ControllerToModeName::value_type &kv, controller_to_mode_name) {
      const std::string &controller_name(kv.first);
      const std::string &mode_name(kv.second);
      const OperationModePtr mode(makeOperationMode(mode_name, param_nh, desc, joint));
      if (!mode) {
        ROS_ERROR_STREAM("GazeboJointDriver::init(): Failed to make operation mode '"
                         << mode_name << "' for the joint '" << name << "'");
        return false;
      }
      controller_name_to_mode[controller_name] = mode;
    }

    // update joint operation mode (embedded controller) on every simulation step
    last_update_time_ = SimTime(*joint->GetWorld());
    update_connection_ =
        gze::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboJointDriver::update, this, _1));
    if (!update_connection_) {
      ROS_ERROR_STREAM("GazeboJointDriver::init(): Failed to scheduling update of operation modes "
                       "for the joint '"
                       << name << "'");
      return false;
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
        const ControllerNameToMode::const_iterator mode_to_stop(
            controller_name_to_mode.find(stopping_controller.name));
        if (mode_to_stop != controller_name_to_mode.end() &&
            mode_to_stop->second == present_mode_) {
          n_modes = 0;
          break;
        }
      }
    }

    // number of modes after starting controllers
    BOOST_FOREACH (const hi::ControllerInfo &starting_controller, starting_controller_list) {
      const ControllerNameToMode::const_iterator mode_to_start(
          controller_name_to_mode.find(starting_controller.name));
      if (mode_to_start != controller_name_to_mode.end() && mode_to_start->second) {
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
                const std::list< hi::ControllerInfo > &stopping_controller_list,
                ti::RawJointData *const data) {
    // stop joint's operation mode according to stopping controller list
    if (present_mode_) {
      BOOST_FOREACH (const hi::ControllerInfo &stopping_controller, stopping_controller_list) {
        const ControllerNameToMode::const_iterator mode_to_stop(
            controller_name_to_mode.find(stopping_controller.name));
        if (mode_to_stop != controller_name_to_mode.end() &&
            mode_to_stop->second == present_mode_) {
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
        const ControllerNameToMode::const_iterator mode_to_start(
            controller_name_to_mode.find(starting_controller.name));
        if (mode_to_start != controller_name_to_mode.end() && mode_to_start->second) {
          ROS_INFO_STREAM("GazeboJointDriver::doSwitch(): Starting the operating mode '"
                          << mode_to_start->second->getName() << "' for the joint '" << name_
                          << "'");
          present_mode_ = mode_to_start->second;
          present_mode_->starting(data);
          break;
        }
      }
    }
  }

  void read(ti::RawJointData *const data) {
    if (present_mode_) {
      present_mode_->read(data);
    }
  }

  void write(ti::RawJointData &data) {
    if (present_mode_) {
      present_mode_->write(data);
    }
  }

  void update(const gzc::UpdateInfo &info) {
    const ros::Time time(info.simTime.sec, info.simTime.nsec);
    if (present_mode_) {
      present_mode_->update(time, time - last_update_time_);
    }
    last_update_time_ = time;
  }

private:
  OperationModePtr makeOperationMode(const std::string &mode_str, const ros::NodeHandle &param_nh,
                                     const urdf::Joint &desc, const gzp::JointPtr &joint) {
    OperationModePtr mode;
    if (mode_str == "effort") {
      mode.reset(new EffortMode(desc, joint));
    } else if (mode_str == "fixed") {
      mode.reset(new FixedMode(desc, joint));
    } else if (mode_str == "mimic") {
      mode.reset(new MimicMode(desc, joint));
    } else if (mode_str == "mimic_pid") {
      mode.reset(new MimicPIDMode(desc, joint));
    } else if (mode_str == "passive") {
      mode.reset(new PassiveMode(joint));
    } else if (mode_str == "position") {
      mode.reset(new PositionMode(desc, joint));
    } else if (mode_str == "position_pid") {
      mode.reset(new PositionPIDMode(desc, joint));
    } else if (mode_str == "posvel") {
      mode.reset(new PosVelMode(desc, joint));
    } else if (mode_str == "posvel_pid") {
      mode.reset(new PosVelPIDMode(desc, joint));
    } else if (mode_str == "velocity") {
      mode.reset(new VelocityMode(desc, joint));
    } else if (mode_str == "velocity_pid") {
      mode.reset(new VelocityPIDMode(desc, joint));
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
  typedef std::map< std::string, OperationModePtr > ControllerNameToMode;

  std::string name_;
  ControllerNameToMode controller_name_to_mode;
  OperationModePtr present_mode_;
  gze::ConnectionPtr update_connection_;
  ros::Time last_update_time_;
};

typedef boost::shared_ptr< GazeboJointDriver > GazeboJointDriverPtr;
typedef boost::shared_ptr< const GazeboJointDriver > GazeboJointDriverConstPtr;
} // namespace layered_hardware_gazebo

#endif