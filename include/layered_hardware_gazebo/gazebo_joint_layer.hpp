#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_JOINT_LAYER_HPP

#include <list>
#include <string>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace layered_hardware_gazebo {

class GazeboJointLayer : public lh::LayerBase {
public:
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {}

  virtual bool prepareSwitch(const std::list< hi::ControllerInfo > &start_list,
                             const std::list< hi::ControllerInfo > &stop_list) {}

  virtual void doSwitch(const std::list< hi::ControllerInfo > &start_list,
                        const std::list< hi::ControllerInfo > &stop_list) {}

  virtual void read(const ros::Time &time, const ros::Duration &period) {}

  virtual void write(const ros::Time &time, const ros::Duration &period) {}
};
} // namespace layered_hardware_gazebo

#endif