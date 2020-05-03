#ifndef LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYER_BASE_HPP
#define LAYERED_HARDWARE_GAZEBO_GAZEBO_LAYER_BASE_HPP

#include <string>

#include <hardware_interface/robot_hw.h>
#include <layered_hardware/layer_base.hpp>
#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>

#include <gazebo/physics/physics.hh>

#include <boost/shared_ptr.hpp>

namespace layered_hardware_gazebo {

class GazeboLayerBase : public lh::LayerBase {
public:
  // init() for gazebo layers
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str, const gzp::ModelPtr model) = 0;

protected:
  // disabled version of init() for non-gazebo layers
  virtual bool init(hi::RobotHW *const hw, const ros::NodeHandle &param_nh,
                    const std::string &urdf_str) {
    ROS_ERROR_STREAM("GazeboLayerBase::init(): Initialized as a normal layer. "
                     "Please initialize as a gazebo layer using an overloaded version of init()");
    return false;
  }
};

typedef boost::shared_ptr< GazeboLayerBase > GazeboLayerPtr;
typedef boost::shared_ptr< const GazeboLayerBase > GazeboLayerConstPtr;
} // namespace layered_hardware_gazebo

#endif