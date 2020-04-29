#ifndef LAYERED_HARDWARE_GAZEBO_OPERATION_MODE_BASE_HPP
#define LAYERED_HARDWARE_GAZEBO_OPERATION_MODE_BASE_HPP

#include <string>

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/transmission_interface_loader.h> // for RawJointData

#include <gazebo/physics/physics.hh>

#include <boost/shared_ptr.hpp>

namespace layered_hardware_gazebo {

class OperationModeBase {
public:
  OperationModeBase(const std::string &name, ti::RawJointData *const data)
      : name_(name), data_(data) {}

  virtual ~OperationModeBase() {}

  std::string getName() const { return name_; }

  void setGazeboJoint(const gzp::JointPtr joint) { joint_ = joint; }

  virtual bool init(const ros::NodeHandle &param_nh) = 0;

  virtual void starting() = 0;

  virtual void read(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void write(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stopping() = 0;

protected:
  const std::string name_;
  ti::RawJointData *data_;
  gzp::JointPtr joint_;
};

typedef boost::shared_ptr< OperationModeBase > OperationModePtr;
typedef boost::shared_ptr< const OperationModeBase > OperationModeConstPtr;
} // namespace layered_hardware_gazebo

#endif