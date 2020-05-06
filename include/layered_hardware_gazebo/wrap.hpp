#ifndef LAYERED_HARDWARE_GAZEBO_WRAP_HPP
#define LAYERED_HARDWARE_GAZEBO_WRAP_HPP

#include <string>

#include <layered_hardware_gazebo/common_namespaces.hpp>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/time.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace layered_hardware_gazebo {

// as of Kinetic, [val NodeHandle::param(key, default_val)] is not a const method
// although [void NodeHandle::param(key, val_ref, default_val)] is.
// this offers the former signature to const NodeHandle.
template < typename T >
static T param(const ros::NodeHandle &nh, const std::string &key, const T &default_val) {
  T val;
  nh.param(key, val, default_val);
  return val;
}

static inline double Position(const gzp::Joint &joint, const unsigned int index) {
#if GAZEBO_MAJOR_VERSION >= 8
  return joint.Position(index);
#else
  return *(joint.GetAngle(index));
#endif
}

static inline ros::Time SimTime(const gzp::World &world) {
#if GAZEBO_MAJOR_VERSION >= 8
  const gzc::Time time(world.SimTime());
#else
  const gzc::Time time(world.GetSimTime());
#endif
  return ros::Time(time.sec, time.nsec);
}
} // namespace layered_hardware_gazebo

#endif