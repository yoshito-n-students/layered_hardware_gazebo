# layered_hardware_gazebo
A layered_hardware implementation for the Gazebo simulator, which contains 2 types of plugins;

* GazeboLayeredHardware: a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)
* gazebo_joint_layer: a ros_control layer plugin running in the node above and driving joints in the virtual robot

See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware) to understand the layered scheme.

## GazeboLayeredHardware
* a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)

### <u>Parameters (should be defined under /layered_hardware_gazebo/<robot_name>)</u>
___robot_description___ or ___<node_namespace>/robot_description___ (string, default: "")
* robot description in URDF
* if both given, ___~robot_description___ will be used

___layers___ (string array, required)
* names of layers from upper (controller-side) to bottom (actuator-side)

___<layer_name>/type___ (string, required)
* lookup name for each layer plugin like 'layered_hardware/JointLimitsLayer' or 'layered_hardware_gazebo/GazeboJointLayer'

## layered_hardware_gazebo/GazeboJointLayer
* a ros_control layer plugin for GazeboLayeredHardware, which implements state & command interfaces for joints in a robot model in the Gazebo simulator

### <u>Layer parameters (should be defined under /layered_hardware_gazebo/<robot_name>/<layer_name>)</u>
___joints___ (struct, required)
* joint parameters (see below)

### <u>Joint parameters (should be defined under /layered_hardware_gazebo/<robot_name>/<layer_name>/joints/<joint_name>)</u>
___operation_mode_map___ (map<string, string>, required)
* map from ROS's controller names to joint operation mode names
* possible operation mode names are 'effort', 'position', & 'velocity'

## Example
see [launch/single_joint_example.launch](launch/single_joint_example.launch)