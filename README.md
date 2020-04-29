# layered_hardware_gazebo
A layered_hardware implementation for the Gazebo simulator, which contains 2 types of plugins;

* GazeboLayeredHardware: a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)
* gazebo_joint_layer: a ros_control layer plugin running in the node above and driving joints in the virtual robot

See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware) to understand the layered scheme.

## GazeboLayeredHardware
* a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)

### <u>Parameters (should be defined under <robot\>/layered_hardware_gazebo)</u>
___robot_description___ or ___<node_ns\>/robot_description___ (string, default: "")
* robot description in URDF
* if both given, ___robot_description___ will be used

___layers___ (string array, required)
* names of layers from upper (controller-side) to bottom (actuator-side)

___<layer\>/type___ (string, required)
* lookup name for each layer plugin like 'layered_hardware/JointLimitsLayer' or 'layered_hardware_gazebo/GazeboJointLayer'

## layered_hardware_gazebo/GazeboJointLayer
* a ros_control layer plugin for GazeboLayeredHardware, which implements state & command interfaces for joints in a robot model in the Gazebo simulator

### <u>Layer parameters (should be defined under <robot\>/layered_hardware_gazebo/<layer\>)</u>
___joints___ (struct, required)
* joint parameters (see below)

### <u>Joint parameters (should be defined under <robot\>/layered_hardware_gazebo/<layer\>/joints/<joint\>)</u>
___operation_mode_map___ (map<string, string>, required)
* map from ROS's controller names to joint operation mode names
* possible operation mode names are 'effort', 'passive', 'position', 'position_pid', 'posvel', 'posvel_pid', 'velocity', & 'velocity_pid'

___position_pid___, ___velocity_pid___, ___posvel_pid___ (struct, required when corresponding modes are used)
* PID parameters which can be loaded by control_toolbox::Pid

## Example
see [launch/single_joint_example.launch](launch/single_joint_example.launch)