# layered_hardware_gazebo
A layered_hardware implementation for the Gazebo simulator, which contains 2 types of plugins;

* GazeboLayeredHardware: a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)
* gazebo_joint_layer: a ros_control layer plugin running in the node above and driving joints in the virtual robot

See [layered_hardware](https://github.com/yoshito-n-students/layered_hardware) to understand the layered scheme.

Several features missing in the [gazebo_ros_control](https://github.com/ros-simulation/gazebo_ros_pkgs) package has been supported.

|                                | This pkg                    | gazebo_ros_control            |
| ---                            | ---                         | ---                           |
| Types of joint hw interface    | Any                         | Pos, Vel, or Eff              |
| Num of hw interfaces per joint | 1+                          | 1                             |
| Joint operation modes          | 10+ (including mimic modes) | 4 (pos, pos_pid, vel, or eff) |
| Initial joint position config  | Yes                         | No                            |

## GazeboLayeredHardware
* a layered_hardware node running on a robot in the Gazebo simulator (as a Gazebo's model plugin)

### <u>Parameters (should be defined under <robot\>/layered_hardware_gazebo)</u>
___control_frequency___ (double, default: 10.0)
* frequency of control step (reading from the layers, updating the controllers, and writing to the layers) in Hz with respect to the simulation time

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
* possible operation mode names are 'effort', 'fixed', 'fixed_pid', 'mimic', 'mimic_pid', 'passive', 'position', 'position_pid', 'posvel', 'posvel_pid', 'velocity', & 'velocity_pid'

___fixed_pid___, ___mimic_pid___, ___position_pid___, ___posvel_pid___, ___velocity_pid___ (struct, required when corresponding modes are used)
* PID parameters which can be loaded by control_toolbox::Pid

___initial_position___ (double, optional)
* initial position of the joint

## Example
see [launch/single_joint_example.launch](launch/single_joint_example.launch)