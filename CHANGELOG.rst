^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package layered_hardware_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (next version)
--------------------
* Support custom control frequency, independent from the simulation step
* Support user-defined gazebo layer plugins which base type is GazeboLayerBase

0.0.3 (2020-04-30)
------------------
* Clamp effort commands by joint limits in effort-based operation modes
* Support ROS kinetic (Gazebo7)

0.0.2 (2020-04-30)
------------------
* Add pisition/velocity/posvel PID operation mode (effort-based position/velocity/posvel mode)
* Add posvel operation mode (position mode with profile velocity)
* Add passive operation mode
* Fix velocity operation mode
* Change parameter namespace to <robot_name>/layered_hardware_gazebo

0.0.1 (2020-04-28)
------------------
* Support multiple types of hardware interfaces per joint
* Support any types of joint hardware interfaces (using transmission_interface::RequisiteProvider plugins)
* Online switching of embedded joint operation modes
* Support ROS melodic (Gazebo9)
* Contributors: Yoshito Okada
