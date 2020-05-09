^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package layered_hardware_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (next version)
--------------------
* Support setting the initial position of joints
* Add fixed_pid operation mode

0.0.7 (2020-05-08)
------------------
* Add fixed operation mode
* Improve position tracking performance of posvel operation modes
* Do not write NaN commands to operation modes

0.0.6 (2020-05-06)
------------------
* Fix to compile on ROS kinetic

0.0.5 (2020-05-06)
------------------
* Add mimic/mimic-PID operation modes
* Supress a warning console message on unloading layer plugins
* Initialize joint states on starting a GazeboJointLayer

0.0.4 (2020-05-05)
------------------
* Support custom control frequency, independent from the simulation step
* Support user-defined gazebo layer plugins which base type is GazeboLayerBase
* Fix a rare issue where GazeboLayeredHardware plugin fails to load

0.0.3 (2020-04-30)
------------------
* Clamp effort commands by joint limits in effort-based operation modes
* Support ROS kinetic (Gazebo7)

0.0.2 (2020-04-30)
------------------
* Add pisition/velocity/posvel -PID operation modes (effort-based position/velocity/posvel mode)
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
