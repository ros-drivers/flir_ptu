^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_ptu_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2023-03-27)
------------------
* Linting
* Merge pull request `#50 <https://github.com/ros-drivers/flir_ptu/issues/50>`_ from ros-drivers/wip-simulation
  Add Gazebo Support
* Add additional dependencies
* Merge pull request `#51 <https://github.com/ros-drivers/flir_ptu/issues/51>`_ from ros-drivers/no-velocity
  Do not include velocity in the published joint_state messages
* Omit the joint_state.velocity array; it was publishing the configured max velocity, not the actual speed of rotation
* Fix some namespace bugs with the PID controllers, fix some copy-paste errors with the pan/tilt angle limits
* Use a pair of JointPositionControllers to provide gazebo simulation support for the PTU
* Merge pull request `#48 <https://github.com/ros-drivers/flir_ptu/issues/48>`_ from jyang-cpr/noetic-devel
  Expose connection_type, ip_addr, and tcp_port as launch arguments.
* Merge pull request `#47 <https://github.com/ros-drivers/flir_ptu/issues/47>`_ from luis-camero/noetic-devel
  Update disable limits to use read and write
* Merge pull request `#46 <https://github.com/ros-drivers/flir_ptu/issues/46>`_ from luis-camero/noetic-devel
  Noetic TCP Flir PTU
* Changed TAB to spaces
* Added TCP to Flir PTU
* Use Python3 instead of Python
* Make pan/tilt test mode move a little more often
* Move TcpClient into its own class file. Improve ROS param setting.
* interim debugging params and msgs
* Added test mode to send random pan/tilt every few seconds
* fix launch file syntax error
* Adding feature/tcpConnection option for model PTU-D48E
* Contributors: Chris Iverach-Brereton, Joey Yang, Luis Camero, Mark Lewis, Tony Baltovski, luis-camero

0.2.1 (2021-03-26)
------------------
* Don't load the description & start the robot_state_publisher by default
* Contributors: Chris Iverach-Brereton

0.2.0 (2018-06-21)
------------------
* Linter fixes.
* Added udev rule (`#39 <https://github.com/ros-drivers/flir_ptu/issues/39>`_)
* Added disable limits and home commands (`#38 <https://github.com/ros-drivers/flir_ptu/issues/38>`_)
  * Adding parameters for software range limits disable
  * Adding parameters for software range limits disable
  * Added ability to disable software limits
  * Added reset subscribed topic to home PTU. This causes driver crash, so
  added respawn directive to launch file.
  * Minor edits to comply with style guide
* Add default velocity support
* Driver was crashing. Change nulls and emptys to 0s to allow typecasting to double
* added queue_size to cmd_angles (`#23 <https://github.com/ros-drivers/flir_ptu/issues/23>`_)
* Fix linter, add sleep to blocking loop.
* Fix publish topic in cmd_angles script.
* Contributors: Allison Thackston, Ilia Baranov, Mike Purvis, TheDash, Tony Baltovski, Will Baker, dniewinski

0.1.4 (2014-07-17)
------------------
* Fix repository and bug-tracker URLs.
* Add more URL elements
* Update package description
* Fix tabs->spaces in launch file.
* Contributors: Mike Purvis

0.1.3 (2014-04-13)
------------------
* catkin_lint fixes
* Contributors: Mike Purvis

0.1.2 (2014-04-12)
------------------

0.1.1 (2014-04-11)
------------------

0.1.0 (2014-04-10)
------------------
* Parameterize the joint name.
* Add a new simple script for commanding the PTU.
* Remove package for actions; should use control_msgs instead.
* Remove Rate in favour of a Timer callback.
* Add roslint.
* Automatic astyle whitespace/padding fixes.
* Major refactoring.
* Contributors: Mike Purvis
