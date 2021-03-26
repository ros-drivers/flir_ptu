^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_ptu_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
