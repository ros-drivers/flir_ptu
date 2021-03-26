^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_ptu_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2021-03-26)
------------------
* Fix missing xacro namespace prefixes
* Updated PTU mesh geometry (`#40 <https://github.com/ros-drivers/flir_ptu/issues/40>`_)
* Contributors: Dave Niewinski, Robert Haschke

0.2.0 (2018-06-21)
------------------
* Add pan offset for centering joint based on calibration
* Update xacro URL to include www to not give warnings from redefinition
* Fixed spaces, fixed gazebo errors about inertia
* Revert "Fix gazebo errors"
* Updated package.xml for my maintainership
* Fix gazebo errors
* updated CMakeLists to install new meshes folder
* added mesh to visual so openrave can view it, added in transmissions
* Added transmissions and inertial elements for sim
* Reverse pan joint direction.
  Per `#15 <https://github.com/ros-drivers/flir_ptu/issues/15>`_.
* Contributors: Allison Thackston, Dash, Devon Ash, DevonAsh, Mike Purvis, Will Baker

0.1.4 (2014-07-17)
------------------
* Fix weird urdf issue with floating tilt joint.
* Add comments to the example URDF.
* Contributors: Mike Purvis

0.1.3 (2014-04-13)
------------------

0.1.2 (2014-04-12)
------------------
* Remove $(find) macro referencing self, since this doesn't work on the first build.
* Contributors: Mike Purvis

0.1.1 (2014-04-11)
------------------
* Remove urdf as build-time dep.
* Reverse the pan joint; now tested with real hardware.
* Contributors: Mike Purvis

0.1.0 (2014-04-10)
------------------
* Completed basic tf tree and model of D46.
* Contributors: Mike Purvis
