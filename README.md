flir_ptu
========

Basic serial ROS driver for FLIR PTUs. Currently tested with:

 - [FLIR D46](http://www.flir.com/mcs/view/?id=53707) using tty
 - [FLIR E46](https://www.flir.com/products/ptu-e46/) using either tty or tcp
 - [FLIR D48E](http://www.flir.com/mcs/view/?id=53670) using either tty or tcp


Usage
------

This repository contains the description and driver ROS packages for using the Flir D46 and E46 pan-tilt units.
Refer to Flir's documentation for physically connecting the unit to your robot.

The driver supports both TTY and TCP connections between the robot's PC and the PTU.

Add the `ptu_d46` macro to your URDF, making sure to add a joint between the mounting location and `${name}_base_link`:
```xml
<xacro:include filename="$(find flir_ptu_description)/urdf/d46.urdf.xacro" />
<xacro:ptu_d46 name="ptu" />
<joint name="ptu_base_joint" type="fixed">
  <parent link="base_link" />
  <child link="ptu_base_link" />
  <origin xyz="0.24 0.0 -0.13" rpy="0 0 0" />
</joint>
```

To start the driver in TTY mode, run
```bash
roslaunch flir_ptu_driver connection_type:=tty port:=/dev/ttyS0
```

To start the driver in TCP mode, run
```bash
roslaunch flir_ptu_driver connection_type:=tcp ip_addr:=192.168.131.70 tcp_port:=4000
```


Topics
-------

Publications:
- `state` (normally remapped to `/joint_states`): `sensor_msgs/JointState` -- the current state of the pan and tilt
  actuators.

Subscriptions:
- `cmd`: `sensor_msgs/JointState` -- command the PTU to move to the desired angle with the requested speed
- `reset`: `std_msgs/Bool` -- publish `true` to reset the PTU, publish `false` to stop the reset

When publishing to the `cmd` topic, the `name`, `position`, and `velocity` arrays must be of length 2, with the
pan actuator first and the tilt actuator second.  The `effort` field is ignored.

```bash
rostopic pub /ptu/cmd sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['ptu_pan', 'ptu_tilt']
position: [$(deg2rad 135), $(deg2rad -20)]
velocity: [0]
effort: [0]" -1
```

Positive pan angles will rotate the unit to the left and negative angles will rotate it to the right.

Positive tilt angles will rotate the unit upward and negative angles will rotate it downward.


Simulation
-----------

The `flir_ptu_driver` package also contains a script and launch file designed to allow the PTU to be simulated using
Gazebo.  The simulation is not a perfect 1:1 replication of the behaviour of the physical PTU, but it does allow the
simulated PTU to be controlled using the same ROS topics as the real product.

To start the simulation driver, run
```bash
roslaunch flir_ptu_driver ptu_simulation.launch name:=ptu
```

Make sure the `name` argument matches the `name` attribute of the `<xacro:ptu_d46>` tag in the robot's URDF.

Under the hood, the simulation driver uses two `velocity_controllers/JointPositionController` controllers, one each
to control the pan and tilt actuators.  The PID gains for these controllers can be found in
`flir_ptu_driver/launch/ptu_simulation.launch`.

In simulation mode, the `velocity` field of the `cmd` topic is ignored; the speed of the joints' rotation is
governed wholly by the PID controllers implemented bu the `JointPositionController`s.

In simulation mode, the driver does not publish to `state`; instead the `JointPositionController`s publish directly
to `/joint_states` via the `/gazebo` ROS node.


License
--------

This repo originated at [Washington University](https://wu-robotics.googlecode.com/svn/branches/stable/wu_ptu),
where the code was licensed as GPLv2. The initial copy was made at svn revision r2226.

Thanks to Nick Hawes (@hawesie) for the first pass at catkinizing this repo.
