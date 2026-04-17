flir_ptu
========

ROS 2 driver for FLIR PTUs. Currently tested with:

 - [FLIR D46](http://www.flir.com/mcs/view/?id=53707) using tty
 - [FLIR E46](https://www.flir.com/products/ptu-e46/) using either tty or tcp
 - [FLIR D48E](http://www.flir.com/mcs/view/?id=53670) using either tty or tcp
 - [FLIR PTU-5](https://www.flir.com/products/ptu-5/) using either tty or tcp


Usage
------

This repository contains the description and driver ROS 2 packages for the FLIR D46, E46, D48E, and PTU-5 pan-tilt
units. Refer to FLIR's documentation for physically connecting the unit to your robot.

The driver supports both TTY (serial) and TCP (Ethernet) connections between the robot's PC and the PTU.

Add the `flir_ptu` macro to your URDF, making sure to add a joint between the mounting location and
`${name}_base_link`. The `model` parameter selects the variant (supported values: `ptu5`, `d46`):
```xml
<xacro:include filename="$(find flir_ptu_description)/urdf/flir_ptu.urdf.xacro" />
<xacro:flir_ptu name="ptu" model="ptu5" />
<joint name="ptu_base_joint" type="fixed">
  <parent link="base_link" />
  <child link="ptu_base_link" />
  <origin xyz="0.24 0.0 -0.13" rpy="0 0 0" />
</joint>
```

The model can also be wired up to a top-level xacro arg:
```xml
<xacro:arg name="ptu_model" default="ptu5" />
<xacro:include filename="$(find flir_ptu_description)/urdf/flir_ptu.urdf.xacro" />
<xacro:flir_ptu name="ptu" model="$(arg ptu_model)" />
```

Driver configuration is supplied via a YAML parameter file. See `flir_ptu_driver/config/ptu.yaml` for the full set of
parameters and their defaults. To launch the driver with the default config:
```bash
ros2 launch flir_ptu_driver ptu.launch.py
```

To launch with a custom parameter file:
```bash
ros2 launch flir_ptu_driver ptu.launch.py params_file:=/path/to/my_ptu.yaml
```

An example TCP config:
```yaml
/**:
  ros__parameters:
    connection_type: "tcp"
    ip_addr: "192.168.131.70"
    tcp_port: 4000
    hz: 10
    limits_enabled: false
    default_velocity: 0.0
    joint_name_prefix: "ptu_"
```

An example TTY config:
```yaml
/**:
  ros__parameters:
    connection_type: "tty"
    port: "/dev/ptu"
    baud: 9600
    hz: 10
    limits_enabled: false
    default_velocity: 0.0
    joint_name_prefix: "ptu_"
```


Topics
-------

Publications:
- `state` (remapped to `/joint_states` by the launch file): `sensor_msgs/msg/JointState` — the current pan and tilt
  actuator positions and velocities.
- `/diagnostics`: `diagnostic_msgs/msg/DiagnosticArray` — connection state, PTU mode, pan/tilt limits, live
  position/velocity, communication error count, and a frequency monitor on `joint_states`.

Subscriptions:
- `cmd`: `sensor_msgs/msg/JointState` — command the PTU to move to the desired angle at the requested speed.
- `reset`: `std_msgs/msg/Empty` — publish an empty message to reset (home) the PTU.

When publishing to the `cmd` topic, the `name`, `position`, and `velocity` arrays must each be of length 2, with the
pan actuator first and the tilt actuator second. The `effort` field is ignored.

```bash
ros2 topic pub --once /ptu/cmd sensor_msgs/msg/JointState \
  "{name: ['ptu_pan', 'ptu_tilt'], position: [0.3, -0.2], velocity: [0.6, 0.6]}"
```

A convenience script is also provided for quick commanding from the terminal:
```bash
ros2 run flir_ptu_driver cmd_angles <pan_rad> <tilt_rad> [velocity]
```

For example, to pan to 0.3 rad and tilt to -0.2 rad at 0.6 rad/s:
```bash
ros2 run flir_ptu_driver cmd_angles 0.3 -0.2 0.6
```

Positive pan angles rotate the unit to the left; negative angles rotate it to the right.

Positive tilt angles rotate the unit upward; negative angles rotate it downward.


Visualization
--------------

The `flir_ptu_viz` package provides RViz launch files to visualize the PTU:
```bash
# View the PTU model alone (defaults to the PTU-5)
ros2 launch flir_ptu_viz view_model.launch.py

# Select a different supported model
ros2 launch flir_ptu_viz view_model.launch.py model:=d46

# View the PTU driven by the live driver
ros2 launch flir_ptu_viz view_ptu.launch.py
```


License
--------

This repo originated at [Washington University](https://wu-robotics.googlecode.com/svn/branches/stable/wu_ptu),
where the code was licensed as GPLv2. The initial copy was made at svn revision r2226.

Thanks to Nick Hawes (@hawesie) for the first pass at catkinizing this repo.
