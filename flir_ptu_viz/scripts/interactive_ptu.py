#!/usr/bin/env python3
"""
Interactive-marker teleop for the FLIR PTU.

Publishes ``sensor_msgs/JointState`` on ``joint_states`` so that
``robot_state_publisher`` reflects the commanded pose in RViz. When
``publish_cmd`` is set, the same values are also published on ``cmd``
as expected by ``flir_ptu_driver`` for live hardware control.
"""

import math

from geometry_msgs.msg import Quaternion
from interactive_markers import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


def yaw_from_quat(q: Quaternion) -> float:
    # Z-axis rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pitch_from_quat(q: Quaternion) -> float:
    # Y-axis rotation
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class InteractivePtu(Node):
    def __init__(self) -> None:
        super().__init__('interactive_ptu')

        self.declare_parameter('joint_name_prefix', 'ptu_')
        self.declare_parameter('base_frame', 'ptu_base_link')
        self.declare_parameter('pan_frame', 'ptu_pan_link')
        self.declare_parameter('pan_min', -3.106)
        self.declare_parameter('pan_max', 3.106)
        self.declare_parameter('tilt_min', -1.414)
        self.declare_parameter('tilt_max', 0.541)
        self.declare_parameter('publish_cmd', False)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('cmd_velocity', 1.0)

        prefix = self.get_parameter('joint_name_prefix').value
        self.pan_joint = f'{prefix}pan'
        self.tilt_joint = f'{prefix}tilt'
        self.base_frame = self.get_parameter('base_frame').value
        self.pan_frame = self.get_parameter('pan_frame').value
        self.pan_min = self.get_parameter('pan_min').value
        self.pan_max = self.get_parameter('pan_max').value
        self.tilt_min = self.get_parameter('tilt_min').value
        self.tilt_max = self.get_parameter('tilt_max').value
        self.publish_cmd = self.get_parameter('publish_cmd').value
        publish_js = self.get_parameter('publish_joint_states').value
        self.cmd_velocity = float(self.get_parameter('cmd_velocity').value)

        self.pan = 0.0
        self.tilt = 0.0
        self._last_cmd = None

        self.js_pub = (
            self.create_publisher(JointState, 'joint_states', 10) if publish_js else None
        )
        self.cmd_pub = (
            self.create_publisher(JointState, 'cmd', 1) if self.publish_cmd else None
        )

        self.server = InteractiveMarkerServer(self, 'ptu_interactive')
        self._make_pan_marker()
        self._make_tilt_marker()
        self.server.applyChanges()

        # Publish the initial joint state and keep a steady stream going so that
        # robot_state_publisher always has fresh TF even when idle.
        self.timer = self.create_timer(0.1, self._publish_state)

    def _rotate_control(self, axis: str) -> InteractiveMarkerControl:
        ctrl = InteractiveMarkerControl()
        ctrl.always_visible = True
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # Orientation defines the rotation axis (its x component).
        ctrl.orientation.w = 1.0
        if axis == 'z':
            ctrl.orientation.x = 0.0
            ctrl.orientation.y = 1.0  # swap y<->x so rotation axis is +Z
            ctrl.orientation.z = 0.0
        elif axis == 'y':
            ctrl.orientation.x = 0.0
            ctrl.orientation.y = 0.0
            ctrl.orientation.z = 1.0  # rotation axis is +Y
        else:  # 'x'
            ctrl.orientation.x = 1.0
            ctrl.orientation.y = 0.0
            ctrl.orientation.z = 0.0
        return ctrl

    def _ring_marker(self, radius: float, color) -> Marker:
        m = Marker()
        m.type = Marker.CYLINDER
        m.scale.x = radius * 2.0
        m.scale.y = radius * 2.0
        m.scale.z = 0.005
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def _make_pan_marker(self) -> None:
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = 'pan'
        im.description = 'Pan'
        im.scale = 0.25

        ctrl = self._rotate_control('z')
        ring = self._ring_marker(0.1, (0.1, 0.4, 0.9, 0.5))
        ctrl.markers.append(ring)
        im.controls.append(ctrl)

        self.server.insert(im, feedback_callback=self._on_pan_feedback)

    def _make_tilt_marker(self) -> None:
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = 'tilt'
        im.description = 'Tilt'
        im.pose.position.y = -0.18
        im.pose.position.z = 0.0962
        im.scale = 0.22

        ctrl = self._rotate_control('y')
        ring = self._ring_marker(0.09, (0.9, 0.5, 0.1, 0.5))
        # Orient the ring disk so it lies in the X-Z plane (axis along Y).
        ring.pose.orientation.x = math.sin(math.pi / 4.0)
        ring.pose.orientation.w = math.cos(math.pi / 4.0)
        ctrl.markers.append(ring)
        im.controls.append(ctrl)

        self.server.insert(im, feedback_callback=self._on_tilt_feedback)

    def _on_pan_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pan = clamp(yaw_from_quat(feedback.pose.orientation),
                             self.pan_min, self.pan_max)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.pan = clamp(yaw_from_quat(feedback.pose.orientation),
                             self.pan_min, self.pan_max)
            self._send_cmd()

    def _on_tilt_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.tilt = clamp(pitch_from_quat(feedback.pose.orientation),
                              self.tilt_min, self.tilt_max)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.tilt = clamp(pitch_from_quat(feedback.pose.orientation),
                              self.tilt_min, self.tilt_max)
            self._send_cmd()

    def _send_cmd(self) -> None:
        if self.cmd_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.pan_joint, self.tilt_joint]
        msg.position = [self.pan, self.tilt]
        msg.velocity = [self.cmd_velocity, self.cmd_velocity]
        self.cmd_pub.publish(msg)

    def _publish_state(self) -> None:
        if self.js_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.pan_joint, self.tilt_joint]
        msg.position = [self.pan, self.tilt]
        msg.velocity = [0.0, 0.0]
        self.js_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = InteractivePtu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
