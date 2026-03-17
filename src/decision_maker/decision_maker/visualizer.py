#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker


class DecisionVisualizerNode(Node):
    def __init__(self):
        super().__init__('decision_visualizer')

        self.declare_parameter('frame_id', 'velodyne')
        self.declare_parameter('x', 11.0)
        self.declare_parameter('y', 4.0)
        self.declare_parameter('z', 5.0)
        # legacy name kept for compatibility: used as x-axis spacing
        self.declare_parameter('y_step', -1.0)
        self.declare_parameter('text_scale', 0.8)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_x = float(self.get_parameter('x').value)
        self.base_y = float(self.get_parameter('y').value)
        self.base_z = float(self.get_parameter('z').value)
        self.x_step = float(self.get_parameter('y_step').value)
        self.text_scale = float(self.get_parameter('text_scale').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if self.text_scale <= 0.0:
            self.text_scale = 0.8
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 10.0

        self.mission_state = 'unknown'
        self.auto_throttle = 0.0
        self.auto_steer_angle = 0.0

        self.create_subscription(String, '/mission_state', self._mission_state_cb, 10)
        self.create_subscription(Float32, '/auto_throttle', self._throttle_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle', self._steer_cb, 10)

        self.marker_pub = self.create_publisher(Marker, '/decision/text_marker', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision_visualizer started: frame=%s, anchor=(%.2f, %.2f, %.2f), x_step=%.2f'
            % (self.frame_id, self.base_x, self.base_y, self.base_z, self.x_step)
        )

    def _mission_state_cb(self, msg: String):
        self.mission_state = msg.data

    def _throttle_cb(self, msg: Float32):
        self.auto_throttle = float(msg.data)

    def _steer_cb(self, msg: Float32):
        self.auto_steer_angle = float(msg.data)

    def _make_text_marker(self, marker_id: int, text: str, x_offset: float, color_rgb: tuple) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'decision_text'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.base_x + x_offset
        marker.pose.position.y = self.base_y
        marker.pose.position.z = self.base_z
        marker.pose.orientation.w = 1.0

        marker.scale.z = self.text_scale
        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 1.0
        marker.text = text
        return marker

    def _on_timer(self):
        mission_marker = self._make_text_marker(
            marker_id=0,
            text=f'mission_state: {self.mission_state}',
            x_offset=0.0,
            color_rgb=(1.0, 1.0, 0.0),
        )
        throttle_marker = self._make_text_marker(
            marker_id=1,
            text=f'/auto_throttle: {self.auto_throttle:.3f}',
            x_offset=self.x_step,
            color_rgb=(0.0, 1.0, 0.0),
        )
        steer_marker = self._make_text_marker(
            marker_id=2,
            text=f'/auto_steer_angle: {self.auto_steer_angle:.3f}',
            x_offset=2.0 * self.x_step,
            color_rgb=(0.2, 0.8, 1.0),
        )

        self.marker_pub.publish(mission_marker)
        self.marker_pub.publish(throttle_marker)
        self.marker_pub.publish(steer_marker)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
