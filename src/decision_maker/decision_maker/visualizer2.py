#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, String
from visualization_msgs.msg import Marker, MarkerArray


class DecisionVisualizerNode(Node):
    def __init__(self):
        super().__init__('decision_visualizer2')

        self.declare_parameter('frame_id', 'velodyne')
        self.declare_parameter('x', 11.0)
        self.declare_parameter('y', 6.0)
        self.declare_parameter('z', 5.0)
        self.declare_parameter('y_step', -1.0)
        self.declare_parameter('text_scale', 0.8)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_x = float(self.get_parameter('x').value)
        self.base_y = float(self.get_parameter('y').value)
        self.base_z = float(self.get_parameter('z').value)
        self.y_step = float(self.get_parameter('y_step').value)
        self.text_scale = float(self.get_parameter('text_scale').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if self.text_scale <= 0.0:
            self.text_scale = 0.8
        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 30.0

        self.mission_state = 'unknown'
        self.auto_throttle = 0.0
        self.auto_steer_angle = 0.0
        self.lane_detected = False
        self.num_lidar_cone = 0
        self.is_emergency = False

        self.create_subscription(String, '/mission_state', self._mission_state_cb, 10)
        self.create_subscription(Float32, '/auto_throttle', self._throttle_cb, 10)
        self.create_subscription(Float32, '/auto_steer_angle', self._steer_cb, 10)
        self.create_subscription(Bool, '/lane_detect', self._lane_detect_cb, 10)
        self.create_subscription(Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10)
        self.create_subscription(Bool, '/emergency', self._emergency_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/decision/text_marker', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            'decision_visualizer2 started: frame=%s, anchor=(%.2f, %.2f, %.2f), y_step=%.2f'
            % (self.frame_id, self.base_x, self.base_y, self.base_z, self.y_step)
        )

    def _mission_state_cb(self, msg: String):
        self.mission_state = msg.data

    def _throttle_cb(self, msg: Float32):
        self.auto_throttle = float(msg.data)

    def _steer_cb(self, msg: Float32):
        self.auto_steer_angle = float(msg.data)

    def _lane_detect_cb(self, msg: Bool):
        self.lane_detected = bool(msg.data)

    def _num_lidar_cone_cb(self, msg: Int8):
        self.num_lidar_cone = int(msg.data)

    def _emergency_cb(self, msg: Bool):
        self.is_emergency = bool(msg.data)

    def _make_text_marker(
        self,
        marker_id: int,
        text: str,
        x_offset: float,
        color_rgb: tuple,
        y_offset: float = 0.0,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'decision_text'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.base_x + x_offset
        marker.pose.position.y = self.base_y + y_offset
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
        mission_colors = {
            'emergency': (1.0, 0.0, 0.0),
            'moon_course': (1.0, 0.5, 0.0),
            'static_obstacle': (1.0, 0.5, 0.0),
            'lane': (0.0, 1.0, 0.0),
            'rrt': (0.0, 0.0, 1.0),
            'gps': (1.0, 1.0, 1.0),
        }
        mission_color = mission_colors.get(self.mission_state, (1.0, 1.0, 1.0))
        mission_marker = self._make_text_marker(
            marker_id=0,
            text=f'mission_state: {self.mission_state}',
            x_offset=0.0,
            color_rgb=mission_color,
        )
        throttle_marker = self._make_text_marker(
            marker_id=1,
            text=f'/auto_throttle: {self.auto_throttle:.3f}',
            x_offset=self.y_step,
            color_rgb=(1.0, 1.0, 1.0),
        )
        steer_marker = self._make_text_marker(
            marker_id=2,
            text=f'/auto_steer_angle: {self.auto_steer_angle:.3f}',
            x_offset=2.0 * self.y_step,
            color_rgb=(1.0, 1.0, 1.0),
        )
        lane_color = (0.0, 1.0, 0.0) if self.lane_detected else (1.0, 1.0, 1.0)
        lane_detect_marker = self._make_text_marker(
            marker_id=3,
            text=f'/lane_detect: {"true" if self.lane_detected else "false"}',
            x_offset=3.0 * self.y_step,
            color_rgb=lane_color,
        )
        cone_marker = self._make_text_marker(
            marker_id=4,
            text=f'/vn/num_lidar_cone: {self.num_lidar_cone}',
            x_offset=4.0 * self.y_step,
            color_rgb=(1.0, 1.0, 1.0),
        )
        emergency_color = (0.0, 1.0, 0.0) if self.is_emergency else (1.0, 1.0, 1.0)
        emergency_marker = self._make_text_marker(
            marker_id=5,
            text=f'/emergency: {"true" if self.is_emergency else "false"}',
            x_offset=5.0 * self.y_step,
            color_rgb=emergency_color,
        )
        marker_array = MarkerArray()
        marker_array.markers = [
            mission_marker,
            throttle_marker,
            steer_marker,
            lane_detect_marker,
            cone_marker,
            emergency_marker,
        ]
        self.marker_pub.publish(marker_array)


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
