#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class GpsPurePursuitNode(Node):
    def __init__(self):
        super().__init__('gps_purepursuit_node')

        # Pure pursuit and IO parameters
        self.declare_parameter('path_topic', '/gps/f9r_roi_path')
        self.declare_parameter('steer_topic', '/auto_steer_angle_gps')
        self.declare_parameter('lookahead_marker_topic', '/gps/lookahead_point')
        self.declare_parameter('target_frame', 'f9r')
        self.declare_parameter('default_path_frame', 'csv')
        self.declare_parameter('gps_purepursuit_ld', 2.0)
        self.declare_parameter('wheelbase', 0.724)
        self.declare_parameter('max_steer_deg', 23.0)
        self.declare_parameter('lookahead_marker_size', 0.5)
        self.declare_parameter('timer_frequency', 20.0)

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.steer_topic = str(self.get_parameter('steer_topic').value)
        self.lookahead_marker_topic = str(self.get_parameter('lookahead_marker_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.default_path_frame = str(self.get_parameter('default_path_frame').value)
        self.gps_purepursuit_ld = float(self.get_parameter('gps_purepursuit_ld').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        self.lookahead_marker_size = float(self.get_parameter('lookahead_marker_size').value)
        self.timer_frequency = float(self.get_parameter('timer_frequency').value)

        if self.gps_purepursuit_ld <= 0.0:
            self.gps_purepursuit_ld = 2.0
        if self.wheelbase <= 0.0:
            self.wheelbase = 0.724
        if self.max_steer_deg <= 0.0:
            self.max_steer_deg = 23.0
        if self.lookahead_marker_size <= 0.0:
            self.lookahead_marker_size = 0.5
        if self.timer_frequency <= 0.0:
            self.timer_frequency = 20.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_points: List[Point] = []
        self.path_frame: str = self.default_path_frame

        self.path_sub = self.create_subscription(
            Marker, self.path_topic, self._path_callback, 10
        )
        self.steer_pub = self.create_publisher(Float32, self.steer_topic, 10)
        self.lookahead_marker_pub = self.create_publisher(Marker, self.lookahead_marker_topic, 10)
        self.timer = self.create_timer(1.0 / self.timer_frequency, self._on_timer)

        self.get_logger().info(
            '[gps_purepursuit] path_topic=%s steer_topic=%s lookahead_marker_topic=%s '
            'target_frame=%s ld=%.2fm wheelbase=%.2fm max_steer=%.2fdeg freq=%.1fHz'
            % (
                self.path_topic,
                self.steer_topic,
                self.lookahead_marker_topic,
                self.target_frame,
                self.gps_purepursuit_ld,
                self.wheelbase,
                self.max_steer_deg,
                self.timer_frequency,
            )
        )

    def _path_callback(self, msg: Marker) -> None:
        if msg.type != Marker.LINE_STRIP:
            return

        if not msg.points:
            self.path_points = []
            self._clear_lookahead_marker()
            return

        self.path_points = list(msg.points)
        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.default_path_frame

    @staticmethod
    def _distance2(ax: float, ay: float, bx: float, by: float) -> float:
        dx = ax - bx
        dy = ay - by
        return dx * dx + dy * dy

    @staticmethod
    def _transform_point(point: Point, transform) -> Tuple[float, float, float]:
        # Apply rigid transform: p_tgt = R(q) * p_src + t
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Rotation matrix from quaternion
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        px = point.x
        py = point.y
        pz = point.z

        x = r00 * px + r01 * py + r02 * pz + tx
        y = r10 * px + r11 * py + r12 * pz + ty
        z = r20 * px + r21 * py + r22 * pz + tz
        return x, y, z

    def _get_robot_xy_in_path_frame(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.path_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed ({self.target_frame}->{self.path_frame}): {ex}',
                throttle_duration_sec=2.0,
            )
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _find_goal_point(self, robot_x: float, robot_y: float) -> Point:
        # Start at nearest path point to current f9r position in path frame.
        nearest_idx = 0
        best_d2 = float('inf')
        for i, p in enumerate(self.path_points):
            d2 = self._distance2(p.x, p.y, robot_x, robot_y)
            if d2 < best_d2:
                best_d2 = d2
                nearest_idx = i

        accum = 0.0
        goal = self.path_points[nearest_idx]
        for i in range(nearest_idx, len(self.path_points) - 1):
            p0 = self.path_points[i]
            p1 = self.path_points[i + 1]
            accum += math.hypot(p1.x - p0.x, p1.y - p0.y)
            goal = p1
            if accum >= self.gps_purepursuit_ld:
                break
        return goal

    def _compute_steer_deg(self, goal_in_target: Tuple[float, float]) -> float:
        x, y = goal_in_target
        ld2 = max(x * x + y * y, 1e-6)
        steer_rad = math.atan2(2.0 * self.wheelbase * y, ld2)
        steer_deg = math.degrees(steer_rad)
        return max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))

    def _publish_steer(self, steer_deg: float) -> None:
        msg = Float32()
        msg.data = float(steer_deg)
        self.steer_pub.publish(msg)

    def _publish_lookahead_marker(self, point_in_path_frame: Point) -> None:
        marker = Marker()
        marker.header.frame_id = self.path_frame if self.path_frame else self.default_path_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gps_lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point_in_path_frame.x)
        marker.pose.position.y = float(point_in_path_frame.y)
        marker.pose.position.z = float(point_in_path_frame.z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.lookahead_marker_size
        marker.scale.y = self.lookahead_marker_size
        marker.scale.z = self.lookahead_marker_size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.0
        self.lookahead_marker_pub.publish(marker)

    def _clear_lookahead_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.path_frame if self.path_frame else self.default_path_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gps_lookahead'
        marker.id = 0
        marker.action = Marker.DELETE
        self.lookahead_marker_pub.publish(marker)

    def _on_timer(self) -> None:
        if len(self.path_points) < 2:
            self._publish_steer(0.0)
            self._clear_lookahead_marker()
            return

        robot_xy = self._get_robot_xy_in_path_frame()
        if robot_xy is None:
            self._publish_steer(0.0)
            self._clear_lookahead_marker()
            return

        goal_path = self._find_goal_point(robot_xy[0], robot_xy[1])
        self._publish_lookahead_marker(goal_path)

        try:
            tf_path_to_target = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.path_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF lookup failed ({self.path_frame}->{self.target_frame}): {ex}',
                throttle_duration_sec=2.0,
            )
            self._publish_steer(0.0)
            self._clear_lookahead_marker()
            return

        gx, gy, _ = self._transform_point(goal_path, tf_path_to_target)
        steer_deg = self._compute_steer_deg((gx, gy))
        self._publish_steer(steer_deg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
