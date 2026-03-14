#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MaRRT Pure Pursuit Node (ROS2)

- /waypoints 토픽(웨이포인트 배열)을 구독하여 B-spline 보간으로 등간격 제어점을 생성합니다.
- /odometry 토픽을 구독하여 차량의 현재 위치(라이다, 후륜축 중심)와 heading을 업데이트합니다.
- velodyne 좌표계상의 제어점을 차량 좌표계로 변환한 후, pure pursuit 제어를 수행합니다.
- std_msgs/Float32 메시지로 조향각(°)을 publish합니다.
- RViz를 위해 최종 보간 경로(/final_waypoints)와 lookahead point(/rrt/lookahead_point)를 publish합니다.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from vehicle_msgs.msg import WaypointsArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import splprep, splev


class MaRRTPurePursuit(Node):
    def __init__(self):
        super().__init__('ma_rrt_purepursuit')

        self.declare_parameter('wheelbase', 0.724)
        self.declare_parameter('lookahead_distance', 2.5)
        self.declare_parameter('max_steer_deg', 23.0)

        # 파라미터
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.Ld = float(self.get_parameter('lookahead_distance').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        if self.max_steer_deg <= 0.0:
            self.max_steer_deg = 23.0

        # 차량의 현재 위치 (velodyne 좌표계, 라이다/후륜축 중심)
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0  # 라디안

        # B-spline 보간을 통해 생성된 등간격 제어점 (velodyne 좌표계, (x, y) 리스트)
        self.control_points = []

        # 퍼블리셔
        self.cmd_pub = self.create_publisher(Float32, '/auto_steer_angle_rrt', 10)
        self.lookahead_marker_pub = self.create_publisher(Marker, '/rrt/lookahead_point', 10)
        self.final_waypoints_pub = self.create_publisher(Path, '/final_waypoints', 10)

        # 구독자
        self.waypoints_sub = self.create_subscription(WaypointsArray, '/waypoints', self.waypoints_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)

        self.get_logger().info('MaRRT Pure Pursuit 노드가 초기화되었습니다. /waypoints와 /odometry를 구독합니다.')

    def _now(self):
        return self.get_clock().now().to_msg()

    @staticmethod
    def _quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def waypoints_callback(self, msg):
        # /waypoints 메시지에서 (x, y) 좌표 추출 (velodyne 좌표계)
        waypoints = []
        for wp in msg.waypoints:
            waypoints.append((wp.x, wp.y))
        if len(waypoints) < 2:
            self.get_logger().warning('보간할 웨이포인트 개수가 부족합니다.')
            return

        self.get_logger().info(f'보간을 위해 {len(waypoints)}개의 웨이포인트를 수신했습니다.')

        # numpy 배열로 변환
        waypoints_np = np.array(waypoints)
        x = waypoints_np[:, 0]
        y = waypoints_np[:, 1]

        # B-spline 보간
        try:
            tck, _ = splprep([x, y], s=5.0, k=min(3, len(waypoints) - 1))
        except Exception as e:
            self.get_logger().error(f'splprep 실행 오류: {e}')
            return

        # 고해상도 스플라인 곡선 평가
        num_samples = 100
        u_fine = np.linspace(0, 1, num_samples)
        x_fine, y_fine = splev(u_fine, tck)

        # 누적 호 길이(arc length) 계산
        arc_lengths = [0]
        for i in range(1, len(x_fine)):
            dx = x_fine[i] - x_fine[i - 1]
            dy = y_fine[i] - y_fine[i - 1]
            arc_lengths.append(arc_lengths[-1] + math.hypot(dx, dy))
        total_length = arc_lengths[-1]

        # 등간격 제어점 추출
        sampling_distance = 0.03
        desired_lengths = np.arange(0, total_length, sampling_distance)
        if desired_lengths.size == 0 or desired_lengths[-1] < total_length:
            desired_lengths = np.append(desired_lengths, total_length)

        u_equally = np.interp(desired_lengths, arc_lengths, u_fine)
        x_eq, y_eq = splev(u_equally, tck)
        self.control_points = list(zip(x_eq, y_eq))

        self.get_logger().info(f'B-spline 보간으로 {len(self.control_points)}개의 제어점을 생성했습니다.')

        # 생성된 등간격 웨이포인트들을 Path 메시지로 publish (RViz 시각화용)
        final_path = Path()
        final_path.header.frame_id = msg.header.frame_id
        final_path.header.stamp = self._now()
        for pt in self.control_points:
            ps = PoseStamped()
            ps.header = final_path.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            final_path.poses.append(ps)
        self.final_waypoints_pub.publish(final_path)

        self.do_pure_pursuit()

    def odometry_callback(self, msg):
        # /odometry 메시지에서 차량의 현재 위치(라이다, 후륜축 중심) 업데이트
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        q = msg.pose.pose.orientation
        self.vehicle_yaw = self._quaternion_to_yaw(q)

        self.do_pure_pursuit()

    def do_pure_pursuit(self):
        if not self.control_points:
            self.get_logger().warning('제어점이 없으므로 pure pursuit 제어를 수행할 수 없습니다.')
            return

        # velodyne 좌표계의 control_points를 차량 좌표계로 변환
        transformed_points = []
        for pt in self.control_points:
            dx = pt[0] - self.vehicle_x
            dy = pt[1] - self.vehicle_y
            x_rel = math.cos(self.vehicle_yaw) * dx + math.sin(self.vehicle_yaw) * dy
            y_rel = -math.sin(self.vehicle_yaw) * dx + math.cos(self.vehicle_yaw) * dy
            transformed_points.append((x_rel, y_rel))

        # Lookahead distance (self.Ld 사용)
        Ld = self.Ld

        # 차량 앞쪽(x_rel > 0)에서 Ld에 가장 가까운 점 찾기
        best_diff = float('inf')
        lookahead_point = None
        for pt in transformed_points:
            x_rel, y_rel = pt
            if x_rel < 0:
                continue
            dist = math.hypot(x_rel, y_rel)
            diff = abs(dist - Ld)
            if diff < best_diff:
                best_diff = diff
                lookahead_point = (x_rel, y_rel)

        # 유효한 lookahead point가 없으면 정지
        if lookahead_point is None:
            self.get_logger().warning('유효한 lookahead point가 없습니다. 정지합니다.')
            self.publish_stop_cmd()
            return

        # 경로의 마지막 제어점이 1m 미만이면 경로 도착으로 간주하여 정지
        last_pt = transformed_points[-1]
        if math.hypot(last_pt[0], last_pt[1]) < 1.0:
            self.get_logger().info('경로의 끝에 도달했습니다. 정지합니다.')
            self.publish_stop_cmd()
            return

        # 순수 추종 제어 계산
        alpha = math.atan2(lookahead_point[1], lookahead_point[0])
        steer_rad = math.atan2(2.0 * self.wheelbase * math.sin(alpha), Ld)
        steer_deg = - math.degrees(steer_rad)
        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))
        self.get_logger().info(
            f'Pure Pursuit: Lookahead=({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f}), '
            f'alpha={math.degrees(alpha):.2f} deg, steer={steer_deg:.2f} deg ({steer_rad:.3f} rad)'
        )

        # 조향각을 std_msgs/Float32 메시지로 publish (deg 단위)
        angle_msg = Float32()
        angle_msg.data = float(steer_deg)
        self.cmd_pub.publish(angle_msg)

        self.publish_lookahead_marker(lookahead_point)

    def publish_stop_cmd(self):
        stop_msg = Float32()
        stop_msg.data = 0.0
        self.cmd_pub.publish(stop_msg)

    def publish_lookahead_marker(self, lookahead_point):
        x_rel, y_rel = lookahead_point
        x_lookahead = self.vehicle_x + math.cos(self.vehicle_yaw) * x_rel - math.sin(self.vehicle_yaw) * y_rel
        y_lookahead = self.vehicle_y + math.sin(self.vehicle_yaw) * x_rel + math.cos(self.vehicle_yaw) * y_rel

        marker = Marker()
        marker.header.frame_id = 'velodyne'
        marker.header.stamp = self._now()
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x_lookahead)
        marker.pose.position.y = float(y_lookahead)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.lookahead_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
