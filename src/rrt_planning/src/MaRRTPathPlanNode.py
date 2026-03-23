#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RRT Path Planning with multiple remote goals.

author: Maxim Yastremsky(@MaxMagazin)
based on the work of AtsushiSakai(@Atsushi_twi)
"""

import os

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] {message}'

import rclpy
import ma_rrt
import numpy as np
import time, math

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from vehicle_msgs.msg import TrackCone, Waypoint, WaypointsArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from scipy.spatial import Delaunay

class MaRRTPathPlanNode(Node):

    def __init__(self):
        super().__init__('ma_rrt_path_plan_node')

        self.declare_parameter('publishWaypoints', True)
        self.declare_parameter('publishPredefined', False)
        self.declare_parameter('path', '')
        self.declare_parameter('filename', '')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('world_frame', 'velodyne')
        self.declare_parameter('desiredWaypointsFrequency', 5.0)
        self.declare_parameter('sample_frequency', 20.0)
        self.declare_parameter('obstacle_topic', '/vn/lidar_cone')
        self.declare_parameter('roi_end_topic', '/gps/f9r_roi_end_velodyne')
        self.declare_parameter('rrt_target_visual_topic', '/rrt/rrt_target')
        self.declare_parameter('use_roi_end_as_rrt_target', True)
        self.declare_parameter('allow_cone_fallback_target', True)
        
        self.declare_parameter('rrt_front_cones_dist', 12.0)
        self.declare_parameter('rrt_front_cones_extended_dist', 15.0)
        self.declare_parameter('rrt_front_behind_dist', 1.0)
        
        self.declare_parameter('rrt_cone_obstacle_radius', 0.4)
        self.declare_parameter('rrt_target_radius', 0.3)
        self.declare_parameter('rrt_cone_fallback_min_dist', 6.0)
        
        self.declare_parameter('rrt_iteration_count', 100)
        self.declare_parameter('rrt_plan_distance', 5.4)
        self.declare_parameter('rrt_expand_distance', 0.6)
        self.declare_parameter('rrt_expand_angle_deg', 22.0)
        
        self.declare_parameter('rrt_waypoint_max_accepted_edge_length', 7.0)
        self.declare_parameter('rrt_waypoint_max_edge_parts_ratio', 3.0)
        self.declare_parameter('rrt_merge_max_dist_to_save_waypoint', 2.0)
        self.declare_parameter('rrt_merge_max_waypoint_save_count', 4)
        self.declare_parameter('rrt_merge_waypoint_dist_tolerance', 1000.0)
        self.declare_parameter('rrt_saved_waypoint_preloop_threshold', 15)
        self.declare_parameter('rrt_filter_dist_change_limit', 2.0)
        self.declare_parameter('rrt_filter_new_point_alpha', 0.2)
        self.declare_parameter('rrt_filter_max_discard_reset', 2)
        
        self.declare_parameter('rrt_branch_cone_dist_limit', 4.0)
        self.declare_parameter('rrt_branch_both_sides_improve_factor', 3.0)
        self.declare_parameter('rrt_branch_min_acceptable_rating', 90.0)

        self.shouldPublishWaypoints = bool(self.get_parameter('publishWaypoints').value)
        self.shouldPublishPredefined = bool(self.get_parameter('publishPredefined').value)
        self.path = str(self.get_parameter('path').value)
        self.filename = str(self.get_parameter('filename').value)
        self.odometry_topic = str(self.get_parameter('odom_topic').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.roi_end_topic = str(self.get_parameter('roi_end_topic').value)
        self.rrt_target_visual_topic = str(self.get_parameter('rrt_target_visual_topic').value)
        self.use_roi_end_as_rrt_target = bool(self.get_parameter('use_roi_end_as_rrt_target').value)
        self.allow_cone_fallback_target = bool(self.get_parameter('allow_cone_fallback_target').value)
        self.rrt_front_cones_dist = float(self.get_parameter('rrt_front_cones_dist').value)
        self.rrt_front_cones_extended_dist = float(self.get_parameter('rrt_front_cones_extended_dist').value)
        self.rrt_front_behind_dist = float(self.get_parameter('rrt_front_behind_dist').value)
        self.rrt_cone_obstacle_radius = float(self.get_parameter('rrt_cone_obstacle_radius').value)
        self.rrt_target_radius = float(self.get_parameter('rrt_target_radius').value)
        self.rrt_cone_fallback_min_dist = float(self.get_parameter('rrt_cone_fallback_min_dist').value)
        self.rrt_iteration_count = int(self.get_parameter('rrt_iteration_count').value)
        self.rrt_plan_distance = float(self.get_parameter('rrt_plan_distance').value)
        self.rrt_expand_distance = float(self.get_parameter('rrt_expand_distance').value)
        self.rrt_expand_angle_deg = float(self.get_parameter('rrt_expand_angle_deg').value)
        self.rrt_waypoint_max_accepted_edge_length = float(self.get_parameter('rrt_waypoint_max_accepted_edge_length').value)
        self.rrt_waypoint_max_edge_parts_ratio = float(self.get_parameter('rrt_waypoint_max_edge_parts_ratio').value)
        self.rrt_merge_max_dist_to_save_waypoint = float(self.get_parameter('rrt_merge_max_dist_to_save_waypoint').value)
        self.rrt_merge_max_waypoint_save_count = int(self.get_parameter('rrt_merge_max_waypoint_save_count').value)
        self.rrt_merge_waypoint_dist_tolerance = float(self.get_parameter('rrt_merge_waypoint_dist_tolerance').value)
        self.rrt_saved_waypoint_preloop_threshold = int(self.get_parameter('rrt_saved_waypoint_preloop_threshold').value)
        self.rrt_filter_dist_change_limit = float(self.get_parameter('rrt_filter_dist_change_limit').value)
        self.rrt_filter_new_point_alpha = float(self.get_parameter('rrt_filter_new_point_alpha').value)
        self.rrt_filter_max_discard_reset = int(self.get_parameter('rrt_filter_max_discard_reset').value)
        self.rrt_branch_cone_dist_limit = float(self.get_parameter('rrt_branch_cone_dist_limit').value)
        self.rrt_branch_both_sides_improve_factor = float(self.get_parameter('rrt_branch_both_sides_improve_factor').value)
        self.rrt_branch_min_acceptable_rating = float(self.get_parameter('rrt_branch_min_acceptable_rating').value)

        waypointsFrequency = float(self.get_parameter('desiredWaypointsFrequency').value)
        if waypointsFrequency <= 0.0:
            waypointsFrequency = 5.0
        self.waypointsPublishInterval = 1.0 / waypointsFrequency
        self.lastPublishWaypointsTime = 0

        sample_frequency = float(self.get_parameter('sample_frequency').value)
        if sample_frequency <= 0.0:
            sample_frequency = 20.0
        self.rrt_front_cones_dist = max(0.1, self.rrt_front_cones_dist)
        self.rrt_front_cones_extended_dist = max(self.rrt_front_cones_dist, self.rrt_front_cones_extended_dist)
        self.rrt_front_behind_dist = max(0.0, self.rrt_front_behind_dist)
        self.rrt_cone_obstacle_radius = max(0.01, self.rrt_cone_obstacle_radius)
        self.rrt_target_radius = max(0.01, self.rrt_target_radius)
        self.rrt_cone_fallback_min_dist = max(0.0, self.rrt_cone_fallback_min_dist)
        self.rrt_iteration_count = max(1, self.rrt_iteration_count)
        self.rrt_expand_distance = max(0.01, self.rrt_expand_distance)
        self.rrt_plan_distance = max(self.rrt_expand_distance + 1e-3, self.rrt_plan_distance)
        self.rrt_expand_angle_deg = max(0.1, self.rrt_expand_angle_deg)
        self.rrt_waypoint_max_accepted_edge_length = max(0.01, self.rrt_waypoint_max_accepted_edge_length)
        self.rrt_waypoint_max_edge_parts_ratio = max(1.0, self.rrt_waypoint_max_edge_parts_ratio)
        self.rrt_merge_max_dist_to_save_waypoint = max(0.01, self.rrt_merge_max_dist_to_save_waypoint)
        self.rrt_merge_max_waypoint_save_count = max(1, self.rrt_merge_max_waypoint_save_count)
        self.rrt_merge_waypoint_dist_tolerance = max(0.01, self.rrt_merge_waypoint_dist_tolerance)
        self.rrt_saved_waypoint_preloop_threshold = max(1, self.rrt_saved_waypoint_preloop_threshold)
        self.rrt_filter_dist_change_limit = max(0.01, self.rrt_filter_dist_change_limit)
        self.rrt_filter_new_point_alpha = max(0.0, min(1.0, self.rrt_filter_new_point_alpha))
        self.rrt_filter_max_discard_reset = max(1, self.rrt_filter_max_discard_reset)
        self.rrt_branch_cone_dist_limit = max(0.01, self.rrt_branch_cone_dist_limit)
        self.rrt_branch_both_sides_improve_factor = max(1.0, self.rrt_branch_both_sides_improve_factor)
        self.rrt_branch_min_acceptable_rating = max(0.0, self.rrt_branch_min_acceptable_rating)

        """
        구독자들
        """
        # VoxelNeXt lidar_cone_detect.py가 발행하는 장애물 중심점 구독
        self.detected_center_sub = self.create_subscription(
            MarkerArray,
            self.obstacle_topic,
            self.detectedCenterCallback,
            10
        )

        # /odometry 토픽 구독하여 차량 yaw를 업데이트
        self.odometry_sub = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometryCallback,
            10
        )

        # /f9r_roi_end_velodyne (PointStamped, velodyne frame) 구독 -> RRT 목표점으로 사용
        self.roi_end_sub = self.create_subscription(
            PointStamped,
            self.roi_end_topic,
            self.roiEndCallback,
            10
        )
        self.rrt_target = None  # 현재 사용 중인 목표 좌표 저장

        """
        퍼블리셔들
        """
        self.waypointsPub = self.create_publisher(WaypointsArray, '/waypoints', 10)
        self.newwaypointsPub = self.create_publisher(WaypointsArray, '/newwaypoints', 10)

        self.treeVisualPub = self.create_publisher(MarkerArray, '/rrt/tree_branch', 10)
        self.bestBranchVisualPub = self.create_publisher(MarkerArray, '/rrt/best_tree_branch', 10)
        self.filteredBranchVisualPub = self.create_publisher(MarkerArray, '/rrt/filtered_tree_branch', 10)
        self.delaunayLinesVisualPub = self.create_publisher(MarkerArray, '/rrt/delaunay_triangles', 10)
        self.waypointsVisualPub = self.create_publisher(MarkerArray, '/rrt/waypoints', 10)
        self.obstacleVisualPub = self.create_publisher(MarkerArray, '/rrt/obstacle_radius', 10)
        self.rrtTargetVisualPub = self.create_publisher(MarkerArray, self.rrt_target_visual_topic, 10)


        
        # 차량의 현재 위치 및 자세 초기값 (in velodyne frame)
        self.carPosX = 0.0
        self.carPosY = 0.0
        self.carPosYaw = 0.0

        self.map = []
        self.savedWaypoints = []
        self.preliminaryLoopClosure = False
        self.loopClosure = False
        self.rrt = None
        self.filteredBestBranch = []
        self.discardAmount = 0
        self.latest_roi_end_point = None
        self.last_roi_warn_time = 0.0

        self.sample_timer = self.create_timer(1.0 / sample_frequency, self.sampleTree)

        self.get_logger().info(
            f'Subscribed obstacle centers from {self.obstacle_topic}, ROI goal from {self.roi_end_topic}, '
            f'ROI-as-target={self.use_roi_end_as_rrt_target}, cone-fallback={self.allow_cone_fallback_target}, '
            f'running planner at {sample_frequency:.1f} Hz, '
            f'iter={self.rrt_iteration_count}, plan={self.rrt_plan_distance:.2f}, '
            f'expand={self.rrt_expand_distance:.2f}, turn={self.rrt_expand_angle_deg:.1f}'
        )

    def _now(self):
        return self.get_clock().now().to_msg()

    def _duration(self, seconds):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    @staticmethod
    def _point(x, y, z=0.0):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    @staticmethod
    def _single_marker_array(marker):
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        return marker_array

    @staticmethod
    def _quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publishObstacleVisuals(self, obstacleList):
        markerArray = MarkerArray()
        for i, (x, y, radius) in enumerate(obstacleList):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self._now()
            marker.ns = "obstacle_radius"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # SPHERE의 scale은 지름이므로, 반지름 * 2 설정
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = 0.1  # 평면 상의 표시이므로 z는 작게

            marker.color.a = 0.2  # 투명도
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0

            # 필요한 경우, marker.lifetime 설정 (예: 0.2초)
            marker.lifetime = self._duration(0.2)

            markerArray.markers.append(marker)
        self.obstacleVisualPub.publish(markerArray)

    def publishRrtTargetVisual(self, target_point):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self._now()
        marker.ns = "rrt_target"
        marker.id = 0

        if target_point is None:
            marker.action = Marker.DELETE
            self.rrtTargetVisualPub.publish(self._single_marker_array(marker))
            return

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_point.x)
        marker.pose.position.y = float(target_point.y)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = self._duration(0.3)

        self.rrtTargetVisualPub.publish(self._single_marker_array(marker))

    def roiEndCallback(self, marker):
        self.latest_roi_end_point = marker

    def _warn_roi_throttle(self, message, period_sec=2.0):
        now = time.time()
        if (now - self.last_roi_warn_time) >= period_sec:
            self.get_logger().warn(message)
            self.last_roi_warn_time = now

    def resolveRoiEndTarget(self):
        point_msg = self.latest_roi_end_point
        if point_msg is None:
            return None

        if not point_msg.header.frame_id:
            self._warn_roi_throttle(f'{self.roi_end_topic} has empty frame_id.')
            return None

        if point_msg.header.frame_id != self.world_frame:
            self._warn_roi_throttle(
                f'{self.roi_end_topic} frame mismatch: got {point_msg.header.frame_id}, expected {self.world_frame}.'
            )
            return None

        return self._point(point_msg.point.x, point_msg.point.y, 0.0)

    def odometryCallback(self, odometry):
        # # /odometry 토픽으로부터 받은 Odometry 메시지를 이용하여 차량 위치 업데이트
        self.carPosX = 0.0
        self.carPosY = 0.0
        q = odometry.pose.pose.orientation
        self.carPosYaw = self._quaternion_to_yaw(q)

    def yawCallback(self, yaw):
        self.carPosYaw = yaw.orientation.z

    def detectedCenterCallback(self, marker_array):
        cones = []
        for marker in marker_array.markers:
            cone = TrackCone()
            cone.x = float(marker.pose.position.x)
            cone.y = float(marker.pose.position.y)
            cone.type = 'traffic_cone'
            cones.append(cone)
        self.map = cones

    def sampleTree(self):
        
        self.coneObstacleList = []
        self.coneObstacleList.clear()
                
        
        if self.loopClosure and len(self.savedWaypoints) > 0:
            self.rrt_target = None
            self.publishRrtTargetVisual(None)
            self.publishWaypoints()
            return

        if not self.map:
            self.rrt_target = None
            self.publishRrtTargetVisual(None)
            return

        frontCones = self.getFrontConeObstacles(self.map, self.rrt_front_cones_dist)

        coneObstacleSize = self.rrt_cone_obstacle_radius
        # 트래픽 콘들로 이루어진 장애물 리스트 생성
        self.coneObstacleList = [(cone.x, cone.y, coneObstacleSize) for cone in frontCones]

        obstacleList = self.coneObstacleList
        
        self.get_logger().info(
            f'obstacles: cones={len(self.coneObstacleList)}'
        )
        
        # 병합된 장애물들에 대한 시각화 메시지 퍼블리시
        self.publishObstacleVisuals(obstacleList)


        
        # 이후 기존 코드대로 rrtTarget 설정 및 RRT 실행
        rrtTarget = []
        targetRadius = self.rrt_target_radius
                
        roi_target = self.resolveRoiEndTarget() if self.use_roi_end_as_rrt_target else None
        if roi_target is not None:
            self.rrt_target = roi_target
            rrtTarget.append((roi_target.x, roi_target.y, targetRadius))
            self.get_logger().info(
                f'[RRT target] f9r_roi_end frame={self.world_frame} '
                f'pos=({roi_target.x:.2f}, {roi_target.y:.2f})'
            )

        # ROI 목표를 사용할 수 없고 fallback 허용 시 멀리 있는 콘들을 목표점으로 사용
        elif self.allow_cone_fallback_target:
            self.rrt_target = None
            fallback_found = False
            for cone in frontCones:
                coneDist = self.dist(self.carPosX, self.carPosY, cone.x, cone.y)
                if coneDist > self.rrt_cone_fallback_min_dist:
                    self.rrt_target = self._point(cone.x, cone.y, 0.0)
                    rrtTarget.append((cone.x, cone.y, coneObstacleSize))
                    self.get_logger().info(
                        f'[RRT target] cone_fallback frame={self.world_frame} '
                        f'pos=({cone.x:.2f}, {cone.y:.2f}), dist={coneDist:.2f}m'
                    )
                    fallback_found = True
                    break  # 조건에 맞는 콘을 하나 찾으면 반복문 종료

            if not fallback_found:
                self.get_logger().warn(
                    f'[RRT target] source=NONE no ROI target and no fallback cone candidate '
                    f'(dist > {self.rrt_cone_fallback_min_dist:.2f}m).'
                )
                self.publishRrtTargetVisual(None)
                return

        else:
            self.rrt_target = None
            self.publishRrtTargetVisual(None)
            if self.use_roi_end_as_rrt_target:
                self._warn_roi_throttle(
                    f'[RRT target] ROI target unavailable on {self.roi_end_topic}; skipping planning '
                    f'(allow_cone_fallback_target={self.allow_cone_fallback_target}).'
                )
            else:
                self.get_logger().warn(
                    '[RRT target] No target source enabled (ROI disabled and cone fallback disabled).'
                )
            return

        self.publishRrtTargetVisual(self.rrt_target)

            
        start = [self.carPosX, self.carPosY, self.carPosYaw]

        rrt = ma_rrt.RRT(
            start,
            self.rrt_plan_distance,
            obstacleList=obstacleList,
            expandDis=self.rrt_expand_distance,
            turnAngle=self.rrt_expand_angle_deg,
            maxIter=self.rrt_iteration_count,
            rrtTargets=rrtTarget,
        )
        nodeList, leafNodes = rrt.Planning()

        self.publishTreeVisual(nodeList, leafNodes)

        # 기본 전방 범위보다 약간 넓은 범위에서 콘들을 모아 경로 평가나 보완에 활용
        largerGroupFrontCones = self.getFrontConeObstacles(self.map, self.rrt_front_cones_extended_dist)

        bestBranch = self.findBestBranch(leafNodes, nodeList, largerGroupFrontCones)

        if bestBranch:
            filteredBestBranch = self.getFilteredBestBranch(bestBranch)

            if filteredBestBranch:
                delaunayEdges = self.getDelaunayEdges(frontCones)
                self.publishDelaunayEdgesVisual(delaunayEdges)
                newWaypoints = []

                if delaunayEdges:
                    newWaypoints = self.getWaypointsFromEdges(filteredBestBranch, delaunayEdges)

                if newWaypoints:
                    self.mergeWaypoints(newWaypoints)

                self.publishWaypoints(newWaypoints)

    def mergeWaypoints(self, newWaypoints):
        if not newWaypoints:
            return

        # 차량의 현재 위치와 후보 웨이포인트 간의 거리가 2.0미터 이하일 때만 해당 웨이포인트를 저장 대상으로 고려
        maxDistToSaveWaypoints = self.rrt_merge_max_dist_to_save_waypoint
        
        # 새로운 웨이포인트 리스트에서 최대 2개까지만 저장 
        maxWaypointAmountToSave = self.rrt_merge_max_waypoint_save_count
        
        # 기존에 저장된 웨이포인트와 새 후보 웨이포인트 간의 거리를 비교 -> 만약 두 점 간의 거리가 이 값보다 작으면, 두 점이 “거의 동일하다”고 판단하여 중복을 제거
        waypointsDistTollerance = self.rrt_merge_waypoint_dist_tolerance

        if len(self.savedWaypoints) > self.rrt_saved_waypoint_preloop_threshold:
            firstSavedWaypoint = self.savedWaypoints[0]

            for waypoint in reversed(newWaypoints):
                distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypoint[0], waypoint[1])
                if distDiff < waypointsDistTollerance:
                    self.preliminaryLoopClosure = False
                    break

        newSavedPoints = []

        for i in range(len(newWaypoints)):
            waypointCandidate = newWaypoints[i]

            carWaypointDist = self.dist(self.carPosX, self.carPosY, waypointCandidate[0], waypointCandidate[1])

            if i >= maxWaypointAmountToSave or carWaypointDist > maxDistToSaveWaypoints:
                break
            else:
                for savedWaypoint in reversed(self.savedWaypoints):
                    waypointsDistDiff = self.dist(savedWaypoint[0], savedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if waypointsDistDiff < waypointsDistTollerance:
                        self.savedWaypoints.remove(savedWaypoint)
                        break

                if (self.preliminaryLoopClosure):
                    distDiff = self.dist(firstSavedWaypoint[0], firstSavedWaypoint[1], waypointCandidate[0], waypointCandidate[1])
                    if distDiff < waypointsDistTollerance:
                        self.loopClosure = False
                        break

                self.savedWaypoints.append(waypointCandidate)
                newSavedPoints.append(waypointCandidate)

        if newSavedPoints:
            for point in newSavedPoints:
                newWaypoints.remove(point)

    def getWaypointsFromEdges(self, filteredBranch, delaunayEdges):
        if not delaunayEdges:
            return

        waypoints = []
        for i in range (len(filteredBranch) - 1):
            node1 = filteredBranch[i]
            node2 = filteredBranch[i+1]
            a1 = np.array([node1.x, node1.y])
            a2 = np.array([node2.x, node2.y])

            maxAcceptedEdgeLength = self.rrt_waypoint_max_accepted_edge_length
            maxEdgePartsRatio = self.rrt_waypoint_max_edge_parts_ratio

            intersectedEdges = []
            for edge in delaunayEdges:

                b1 = np.array([edge.x1, edge.y1])
                b2 = np.array([edge.x2, edge.y2])

                # 1) RRT 브랜치(a1~a2)와 Delaunay 에지(b1~b2)가 교차하는지 확인
                if self.getLineSegmentIntersection(a1, a2, b1, b2):
                    
                    # 2) 에지 길이 및 분할 비율 제한
                    if edge.length() < maxAcceptedEdgeLength:
                        edge.intersection = self.getLineIntersection(a1, a2, b1, b2)

                        edgePartsRatio = edge.getPartsLengthRatio()

                        if edgePartsRatio < maxEdgePartsRatio:
                            intersectedEdges.append(edge)

            # 교차 에지가 있으면, 그 에지의 중간점(혹은 교차점)을 웨이포인트로 추가
            if intersectedEdges:

                if len(intersectedEdges) == 1:
                    edge = intersectedEdges[0]

                    waypoints.append(edge.getMiddlePoint())
                    
                # 여러 에지가 교차하면, 거리 순으로 정렬 후 모두 추가
                else:
                    intersectedEdges.sort(key=lambda edge: self.dist(node1.x, node1.y, edge.intersection[0], edge.intersection[1], shouldSqrt = False))

                    for edge in intersectedEdges:
                        waypoints.append(edge.getMiddlePoint())

        return waypoints

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4:
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.x, cone.y])

        tri = Delaunay(conePoints)

        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)

        return delaunayEdges

    def dist(self, x1, y1, x2, y2, shouldSqrt = True):
        distSq = (x1 - x2) ** 2 + (y1 - y2) ** 2
        return math.sqrt(distSq) if shouldSqrt else distSq

    def publishWaypoints(self, newWaypoints = None):
        if (time.time() - self.lastPublishWaypointsTime) < self.waypointsPublishInterval:
            return

        waypointsArray = WaypointsArray()
        newwaypointsArray = WaypointsArray()
        waypointsArray.header.frame_id = self.world_frame
        waypointsArray.header.stamp = self._now()
        newwaypointsArray.header.frame_id = self.world_frame
        newwaypointsArray.header.stamp = self._now()

        for i in range(len(self.savedWaypoints)):
            waypoint = self.savedWaypoints[i]
            waypointId = len(waypointsArray.waypoints)
            w = Waypoint()
            w.x = float(waypoint[0])
            w.y = float(waypoint[1])
            w.id = float(waypointId)
            waypointsArray.waypoints.append(w)

        if newWaypoints is not None:
            for i in range(len(newWaypoints)):
                waypoint = newWaypoints[i]
                waypointId = len(waypointsArray.waypoints)
                w = Waypoint()
                w.x = float(waypoint[0])
                w.y = float(waypoint[1])
                w.id = float(waypointId)
                waypointsArray.waypoints.append(w)
                newwaypointsArray.waypoints.append(w)

        if self.shouldPublishWaypoints:
            self.waypointsPub.publish(waypointsArray)
            self.newwaypointsPub.publish(newwaypointsArray)
            self.lastPublishWaypointsTime = time.time()
            self.publishWaypointsVisuals(newWaypoints)


    def publishWaypointsVisuals(self, newWaypoints = None):

        markerArray = MarkerArray()

        savedWaypointsMarker = Marker()
        savedWaypointsMarker.header.frame_id = self.world_frame
        savedWaypointsMarker.header.stamp = self._now()
        savedWaypointsMarker.lifetime = self._duration(1)
        savedWaypointsMarker.ns = "saved-publishWaypointsVisuals"
        savedWaypointsMarker.id = 1

        savedWaypointsMarker.type = savedWaypointsMarker.SPHERE_LIST
        savedWaypointsMarker.action = savedWaypointsMarker.ADD
        savedWaypointsMarker.pose.orientation.w = 1.0
        savedWaypointsMarker.scale.x = 0.15
        savedWaypointsMarker.scale.y = 0.15
        savedWaypointsMarker.scale.z = 0.15

        savedWaypointsMarker.color.a = 1.0
        savedWaypointsMarker.color.r = 0.0
        savedWaypointsMarker.color.g = 1.0
        savedWaypointsMarker.color.b = 1.0

        for waypoint in self.savedWaypoints:
            p = self._point(waypoint[0], waypoint[1], 0.0)
            savedWaypointsMarker.points.append(p)

        markerArray.markers.append(savedWaypointsMarker)

        if newWaypoints is not None:
            newWaypointsMarker = Marker()
            newWaypointsMarker.header.frame_id = self.world_frame
            newWaypointsMarker.header.stamp = self._now()
            newWaypointsMarker.lifetime = self._duration(1)
            newWaypointsMarker.ns = "new-publishWaypointsVisuals"
            newWaypointsMarker.id = 2

            newWaypointsMarker.type = newWaypointsMarker.SPHERE_LIST
            newWaypointsMarker.action = newWaypointsMarker.ADD
            newWaypointsMarker.pose.orientation.w = 1.0
            newWaypointsMarker.scale.x = 0.25
            newWaypointsMarker.scale.y = 0.25
            newWaypointsMarker.scale.z = 0.25

            newWaypointsMarker.color.a = 1.0
            newWaypointsMarker.color.r = 0.0
            newWaypointsMarker.color.g = 1.0
            newWaypointsMarker.color.b = 1.0

            for waypoint in newWaypoints:
                p = self._point(waypoint[0], waypoint[1], 0.0)
                newWaypointsMarker.points.append(p)

            markerArray.markers.append(newWaypointsMarker)

        self.waypointsVisualPub.publish(markerArray)

    def getLineIntersection(self, a1, a2, b1, b2):
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def getLineSegmentIntersection(self, a1, a2, b1, b2):
        return self.ccw(a1,b1,b2) != self.ccw(a2,b1,b2) and self.ccw(a1,a2,b1) != self.ccw(a1,a2,b2)

    def ccw(self, A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    def getFilteredBestBranch(self, bestBranch):
        if not bestBranch:
            return

        everyPointDistChangeLimit = self.rrt_filter_dist_change_limit
        newPointFilter = self.rrt_filter_new_point_alpha
        maxDiscardAmountForReset = self.rrt_filter_max_discard_reset

        if not self.filteredBestBranch:
            self.filteredBestBranch = list(bestBranch)
        else:
            changeRate = 0
            shouldDiscard = False
            for i in range(len(bestBranch)):
                node = bestBranch[i]
                filteredNode = self.filteredBestBranch[i]

                dist = math.sqrt((node.x - filteredNode.x) ** 2 + (node.y - filteredNode.y) ** 2)
                if dist > everyPointDistChangeLimit:
                    shouldDiscard = True
                    self.discardAmount += 1

                    if self.discardAmount >= maxDiscardAmountForReset:
                        self.discardAmount = 0
                        self.filteredBestBranch = list(bestBranch)
                    break

                changeRate += (everyPointDistChangeLimit - dist)

            if not shouldDiscard:

                for i in range(len(bestBranch)):
                    self.filteredBestBranch[i].x = self.filteredBestBranch[i].x * (1 - newPointFilter) + newPointFilter * bestBranch[i].x
                    self.filteredBestBranch[i].y = self.filteredBestBranch[i].y * (1 - newPointFilter) + newPointFilter * bestBranch[i].y

                self.discardAmount = 0

        self.publishFilteredBranchVisual()
        return list(self.filteredBestBranch)

    def publishDelaunayEdgesVisual(self, edges):
        if not edges:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self._now()
        marker.lifetime = self._duration(1)
        marker.ns = "publishDelaunayLinesVisual"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0

        for edge in edges:
            # print edge

            p1 = self._point(edge.x1, edge.y1, 0.0)
            p2 = self._point(edge.x2, edge.y2, 0.0)

            marker.points.append(p1)
            marker.points.append(p2)

        self.delaunayLinesVisualPub.publish(self._single_marker_array(marker))

    def findBestBranch(self, leafNodes, nodeList, largerGroupFrontCones):
        if not leafNodes:
            return

        coneDistLimit = self.rrt_branch_cone_dist_limit
        coneDistanceLimitSq = coneDistLimit * coneDistLimit

        bothSidesImproveFactor = self.rrt_branch_both_sides_improve_factor
        minAcceptableBranchRating = self.rrt_branch_min_acceptable_rating

        leafRatings = []
        for leaf in leafNodes:
            branchRating = 0
            node = leaf

            while node.parent is not None:
                nodeRating = 0

                leftCones = []
                rightCones = []

                for cone in largerGroupFrontCones:
                    coneDistSq = ((cone.x - node.x) ** 2 + (cone.y - node.y) ** 2)

                    if coneDistSq < coneDistanceLimitSq:
                        actualDist = math.sqrt(coneDistSq)

                        if actualDist < self.rrt_cone_obstacle_radius:
                            continue

                        nodeRating += (coneDistLimit - actualDist)

                        if self.isLeftCone(node, nodeList[node.parent], cone):
                            leftCones.append(cone)
                        else:
                            rightCones.append(cone)

                if ((len(leftCones) == 0 and len(rightCones)) > 0 or (len(leftCones) > 0 and len(rightCones) == 0)):
                    nodeRating /= bothSidesImproveFactor

                if (len(leftCones) > 0 and len(rightCones) > 0):
                    nodeRating *= bothSidesImproveFactor

                nodeFactor = (
                    (node.cost - self.rrt_expand_distance)
                    / (self.rrt_plan_distance - self.rrt_expand_distance)
                ) + 1

                branchRating += nodeRating * nodeFactor
                node = nodeList[node.parent]

            leafRatings.append(branchRating)

        maxRating = max(leafRatings)
        maxRatingInd = leafRatings.index(maxRating)

        node = leafNodes[maxRatingInd]

        if maxRating < minAcceptableBranchRating:
            return

        self.publishBestBranchVisual(nodeList, node)

        reverseBranch = []
        reverseBranch.append(node)
        while node.parent is not None:
            node = nodeList[node.parent]
            reverseBranch.append(node)

        directBranch = []
        for n in reversed(reverseBranch):
            directBranch.append(n)

        return directBranch

    def isLeftCone(self, node, parentNode, cone):
        return ((node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x)) > 0;

    def publishBestBranchVisual(self, nodeList, leafNode):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self._now()
        marker.lifetime = self._duration(0.2)
        marker.ns = "publishBestBranchVisual"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.775
        marker.pose.orientation.w = 1.0
        marker.color.a = 0.7
        marker.color.r = 0.0
        marker.color.g = 122.0 / 255.0
        marker.color.b = 204.0 / 255.0

        node = leafNode

        parentNodeInd = node.parent
        while parentNodeInd is not None:
            parentNode = nodeList[parentNodeInd]
            p = self._point(node.x, node.y, 0.0)
            marker.points.append(p)

            p = self._point(parentNode.x, parentNode.y, 0.0)
            marker.points.append(p)

            parentNodeInd = node.parent
            node = parentNode

        self.bestBranchVisualPub.publish(self._single_marker_array(marker))

    def publishFilteredBranchVisual(self):

        if not self.filteredBestBranch:
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self._now()
        marker.lifetime = self._duration(0.2)
        marker.ns = "publishFilteredBranchVisual"
        marker.type = marker.LINE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.07
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.b = 1.0

        for i in range(len(self.filteredBestBranch)):
            node = self.filteredBestBranch[i]
            p = self._point(node.x, node.y, 0.0)
            if i != 0:
                marker.points.append(p)

            if i != len(self.filteredBestBranch) - 1:
                marker.points.append(p)

        self.filteredBranchVisualPub.publish(self._single_marker_array(marker))

    def publishTreeVisual(self, nodeList, leafNodes):

        if not nodeList and not leafNodes:
            return

        markerArray = MarkerArray()

        # tree lines marker
        treeMarker = Marker()
        treeMarker.header.frame_id = self.world_frame
        treeMarker.header.stamp = self._now()
        treeMarker.ns = "rrt"

        treeMarker.type = treeMarker.LINE_LIST
        treeMarker.action = treeMarker.ADD
        treeMarker.scale.x = 0.03
        treeMarker.pose.orientation.w = 1.0
        treeMarker.color.a = 0.9
        treeMarker.color.r = 0.0
        treeMarker.color.g = 122.0 / 255.0
        treeMarker.color.b = 204.0 / 255.0

        treeMarker.lifetime = self._duration(0.2)

        for node in nodeList:
            if node.parent is not None:
                p = self._point(node.x, node.y, 0.0)
                treeMarker.points.append(p)

                p = self._point(nodeList[node.parent].x, nodeList[node.parent].y, 0.0)
                treeMarker.points.append(p)

        markerArray.markers.append(treeMarker)

        # leaves nodes marker
        leavesMarker = Marker()
        leavesMarker.header.frame_id = self.world_frame
        leavesMarker.header.stamp = self._now()
        leavesMarker.lifetime = self._duration(0.2)
        leavesMarker.ns = "rrt-leaves"

        leavesMarker.type = leavesMarker.SPHERE_LIST
        leavesMarker.action = leavesMarker.ADD
        leavesMarker.pose.orientation.w = 1.0
        leavesMarker.scale.x = 0.05
        leavesMarker.scale.y = 0.05
        leavesMarker.scale.z = 0.05

        leavesMarker.color.a = 0.7
        leavesMarker.color.r = 0.0
        leavesMarker.color.g = 122.0 / 255.0
        leavesMarker.color.b = 204.0 / 255.0

        for node in leafNodes:
            p = self._point(node.x, node.y, 0.0)
            leavesMarker.points.append(p)

        markerArray.markers.append(leavesMarker)

        # publis marker array
        self.treeVisualPub.publish(markerArray)

    def getFrontConeObstacles(self, map, frontDist):
        if not map:
            return []

        headingVector = self.getHeadingVector()
        headingVectorOrt = [-headingVector[1], headingVector[0]]

        behindDist = self.rrt_front_behind_dist
        carPosBehindPoint = [self.carPosX - behindDist * headingVector[0], self.carPosY - behindDist * headingVector[1]]


        frontDistSq = frontDist ** 2

        frontConeList = []
        for cone in map:
            if (headingVectorOrt[0] * (cone.y - carPosBehindPoint[1]) - headingVectorOrt[1] * (cone.x - carPosBehindPoint[0])) < 0:
                if ((cone.x) ** 2 + (cone.y) ** 2) < frontDistSq:
                    frontConeList.append(cone)
        return frontConeList

    def getHeadingVector(self):
        headingVector = [1.0, 0]
        carRotMat = np.array([[math.cos(self.carPosYaw), -math.sin(self.carPosYaw)], [math.sin(self.carPosYaw), math.cos(self.carPosYaw)]])
        headingVector = np.dot(carRotMat, headingVector)
        return headingVector

    def getConesInRadius(self, map, x, y, radius):
        coneList = []
        radiusSq = radius * radius
        for cone in map:
            if ((cone.x - x) ** 2 + (cone.y - y) ** 2) < radiusSq:
                coneList.append(cone)
        return coneList
    
class Edge():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.intersection = None

    def getMiddlePoint(self):
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)



def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPathPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
