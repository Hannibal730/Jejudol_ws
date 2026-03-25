#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, Int32, String
from visualization_msgs.msg import Marker, MarkerArray


class DecisionFlowchartVisualizer(Node):
    def __init__(self):
        super().__init__('decision_flowchart_visualizer')

        self.declare_parameter('frame_id', 'velodyne')
        self.declare_parameter('origin_x', 11.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('origin_z', 4.0)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('text_scale', 0.35)
        self.declare_parameter('node_size_x', 1.0) # 사실 세로
        self.declare_parameter('node_size_y', 2.85) # 사실 가로
        self.declare_parameter('node_size_z', 0.12)
        self.declare_parameter('layout_scale', 1.35)
        self.declare_parameter('horizontal_spacing_scale', 0.8) # 사실 세로
        self.declare_parameter('vertical_spacing_scale', 1.15) # 사실 가로
        self.declare_parameter('end_idx_topic', '/gps/roi_end_idx')
        self.declare_parameter('end_idx_min', 225)
        self.declare_parameter('end_idx_max', 530)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_z = float(self.get_parameter('origin_z').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.text_scale = float(self.get_parameter('text_scale').value)
        self.node_size_x = float(self.get_parameter('node_size_x').value)
        self.node_size_y = float(self.get_parameter('node_size_y').value)
        self.node_size_z = float(self.get_parameter('node_size_z').value)
        self.layout_scale = float(self.get_parameter('layout_scale').value)
        self.horizontal_spacing_scale = float(self.get_parameter('horizontal_spacing_scale').value)
        self.vertical_spacing_scale = float(self.get_parameter('vertical_spacing_scale').value)
        self.end_idx_topic = str(self.get_parameter('end_idx_topic').value)
        self.end_idx_min = int(self.get_parameter('end_idx_min').value)
        self.end_idx_max = int(self.get_parameter('end_idx_max').value)

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 15.0
        if self.text_scale <= 0.0:
            self.text_scale = 0.35
        if self.node_size_x <= 0.0:
            self.node_size_x = 2.2
        if self.node_size_y <= 0.0:
            self.node_size_y = 0.9
        if self.node_size_z <= 0.0:
            self.node_size_z = 0.12
        if self.layout_scale <= 0.0:
            self.layout_scale = 1.35
        if self.horizontal_spacing_scale <= 0.0:
            self.horizontal_spacing_scale = 1.35
        if self.vertical_spacing_scale <= 0.0:
            self.vertical_spacing_scale = 1.0
        if self.end_idx_min > self.end_idx_max:
            self.end_idx_min, self.end_idx_max = self.end_idx_max, self.end_idx_min

        self.mission_state = 'unknown'
        self.lane_detect = False
        self.end_idx = 0
        self.is_emergency = False
        self.num_lidar_cone = 0

        self.create_subscription(String, '/mission_state', self._mission_state_cb, 10)
        self.create_subscription(Bool, '/lane_detect', self._lane_detect_cb, 10)
        self.create_subscription(Int32, self.end_idx_topic, self._end_idx_cb, 10)
        self.create_subscription(Bool, '/emergency', self._emergency_cb, 10)
        self.create_subscription(Int8, '/vn/num_lidar_cone', self._num_lidar_cone_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/decision/flowchart_marker', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        # Relative layout in XY plane.
        self.node_specs = {
            'd1_lane': {'pos': (0.0, 0.0), 'kind':'decision','label':'lane_detect?'},
            'd2_end_idx': {'pos': (3.0, 1.6), 'kind':'decision','label': ''},
            'd3_emergency': {'pos': (6.0, 1.6), 'kind':'decision','label': 'emergency?'},
            'd4_cone': {'pos': (3.0, -1.6), 'kind':'decision','label': 'num_lidar_cone>0?'},
            's_rrt': {'pos': (9.0, 3.0), 'kind':'state','label': 'MISSION_RRT'},
            's_emergency': {'pos': (9.0, 1.6), 'kind':'state','label': 'MISSION_EMERGENCY'},
            's_yolotl': {'pos': (9.0, 0.2), 'kind':'state','label': 'MISSION_YOLOTL'},
            's_gps': {'pos': (9.0, -1.6), 'kind':'state','label': 'MISSION_GPS'},
        }

        self.edge_specs = [
            ('e_d1_d2', 'd1_lane', 'd2_end_idx', 'yes'),
            ('e_d1_d4', 'd1_lane', 'd4_cone', 'no'),
            ('e_d2_rrt', 'd2_end_idx', 's_rrt', 'yes'),
            ('e_d2_d3', 'd2_end_idx', 'd3_emergency', 'no'),
            ('e_d3_emergency', 'd3_emergency', 's_emergency', 'yes'),
            ('e_d3_yolotl', 'd3_emergency', 's_yolotl', 'no'),
            ('e_d4_rrt', 'd4_cone', 's_rrt', 'yes'),
            ('e_d4_gps', 'd4_cone', 's_gps', 'no'),
        ]

        self.get_logger().info(
            'decision_flowchart_visualizer started: frame=%s, origin=(%.2f, %.2f, %.2f), '
            'layout_scale=%.2f, horizontal_spacing_scale=%.2f, vertical_spacing_scale=%.2f, end_idx_topic=%s, end_idx_range=[%d,%d]'
            % (
                self.frame_id,
                self.origin_x,
                self.origin_y,
                self.origin_z,
                self.layout_scale,
                self.horizontal_spacing_scale,
                self.vertical_spacing_scale,
                self.end_idx_topic,
                self.end_idx_min,
                self.end_idx_max,
            )
        )

    def _mission_state_cb(self, msg: String):
        self.mission_state = msg.data

    def _lane_detect_cb(self, msg: Bool):
        self.lane_detect = bool(msg.data)

    def _end_idx_cb(self, msg: Int32):
        self.end_idx = int(msg.data)

    def _emergency_cb(self, msg: Bool):
        self.is_emergency = bool(msg.data)

    def _num_lidar_cone_cb(self, msg: Int8):
        self.num_lidar_cone = int(msg.data)

    @staticmethod
    def _scale_color(rgb, factor):
        return (
            min(1.0, rgb[0] * factor),
            min(1.0, rgb[1] * factor),
            min(1.0, rgb[2] * factor),
        )

    def _abs_from_rel(self, rx, ry, z_offset=0.0):
        # Rotate whole flowchart 90 degrees to the right (clockwise): (x, y) -> (y, -x)
        # and scale layout to lengthen node-to-node edges.
        sx = rx * self.layout_scale * self.vertical_spacing_scale
        sy = ry * self.layout_scale * self.horizontal_spacing_scale
        p = Point()
        p.x = self.origin_x + sy
        p.y = self.origin_y - sx
        p.z = self.origin_z + z_offset
        return p

    def _abs_point(self, key, z_offset=0.0):
        rx, ry = self.node_specs[key]['pos']
        return self._abs_from_rel(rx, ry, z_offset)

    def _compute_expected_state(self):
        in_end_idx_range = self.end_idx_min <= self.end_idx <= self.end_idx_max
        if self.lane_detect:
            if in_end_idx_range:
                return 'rrt'
            if self.is_emergency:
                return 'emergency'
            return 'yolotl'
        if self.num_lidar_cone > 0:
            return 'rrt'
        return 'gps'

    def _compute_active_path(self):
        in_end_idx_range = self.end_idx_min <= self.end_idx <= self.end_idx_max
        active_nodes = {'d1_lane'}
        active_edges = set()

        if self.lane_detect:
            active_nodes.add('d2_end_idx')
            active_edges.add('e_d1_d2')
            if in_end_idx_range:
                active_nodes.add('s_rrt')
                active_edges.add('e_d2_rrt')
            else:
                active_nodes.add('d3_emergency')
                active_edges.add('e_d2_d3')
                if self.is_emergency:
                    active_nodes.add('s_emergency')
                    active_edges.add('e_d3_emergency')
                else:
                    active_nodes.add('s_yolotl')
                    active_edges.add('e_d3_yolotl')
        else:
            active_nodes.add('d4_cone')
            active_edges.add('e_d1_d4')
            if self.num_lidar_cone > 0:
                active_nodes.add('s_rrt')
                active_edges.add('e_d4_rrt')
            else:
                active_nodes.add('s_gps')
                active_edges.add('e_d4_gps')

        return active_nodes, active_edges

    def _make_box_marker(self, marker_id, node_key, color_rgb):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'flowchart_boxes'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        pos = self._abs_point(node_key)
        marker.pose.position = pos
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.node_size_x
        marker.scale.y = self.node_size_y
        marker.scale.z = self.node_size_z

        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 0.85
        return marker

    def _make_text_marker(self, marker_id, ns, text, point, color_rgb, scale=None):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.text_scale if scale is None else scale

        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 1.0
        marker.text = text
        return marker

    def _make_arrow_marker(self, marker_id, edge_id, src_key, dst_key, color_rgb, shaft_diameter):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'flowchart_edges'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = shaft_diameter
        marker.scale.y = shaft_diameter * 1.7
        marker.scale.z = shaft_diameter * 1.7

        marker.color.r = float(color_rgb[0])
        marker.color.g = float(color_rgb[1])
        marker.color.b = float(color_rgb[2])
        marker.color.a = 0.95

        start = self._abs_point(src_key, z_offset=0.01)
        end = self._abs_point(dst_key, z_offset=0.01)
        marker.points = [start, end]
        return marker

    def _on_timer(self):
        pulse = 0.65 + 0.35 * (0.5 + 0.5 * math.sin(time.monotonic() * 6.0))
        expected_state = self._compute_expected_state()
        active_nodes, active_edges = self._compute_active_path()

        mission_node_by_state = {
            'rrt': 's_rrt',
            'emergency': 's_emergency',
            'yolotl': 's_yolotl',
            'gps': 's_gps',
        }
        mission_node_key = mission_node_by_state.get(self.mission_state, None)
        mission_match = (self.mission_state == expected_state)

        state_base_colors = {
            's_rrt': (0.1, 0.3, 1.0),
            's_emergency': (1.0, 0.1, 0.1),
            's_yolotl': (0.1, 0.9, 0.2),
            's_gps': (0.72, 0.25, 0.95),
        }

        marker_array = MarkerArray()

        # Node markers (box + text)
        node_keys = list(self.node_specs.keys())
        for idx, key in enumerate(node_keys):
            spec = self.node_specs[key]

            if spec['kind'] == 'decision':
                # Slightly darker inactive nodes for clearer active/inactive contrast.
                base = (0.15, 0.15, 0.15)
                active = (0.25, 0.90, 1.00)
            else:
                base = self._scale_color(state_base_colors[key], 0.20)
                active = state_base_colors[key]

            color = base
            if key in active_nodes:
                color = self._scale_color(active, pulse)

            if mission_node_key == key:
                if mission_match:
                    color = (0.15, 1.0, 0.25)
                else:
                    color = (1.0, 0.25 + 0.5 * pulse, 0.15)

            marker_array.markers.append(self._make_box_marker(idx, key, color))

            label = spec['label']
            if key == 'd2_end_idx':
                label = f'end_idx[{self.end_idx_min},{self.end_idx_max}]?'

            text_point = self._abs_point(key, z_offset=0.35)
            text_color = (1.0, 1.0, 1.0)
            marker_array.markers.append(
                self._make_text_marker(
                    marker_id=100 + idx,
                    ns='flowchart_node_text',
                    text=label,
                    point=text_point,
                    color_rgb=text_color,
                )
            )

        # Edge markers (arrow + yes/no label)
        for idx, (edge_id, src, dst, label) in enumerate(self.edge_specs):
            edge_active = edge_id in active_edges
            edge_color = (0.35, 0.35, 0.35)
            shaft = 0.05
            if edge_active:
                edge_color = (1.0, 0.95 * pulse, 0.15)
                shaft = 0.11

            marker_array.markers.append(
                self._make_arrow_marker(
                    marker_id=200 + idx,
                    edge_id=edge_id,
                    src_key=src,
                    dst_key=dst,
                    color_rgb=edge_color,
                    shaft_diameter=shaft,
                )
            )

            src_pt = self._abs_point(src, z_offset=0.28)
            dst_pt = self._abs_point(dst, z_offset=0.28)
            mid = Point()
            mid.x = (src_pt.x + dst_pt.x) * 0.5
            mid.y = (src_pt.y + dst_pt.y) * 0.5
            mid.z = (src_pt.z + dst_pt.z) * 0.5 + 0.15
            label_color = (1.0, 1.0, 1.0) if edge_active else (0.7, 0.7, 0.7)

            marker_array.markers.append(
                self._make_text_marker(
                    marker_id=300 + idx,
                    ns='flowchart_edge_text',
                    text=label,
                    point=mid,
                    color_rgb=label_color,
                    scale=self.text_scale * 0.8,
                )
            )

        # Status panel
        status = (
            f'lane_detect={str(self.lane_detect).lower()}  end_idx={self.end_idx}  '
            f'range=[{self.end_idx_min},{self.end_idx_max}]\n'
            f'emergency={str(self.is_emergency).lower()}  num_lidar_cone={self.num_lidar_cone}\n'
            f'expected={expected_state}  mission_state={self.mission_state}'
        )
        status_color = (0.2, 1.0, 0.2) if mission_match else (1.0, 0.35, 0.2)
        status_pos = self._abs_from_rel(5.5, -3.1, z_offset=0.8)
        marker_array.markers.append(
            self._make_text_marker(
                marker_id=900,
                ns='flowchart_status',
                text=status,
                point=status_pos,
                color_rgb=status_color,
                scale=self.text_scale * 0.9,
            )
        )

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionFlowchartVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
