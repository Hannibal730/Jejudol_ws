#!/usr/bin/env python3

import sys
import os

# 로그 포맷 설정: 노드 이름([VoxelNeXt_center_object_detect]) 제거
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time}]: {message}'
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}]: {message}'


import rclpy
from rclpy.node import Node
import torch
import numpy as np
import queue
import threading
import time
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2 as pc2

script_path = os.path.realpath(__file__)
sys.path.append(os.path.dirname(script_path))
sys.path.insert(0, os.path.dirname(os.path.dirname(script_path)))

from model import load_voxelnext
import math

from builtin_interfaces.msg import Duration

# -------------------------
# ROI configuration for counting detections in velodyne frame
# -------------------------
ROI_X_MIN = 0.0
ROI_X_MAX = 3.0
ROI_Y_MIN = -1.0
ROI_Y_MAX = 1.0

# -------------------------
# Define colors for each class
# -------------------------
color_map = {
    'car': [1, 0.5, 0.5], # Light Red
    'truck': [1, 0, 0], 
    'construction_vehicle': [0, 0, 1], # Blue
    'bus': [1, 1, 0], # Yellow
    'trailer': [1, 0, 1], # Magenta
    'barrier': [0, 1, 1], # Cyan
    'motorcycle': [0.5, 0.5, 0.5], # Gray
    'bicycle': [1, 0.5, 0], # Orange
    'pedestrian': [0.5, 0, 0.5], # Purple
    'traffic_cone': [0, 1, 0]  
}
default_color = [0, 0, 0]   # Default: Black


# -------------------------
# Define class number for each class
# -------------------------
nuscenes_class_names = [
    'car',                   # 1
    'truck',                 # 2
    'construction_vehicle',  # 3
    'bus',                   # 4
    'trailer',               # 5
    'barrier',               # 6
    'motorcycle',            # 7
    'bicycle',               # 8
    'pedestrian',            # 9
    'traffic_cone'           # 10
]

# -------------------------
# Function: Convert PointCloud2 message to a NumPy array in (N, 5) format
# -------------------------
_pc2_dtype_cache = {}


def _get_pc2_dtype(msg):
    key = (
        msg.point_step,
        tuple((f.name, f.offset, f.datatype, f.count) for f in msg.fields),
    )
    dtype = _pc2_dtype_cache.get(key)
    if dtype is None:
        dtype = pc2.dtype_from_fields(msg.fields, point_step=msg.point_step)
        _pc2_dtype_cache[key] = dtype
    return dtype


def pointcloud2_to_numpy(msg):
    """
    Convert a ROS PointCloud2 message to a NumPy array with shape (N, 5).
    - Format: [x, y, z, intensity, timestamp]
    - Extracts only points from the region of interest (ROI).
    """
    num_raw_points = int(msg.width) * int(msg.height)
    if num_raw_points == 0:
        return np.zeros((0, 5), dtype=np.float32)

    dtype = _get_pc2_dtype(msg)
    points = np.frombuffer(msg.data, dtype=dtype, count=num_raw_points)

    if bool(sys.byteorder != 'little') != bool(msg.is_bigendian):
        points = points.byteswap().newbyteorder()

    required_fields = ("x", "y", "z", "intensity")
    if points.dtype.names is None or any(f not in points.dtype.names for f in required_fields):
        # Fallback for unexpected PointCloud2 field layout.
        fallback = pc2.read_points(msg, skip_nans=True, field_names=required_fields)
        if fallback.size == 0:
            return np.zeros((0, 5), dtype=np.float32)
        xyz_i = np.empty((fallback.shape[0], 4), dtype=np.float32)
        xyz_i[:, 0] = fallback["x"]
        xyz_i[:, 1] = fallback["y"]
        xyz_i[:, 2] = fallback["z"]
        xyz_i[:, 3] = fallback["intensity"]
    else:
        x = points["x"].astype(np.float32, copy=False)
        y = points["y"].astype(np.float32, copy=False)
        z = points["z"].astype(np.float32, copy=False)
        intensity = points["intensity"].astype(np.float32, copy=False)

        if msg.is_dense:
            mask = None
        else:
            mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity)

        if mask is None:
            n = x.shape[0]
            xyz_i = np.empty((n, 4), dtype=np.float32)
            xyz_i[:, 0] = x
            xyz_i[:, 1] = y
            xyz_i[:, 2] = z
            xyz_i[:, 3] = intensity
        else:
            n = int(mask.sum())
            if n == 0:
                return np.zeros((0, 5), dtype=np.float32)
            xyz_i = np.empty((n, 4), dtype=np.float32)
            xyz_i[:, 0] = x[mask]
            xyz_i[:, 1] = y[mask]
            xyz_i[:, 2] = z[mask]
            xyz_i[:, 3] = intensity[mask]

    points_with_timestamp = np.empty((xyz_i.shape[0], 5), dtype=np.float32)
    points_with_timestamp[:, :4] = xyz_i
    points_with_timestamp[:, 4] = 0.0
    return points_with_timestamp
    
class CenterObjectDetect(Node):
    def __init__(self):
        super().__init__('VoxelNeXt_center_object_detect')
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Define the project root directory (assumed to be one level up from the current script)
        project_dir = os.path.abspath(os.path.join(script_dir, '..'))

        # Change the working directory to the project root (important for resolving relative paths)
        os.chdir(project_dir)

        # Define absolute paths for the configuration file and the model checkpoint
        config_path = os.path.join(project_dir, 'tools', 'cfgs', 'nuscenes_models', 'cbgs_voxel0075_voxelnext.yaml')
        model_checkpoint = os.path.join(project_dir, 'checkpoints', 'voxelnext_nuscenes_kernel1.pth')

        self.get_logger().info(f"Config Path: {config_path}")
        self.get_logger().info(f"Model Checkpoint Path: {model_checkpoint}")
        self.get_logger().info(f"Config file exists: {os.path.exists(config_path)}")
        self.get_logger().info(f"Model checkpoint exists: {os.path.exists(model_checkpoint)}")

        # Exit if configuration or model checkpoint is not found
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found: {config_path}")
            sys.exit(1)
        if not os.path.exists(model_checkpoint):
            self.get_logger().error(f"Model checkpoint not found: {model_checkpoint}")
            sys.exit(1)

        # Load the VoxelNeXt model and associated lidar dataset
        self.voxelnext_model, self.lidar_dataset = load_voxelnext(config_path, model_checkpoint)
        self.voxelnext_model.eval() # Set the model to evaluation mode
        self.get_logger().info("✅ VoxelNeXt model load completed")

        # Create a ROS publisher for detected objects (center points markers)
        self.pub_detected_centers = self.create_publisher(MarkerArray, '/vn/detected_center', 10)
        self.get_logger().info("✅ Publishers for /vn/detected_center created")

        self.pub_detected_class   = self.create_publisher(MarkerArray, '/vn/detected_class', 10)
        self.get_logger().info("✅ Publishers for /vn/detected_class created")

        self.pub_num_detected = self.create_publisher(Int8, '/vn/num_detected', 10)
        self.num_detected_msg = Int8()
        self.get_logger().info("✅ Publisher for /vn/num_detected created")

        self.pub_num_detected_roi = self.create_publisher(Int8, '/vn/num_detected_roi', 10)
        self.num_detected_roi_msg = Int8()
        self.get_logger().info("✅ Publisher for /vn/num_detected_roi created")

        # Separate ROI visualization topic for num_detected_roi.
        self.pub_detected_roi = self.create_publisher(Marker, '/vn/detected_roi', 10)
        self.roi_marker = Marker()
        self.roi_marker.header.frame_id = "velodyne"
        self.roi_marker.ns = "detected_roi"
        self.roi_marker.id = 0
        self.roi_marker.type = Marker.LINE_STRIP
        self.roi_marker.action = Marker.ADD
        self.roi_marker.pose.orientation.w = 1.0
        self.roi_marker.scale.x = 0.08
        self.roi_marker.color.a = 1.0
        self.roi_marker.lifetime = Duration(sec=0, nanosec=200000000)
        self.roi_marker.points = [
            Point(x=ROI_X_MIN, y=ROI_Y_MIN, z=0.0),
            Point(x=ROI_X_MAX, y=ROI_Y_MIN, z=0.0),
            Point(x=ROI_X_MAX, y=ROI_Y_MAX, z=0.0),
            Point(x=ROI_X_MIN, y=ROI_Y_MAX, z=0.0),
            Point(x=ROI_X_MIN, y=ROI_Y_MIN, z=0.0),
        ]
        self.latest_roi_detected_count = 0
        self.roi_visual_timer = self.create_timer(0.2, self.publish_roi_marker)
        self.get_logger().info("✅ Publisher for /vn/detected_roi created")

        # Create a ROS subscriber to receive PointCloud2 messages from the LiDAR sensor
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_callback,
            1) # QoS profile 1 for compatibility with buff_size
        self.get_logger().info("✅ Subscriber for '/velodyne_points' created")
        self.get_logger().info("🚀 Now everything is ready. Run the rosbag file or launch the Velodyne LiDAR")

        # Filtering Class for Autonomous Driving Competition
        self.target_classes = ['traffic_cone']

        # Timing diagnostics
        self.declare_parameter('timing_log_every_n', 1)
        self.declare_parameter('slow_frame_threshold_ms', 120.0)
        self.declare_parameter('expected_output_hz', 10.0)
        self.declare_parameter('sync_cuda_for_timing', True)

        self.timing_log_every_n = max(1, int(self.get_parameter('timing_log_every_n').value))
        self.slow_frame_threshold_ms = float(self.get_parameter('slow_frame_threshold_ms').value)
        self.expected_output_hz = max(0.1, float(self.get_parameter('expected_output_hz').value))
        self.sync_cuda_for_timing = bool(self.get_parameter('sync_cuda_for_timing').value)
        self.frame_index = 0
        self.dropped_input_frames = 0
        self.prev_frame_start_wall = None
        self.prev_publish_wall = None
        self.get_logger().info(
            "⏱️ Timing enabled: pre/voxel/infer/publish "
            f"(every_n={self.timing_log_every_n}, slow>{self.slow_frame_threshold_ms:.1f}ms)"
        )

        # Keep only the latest frame to avoid latency accumulation.
        self.frame_queue = queue.Queue(maxsize=1)
        self.worker_running = True
        self.inference_thread = threading.Thread(target=self.inference_worker, daemon=True)
        self.inference_thread.start()

    def lidar_callback(self, msg):
        self.enqueue_latest_frame(msg)

    def enqueue_latest_frame(self, msg):
        # Drop stale frame when queue is full so inference always uses latest data.
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
                self.dropped_input_frames += 1
            except queue.Empty:
                pass

        try:
            self.frame_queue.put_nowait(msg)
        except queue.Full:
            # Another thread may have filled the queue between full() and put_nowait().
            pass

    def inference_worker(self):
        while self.worker_running and rclpy.ok():
            try:
                msg = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if msg is None:
                break

            frame_idx = self.frame_index
            self.frame_index += 1
            frame_start = time.perf_counter()
            input_period_ms = None
            if self.prev_frame_start_wall is not None:
                input_period_ms = (frame_start - self.prev_frame_start_wall) * 1000.0
            self.prev_frame_start_wall = frame_start

            try:
                preprocess_start = time.perf_counter()
                points = pointcloud2_to_numpy(msg)
                preprocess_ms = (time.perf_counter() - preprocess_start) * 1000.0
                if points.shape[1] != 5:
                    self.get_logger().warn(f"❌ Incorrect point format! Expected (N,5), got: {points.shape}")
                    continue

                output_dicts, detect_timing = self.detect_objects(points, self.voxelnext_model, self.lidar_dataset)

                publish_start = time.perf_counter()
                publish_info = self.publish_markers(
                    output_dicts,
                    self.pub_detected_centers,
                    self.pub_detected_class,
                    self.voxelnext_model.class_names
                )
                publish_ms = (time.perf_counter() - publish_start) * 1000.0

                frame_total_ms = (time.perf_counter() - frame_start) * 1000.0
                publish_wall = time.perf_counter()
                output_period_ms = None
                if self.prev_publish_wall is not None:
                    output_period_ms = (publish_wall - self.prev_publish_wall) * 1000.0
                self.prev_publish_wall = publish_wall

                self.log_timing(
                    frame_idx=frame_idx,
                    preprocess_ms=preprocess_ms,
                    voxel_ms=detect_timing["voxel_ms"],
                    infer_ms=detect_timing["infer_ms"],
                    publish_ms=publish_ms,
                    total_ms=frame_total_ms,
                    input_period_ms=input_period_ms,
                    output_period_ms=output_period_ms,
                    center_marker_count=publish_info["center_marker_count"],
                    roi_detected_count=publish_info["roi_detected_count"],
                )
            except Exception as e:
                self.get_logger().error(f"❌ Frame {frame_idx} error during object detection/publishing: {e}")

    def destroy_node(self):
        self.worker_running = False

        # Wake up worker thread if it is waiting on queue.get().
        try:
            self.frame_queue.put_nowait(None)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass

        if hasattr(self, 'inference_thread') and self.inference_thread.is_alive():
            self.inference_thread.join(timeout=1.0)

        return super().destroy_node()

    def publish_roi_marker(self):
        roi_detected_count = self.latest_roi_detected_count
        self.roi_marker.header.stamp = self.get_clock().now().to_msg()

        if roi_detected_count == 0:
            self.roi_marker.color.r = 1.0
            self.roi_marker.color.g = 1.0
            self.roi_marker.color.b = 1.0
        elif roi_detected_count < 4:
            self.roi_marker.color.r = 1.0
            self.roi_marker.color.g = 0.55
            self.roi_marker.color.b = 0.0
        else:
            self.roi_marker.color.r = 1.0
            self.roi_marker.color.g = 0.0
            self.roi_marker.color.b = 0.0

        self.pub_detected_roi.publish(self.roi_marker)

    def detect_objects(self, points, voxelnext_model, lidar_dataset):
        # self.get_logger().info("Processing LiDAR data.")
        data_dict = {"points": points}

        # Perform point feature encoding
        data_dict = lidar_dataset.point_feature_encoder.forward(data_dict)

        # Process data using each processor in the dataset configuration
        voxel_start = time.perf_counter()
        for processor in lidar_dataset.dataset_cfg.DATA_PROCESSOR:
            if processor["NAME"] == "transform_points_to_voxels":
                voxels, coords, num_points_per_voxel = lidar_dataset.voxel_generator.generate(data_dict["points"])
                data_dict["voxels"] = voxels
                data_dict["voxel_coords"] = coords
                data_dict["voxel_num_points"] = num_points_per_voxel
        voxel_ms = (time.perf_counter() - voxel_start) * 1000.0

        # Prepare tensors for model inference
        device = next(voxelnext_model.parameters()).device
        voxel_coords_tensor = torch.from_numpy(data_dict["voxel_coords"]).int().to(device)

        infer_start = time.perf_counter()
        with torch.no_grad():
            batch_dict = {
                "batch_size": 1,
                "points": torch.from_numpy(data_dict["points"]).to(device),
                "voxels": torch.from_numpy(data_dict["voxels"]).to(device),
                "voxel_coords": voxel_coords_tensor,
                "voxel_num_points": torch.from_numpy(data_dict["voxel_num_points"]).to(device),
            }
            output_dicts, _ = voxelnext_model(batch_dict)
            if self.sync_cuda_for_timing and device.type == "cuda":
                torch.cuda.synchronize(device)
        infer_ms = (time.perf_counter() - infer_start) * 1000.0

        return output_dicts, {"voxel_ms": voxel_ms, "infer_ms": infer_ms}

    def publish_markers(self, output_dicts, pub_detected_centers, pub_detected_class, class_names):
        center_markers = MarkerArray()
        text_markers = MarkerArray()

        # Optimization 1: Get timestamp once per frame (avoids system calls inside loop)
        current_time = self.get_clock().now().to_msg()

        # Optimization 2: Pre-calculate target label indices for vector filtering
        target_indices = [class_names.index(c) + 1 for c in self.target_classes if c in class_names]
        roi_detected_count = 0

        for i, output in enumerate(output_dicts):
            # Optimization: Move tensors to CPU and convert to NumPy once before iterating
            # Calling .cpu().item() inside a loop causes severe GPU-CPU synchronization overhead.
            pred_boxes = output["pred_boxes"].cpu().numpy()
            pred_labels = output["pred_labels"].cpu().numpy()
            pred_scores = output["pred_scores"].cpu().numpy()

            # Optimization 3: Vectorized filtering (NumPy)
            # Filter out non-target classes BEFORE the loop to reduce Python iteration overhead
            if len(target_indices) > 0:
                mask = np.isin(pred_labels, target_indices)
                pred_boxes = pred_boxes[mask]
                pred_labels = pred_labels[mask]
                pred_scores = pred_scores[mask]

            for j in range(len(pred_boxes)):
                box = pred_boxes[j]
                label = int(pred_labels[j])
                score = float(pred_scores[j])

                # Extract center position and z_length for text offset
                x_center = float(box[0])
                y_center = float(box[1])
                z_center = float(box[2])
                z_length = float(box[5])

                if ROI_X_MIN < x_center < ROI_X_MAX and ROI_Y_MIN < y_center < ROI_Y_MAX:
                    roi_detected_count += 1

                class_name = class_names[label - 1] # Adjust class label index

                color = color_map.get(class_name, default_color)
                # self.get_logger().info(f"✅ Object {j+1},  Class: {class_name},  Score: {score:.2f},  Position: ({x_center:.2f}, {y_center:.2f}, {z_center:.2f})")


                # 1. Create Center Point marker (SPHERE)
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = current_time
                marker.header.frame_id = "velodyne"
                marker.ns = "detected_center"
                marker.id = i * 1000 + j
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x_center
                marker.pose.position.y = y_center
                # marker.pose.position.z = z_center
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0 # No rotation needed for a sphere

                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3

                marker.color.a = 1.0
                marker.color.r = float(color[0])
                marker.color.g = float(color[1])
                marker.color.b = float(color[2])
                marker.lifetime = Duration(sec=0, nanosec=200000000)
                center_markers.markers.append(marker)


                # 2. # Create text marker
                text = Marker()
                text.header = Header(stamp=current_time, frame_id="velodyne")
                text.ns     = "detected_class"
                text.id     = i * 1000 + j
                text.type   = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x_center
                text.pose.position.y = y_center
                text.pose.position.z = z_center + z_length / 2 + 0.2
                text.text    = f"{class_name}: {score:.2f}"
                text.scale.z = 0.2
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.lifetime = Duration(sec=0, nanosec=200000000)
                text_markers.markers.append(text)

        # publish two topics
        pub_detected_centers.publish(center_markers)
        pub_detected_class.publish(text_markers)

        # Publish number of detected objects in ROI as Int8 with saturation.
        self.num_detected_msg.data = min(roi_detected_count, 127)
        self.pub_num_detected.publish(self.num_detected_msg)

        # Publish number of detected objects inside ROI as Int8 with saturation.
        self.num_detected_roi_msg.data = min(roi_detected_count, 127)
        self.pub_num_detected_roi.publish(self.num_detected_roi_msg)
        self.latest_roi_detected_count = roi_detected_count

        return {
            "center_marker_count": len(center_markers.markers),
            "roi_detected_count": roi_detected_count,
        }

    def log_timing(
        self,
        frame_idx,
        preprocess_ms,
        voxel_ms,
        infer_ms,
        publish_ms,
        total_ms,
        input_period_ms,
        output_period_ms,
        center_marker_count,
        roi_detected_count,
    ):
        stage_pairs = [
            ("pre", preprocess_ms),
            ("voxel", voxel_ms),
            ("infer", infer_ms),
            ("pub", publish_ms),
        ]
        slowest_stage, slowest_stage_ms = max(stage_pairs, key=lambda x: x[1])
        output_hz = 0.0 if output_period_ms is None or output_period_ms <= 0.0 else (1000.0 / output_period_ms)
        target_frame_ms = 1000.0 / self.expected_output_hz

        msg = (
            f"⏱️ frame={frame_idx:05d} "
            f"pre={preprocess_ms:7.2f}ms voxel={voxel_ms:7.2f}ms infer={infer_ms:7.2f}ms pub={publish_ms:7.2f}ms "
            f"total={total_ms:7.2f}ms slowest={slowest_stage}:{slowest_stage_ms:7.2f}ms "
            f"in_dt={'NA' if input_period_ms is None else f'{input_period_ms:7.2f}ms'} "
            f"out_dt={'NA' if output_period_ms is None else f'{output_period_ms:7.2f}ms'} "
            f"out_hz={'NA' if output_hz == 0.0 else f'{output_hz:5.2f}'} "
            f"center={center_marker_count:3d} roi={roi_detected_count:3d} dropped_in={self.dropped_input_frames}"
        )

        is_slow_frame = total_ms > self.slow_frame_threshold_ms
        is_below_target = total_ms > target_frame_ms

        if is_slow_frame:
            self.get_logger().warn(msg)
        elif (frame_idx % self.timing_log_every_n) == 0:
            if is_below_target:
                self.get_logger().warn(msg)
            else:
                self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    center_object_detect_node = CenterObjectDetect()
    try:
        rclpy.spin(center_object_detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        center_object_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
