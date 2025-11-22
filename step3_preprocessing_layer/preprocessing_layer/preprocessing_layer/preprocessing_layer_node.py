#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
import tf2_ros


@dataclass
class StageStatistics:
    """Runtime counters for each processing stage."""

    name: str
    count: int = 0
    last_stamp: Optional[Time] = None

    def tick(self, stamp: Optional[Time]) -> None:
        self.count += 1
        self.last_stamp = stamp

    def format(self, now: Time) -> str:
        if self.count == 0:
            return f"{self.name}: waiting for data"
        if self.last_stamp is None:
            return f"{self.name}: {self.count} msgs"
        delay = (now - self.last_stamp).nanoseconds / 1e9
        return f"{self.name}: {self.count} msgs, age {delay:.2f}s"


class PreprocessingLayerNode(Node):
    """Transforms depth images into filtered point clouds and LaserScan."""

    def __init__(self) -> None:
        super().__init__('preprocessing_layer_node')

        # Parameters
        self.declare_parameter('depth_image_topic', '/sensor_layer/depth/image_raw')
        self.declare_parameter('depth_info_topic', '/sensor_layer/depth/camera_info')
        self.declare_parameter('output_namespace', '/preprocessing_layer')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('voxel_size', 0.04)
        self.declare_parameter('min_z', 0.15)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('outlier_min_points', 3)
        self.declare_parameter('laser_min_height', -0.05)
        self.declare_parameter('laser_max_height', 0.05)
        self.declare_parameter('laser_angle_min', -3.14)
        self.declare_parameter('laser_angle_max', 3.14)
        self.declare_parameter('laser_angle_increment', 0.01)
        self.declare_parameter('laser_range_min', 0.1)
        self.declare_parameter('laser_range_max', 10.0)
        self.declare_parameter('status_publish_period', 2.0)

        self._namespace = self.get_parameter('output_namespace').value.rstrip('/')
        self._base_frame = self.get_parameter('base_frame').value
        self._voxel_size = float(self.get_parameter('voxel_size').value)
        self._min_z = float(self.get_parameter('min_z').value)
        self._max_z = float(self.get_parameter('max_z').value)
        self._outlier_min_points = int(self.get_parameter('outlier_min_points').value)
        self._laser_params = {
            'min_height': float(self.get_parameter('laser_min_height').value),
            'max_height': float(self.get_parameter('laser_max_height').value),
            'angle_min': float(self.get_parameter('laser_angle_min').value),
            'angle_max': float(self.get_parameter('laser_angle_max').value),
            'angle_increment': float(self.get_parameter('laser_angle_increment').value),
            'range_min': float(self.get_parameter('laser_range_min').value),
            'range_max': float(self.get_parameter('laser_range_max').value),
        }

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST)
        self._stats: Dict[str, StageStatistics] = {}
        self._camera_info: Optional[CameraInfo] = None

        depth_topic = self.get_parameter('depth_image_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value

        self.create_subscription(CameraInfo, depth_info_topic, self._camera_info_callback, qos)
        self.create_subscription(Image, depth_topic, self._depth_callback, qos)

        self._raw_cloud_pub = self.create_publisher(PointCloud2, f'{self._namespace}/camera/depth/points', 10)
        self._filtered_cloud_pub = self.create_publisher(PointCloud2, f'{self._namespace}/camera/points_filtered', 10)
        self._base_cloud_pub = self.create_publisher(PointCloud2, f'{self._namespace}/points_in_base_link', 10)
        self._scan_pub = self.create_publisher(LaserScan, f'{self._namespace}/scan', 10)
        self._status_pub = self.create_publisher(String, f'{self._namespace}/status', 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        period = float(self.get_parameter('status_publish_period').value)
        self.create_timer(period, self._publish_status)

        for name in ['depth', 'raw_cloud', 'filtered_cloud', 'base_cloud', 'scan']:
            self._stats[name] = StageStatistics(name=name)

        self._log_no_camera_info = True
        self._tf_warned = False
        self.get_logger().info('Preprocessing layer node initialized')

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _depth_callback(self, msg: Image) -> None:
        self._stats['depth'].tick(Time.from_msg(msg.header.stamp))
        if self._camera_info is None:
            if self._log_no_camera_info:
                self.get_logger().warn('Waiting for camera info before processing depth images')
                self._log_no_camera_info = False
            return

        points = self._depth_to_points(msg, self._camera_info)
        if points.size == 0:
            return

        raw_cloud = self._points_to_cloud(points, msg.header.frame_id, msg.header.stamp)
        self._raw_cloud_pub.publish(raw_cloud)
        self._stats['raw_cloud'].tick(Time.from_msg(msg.header.stamp))

        filtered = self._filter_point_cloud(points)
        filtered_cloud = self._points_to_cloud(filtered, msg.header.frame_id, msg.header.stamp)
        self._filtered_cloud_pub.publish(filtered_cloud)
        self._stats['filtered_cloud'].tick(Time.from_msg(msg.header.stamp))

        base_points, base_cloud = self._transform_to_base(filtered, msg.header)
        if base_cloud:
            self._base_cloud_pub.publish(base_cloud)
            self._stats['base_cloud'].tick(Time.from_msg(msg.header.stamp))

        if base_points is not None:
            scan = self._points_to_laserscan(base_points, msg.header)
            self._scan_pub.publish(scan)
            self._stats['scan'].tick(Time.from_msg(msg.header.stamp))

    def _depth_to_points(self, depth_msg: Image, info: CameraInfo) -> np.ndarray:
        try:
            depth_array = self._depth_to_array(depth_msg)
        except ValueError as exc:
            self.get_logger().warn(f'Unsupported depth encoding: {exc}')
            return np.empty((0, 3), dtype=np.float32)

        height, width = depth_msg.height, depth_msg.width
        if depth_array.size != height * width:
            self.get_logger().warn('Depth array size does not match image dimensions')
            return np.empty((0, 3), dtype=np.float32)

        depth_array = depth_array.reshape((height, width))
        v_coords, u_coords = np.indices((height, width))
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        z = depth_array.astype(np.float32)
        x = (u_coords - cx) * z / fx
        y = (v_coords - cy) * z / fy

        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1)
        valid &= points[:, 2] > 0.0
        points = points[valid]

        return points

    def _depth_to_array(self, depth_msg: Image) -> np.ndarray:
        if depth_msg.encoding in ('32FC1', '32FC'):
            dtype = np.float32
        elif depth_msg.encoding == '16UC1':
            dtype = np.uint16
        else:
            raise ValueError(depth_msg.encoding)

        depth_array = np.frombuffer(depth_msg.data, dtype=dtype)
        if dtype == np.uint16:
            depth_array = depth_array.astype(np.float32) / 1000.0
        return depth_array

    def _filter_point_cloud(self, points: np.ndarray) -> np.ndarray:
        within_z = (points[:, 2] >= self._min_z) & (points[:, 2] <= self._max_z)
        filtered = points[within_z]

        if filtered.size == 0:
            return filtered

        if self._voxel_size > 0:
            voxel_indices = np.floor(filtered / self._voxel_size).astype(np.int32)
            _, unique_indices, counts = np.unique(
                voxel_indices, axis=0, return_index=True, return_counts=True
            )
            dense_mask = counts >= self._outlier_min_points
            kept_indices = unique_indices[dense_mask]
            filtered = filtered[kept_indices]

        return filtered

    def _transform_to_base(self, points: np.ndarray, header) -> Tuple[Optional[np.ndarray], Optional[PointCloud2]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, header.frame_id, Time.from_msg(header.stamp), timeout=Duration(seconds=0.1)
            )
        except Exception:
            if not self._tf_warned:
                self.get_logger().warn('TF transform to base_link unavailable, skipping projection')
                self._tf_warned = True
            return None, None

        transform_matrix = self._transform_to_matrix(transform)
        homogeneous = np.hstack((points, np.ones((points.shape[0], 1), dtype=np.float32)))
        transformed = (transform_matrix @ homogeneous.T).T[:, :3]

        cloud_msg = self._points_to_cloud(transformed, self._base_frame, header.stamp)
        return transformed, cloud_msg

    def _transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        tx, ty, tz = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z

        # Rotation matrix from quaternion
        r00 = 1 - 2 * (y * y + z * z)
        r01 = 2 * (x * y - z * w)
        r02 = 2 * (x * z + y * w)
        r10 = 2 * (x * y + z * w)
        r11 = 1 - 2 * (x * x + z * z)
        r12 = 2 * (y * z - x * w)
        r20 = 2 * (x * z - y * w)
        r21 = 2 * (y * z + x * w)
        r22 = 1 - 2 * (x * x + y * y)

        matrix = np.array(
            [
                [r00, r01, r02, tx],
                [r10, r11, r12, ty],
                [r20, r21, r22, tz],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        return matrix

    def _points_to_cloud(self, points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
        time_stamp = self.get_clock().now().to_msg() if stamp is None else stamp
        header = Header(frame_id=frame_id, stamp=time_stamp)
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def _points_to_laserscan(self, points: np.ndarray, header) -> LaserScan:
        params = self._laser_params
        num_readings = int((params['angle_max'] - params['angle_min']) / params['angle_increment']) + 1
        ranges = [math.inf] * num_readings

        for x, y, z in points:
            if z < params['min_height'] or z > params['max_height']:
                continue
            r = math.hypot(x, y)
            if r < params['range_min'] or r > params['range_max']:
                continue
            angle = math.atan2(y, x)
            if angle < params['angle_min'] or angle > params['angle_max']:
                continue
            idx = int((angle - params['angle_min']) / params['angle_increment'])
            if 0 <= idx < num_readings and r < ranges[idx]:
                ranges[idx] = r

        scan = LaserScan()
        scan.header.stamp = header.stamp
        scan.header.frame_id = self._base_frame
        scan.angle_min = params['angle_min']
        scan.angle_max = params['angle_max']
        scan.angle_increment = params['angle_increment']
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = params['range_min']
        scan.range_max = params['range_max']
        scan.ranges = ranges
        return scan

    def _publish_status(self) -> None:
        now = self.get_clock().now()
        lines = [stat.format(now) for stat in self._stats.values()]
        msg = String()
        msg.data = '\n'.join(lines)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreprocessingLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
