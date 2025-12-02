from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid as RosOccupancyGrid
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .gmapping import GMapping
from .motion_model import DifferentialDriveMotionModel
from .sensor_model import LikelihoodFieldSensorModel


class GMappingNode(Node):
    """ROS 2 node that wraps the pure-Python GMapping class."""

    def __init__(self) -> None:
        super().__init__("python_gmapping")
        self._declare_default_parameter("use_sim_time", True)
        self.scan_topic = self._declare_default_parameter("scan_topic", "/preprocessing_layer/scan")
        self.frame_base = self._declare_default_parameter("base_frame", "base_link")
        self.frame_odom = self._declare_default_parameter("odom_frame", "odom")
        self.frame_map = self._declare_default_parameter("map_frame", "map")
        self.tf_timeout = float(self._declare_default_parameter("tf_timeout", 0.5))
        self.map_update_interval = float(self._declare_default_parameter("map_update_interval", 2.0))
        self.queue_size = 5

        num_particles = int(self._declare_default_parameter("num_particles", 30))
        resample_threshold = float(self._declare_default_parameter("resample_threshold", 0.5))
        map_resolution = float(self._declare_default_parameter("resolution", 0.05))
        max_range = float(self._declare_default_parameter("max_range", 5.0))
        sigma_hit = float(self._declare_default_parameter("sigma_hit", 0.2))

        srr = float(self._declare_default_parameter("srr", 0.1))
        srt = float(self._declare_default_parameter("srt", 0.2))
        str_ = float(self._declare_default_parameter("str", 0.1))
        stt = float(self._declare_default_parameter("stt", 0.2))

        motion_model = DifferentialDriveMotionModel(alpha1=srr, alpha2=srt, alpha3=str_, alpha4=stt)
        sensor_model = LikelihoodFieldSensorModel(sigma_hit=sigma_hit, max_range=max_range)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.slam = GMapping(
            num_particles=num_particles,
            resample_threshold=resample_threshold,
            map_resolution=map_resolution,
            motion_model=motion_model,
            sensor_model=sensor_model,
            max_range=max_range,
        )
        self.map_pub = self.create_publisher(RosOccupancyGrid, "map", self.queue_size)
        self.map_metadata_pub = self.create_publisher(MapMetaData, "map_metadata", 1)
        self.particles_pub = self.create_publisher(PoseArray, "gmapping_particles", 1)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, self.queue_size)
        self._last_map_publish: Time | None = None
        self._warned_tf_failure = False

    def _scan_callback(self, scan: LaserScan) -> None:
        if self.slam.map_angles is None:
            self.slam.initialize_angles(scan.angle_min, scan.angle_increment, len(scan.ranges))

        try:
            transform_time = Time.from_msg(scan.header.stamp)
            tf_msg = self._lookup_transform_with_fallback(transform_time, scan.header.frame_id)
        except Exception as timed_exc:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {timed_exc}")
            return
        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        yaw = self._yaw_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        odom_pose = (trans.x, trans.y, yaw)
        
        self.slam.predict(odom_pose)
        self.slam.update(scan.ranges)
        self._publish_tf(odom_pose, scan.header.stamp)
        self._publish_particles(scan)
        self._maybe_publish_map(scan)
    def _publish_tf(self, odom_pose: Tuple[float, float, float], stamp) -> None:
        best = self.slam.best_particle().pose
        map_x, map_y, map_yaw = best
        odom_x, odom_y, odom_yaw = odom_pose

        map_to_odom_yaw = map_yaw - odom_yaw
        cos_mo = math.cos(map_to_odom_yaw)
        sin_mo = math.sin(map_to_odom_yaw)
        rot_odom = (
            cos_mo * odom_x - sin_mo * odom_y,
            sin_mo * odom_x + cos_mo * odom_y,
        )
        trans_x = map_x - rot_odom[0]
        trans_y = map_y - rot_odom[1]

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_map
        transform.child_frame_id = self.frame_odom
        transform.transform.translation.x = trans_x
        transform.transform.translation.y = trans_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = geometry_quaternion(map_to_odom_yaw)

        self.tf_broadcaster.sendTransform(transform)

    
    def _maybe_publish_map(self, scan: LaserScan) -> None:
        stamp_time = Time.from_msg(scan.header.stamp)
        if self._last_map_publish is None:
            elapsed = None
        else:
            elapsed = stamp_time - self._last_map_publish

        if elapsed is None or elapsed > Duration(seconds=self.map_update_interval):
            best = self.slam.best_particle()
            occ_prob = (best.grid.occupancy_probability() * 100.0).astype(np.int8)
            metadata = MapMetaData()
            metadata.map_load_time = scan.header.stamp
            metadata.resolution = best.grid.resolution
            metadata.width = best.grid.size_x
            metadata.height = best.grid.size_y
            metadata.origin.position.x = best.grid.origin[0]
            metadata.origin.position.y = best.grid.origin[1]
            metadata.origin.orientation.w = 1.0

            msg = RosOccupancyGrid()
            msg.header.stamp = scan.header.stamp
            msg.header.frame_id = self.frame_map
            msg.info = metadata
            msg.data = occ_prob.flatten().tolist()

            self.map_pub.publish(msg)
            self.map_metadata_pub.publish(metadata)
            self._last_map_publish = stamp_time

    def _publish_particles(self, scan: LaserScan) -> None:
        poses = PoseArray()
        poses.header = scan.header
        poses.header.frame_id = self.frame_map
        for p in self.slam.particles:
            pose_msg = geometry_pose(p.pose)
            poses.poses.append(pose_msg)
        self.particles_pub.publish(poses)

    @staticmethod
    def _yaw_from_quaternion(quat: List[float]) -> float:
        x, y, z, w = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _declare_default_parameter(self, name: str, default_value):
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            pass
        return self.get_parameter(name).value

    def _lookup_transform_with_fallback(self, transform_time: Time, laser_frame: str) -> TransformStamped:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.frame_odom,
                laser_frame,
                transform_time,
                timeout=Duration(seconds=self.tf_timeout),
            )
            self._warned_tf_failure = False
            return tf_msg
        except Exception as timed_exc:  # noqa: BLE001
            # If TF is delayed beyond the scan timestamp, fall back to the latest transform
            # instead of dropping the scan outright. This handles cases where TF publishers
            # are running but the timestamps lag (e.g., bag playback or slow sim clocks).
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.frame_odom,
                    laser_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
                self._warned_tf_failure = False
                return tf_msg
            except Exception as fallback_exc:  # noqa: BLE001
                if not self._warned_tf_failure:
                    self.get_logger().warn(
                        "TF missing or delayed from %s to %s (scan time %.3f). Check odometry "
                        "publisher, robot_state_publisher, and /clock synchronization. "
                        "Last errors: %s | %s"
                        % (self.frame_odom, laser_frame, transform_time.nanoseconds * 1e-9, timed_exc, fallback_exc)
                    )
                    self._warned_tf_failure = True
                raise


def geometry_pose(pose) -> Pose:
    x, y, theta = pose
    q = Quaternion()
    q.z = math.sin(theta / 2)
    q.w = math.cos(theta / 2)
    msg = Pose()
    msg.position.x = x
    msg.position.y = y
    msg.orientation = q
    return msg

def geometry_quaternion(theta: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(theta / 2)
    q.w = math.cos(theta / 2)
    return q

def main() -> None:
    rclpy.init()
    node = GMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
