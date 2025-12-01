from __future__ import annotations

import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .gmapping import GMapping


class GMappingNode(Node):
    """ROS 2 node that wraps the pure-Python GMapping class."""

    def __init__(self) -> None:
        super().__init__("python_gmapping")
        self.frame_odom = "odom"
        self.frame_base = "base_footprint"
        self.frame_map = "map"
        self.queue_size = 5

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.slam = GMapping()
        self.map_pub = self.create_publisher(RosOccupancyGrid, "map", self.queue_size)
        self.particles_pub = self.create_publisher(PoseArray, "gmapping_particles", 1)
        self.create_subscription(LaserScan, "scan", self._scan_callback, self.queue_size)

    def _scan_callback(self, scan: LaserScan) -> None:
        if self.slam.map_angles is None:
            self.slam.initialize_angles(scan.angle_min, scan.angle_increment, len(scan.ranges))

        try:
            transform_time = Time.from_msg(scan.header.stamp)
            tf_msg = self.tf_buffer.lookup_transform(
                self.frame_odom,
                scan.header.frame_id,
                transform_time,
                timeout=Duration(seconds=0.5),
            )
            trans = tf_msg.transform.translation
            rot = tf_msg.transform.rotation
            yaw = self._yaw_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            odom_pose = (trans.x, trans.y, yaw)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        self.slam.predict(odom_pose)
        self.slam.update(scan.ranges)
        self._publish_map(scan)
        self._publish_particles(scan)

    def _publish_map(self, scan: LaserScan) -> None:
        best = self.slam.best_particle()
        occ_prob = (best.grid.occupancy_probability() * 100.0).astype(np.int8)
        msg = RosOccupancyGrid()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.frame_map
        msg.info.resolution = best.grid.resolution
        msg.info.width = best.grid.size_x
        msg.info.height = best.grid.size_y
        msg.info.origin.position.x = best.grid.origin[0]
        msg.info.origin.position.y = best.grid.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = occ_prob.flatten().tolist()
        self.map_pub.publish(msg)

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


def main() -> None:
    rclpy.init()
    node = GMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
