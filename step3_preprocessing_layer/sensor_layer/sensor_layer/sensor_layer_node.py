from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


@dataclass
class TopicStatistics:
    """Runtime statistics for a single subscribed topic."""

    name: str
    last_stamp: Optional[Time] = None
    message_count: int = 0

    def update(self, stamp: Optional[Time]) -> None:
        self.message_count += 1
        self.last_stamp = stamp

    def format_status(self, now: Time) -> str:
        if self.message_count == 0:
            return f"{self.name}: waiting for data"
        if self.last_stamp is None:
            return f"{self.name}: {self.message_count} msgs received"
        delay = (now - self.last_stamp).nanoseconds / 1e9
        return f"{self.name}: {self.message_count} msgs, age {delay:.2f}s"


class SensorLayerNode(Node):
    """Node that subscribes to Gazebo sensor topics and republishes them."""

    def __init__(self) -> None:
        super().__init__('sensor_layer_node')

        self.declare_parameter('color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('output_namespace', '/sensor_layer')
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('status_publish_period', 2.0)

        self._output_namespace = self.get_parameter('output_namespace').value.rstrip('/')
        self._stats: Dict[str, TopicStatistics] = {}

        image_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._create_pipeline(
            'color_image',
            Image,
            self.get_parameter('color_image_topic').value,
            f'{self._output_namespace}/color/image_raw',
            image_qos,
        )
        self._create_pipeline(
            'depth_image',
            Image,
            self.get_parameter('depth_image_topic').value,
            f'{self._output_namespace}/depth/image_raw',
            image_qos,
        )
        self._create_pipeline(
            'color_info',
            CameraInfo,
            self.get_parameter('color_info_topic').value,
            f'{self._output_namespace}/color/camera_info',
            reliable_qos,
        )
        self._create_pipeline(
            'depth_info',
            CameraInfo,
            self.get_parameter('depth_info_topic').value,
            f'{self._output_namespace}/depth/camera_info',
            reliable_qos,
        )
        self._create_pipeline(
            'odom',
            Odometry,
            self.get_parameter('odom_topic').value,
            f'{self._output_namespace}/odom',
            reliable_qos,
        )

        if self.get_parameter('enable_imu').value:
            self._create_pipeline(
                'imu',
                Imu,
                self.get_parameter('imu_topic').value,
                f'{self._output_namespace}/imu',
                reliable_qos,
            )

        self._status_pub = self.create_publisher(String, f'{self._output_namespace}/status', 10)
        period = float(self.get_parameter('status_publish_period').value)
        self.create_timer(period, self._publish_status)
        self.get_logger().info('Sensor layer node initialized')

    def _create_pipeline(
        self,
        name: str,
        msg_type,
        input_topic: str,
        output_topic: str,
        qos_profile: QoSProfile,
    ) -> None:
        self._stats[name] = TopicStatistics(name=name)
        publisher = self.create_publisher(msg_type, output_topic, qos_profile)

        def forward(msg) -> None:
            publisher.publish(msg)
            stamp = Time.from_msg(msg.header.stamp) if hasattr(msg, 'header') else None
            self._stats[name].update(stamp)

        self.create_subscription(msg_type, input_topic, forward, qos_profile)
        self.get_logger().info(
            f'Relaying {name} from {input_topic} -> {output_topic} with QoS depth {qos_profile.depth}'
        )

    def _publish_status(self) -> None:
        now = self.get_clock().now()
        status_lines = [stat.format_status(now) for stat in self._stats.values()]
        msg = String()
        msg.data = '\n'.join(status_lines)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorLayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
