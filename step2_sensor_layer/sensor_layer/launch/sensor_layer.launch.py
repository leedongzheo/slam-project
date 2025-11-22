from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('sensor_layer')
    config = os.path.join(package_share, 'config', 'topics.yaml')

    return LaunchDescription([
        Node(
            package='sensor_layer',
            executable='sensor_layer_node',
            name='sensor_layer_node',
            output='screen',
            parameters=[config],
        )
    ])
