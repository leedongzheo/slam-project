from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('preprocessing_layer')
    config = os.path.join(package_share, 'config', 'preprocessing.yaml')

    return LaunchDescription([
        Node(
            package='preprocessing_layer',
            executable='preprocessing_layer_node',
            name='preprocessing_layer_node',
            output='screen',
            parameters=[config],
        )
    ])
