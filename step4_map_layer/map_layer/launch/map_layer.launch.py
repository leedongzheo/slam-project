from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("map_layer"), "config", "gmapping_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML file with parameters for the Python GMapping node.",
            ),
            Node(
                package="map_layer",
                executable="gmapping_node",
                name="python_gmapping",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
