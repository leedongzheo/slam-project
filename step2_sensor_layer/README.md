# Sensor Layer Integration

The Sensor Layer is responsible for collecting raw sensor data from the
Gazebo simulation and publishing it in a format that can be consumed by
the ROS 2 system. The primary focus here is on capturing the RGB-D
streams (color and depth images), camera calibration data, and odometry
information, with optional IMU data if available.

## Overview

In this section, we integrate the RGB-D camera sensor streams, including
the color and depth images, camera intrinsic information, as well as
odometry data from Gazebo, into the ROS 2 ecosystem. The data streams
provided by Gazebo are critical for tasks like object detection, 3D
mapping, navigation, and localization.

## ROS 2 Topics for Sensor Data

When the simulation is running, the following ROS 2 topics will be
published from Gazebo, corresponding to the RGB-D camera and odometry
data:

  -----------------------------------------------------------------------------------------------------------
  Topic                         Message Type                   Description
  ----------------------------- ------------------------------ ----------------------------------------------
  `/camera/color/image_raw`     `sensor_msgs/msg/Image`        Raw RGB image stream from the camera (color
                                                               image).

  `/camera/depth/image_raw`     `sensor_msgs/msg/Image`        Raw depth image from the camera.

  `/camera/color/camera_info`   `sensor_msgs/msg/CameraInfo`   Camera intrinsics for the color stream.

  `/camera/depth/camera_info`   `sensor_msgs/msg/CameraInfo`   Camera intrinsics for the depth stream.

  `/odom`                       `nav_msgs/msg/Odometry`        Odometry data from the robot.

  `/imu` (optional)             `sensor_msgs/msg/Imu`          IMU data if available.
  -----------------------------------------------------------------------------------------------------------

## RGB-D Stream Processing

### Color Image (`/camera/color/image_raw`)

Provides raw RGB data for perception tasks including SLAM, object
recognition, and mapping.

### Depth Image (`/camera/depth/image_raw`)

Publishes depth data for 3D perception, obstacle avoidance, and mapping.

### Camera Info (`/camera/color/camera_info`, `/camera/depth/camera_info`)

Includes camera calibration parameters such as focal lengths and
distortion coefficients.

## Odometry Data (`/odom`)

Odometry includes:

-   Robot position (x, y, z)
-   Orientation (quaternion)
-   Linear and angular velocity

## IMU Data (Optional)

If provided, `/imu` includes:

-   Linear acceleration
-   Angular velocity
-   Orientation quaternion

## Sensor Data Handling in ROS 2

ROS 2 nodes subscribe to the sensor topics for:

-   3D mapping and localization\
-   Navigation and obstacle avoidance\
-   Sensor fusion

## Configurations for Sensor Stream Activation

### Gazebo Model

The `model.sdf` file defines the RGB-D camera sensor and its Gazebo
plugin.

### ROS 2 Bridge

The `turtlebot3_waffle_bridge.yaml` configures topic bridges for sensor
streams.

## Simulation Usage

### Visualization

    ros2 run rviz2 rviz2 -d turtlebot3_simulations/turtlebot3_gazebo/rviz/tb3_gazebo.rviz

### Inspect Raw Data

    ros2 topic echo /camera/depth/points
    ros2 topic echo /camera/color/image_raw

### Sensor Layer Node

    ros2 launch sensor_layer sensor_layer.launch.py

The launch file starts the `sensor_layer_node`, which subscribes to the
Gazebo-provided topics and republishes them under the `/sensor_layer`
namespace while broadcasting a status summary.

## ROS 2 Sensor Layer Package

The `sensor_layer` ROS 2 Python package encapsulates the logic of the
sensor layer described above. It:

-   Relays color and depth images, camera info, odometry, and optionally
    IMU data into a dedicated namespace.
-   Provides configurable topic names through
    `sensor_layer/config/topics.yaml`.
-   Publishes human-readable health summaries on
    `/sensor_layer/status`.

### Republished Topics

| Topic | Message | Source |
| :--- | :--- | :--- |
| `/sensor_layer/color/image_raw` | `sensor_msgs/msg/Image` | `/camera/color/image_raw` |
| `/sensor_layer/depth/image_raw` | `sensor_msgs/msg/Image` | `/camera/depth/image_raw` |
| `/sensor_layer/color/camera_info` | `sensor_msgs/msg/CameraInfo` | `/camera/color/camera_info` |
| `/sensor_layer/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `/camera/depth/camera_info` |
| `/sensor_layer/odom` | `nav_msgs/msg/Odometry` | `/odom` |
| `/sensor_layer/imu` | `sensor_msgs/msg/Imu` | `/imu` (optional) |

### Parameters

Parameter defaults live in `sensor_layer/config/topics.yaml` and can be
overridden via the launch file or command line.

## Conclusion

This sensor layer delivers critical data streams enabling advanced
robotics capabilities including 3D SLAM, navigation, and perception.
Integrating RGB-D sensors and odometry ensures the robot can operate
effectively in dynamic environments.
