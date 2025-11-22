# Perception Preprocessing Layer

The **Preprocessing Layer** is responsible for transforming the raw sensor streams provided by the Sensor Layer into **processed, structured, and task-ready perception data**. These transformations prepare the robot for downstream modules including SLAM, navigation, 3D mapping, obstacle avoidance, and laser-based planners.

This layer focuses on four primary perception operations:

1.  **Depth Image → PointCloud2** conversion.
2.  **Spatial filtering** and cleaning of the point cloud.
3.  **TF-based projection** of the cloud into robot coordinate frames.
4.  **Point cloud → LaserScan** conversion for compatibility with 2D SLAM systems.

---

## Overview

Once the Sensor Layer republishes synchronized RGB-D and odometry data into a clean namespace, the Preprocessing Layer consumes the raw depth image and performs perception preprocessing steps that are critical for high-quality SLAM and mapping. This includes generating a 3D point cloud, applying spatial filtering, transforming the cloud into the robot’s base frame, and optionally converting the 3D cloud into a 2D LaserScan.

---

## 1. Depth-to-PointCloud2 Conversion

**(`depth_image_proc`)**

The first step converts the raw depth image and camera intrinsics into a structured point cloud message.

| Input Topic | Output Topic | Node |
| :--- | :--- | :--- |
| `/sensor_layer/depth/image_raw` |  |
| `/sensor_layer/depth/camera_info` |→ `/camera/depth/points` | `depth_image_proc/point_cloud_xyz`| |  

* **Description:**
   * Performs per-pixel back-projection using camera intrinsics.
   * Produces a dense or sparse point cloud depending on depth data availability.
   * Output is a properly formed sensor_msgs/msg/PointCloud2 message.

> **Launch Example:**
> ```bash
> ros2 run depth_image_proc point_cloud_xyz
> ```
> (typically included in a launch file)
---

## 2. PointCloud Filtering

**(Voxel Grid, Passthrough, Outlier Removal)**

The raw point cloud is often noisy or too dense for real-time SLAM. A filtering pipeline refines the cloud and reduces computational overhead.

### Recommended Filtering Sequence

#### (A) Voxel Grid Downsampling
* **Purpose:** Reduces cloud resolution while preserving structure.
* **Typical Voxel Size:** $0.02 - 0.06\ \text{m}$.

#### (B) Passthrough Filter (Z-axis)
* **Purpose:** Removes ground noise and upper-ceiling points.
* **Example Z Limits:** $0.15\ \text{m} < z < 2.0\ \text{m}$.

#### (C) Statistical Outlier Removal
* **Purpose:** Removes sparse noise clusters.
* **Example Parameters:** `mean_k`: 20, `stddev_mul`: 1.0

### ROS 2 Output Topic

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/camera/points_filtered` | `sensor_msgs/msg/PointCloud2` | Cleaned & filtered point cloud |

### Node Implementation

Filtering is typically implemented using:
* `pcl_ros` filters
* Custom `rclpy` / `rclcpp` filtering node
* A composed filtering pipeline

---

## 3. TF-Based Projection of Point Cloud

**(`camera_link` → `base_link`)**

SLAM and navigation algorithms require all perception data to be expressed in the robot’s primary body frame (`base_link`). The Preprocessing Layer transforms the filtered cloud using `TF2`.

### ROS 2 TF Transform

| Source | Target | Output Topic |
| :--- | :--- | :--- |
| `/camera/points_filtered` | `camera_link` → `base_link` | `/points_in_base_link` |

* The transform uses data from `/tf`, `/tf_static`, and the camera mounting geometry defined in the SDF.

### Result

A point cloud aligned with the robot’s primary frame enables:
* Local obstacle avoidance
* Laser-based navigation
* 3D SLAM compatibility
* Consistency across sensor modalities

---

## 4. PointCloud → LaserScan

**(`pointcloud_to_laserscan`)**

For 2D SLAM algorithms such as `gmapping`, `slam_toolbox` (2D mode), or `hector_slam`, a planar `LaserScan` is required. This node converts the 3D transformed point cloud into a 2D scan.
### ROS 2 Node
> ```bash
> pointcloud_to_laserscan
> ```
### Topic Mapping

| Input | Output | Description |
| :--- | :--- | :--- |
| `/points_in_base_link` | `/scan` | 360-degree or 180-degree planar LaserScan |

### Typical Parameters

| Parameter | Value | Note |
| :--- | :--- | :--- |
| `min_height` | $-0.05$ | Min Z value to consider for the planar cut. |
| `max_height` | $+0.05$ | Max Z value to consider for the planar cut. |
| `angle_min` | $-3.14$ | Minimum horizontal angle ($-180^\circ$). |
| `angle_max` | $+3.14$ | Maximum horizontal angle ($+180^\circ$). |
| `range_min` | $0.1$ | Minimum distance for a valid ray. |
| `range_max` | $10.0$ | Maximum distance for a valid ray. |

### Output

The `/scan` topic is now fully compatible with any standard 2D SLAM and navigation system.

---
## Running the Node

The launch file launch/preprocessing_layer.launch.py loads the preprocessing.yaml configuration and starts the executable preprocessing_layer_node.
When you need to adjust parameters, simply edit the YAML file or add parameters=[...] directly inside the launch file.

### Quick Setup / Testing Guide

1. **Ensure the Sensor Layer is publishing**
Confirm that depth/image_raw and depth/camera_info are being published with the exact topic names specified in the YAML file.

2. **Start the preprocessing node**
>>   ```bash
>> ros2 launch preprocessing_layer preprocessing_layer.launch.py
>>   ```
>> This launches the node with the default configuration.
3. **Monitor output topics**
Check the following topics to verify that each stage is working correctly:

* /preprocessing_layer/camera/depth/points
* /camera/points_filtered***
* /points_in_base_link***
* /scan***
* .../status (progress/debug info)
### If You Want to Extend or Customize
* **Adjust voxel size, Z limits, or outlier thresholds** in the YAML file to balance smoothness vs. processing speed.

* **Add additional filtering steps**
(e.g., radius outlier removal) directly in the _filter_point_cloud section of the node implementation.
* **Tune LaserScan generation parameters**
such as angle limits or resolution in the YAML file to better match the SLAM algorithm you are using.
 ---
## Preprocessing Layer Summary

| Stage | Module | Input | Output |
| :--- | :--- | :--- | :--- |
| **1. Depth → PointCloud** | `depth_image_proc` | Depth Image + CameraInfo | `/camera/depth/points` |
| **2. Filtering** | `pcl_filters` / custom node | Raw PointCloud2 | `/camera/points_filtered` |
| **3. TF Projection** | `tf2_ros` | Filtered PointCloud2 | `/points_in_base_link` |
| **4. Cloud → LaserScan ** | `pointcloud_to_laserscan` | TF-aligned PointCloud | `/scan` |
---
## 💡 Conclusion

The Preprocessing Layer transforms raw depth and RGB-D data into optimized, structured, and robot-aligned perception information. It enables both 2D and 3D navigation stacks to operate reliably and forms the foundation for downstream SLAM, obstacle avoidance, and autonomous navigation.

This layer bridges the gap between raw sensor acquisition and high-level perception and mapping systems, ensuring data consistency, performance, and scalability.

---
## 🔍 Monitoring the output topics

Run these commands in another terminal while the preprocessing node is active:

1. **List and verify topics**
   ```bash
   ros2 topic list
   ros2 topic list | grep camera_info
   ros2 topic list | grep preprocessing_layer
   ros2 topic info /preprocessing_layer/camera/depth/points
   ...
   ```

2. **Inspect the point clouds**
   ```bash
   ros2 topic echo --qos-profile sensor_data /preprocessing_layer/camera/depth/points
   ros2 topic echo --qos-profile sensor_data /camera/points_filtered
   ros2 topic echo --qos-profile sensor_data /points_in_base_link
   Or:
   ros2 topic echo /preprocessing_layer/camera/depth/points
   ...
   ```

3. **Watch the LaserScan**
   ```bash
   ros2 topic echo --qos-profile sensor_data /scan
   Or:
   ros2 topic echo /scan
   ```

4. **Check pipeline status**
   ```bash
   ros2 topic echo /preprocessing_layer/status
   ```

For visual inspection, you can also open RViz2 and add displays for **PointCloud2** (using `/camera/depth/points`, `/camera/points_filtered`, `/points_in_base_link`) and **LaserScan** (using `/scan`).
