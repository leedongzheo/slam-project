# Map Layer – Python GMapping for TurtleBot3

This step provides a **from-scratch Python reimplementation of GMapping** that lives in the [`map_layer`](./map_layer/) folder. It replaces the dependency on the C++ `openslam_gmapping` tree while keeping the same workflow used in the previous steps of the project: TurtleBot3 simulation in Gazebo (step 1), RGB-D capture and preprocessing (steps 2–3), and map visualization in RViz.

- The `map_layer` package contains modular building blocks (motion model, sensor model, occupancy grid, particle filter, resampler) plus a ROS 2 node that publishes `/map` and `/gmapping_particles` from `/scan` and TF.
- See [`map_layer/README.md`](./map_layer/README.md) for detailed build instructions, algorithm explanation, and a demo recipe on TurtleBot3 with ROS 2 Jazzy and Gazebo Harmonic.
  
## Pipeline Integration

This layer sits directly downstream of the Preprocessing Layer:

1.  **Step 1 & 2 (Sensor):** Raw RGB-D data is acquired.
2.  **Step 3 (Preprocessing):** Depth data is converted to a 2D `LaserScan` on topic `/preprocessing_layer/scan`.
3.  **Step 4 (Map Layer - This Step):** Consumes the clean `LaserScan` to generate `/map` and correct odometry drift.

-----

## Architecture & Node Design

The core executable is `gmapping_node.py`. Below is the detailed design of inputs, outputs, and configuration parameters tailored to work with the previous steps.

### 1\. Subscribers (Inputs)

| Topic Name | Message Type | Source | Description |
| :--- | :--- | :--- | :--- |
| **`/preprocessing_layer/scan`** | `sensor_msgs/LaserScan` | **Step 3 (Preprocessing)** | The 2D scan slice generated from the depth camera. **Critical: This is not the raw `/scan`.** |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | Robot State Publisher | Transforms for `odom` $\to$ `base_link` $\to$ `base_scan`. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped`| RViz2 | (Optional) Manual pose initialization. |

### 2\. Publishers (Outputs)

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| **`/map`** | `nav_msgs/OccupancyGrid` | The global 2D grid map (0=Free, 100=Occupied, -1=Unknown). |
| `/map_metadata` | `nav_msgs/MapMetaData` | Map info (resolution, width, height, origin). |
| `/gmapping_particles`| `geometry_msgs/PoseArray` | Visualization of the particle cloud (hypothesis distribution) in RViz. |
| `/tf` | `tf2_msgs/TFMessage` | **Transform:** `map` $\to$ `odom`. Corrects odometry drift. |

-----

## Configuration & Parameters

The node behavior is controlled via `config/gmapping_params.yaml`.

### A. System & TF Parameters

These parameters ensure the SLAM node connects to the correct topics from Step 3.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `scan_topic` | **`/preprocessing_layer/scan`** | **Must match the output topic of Step 3.** |
| `base_frame` | `base_link` | Robot's footprint frame. |
| `odom_frame` | `odom` | Odometry frame provided by Gazebo/Robot. |
| `map_frame` | `map` | The global fixed frame. |
| `map_update_interval`| `2.0` | Seconds between map publications (throttle to save CPU). |

### B. Particle Filter & Motion Model

Controls the "brain" of the SLAM algorithm.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `num_particles` | `30` | Number of particles. Higher = better map but more CPU usage. |
| `resample_threshold`| `0.5` | Threshold ($N_{eff}/N$) to trigger resampling. |
| `srr`, `srt` | `0.1`, `0.2` | Odometry error parameters (Rotation noise). |
| `str`, `stt` | `0.1`, `0.2` | Odometry error parameters (Translation noise). |

### C. Sensor Model & Map

Controls how the laser data is interpreted and drawn.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `max_range` | `5.0` | Max laser range to trust for mapping (meters). |
| `sigma_hit` | `0.2` | Gaussian standard deviation for likelihood field (sensor confidence). |
| `resolution` | `0.05` | Map resolution (meters/pixel). |

-----

## Project Structure

```text
map_layer/
├── config/
│   └── gmapping_params.yaml      # Configuration file
├── launch/
│   └── map_layer.launch.py       # Launches node with params
├── map_layer/
│   ├── __init__.py
│   ├── gmapping_node.py          # ROS 2 Node Interface
│   ├── gmapping.py               # Core Particle Filter Logic
│   ├── motion_model.py           # Odometry Probabilistic Model
│   ├── sensor_model.py           # Likelihood Field Model
│   ├── resampler.py              # Low-variance Resampling
│   └── occupancy_grid.py         # Grid Map Data Structure
├── package.xml
└── setup.py
```

-----

## Running the Pipeline

To run the full SLAM system, you must launch the layers in order.

### 1\. Prerequisites (Launch Steps 1-3)

Ensure the simulation and preprocessing layers are running:

```bash
# Terminal 1: Gazebo World
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# Terminal 2:
ros2 launch sensor_layer sensor_layer.launch.py

# Terminal 3: Preprocessing Layer (Generates the scan)
ros2 launch preprocessing_layer preprocessing_layer.launch.py
```

*Verify that `/preprocessing_layer/scan` is active before proceeding.*

### 2\. Launch Map Layer

Start the Python GMapping node:

```bash
# Method 1: Default Launch
ros2 launch map_layer map_layer.launch.py

# Method 2: Manual Run with params
ros2 run map_layer gmapping_node --ros-args --params-file config/gmapping_params.yaml
```

### 3\. Visualization (RViz2)

Open RViz2 and configure:

1.  **Global Options \> Fixed Frame:** Set to `map`.
2.  **Add \> Map:** Topic `/map`.
3.  **Add \> PoseArray:** Topic `/gmapping_particles` (to see the algorithm "thinking").
4.  **Add \> LaserScan:** Topic `/preprocessing_layer/scan`.

### 4\. Teleoperation

Drive the robot to build the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

-----

## Troubleshooting

**1. Map is not appearing?**

  * Check if `/preprocessing_layer/scan` is publishing data: `ros2 topic hz /preprocessing_layer/scan`.
  * Check TF tree: `ros2 run tf2_tools view_frames`. You should see a continuous chain: `map` $\to$ `odom` $\to$ `base_link`.

**2. Map is drifting or rotating wildly?**

  * The odometry noise parameters (`srr`, `str`...) in `gmapping_params.yaml` might be too high. Try reducing them.
  * Ensure the `max_range` matches the simulated laser range.

**3. "Transform Timeout" errors?**

  * The pure Python implementation might be running too slowly. Try reducing `num_particles` to 15 or 20.
