# Python GMapping (Map Layer)

This folder reimplements the core ideas of the **GMapping** algorithm in pure Python and wraps them in a ROS 2 Jazzy node for TurtleBot3 simulation on Ubuntu 24.04 with Gazebo Harmonic. The implementation does not reuse any code from `openslam_gmapping` but follows the same particle-filter SLAM principles: a motion model predicts particle poses from odometry, a likelihood-field sensor model scores each particle against the occupancy grid, and a low-variance resampler focuses computation on likely poses. The best particle's map is published as a `nav_msgs/OccupancyGrid` for RViz.

## Repository structure
- `map_layer/` – Python package directory containing the sources below.
  - `gmapping.py` – particle filter logic (prediction, weighting, resampling) and map updates.
  - `motion_model.py` – differential-drive odometry noise model that samples new poses.
  - `sensor_model.py` – likelihood-field beam model using occupancy probabilities.
  - `occupancy_grid.py` – log-odds grid with Bresenham ray-casting for scan insertion.
  - `resampler.py` – systematic low-variance resampling.
- `gmapping_node.py` – ROS 2 node that consumes `/preprocessing_layer/scan` and publishes `/map` plus particle poses.
- `package.xml`, `setup.py`, `setup.cfg`, `resource/map_layer` – ament_python metadata and install layout so `ros2 run` can find the executable under `lib/map_layer`.

## How the algorithm is built
1. **Map representation** – `OccupancyGrid` stores log-odds values and converts them to probabilities for publishing. Each laser beam is traced with Bresenham to mark free space; the end cell is marked as an obstacle when a hit occurs.
2. **Motion update** – `DifferentialDriveMotionModel.sample` draws a new pose for every particle from the odometry delta `(rot1, trans, rot2)` with configurable noise parameters `alpha1..alpha4`.
3. **Sensor update** – `LikelihoodFieldSensorModel.compute_weight` projects every beam into the grid to estimate how likely the measurement is, mixing a Gaussian hit term with a small uniform random component. The resulting product becomes the particle weight.
4. **Resampling** – when the effective sample size drops below `resample_threshold * N`, `LowVarianceResampler` duplicates likely particles while preserving diversity.
5. **Best map selection** – the highest-weight particle is treated as the current map and published to `/map` for visualization.

## Build & runtime requirements (ROS 2 Jazzy + Gazebo Harmonic)
- Ubuntu 24.04 with **ROS 2 Jazzy** desktop install (includes RViz and tf2), plus `gazebo-harmonic`.
- Python dependencies available in the overlay: `numpy` (installed via `rosdep` or `pip`).
- TurtleBot3 simulation assets from previous steps (`step1_gazebo`, `step2_sensor_layer`, `step3_preprocessing_layer`).

### Build with colcon
Place the `map_layer` folder inside your ROS 2 workspace (e.g., `~/turtlebot3_ws/src/slam-project/step4_map_layer/map_layer`) and build with `ament_python`:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select map_layer
source install/setup.bash
```

## Running the demo on TurtleBot3 (Gazebo Harmonic + RViz)
1. **Start Gazebo world (from step 1):**
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
2. **Bring up the RGB-D to laser pipeline (from steps 2 & 3):** ensure the depth-to-laser node publishes `/preprocessing_layer/scan` in the same namespace.
3. **Launch the Python GMapping node:**
   ```bash
   ros2 run map_layer gmapping_node
   ```
   The node enables `use_sim_time` by default so TF lookups align with Gazebo's `/clock`; override with `--ros-args -p use_sim_time:=false` when running on real hardware. If your `/preprocessing_layer/scan` timestamps are slightly ahead of or behind the TF buffer, the node will fall back to the latest available transform to avoid extrapolation warnings in either direction. The node also publishes the `map -> odom` transform so RViz can use `map` as the fixed frame without TF errors.
4. **Visualize the map in RViz2:**
   - Set `Fixed Frame` to `map`.
   - Add `Map` display subscribing to `/map`.
   - Optionally add `PoseArray` display on `/gmapping_particles` to inspect particle spread.
5. **Drive the robot:** use `teleop_twist_keyboard` or Nav2 waypoint tools to explore the environment. As scans arrive, the map will converge and appear in RViz2.
6. **Save the map (optional):**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/python_gmapping
   ```

## Tuning tips
- Increase `num_particles` in `gmapping.py` for larger environments at the cost of CPU.
- Adjust `map_size`, `map_resolution`, and `map_origin` to fit your world dimensions.
- Laser parameters such as `max_range`, `z_hit`, and `sigma_hit` inside `sensor_model.py` help balance hit confidence vs. random measurements.
- Motion noise parameters `alpha1..alpha4` should reflect your odometry quality; start small for simulated robots.

## Notes on differences from `openslam_gmapping`
- This implementation is intentionally compact and Pythonic for readability and education; it omits advanced optimizations (scan matching, entropy-based resampling triggers) from the original C++ project.
- Map publication happens at scan rate; if you prefer throttling, wrap the `_publish_map` call with a timer or message counter.
