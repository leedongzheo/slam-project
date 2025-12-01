# Map Layer – Python GMapping for TurtleBot3

This step provides a **from-scratch Python reimplementation of GMapping** that lives in the [`map_layer`](./map_layer/) folder. It replaces the dependency on the C++ `openslam_gmapping` tree while keeping the same workflow used in the previous steps of the project: TurtleBot3 simulation in Gazebo (step 1), RGB-D capture and preprocessing (steps 2–3), and map visualization in RViz.

- The `map_layer` package contains modular building blocks (motion model, sensor model, occupancy grid, particle filter, resampler) plus a ROS 2 node that publishes `/map` and `/gmapping_particles` from `/scan` and TF.
- See [`map_layer/README.md`](./map_layer/README.md) for detailed build instructions, algorithm explanation, and a demo recipe on TurtleBot3 with ROS 2 Jazzy and Gazebo Harmonic.

## Quickstart (ROS 2 Jazzy on Ubuntu 24.04)
1. Source your ROS 2 environment and build the package with colcon:
   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/turtlebot3_ws
   rosdep install --from-paths src --ignore-src -y
   colcon build --packages-select map_layer
   source install/setup.bash
   ```
2. Launch the simulated robot and sensor pipeline (steps 1–3) in Gazebo Harmonic.
3. Run the new GMapping node:
   ```bash
   ros2 run map_layer gmapping_node
   ```
4. Open RViz2, set the fixed frame to `map`, and add a `Map` display on `/map` to watch the occupancy grid grow as you drive the robot.
