# Gazebo Controller

**EN605.613 Final Project Template**  
Created by: Galen Mullins (gmullin3@jh.edu)

A ROS 2 differential drive robot controller with Gazebo simulation and multiple maze environments.

## Overview

This package provides a complete simulation environment for a differential drive robot navigating through various maze configurations. It includes:

- Differential drive robot with LiDAR sensor
- Multiple maze environments (basic_maze, Maze_hr, Maze_ng, Maze_ql_1)
- PID-based velocity controller
- RViz visualization with occupancy grid maps
- Goal point markers for navigation tasks

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12+
- Required ROS 2 packages:
  - `ros_gz_sim`
  - `ros_gz_bridge`
  - `robot_state_publisher`
  - `tf2_ros`

## Installation

1. Clone or copy this package to your ROS 2 workspace:
   ```bash
   mkdir -p ~/gazebo_controller_ws/src
   <Copy the gazeb_controller folder into src>
   ```

2. Build the package:
   ```bash
   cd ~/gazebo_controller_ws
   colcon build --packages-select gazebo_controller
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launching the Simulation

The main launch file accepts a `map_folder` argument to select which maze environment to load:

```bash
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=<maze_name>
```

**Available Mazes:**
- `basic_maze` (default) - Simple 9m x 9m maze with three rooms
- `Maze_hr` - Complex maze (requires map.yaml for visualization)
- `Maze_ng` - Dense maze with many walls
- `Maze_ql_1` - Large maze environment

**Examples:**

```bash
# Launch with default basic_maze
ros2 launch gazebo_controller full_simulation.launch.py

# Launch with Maze_hr
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_hr

# Launch with Maze_ng
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_ng

# Launch with Maze_ql_1
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_ql_1
```

### What Gets Launched

The full simulation includes:

1. **Gazebo Simulator** - Physics simulation and rendering
2. **Robot Model** - Differential drive robot with:
   - Blue rectangular body (0.4m x 0.2m x 0.1m)
   - Two drive wheels (0.08m radius, 0.24m separation)
   - LiDAR sensor (360° coverage, 30m range)
3. **Goal Spheres** - Three green goal markers placed in the maze
4. **RViz** - Visualization with:
   - Robot model
   - LiDAR scan data
   - Occupancy grid map (if map.yaml exists)
   - Goal point markers
   - TF frames
5. **Mapper Node** - Occupancy grid map publisher
6. **Planner Node** - A* path planner (map -> path -> waypoints)
7. **Controller Node** - PID-based differential drive waypoint follower
8. **Publishers** - Goal point publishers

## Robot Control

The robot subscribes to velocity commands on `/cmd_vel`:

```bash
# Example: Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Example: Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

## Topics

**Subscribed:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Published:**
- `/lidar` (sensor_msgs/LaserScan) - LiDAR scan data
- `/odom` (nav_msgs/Odometry) - Odometry from Gazebo
- `/map` (nav_msgs/OccupancyGrid) - Static occupancy grid map
- `/goal_points` (visualization_msgs/MarkerArray) - Goal point markers for RViz

## File Structure

```
gazebo_controller/
├── config/
│   └── rviz_config.rviz          # RViz configuration
├── gazebo_controller/
│   ├── diffdrive_pid.py           # PID velocity controller
│   ├── map_publisher.py           # Occupancy grid publisher
│   └── goal_points_publisher.py  # Goal marker publisher
├── launch/
│   ├── full_simulation.launch.py # Main launch file
│   ├── spawn_entities.launch.py  # Robot/goal spawner
│   └── controller.launch.py      # Controller nodes
├── sdf/
│   ├── vehicle_blue_model.sdf    # Robot model definition
│   ├── goal_sphere.sdf           # Goal marker model
│   ├── basic_maze/
│   │   ├── maze_world.sdf        # Maze world file
│   │   ├── poses.csv             # Robot/goal positions
│   │   └── map.yaml              # Map definition
│   ├── Maze_hr/
│   ├── Maze_ng/
│   └── Maze_ql_1/
└── scripts/
    └── generate_maps.py          # Utility to generate map.yaml files
```

## Creating Custom Mazes

To add a new maze:

1. Create a new directory in `sdf/`:
   ```bash
   mkdir sdf/my_maze
   ```

2. Create `maze_world.sdf` with your maze definition (see existing mazes for format)

3. Create `poses.csv` with robot and goal positions:
   ```csv
   name,x,y,z,yaw
   robot,0.0,0.0,0.08,0.0
   goal_1,1.0,1.0,0.04,0.0
   goal_2,2.0,2.0,0.04,0.0
   goal_3,3.0,3.0,0.04,0.0
   ```

4. (Optional) Create `map.yaml` for occupancy grid visualization:
   ```yaml
   resolution: 0.05
   width: 10.0
   height: 10.0
   origin_x: -5.0
   origin_y: -5.0
   walls:
     - [0.0, 5.0, 10.0, 0.15]  # [center_x, center_y, size_x, size_y]
   ```

5. Update `setup.py` to include your maze:
   ```python
   ('share/' + package_name + '/sdf/my_maze', glob('sdf/my_maze/*')),
   ```

6. Rebuild:
   ```bash
   colcon build --packages-select gazebo_controller
   ```

## Troubleshooting

**Issue:** Gazebo window is black or frozen
- Wait a few seconds for initialization
- Check GPU drivers are properly installed

**Issue:** Robot doesn't appear
- Check that poses.csv exists in the maze directory
- Verify robot spawn position is not inside a wall

**Issue:** Map doesn't show in RViz
- Ensure map.yaml exists for your maze
- Check map_publisher logs: `ros2 topic echo /map`
- If map.yaml is missing, the publisher will log a warning but continue running

**Issue:** LiDAR not working
- Verify the lidar transform is correct
- Check `/lidar` topic: `ros2 topic echo /lidar`

**Issue:** Robot moves asymmetrically
- Check wheel parameters in vehicle_blue_model.sdf
- Verify max_wheel_torque is set correctly (no acceleration limits)

## Development

To modify the robot behavior:

1. **Controller tuning:** Edit PID gains in `diffdrive_pid.py`
2. **Robot dimensions:** Edit `sdf/vehicle_blue_model.sdf`
3. **Sensor configuration:** Modify LiDAR parameters in vehicle_blue_model.sdf
4. **Visualization:** Customize `config/rviz_config.rviz`

## License

MIT License

## Contact

For questions or issues, contact Galen Mullins at gmullin3@jh.edu
