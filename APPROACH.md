# Frontier-Based Maze Navigation — Approach Document

## 1. Robot Model Changes

No hardware changes were made to the default `vehicle_blue` differential-drive robot.  The robot retains its original geometry (0.4 × 0.2 × 0.1 m body, 0.08 m wheels, 0.04 m caster) and sensor suite (2D GPU Lidar: 36 samples, ±180°, range 0.1–30 m).  The only configuration change is that the static pre-built map is no longer pre-loaded — the robot now builds its map online from lidar data.

## 2. Algorithm

### Mapping — Log-Odds Lidar Occupancy Grid (`lidar_mapper`)

The mapper maintains a 24 × 24 m occupancy grid at 0.05 m/cell (480 × 480 cells, centred at the world origin) using the **log-odds** probabilistic model:

- Each cell accumulates evidence from successive lidar sweeps.
- Per scan, a Bresenham integer line is traced from the lidar origin to each beam endpoint.  Cells along the ray are decremented (free evidence); the endpoint cell is incremented (occupied evidence).
- Thresholds convert log-odds to the three-valued `OccupancyGrid`: **free** (< −0.10), **occupied** (> +0.10), or **unknown**.
- The map is published on `/map` at 2 Hz and consumed by both the planner and RViz.

### Planning — Frontier Exploration with LIFO Stack (`frontier_planner`)

**Frontier detection** uses a vectorised NumPy pass over the occupancy grid:

1. A *frontier cell* is defined as a **free** cell that has at least one **unknown** 4-neighbour.
2. Adjacent frontier cells are merged into clusters via BFS.  Clusters smaller than 4 cells are discarded as noise.
3. Each cluster's centroid (in world coordinates) is a candidate exploration target.

**LIFO stack management** (depth-first exploration):

- Candidate centroids are pushed onto a Python list used as a stack (`append` / `pop()`).
- Before pushing, the centroid is compared against existing stack entries; duplicates within 0.80 m are suppressed.
- Stale entries (centroids whose immediate neighbourhood is now fully explored) are pruned on every update cycle.
- The robot always selects the **most recently pushed** frontier (LIFO), creating a depth-first search pattern that quickly penetrates maze corridors.

**Goal priority** — At every navigation tick the planner first attempts to reach each remaining mission goal via A*.  Only if no goal has a viable path does it fall back to the frontier stack.  This means the robot navigates directly to a goal the moment a route is available, without waiting for full map coverage.

**Path execution** reuses the A* waypoint-stride approach from the original template:

- The A* path is sub-sampled (every 6 cells by default) to produce sparse waypoints.
- Each waypoint is published to `/goal_pose` in sequence.
- A stuck-detection timeout (10 s without progress) causes the planner to abandon and re-select.

### Control — Trailer-Hitch PID (`diffdrive_pid`)

The existing PID controller is **unchanged**.  It tracks each `/goal_pose` target using a virtual trailer-hitch point 0.60 m ahead of the robot, converting the 2-D Cartesian error into linear and angular velocity commands via an inverse-rotation matrix.  Per-maze gains (speeds, lookahead) remain as tuned in the original launch file.

## 3. ROS Node Architecture

```
/lidar (LaserScan)  ──► lidar_mapper ──► /map (OccupancyGrid)
/odom  (Odometry)  ──►   └──────────────────────────────────────►  frontier_planner
/goal_points        ──►                                             /goal_points (MarkerArray)
                                            frontier_planner ──► /goal_pose (PoseStamped)
                                            frontier_planner ──► /planned_path (Path)
                                            frontier_planner ──► /frontiers (MarkerArray)
/goal_pose          ──► diffdrive_pid ──► /cmd_vel (Twist)
/odom               ──►
```

| Node | Package | Role |
|---|---|---|
| `lidar_mapper` | `gazebo_controller` | Builds `/map` from lidar scans (log-odds ray casting) |
| `frontier_planner` | `gazebo_controller` | Frontier detection, LIFO stack, A* planning, waypoint management |
| `diffdrive_pid` | `gazebo_controller` | Trailer-hitch PID generates `/cmd_vel` |
| `goal_points_publisher` | `gazebo_controller` | Reads `poses.csv` → publishes known goal positions on `/goal_points` |
| `ros_gz_bridge` | `ros_gz_bridge` | Bridges `/lidar`, `/odom`, `/cmd_vel`, `/tf` between Gazebo and ROS 2 |

## 4. Challenges

**Technical ROS challenges:**
- The default Python environment in the development container is a pyenv-managed build that does not share packages with the system APT Python.  NumPy had to be installed separately via `pip` before `colcon build` would resolve all imports at runtime.
- The Gazebo lidar frame ID (`vehicle_blue/lidar`) differs from the TF frame published by `spawn_entities` (`vehicle_blue/lidar/lidar_sensor`).  To avoid a TF dependency in the mapper, the robot yaw from `/odom` and the fixed 0.16 m forward offset are applied directly in the mapper callback.

**Algorithmic challenges:**
- **Map initialisation lag**: For the first ~2 s before enough scan coverage exists, the planner has no free cells and hence no frontiers.  The stuck-detection timer is set high enough (10 s) to tolerate this cold-start period.
- **Frontier near occupied cells**: Bresenham ray casting with 36-sample lidar and log-odds filtering occasionally marks cells just behind thin maze walls as free before they are revisited and corrected.  The 3-cell obstacle inflation in the planner provides a safety margin that absorbs most such errors.
- **LIFO vs. coverage**: Pure LIFO (depth-first) is efficient in corridor mazes but can leave pockets unexplored near the start.  The goal-priority check mitigates this: once a goal is unblocked (even partially explored corridors suffice), the robot navigates there immediately rather than continuing deep exploration.

## 5. Potential Improvements

- **Increase lidar resolution**: The default 36-ray, 10°-increment lidar misses narrow gaps.  Raising to 360 samples (1° resolution) would yield a far more accurate occupancy grid and fewer phantom free cells.
- **Loop-closure / SLAM**: The current mapper accumulates odometry drift over long runs.  Integrating a scan-matching step (ICP or correlative scan matching) would correct pose estimates and reduce map distortion.
- **Frontier clustering with size weighting**: Larger frontier clusters represent more unexplored territory.  Sorting the stack by cluster size (instead of pure LIFO) would improve exploration efficiency in open maps.
- **Dynamic re-inflation**: Obstacle inflation is fixed at 3 cells.  An adaptive radius based on corridor width estimates from the lidar would allow faster traversal in wide sections while preserving safety in tight ones.
- **Recovery behaviours**: The current stuck handler simply abandons the waypoint.  A dedicated recovery (short reverse + turn) would handle cases where the robot wedges against a wall more gracefully.

---

## How to Run

### Environment

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| Python dependencies | `numpy` (≥ 1.26), standard ROS 2 Python client libs |

### Build

```bash
cd /workspaces/finalExam
source /opt/ros/jazzy/setup.bash
colcon build --packages-select gazebo_controller
source install/setup.bash
```

### Launch

```bash
# Basic maze (default)
ros2 launch gazebo_controller full_simulation.launch.py

# Named maze environments
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_hr
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_ng
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_ql_1

# Slower, safer tuning profile
ros2 launch gazebo_controller full_simulation.launch.py map_folder:=Maze_ng drive_profile:=presentation_safe
```

The launch file starts Gazebo, spawns the robot and goal spheres, starts the ROS–Gazebo bridge, `lidar_mapper`, `frontier_planner`, `diffdrive_pid`, and RViz in a single command.

### Visualisation (noVNC in Codespace)

Gazebo and RViz render to virtual display `:99`.  Open port **6080** in the Codespace Ports panel to access the noVNC browser viewer.  The RViz configuration shows: occupancy map, laser scan, planned path, frontier markers (orange spheres), goal markers (green spheres), and robot odometry trail.

### All Source Files Modified / Added

| File | Status |
|---|---|
| `gazebo_controller/lidar_mapper.py` | **New** — log-odds lidar mapper |
| `gazebo_controller/frontier_planner.py` | **New** — frontier exploration + A* planner |
| `launch/full_simulation.launch.py` | Modified — uses `lidar_mapper` + `frontier_planner` |
| `launch/spawn_entities.launch.py` | Modified — removes static `map_publisher` |
| `setup.py` | Modified — registers new entry points |
| `rviz/rviz_view.rviz` | Modified — adds Frontiers MarkerArray display |
