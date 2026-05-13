# Puzzlebot Fully Autonomous Navigation

A complete ROS 2 Humble autonomous navigation system for the Puzzlebot differential-drive robot. The robot builds its own map with SLAM, localises inside it, plans collision-free paths with A\*, follows them with a pure-pursuit controller, avoids dynamic obstacles in real time, and streams its camera feed — all without human intervention.

> Course: Integración de robótica y sistemas inteligentes — Tecnológico de Monterrey

---

## System Architecture

```
Camera + LiDAR (Gazebo simulation)
        │
        ▼
   Sensor Bridge (ros_gz_bridge)
  /scan  /camera/image_raw  /imu
        │
        ▼
  ┌─────────────┐        ┌──────────────────┐
  │  SLAM Node  │──/map──▶  A* Planner Node │◀── /goal_pose  (user input)
  │ (slam_node) │        └──────────────────┘
  └─────────────┘                 │ /path
        │ /slam_pose              ▼
        │              ┌─────────────────────┐
        └─────────────▶│  Path Follower Node │◀── /scan  (obstacle avoidance)
                       └─────────────────────┘
                                 │ /cmd_vel  (Twist)
                                 ▼
                          twist_relay node
                                 │ puzzlebot_controller/cmd_vel  (TwistStamped)
                                 ▼
                        simple_controller node
                                 │ wheel velocity commands
                                 ▼
                          Gazebo Simulation
                          (Puzzlebot moves!)
```

### TF Tree

```
map ──▶ odom ──▶ base_footprint ──▶ base_link ──▶ lidar_link
                                               ──▶ camera_link
                                               ──▶ wheel_right_link
                                               ──▶ wheel_left_link
```

---

## ROS 2 Packages

| Package | Description |
|---|---|
| `puzzlebot_description` | URDF/Xacro model, Gazebo worlds, RViz configs |
| `puzzlebot_bringup` | Top-level launch: Gazebo + controller |
| `puzzlebot_controller` | Differential drive (`simple_controller`), `twist_relay` |
| `puzzlebot_localization` | Monte Carlo Localization (MCL / particle filter) |
| `puzzlebot_localization_cpp` | EKF localization (C++) |
| `puzzlebot_slam` | Custom occupancy-grid SLAM with optional ICP scan matching |
| `puzzlebot_navigation` | **A\* planner, pure-pursuit follower, dynamic obstacle spawner** |

---

## Prerequisites

### ROS 2 Humble + simulation dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  python3-scipy
```

### Python dependencies

```bash
pip install scipy numpy
```

---

## Build

```bash
cd ~/puzzlebot_nav_ws
colcon build --symlink-install
source install/setup.bash
```

> Run `source install/setup.bash` in **every new terminal** before any ROS 2 command.

---

## How to Run

There are two modes: **Quick Start** (2 terminals, everything automated) and **Step-by-Step** (5 terminals, for development and debugging).

---

## Quick Start — 2 Terminals

### Terminal 1 — Launch everything

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 launch puzzlebot_navigation autonomous_nav.launch.py
```

This single launch file starts **all** components simultaneously:

| Component | What it does |
|---|---|
| Gazebo | Physics simulation with robot, LiDAR, camera, IMU |
| `simple_controller` + `twist_relay` | Converts `/cmd_vel` → wheel velocity commands |
| `slam_node` | Builds the occupancy map in real time; publishes `/map` and `/slam_pose` |
| `astar_planner` | Reads `/map` + `/goal_pose`, publishes `/path` using A\* |
| `path_follower` | Follows `/path` with pure pursuit; emergency-stops on close obstacles |
| `obstacle_spawner` | Adds a random box to Gazebo every 20 s (forces replanning) |
| RViz2 | Visualises map, path, robot model, LiDAR scan |

### Terminal 2 — Send a navigation goal

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Change `x` and `y` to reach different targets. The robot plans and executes the path fully autonomously.

**To send another goal without stopping:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: -2.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Step-by-Step — 5 Terminals

Use this layout when you need to inspect individual node output or restart components independently.

### Terminal 1 — Gazebo Simulation

```bash
  source ~/puzzlebot_nav_ws/install/setup.bash
  ros2 launch puzzlebot_bringup simulated_robot.launch.py world_name:=obstacles
```

**What starts:**
- Ignition Gazebo with the `obstacles` world (4 walls + 3 static obstacles inside)
- Puzzlebot spawned with LiDAR, RGB camera, and IMU
- `simple_controller` — computes and publishes `/puzzlebot_controller/odom`
- `twist_relay` — converts incoming `Twist` on `/cmd_vel` → `TwistStamped`
- Sensor bridge — publishes `/scan`, `/camera/image_raw`, `/camera/camera_info`

**Verify sensors are working:**
```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /camera/image_raw --once
```

---

### Terminal 2 — SLAM

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 launch puzzlebot_slam slam.launch.py mode:=sim world_name:=obstacles
```

**What starts:**
- `slam_node` — subscribes to `/scan` + `/puzzlebot_controller/odom`, publishes `/map` (OccupancyGrid) and `/slam_pose` (PoseStamped), broadcasts `map→odom` TF
- RViz2 showing the growing map
- `teleop_twist_keyboard` in an xterm window — **use this to drive the robot** and build the map

**Drive the robot to build the map:**
- In the teleop window: use `i / j / l / ,` keys to move the robot
- Watch the map grow in RViz as you explore the environment
- Once the walls and obstacles are mapped, close the teleop (Ctrl+C in xterm)

---

### Terminal 3 — A\* Global Planner

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 run puzzlebot_navigation astar_planner
```

**What it does:**
- Waits for `/map` (latched — receives last published map immediately)
- Waits for `/slam_pose` (current robot pose)
- On each new `/goal_pose`: inflates obstacles by 0.25 m, runs A\* with Euclidean heuristic, publishes the smoothed path to `/path`
- Monitors `/scan`: when a new obstacle appears within 0.55 m, triggers automatic replanning

**Verify planning works:**
```bash
ros2 topic echo /path --once
```

---

### Terminal 4 — Path Follower

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 run puzzlebot_navigation path_follower
```

**What it does:**
- Subscribes to `/path`, `/slam_pose`, and `/scan`
- 20 Hz pure-pursuit control loop: aims at a look-ahead point 0.40 m ahead on the path
- Slows down when an obstacle enters the 0.60 m zone, full stops at 0.35 m
- Publishes to `/cmd_vel` (Twist) which the `twist_relay` converts for the controller
- Logs "Goal reached!" and stops when within 0.15 m of the final waypoint

---

### Terminal 5 — Send Goal / Monitor

**Send a goal:**
```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
```

**Monitor velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Monitor robot pose:**
```bash
ros2 topic echo /slam_pose
```

---

## Optional Components

### Dynamic obstacle spawner (any terminal)

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 run puzzlebot_navigation obstacle_spawner
```

Drops a new coloured box in Gazebo every 20 s at a random position. The A\* planner detects the new obstacle via `/scan` and replans the path automatically.

### Live camera feed

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### MCL localization (alternative to SLAM pose)

If you have a saved map and prefer Monte Carlo Localization over the SLAM estimated pose:

```bash
source ~/puzzlebot_nav_ws/install/setup.bash
ros2 launch puzzlebot_localization mcl.launch.py
```

Then change the planner/follower subscriptions from `/slam_pose` to `/mcl_pose`.

---

## Key Topics Reference

| Topic | Message Type | Publisher | Subscribers |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge | SLAM, A\* planner, path follower |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge | RViz, rqt_image_view |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge | — |
| `/odom` (← `/puzzlebot_controller/odom`) | `nav_msgs/Odometry` | simple_controller | SLAM |
| `/map` | `nav_msgs/OccupancyGrid` | slam_node | A\* planner |
| `/slam_pose` | `geometry_msgs/PoseStamped` | slam_node | A\* planner, path follower |
| `/goal_pose` | `geometry_msgs/PoseStamped` | **user** | A\* planner |
| `/path` | `nav_msgs/Path` | astar_planner | path follower |
| `/cmd_vel` | `geometry_msgs/Twist` | path_follower | twist_relay |

---

## Terminal Summary

### Quick Start (recommended)

| Terminal | Command |
|---|---|
| **1** | `ros2 launch puzzlebot_navigation autonomous_nav.launch.py` |
| **2** | `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{...}'` |

### Step-by-Step (debugging)

| Terminal | Command |
|---|---|
| **1** | `ros2 launch puzzlebot_bringup simulated_robot.launch.py world_name:=obstacles` |
| **2** | `ros2 launch puzzlebot_slam slam.launch.py mode:=sim` |
| **3** | `ros2 run puzzlebot_navigation astar_planner` |
| **4** | `ros2 run puzzlebot_navigation path_follower` |
| **5** | `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{...}'` |

---

## Navigation Parameter Tuning

Edit `src/puzzlebot_navigation/config/nav_params.yaml`:

| Parameter | Default | Effect |
|---|---|---|
| `inflation_radius` | 0.25 m | Increase for wider clearance around walls |
| `obstacle_threshold` | 50 | Lower = more cells treated as obstacles |
| `lookahead_distance` | 0.40 m | Larger = smoother but less precise turns |
| `linear_speed` | 0.20 m/s | Increase for faster navigation |
| `angular_gain` | 2.0 | Increase for snappier steering |
| `goal_tolerance` | 0.15 m | How close = "arrived" |
| `obstacle_stop_dist` | 0.35 m | Emergency stop threshold |
| `obstacle_slow_dist` | 0.60 m | Begin slowing at this distance |
| `spawn_interval` | 20 s | Seconds between dynamic obstacle spawns |

---

## Worlds

| World | Description | Use when |
|---|---|---|
| `empty` | Flat ground only | Testing SLAM and basic navigation |
| `obstacles` | 4 walls + 2 boxes + 1 cylinder + 1 diagonal wall | Full challenge with replanning |

```bash
# Empty world
ros2 launch puzzlebot_navigation autonomous_nav.launch.py world_name:=empty

# Full obstacle world (default)
ros2 launch puzzlebot_navigation autonomous_nav.launch.py world_name:=obstacles

# Disable dynamic spawner
ros2 launch puzzlebot_navigation autonomous_nav.launch.py spawn_obstacles:=false
```

---

## Useful Debug Commands

```bash
# List all active topics
ros2 topic list

# Check sensor data
ros2 topic hz /scan
ros2 topic hz /map

# Inspect TF tree
ros2 run tf2_tools view_frames

# Node graph
ros2 run rqt_graph rqt_graph

# Check robot doctor (connectivity)
ros2 doctor

# Echo any topic
ros2 topic echo /slam_pose
ros2 topic echo /path
ros2 topic echo /cmd_vel
```

---

## Development Phases

- [x] Phase 1 — Gazebo simulation with robot, LiDAR, camera, IMU
- [x] Phase 2 — Custom SLAM (occupancy grid + optional ICP refinement)
- [x] Phase 3 — Localization (MCL particle filter + EKF)
- [x] Phase 4 — A\* global path planning with obstacle inflation and path smoothing
- [x] Phase 5 — Autonomous navigation (pure-pursuit path follower)
- [x] Phase 6 — Dynamic obstacle avoidance (LiDAR emergency stop + automatic replanning)
- [x] Phase 7 — Camera integration (RGB stream bridged to ROS 2, visualised in RViz)

---

## Future Extensions

- YOLO object detection on camera feed
- Nav2 integration (standard navigation stack)
- Semantic mapping
- Multi-robot coordination
- Digital twin synchronisation
- Web dashboard via ROS Bridge
