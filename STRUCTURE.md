# TurtleBot3 Swarm Exploration Workspace - Final Structure

**Location:** `~/tb3_swarm_ws`  
**Total Size:** 117 MB  
**ROS2 Distro:** Humble  
**Status:** ✅ Built & Ready to Test

---

## 📁 Directory Tree

```
tb3_swarm_ws/
├── src/
│   ├── swarm_explorer/          (🆕 Custom frontier exploration package)
│   │   ├── swarm_explorer/
│   │   │   ├── map_utils.py
│   │   │   ├── frontier_detector.py
│   │   │   ├── goal_selector.py
│   │   │   ├── nav2_goal_client.py
│   │   │   ├── robot_explorer.py
│   │   │   └── map_merger.py
│   │   ├── launch/
│   │   │   ├── two_robot_exploration.launch.py
│   │   │   └── map_merge.launch.py
│   │   └── setup.py
│   │
│   ├── turtlebot3/
│   │   ├── turtlebot3_bringup/
│   │   ├── turtlebot3_cartographer/
│   │   │   ├── config/
│   │   │   │   ├── robot1_cartographer.lua (🆕)
│   │   │   │   └── robot2_cartographer.lua (🆕)
│   │   │   └── launch/two_tb3_cartographer.launch.py (🆕)
│   │   ├── turtlebot3_description/
│   │   └── turtlebot3_navigation2/
│   │       ├── param/
│   │       │   ├── robot1_nav2_params.yaml (🆕)
│   │       │   └── robot2_nav2_params.yaml (🆕)
│   │       └── launch/two_tb3_navigation2.launch.py (🆕)
│   │
│   └── turtlebot3_simulations/
│       └── turtlebot3_gazebo/
│           ├── launch/swarm_indoor_world.launch.py (🆕)
│           ├── worlds/swarm_indoor_world.world (🆕)
│           └── models/ (burger + common only)
│
├── build/       (colcon build artifacts)
├── install/     (colcon install - source for runtime)
└── log/         (colcon build logs)
```

---

## 🆕 Newly Created Files (Swarm-Specific)

### Gazebo
- `src/turtlebot3_simulations/turtlebot3_gazebo/worlds/swarm_indoor_world.world` — 10×10m room with walls, partitions, furniture
- `src/turtlebot3_simulations/turtlebot3_gazebo/launch/swarm_indoor_world.launch.py` — Dual-robot spawn at (-2,-2) and (2,-2)

### Cartographer
- `src/turtlebot3/turtlebot3_cartographer/config/robot1_cartographer.lua` — SLAM config for robot1
- `src/turtlebot3/turtlebot3_cartographer/config/robot2_cartographer.lua` — SLAM config for robot2
- `src/turtlebot3/turtlebot3_cartographer/launch/two_tb3_cartographer.launch.py` — Launch 2 SLAM instances

### Nav2
- `src/turtlebot3/turtlebot3_navigation2/param/robot1_nav2_params.yaml` — Navigation params for robot1
- `src/turtlebot3/turtlebot3_navigation2/param/robot2_nav2_params.yaml` — Navigation params for robot2
- `src/turtlebot3/turtlebot3_navigation2/launch/two_tb3_navigation2.launch.py` — Launch 2 Nav2 stacks

### Frontier Explorer (swarm_explorer package)
- `src/swarm_explorer/swarm_explorer/map_utils.py` — Grid utilities
- `src/swarm_explorer/swarm_explorer/frontier_detector.py` — Frontier detection & clustering
- `src/swarm_explorer/swarm_explorer/goal_selector.py` — Frontier scoring
- `src/swarm_explorer/swarm_explorer/nav2_goal_client.py` — Nav2 action client
- `src/swarm_explorer/swarm_explorer/robot_explorer.py` — Main exploration node
- `src/swarm_explorer/swarm_explorer/map_merger.py` — Map merging node
- `src/swarm_explorer/launch/two_robot_exploration.launch.py` — Launch 2 explorers
- `src/swarm_explorer/launch/map_merge.launch.py` — Launch map merger

---

## 🚀 Quick Start

### Setup
```bash
cd ~/tb3_swarm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### Terminal 1 — Gazebo
```bash
ros2 launch turtlebot3_gazebo swarm_indoor_world.launch.py
```

### Terminal 2 — Cartographer
```bash
ros2 launch turtlebot3_cartographer two_tb3_cartographer.launch.py
```

### Terminal 3 — Nav2
```bash
ros2 launch turtlebot3_navigation2 two_tb3_navigation2.launch.py
```

### Terminal 4 — Map Merger
```bash
ros2 launch swarm_explorer map_merge.launch.py
```

### Terminal 5 — Frontier Exploration
```bash
ros2 launch swarm_explorer two_robot_exploration.launch.py
```

### Terminal 6 — RViz2
```bash
rviz2
```

---

## 📊 Topics Published

**Robot 1:**
- `/robot1/map` — Cartographer map
- `/robot1/scan` — LiDAR scan
- `/robot1/odom` — Odometry
- `/robot1/frontier_markers` — Frontier visualization (MarkerArray)

**Robot 2:**
- `/robot2/map` — Cartographer map
- `/robot2/scan` — LiDAR scan
- `/robot2/odom` — Odometry
- `/robot2/frontier_markers` — Frontier visualization (MarkerArray)

**Merged:**
- `/merged_map` — Combined occupancy grid

---

## 📦 Key Packages

| Package | Type | Purpose |
|---------|------|---------|
| **swarm_explorer** | Custom (ament_python) | Frontier detection, goal selection, map merging |
| **turtlebot3_bringup** | ament_cmake | Robot startup & parameters |
| **turtlebot3_cartographer** | ament_cmake | SLAM configuration |
| **turtlebot3_description** | ament_cmake | Robot URDF & meshes |
| **turtlebot3_navigation2** | ament_cmake | Nav2 stack |
| **turtlebot3_gazebo** | ament_cmake | Gazebo simulation |

---

## ✅ Build Status

```
Summary: 6 packages finished [30.0s]
  ✅ swarm_explorer
  ✅ turtlebot3_bringup
  ✅ turtlebot3_cartographer
  ✅ turtlebot3_description
  ✅ turtlebot3_navigation2
  ✅ turtlebot3_gazebo
```

---

## 📋 File Counts

- **Python modules:** 7 (swarm_explorer)
- **Launch files:** 9 (including 4 dual-robot specific)
- **Config files:** 6 (2 Cartographer, 2 Nav2, 2 custom)
- **World files:** 1 custom (swarm_indoor_world.world)
- **Total source files:** ~85

---

## 🗑️ Cleaned Up

Removed to reduce clutter:
- ❌ docker/, examples/, test/ directories
- ❌ turtlebot3_teleop, turtlebot3_node, turtlebot3_example packages
- ❌ Unnecessary model variants (waffle, house, autorace)
- ❌ Root-level launch/, param/, script/ directories
- ❌ Unused world files

---

Generated: 2026-05-16  
Status: Ready for testing
