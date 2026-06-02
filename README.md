# Swarm Explorer - Multi-Robot Autonomous Exploration & Mapping

<div align="center">

<img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros" alt="ROS 2 Humble">
<img src="https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge&logo=ros" alt="Gazebo Classic">
<img src="https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python" alt="Python">
<img src="https://img.shields.io/badge/Ubuntu-22.04-red?style=for-the-badge&logo=ubuntu" alt="Ubuntu">

### Two TurtleBot3 robots autonomously explore, map, coordinate, and merge unknown environments in real time.

</div>

---

# 📖 Overview

Swarm Explorer is a ROS 2 Humble multi-robot exploration framework where two TurtleBot3 Burger robots perform autonomous SLAM, frontier exploration, cooperative task allocation, and map merging in Gazebo.

## ✨ Key Capabilities

* 🗺️ Cartographer-based SLAM for each robot
* 🔧 Custom submap decoder converting Cartographer textures into ROS OccupancyGrid maps
* 🔄 Real-time map merging with live TF alignment
* 🎯 Frontier-based autonomous exploration
* 🤝 Multi-robot coordination through frontier claiming
* 🚧 Robot-to-robot collision avoidance
* 📍 Nav2 autonomous navigation
* 💾 Automatic map saving on exploration completion
* 📊 RViz visualization and debugging tools

---

# 🛠️ Tech Stack

| Component     | Technology          |
| ------------- | ------------------- |
| OS            | Ubuntu 22.04        |
| Middleware    | ROS 2 Humble        |
| Simulator     | Gazebo Classic      |
| SLAM          | Google Cartographer |
| Navigation    | Nav2                |
| Robots        | TurtleBot3 Burger   |
| Language      | Python 3.10         |
| Visualization | RViz2               |

---

# 🏗️ System Architecture

```text
Robot1 Scan/Odom ──► Cartographer ──► Custom Submap Decoder ──► /robot1/map
                                                                    │
                                                                    ▼
                                                              Map Merger
                                                                    ▲
                                                                    │
Robot2 Scan/Odom ──► Cartographer ──► Custom Submap Decoder ──► /robot2/map

                              ▼
                      Frontier Detection
                              ▼
                      Goal Selection
                              ▼
                             Nav2
                              ▼
                       Robot Navigation
```

### Why a Custom Submap Decoder?

Cartographer does not natively publish standard `OccupancyGrid` maps in namespaced multi-robot configurations.

The custom decoder:

1. Queries Cartographer's internal submap service.
2. Decompresses gzipped texture data.
3. Converts grayscale textures into occupancy probabilities.
4. Applies Cartographer's occupancy conversion formula.
5. Publishes standard ROS OccupancyGrid maps for Nav2 and map merging.

This bridges Cartographer's internal representation and the ROS navigation stack.

---

# 🔄 TF Architecture

```text
map
├── robot1/map
│   └── robot1/odom
│       └── robot1/base_footprint
│           └── robot1/base_link
│
└── robot2/map
    └── robot2/odom
        └── robot2/base_footprint
            └── robot2/base_link
```

### TF Publishers

| Transform                                | Publisher             |
| ---------------------------------------- | --------------------- |
| robotX/map → robotX/odom                 | Cartographer          |
| robotX/odom → robotX/base_footprint      | Gazebo                |
| robotX/base_footprint → robotX/base_link | robot_state_publisher |

---

# 📂 Repository Structure

```text
tb3_swarm_ws/
├── src/
│
├── swarm_explorer/
│   ├── launch/
│   │   ├── swarm_mapping.launch.py
│   │   ├── swarm_exploration.launch.py
│   │   ├── swarm_exploration_coordinated.launch.py
│   │   ├── rviz_swarm.launch.py
│   │   └── two_robot_patrol.launch.py
│   │
│   ├── config/
│   │   ├── map_merge.yaml
│   │   ├── robot1_explorer.yaml
│   │   └── robot2_explorer.yaml
│   │
│   ├── rviz/
│   │   └── two_tb3_swarm.rviz
│   │
│   └── swarm_explorer/
│       ├── submap_to_map_node.py
│       ├── map_merger.py
│       ├── robot_explorer.py
│       ├── frontier_detector.py
│       ├── frontier_coordinator.py
│       ├── goal_selector.py
│       ├── nav2_goal_client.py
│       ├── robot_footprint_broadcaster.py
│       └── waypoint_patrol.py
│
├── turtlebot3_cartographer/
│   ├── launch/
│   └── config/
│
├── turtlebot3_navigation2/
│   ├── launch/
│   └── param/
│
└── turtlebot3_gazebo/
    ├── launch/
    └── worlds/
```

---

# 🚀 Quick Start

## Build Workspace

```bash
cd ~/tb3_swarm_ws

colcon build --symlink-install

source install/setup.bash
```

## Launch Autonomous Exploration

```bash
ros2 launch swarm_explorer swarm_exploration.launch.py
```

## Launch Coordinated Exploration

```bash
ros2 launch swarm_explorer swarm_exploration_coordinated.launch.py
```

## Launch Waypoint Patrol

```bash
ros2 launch swarm_explorer swarm_mapping.launch.py
```

---

# 🎮 Modes of Operation

| Mode              | Description                              |
| ----------------- | ---------------------------------------- |
| Waypoint Patrol   | Hardcoded waypoint-based mapping         |
| Single Explorer   | Autonomous exploration with one robot    |
| Dual Explorer     | Independent exploration by two robots    |
| Coordinated Swarm | Frontier sharing and collision avoidance |

---

# 🔧 Core Components

## 🗺️ Custom Submap Decoder

Converts Cartographer's internal submap textures into standard ROS OccupancyGrid maps.

Features:

* Queries `/robotX/submap_query`
* Decompresses gzipped texture data
* Handles 7-bit and 8-bit pixel formats
* Applies Cartographer occupancy conversion
* Performs speckle filtering
* Publishes `/robotX/map` with TRANSIENT_LOCAL QoS

---

## 🔄 Map Merger

Combines:

```text
/robot1/map
+
/robot2/map
=
/merged_map
```

Uses live TF lookups instead of static offsets, automatically accounting for Cartographer pose graph optimization.

---

## 🎯 Frontier Exploration

Autonomously discovers and explores unknown regions by:

1. Detecting frontiers.
2. Ranking candidates.
3. Selecting exploration goals.
4. Sending Nav2 goals.

---

## 🤝 Frontier Coordination

Prevents duplicate exploration.

Mechanism:

* Robots publish claimed frontiers.
* Claimed frontiers are ignored by teammates.
* Claims automatically expire after timeout.

---

## 🚧 Collision Avoidance

Each robot publishes its footprint as an obstacle.

The teammate's Nav2 costmap treats the footprint as a dynamic obstacle, preventing path planning through occupied robot locations.

---

# 📊 Verification Commands

## Map Inspection

```bash
ros2 topic echo /robot1/map --once --field data | head -20
```

## Frontier Visualization

```bash
ros2 topic echo /robot1/frontier_markers --once
```

## Exploration Status

```bash
ros2 topic echo /robot1/exploration_status --once
```

## Swarm Completion

```bash
ros2 topic echo /swarm/exploration_done --once
```

---

# 🐛 Common Issues

| Problem                 | Cause                          | Fix                                 |
| ----------------------- | ------------------------------ | ----------------------------------- |
| Robots not moving       | Stamped cmd_vel mismatch       | Set `enable_stamped_cmd_vel: false` |
| Maps all unknown        | Incorrect occupancy conversion | Use Cartographer occupancy formula  |
| TF conflicts            | Multiple TF publishers         | Disable duplicate odom publishers   |
| Ghosting / double walls | Coordinate conversion issue    | Correct Cairo → ROS transform       |
| Obstacle deadlock       | Avoidance state never exits    | Add timeout recovery                |

---

# 📝 License

Developed for academic and research purposes.

---

# 🙏 Acknowledgments

* Google Cartographer
* Nav2
* TurtleBot3
* Gazebo Classic
* ROS 2 Humble
* Open Source Robotics Community
