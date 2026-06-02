# 🤖 Swarm Explorer — Multi-Robot Autonomous Exploration & Mapping

<div align="center">

<img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros" alt="ROS 2 Humble">
<img src="https://img.shields.io/badge/Gazebo-Classic-orange?style=for-the-badge&logo=ros" alt="Gazebo Classic">
<img src="https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python" alt="Python">
<img src="https://img.shields.io/badge/Ubuntu-22.04-red?style=for-the-badge&logo=ubuntu" alt="Ubuntu">

### Two TurtleBot3 robots autonomously explore, map, coordinate, and merge unknown environments in real time.

</div>

---

## 📖 Overview

Swarm Explorer is a ROS 2 Humble multi-robot exploration framework where two TurtleBot3 Burger robots perform autonomous SLAM, frontier exploration, cooperative task allocation, and map merging in Gazebo.

### Key Capabilities

* 🗺️ Cartographer-based SLAM for each robot
* 🔄 Real-time map merging
* 🎯 Frontier-based autonomous exploration
* 🤝 Multi-robot coordination
* 🚧 Robot-to-robot collision avoidance
* 📍 Nav2 autonomous navigation
* 💾 Automatic map saving
* 📊 RViz visualization and debugging

---

## 🛠️ Tech Stack

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

## 🏗️ System Architecture

```text
Robot1 Scan/Odom ──► Cartographer ──► Submap Decoder ──► /robot1/map
                                                            │
                                                            ▼
                                                      Map Merger
                                                            ▲
                                                            │
Robot2 Scan/Odom ──► Cartographer ──► Submap Decoder ──► /robot2/map

                              ▼
                      Frontier Detection
                              ▼
                      Goal Selection
                              ▼
                             Nav2
                              ▼
                       Robot Navigation
```

---

## 🔄 TF Architecture

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

### Publishers

* Cartographer → `robotX/map → robotX/odom`
* Gazebo → `robotX/odom → robotX/base_footprint`
* robot_state_publisher → `robotX/base_footprint → robotX/base_link`

---

## 📂 Repository Structure

```text
tb3_swarm_ws/
├── src/
│   ├── swarm_explorer/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── rviz/
│   │   └── swarm_explorer/
│   │       ├── submap_to_map_node.py
│   │       ├── map_merger.py
│   │       ├── robot_explorer.py
│   │       ├── frontier_detector.py
│   │       ├── frontier_coordinator.py
│   │       ├── nav2_goal_client.py
│   │       └── waypoint_patrol.py
│   │
│   ├── turtlebot3_cartographer/
│   ├── turtlebot3_navigation2/
│   └── turtlebot3_gazebo/
```

---

## 🚀 Quick Start

### Build

```bash
cd ~/tb3_swarm_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Autonomous Exploration

```bash
ros2 launch swarm_explorer swarm_exploration.launch.py
```

### Launch Coordinated Exploration

```bash
ros2 launch swarm_explorer swarm_exploration_coordinated.launch.py
```

### Launch Waypoint Patrol

```bash
ros2 launch swarm_explorer swarm_mapping.launch.py
```

---

## 🎮 Modes of Operation

| Mode              | Description                              |
| ----------------- | ---------------------------------------- |
| Waypoint Patrol   | Hardcoded waypoint-based mapping         |
| Single Explorer   | Autonomous exploration with one robot    |
| Dual Explorer     | Independent exploration by two robots    |
| Coordinated Swarm | Frontier sharing and collision avoidance |

---

## 🔧 Core Components

### 🗺️ Submap Decoder

Converts Cartographer submap textures into standard ROS `OccupancyGrid` maps.

### 🔄 Map Merger

Combines `/robot1/map` and `/robot2/map` into `/merged_map` using live TF alignment.

### 🎯 Frontier Exploration

Detects boundaries between known and unknown space and autonomously selects exploration targets.

### 🤝 Frontier Coordination

Prevents robots from selecting the same frontier through a shared claiming protocol.

### 🚧 Collision Avoidance

Broadcasts robot footprints as obstacles so teammates avoid planning through one another.

---

## 📊 Verification Commands

```bash
# Frontier visualization
ros2 topic echo /robot1/frontier_markers --once

# Exploration status
ros2 topic echo /robot1/exploration_status --once

# Swarm completion
ros2 topic echo /swarm/exploration_done --once
```

---

## 🐛 Common Issues

| Problem                 | Cause                          | Fix                                 |
| ----------------------- | ------------------------------ | ----------------------------------- |
| Robots not moving       | Stamped cmd_vel mismatch       | Set `enable_stamped_cmd_vel: false` |
| Maps all unknown        | Incorrect occupancy conversion | Use Cartographer occupancy formula  |
| TF conflicts            | Multiple TF publishers         | Disable duplicate odom publishers   |
| Ghosting / double walls | Coordinate conversion issue    | Correct Cairo → ROS transform       |
| Obstacle deadlock       | Avoidance state never exits    | Add timeout recovery                |

---

## 📝 License

Developed for academic and research purposes.

---

## 🙏 Acknowledgments

* Google Cartographer
* Nav2
* TurtleBot3
* Gazebo Classic
* ROS 2 Humble
