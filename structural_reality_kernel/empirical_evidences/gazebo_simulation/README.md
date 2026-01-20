# MAPF Gazebo Simulation - 12x12 Grid, 8 Agents

Complete simulation package for executing verified MAPF solutions in Gazebo.

## Overview

This package demonstrates the full pipeline:
1. **MAPF Solving**: CBS finds optimal collision-free paths
2. **Verification**: V1-V5 truth gate validates solution
3. **Simulation**: Gazebo executes with 8 TurtleBot3 robots
4. **Safety Monitoring**: Real-time collision detection

## Scenario

```
Grid:           12 × 12 (144 cells)
Agents:         8 TurtleBot3 robots
Cell Size:      0.5m × 0.5m
World Size:     6m × 6m
Time per Step:  2.0 seconds
```

### Agent Configuration

| Agent | Start (grid) | Goal (grid) | Start (world) | Goal (world) | Color |
|-------|--------------|-------------|---------------|--------------|-------|
| 0 | (0, 0) | (11, 11) | (0.25, 0.25) | (5.75, 5.75) | Red |
| 1 | (11, 0) | (0, 11) | (5.75, 0.25) | (0.25, 5.75) | Teal |
| 2 | (0, 11) | (11, 0) | (0.25, 5.75) | (5.75, 0.25) | Blue |
| 3 | (11, 11) | (0, 0) | (5.75, 5.75) | (0.25, 0.25) | Green |
| 4 | (5, 0) | (6, 11) | (2.75, 0.25) | (3.25, 5.75) | Yellow |
| 5 | (6, 0) | (5, 11) | (3.25, 0.25) | (2.75, 5.75) | Plum |
| 6 | (5, 11) | (6, 0) | (2.75, 5.75) | (3.25, 0.25) | Mint |
| 7 | (6, 11) | (5, 0) | (3.25, 5.75) | (2.75, 0.25) | Gold |

## Requirements

### ROS2 (Humble or later)
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### Python Dependencies
```bash
pip install numpy
```

## Quick Start

### 1. Source ROS2
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

### 2. Run the Simulation
```bash
# Terminal 1: Launch Gazebo with 8 robots
ros2 launch mapf_simulation multi_robot_gazebo.launch.py

# Terminal 2: Execute MAPF solution
ros2 run mapf_simulation execute_mapf_solution.py

# Terminal 3: Monitor safety (optional)
ros2 run mapf_simulation safety_monitor.py
```

### 3. Standalone Test (no ROS2 required)
```bash
python3 scripts/standalone_simulation.py
```

## Files

```
gazebo_simulation/
├── README.md                    # This file
├── config/
│   ├── robots.yaml              # Robot spawn positions
│   ├── mapf_solution.json       # Pre-computed CBS solution
│   └── waypoints.yaml           # Navigation waypoints
├── launch/
│   ├── multi_robot_gazebo.launch.py  # Main launch file
│   └── single_robot.launch.py        # Single robot test
├── worlds/
│   └── mapf_grid_12x12.world    # Gazebo world file
└── scripts/
    ├── standalone_simulation.py  # No-ROS test
    ├── execute_mapf_solution.py  # Waypoint executor
    └── safety_monitor.py         # Collision detector
```

## Solution Metrics

From CBS solver:
- **Makespan**: 22 timesteps
- **Sum of Costs**: 136
- **CBS Nodes Explored**: 2,516
- **Naive Space**: 144^8 = 1.85 × 10^17
- **Compression**: 10^13.9

## Verification

All solutions pass V1-V5 truth gate:
- V1 (Start): ✓ All robots at correct initial positions
- V2 (Goal): ✓ All robots reach destinations
- V3 (Dynamics): ✓ All movements valid (4-connected)
- V4 (Vertex): ✓ No two robots occupy same cell
- V5 (Edge): ✓ No head-on collisions

## Visualization

The simulation provides:
- Gazebo 3D view of all 8 robots
- RViz path visualization
- Real-time position tracking
- Collision warning alerts

## Safety Guarantees

The safety monitor ensures:
1. **Spatial separation**: Minimum 0.3m between robots
2. **Reservation compliance**: Robots follow planned paths
3. **Emergency stop**: Halt on any violation detection
