"""
mapf_unity_adapter.py - Unity ROS-TCP Adapter for MAPF Solutions.

Unity with ROS-TCP-Connector enables:
1. Mixed reality visualization (AR/VR)
2. ML-Agents training environments
3. Custom rendering and effects
4. Cross-platform deployment
5. Educational/demo applications

References:
- ROS-TCP-Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub

This module provides the interface between verified MAPF solutions
and Unity's game object system via ROS messages.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from enum import Enum
import json
import math
from pathlib import Path
import struct
import time

from ..model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)


# ============================================================
# UNITY CONFIGURATION
# ============================================================

@dataclass
class UnityConfig:
    """
    Configuration for Unity simulation.

    Defines coordinate transforms, robot properties,
    and visualization settings.
    """
    # Coordinate system
    cell_size: float = 1.0          # Unity units per grid cell
    height_offset: float = 0.5      # Y offset for robots (Unity Y-up)

    # Robot settings
    robot_prefab: str = "Prefabs/Robot"
    robot_scale: float = 0.3
    move_speed: float = 2.0         # Units per second
    rotation_speed: float = 180.0   # Degrees per second

    # Visualization
    show_paths: bool = True
    show_goals: bool = True
    path_line_width: float = 0.1
    goal_marker_scale: float = 0.5

    # Colors (RGBA 0-1)
    path_colors: List[Tuple[float, float, float, float]] = field(default_factory=lambda: [
        (1.0, 0.2, 0.2, 0.8),   # Red
        (0.2, 1.0, 0.2, 0.8),   # Green
        (0.2, 0.2, 1.0, 0.8),   # Blue
        (1.0, 1.0, 0.2, 0.8),   # Yellow
        (1.0, 0.2, 1.0, 0.8),   # Magenta
        (0.2, 1.0, 1.0, 0.8),   # Cyan
    ])

    # ROS settings
    ros_namespace: str = "mapf"
    ros_tcp_ip: str = "127.0.0.1"
    ros_tcp_port: int = 10000

    # Output
    output_dir: str = "./unity_output"


class RobotAction(Enum):
    """Robot action types for Unity."""
    IDLE = "idle"
    MOVE_FORWARD = "move_forward"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    WAIT = "wait"
    ARRIVED = "arrived"


# ============================================================
# COORDINATE TRANSFORM
# ============================================================

@dataclass
class UnityCoordinateTransform:
    """
    Transform between MAPF grid and Unity world coordinates.

    Unity uses:
    - Left-handed coordinate system
    - Y-up by default
    - Variable units (typically 1 unit = 1 meter)
    """
    cell_size: float
    origin_x: float = 0.0
    origin_z: float = 0.0       # Unity Z is horizontal
    ground_y: float = 0.0       # Unity Y is vertical

    def grid_to_unity(self, gx: int, gy: int) -> Tuple[float, float, float]:
        """
        Convert grid coordinates to Unity position (x, y, z).

        Note: MAPF y maps to Unity z (horizontal plane).
        """
        ux = self.origin_x + (gx + 0.5) * self.cell_size
        uy = self.ground_y
        uz = self.origin_z + (gy + 0.5) * self.cell_size
        return (ux, uy, uz)

    def unity_to_grid(self, ux: float, uz: float) -> Tuple[int, int]:
        """Convert Unity position to grid coordinates."""
        gx = int((ux - self.origin_x) / self.cell_size)
        gy = int((uz - self.origin_z) / self.cell_size)
        return (gx, gy)

    def compute_rotation(
        self,
        from_pos: Tuple[float, float, float],
        to_pos: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Compute Unity Euler rotation (degrees) to face target.

        Returns (x, y, z) rotation where Y is the yaw.
        """
        dx = to_pos[0] - from_pos[0]
        dz = to_pos[2] - from_pos[2]
        yaw = math.degrees(math.atan2(dx, dz))
        return (0.0, yaw, 0.0)


# ============================================================
# UNITY ROBOT DATA
# ============================================================

@dataclass
class UnityWaypoint:
    """Single waypoint in Unity coordinates."""
    time: float
    position: Tuple[float, float, float]  # (x, y, z) Unity
    rotation: Tuple[float, float, float]  # Euler degrees (x, y, z)
    action: RobotAction

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
            "rotation": {"x": self.rotation[0], "y": self.rotation[1], "z": self.rotation[2]},
            "action": self.action.value
        }


@dataclass
class UnityRobotTrajectory:
    """Complete trajectory for one robot in Unity."""
    robot_id: int
    robot_name: str
    waypoints: List[UnityWaypoint]
    color: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    prefab: str = "Prefabs/Robot"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "robot_name": self.robot_name,
            "prefab": self.prefab,
            "color": {"r": self.color[0], "g": self.color[1],
                     "b": self.color[2], "a": self.color[3]},
            "waypoints": [w.to_dict() for w in self.waypoints]
        }

    @property
    def duration(self) -> float:
        """Total trajectory duration."""
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].time

    @property
    def start_position(self) -> Tuple[float, float, float]:
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        return self.waypoints[0].position

    @property
    def goal_position(self) -> Tuple[float, float, float]:
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        return self.waypoints[-1].position


# ============================================================
# PATH CONVERSION
# ============================================================

def convert_mapf_path_to_unity(
    path: List[int],
    robot_id: int,
    graph: Graph,
    transform: UnityCoordinateTransform,
    config: UnityConfig
) -> UnityRobotTrajectory:
    """
    Convert MAPF vertex path to Unity trajectory.

    Args:
        path: List of vertex IDs
        robot_id: Robot identifier
        graph: MAPF graph
        transform: Coordinate transform
        config: Unity configuration

    Returns:
        Complete trajectory with waypoints
    """
    waypoints = []
    current_time = 0.0

    # Extract vertex positions
    positions = getattr(graph, 'pos', {})

    for i, v in enumerate(path):
        # Get grid position
        if v in positions:
            gx, gy = positions[v]
        else:
            gx = v % 100
            gy = v // 100

        # Convert to Unity coordinates
        ux, uy, uz = transform.grid_to_unity(gx, gy)

        # Determine action and rotation
        if i == 0:
            action = RobotAction.IDLE
            rotation = (0.0, 0.0, 0.0)
        elif path[i] == path[i-1]:
            action = RobotAction.WAIT
            # Keep previous rotation
            rotation = waypoints[-1].rotation if waypoints else (0.0, 0.0, 0.0)
        else:
            action = RobotAction.MOVE_FORWARD
            prev_pos = waypoints[-1].position if waypoints else (ux, uy, uz)
            rotation = transform.compute_rotation(prev_pos, (ux, uy, uz))

        # Final waypoint
        if i == len(path) - 1:
            action = RobotAction.ARRIVED

        # Compute time based on distance
        if i > 0 and path[i] != path[i-1]:
            step_time = config.cell_size / config.move_speed
        else:
            step_time = config.cell_size / config.move_speed  # Standard timestep

        waypoint = UnityWaypoint(
            time=current_time,
            position=(ux, uy, uz),
            rotation=rotation,
            action=action
        )
        waypoints.append(waypoint)

        current_time += step_time

    # Assign color
    color_idx = robot_id % len(config.path_colors)
    color = config.path_colors[color_idx]

    return UnityRobotTrajectory(
        robot_id=robot_id,
        robot_name=f"Robot_{robot_id}",
        waypoints=waypoints,
        color=color,
        prefab=config.robot_prefab
    )


def convert_solution_to_unity(
    instance: MAPFInstance,
    result: MAPFResult,
    transform: UnityCoordinateTransform,
    config: UnityConfig
) -> List[UnityRobotTrajectory]:
    """Convert complete MAPF solution to Unity trajectories."""
    if result.status != ResultStatus.UNIQUE or not result.paths:
        return []

    trajectories = []
    for i, path in enumerate(result.paths):
        traj = convert_mapf_path_to_unity(
            path, i, instance.graph, transform, config
        )
        trajectories.append(traj)

    return trajectories


# ============================================================
# ROS MESSAGE GENERATION
# ============================================================

@dataclass
class ROSMessage:
    """Generic ROS message container."""
    topic: str
    msg_type: str
    data: Dict[str, Any]
    timestamp: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "topic": self.topic,
            "msg_type": self.msg_type,
            "timestamp": self.timestamp,
            "data": self.data
        }


def generate_transform_msg(
    robot_name: str,
    position: Tuple[float, float, float],
    rotation: Tuple[float, float, float],
    timestamp: float
) -> ROSMessage:
    """
    Generate geometry_msgs/TransformStamped message.

    Used for broadcasting robot positions.
    """
    # Convert Euler to quaternion
    roll, pitch, yaw = [math.radians(r) for r in rotation]
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return ROSMessage(
        topic=f"/tf",
        msg_type="geometry_msgs/TransformStamped",
        timestamp=timestamp,
        data={
            "header": {
                "stamp": {"sec": int(timestamp), "nanosec": int((timestamp % 1) * 1e9)},
                "frame_id": "world"
            },
            "child_frame_id": f"{robot_name}/base_link",
            "transform": {
                "translation": {"x": position[0], "y": position[1], "z": position[2]},
                "rotation": {"x": qx, "y": qy, "z": qz, "w": qw}
            }
        }
    )


def generate_twist_msg(
    robot_name: str,
    linear_vel: Tuple[float, float, float],
    angular_vel: Tuple[float, float, float]
) -> ROSMessage:
    """Generate geometry_msgs/Twist message for velocity commands."""
    return ROSMessage(
        topic=f"/{robot_name}/cmd_vel",
        msg_type="geometry_msgs/Twist",
        data={
            "linear": {"x": linear_vel[0], "y": linear_vel[1], "z": linear_vel[2]},
            "angular": {"x": angular_vel[0], "y": angular_vel[1], "z": angular_vel[2]}
        }
    )


def generate_path_msg(
    robot_name: str,
    trajectory: UnityRobotTrajectory
) -> ROSMessage:
    """Generate nav_msgs/Path message for visualization."""
    poses = []
    for wp in trajectory.waypoints:
        # Convert Euler to quaternion
        roll, pitch, yaw = [math.radians(r) for r in wp.rotation]
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        poses.append({
            "header": {
                "stamp": {"sec": int(wp.time), "nanosec": int((wp.time % 1) * 1e9)},
                "frame_id": "world"
            },
            "pose": {
                "position": {"x": wp.position[0], "y": wp.position[1], "z": wp.position[2]},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw}
            }
        })

    return ROSMessage(
        topic=f"/{robot_name}/planned_path",
        msg_type="nav_msgs/Path",
        data={
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "world"},
            "poses": poses
        }
    )


def generate_marker_msg(
    robot_name: str,
    marker_id: int,
    position: Tuple[float, float, float],
    color: Tuple[float, float, float, float],
    scale: float = 0.5,
    marker_type: int = 2  # SPHERE
) -> ROSMessage:
    """Generate visualization_msgs/Marker message."""
    return ROSMessage(
        topic="/visualization_markers",
        msg_type="visualization_msgs/Marker",
        data={
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "world"},
            "ns": robot_name,
            "id": marker_id,
            "type": marker_type,
            "action": 0,  # ADD
            "pose": {
                "position": {"x": position[0], "y": position[1], "z": position[2]},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            },
            "scale": {"x": scale, "y": scale, "z": scale},
            "color": {"r": color[0], "g": color[1], "b": color[2], "a": color[3]}
        }
    )


# ============================================================
# UNITY SCENE EXPORT
# ============================================================

@dataclass
class UnityGameObject:
    """Specification for a Unity GameObject."""
    name: str
    prefab: Optional[str] = None
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    components: List[Dict[str, Any]] = field(default_factory=list)
    children: List["UnityGameObject"] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "name": self.name,
            "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
            "rotation": {"x": self.rotation[0], "y": self.rotation[1], "z": self.rotation[2]},
            "scale": {"x": self.scale[0], "y": self.scale[1], "z": self.scale[2]}
        }
        if self.prefab:
            d["prefab"] = self.prefab
        if self.components:
            d["components"] = self.components
        if self.children:
            d["children"] = [c.to_dict() for c in self.children]
        return d


def generate_scene_hierarchy(
    trajectories: List[UnityRobotTrajectory],
    obstacles: List[Tuple[int, int]],
    grid_width: int,
    grid_height: int,
    transform: UnityCoordinateTransform,
    config: UnityConfig
) -> UnityGameObject:
    """
    Generate Unity scene hierarchy.

    Returns root GameObject with all scene elements.
    """
    root = UnityGameObject(name="MAPFScene")

    # Ground plane
    ground_width = grid_width * config.cell_size
    ground_height = grid_height * config.cell_size
    ground = UnityGameObject(
        name="Ground",
        prefab="Prefabs/GroundPlane",
        position=(ground_width / 2, 0, ground_height / 2),
        scale=(ground_width / 10, 1, ground_height / 10)
    )
    root.children.append(ground)

    # Obstacles container
    obstacles_container = UnityGameObject(name="Obstacles")
    for i, (ox, oy) in enumerate(obstacles):
        ux, uy, uz = transform.grid_to_unity(ox, oy)
        obstacle = UnityGameObject(
            name=f"Obstacle_{i}",
            prefab="Prefabs/Obstacle",
            position=(ux, config.cell_size / 2, uz),
            scale=(config.cell_size * 0.9, config.cell_size, config.cell_size * 0.9)
        )
        obstacles_container.children.append(obstacle)
    root.children.append(obstacles_container)

    # Robots container
    robots_container = UnityGameObject(name="Robots")
    for traj in trajectories:
        start_pos = traj.start_position
        robot = UnityGameObject(
            name=traj.robot_name,
            prefab=traj.prefab,
            position=start_pos,
            scale=(config.robot_scale, config.robot_scale, config.robot_scale),
            components=[
                {
                    "type": "MAPFRobotController",
                    "properties": {
                        "robotId": traj.robot_id,
                        "moveSpeed": config.move_speed,
                        "rotationSpeed": config.rotation_speed
                    }
                },
                {
                    "type": "MeshRenderer",
                    "properties": {
                        "material": {
                            "color": {"r": traj.color[0], "g": traj.color[1],
                                    "b": traj.color[2], "a": traj.color[3]}
                        }
                    }
                }
            ]
        )
        robots_container.children.append(robot)
    root.children.append(robots_container)

    # Goals container
    if config.show_goals:
        goals_container = UnityGameObject(name="Goals")
        for traj in trajectories:
            goal_pos = traj.goal_position
            goal = UnityGameObject(
                name=f"Goal_{traj.robot_id}",
                prefab="Prefabs/GoalMarker",
                position=(goal_pos[0], 0.05, goal_pos[2]),
                scale=(config.goal_marker_scale, 0.1, config.goal_marker_scale),
                components=[
                    {
                        "type": "MeshRenderer",
                        "properties": {
                            "material": {
                                "color": {"r": traj.color[0], "g": traj.color[1],
                                        "b": traj.color[2], "a": 0.5}
                            }
                        }
                    }
                ]
            )
            goals_container.children.append(goal)
        root.children.append(goals_container)

    # Paths container (LineRenderers)
    if config.show_paths:
        paths_container = UnityGameObject(name="Paths")
        for traj in trajectories:
            points = [wp.position for wp in traj.waypoints]
            path = UnityGameObject(
                name=f"Path_{traj.robot_id}",
                components=[
                    {
                        "type": "LineRenderer",
                        "properties": {
                            "positions": [{"x": p[0], "y": 0.1, "z": p[2]} for p in points],
                            "startWidth": config.path_line_width,
                            "endWidth": config.path_line_width,
                            "startColor": {"r": traj.color[0], "g": traj.color[1],
                                          "b": traj.color[2], "a": traj.color[3]},
                            "endColor": {"r": traj.color[0], "g": traj.color[1],
                                        "b": traj.color[2], "a": traj.color[3]}
                        }
                    }
                ]
            )
            paths_container.children.append(path)
        root.children.append(paths_container)

    return root


# ============================================================
# C# SCRIPT GENERATION
# ============================================================

def generate_robot_controller_script() -> str:
    """Generate C# script for robot control in Unity."""
    return '''using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// MAPF Robot Controller - Auto-generated
/// Controls robot movement along pre-planned path
/// </summary>
public class MAPFRobotController : MonoBehaviour
{
    [Header("Robot Settings")]
    public int robotId;
    public float moveSpeed = 2.0f;
    public float rotationSpeed = 180.0f;
    public float waypointThreshold = 0.1f;

    [Header("Path Data")]
    public List<Waypoint> waypoints = new List<Waypoint>();
    private int currentWaypointIndex = 0;
    private bool isMoving = false;

    [System.Serializable]
    public class Waypoint
    {
        public float time;
        public Vector3 position;
        public Vector3 rotation;
        public string action;
    }

    void Start()
    {
        if (waypoints.Count > 0)
        {
            transform.position = waypoints[0].position;
            transform.eulerAngles = waypoints[0].rotation;
        }
    }

    void Update()
    {
        if (!isMoving || currentWaypointIndex >= waypoints.Count - 1)
            return;

        Waypoint target = waypoints[currentWaypointIndex + 1];

        // Move towards target
        Vector3 direction = (target.position - transform.position).normalized;
        float distance = Vector3.Distance(transform.position, target.position);

        if (distance > waypointThreshold)
        {
            // Rotate to face target
            if (direction != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                transform.rotation = Quaternion.RotateTowards(
                    transform.rotation,
                    targetRotation,
                    rotationSpeed * Time.deltaTime
                );
            }

            // Move forward
            transform.position = Vector3.MoveTowards(
                transform.position,
                target.position,
                moveSpeed * Time.deltaTime
            );
        }
        else
        {
            // Reached waypoint
            currentWaypointIndex++;

            if (currentWaypointIndex >= waypoints.Count - 1)
            {
                OnReachedGoal();
            }
        }
    }

    public void StartMoving()
    {
        isMoving = true;
    }

    public void StopMoving()
    {
        isMoving = false;
    }

    public void LoadWaypoints(List<Waypoint> newWaypoints)
    {
        waypoints = newWaypoints;
        currentWaypointIndex = 0;
    }

    private void OnReachedGoal()
    {
        Debug.Log($"Robot {robotId} reached goal!");
        isMoving = false;
    }

    public float GetProgress()
    {
        if (waypoints.Count <= 1) return 1.0f;
        return (float)currentWaypointIndex / (waypoints.Count - 1);
    }
}
'''


def generate_simulation_manager_script() -> str:
    """Generate C# script for simulation management."""
    return '''using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// MAPF Simulation Manager - Auto-generated
/// Manages overall simulation execution
/// </summary>
public class MAPFSimulationManager : MonoBehaviour
{
    [Header("Simulation Settings")]
    public float simulationSpeed = 1.0f;
    public bool autoStart = false;

    [Header("References")]
    public List<MAPFRobotController> robots = new List<MAPFRobotController>();

    private bool isRunning = false;
    private float simulationTime = 0.0f;

    void Start()
    {
        // Find all robot controllers
        robots.AddRange(FindObjectsOfType<MAPFRobotController>());

        if (autoStart)
        {
            StartSimulation();
        }
    }

    void Update()
    {
        if (isRunning)
        {
            simulationTime += Time.deltaTime * simulationSpeed;

            // Check completion
            if (AllRobotsAtGoal())
            {
                OnSimulationComplete();
            }
        }
    }

    public void StartSimulation()
    {
        isRunning = true;
        simulationTime = 0.0f;

        foreach (var robot in robots)
        {
            robot.StartMoving();
        }

        Debug.Log("MAPF Simulation Started");
    }

    public void PauseSimulation()
    {
        isRunning = false;

        foreach (var robot in robots)
        {
            robot.StopMoving();
        }
    }

    public void ResetSimulation()
    {
        isRunning = false;
        simulationTime = 0.0f;

        // Reset all robots to start positions
        // (Requires storing initial positions)
    }

    private bool AllRobotsAtGoal()
    {
        foreach (var robot in robots)
        {
            if (robot.GetProgress() < 1.0f)
                return false;
        }
        return true;
    }

    private void OnSimulationComplete()
    {
        isRunning = false;
        Debug.Log($"MAPF Simulation Complete! Time: {simulationTime:F2}s");
    }

    // UI Methods
    public float GetSimulationTime() => simulationTime;
    public int GetRobotCount() => robots.Count;
    public float GetOverallProgress()
    {
        if (robots.Count == 0) return 0;
        float total = 0;
        foreach (var robot in robots)
        {
            total += robot.GetProgress();
        }
        return total / robots.Count;
    }
}
'''


def generate_ros_bridge_script() -> str:
    """Generate C# script for ROS-TCP-Connector integration."""
    return '''using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;

/// <summary>
/// MAPF ROS Bridge - Auto-generated
/// Handles ROS communication for MAPF robots
/// </summary>
public class MAPFROSBridge : MonoBehaviour
{
    [Header("ROS Settings")]
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;
    public float publishRate = 10.0f; // Hz

    [Header("Topic Names")]
    public string tfTopic = "/tf";
    public string pathTopicPrefix = "/robot_";

    private ROSConnection ros;
    private List<MAPFRobotController> robots;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect(rosIP, rosPort);

        robots = new List<MAPFRobotController>(
            FindObjectsOfType<MAPFRobotController>()
        );

        // Register publishers
        ros.RegisterPublisher<TransformStampedMsg>(tfTopic);

        foreach (var robot in robots)
        {
            string topic = pathTopicPrefix + robot.robotId + "/path";
            ros.RegisterPublisher<PathMsg>(topic);
        }
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishTransforms();
            lastPublishTime = Time.time;
        }
    }

    private void PublishTransforms()
    {
        foreach (var robot in robots)
        {
            TransformStampedMsg msg = new TransformStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeMsg { sec = (uint)Time.time, nanosec = 0 },
                    frame_id = "world"
                },
                child_frame_id = $"robot_{robot.robotId}/base_link",
                transform = new TransformMsg
                {
                    translation = new Vector3Msg
                    {
                        x = robot.transform.position.x,
                        y = robot.transform.position.y,
                        z = robot.transform.position.z
                    },
                    rotation = new QuaternionMsg
                    {
                        x = robot.transform.rotation.x,
                        y = robot.transform.rotation.y,
                        z = robot.transform.rotation.z,
                        w = robot.transform.rotation.w
                    }
                }
            };

            ros.Publish(tfTopic, msg);
        }
    }

    public void PublishPath(int robotId, List<PoseStampedMsg> poses)
    {
        PathMsg pathMsg = new PathMsg
        {
            header = new HeaderMsg { frame_id = "world" },
            poses = poses.ToArray()
        };

        string topic = pathTopicPrefix + robotId + "/path";
        ros.Publish(topic, pathMsg);
    }
}
'''


# ============================================================
# UNITY ADAPTER
# ============================================================

class UnityAdapter:
    """
    Complete Unity adapter for MAPF solutions.

    Provides:
    - Solution to trajectory conversion
    - Unity scene hierarchy export
    - ROS message generation
    - C# script generation
    """

    def __init__(self, config: Optional[UnityConfig] = None):
        self.config = config or UnityConfig()
        self.transform = UnityCoordinateTransform(cell_size=self.config.cell_size)

        Path(self.config.output_dir).mkdir(parents=True, exist_ok=True)

    def convert_solution(
        self,
        instance: MAPFInstance,
        result: MAPFResult
    ) -> List[UnityRobotTrajectory]:
        """Convert MAPF solution to Unity trajectories."""
        return convert_solution_to_unity(
            instance, result, self.transform, self.config
        )

    def export_trajectories(
        self,
        trajectories: List[UnityRobotTrajectory],
        filename: str
    ) -> str:
        """Export trajectories to JSON."""
        filepath = Path(self.config.output_dir) / filename
        data = {
            "trajectories": [t.to_dict() for t in trajectories],
            "config": {
                "cell_size": self.config.cell_size,
                "move_speed": self.config.move_speed,
                "robot_prefab": self.config.robot_prefab
            }
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        return str(filepath)

    def export_scene(
        self,
        trajectories: List[UnityRobotTrajectory],
        obstacles: List[Tuple[int, int]],
        grid_width: int,
        grid_height: int,
        filename: str
    ) -> str:
        """Export Unity scene hierarchy to JSON."""
        scene = generate_scene_hierarchy(
            trajectories, obstacles, grid_width, grid_height,
            self.transform, self.config
        )
        filepath = Path(self.config.output_dir) / filename
        with open(filepath, 'w') as f:
            json.dump(scene.to_dict(), f, indent=2)
        return str(filepath)

    def export_ros_messages(
        self,
        trajectories: List[UnityRobotTrajectory],
        filename: str
    ) -> str:
        """Export ROS messages for all robots."""
        messages = []

        for traj in trajectories:
            # Transform messages at each waypoint
            for wp in traj.waypoints:
                tf_msg = generate_transform_msg(
                    traj.robot_name, wp.position, wp.rotation, wp.time
                )
                messages.append(tf_msg.to_dict())

            # Path message
            path_msg = generate_path_msg(traj.robot_name, traj)
            messages.append(path_msg.to_dict())

            # Goal marker
            goal_marker = generate_marker_msg(
                traj.robot_name,
                traj.robot_id + 1000,
                traj.goal_position,
                traj.color,
                self.config.goal_marker_scale
            )
            messages.append(goal_marker.to_dict())

        filepath = Path(self.config.output_dir) / filename
        with open(filepath, 'w') as f:
            json.dump(messages, f, indent=2)
        return str(filepath)

    def export_scripts(self) -> Dict[str, str]:
        """Export all C# scripts."""
        scripts = {}

        # Robot controller
        script_path = Path(self.config.output_dir) / "MAPFRobotController.cs"
        with open(script_path, 'w') as f:
            f.write(generate_robot_controller_script())
        scripts["robot_controller"] = str(script_path)

        # Simulation manager
        script_path = Path(self.config.output_dir) / "MAPFSimulationManager.cs"
        with open(script_path, 'w') as f:
            f.write(generate_simulation_manager_script())
        scripts["simulation_manager"] = str(script_path)

        # ROS bridge
        script_path = Path(self.config.output_dir) / "MAPFROSBridge.cs"
        with open(script_path, 'w') as f:
            f.write(generate_ros_bridge_script())
        scripts["ros_bridge"] = str(script_path)

        return scripts

    def export_complete(
        self,
        name: str,
        instance: MAPFInstance,
        result: MAPFResult,
        obstacles: List[Tuple[int, int]],
        grid_width: int,
        grid_height: int
    ) -> Dict[str, str]:
        """Export complete Unity package."""
        trajectories = self.convert_solution(instance, result)

        files = {}

        # Trajectories
        files["trajectories"] = self.export_trajectories(
            trajectories, f"{name}_trajectories.json"
        )

        # Scene hierarchy
        files["scene"] = self.export_scene(
            trajectories, obstacles, grid_width, grid_height,
            f"{name}_scene.json"
        )

        # ROS messages
        files["ros_messages"] = self.export_ros_messages(
            trajectories, f"{name}_ros_messages.json"
        )

        # C# scripts
        script_files = self.export_scripts()
        files.update(script_files)

        # Summary
        summary = {
            "name": name,
            "num_robots": len(trajectories),
            "max_duration": max(t.duration for t in trajectories) if trajectories else 0,
            "grid_size": {"width": grid_width, "height": grid_height},
            "num_obstacles": len(obstacles),
            "files": files
        }

        summary_path = Path(self.config.output_dir) / f"{name}_summary.json"
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        files["summary"] = str(summary_path)

        return files

    def generate_setup_instructions(self) -> str:
        """Generate Unity project setup instructions."""
        return """
# Unity MAPF Project Setup Instructions

## Prerequisites
1. Unity 2021.3 LTS or later
2. ROS-TCP-Connector package (com.unity.robotics.ros-tcp-connector)
3. ROS Message packages for geometry_msgs, nav_msgs, std_msgs

## Package Installation
1. Open Unity Package Manager (Window > Package Manager)
2. Click + > Add package from git URL
3. Add: https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
4. Add message packages as needed

## Project Setup
1. Create new 3D project
2. Import the generated C# scripts into Assets/Scripts/
3. Create prefabs for:
   - Robot (with MAPFRobotController component)
   - Obstacle (simple cube with collider)
   - GoalMarker (flat cylinder or plane)
   - GroundPlane

## Scene Setup
1. Create empty GameObject named "MAPFScene"
2. Add MAPFSimulationManager component to it
3. Import scene JSON to create hierarchy
4. Load trajectory JSON into robot controllers

## ROS Integration
1. Add MAPFROSBridge to scene
2. Configure ROS IP/port in inspector
3. Start ROS TCP endpoint: roslaunch ros_tcp_endpoint endpoint.launch
4. Run simulation in Unity

## Running the Simulation
1. Press Play in Unity
2. Click "Start Simulation" or set autoStart = true
3. Monitor progress in console

## Building for Deployment
1. Configure build settings (File > Build Settings)
2. Add scenes to build
3. Build for target platform
"""


# ============================================================
# ML-AGENTS INTEGRATION
# ============================================================

def generate_ml_agents_config(num_robots: int) -> Dict[str, Any]:
    """
    Generate ML-Agents configuration for training.

    Can be used to train agents that learn MAPF-like behavior.
    """
    return {
        "behaviors": {
            "MAPFAgent": {
                "trainer_type": "ppo",
                "hyperparameters": {
                    "batch_size": 128,
                    "buffer_size": 2048,
                    "learning_rate": 3e-4,
                    "beta": 5e-3,
                    "epsilon": 0.2,
                    "lambd": 0.95,
                    "num_epoch": 3,
                    "learning_rate_schedule": "linear"
                },
                "network_settings": {
                    "normalize": True,
                    "hidden_units": 256,
                    "num_layers": 2
                },
                "reward_signals": {
                    "extrinsic": {
                        "gamma": 0.99,
                        "strength": 1.0
                    }
                },
                "max_steps": 5000000,
                "time_horizon": 64,
                "summary_freq": 10000
            }
        },
        "environment_parameters": {
            "num_robots": num_robots,
            "grid_size": 10,
            "obstacle_density": 0.2
        }
    }


def generate_training_environment_script() -> str:
    """Generate C# script for ML-Agents training environment."""
    return '''using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// MAPF Training Agent - For ML-Agents training
/// </summary>
public class MAPFTrainingAgent : Agent
{
    [Header("Settings")]
    public float moveSpeed = 2.0f;
    public float rotationSpeed = 180.0f;
    public Transform goal;

    private Rigidbody rb;
    private Vector3 startPosition;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startPosition = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        // Reset agent position
        transform.position = startPosition;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent position
        sensor.AddObservation(transform.position);

        // Goal position
        sensor.AddObservation(goal.position);

        // Direction to goal
        Vector3 toGoal = (goal.position - transform.position).normalized;
        sensor.AddObservation(toGoal);

        // Agent velocity
        sensor.AddObservation(rb.velocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Discrete actions: 0=forward, 1=left, 2=right, 3=wait
        int action = actions.DiscreteActions[0];

        switch (action)
        {
            case 0: // Forward
                rb.AddForce(transform.forward * moveSpeed);
                break;
            case 1: // Rotate left
                transform.Rotate(0, -rotationSpeed * Time.deltaTime, 0);
                break;
            case 2: // Rotate right
                transform.Rotate(0, rotationSpeed * Time.deltaTime, 0);
                break;
            case 3: // Wait
                break;
        }

        // Distance to goal reward
        float distance = Vector3.Distance(transform.position, goal.position);
        AddReward(-0.001f * distance);

        // Goal reached
        if (distance < 0.5f)
        {
            AddReward(1.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = 0; // Default: forward

        if (Input.GetKey(KeyCode.W)) discreteActions[0] = 0;
        if (Input.GetKey(KeyCode.A)) discreteActions[0] = 1;
        if (Input.GetKey(KeyCode.D)) discreteActions[0] = 2;
        if (Input.GetKey(KeyCode.S)) discreteActions[0] = 3;
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Obstacle") ||
            collision.gameObject.CompareTag("Robot"))
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }
}
'''
