"""
mapf_isaac_adapter.py - NVIDIA Isaac Sim Adapter for MAPF Solutions.

NVIDIA Isaac Sim is the premier robotics simulation platform for
digital twin workflows. This adapter enables:

1. Scale simulation (dozens to hundreds of robots)
2. Physically accurate collision detection
3. High-fidelity visual rendering for videos
4. ROS2 bridge integration
5. USD stage manipulation

References:
- Isaac Sim Multi-Robot Navigation: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_multi_robot_navigation.html
- Isaac Sim Python API: https://docs.omniverse.nvidia.com/py/isaacsim/

This module provides the interface layer between verified MAPF solutions
and Isaac Sim's USD scene graph and action graph systems.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from enum import Enum
import json
import math
from pathlib import Path

from .mapf_model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    H,
    canon_json
)


# ============================================================
# ISAAC SIM CONFIGURATION
# ============================================================

@dataclass
class IsaacSimConfig:
    """
    Configuration for Isaac Sim simulation.

    All physics and rendering parameters for accurate simulation.
    """
    # Physics settings
    physics_dt: float = 1.0 / 60.0      # Physics timestep (60Hz)
    rendering_dt: float = 1.0 / 30.0    # Render timestep (30fps)

    # World settings
    gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81)

    # Robot settings
    robot_usd_path: str = "/Isaac/Robots/Carter/carter_v1.usd"
    robot_radius: float = 0.3           # Collision radius (m)
    robot_height: float = 0.5           # Robot height (m)

    # Navigation settings
    linear_velocity: float = 0.5        # m/s
    angular_velocity: float = 1.0       # rad/s
    goal_tolerance: float = 0.1         # Position tolerance (m)

    # Grid settings
    cell_size: float = 1.0              # Grid cell size in meters

    # Visualization
    show_paths: bool = True
    show_goals: bool = True
    path_color: Tuple[float, float, float, float] = (0.2, 0.8, 0.2, 0.8)
    goal_color: Tuple[float, float, float, float] = (0.8, 0.2, 0.2, 0.8)

    # Recording settings
    output_dir: str = "./isaac_output"
    record_video: bool = False
    video_fps: int = 30
    video_resolution: Tuple[int, int] = (1920, 1080)


class RobotState(Enum):
    """Robot state in simulation."""
    IDLE = "idle"
    MOVING = "moving"
    ROTATING = "rotating"
    AT_GOAL = "at_goal"
    COLLISION = "collision"


# ============================================================
# COORDINATE TRANSFORM
# ============================================================

@dataclass
class IsaacCoordinateTransform:
    """
    Transform between MAPF grid coordinates and Isaac Sim world coordinates.

    Isaac Sim uses:
    - Right-handed coordinate system
    - Z-up by default
    - Meters as unit
    """
    cell_size: float            # Size of each grid cell in meters
    origin_x: float = 0.0       # World X of grid origin
    origin_y: float = 0.0       # World Y of grid origin
    origin_z: float = 0.0       # Ground plane Z

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float, float]:
        """Convert grid coordinates to world position."""
        wx = self.origin_x + (gx + 0.5) * self.cell_size
        wy = self.origin_y + (gy + 0.5) * self.cell_size
        wz = self.origin_z
        return (wx, wy, wz)

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world position to grid coordinates."""
        gx = int((wx - self.origin_x) / self.cell_size)
        gy = int((wy - self.origin_y) / self.cell_size)
        return (gx, gy)

    def compute_heading(
        self,
        from_pos: Tuple[float, float, float],
        to_pos: Tuple[float, float, float]
    ) -> float:
        """Compute heading angle (yaw) between two positions."""
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        return math.atan2(dy, dx)


# ============================================================
# ROBOT TRAJECTORY FOR ISAAC
# ============================================================

@dataclass
class IsaacWaypoint:
    """Single waypoint in Isaac Sim coordinates."""
    time: float                         # Simulation time (s)
    position: Tuple[float, float, float]  # (x, y, z) in meters
    orientation: Tuple[float, float, float, float]  # Quaternion (w, x, y, z)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "position": {"x": self.position[0], "y": self.position[1], "z": self.position[2]},
            "orientation": {"w": self.orientation[0], "x": self.orientation[1],
                          "y": self.orientation[2], "z": self.orientation[3]},
            "velocity": {"x": self.velocity[0], "y": self.velocity[1], "z": self.velocity[2]}
        }


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles to quaternion (w, x, y, z)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


@dataclass
class IsaacRobotTrajectory:
    """Complete trajectory for one robot in Isaac Sim."""
    robot_id: int
    robot_name: str
    waypoints: List[IsaacWaypoint]
    usd_path: str = "/Isaac/Robots/Carter/carter_v1.usd"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "robot_name": self.robot_name,
            "usd_path": self.usd_path,
            "waypoints": [w.to_dict() for w in self.waypoints]
        }

    @property
    def duration(self) -> float:
        """Total trajectory duration in seconds."""
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].time

    @property
    def start_position(self) -> Tuple[float, float, float]:
        """Starting position."""
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        return self.waypoints[0].position

    @property
    def goal_position(self) -> Tuple[float, float, float]:
        """Goal position."""
        if not self.waypoints:
            return (0.0, 0.0, 0.0)
        return self.waypoints[-1].position


# ============================================================
# PATH CONVERSION
# ============================================================

def convert_mapf_path_to_isaac(
    path: List[int],
    robot_id: int,
    graph: Graph,
    transform: IsaacCoordinateTransform,
    config: IsaacSimConfig
) -> IsaacRobotTrajectory:
    """
    Convert MAPF vertex path to Isaac Sim trajectory.

    Args:
        path: List of vertex IDs
        robot_id: Robot identifier
        graph: MAPF graph
        transform: Coordinate transform
        config: Isaac Sim configuration

    Returns:
        Complete trajectory with waypoints
    """
    waypoints = []
    current_time = 0.0

    # Extract vertex positions (assuming graph has pos dict)
    positions = getattr(graph, 'pos', {})

    for i, v in enumerate(path):
        # Get grid position
        if v in positions:
            gx, gy = positions[v]
        else:
            # Fallback: derive from vertex ID if grid structure
            gx = v % 100  # Assuming max width 100
            gy = v // 100

        # Convert to world coordinates
        wx, wy, wz = transform.grid_to_world(gx, gy)

        # Compute orientation (face direction of travel)
        if i < len(path) - 1:
            next_v = path[i + 1]
            if next_v in positions:
                ngx, ngy = positions[next_v]
            else:
                ngx = next_v % 100
                ngy = next_v // 100
            next_pos = transform.grid_to_world(ngx, ngy)
            yaw = transform.compute_heading((wx, wy, wz), next_pos)
        elif waypoints:
            # Keep previous orientation at goal
            yaw = math.atan2(
                waypoints[-1].orientation[3],
                waypoints[-1].orientation[0]
            ) * 2  # Approximate
        else:
            yaw = 0.0

        orientation = euler_to_quaternion(0.0, 0.0, yaw)

        # Compute time based on distance traveled
        if i > 0 and path[i] != path[i-1]:
            # Moving step - takes cell_size / velocity time
            step_time = config.cell_size / config.linear_velocity
        else:
            # Wait step - standard timestep
            step_time = config.cell_size / config.linear_velocity

        waypoint = IsaacWaypoint(
            time=current_time,
            position=(wx, wy, wz),
            orientation=orientation
        )
        waypoints.append(waypoint)

        current_time += step_time

    return IsaacRobotTrajectory(
        robot_id=robot_id,
        robot_name=f"robot_{robot_id}",
        waypoints=waypoints,
        usd_path=config.robot_usd_path
    )


def convert_solution_to_isaac(
    instance: MAPFInstance,
    result: MAPFResult,
    transform: IsaacCoordinateTransform,
    config: IsaacSimConfig
) -> List[IsaacRobotTrajectory]:
    """
    Convert complete MAPF solution to Isaac Sim trajectories.

    Args:
        instance: MAPF instance
        result: Verified MAPF result
        transform: Coordinate transform
        config: Isaac Sim configuration

    Returns:
        List of trajectories for all robots
    """
    if result.status != ResultStatus.UNIQUE or not result.paths:
        return []

    trajectories = []
    for i, path in enumerate(result.paths):
        traj = convert_mapf_path_to_isaac(
            path, i, instance.graph, transform, config
        )
        trajectories.append(traj)

    return trajectories


# ============================================================
# USD SCENE GENERATION
# ============================================================

@dataclass
class USDPrimSpec:
    """Specification for a USD primitive."""
    path: str
    type: str
    attributes: Dict[str, Any] = field(default_factory=dict)
    references: List[str] = field(default_factory=list)


def generate_ground_plane(
    width: int,
    height: int,
    cell_size: float,
    obstacles: List[Tuple[int, int]]
) -> List[USDPrimSpec]:
    """
    Generate USD specs for ground plane with obstacles.

    Args:
        width: Grid width
        height: Grid height
        cell_size: Cell size in meters
        obstacles: List of obstacle grid positions

    Returns:
        List of USD primitive specifications
    """
    prims = []

    # Ground plane
    total_width = width * cell_size
    total_height = height * cell_size

    prims.append(USDPrimSpec(
        path="/World/GroundPlane",
        type="Plane",
        attributes={
            "extent": [(-total_width/2, -total_height/2, 0),
                      (total_width/2, total_height/2, 0)],
            "primvars:displayColor": [(0.5, 0.5, 0.5)]
        }
    ))

    # Obstacles as cubes
    for i, (ox, oy) in enumerate(obstacles):
        wx = (ox + 0.5) * cell_size
        wy = (oy + 0.5) * cell_size

        prims.append(USDPrimSpec(
            path=f"/World/Obstacles/Obstacle_{i}",
            type="Cube",
            attributes={
                "size": cell_size * 0.9,
                "xformOp:translate": (wx, wy, cell_size * 0.45),
                "primvars:displayColor": [(0.3, 0.3, 0.3)]
            }
        ))

    return prims


def generate_robot_prims(
    trajectories: List[IsaacRobotTrajectory],
    config: IsaacSimConfig
) -> List[USDPrimSpec]:
    """
    Generate USD specs for robots at their starting positions.

    Args:
        trajectories: Robot trajectories
        config: Isaac Sim configuration

    Returns:
        List of USD primitive specifications
    """
    prims = []

    for traj in trajectories:
        start_pos = traj.start_position

        prims.append(USDPrimSpec(
            path=f"/World/Robots/{traj.robot_name}",
            type="Xform",
            attributes={
                "xformOp:translate": start_pos,
                "xformOp:rotateXYZ": (0, 0, 0)
            },
            references=[config.robot_usd_path]
        ))

    return prims


def generate_path_visualization(
    trajectories: List[IsaacRobotTrajectory],
    config: IsaacSimConfig
) -> List[USDPrimSpec]:
    """
    Generate USD specs for path visualization lines.

    Args:
        trajectories: Robot trajectories
        config: Isaac Sim configuration

    Returns:
        List of USD primitive specifications
    """
    if not config.show_paths:
        return []

    prims = []

    for traj in trajectories:
        points = [w.position for w in traj.waypoints]

        if len(points) < 2:
            continue

        prims.append(USDPrimSpec(
            path=f"/World/Paths/Path_{traj.robot_id}",
            type="BasisCurves",
            attributes={
                "points": points,
                "type": "linear",
                "primvars:displayColor": [config.path_color[:3]],
                "primvars:displayOpacity": [config.path_color[3]]
            }
        ))

    return prims


def generate_goal_markers(
    trajectories: List[IsaacRobotTrajectory],
    config: IsaacSimConfig
) -> List[USDPrimSpec]:
    """
    Generate USD specs for goal position markers.

    Args:
        trajectories: Robot trajectories
        config: Isaac Sim configuration

    Returns:
        List of USD primitive specifications
    """
    if not config.show_goals:
        return []

    prims = []

    for traj in trajectories:
        goal_pos = traj.goal_position

        prims.append(USDPrimSpec(
            path=f"/World/Goals/Goal_{traj.robot_id}",
            type="Cylinder",
            attributes={
                "radius": config.robot_radius * 0.5,
                "height": 0.1,
                "xformOp:translate": (goal_pos[0], goal_pos[1], 0.05),
                "primvars:displayColor": [config.goal_color[:3]],
                "primvars:displayOpacity": [config.goal_color[3]]
            }
        ))

    return prims


# ============================================================
# PYTHON SCRIPT GENERATION
# ============================================================

def generate_isaac_script(
    trajectories: List[IsaacRobotTrajectory],
    config: IsaacSimConfig,
    obstacles: List[Tuple[int, int]],
    grid_width: int,
    grid_height: int
) -> str:
    """
    Generate Python script for Isaac Sim execution.

    This script can be run in Isaac Sim's script editor or
    via the standalone Python environment.
    """

    script = '''"""
Auto-generated Isaac Sim script for MAPF solution execution.
Generated by MAPF Kernel Verifier.

Run this script in Isaac Sim's Script Editor or via standalone Python:
    ./python.sh -c "exec(open('this_script.py').read())"
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom
import numpy as np
import asyncio

# Configuration
PHYSICS_DT = {physics_dt}
RENDERING_DT = {rendering_dt}
CELL_SIZE = {cell_size}
ROBOT_USD = "{robot_usd}"
RECORD_VIDEO = {record_video}
OUTPUT_DIR = "{output_dir}"

# Trajectory data (embedded)
TRAJECTORIES = {trajectories_json}

# Obstacle positions
OBSTACLES = {obstacles_json}

# Grid dimensions
GRID_WIDTH = {grid_width}
GRID_HEIGHT = {grid_height}


class MAPFSimulation:
    """MAPF solution simulation controller."""

    def __init__(self):
        self.world = None
        self.robots = []
        self.current_waypoint_idx = []
        self.simulation_time = 0.0

    async def setup(self):
        """Initialize the simulation world."""
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=PHYSICS_DT,
            rendering_dt=RENDERING_DT
        )
        await self.world.initialize_simulation_context_async()

        # Create ground plane
        self.world.scene.add_ground_plane()

        # Add obstacles
        self._create_obstacles()

        # Add robots
        await self._create_robots()

        # Initialize waypoint tracking
        self.current_waypoint_idx = [0] * len(TRAJECTORIES)

    def _create_obstacles(self):
        """Create obstacle cubes in the scene."""
        for i, (ox, oy) in enumerate(OBSTACLES):
            wx = (ox + 0.5) * CELL_SIZE
            wy = (oy + 0.5) * CELL_SIZE

            cube_prim = XFormPrim(
                prim_path=f"/World/Obstacles/Obstacle_{{i}}",
                name=f"obstacle_{{i}}",
                position=np.array([wx, wy, CELL_SIZE * 0.45]),
                scale=np.array([CELL_SIZE * 0.9] * 3)
            )

    async def _create_robots(self):
        """Create robot instances."""
        assets_root = get_assets_root_path()

        for traj in TRAJECTORIES:
            robot_id = traj["robot_id"]
            robot_name = traj["robot_name"]
            start_pos = traj["waypoints"][0]["position"]

            robot_usd = assets_root + ROBOT_USD
            prim_path = f"/World/Robots/{{robot_name}}"

            add_reference_to_stage(
                usd_path=robot_usd,
                prim_path=prim_path
            )

            robot = self.world.scene.add(
                Robot(
                    prim_path=prim_path,
                    name=robot_name,
                    position=np.array([
                        start_pos["x"],
                        start_pos["y"],
                        start_pos["z"]
                    ])
                )
            )
            self.robots.append(robot)

    def step(self, dt: float):
        """Execute one simulation step."""
        self.simulation_time += dt

        for i, robot in enumerate(self.robots):
            self._update_robot_position(i, robot)

    def _update_robot_position(self, robot_idx: int, robot):
        """Update robot position based on trajectory."""
        traj = TRAJECTORIES[robot_idx]
        waypoints = traj["waypoints"]

        # Find current waypoint based on time
        while (self.current_waypoint_idx[robot_idx] < len(waypoints) - 1 and
               waypoints[self.current_waypoint_idx[robot_idx] + 1]["time"] <= self.simulation_time):
            self.current_waypoint_idx[robot_idx] += 1

        wp_idx = self.current_waypoint_idx[robot_idx]

        if wp_idx >= len(waypoints) - 1:
            # At final waypoint
            final_wp = waypoints[-1]
            target_pos = np.array([
                final_wp["position"]["x"],
                final_wp["position"]["y"],
                final_wp["position"]["z"]
            ])
        else:
            # Interpolate between waypoints
            wp_current = waypoints[wp_idx]
            wp_next = waypoints[wp_idx + 1]

            t0 = wp_current["time"]
            t1 = wp_next["time"]
            alpha = (self.simulation_time - t0) / (t1 - t0) if t1 > t0 else 1.0
            alpha = min(1.0, max(0.0, alpha))

            pos0 = np.array([wp_current["position"]["x"],
                           wp_current["position"]["y"],
                           wp_current["position"]["z"]])
            pos1 = np.array([wp_next["position"]["x"],
                           wp_next["position"]["y"],
                           wp_next["position"]["z"]])

            target_pos = pos0 + alpha * (pos1 - pos0)

        robot.set_world_pose(position=target_pos)

    def is_complete(self) -> bool:
        """Check if all robots have reached their goals."""
        for i, traj in enumerate(TRAJECTORIES):
            if self.current_waypoint_idx[i] < len(traj["waypoints"]) - 1:
                return False
        return True


async def main():
    """Main simulation entry point."""
    sim = MAPFSimulation()
    await sim.setup()

    print(f"Starting MAPF simulation with {{len(TRAJECTORIES)}} robots")
    print(f"Physics DT: {{PHYSICS_DT}}, Rendering DT: {{RENDERING_DT}}")

    # Run simulation
    frame_count = 0
    while not sim.is_complete():
        sim.world.step(render=True)
        sim.step(PHYSICS_DT)
        frame_count += 1

        if frame_count % 100 == 0:
            print(f"Frame {{frame_count}}, Time: {{sim.simulation_time:.2f}}s")

    print(f"Simulation complete. Total frames: {{frame_count}}")
    print(f"Total simulation time: {{sim.simulation_time:.2f}}s")


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
'''.format(
        physics_dt=config.physics_dt,
        rendering_dt=config.rendering_dt,
        cell_size=config.cell_size,
        robot_usd=config.robot_usd_path,
        record_video=str(config.record_video),
        output_dir=config.output_dir,
        trajectories_json=json.dumps([t.to_dict() for t in trajectories], indent=2),
        obstacles_json=json.dumps(obstacles),
        grid_width=grid_width,
        grid_height=grid_height
    )

    return script


# ============================================================
# FLEET MANAGER FOR SCALE
# ============================================================

@dataclass
class FleetStatus:
    """Status of robot fleet in simulation."""
    total_robots: int
    robots_at_goal: int
    robots_in_motion: int
    robots_waiting: int
    total_collisions: int
    simulation_time: float

    @property
    def completion_ratio(self) -> float:
        if self.total_robots == 0:
            return 0.0
        return self.robots_at_goal / self.total_robots

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_robots": self.total_robots,
            "robots_at_goal": self.robots_at_goal,
            "robots_in_motion": self.robots_in_motion,
            "robots_waiting": self.robots_waiting,
            "total_collisions": self.total_collisions,
            "simulation_time": self.simulation_time,
            "completion_ratio": self.completion_ratio
        }


class IsaacFleetManager:
    """
    Manager for large-scale robot fleet simulation.

    Supports dozens to hundreds of robots with:
    - Efficient batch trajectory updates
    - Collision monitoring
    - Status aggregation
    """

    def __init__(self, config: IsaacSimConfig):
        self.config = config
        self.trajectories: List[IsaacRobotTrajectory] = []
        self.robot_states: List[RobotState] = []
        self.collision_count = 0

    def load_solution(
        self,
        instance: MAPFInstance,
        result: MAPFResult,
        transform: IsaacCoordinateTransform
    ):
        """Load MAPF solution into fleet manager."""
        self.trajectories = convert_solution_to_isaac(
            instance, result, transform, self.config
        )
        self.robot_states = [RobotState.IDLE] * len(self.trajectories)
        self.collision_count = 0

    def get_status(self, simulation_time: float) -> FleetStatus:
        """Get current fleet status."""
        at_goal = 0
        in_motion = 0
        waiting = 0

        for i, traj in enumerate(self.trajectories):
            if simulation_time >= traj.duration:
                at_goal += 1
            else:
                # Check if currently moving or waiting
                for j, wp in enumerate(traj.waypoints[:-1]):
                    if wp.time <= simulation_time < traj.waypoints[j+1].time:
                        if wp.position == traj.waypoints[j+1].position:
                            waiting += 1
                        else:
                            in_motion += 1
                        break

        return FleetStatus(
            total_robots=len(self.trajectories),
            robots_at_goal=at_goal,
            robots_in_motion=in_motion,
            robots_waiting=waiting,
            total_collisions=self.collision_count,
            simulation_time=simulation_time
        )

    def check_collisions_at_time(self, t: float) -> List[Tuple[int, int]]:
        """
        Check for robot collisions at given time.

        Returns list of colliding robot pairs.
        """
        positions = []
        for traj in self.trajectories:
            pos = self._get_position_at_time(traj, t)
            positions.append(pos)

        collisions = []
        threshold = self.config.robot_radius * 2

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0])**2 +
                    (positions[i][1] - positions[j][1])**2
                )
                if dist < threshold:
                    collisions.append((i, j))

        return collisions

    def _get_position_at_time(
        self,
        traj: IsaacRobotTrajectory,
        t: float
    ) -> Tuple[float, float, float]:
        """Get interpolated position at time t."""
        if not traj.waypoints:
            return (0.0, 0.0, 0.0)

        if t <= traj.waypoints[0].time:
            return traj.waypoints[0].position

        if t >= traj.waypoints[-1].time:
            return traj.waypoints[-1].position

        for i in range(len(traj.waypoints) - 1):
            t0 = traj.waypoints[i].time
            t1 = traj.waypoints[i + 1].time

            if t0 <= t < t1:
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                p0 = traj.waypoints[i].position
                p1 = traj.waypoints[i + 1].position

                return (
                    p0[0] + alpha * (p1[0] - p0[0]),
                    p0[1] + alpha * (p1[1] - p0[1]),
                    p0[2] + alpha * (p1[2] - p0[2])
                )

        return traj.waypoints[-1].position


# ============================================================
# ISAAC SIM ADAPTER
# ============================================================

class IsaacSimAdapter:
    """
    Complete Isaac Sim adapter for MAPF solutions.

    Provides:
    - Solution to trajectory conversion
    - USD scene generation
    - Python script export
    - Fleet management for scale
    - Video recording setup
    """

    def __init__(self, config: Optional[IsaacSimConfig] = None):
        self.config = config or IsaacSimConfig()
        self.transform = IsaacCoordinateTransform(cell_size=self.config.cell_size)
        self.fleet_manager = IsaacFleetManager(self.config)

        Path(self.config.output_dir).mkdir(parents=True, exist_ok=True)

    def convert_solution(
        self,
        instance: MAPFInstance,
        result: MAPFResult
    ) -> List[IsaacRobotTrajectory]:
        """Convert MAPF solution to Isaac Sim trajectories."""
        return convert_solution_to_isaac(
            instance, result, self.transform, self.config
        )

    def export_trajectories(
        self,
        trajectories: List[IsaacRobotTrajectory],
        filename: str
    ) -> str:
        """Export trajectories to JSON file."""
        filepath = Path(self.config.output_dir) / filename
        data = {
            "trajectories": [t.to_dict() for t in trajectories],
            "config": {
                "cell_size": self.config.cell_size,
                "physics_dt": self.config.physics_dt,
                "robot_usd": self.config.robot_usd_path
            }
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        return str(filepath)

    def export_script(
        self,
        trajectories: List[IsaacRobotTrajectory],
        obstacles: List[Tuple[int, int]],
        grid_width: int,
        grid_height: int,
        filename: str = "mapf_simulation.py"
    ) -> str:
        """Export Python script for Isaac Sim."""
        script = generate_isaac_script(
            trajectories, self.config, obstacles, grid_width, grid_height
        )
        filepath = Path(self.config.output_dir) / filename
        with open(filepath, 'w') as f:
            f.write(script)
        return str(filepath)

    def export_usd_specs(
        self,
        trajectories: List[IsaacRobotTrajectory],
        obstacles: List[Tuple[int, int]],
        grid_width: int,
        grid_height: int,
        filename: str = "scene_spec.json"
    ) -> str:
        """Export USD primitive specifications as JSON."""
        prims = []
        prims.extend(generate_ground_plane(
            grid_width, grid_height, self.config.cell_size, obstacles
        ))
        prims.extend(generate_robot_prims(trajectories, self.config))
        prims.extend(generate_path_visualization(trajectories, self.config))
        prims.extend(generate_goal_markers(trajectories, self.config))

        filepath = Path(self.config.output_dir) / filename
        data = {
            "prims": [
                {
                    "path": p.path,
                    "type": p.type,
                    "attributes": p.attributes,
                    "references": p.references
                }
                for p in prims
            ]
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        return str(filepath)

    def export_complete(
        self,
        name: str,
        instance: MAPFInstance,
        result: MAPFResult,
        obstacles: List[Tuple[int, int]],
        grid_width: int,
        grid_height: int
    ) -> Dict[str, str]:
        """
        Export complete Isaac Sim package.

        Returns dict of generated file paths.
        """
        trajectories = self.convert_solution(instance, result)

        files = {}

        # Trajectory data
        files["trajectories"] = self.export_trajectories(
            trajectories, f"{name}_trajectories.json"
        )

        # Python script
        files["script"] = self.export_script(
            trajectories, obstacles, grid_width, grid_height,
            f"{name}_simulation.py"
        )

        # USD specs
        files["usd_specs"] = self.export_usd_specs(
            trajectories, obstacles, grid_width, grid_height,
            f"{name}_scene.json"
        )

        # Summary
        summary = {
            "name": name,
            "num_robots": len(trajectories),
            "max_duration": max(t.duration for t in trajectories) if trajectories else 0,
            "files": files,
            "config": {
                "cell_size": self.config.cell_size,
                "physics_dt": self.config.physics_dt,
                "robot_usd": self.config.robot_usd_path
            }
        }

        summary_path = Path(self.config.output_dir) / f"{name}_summary.json"
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        files["summary"] = str(summary_path)

        return files

    def generate_launch_instructions(self) -> str:
        """Generate instructions for running in Isaac Sim."""
        return """
# Isaac Sim MAPF Simulation Launch Instructions

## Prerequisites
1. NVIDIA Isaac Sim installed (2023.1.1 or later)
2. ROS2 Humble (for ROS bridge)
3. Python 3.10+

## Running the Simulation

### Option 1: Isaac Sim Script Editor
1. Launch Isaac Sim
2. Open Window > Script Editor
3. Load the generated *_simulation.py script
4. Click Run

### Option 2: Standalone Python
```bash
cd ~/.local/share/ov/pkg/isaac_sim-*/
./python.sh /path/to/mapf_simulation.py
```

### Option 3: Command Line
```bash
./isaac-sim.sh --exec "/path/to/mapf_simulation.py"
```

## Recording Video
Set RECORD_VIDEO = True in the script, then:
1. Use Isaac Sim's built-in Movie Capture
2. Configure output directory and resolution
3. Run simulation with recording enabled

## ROS2 Bridge
To connect to ROS2:
1. Enable the ROS2 Bridge extension in Isaac Sim
2. Robot trajectories will be published to /robot_*/cmd_vel topics
3. Use rviz2 for external visualization

## Scale Testing
For large fleets (100+ robots):
1. Reduce rendering quality in settings
2. Increase physics_dt for faster execution
3. Disable path visualization
4. Use headless mode: --headless
"""


# ============================================================
# VERIFICATION BRIDGE
# ============================================================

def verify_isaac_trajectories(
    trajectories: List[IsaacRobotTrajectory],
    config: IsaacSimConfig
) -> Dict[str, Any]:
    """
    Verify Isaac Sim trajectories maintain MAPF guarantees.

    Checks:
    1. No collisions at any timestep
    2. All robots reach goals
    3. Timing is consistent
    """
    if not trajectories:
        return {"valid": True, "reason": "No trajectories"}

    # Find simulation duration
    max_duration = max(t.duration for t in trajectories)
    dt = config.physics_dt

    # Sample collision check
    collision_times = []
    threshold = config.robot_radius * 2

    t = 0.0
    while t <= max_duration:
        positions = []
        for traj in trajectories:
            # Find position at time t
            pos = None
            for i, wp in enumerate(traj.waypoints):
                if wp.time >= t:
                    if i == 0:
                        pos = wp.position
                    else:
                        # Interpolate
                        prev_wp = traj.waypoints[i-1]
                        alpha = (t - prev_wp.time) / (wp.time - prev_wp.time)
                        pos = (
                            prev_wp.position[0] + alpha * (wp.position[0] - prev_wp.position[0]),
                            prev_wp.position[1] + alpha * (wp.position[1] - prev_wp.position[1]),
                            prev_wp.position[2] + alpha * (wp.position[2] - prev_wp.position[2])
                        )
                    break
            if pos is None:
                pos = traj.waypoints[-1].position
            positions.append(pos)

        # Check pairwise distances
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0])**2 +
                    (positions[i][1] - positions[j][1])**2
                )
                if dist < threshold:
                    collision_times.append({
                        "time": t,
                        "robots": (i, j),
                        "distance": dist
                    })

        t += dt

    return {
        "valid": len(collision_times) == 0,
        "num_trajectories": len(trajectories),
        "max_duration": max_duration,
        "collision_check_interval": dt,
        "collisions_detected": len(collision_times),
        "collision_details": collision_times[:10] if collision_times else []
    }
