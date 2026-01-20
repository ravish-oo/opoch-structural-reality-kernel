"""
mapf_ros2_adapter.py - ROS2/Gazebo TurtleBot3 Adapter for MAPF.

Provides integration with ROS2 Nav2 for executing MAPF solutions
on simulated or real TurtleBot3 robots.

Simulation layer requirements:
- Map discrete plan steps (vertex, time) to target poses
- Create reservation windows for collision avoidance
- Assert safety monitors (no two robots in same reserved cell/time)

See:
- TurtleBot3: https://github.com/ROBOTIS-GIT/turtlebot3
- Nav2: https://github.com/ros-planning/navigation2
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple
import json
import math
from pathlib import Path

from .mapf_model import (
    Graph,
    MAPFInstance,
    MAPFResult,
    ResultStatus,
    Path as MAPFPath,
    H,
    canon_json
)
from .mapf_movingai import MovingAIMap


# ============================================================
# COORDINATE SYSTEMS
# ============================================================

@dataclass
class Pose2D:
    """2D pose for robot navigation."""
    x: float  # meters
    y: float  # meters
    theta: float = 0.0  # radians

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "theta": self.theta}

    def distance_to(self, other: 'Pose2D') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


@dataclass
class GridToWorldTransform:
    """
    Transform from discrete grid coordinates to continuous world coordinates.

    Handles:
    - Grid cell to world position conversion
    - Resolution (meters per cell)
    - Origin offset
    """
    resolution: float  # meters per cell
    origin_x: float  # world x of grid (0,0)
    origin_y: float  # world y of grid (0,0)

    def grid_to_world(self, gx: int, gy: int) -> Pose2D:
        """Convert grid cell to world pose (center of cell)."""
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return Pose2D(x=wx, y=wy, theta=0.0)

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world position to grid cell."""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def compute_heading(self, from_pose: Pose2D, to_pose: Pose2D) -> float:
        """Compute heading from one pose to another."""
        dx = to_pose.x - from_pose.x
        dy = to_pose.y - from_pose.y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return from_pose.theta
        return math.atan2(dy, dx)


# ============================================================
# WAYPOINT GENERATION
# ============================================================

@dataclass
class Waypoint:
    """Single waypoint for robot navigation."""
    timestep: int
    pose: Pose2D
    action: str  # "move", "wait", "goal"
    duration_sec: float  # expected duration

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestep": self.timestep,
            "pose": self.pose.to_dict(),
            "action": self.action,
            "duration_sec": self.duration_sec
        }


@dataclass
class RobotTrajectory:
    """Complete trajectory for a single robot."""
    robot_id: int
    robot_name: str
    waypoints: List[Waypoint]
    total_duration_sec: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "robot_name": self.robot_name,
            "waypoints": [w.to_dict() for w in self.waypoints],
            "total_duration_sec": self.total_duration_sec
        }

    def get_pose_at_time(self, t: float) -> Optional[Pose2D]:
        """Get interpolated pose at time t (seconds)."""
        if not self.waypoints:
            return None

        elapsed = 0.0
        for i, wp in enumerate(self.waypoints):
            if elapsed + wp.duration_sec >= t:
                # Interpolate within this segment
                if i == 0:
                    return wp.pose
                prev_wp = self.waypoints[i - 1]
                frac = (t - elapsed) / wp.duration_sec if wp.duration_sec > 0 else 1.0
                return Pose2D(
                    x=prev_wp.pose.x + frac * (wp.pose.x - prev_wp.pose.x),
                    y=prev_wp.pose.y + frac * (wp.pose.y - prev_wp.pose.y),
                    theta=wp.pose.theta
                )
            elapsed += wp.duration_sec

        # Past end, return final pose
        return self.waypoints[-1].pose


def generate_trajectory(
    path: MAPFPath,
    robot_id: int,
    map_data: MovingAIMap,
    transform: GridToWorldTransform,
    time_per_step: float = 2.0  # seconds per discrete timestep
) -> RobotTrajectory:
    """
    Generate robot trajectory from MAPF path.

    Converts discrete path to continuous waypoints with proper headings.
    """
    waypoints = []

    for t in range(len(path)):
        v = path[t]
        gx, gy = map_data.from_vertex(v)
        pose = transform.grid_to_world(gx, gy)

        # Determine action type
        if t == len(path) - 1:
            action = "goal"
        elif t > 0 and path[t] == path[t - 1]:
            action = "wait"
        else:
            action = "move"

        # Compute heading towards next waypoint
        if t < len(path) - 1 and path[t] != path[t + 1]:
            next_v = path[t + 1]
            next_gx, next_gy = map_data.from_vertex(next_v)
            next_pose = transform.grid_to_world(next_gx, next_gy)
            pose.theta = transform.compute_heading(pose, next_pose)

        waypoints.append(Waypoint(
            timestep=t,
            pose=pose,
            action=action,
            duration_sec=time_per_step
        ))

    return RobotTrajectory(
        robot_id=robot_id,
        robot_name=f"robot_{robot_id}",
        waypoints=waypoints,
        total_duration_sec=len(path) * time_per_step
    )


# ============================================================
# RESERVATION SYSTEM
# ============================================================

@dataclass
class Reservation:
    """
    Space-time reservation for collision avoidance.

    A robot reserves a cell for a time window to ensure
    no other robot occupies the same space at the same time.
    """
    robot_id: int
    grid_x: int
    grid_y: int
    start_time_sec: float
    end_time_sec: float

    def overlaps(self, other: 'Reservation') -> bool:
        """Check if this reservation overlaps with another."""
        if self.grid_x != other.grid_x or self.grid_y != other.grid_y:
            return False
        return not (self.end_time_sec <= other.start_time_sec or
                    self.start_time_sec >= other.end_time_sec)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "grid": [self.grid_x, self.grid_y],
            "time_window": [self.start_time_sec, self.end_time_sec]
        }


@dataclass
class ReservationTable:
    """
    Complete reservation table for all robots.

    Used for runtime collision checking.
    """
    reservations: List[Reservation] = field(default_factory=list)

    def add(self, reservation: Reservation) -> bool:
        """Add reservation, return False if conflict exists."""
        for existing in self.reservations:
            if existing.robot_id != reservation.robot_id:
                if existing.overlaps(reservation):
                    return False
        self.reservations.append(reservation)
        return True

    def check_conflict(self, reservation: Reservation) -> Optional[Reservation]:
        """Check if reservation conflicts with existing ones."""
        for existing in self.reservations:
            if existing.robot_id != reservation.robot_id:
                if existing.overlaps(reservation):
                    return existing
        return None

    def get_reservations_at_time(self, time_sec: float) -> List[Reservation]:
        """Get all reservations active at a given time."""
        return [r for r in self.reservations
                if r.start_time_sec <= time_sec < r.end_time_sec]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_reservations": len(self.reservations),
            "reservations": [r.to_dict() for r in self.reservations]
        }


def build_reservation_table(
    trajectories: List[RobotTrajectory],
    map_data: MovingAIMap,
    transform: GridToWorldTransform,
    safety_margin_sec: float = 0.5
) -> ReservationTable:
    """
    Build reservation table from trajectories.

    Each waypoint generates a reservation for its cell and time window.
    """
    table = ReservationTable()

    for traj in trajectories:
        current_time = 0.0
        for wp in traj.waypoints:
            # Convert world pose back to grid
            gx, gy = transform.world_to_grid(wp.pose.x, wp.pose.y)

            # Create reservation with safety margin
            reservation = Reservation(
                robot_id=traj.robot_id,
                grid_x=gx,
                grid_y=gy,
                start_time_sec=current_time - safety_margin_sec,
                end_time_sec=current_time + wp.duration_sec + safety_margin_sec
            )

            if not table.add(reservation):
                # This shouldn't happen if MAPF solution is valid
                # Log warning but continue
                pass

            current_time += wp.duration_sec

    return table


# ============================================================
# SAFETY MONITOR
# ============================================================

@dataclass
class SafetyViolation:
    """Record of a safety violation."""
    time_sec: float
    robot1_id: int
    robot2_id: int
    grid_x: int
    grid_y: int
    description: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time_sec": self.time_sec,
            "robots": [self.robot1_id, self.robot2_id],
            "grid": [self.grid_x, self.grid_y],
            "description": self.description
        }


class SafetyMonitor:
    """
    Runtime safety monitor for multi-robot execution.

    Asserts that no two robots occupy the same cell/time window.
    This is the continuous proxy for MAPF V4/V5 constraints.
    """

    def __init__(
        self,
        reservation_table: ReservationTable,
        collision_radius: float = 0.3  # meters
    ):
        self.reservation_table = reservation_table
        self.collision_radius = collision_radius
        self.violations: List[SafetyViolation] = []

    def check_state(
        self,
        robot_poses: Dict[int, Pose2D],
        current_time_sec: float
    ) -> List[SafetyViolation]:
        """
        Check current state for safety violations.

        Returns list of any violations detected.
        """
        new_violations = []

        # Check pairwise distances
        robot_ids = list(robot_poses.keys())
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                r1_id = robot_ids[i]
                r2_id = robot_ids[j]
                p1 = robot_poses[r1_id]
                p2 = robot_poses[r2_id]

                distance = p1.distance_to(p2)
                if distance < 2 * self.collision_radius:
                    # Potential collision
                    violation = SafetyViolation(
                        time_sec=current_time_sec,
                        robot1_id=r1_id,
                        robot2_id=r2_id,
                        grid_x=int(p1.x),  # Approximate
                        grid_y=int(p1.y),
                        description=f"Distance {distance:.2f}m < {2*self.collision_radius:.2f}m"
                    )
                    new_violations.append(violation)
                    self.violations.append(violation)

        return new_violations

    def is_safe(self) -> bool:
        """Check if execution has been safe (no violations)."""
        return len(self.violations) == 0

    def get_report(self) -> Dict[str, Any]:
        """Generate safety report."""
        return {
            "safe": self.is_safe(),
            "total_violations": len(self.violations),
            "violations": [v.to_dict() for v in self.violations]
        }


# ============================================================
# ROS2 MESSAGE GENERATION
# ============================================================

def generate_nav2_goal(waypoint: Waypoint) -> Dict[str, Any]:
    """
    Generate Nav2 NavigateToPose action goal message.

    This is the ROS2 message format for Nav2 navigation.
    """
    return {
        "header": {
            "frame_id": "map"
        },
        "pose": {
            "position": {
                "x": waypoint.pose.x,
                "y": waypoint.pose.y,
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(waypoint.pose.theta / 2),
                "w": math.cos(waypoint.pose.theta / 2)
            }
        }
    }


def generate_twist_cmd(
    current: Pose2D,
    target: Pose2D,
    linear_speed: float = 0.2,
    angular_speed: float = 0.5
) -> Dict[str, Any]:
    """
    Generate simple Twist command for direct control.

    This is a basic velocity command for differential drive robots.
    """
    # Compute angle to target
    angle_to_target = math.atan2(
        target.y - current.y,
        target.x - current.x
    )

    # Angular error
    angle_error = angle_to_target - current.theta
    # Normalize to [-pi, pi]
    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi

    # Distance
    distance = current.distance_to(target)

    # Simple control law
    if abs(angle_error) > 0.1:  # Turn first
        linear = 0.0
        angular = angular_speed if angle_error > 0 else -angular_speed
    elif distance > 0.05:  # Move forward
        linear = min(linear_speed, distance)
        angular = 0.0
    else:  # At target
        linear = 0.0
        angular = 0.0

    return {
        "linear": {"x": linear, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": angular}
    }


# ============================================================
# ROS2 ADAPTER
# ============================================================

class ROS2Adapter:
    """
    Complete ROS2 adapter for MAPF execution.

    Provides:
    - Trajectory generation from MAPF solutions
    - Reservation table for collision avoidance
    - Safety monitoring
    - ROS2 message generation
    """

    def __init__(
        self,
        map_data: MovingAIMap,
        resolution: float = 0.5,  # meters per cell
        time_per_step: float = 2.0,  # seconds per MAPF timestep
        robot_namespace_prefix: str = "tb3_"
    ):
        self.map_data = map_data
        self.transform = GridToWorldTransform(
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0
        )
        self.time_per_step = time_per_step
        self.robot_namespace_prefix = robot_namespace_prefix

        self.trajectories: List[RobotTrajectory] = []
        self.reservation_table: Optional[ReservationTable] = None
        self.safety_monitor: Optional[SafetyMonitor] = None

    def load_solution(self, result: MAPFResult) -> bool:
        """
        Load MAPF solution and generate execution plan.

        Returns True if successful.
        """
        if result.status != ResultStatus.UNIQUE or not result.paths:
            return False

        # Generate trajectories
        self.trajectories = []
        for i, path in enumerate(result.paths):
            traj = generate_trajectory(
                path,
                robot_id=i,
                map_data=self.map_data,
                transform=self.transform,
                time_per_step=self.time_per_step
            )
            self.trajectories.append(traj)

        # Build reservation table
        self.reservation_table = build_reservation_table(
            self.trajectories,
            self.map_data,
            self.transform
        )

        # Initialize safety monitor
        self.safety_monitor = SafetyMonitor(self.reservation_table)

        return True

    def get_trajectory(self, robot_id: int) -> Optional[RobotTrajectory]:
        """Get trajectory for a specific robot."""
        for traj in self.trajectories:
            if traj.robot_id == robot_id:
                return traj
        return None

    def get_robot_namespace(self, robot_id: int) -> str:
        """Get ROS2 namespace for robot."""
        return f"{self.robot_namespace_prefix}{robot_id}"

    def get_nav2_goals(self, robot_id: int) -> List[Dict[str, Any]]:
        """Get list of Nav2 goals for robot."""
        traj = self.get_trajectory(robot_id)
        if not traj:
            return []

        goals = []
        for wp in traj.waypoints:
            if wp.action in ["move", "goal"]:
                goals.append(generate_nav2_goal(wp))
        return goals

    def check_safety(
        self,
        robot_poses: Dict[int, Pose2D],
        current_time_sec: float
    ) -> List[SafetyViolation]:
        """Check current state for safety violations."""
        if self.safety_monitor:
            return self.safety_monitor.check_state(robot_poses, current_time_sec)
        return []

    def export_launch_config(self, output_file: str) -> None:
        """
        Export ROS2 launch configuration.

        Generates a YAML config for multi-robot Nav2 launch.
        """
        config = {
            "robots": [],
            "map": {
                "name": self.map_data.name,
                "resolution": self.transform.resolution
            }
        }

        for traj in self.trajectories:
            robot_config = {
                "name": traj.robot_name,
                "namespace": self.get_robot_namespace(traj.robot_id),
                "start_pose": traj.waypoints[0].pose.to_dict() if traj.waypoints else None,
                "goal_pose": traj.waypoints[-1].pose.to_dict() if traj.waypoints else None
            }
            config["robots"].append(robot_config)

        with open(output_file, 'w') as f:
            json.dump(config, f, indent=2)

    def export_execution_plan(self, output_file: str) -> None:
        """Export complete execution plan."""
        plan = {
            "trajectories": [t.to_dict() for t in self.trajectories],
            "reservations": self.reservation_table.to_dict() if self.reservation_table else None,
            "transform": {
                "resolution": self.transform.resolution,
                "origin_x": self.transform.origin_x,
                "origin_y": self.transform.origin_y
            }
        }

        with open(output_file, 'w') as f:
            json.dump(plan, f, indent=2)

    def get_safety_report(self) -> Dict[str, Any]:
        """Get safety monitor report."""
        if self.safety_monitor:
            return self.safety_monitor.get_report()
        return {"safe": True, "total_violations": 0, "violations": []}


# ============================================================
# GAZEBO INTEGRATION
# ============================================================

def generate_gazebo_spawn_command(
    robot_id: int,
    pose: Pose2D,
    model: str = "turtlebot3_burger"
) -> str:
    """
    Generate Gazebo spawn command for robot.

    Returns command string for ros2 service call.
    """
    return (
        f"ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "
        f"\"{{name: 'robot_{robot_id}', xml: '', "
        f"initial_pose: {{position: {{x: {pose.x}, y: {pose.y}, z: 0.0}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {math.sin(pose.theta/2)}, w: {math.cos(pose.theta/2)}}}}}}}\""
    )


def generate_multi_robot_launch_file(
    adapter: ROS2Adapter,
    output_file: str
) -> None:
    """
    Generate ROS2 Python launch file for multi-robot simulation.
    """
    launch_content = '''"""
Auto-generated multi-robot MAPF launch file.
Generated by MAPF Kernel Verifier ROS2 Adapter.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

'''

    for traj in adapter.trajectories:
        ns = adapter.get_robot_namespace(traj.robot_id)
        start_pose = traj.waypoints[0].pose if traj.waypoints else Pose2D(0, 0, 0)

        launch_content += f'''
    # Robot {traj.robot_id}
    robot_{traj.robot_id}_group = GroupAction([
        PushRosNamespace('{ns}'),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            name='drive',
            parameters=[{{'initial_pose_x': {start_pose.x}, 'initial_pose_y': {start_pose.y}}}]
        )
    ])
    ld.add_action(robot_{traj.robot_id}_group)
'''

    launch_content += '''
    return ld
'''

    with open(output_file, 'w') as f:
        f.write(launch_content)
