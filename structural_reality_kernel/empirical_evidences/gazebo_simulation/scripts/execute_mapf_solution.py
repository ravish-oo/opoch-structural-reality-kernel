#!/usr/bin/env python3
"""
MAPF Solution Executor for ROS2/Gazebo

Executes the pre-computed CBS solution by publishing velocity commands
to each TurtleBot3 robot to follow their planned paths.

This node:
1. Loads the MAPF solution (waypoints for each agent)
2. Synchronizes all robots at each timestep
3. Publishes cmd_vel commands for smooth motion
4. Monitors position feedback for accuracy

Usage:
    ros2 run mapf_simulation execute_mapf_solution.py
"""

import os
import sys
import json
import yaml
import math
import threading
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("ROS2 not available. Install ros-humble-desktop for full functionality.")


class ExecutionState(Enum):
    """Robot execution state."""
    IDLE = "idle"
    MOVING = "moving"
    WAITING = "waiting"
    ARRIVED = "arrived"
    ERROR = "error"


@dataclass
class RobotState:
    """Track individual robot state."""
    id: int
    name: str
    current_pos: Tuple[float, float] = (0.0, 0.0)
    current_yaw: float = 0.0
    target_pos: Tuple[float, float] = (0.0, 0.0)
    path: List[Tuple[int, int]] = field(default_factory=list)
    current_waypoint: int = 0
    state: ExecutionState = ExecutionState.IDLE
    at_goal: bool = False


if HAS_ROS2:

    class MAPFExecutor(Node):
        """
        ROS2 Node for executing MAPF solutions.

        Coordinates multiple robots to follow their collision-free paths
        with synchronized timestep execution.
        """

        def __init__(self):
            super().__init__('mapf_executor')

            # Declare parameters
            self.declare_parameter('solution_file', '')
            self.declare_parameter('robots_file', '')
            self.declare_parameter('time_per_step', 2.0)
            self.declare_parameter('position_tolerance', 0.1)
            self.declare_parameter('angle_tolerance', 0.15)
            self.declare_parameter('linear_speed', 0.15)
            self.declare_parameter('angular_speed', 0.5)

            # Get parameters
            self.time_per_step = self.get_parameter('time_per_step').value
            self.position_tolerance = self.get_parameter('position_tolerance').value
            self.angle_tolerance = self.get_parameter('angle_tolerance').value
            self.linear_speed = self.get_parameter('linear_speed').value
            self.angular_speed = self.get_parameter('angular_speed').value

            # Configuration
            config_dir = Path(__file__).parent.parent / "config"
            solution_file = self.get_parameter('solution_file').value
            if not solution_file:
                solution_file = str(config_dir / "mapf_solution.json")
            robots_file = self.get_parameter('robots_file').value
            if not robots_file:
                robots_file = str(config_dir / "robots.yaml")

            # Load configurations
            self.cell_size = 0.5
            self.robots = self._load_robots(robots_file)
            self._load_solution(solution_file)

            self.get_logger().info(f"Loaded {len(self.robots)} robots")

            # Publishers and subscribers
            self.cmd_pubs = {}
            self.odom_subs = {}

            for robot_id, robot in self.robots.items():
                # Velocity command publisher
                cmd_topic = f'/{robot.name}/cmd_vel'
                self.cmd_pubs[robot_id] = self.create_publisher(
                    Twist, cmd_topic, 10
                )

                # Odometry subscriber
                odom_topic = f'/{robot.name}/odom'
                self.odom_subs[robot_id] = self.create_subscription(
                    Odometry, odom_topic,
                    lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                    10
                )

            # Status publisher
            self.status_pub = self.create_publisher(
                String, '/mapf/execution_status', 10
            )

            # Control timer (10 Hz)
            self.control_timer = self.create_timer(0.1, self._control_loop)

            # State
            self.current_timestep = 0
            self.max_timestep = max(len(r.path) - 1 for r in self.robots.values())
            self.execution_started = False
            self.execution_complete = False
            self.lock = threading.Lock()

            self.get_logger().info(f"MAPF Executor initialized")
            self.get_logger().info(f"Makespan: {self.max_timestep} timesteps")
            self.get_logger().info(f"Time per step: {self.time_per_step}s")

            # Start execution after 3 seconds
            self.create_timer(3.0, self._start_execution, callback_group=None)

        def _load_robots(self, robots_file: str) -> Dict[int, RobotState]:
            """Load robot configurations."""
            with open(robots_file, 'r') as f:
                config = yaml.safe_load(f)

            robots = {}
            for r in config['robots']:
                robot = RobotState(
                    id=r['id'],
                    name=r['name'],
                    current_pos=(r['start']['world'][0], r['start']['world'][1]),
                    target_pos=(r['start']['world'][0], r['start']['world'][1])
                )
                robots[r['id']] = robot

            return robots

        def _load_solution(self, solution_file: str):
            """Load MAPF solution and assign paths."""
            with open(solution_file, 'r') as f:
                solution = json.load(f)

            for agent in solution['agents']:
                robot_id = agent['id']
                if robot_id in self.robots:
                    self.robots[robot_id].path = [tuple(p) for p in agent['path']]

        def _grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
            """Convert grid to world coordinates."""
            x = grid_pos[0] * self.cell_size + self.cell_size / 2
            y = grid_pos[1] * self.cell_size + self.cell_size / 2
            return (x, y)

        def _odom_callback(self, msg: Odometry, robot_id: int):
            """Handle odometry updates."""
            with self.lock:
                if robot_id not in self.robots:
                    return

                robot = self.robots[robot_id]
                robot.current_pos = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y
                )

                # Extract yaw from quaternion
                q = msg.pose.pose.orientation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                robot.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        def _start_execution(self):
            """Start MAPF execution."""
            if self.execution_started:
                return

            self.execution_started = True
            self.get_logger().info("Starting MAPF execution!")

            # Set initial targets
            self._advance_timestep()

        def _advance_timestep(self):
            """Advance to next timestep."""
            if self.current_timestep > self.max_timestep:
                self._complete_execution()
                return

            self.get_logger().info(f"Timestep {self.current_timestep}/{self.max_timestep}")

            with self.lock:
                for robot in self.robots.values():
                    waypoint_idx = min(self.current_timestep, len(robot.path) - 1)
                    grid_pos = robot.path[waypoint_idx]
                    robot.target_pos = self._grid_to_world(grid_pos)
                    robot.state = ExecutionState.MOVING

            # Publish status
            status_msg = String()
            status_msg.data = f"timestep:{self.current_timestep}/{self.max_timestep}"
            self.status_pub.publish(status_msg)

        def _control_loop(self):
            """Main control loop - runs at 10 Hz."""
            if not self.execution_started or self.execution_complete:
                return

            all_arrived = True

            with self.lock:
                for robot_id, robot in self.robots.items():
                    if robot.state == ExecutionState.MOVING:
                        # Calculate distance to target
                        dx = robot.target_pos[0] - robot.current_pos[0]
                        dy = robot.target_pos[1] - robot.current_pos[1]
                        distance = math.sqrt(dx*dx + dy*dy)

                        if distance < self.position_tolerance:
                            # Arrived at waypoint
                            robot.state = ExecutionState.WAITING
                            self._stop_robot(robot_id)
                        else:
                            # Calculate control commands
                            all_arrived = False
                            self._move_robot(robot_id, dx, dy, distance)

                    elif robot.state == ExecutionState.WAITING:
                        # Robot waiting for others
                        pass

            # Check if all robots arrived
            if all_arrived and self.execution_started:
                self.current_timestep += 1
                self._advance_timestep()

        def _move_robot(self, robot_id: int, dx: float, dy: float, distance: float):
            """Send velocity commands to move robot toward target."""
            robot = self.robots[robot_id]

            # Calculate target angle
            target_yaw = math.atan2(dy, dx)

            # Calculate angle error
            angle_error = target_yaw - robot.current_yaw
            # Normalize to [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            cmd = Twist()

            if abs(angle_error) > self.angle_tolerance:
                # Rotate first
                cmd.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
                cmd.linear.x = 0.0
            else:
                # Move forward
                cmd.linear.x = min(self.linear_speed, distance)
                cmd.angular.z = 0.5 * angle_error  # Small correction

            self.cmd_pubs[robot_id].publish(cmd)

        def _stop_robot(self, robot_id: int):
            """Stop robot motion."""
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pubs[robot_id].publish(cmd)

        def _complete_execution(self):
            """Handle execution completion."""
            if self.execution_complete:
                return

            self.execution_complete = True
            self.get_logger().info("="*40)
            self.get_logger().info("MAPF EXECUTION COMPLETE!")
            self.get_logger().info("="*40)

            # Stop all robots
            for robot_id in self.robots:
                self._stop_robot(robot_id)

            # Verify final positions
            self.get_logger().info("\nFinal Position Check:")
            all_at_goal = True
            for robot in self.robots.values():
                goal_grid = robot.path[-1]
                goal_world = self._grid_to_world(goal_grid)
                dist = math.sqrt(
                    (robot.current_pos[0] - goal_world[0])**2 +
                    (robot.current_pos[1] - goal_world[1])**2
                )
                status = "OK" if dist < self.position_tolerance * 2 else "ERROR"
                if status != "OK":
                    all_at_goal = False
                self.get_logger().info(
                    f"  {robot.name}: {robot.current_pos} -> {goal_world} "
                    f"(error: {dist:.3f}m) [{status}]"
                )

            if all_at_goal:
                self.get_logger().info("\nAll robots reached their goals!")
            else:
                self.get_logger().warn("\nSome robots may have position errors")

            # Publish completion status
            status_msg = String()
            status_msg.data = "complete"
            self.status_pub.publish(status_msg)


def main():
    if not HAS_ROS2:
        print("ERROR: ROS2 not available!")
        print("For standalone simulation, use: python3 standalone_simulation.py")
        sys.exit(1)

    rclpy.init()
    executor_node = MAPFExecutor()

    try:
        rclpy.spin(executor_node)
    except KeyboardInterrupt:
        executor_node.get_logger().info("Shutting down...")
    finally:
        # Stop all robots
        for robot_id in executor_node.robots:
            executor_node._stop_robot(robot_id)
        executor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
