#!/usr/bin/env python3
"""
MAPF Safety Monitor for ROS2/Gazebo

Real-time collision detection and safety monitoring for multi-robot MAPF execution.

This node:
1. Subscribes to odometry from all robots
2. Monitors pairwise distances in real-time
3. Detects potential collisions
4. Can trigger emergency stops if threshold violated
5. Logs all near-miss events

Usage:
    ros2 run mapf_simulation safety_monitor.py
"""

import os
import sys
import math
import yaml
import threading
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("ROS2 not available. This script requires ros-humble-desktop.")


@dataclass
class RobotTracking:
    """Track robot position and history."""
    id: int
    name: str
    position: Tuple[float, float] = (0.0, 0.0)
    velocity: Tuple[float, float] = (0.0, 0.0)
    last_update: float = 0.0
    position_history: deque = field(default_factory=lambda: deque(maxlen=100))


@dataclass
class CollisionEvent:
    """Record a collision or near-miss event."""
    timestamp: float
    robot1_id: int
    robot2_id: int
    distance: float
    position1: Tuple[float, float]
    position2: Tuple[float, float]
    severity: str  # "warning", "critical", "collision"


if HAS_ROS2:

    class SafetyMonitor(Node):
        """
        ROS2 Safety Monitor Node.

        Monitors all robots for collision risks and can trigger
        emergency stops if safety thresholds are violated.
        """

        def __init__(self):
            super().__init__('safety_monitor')

            # Declare parameters
            self.declare_parameter('min_distance', 0.3)
            self.declare_parameter('warning_distance', 0.5)
            self.declare_parameter('robot_radius', 0.1)
            self.declare_parameter('emergency_stop_enabled', True)
            self.declare_parameter('check_rate', 20.0)  # Hz

            # Get parameters
            self.min_distance = self.get_parameter('min_distance').value
            self.warning_distance = self.get_parameter('warning_distance').value
            self.robot_radius = self.get_parameter('robot_radius').value
            self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
            self.check_rate = self.get_parameter('check_rate').value

            # Load robot configuration
            config_dir = Path(__file__).parent.parent / "config"
            robots_file = config_dir / "robots.yaml"
            self.robots = self._load_robots(str(robots_file))

            self.get_logger().info(f"Monitoring {len(self.robots)} robots")
            self.get_logger().info(f"Min distance: {self.min_distance}m")
            self.get_logger().info(f"Warning distance: {self.warning_distance}m")

            # Subscribers and publishers
            self.odom_subs = {}
            self.cmd_pubs = {}

            for robot_id, robot in self.robots.items():
                # Odometry subscriber
                odom_topic = f'/{robot.name}/odom'
                self.odom_subs[robot_id] = self.create_subscription(
                    Odometry, odom_topic,
                    lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                    10
                )

                # Velocity publisher for emergency stop
                cmd_topic = f'/{robot.name}/cmd_vel'
                self.cmd_pubs[robot_id] = self.create_publisher(
                    Twist, cmd_topic, 10
                )

            # Status publishers
            self.safety_status_pub = self.create_publisher(
                String, '/mapf/safety_status', 10
            )
            self.collision_alert_pub = self.create_publisher(
                String, '/mapf/collision_alert', 10
            )

            # Monitor timer
            period = 1.0 / self.check_rate
            self.monitor_timer = self.create_timer(period, self._safety_check)

            # Statistics timer (every 5 seconds)
            self.stats_timer = self.create_timer(5.0, self._publish_stats)

            # State
            self.lock = threading.Lock()
            self.collision_events: List[CollisionEvent] = []
            self.emergency_stop_active = False
            self.start_time = time.time()

            # Statistics
            self.min_observed_distance = float('inf')
            self.warning_count = 0
            self.critical_count = 0

            self.get_logger().info("Safety Monitor initialized and active")

        def _load_robots(self, robots_file: str) -> Dict[int, RobotTracking]:
            """Load robot configurations."""
            with open(robots_file, 'r') as f:
                config = yaml.safe_load(f)

            robots = {}
            for r in config['robots']:
                robot = RobotTracking(
                    id=r['id'],
                    name=r['name']
                )
                robots[r['id']] = robot

            return robots

        def _odom_callback(self, msg: Odometry, robot_id: int):
            """Handle odometry updates."""
            with self.lock:
                if robot_id not in self.robots:
                    return

                robot = self.robots[robot_id]
                new_pos = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y
                )

                robot.velocity = (
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y
                )

                robot.position = new_pos
                robot.last_update = time.time()
                robot.position_history.append((time.time(), new_pos))

        def _safety_check(self):
            """Main safety check loop."""
            with self.lock:
                # Check all robot pairs
                robot_ids = list(self.robots.keys())

                for i in range(len(robot_ids)):
                    for j in range(i + 1, len(robot_ids)):
                        r1 = self.robots[robot_ids[i]]
                        r2 = self.robots[robot_ids[j]]

                        # Calculate distance
                        dx = r2.position[0] - r1.position[0]
                        dy = r2.position[1] - r1.position[1]
                        distance = math.sqrt(dx*dx + dy*dy)

                        # Update minimum observed
                        if distance < self.min_observed_distance:
                            self.min_observed_distance = distance

                        # Check thresholds
                        if distance < self.min_distance:
                            self._handle_collision(r1, r2, distance)
                        elif distance < self.warning_distance:
                            self._handle_warning(r1, r2, distance)

            # Publish current status
            self._publish_status()

        def _handle_collision(self, r1: RobotTracking, r2: RobotTracking, distance: float):
            """Handle collision detection."""
            self.critical_count += 1

            event = CollisionEvent(
                timestamp=time.time(),
                robot1_id=r1.id,
                robot2_id=r2.id,
                distance=distance,
                position1=r1.position,
                position2=r2.position,
                severity="critical" if distance > 2 * self.robot_radius else "collision"
            )
            self.collision_events.append(event)

            self.get_logger().error(
                f"COLLISION RISK: {r1.name} <-> {r2.name} "
                f"distance={distance:.3f}m (min={self.min_distance}m)"
            )

            # Publish alert
            alert_msg = String()
            alert_msg.data = (
                f"CRITICAL:{r1.name},{r2.name},"
                f"{distance:.3f},{r1.position},{r2.position}"
            )
            self.collision_alert_pub.publish(alert_msg)

            # Emergency stop if enabled
            if self.emergency_stop_enabled and not self.emergency_stop_active:
                self._emergency_stop()

        def _handle_warning(self, r1: RobotTracking, r2: RobotTracking, distance: float):
            """Handle near-miss warning."""
            self.warning_count += 1

            self.get_logger().warn(
                f"WARNING: {r1.name} <-> {r2.name} "
                f"distance={distance:.3f}m (warning={self.warning_distance}m)"
            )

            # Record event (limit rate to avoid spam)
            if len(self.collision_events) == 0 or \
               (time.time() - self.collision_events[-1].timestamp) > 1.0:
                event = CollisionEvent(
                    timestamp=time.time(),
                    robot1_id=r1.id,
                    robot2_id=r2.id,
                    distance=distance,
                    position1=r1.position,
                    position2=r2.position,
                    severity="warning"
                )
                self.collision_events.append(event)

        def _emergency_stop(self):
            """Trigger emergency stop for all robots."""
            self.emergency_stop_active = True
            self.get_logger().error("EMERGENCY STOP TRIGGERED!")

            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0

            for robot_id in self.robots:
                self.cmd_pubs[robot_id].publish(stop_cmd)

        def _publish_status(self):
            """Publish current safety status."""
            with self.lock:
                # Calculate current minimum distance
                min_dist = float('inf')
                robot_ids = list(self.robots.keys())

                for i in range(len(robot_ids)):
                    for j in range(i + 1, len(robot_ids)):
                        r1 = self.robots[robot_ids[i]]
                        r2 = self.robots[robot_ids[j]]
                        dx = r2.position[0] - r1.position[0]
                        dy = r2.position[1] - r1.position[1]
                        dist = math.sqrt(dx*dx + dy*dy)
                        min_dist = min(min_dist, dist)

            status = "OK" if min_dist > self.warning_distance else \
                     "WARNING" if min_dist > self.min_distance else \
                     "CRITICAL"

            status_msg = String()
            status_msg.data = f"{status}:{min_dist:.3f}"
            self.safety_status_pub.publish(status_msg)

        def _publish_stats(self):
            """Publish periodic statistics."""
            runtime = time.time() - self.start_time

            self.get_logger().info("="*40)
            self.get_logger().info("Safety Monitor Statistics")
            self.get_logger().info("="*40)
            self.get_logger().info(f"Runtime: {runtime:.1f}s")
            self.get_logger().info(f"Min observed distance: {self.min_observed_distance:.3f}m")
            self.get_logger().info(f"Warning events: {self.warning_count}")
            self.get_logger().info(f"Critical events: {self.critical_count}")
            self.get_logger().info(f"Collision events logged: {len(self.collision_events)}")
            self.get_logger().info(f"Emergency stop: {'ACTIVE' if self.emergency_stop_active else 'inactive'}")

            # Current robot positions
            self.get_logger().info("\nRobot Positions:")
            with self.lock:
                for robot in self.robots.values():
                    age = time.time() - robot.last_update if robot.last_update > 0 else float('inf')
                    status = "OK" if age < 1.0 else "STALE"
                    self.get_logger().info(
                        f"  {robot.name}: ({robot.position[0]:.2f}, {robot.position[1]:.2f}) [{status}]"
                    )

        def shutdown(self):
            """Clean shutdown with final report."""
            self.get_logger().info("\n" + "="*50)
            self.get_logger().info("SAFETY MONITOR FINAL REPORT")
            self.get_logger().info("="*50)

            runtime = time.time() - self.start_time
            self.get_logger().info(f"Total runtime: {runtime:.1f} seconds")
            self.get_logger().info(f"Minimum observed distance: {self.min_observed_distance:.3f}m")
            self.get_logger().info(f"Total warnings: {self.warning_count}")
            self.get_logger().info(f"Total critical events: {self.critical_count}")

            if self.critical_count == 0:
                self.get_logger().info("\n*** SAFETY VERIFICATION PASSED ***")
                self.get_logger().info("No collision violations detected!")
            else:
                self.get_logger().error("\n*** SAFETY VIOLATIONS DETECTED ***")
                self.get_logger().error(f"{self.critical_count} critical events occurred")

            # Log all collision events
            if self.collision_events:
                self.get_logger().info("\nCollision Event Log:")
                for i, event in enumerate(self.collision_events):
                    self.get_logger().info(
                        f"  {i+1}. [{event.severity.upper()}] "
                        f"Robot {event.robot1_id} <-> Robot {event.robot2_id}: "
                        f"{event.distance:.3f}m at t={event.timestamp - self.start_time:.2f}s"
                    )


def main():
    if not HAS_ROS2:
        print("ERROR: ROS2 not available!")
        print("This safety monitor requires ROS2 (ros-humble-desktop)")
        print("\nFor standalone simulation without ROS2, use:")
        print("  python3 standalone_simulation.py --verify")
        sys.exit(1)

    rclpy.init()
    monitor = SafetyMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Shutting down Safety Monitor...")
    finally:
        monitor.shutdown()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
