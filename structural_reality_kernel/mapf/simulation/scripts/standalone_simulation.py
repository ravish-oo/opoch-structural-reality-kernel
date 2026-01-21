#!/usr/bin/env python3
"""
Standalone MAPF Simulation - No ROS2 Required

Visualizes the 12x12 8-agent MAPF solution using matplotlib animation.
This allows testing and visualization without Gazebo/ROS2 dependencies.

Usage:
    python3 standalone_simulation.py [--speed 0.5] [--save output.gif]
"""

import os
import sys
import json
import yaml
import argparse
import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import matplotlib.animation as animation
    from matplotlib.collections import PatchCollection
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available. Install with: pip install matplotlib")


@dataclass
class Robot:
    """Robot state representation."""
    id: int
    name: str
    color: Tuple[float, float, float]
    start_grid: Tuple[int, int]
    goal_grid: Tuple[int, int]
    path: List[Tuple[int, int]]
    current_pos: Tuple[float, float] = (0.0, 0.0)


class StandaloneSimulation:
    """
    Standalone MAPF simulation visualizer.

    Renders the CBS-solved paths for 8 agents on a 12x12 grid
    with smooth interpolation between timesteps.
    """

    def __init__(self, config_dir: str = None):
        if config_dir is None:
            config_dir = Path(__file__).parent.parent / "config"
        else:
            config_dir = Path(config_dir)

        self.config_dir = config_dir
        self.grid_size = (12, 12)
        self.cell_size = 0.5  # meters

        # Load configurations
        self.robots = self._load_robots()
        self.solution = self._load_solution()

        # Animation state
        self.current_timestep = 0
        self.max_timestep = self._get_makespan()
        self.interpolation_steps = 20  # frames per timestep

        print(f"Loaded {len(self.robots)} robots")
        print(f"Solution makespan: {self.max_timestep} timesteps")

    def _load_robots(self) -> List[Robot]:
        """Load robot configurations from YAML."""
        robots_path = self.config_dir / "robots.yaml"

        with open(robots_path, 'r') as f:
            config = yaml.safe_load(f)

        robots = []
        for r in config['robots']:
            robot = Robot(
                id=r['id'],
                name=r['name'],
                color=tuple(r['color']),
                start_grid=tuple(r['start']['grid']),
                goal_grid=tuple(r['goal']['grid']),
                path=[]
            )
            robots.append(robot)

        return robots

    def _load_solution(self) -> Dict:
        """Load MAPF solution from JSON."""
        solution_path = self.config_dir / "mapf_solution.json"

        with open(solution_path, 'r') as f:
            solution = json.load(f)

        # Assign paths to robots - handle different JSON formats
        if 'agents' in solution:
            # Old format: solution['agents'][i]['path']
            for agent in solution['agents']:
                robot_id = agent['id']
                path = [tuple(p) for p in agent['path']]
                self.robots[robot_id].path = path
        elif 'paths' in solution:
            # New format: solution['paths'][i]['waypoints']
            for agent_path in solution['paths']:
                robot_id = agent_path['agent_id']
                # Extract grid coordinates from waypoints
                path = [tuple(wp['grid']) for wp in agent_path['waypoints']]
                self.robots[robot_id].path = path

        return solution

    def _get_makespan(self) -> int:
        """Get the maximum path length (makespan)."""
        return max(len(r.path) for r in self.robots)

    def _grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        x = grid_pos[0] * self.cell_size + self.cell_size / 2
        y = grid_pos[1] * self.cell_size + self.cell_size / 2
        return (x, y)

    def _get_position_at_time(self, robot: Robot, t: float) -> Tuple[float, float]:
        """
        Get interpolated position at continuous time t.

        Args:
            robot: Robot object with path
            t: Continuous time (can be fractional)

        Returns:
            (x, y) world coordinates
        """
        if not robot.path:
            return self._grid_to_world(robot.start_grid)

        # Clamp to valid range
        t = max(0, min(t, len(robot.path) - 1))

        # Get discrete timesteps
        t0 = int(t)
        t1 = min(t0 + 1, len(robot.path) - 1)

        # Interpolation factor
        alpha = t - t0

        # Get grid positions
        pos0 = robot.path[t0]
        pos1 = robot.path[t1]

        # Convert to world coordinates
        world0 = self._grid_to_world(pos0)
        world1 = self._grid_to_world(pos1)

        # Linear interpolation
        x = world0[0] + alpha * (world1[0] - world0[0])
        y = world0[1] + alpha * (world1[1] - world0[1])

        return (x, y)

    def verify_solution(self) -> bool:
        """Run V1-V5 verification on loaded solution."""
        print("\n=== Solution Verification ===")

        # V1: Start positions
        v1_pass = all(r.path[0] == r.start_grid for r in self.robots)
        print(f"V1 (Start):  {'PASS' if v1_pass else 'FAIL'}")

        # V2: Goal positions
        v2_pass = all(r.path[-1] == r.goal_grid for r in self.robots)
        print(f"V2 (Goal):   {'PASS' if v2_pass else 'FAIL'}")

        # V3: Valid movements (4-connected)
        v3_pass = True
        for r in self.robots:
            for i in range(len(r.path) - 1):
                p1, p2 = r.path[i], r.path[i+1]
                dx, dy = abs(p2[0] - p1[0]), abs(p2[1] - p1[1])
                if dx + dy > 1:  # More than one cell move
                    v3_pass = False
                    break
        print(f"V3 (Dynamics): {'PASS' if v3_pass else 'FAIL'}")

        # V4: Vertex conflicts
        v4_pass = True
        for t in range(self.max_timestep):
            positions = {}
            for r in self.robots:
                pos = r.path[min(t, len(r.path)-1)]
                if pos in positions:
                    v4_pass = False
                    print(f"  Conflict at t={t}: {positions[pos]} and {r.name} at {pos}")
                positions[pos] = r.name
        print(f"V4 (Vertex): {'PASS' if v4_pass else 'FAIL'}")

        # V5: Edge conflicts (swaps)
        v5_pass = True
        for t in range(self.max_timestep - 1):
            for i, r1 in enumerate(self.robots):
                for r2 in self.robots[i+1:]:
                    p1_t = r1.path[min(t, len(r1.path)-1)]
                    p1_t1 = r1.path[min(t+1, len(r1.path)-1)]
                    p2_t = r2.path[min(t, len(r2.path)-1)]
                    p2_t1 = r2.path[min(t+1, len(r2.path)-1)]

                    if p1_t == p2_t1 and p2_t == p1_t1:
                        v5_pass = False
                        print(f"  Edge conflict at t={t}: {r1.name} and {r2.name}")
        print(f"V5 (Edge):   {'PASS' if v5_pass else 'FAIL'}")

        all_pass = v1_pass and v2_pass and v3_pass and v4_pass and v5_pass
        print(f"\nOverall: {'ALL PASS' if all_pass else 'VERIFICATION FAILED'}")

        return all_pass

    def run_text_simulation(self):
        """Run a text-based simulation (no matplotlib required)."""
        print("\n=== Text Simulation ===")
        print(f"Grid: {self.grid_size[0]}x{self.grid_size[1]}")
        print(f"Robots: {len(self.robots)}")
        print(f"Makespan: {self.max_timestep}")
        print()

        for t in range(self.max_timestep + 1):
            print(f"--- Timestep {t} ---")

            # Create grid
            grid = [['.' for _ in range(self.grid_size[0])] for _ in range(self.grid_size[1])]

            # Place goals
            for r in self.robots:
                gx, gy = r.goal_grid
                if grid[gy][gx] == '.':
                    grid[gy][gx] = 'G'

            # Place robots
            for r in self.robots:
                pos = r.path[min(t, len(r.path)-1)]
                grid[pos[1]][pos[0]] = str(r.id)

            # Print grid (flipped for proper orientation)
            for row in reversed(grid):
                print(' '.join(row))
            print()

        print("=== Simulation Complete ===")

    def run_animation(self, speed: float = 1.0, save_path: Optional[str] = None):
        """
        Run animated visualization using matplotlib.

        Args:
            speed: Animation speed multiplier
            save_path: If provided, save animation to file (GIF or MP4)
        """
        if not HAS_MATPLOTLIB:
            print("matplotlib not available, falling back to text simulation")
            self.run_text_simulation()
            return

        # Setup figure
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(-0.5, self.grid_size[0] * self.cell_size + 0.5)
        ax.set_ylim(-0.5, self.grid_size[1] * self.cell_size + 0.5)
        ax.set_aspect('equal')
        ax.set_title('MAPF 12x12 8-Agent Simulation', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')

        # Draw grid
        for i in range(self.grid_size[0] + 1):
            x = i * self.cell_size
            ax.axvline(x, color='lightgray', linewidth=0.5)
        for j in range(self.grid_size[1] + 1):
            y = j * self.cell_size
            ax.axhline(y, color='lightgray', linewidth=0.5)

        # Draw goals
        goal_markers = []
        for r in self.robots:
            gx, gy = self._grid_to_world(r.goal_grid)
            goal = patches.Circle((gx, gy), 0.15,
                                  facecolor=(*r.color, 0.3),
                                  edgecolor=r.color,
                                  linewidth=2)
            ax.add_patch(goal)
            goal_markers.append(goal)

        # Create robot circles
        robot_patches = []
        robot_labels = []
        for r in self.robots:
            pos = self._grid_to_world(r.start_grid)
            circle = patches.Circle(pos, 0.12,
                                   facecolor=r.color,
                                   edgecolor='black',
                                   linewidth=1)
            ax.add_patch(circle)
            robot_patches.append(circle)

            # Robot label
            label = ax.text(pos[0], pos[1], str(r.id),
                           ha='center', va='center',
                           fontsize=10, fontweight='bold',
                           color='white')
            robot_labels.append(label)

        # Trail lines
        trail_lines = []
        for r in self.robots:
            line, = ax.plot([], [], color=r.color, alpha=0.5, linewidth=2)
            trail_lines.append(line)

        # Time display
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           fontsize=12, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        # Status text
        status_text = ax.text(0.98, 0.98, '', transform=ax.transAxes,
                             fontsize=10, verticalalignment='top',
                             horizontalalignment='right',
                             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

        total_frames = (self.max_timestep + 1) * self.interpolation_steps

        def init():
            for patch in robot_patches:
                patch.center = (0, 0)
            time_text.set_text('')
            status_text.set_text('')
            for line in trail_lines:
                line.set_data([], [])
            return robot_patches + robot_labels + trail_lines + [time_text, status_text]

        def animate(frame):
            # Calculate continuous time
            t = frame / self.interpolation_steps

            # Count robots at goal
            at_goal = 0

            for i, r in enumerate(self.robots):
                # Get interpolated position
                pos = self._get_position_at_time(r, t)
                robot_patches[i].center = pos
                robot_labels[i].set_position(pos)

                # Check if at goal
                goal_world = self._grid_to_world(r.goal_grid)
                dist = np.sqrt((pos[0] - goal_world[0])**2 + (pos[1] - goal_world[1])**2)
                if dist < 0.1:
                    at_goal += 1

                # Update trail
                trail_t = max(0, t - 3)  # Show last 3 timesteps
                trail_points = []
                for tt in np.linspace(trail_t, t, 20):
                    trail_points.append(self._get_position_at_time(r, tt))
                if trail_points:
                    xs, ys = zip(*trail_points)
                    trail_lines[i].set_data(xs, ys)

            time_text.set_text(f'Timestep: {t:.1f} / {self.max_timestep}')
            status_text.set_text(f'At Goal: {at_goal}/{len(self.robots)}')

            return robot_patches + robot_labels + trail_lines + [time_text, status_text]

        interval = int(100 / speed)  # milliseconds per frame
        anim = animation.FuncAnimation(
            fig, animate, init_func=init,
            frames=total_frames, interval=interval,
            blit=True, repeat=True
        )

        if save_path:
            print(f"Saving animation to {save_path}...")
            if save_path.endswith('.gif'):
                anim.save(save_path, writer='pillow', fps=20)
            elif save_path.endswith('.mp4'):
                anim.save(save_path, writer='ffmpeg', fps=20)
            print(f"Saved to {save_path}")
        else:
            plt.tight_layout()
            plt.show()


def print_solution_stats(sim: StandaloneSimulation):
    """Print detailed solution statistics."""
    print("\n" + "="*50)
    print("       MAPF Solution Statistics")
    print("="*50)

    print(f"\nGrid Configuration:")
    print(f"  Size: {sim.grid_size[0]} x {sim.grid_size[1]} = {sim.grid_size[0] * sim.grid_size[1]} cells")
    print(f"  Cell size: {sim.cell_size}m")
    print(f"  World size: {sim.grid_size[0] * sim.cell_size}m x {sim.grid_size[1] * sim.cell_size}m")

    print(f"\nAgents: {len(sim.robots)}")
    for r in sim.robots:
        path_len = len(r.path) - 1
        print(f"  {r.name}: {r.start_grid} -> {r.goal_grid} (path length: {path_len})")

    makespan = sim.max_timestep
    sum_of_costs = sum(len(r.path) - 1 for r in sim.robots)

    print(f"\nSolution Metrics:")
    print(f"  Makespan: {makespan}")
    print(f"  Sum of Costs: {sum_of_costs}")

    # Search space metrics from solution file
    if 'metadata' in sim.solution:
        meta = sim.solution['metadata']
        print(f"  CBS Nodes: {meta.get('nodes_expanded', 'N/A')}")
        print(f"  Naive Space: {meta.get('naive_space', 'N/A')}")
        print(f"  Compression: 10^{meta.get('compression_log10', 'N/A')}")

    print("\n" + "="*50)


def main():
    parser = argparse.ArgumentParser(
        description='Standalone MAPF Simulation Visualizer'
    )
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Animation speed multiplier (default: 1.0)')
    parser.add_argument('--save', type=str, default=None,
                       help='Save animation to file (GIF or MP4)')
    parser.add_argument('--text', action='store_true',
                       help='Run text-only simulation')
    parser.add_argument('--verify', action='store_true',
                       help='Run verification only')
    parser.add_argument('--stats', action='store_true',
                       help='Print solution statistics')
    parser.add_argument('--config', type=str, default=None,
                       help='Path to config directory')

    args = parser.parse_args()

    # Initialize simulation
    sim = StandaloneSimulation(config_dir=args.config)

    # Print stats
    if args.stats or args.verify:
        print_solution_stats(sim)

    # Verify solution
    if args.verify:
        sim.verify_solution()
        return

    # Run simulation
    if args.text:
        sim.run_text_simulation()
    else:
        sim.run_animation(speed=args.speed, save_path=args.save)


if __name__ == '__main__':
    main()
