"""
400-Robot MAPF Visualization

Generates an animated GIF showing 400 robots moving through a warehouse
without collisions, demonstrating the quotient collapse solution.
"""

import random
import time
from typing import List, Tuple, Dict
import os

# Check for visualization dependencies
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation, PillowWriter
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("matplotlib not available - install with: pip install matplotlib")

try:
    from PIL import Image
    HAS_PILLOW = True
except ImportError:
    HAS_PILLOW = False
    print("Pillow not available - install with: pip install Pillow")

from .time_expanded_graph import TimeExpandedGraph, create_warehouse_graph
from .flow_solver import solve_production_mapf
from .path_decompose import extract_paths


def generate_colors(n: int) -> List[str]:
    """Generate n distinct colors for robots."""
    import colorsys
    colors = []
    for i in range(n):
        hue = i / n
        rgb = colorsys.hsv_to_rgb(hue, 0.7, 0.9)
        colors.append('#%02x%02x%02x' % (int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255)))
    return colors


def solve_400_robots(seed: int = 42) -> Tuple[Dict, List[List[int]], int, int]:
    """
    Solve 400 robot MAPF and return paths.

    Returns:
        (warehouse_info, paths, width, height)
    """
    print("Creating 40x40 warehouse...")
    warehouse = create_warehouse_graph(40, 40)
    width, height = 40, 40

    # Get valid vertices
    valid_vertices = [v for v in range(warehouse.num_vertices)
                      if v not in warehouse.obstacles]

    print(f"Valid vertices: {len(valid_vertices)}")

    # Generate 400 robots
    num_robots = 400
    random.seed(seed)
    random.shuffle(valid_vertices)
    starts = frozenset(valid_vertices[:num_robots])
    goals = frozenset(valid_vertices[num_robots:num_robots*2])

    print(f"Solving for {num_robots} robots...")

    # Build time-expanded graph
    horizon = 60
    te_graph = TimeExpandedGraph(
        base_graph=warehouse,
        horizon=horizon,
        starts=starts,
        goals=goals
    )

    # Solve
    start_time = time.time()
    result = solve_production_mapf(te_graph)
    solve_time = time.time() - start_time

    print(f"Solved in {solve_time:.2f}s - Status: {result.status.name}")

    if result.status.name != 'UNIQUE':
        raise ValueError(f"Failed to solve: {result.status.name}")

    # Extract paths
    paths_result = extract_paths(te_graph, result)

    # Convert paths to list of vertex sequences
    paths = []
    for ep in paths_result.extracted_paths:
        path = [ep.vertices[t] if t < len(ep.vertices) else ep.vertices[-1]
                for t in range(horizon + 1)]
        paths.append(path)

    warehouse_info = {
        'obstacles': warehouse.obstacles,
        'width': width,
        'height': height,
    }

    return warehouse_info, paths, width, height


def create_animation(warehouse_info: Dict, paths: List[List[int]],
                     width: int, height: int,
                     output_path: str = "400_robots.gif",
                     fps: int = 4,
                     max_frames: int = 60) -> str:
    """
    Create animated GIF of robot paths.

    Args:
        warehouse_info: Dict with obstacles
        paths: List of paths (each path is list of vertex IDs)
        width, height: Grid dimensions
        output_path: Output file path
        fps: Frames per second
        max_frames: Maximum frames to render

    Returns:
        Path to generated GIF
    """
    if not HAS_MATPLOTLIB:
        raise ImportError("matplotlib required for visualization")

    obstacles = warehouse_info['obstacles']
    num_robots = len(paths)
    colors = generate_colors(num_robots)

    # Convert vertex to (x, y)
    def v_to_xy(v: int) -> Tuple[int, int]:
        return (v % width, v // width)

    # Create figure
    fig, ax = plt.subplots(figsize=(12, 12))

    def draw_frame(t: int):
        ax.clear()
        ax.set_xlim(-0.5, width - 0.5)
        ax.set_ylim(-0.5, height - 0.5)
        ax.set_aspect('equal')
        ax.set_title(f'400 Robots - Quotient Collapse MAPF\nTimestep {t}/{max_frames}',
                     fontsize=14, fontweight='bold')
        ax.axis('off')

        # Draw grid
        for x in range(width + 1):
            ax.axvline(x - 0.5, color='#EEEEEE', linewidth=0.5)
        for y in range(height + 1):
            ax.axhline(y - 0.5, color='#EEEEEE', linewidth=0.5)

        # Draw obstacles
        for obs in obstacles:
            x, y = v_to_xy(obs)
            rect = patches.Rectangle((x - 0.5, y - 0.5), 1, 1,
                                      facecolor='#333333', edgecolor='none')
            ax.add_patch(rect)

        # Draw robots
        for i, path in enumerate(paths):
            if t < len(path):
                v = path[t]
            else:
                v = path[-1]
            x, y = v_to_xy(v)
            circle = patches.Circle((x, y), 0.35,
                                     facecolor=colors[i],
                                     edgecolor='black',
                                     linewidth=0.3,
                                     alpha=0.8)
            ax.add_patch(circle)

    # Generate frames
    print(f"Rendering {max_frames} frames...")
    frames = []

    for t in range(max_frames):
        if t % 10 == 0:
            print(f"  Frame {t}/{max_frames}")
        draw_frame(t)
        fig.canvas.draw()

        # Convert to image
        image = Image.frombytes('RGB',
                                fig.canvas.get_width_height(),
                                fig.canvas.tostring_rgb())
        frames.append(image)

    plt.close(fig)

    # Save as GIF
    print(f"Saving GIF to {output_path}...")
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000/fps),
        loop=0
    )

    file_size = os.path.getsize(output_path) / (1024 * 1024)
    print(f"GIF saved: {output_path} ({file_size:.1f} MB)")

    return output_path


def main():
    """Main entry point for 400-robot visualization."""
    print("=" * 60)
    print("400-ROBOT MAPF VISUALIZATION")
    print("Quotient Collapse Solution - O(|V|×T) Complexity")
    print("=" * 60)

    # Check dependencies
    if not HAS_MATPLOTLIB or not HAS_PILLOW:
        print("\nMissing dependencies. Install with:")
        print("  pip install matplotlib Pillow")
        return

    # Solve
    warehouse_info, paths, width, height = solve_400_robots()
    print(f"Extracted {len(paths)} paths")

    # Create animation
    output_path = "kernel/mapf/production/400_robots_demo.gif"
    create_animation(
        warehouse_info, paths, width, height,
        output_path=output_path,
        fps=4,
        max_frames=60
    )

    print("\n" + "=" * 60)
    print("VISUALIZATION COMPLETE")
    print("=" * 60)
    print(f"Output: {output_path}")
    print("400 robots moving collision-free through 40x40 warehouse")
    print("V1-V5 all satisfied by construction (quotient collapse)")


if __name__ == "__main__":
    main()
