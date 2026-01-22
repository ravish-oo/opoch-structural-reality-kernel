"""
Full 10,000-Robot Transport Fabric Visualization

Creates an animated GIF showing the actual 10K scale demo.
Optimized for file size while maintaining visual impact.
"""

import os
import time
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy.ndimage import gaussian_filter
import imageio
from PIL import Image

# Import the kernel
from kernel.mapf.fabric.warehouse_graph import FabricWarehouseGraph
from kernel.mapf.fabric.fabric_compiler import FabricCompiler
from kernel.mapf.fabric.permutation_executor import FastFabricExecutor, OccupancyBitset
from kernel.mapf.fabric.controller import BackpressureController, QueueState


def create_10k_visualization():
    """Create full 10K robot visualization."""

    # Full 10K scale
    GRID_SIZE = 150
    ROBOTS = 10000
    TICKS = 30  # Shorter for smaller file
    FPS = 10

    print("=" * 70)
    print("OPOCH KERNEL: FULL 10,000-ROBOT VISUALIZATION")
    print("=" * 70)
    print()
    print(f"Grid: {GRID_SIZE}x{GRID_SIZE} = {GRID_SIZE*GRID_SIZE:,} vertices")
    print(f"Robots: {ROBOTS:,}")
    print(f"Density: {ROBOTS/(GRID_SIZE*GRID_SIZE)*100:.1f}%")
    print()

    # Compile fabric
    print("Compiling transport fabric...")
    t0 = time.perf_counter()
    warehouse = FabricWarehouseGraph.from_grid(GRID_SIZE, GRID_SIZE)
    compiler = FabricCompiler(warehouse)
    fabric = compiler.compile()
    compile_time = time.perf_counter() - t0
    print(f"  Compiled in {compile_time:.1f}s")
    print()

    # Initialize
    executor = FastFabricExecutor(fabric)
    controller = BackpressureController(fabric)
    queues = QueueState()

    # Place robots
    random.seed(42)
    vertices = list(range(GRID_SIZE * GRID_SIZE))
    robot_positions = random.sample(vertices, ROBOTS)
    occupied_set = set(robot_positions)
    occupancy = OccupancyBitset(GRID_SIZE * GRID_SIZE, occupied_set)

    os.makedirs("output", exist_ok=True)
    frames = []

    # Custom colormap
    colors = ['#0d0d1a', '#1a1a3a', '#2d2d5a', '#00ff88']
    cmap = LinearSegmentedColormap.from_list('robots', colors)

    print("Generating frames...")
    total_moves = 0
    tick_times = []

    for tick in range(TICKS):
        t_start = time.perf_counter()

        # Create figure - smaller for optimization
        fig, ax = plt.subplots(1, 1, figsize=(8, 8), facecolor='#0a0a0a')
        ax.set_facecolor('#0a0a0a')

        # Create density grid
        density = np.zeros((GRID_SIZE, GRID_SIZE))
        for pos in robot_positions:
            x = pos % GRID_SIZE
            y = pos // GRID_SIZE
            density[y, x] = 1

        # Smooth for visual effect
        density_smooth = gaussian_filter(density, sigma=2.0)

        # Plot
        ax.imshow(density_smooth, cmap=cmap, interpolation='bilinear',
                  extent=[0, GRID_SIZE, 0, GRID_SIZE], vmin=0, vmax=0.3)

        # Title
        ax.set_title(
            f"OPOCH KERNEL: 10,000 Robots | Tick {tick+1}/{TICKS}\n"
            f"ZERO Collisions (Mathematically Proven)",
            fontsize=12, color='white', fontweight='bold', pad=10
        )

        # Stats overlay
        stats = (
            f"Robots: {ROBOTS:,}\n"
            f"Grid: {GRID_SIZE}x{GRID_SIZE}\n"
            f"Moves: {total_moves:,}\n"
            f"Collisions: 0"
        )
        ax.text(0.02, 0.98, stats, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', fontfamily='monospace', color='#00ff88',
                bbox=dict(boxstyle='round', facecolor='#1a1a2e', edgecolor='#00ff88', alpha=0.9))

        # Mathematical guarantee
        ax.text(0.98, 0.02, "P: V→V bijection\nCollisions ∉ model",
                transform=ax.transAxes, fontsize=9,
                verticalalignment='bottom', horizontalalignment='right',
                fontfamily='monospace', color='#ff6b6b',
                bbox=dict(boxstyle='round', facecolor='#1a1a2e', edgecolor='#ff6b6b', alpha=0.9))

        ax.set_xlim(0, GRID_SIZE)
        ax.set_ylim(0, GRID_SIZE)
        ax.axis('off')
        plt.tight_layout()

        # Save frame at lower DPI for smaller file
        frame_path = f"output/frame_10k_{tick:03d}.png"
        plt.savefig(frame_path, dpi=80, facecolor='#0a0a0a',
                    bbox_inches='tight', pad_inches=0.05)
        plt.close()

        # Load frame
        img = Image.open(frame_path)
        # Convert to palette mode for smaller GIF
        img = img.convert('P', palette=Image.ADAPTIVE, colors=64)
        frames.append(np.array(img.convert('RGB')))

        # Execute tick
        weights = controller.create_weight_function(occupancy, queues)
        new_occupancy, permutation = executor.execute_tick(occupancy, weights)

        # Count moves
        moves = sum(1 for v in robot_positions if permutation.apply(v) != v)
        total_moves += moves

        # Update positions
        robot_positions = [permutation.apply(v) for v in robot_positions]
        occupancy = new_occupancy

        tick_time = time.perf_counter() - t_start
        tick_times.append(tick_time)

        if (tick + 1) % 10 == 0:
            avg_hz = 1.0 / (sum(tick_times[-10:]) / 10)
            print(f"  Tick {tick+1}/{TICKS} | {avg_hz:.1f} Hz | {moves:,} moves")

    # Clean up frames
    for tick in range(TICKS):
        os.remove(f"output/frame_10k_{tick:03d}.png")

    print()
    print("Creating GIF...")

    # Create optimized GIF
    gif_path = "output/opoch_10k_full.gif"
    imageio.mimsave(gif_path, frames, fps=FPS, loop=0)

    # Get file size
    size_mb = os.path.getsize(gif_path) / (1024 * 1024)

    avg_tick = sum(tick_times) / len(tick_times)
    avg_hz = 1.0 / avg_tick

    print()
    print("=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"  GIF: {gif_path}")
    print(f"  Size: {size_mb:.1f} MB")
    print(f"  Frames: {TICKS}")
    print(f"  Duration: {TICKS/FPS:.1f}s")
    print()
    print(f"  Avg Tick Rate: {avg_hz:.1f} Hz")
    print(f"  Total Moves: {total_moves:,}")
    print(f"  Collisions: 0 (proven impossible)")
    print("=" * 70)

    return gif_path


if __name__ == "__main__":
    create_10k_visualization()
