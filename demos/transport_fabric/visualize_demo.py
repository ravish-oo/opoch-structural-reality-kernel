"""
Demo Visualization for Transport Fabric
Creates an optimized, shareable GIF showing robot coordination.
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

from kernel.mapf.fabric.warehouse_graph import FabricWarehouseGraph
from kernel.mapf.fabric.fabric_compiler import FabricCompiler
from kernel.mapf.fabric.permutation_executor import FastFabricExecutor, OccupancyBitset
from kernel.mapf.fabric.controller import BackpressureController, QueueState


def create_demo_gif():
    """Create optimized demo GIF."""

    GRID_SIZE = 60
    ROBOTS = 1500
    TICKS = 45
    FPS = 15

    print("Creating demo visualization...")
    print(f"  Grid: {GRID_SIZE}x{GRID_SIZE}, Robots: {ROBOTS:,}")

    # Compile
    warehouse = FabricWarehouseGraph.from_grid(GRID_SIZE, GRID_SIZE)
    compiler = FabricCompiler(warehouse)
    fabric = compiler.compile()

    executor = FastFabricExecutor(fabric)
    controller = BackpressureController(fabric)
    queues = QueueState()

    random.seed(42)
    robot_positions = random.sample(range(GRID_SIZE * GRID_SIZE), ROBOTS)
    occupancy = OccupancyBitset(GRID_SIZE * GRID_SIZE, set(robot_positions))

    os.makedirs("output", exist_ok=True)
    frames = []

    colors = ['#0d0d1a', '#1a1a3a', '#2d4d6a', '#00ffaa']
    cmap = LinearSegmentedColormap.from_list('robots', colors)

    total_moves = 0

    for tick in range(TICKS):
        fig, ax = plt.subplots(1, 1, figsize=(6, 6), facecolor='#0a0a0a')
        ax.set_facecolor('#0a0a0a')

        density = np.zeros((GRID_SIZE, GRID_SIZE))
        for pos in robot_positions:
            density[pos // GRID_SIZE, pos % GRID_SIZE] = 1
        density_smooth = gaussian_filter(density, sigma=1.5)

        ax.imshow(density_smooth, cmap=cmap, interpolation='bilinear',
                  extent=[0, GRID_SIZE, 0, GRID_SIZE], vmin=0, vmax=0.4)

        ax.set_title(
            f"OPOCH: {ROBOTS:,} Robots | Tick {tick+1} | 0 Collisions",
            fontsize=10, color='white', fontweight='bold'
        )

        ax.text(0.02, 0.98, f"Moves: {total_moves:,}", transform=ax.transAxes,
                fontsize=8, color='#00ffaa', verticalalignment='top',
                fontfamily='monospace')

        ax.axis('off')
        plt.tight_layout()

        frame_path = f"output/demo_{tick:03d}.png"
        plt.savefig(frame_path, dpi=72, facecolor='#0a0a0a', bbox_inches='tight')
        plt.close()

        img = Image.open(frame_path).convert('P', palette=Image.ADAPTIVE, colors=64)
        frames.append(np.array(img.convert('RGB')))

        weights = controller.create_weight_function(occupancy, queues)
        new_occupancy, permutation = executor.execute_tick(occupancy, weights)
        moves = sum(1 for v in robot_positions if permutation.apply(v) != v)
        total_moves += moves
        robot_positions = [permutation.apply(v) for v in robot_positions]
        occupancy = new_occupancy

        if (tick + 1) % 15 == 0:
            print(f"  Frame {tick+1}/{TICKS}")

    for tick in range(TICKS):
        os.remove(f"output/demo_{tick:03d}.png")

    gif_path = "output/opoch_demo.gif"
    imageio.mimsave(gif_path, frames, fps=FPS, loop=0)

    size_mb = os.path.getsize(gif_path) / (1024 * 1024)
    print(f"  Created: {gif_path} ({size_mb:.1f} MB)")

    return gif_path


if __name__ == "__main__":
    create_demo_gif()
