"""
Path visualization utilities for drone motion planning.
Provides functions to visualize planned paths on 2D obstacle maps.
"""

import matplotlib.pyplot as plt
import numpy as np
from planning_utils import create_grid, a_star, heuristic, prune_path, validate_position


def calculate_boundaries(data):
    """Calculate grid boundaries from obstacle data."""
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))
    return north_min, north_max, east_min, east_max


def plot_obstacles(ax, data, max_height=200):
    """Plot obstacles as colored rectangles on the given axis."""
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Rectangle bounds
        e_min, e_max = east - d_east, east + d_east
        n_min, n_max = north - d_north, north + d_north

        # Color by height
        color = plt.cm.viridis((alt + d_alt) / max_height)

        # Draw rectangle
        rect = plt.Rectangle(
            (e_min, n_min),
            e_max - e_min,
            n_max - n_min,
            facecolor=color,
            edgecolor='black',
            linewidth=0.3,
            alpha=0.6
        )
        ax.add_patch(rect)


def grid_to_local(grid_coords, offset):
    """Convert grid coordinates to local NED coordinates."""
    return grid_coords[0] + offset[0], grid_coords[1] + offset[1]


def plot_path(ax, path, north_offset, east_offset, label='Path', **kwargs):
    """Plot path on the given axis."""
    if len(path) == 0:
        return

    path_north = [p[0] + north_offset for p in path]
    path_east = [p[1] + east_offset for p in path]

    # Default styling
    line_style = kwargs.get('linestyle', '-')
    line_width = kwargs.get('linewidth', 3)
    color = kwargs.get('color', 'r')
    alpha = kwargs.get('alpha', 0.7)
    marker_size = kwargs.get('markersize', 5)

    ax.plot(path_east, path_north, color=color, linestyle=line_style,
            linewidth=line_width, alpha=alpha, label=label, zorder=5)
    ax.plot(path_east, path_north, f'{color}o', markersize=marker_size,
            alpha=alpha*0.8, zorder=5)


def plot_markers(ax, grid_start, grid_goal, north_offset, east_offset, show_home=True, custom_point=None):
    """Plot start, goal, and home markers."""
    start_north, start_east = grid_to_local(grid_start, (north_offset, east_offset))
    goal_north, goal_east = grid_to_local(grid_goal, (north_offset, east_offset))

    # Start (green hollow circle)
    ax.plot(start_east, start_north, 'go', markersize=18, label='Start',
            zorder=10, markerfacecolor='none', markeredgecolor='darkgreen', markeredgewidth=3)

    # Goal (red hollow circle)
    ax.plot(goal_east, goal_north, 'ro', markersize=18, label='Goal',
            zorder=10, markerfacecolor='none', markeredgecolor='darkred', markeredgewidth=3)

    # Home (blue hollow circle)
    if show_home:
        ax.plot(0, 0, 'bo', markersize=15, label='Home',
                zorder=10, markerfacecolor='none', markeredgecolor='darkblue', markeredgewidth=2.5)

    # Custom point (small purple cross marker to show actual point size)
    if custom_point is not None:
        custom_north, custom_east = grid_to_local(custom_point, (north_offset, east_offset))
        ax.plot(custom_east, custom_north, 'x', markersize=10, label='Custom',
                zorder=11, color='purple', markeredgewidth=2.5)


def add_compass(ax, east_min, east_max, north_min, north_max):
    """Add a compass rose to the plot."""
    arrow_len = (east_max - east_min) * 0.08
    compass_e = east_max - (east_max - east_min) * 0.12
    compass_n = north_max - (north_max - north_min) * 0.12

    ax.arrow(compass_e, compass_n, 0, arrow_len,
             head_width=15, head_length=12,
             fc='red', ec='red', linewidth=2.5, zorder=15)
    ax.text(compass_e, compass_n + arrow_len + 25, 'N',
            fontsize=14, weight='bold', color='red', ha='center', zorder=15)


def format_axis(ax, title, show_colorbar=False, max_height=200):
    """Apply common formatting to axis."""
    ax.set_xlabel('East (meters)', fontsize=12, weight='bold')
    ax.set_ylabel('North (meters)', fontsize=12, weight='bold')
    ax.set_title(title, fontsize=14, weight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.4, linestyle='--')
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9)

    if show_colorbar:
        sm = plt.cm.ScalarMappable(
            cmap=plt.cm.viridis,
            norm=plt.Normalize(vmin=0, vmax=max_height)
        )
        sm.set_array([])
        plt.colorbar(sm, ax=ax, label='Obstacle Height (meters)')


def visualize_path_2d(data, grid, north_offset, east_offset, path, grid_start,
                      grid_goal, path_cost=None, save_path='../Logs/path_visualization.png',
                      custom_point=None):
    """
    Visualize 2D obstacle map with planned path.

    Args:
        data: Obstacle data array from colliders.csv
        grid: 2D occupancy grid
        north_offset: North offset of grid origin
        east_offset: East offset of grid origin
        path: Path as list of (row, col) tuples in grid coordinates
        grid_start: Start position in grid coordinates
        grid_goal: Goal position in grid coordinates
        path_cost: Optional total path cost
        save_path: Path to save visualization image
        custom_point: Optional custom point to mark on the map (grid coordinates)
    """
    fig, ax = plt.subplots(figsize=(16, 14))

    # Calculate boundaries
    north_min, north_max, east_min, east_max = calculate_boundaries(data)

    # Plot components
    plot_obstacles(ax, data)
    plot_path(ax, path, north_offset, east_offset, label='Path')
    plot_markers(ax, grid_start, grid_goal, north_offset, east_offset, custom_point=custom_point)
    add_compass(ax, east_min, east_max, north_min, north_max)

    # Format
    format_axis(ax, 'Planned Drone Path with Obstacles\n(Color indicates obstacle height)',
                show_colorbar=True)

    # Add path info
    if len(path) > 0:
        info_lines = [
            f'Path Statistics:',
            f'- Waypoints: {len(path)}',
        ]
        if path_cost is not None:
            info_lines.append(f'- Path Cost: {path_cost:.2f}')
        info_lines.extend([
            f'- Start: ({grid_start[0]}, {grid_start[1]})',
            f'- Goal: ({grid_goal[0]}, {grid_goal[1]})',
            f'- Grid Size: {grid.shape[0]} × {grid.shape[1]}'
        ])

        ax.text(0.02, 0.98, '\n'.join(info_lines),
                transform=ax.transAxes, fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))

    # Add annotations
    start_north, start_east = grid_to_local(grid_start, (north_offset, east_offset))
    goal_north, goal_east = grid_to_local(grid_goal, (north_offset, east_offset))

    ax.annotate('START', xy=(start_east, start_north), xytext=(20, 20),
                textcoords='offset points', fontsize=12, color='darkgreen', weight='bold',
                arrowprops=dict(arrowstyle='->', color='darkgreen', lw=2),
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.8))

    ax.annotate('GOAL', xy=(goal_east, goal_north), xytext=(20, -30),
                textcoords='offset points', fontsize=12, color='darkred', weight='bold',
                arrowprops=dict(arrowstyle='->', color='darkred', lw=2),
                bbox=dict(boxstyle='round,pad=0.5', facecolor='lightcoral', alpha=0.8))

    if custom_point is not None:
        custom_north, custom_east = grid_to_local(custom_point, (north_offset, east_offset))
        ax.annotate('CUSTOM', xy=(custom_east, custom_north), xytext=(-30, 20),
                    textcoords='offset points', fontsize=12, color='purple', weight='bold',
                    arrowprops=dict(arrowstyle='->', color='purple', lw=2),
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='plum', alpha=0.8))

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Path visualization saved to: {save_path}")
    plt.show()


def visualize_path_comparison(data, grid, north_offset, east_offset,
                               path_original, path_pruned, grid_start, grid_goal,
                               save_path='../Logs/path_comparison.png'):
    """
    Compare original and pruned paths side by side.

    Args:
        data: Obstacle data array
        grid: 2D occupancy grid
        north_offset: North offset of grid origin
        east_offset: East offset of grid origin
        path_original: Original path from A*
        path_pruned: Pruned path with reduced waypoints
        grid_start: Start position in grid coordinates
        grid_goal: Goal position in grid coordinates
        save_path: Path to save comparison image
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(28, 12))

    for ax, path, title in [(ax1, path_original, 'Original Path'),
                             (ax2, path_pruned, 'Pruned Path')]:
        # Plot components
        plot_obstacles(ax, data)
        plot_path(ax, path, north_offset, east_offset)
        plot_markers(ax, grid_start, grid_goal, north_offset, east_offset)

        # Format
        format_axis(ax, f'{title}\nWaypoints: {len(path)}')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Path comparison saved to: {save_path}")
    plt.show()


def main():
    """Main function for testing visualization."""
    # Configuration
    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5

    # Load data
    print("Loading colliders data...")
    data = np.loadtxt('../colliders.csv', delimiter=',', dtype='float64', skiprows=2)

    # Create grid
    print("Creating grid...")
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
    print(f"Grid shape: {grid.shape}")
    print(f"Offsets - North: {north_offset}, East: {east_offset}")

    # Define start and goal
    grid_start = (316, 445)
    grid_goal = (427, 532)

    # Validate start and goal positions using the abstracted method
    grid_start = validate_position(grid_start, grid, "start")
    grid_goal = validate_position(grid_goal, grid, "goal")

    # Get custom point from user
    custom_point = (427, 532)

    # Find path
    print(f"\nFinding path from {grid_start} to {grid_goal}...")
    path, path_cost = a_star(grid, heuristic, grid_start, grid_goal)

    if len(path) == 0:
        print("No path found!")
        return

    # Prune path
    path_pruned = prune_path(path)

    print(f"Original path: {len(path)} waypoints")
    print(f"Pruned path: {len(path_pruned)} waypoints ({len(path_pruned)/len(path)*100:.1f}%)")
    print(f"Path cost: {path_cost:.2f}")

    # Visualize
    print("\nGenerating visualizations...")
    visualize_path_2d(data, grid, north_offset, east_offset, path,
                      grid_start, grid_goal, path_cost, custom_point=custom_point)
    visualize_path_comparison(data, grid, north_offset, east_offset,
                              path, path_pruned, grid_start, grid_goal)

    print("\nDone!")


if __name__ == '__main__':
    main()
