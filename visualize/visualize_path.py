"""
Path visualization utilities for drone motion planning.
Provides functions to visualize planned paths on 2D obstacle maps.
"""

import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Standard imports - works after installing package with 'pip install -e .'
from planning_utils import create_grid, a_star, heuristic, prune_path, validate_position, prune_path_bresenham


def get_project_path(*paths):
    """
    Get absolute path relative to project root directory.
    This ensures files can be found regardless of where the script is run from.

    Args:
        *paths: Path components to join (e.g., 'Logs', 'output.png')

    Returns:
        Absolute path to the file

    Example:
        path = get_project_path('Logs', 'path.png')
        path.endswith(os.path.join('FCND-Motion-Planning', 'Logs', 'path.png'))
    """
    # Get the directory where this script is located (visualize/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Project root is parent directory
    project_root = os.path.dirname(script_dir)
    # Join with provided paths
    return os.path.join(project_root, *paths)


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


def plot_intermediate_waypoints(ax, intermediate_waypoints, north_offset, east_offset):
    """
    Plot intermediate waypoints as distinct markers.

    Args:
        ax: Matplotlib axis
        intermediate_waypoints: List of waypoint tuples in grid coordinates
        north_offset: North offset of grid origin
        east_offset: East offset of grid origin
    """
    if intermediate_waypoints is None or len(intermediate_waypoints) == 0:
        return

    for i, wp in enumerate(intermediate_waypoints):
        wp_north, wp_east = grid_to_local(wp, (north_offset, east_offset))

        # Plot waypoint marker (orange diamond, smaller size)
        ax.plot(wp_east, wp_north, 'D', markersize=6,
                color='orange', markeredgecolor='darkorange', markeredgewidth=1.5,
                label='Intermediate WP' if i == 0 else '', zorder=11)


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
                      grid_goal, path_cost=None, save_path=None,
                      custom_point=None, intermediate_waypoints=None, show=True):
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
        save_path: Path to save visualization image (default: Logs/path_visualization.png)
        custom_point: Optional custom point to mark on the map (grid coordinates)
        intermediate_waypoints: Optional list of intermediate waypoints in grid coordinates
        show: Whether to display the plot window (default: True). Set to False to only save the image.
    """
    if save_path is None:
        save_path = get_project_path('Logs', 'path_visualization.png')

    # Reduced figure size for faster saving (was 16x14)
    fig, ax = plt.subplots(figsize=(12, 10))

    # Calculate boundaries
    north_min, north_max, east_min, east_max = calculate_boundaries(data)

    # Plot components
    plot_obstacles(ax, data)
    plot_path(ax, path, north_offset, east_offset, label='Path')
    plot_markers(ax, grid_start, grid_goal, north_offset, east_offset, custom_point=custom_point)
    plot_intermediate_waypoints(ax, intermediate_waypoints, north_offset, east_offset)
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
        if intermediate_waypoints is not None and len(intermediate_waypoints) > 0:
            info_lines.append(f'- Intermediate WPs: {len(intermediate_waypoints)}')
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
    plt.savefig(save_path, dpi=500, bbox_inches='tight')
    print(f"Path visualization saved to: {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)  # Close the figure to free memory when not showing


def visualize_path_comparison(data, grid, north_offset, east_offset,
                               path_original, path_pruned, grid_start, grid_goal,
                               save_path=None):
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
        save_path: Path to save comparison image (default: Logs/path_comparison.png)
    """
    if save_path is None:
        save_path = get_project_path('Logs', 'path_comparison.png')

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


def visualize_path_comparison_3methods(grid, path_original, path_collinear, path_bresenham,
                                        grid_start, grid_goal, north_offset=0, east_offset=0,
                                        save_path=None):
    """
    Compare three path pruning methods: original, collinear, and Bresenham.

    Args:
        grid: 2D occupancy grid
        path_original: Original path from A*
        path_collinear: Path pruned using collinearity test
        path_bresenham: Path pruned using Bresenham algorithm
        grid_start: Start position in grid coordinates
        grid_goal: Goal position in grid coordinates
        north_offset: North offset of grid origin
        east_offset: East offset of grid origin
        save_path: Path to save comparison image (default: Logs/path_comparison_3methods.png)
    """
    if save_path is None:
        save_path = get_project_path('Logs', 'path_comparison_3methods.png')

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))

    # Prepare data for all three methods
    methods = [
        (ax1, path_original, 'Original Path', 'blue', 'red', 'o', 20),
        (ax2, path_collinear, 'Collinear Pruned Path', 'green', 'blue', 'o', 50),
        (ax3, path_bresenham, 'Bresenham Pruned Path', 'purple', 'red', 's', 100)
    ]

    for ax, path, title, line_color, marker_color, marker_shape, marker_size in methods:
        if len(path) == 0:
            continue

        # Extract coordinates (North, East)
        path_north = [p[0] + north_offset for p in path]
        path_east = [p[1] + east_offset for p in path]

        # Plot path line
        ax.plot(path_east, path_north, color=line_color, linestyle='-',
                linewidth=2 if title == 'Original Path' else 3,
                alpha=0.5 if title == 'Original Path' else 0.7, label='Path')

        # Plot waypoints
        ax.scatter(path_east, path_north, c=marker_color, s=marker_size, zorder=5,
                   marker=marker_shape, edgecolors='black',
                   linewidth=1.5 if marker_size >= 50 else 0,
                   label=f'Waypoints ({len(path)})')

        # Plot start marker
        start_north, start_east = grid_to_local(grid_start, (north_offset, east_offset))
        ax.scatter(start_east, start_north, c='green', s=200, marker='o',
                   edgecolors='black', linewidth=2, zorder=10, label='Start')

        # Plot goal marker
        goal_north, goal_east = grid_to_local(grid_goal, (north_offset, east_offset))
        ax.scatter(goal_east, goal_north, c='orange', s=200, marker='*',
                   edgecolors='black', linewidth=2, zorder=10, label='Goal')

        # Format axis
        ax.set_xlabel('East (grid units)', fontsize=12)
        ax.set_ylabel('North (grid units)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=10)
        ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n3-method comparison saved to: {save_path}")

    # Print statistics comparison
    print(f"\n{'='*70}")
    print(f"Path Optimization Statistics Comparison")
    print(f"{'='*70}")
    print(f"{'Method':<20} {'Waypoints':<12} {'Reduction':<15} {'Optimization %':<15}")
    print(f"{'-'*70}")
    print(f"{'Original Path':<20} {len(path_original):>8} pts {'-':>13} {'-':>15}")

    collinear_reduction = len(path_original) - len(path_collinear)
    collinear_percent = (1 - len(path_collinear) / len(path_original)) * 100
    print(f"{'Collinear Pruning':<20} {len(path_collinear):>8} pts "
          f"{collinear_reduction:>10} pts {collinear_percent:>14.1f}%")

    bresenham_reduction = len(path_original) - len(path_bresenham)
    bresenham_percent = (1 - len(path_bresenham) / len(path_original)) * 100
    print(f"{'Bresenham Pruning':<20} {len(path_bresenham):>8} pts "
          f"{bresenham_reduction:>10} pts {bresenham_percent:>14.1f}%")
    print(f"{'='*70}")

    # Compare pruning methods
    print(f"\nPruning Methods Comparison:")
    print(f"Bresenham vs Collinear:")
    if len(path_bresenham) < len(path_collinear):
        additional_reduction = len(path_collinear) - len(path_bresenham)
        additional_percent = (1 - len(path_bresenham) / len(path_collinear)) * 100
        print(f"  - Additional reduction: {additional_reduction} waypoints")
        print(f"  - Additional optimization: {additional_percent:.1f}%")
        print(f"  >> Bresenham method is more efficient!")
    elif len(path_bresenham) > len(path_collinear):
        print(f"  Note: Bresenham has more waypoints ({len(path_bresenham)} vs {len(path_collinear)})")
    else:
        print(f"  Both methods produce the same result")

    plt.show()


def load_planning_config():
    """
    Load planning configuration from saved JSON file.
    If file doesn't exist, return default configuration.
    """
    import json

    planning_data_path = get_project_path('Logs', 'planning_data.json')

    if not os.path.exists(planning_data_path):
        print(f"No saved planning data found at {planning_data_path}")
        print("Using default configuration...")

        # Default configuration (grid coordinates for example scenario)
        config = {
            'grid_start': (316, 445),
            'grid_goal': (427, 532),
            'intermediate_waypoints_grid': None,
            'north_offset': -316,
            'east_offset': -445,
            'use_multi_waypoint': False,
            'target_altitude': 5,
            'safety_distance': 5,
            'timestamp': 'default'
        }
        return config

    with open(planning_data_path, 'r') as f:
        config = json.load(f)

    # Convert lists back to tuples
    config['grid_start'] = tuple(config['grid_start'])
    config['grid_goal'] = tuple(config['grid_goal'])
    if config['intermediate_waypoints_grid']:
        config['intermediate_waypoints_grid'] = [tuple(wp) for wp in config['intermediate_waypoints_grid']]

    print(f"Loaded planning configuration from: {planning_data_path}")
    print(f"Timestamp: {config['timestamp']}")

    return config


def main():
    """Main function for path visualization."""
    # Load colliders data
    print("Loading colliders data...")
    colliders_path = get_project_path('colliders.csv')
    data = np.loadtxt(colliders_path, delimiter=',', dtype='float64', skiprows=2)

    # Load planning configuration (from file or use defaults)
    print("\n" + "="*70)
    config = load_planning_config()
    print("="*70)

    # Extract configuration
    grid_start = config['grid_start']
    grid_goal = config['grid_goal']
    intermediate_waypoints_grid = config['intermediate_waypoints_grid']
    north_offset = config['north_offset']
    east_offset = config['east_offset']
    use_multi_waypoint = config['use_multi_waypoint']
    TARGET_ALTITUDE = config['target_altitude']
    SAFETY_DISTANCE = config['safety_distance']

    print(f"\nConfiguration:")
    print(f"  - Grid start: {grid_start}")
    print(f"  - Grid goal: {grid_goal}")
    print(f"  - Multi-waypoint: {use_multi_waypoint}")
    print(f"  - Target altitude: {TARGET_ALTITUDE}m")
    print(f"  - Safety distance: {SAFETY_DISTANCE}m")
    if intermediate_waypoints_grid:
        print(f"  - Intermediate waypoints: {len(intermediate_waypoints_grid)}")

    # Create grid
    print("\nCreating grid...")
    grid, calc_north_offset, calc_east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
    print(f"Grid shape: {grid.shape}")
    print(f"Offsets - North: {calc_north_offset}, East: {calc_east_offset}")

    # Verify offsets match (they should always match for same colliders.csv)
    if calc_north_offset != north_offset or calc_east_offset != east_offset:
        print(f"Warning: Calculated offsets ({calc_north_offset}, {calc_east_offset}) "
              f"differ from saved offsets ({north_offset}, {east_offset})")
        print("Using calculated offsets...")
        north_offset = calc_north_offset
        east_offset = calc_east_offset

    # Validate positions
    grid_start = validate_position(grid_start, grid, "start")
    grid_goal = validate_position(grid_goal, grid, "goal")

    if intermediate_waypoints_grid:
        validated_waypoints = []
        for i, wp_grid in enumerate(intermediate_waypoints_grid):
            wp_grid = validate_position(wp_grid, grid, f"intermediate_waypoint_{i+1}")
            validated_waypoints.append(wp_grid)
        intermediate_waypoints_grid = validated_waypoints

    # Execute path planning
    print(f"\nExecuting path planning...")
    if use_multi_waypoint and intermediate_waypoints_grid:
        from planning_utils import a_star_multi_waypoints, heuristic_diagonal
        path, _ = a_star_multi_waypoints(grid, heuristic_diagonal, grid_start, grid_goal,
                                         intermediate_waypoints_grid, prune_segments=True)
        print(f"Multi-waypoint path length: {len(path)} waypoints")
    else:
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print(f"Original path length: {len(path)} waypoints")
        path = prune_path_bresenham(grid, path)
        print(f"Pruned path length: {len(path)} waypoints")

    # Generate visualization
    print("\nGenerating visualization...")
    visualize_path_2d(data, grid, north_offset, east_offset, path,
                     grid_start, grid_goal,
                     intermediate_waypoints=intermediate_waypoints_grid,
                     show=True)
    print("Done!")


if __name__ == '__main__':
    main()
