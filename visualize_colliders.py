"""
Visualize the colliders.csv obstacle map in 2D and 3D
Shows home position, obstacles, and grid boundaries
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_colliders(filename='colliders.csv'):
    """
    Load colliders data and extract home position

    Returns:
        data: numpy array of obstacle data [posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ]
        lat0: home latitude
        lon0: home longitude
    """
    # Read the first line to get home position
    with open(filename, 'r') as f:
        first_line = f.readline()

    # Parse: "lat0 37.792480, lon0 -122.397450"
    parts = first_line.strip().split(',')
    lat0 = float(parts[0].split()[1])
    lon0 = float(parts[1].split()[1])

    # Load obstacle data (skip first 2 lines: home + header)
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

    print(f"Home Position: lat={lat0}, lon={lon0}")
    print(f"Loaded {data.shape[0]} obstacles")

    return data, lat0, lon0


def calculate_boundaries(data, safety_distance=5):
    """
    Calculate grid boundaries

    Returns:
        north_min, north_max, east_min, east_max
    """
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    return north_min, north_max, east_min, east_max


def visualize_2d(data, lat0, lon0, save_path='colliders_2d.png'):
    """
    Create 2D top-down view of obstacles
    """
    fig, ax = plt.subplots(figsize=(14, 12))

    # Calculate boundaries
    north_min, north_max, east_min, east_max = calculate_boundaries(data)

    # Plot each obstacle as a rectangle
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Calculate rectangle corners
        n_min = north - d_north
        n_max = north + d_north
        e_min = east - d_east
        e_max = east + d_east

        # Color by height
        height = alt + d_alt
        color = plt.cm.viridis(height / 200)  # Normalize by max expected height

        # Draw rectangle
        width = e_max - e_min
        height_rect = n_max - n_min
        rect = plt.Rectangle((e_min, n_min), width, height_rect,
                            facecolor=color, edgecolor='black', linewidth=0.5, alpha=0.7)
        ax.add_patch(rect)

    # Plot home position (0, 0 in NED coordinates)
    ax.plot(0, 0, 'r*', markersize=20, label='Home', zorder=10)
    ax.annotate('HOME\n(0, 0)', xy=(0, 0), xytext=(50, 50),
                fontsize=12, color='red', weight='bold',
                arrowprops=dict(arrowstyle='->', color='red', lw=2))

    # Plot boundary corners
    boundary_points = [
        (east_min, north_min, 'SW'),
        (east_max, north_min, 'SE'),
        (east_min, north_max, 'NW'),
        (east_max, north_max, 'NE')
    ]

    for e, n, label in boundary_points:
        ax.plot(e, n, 'bo', markersize=10, zorder=10)
        ax.annotate(f'{label}\n({e:.0f}, {n:.0f})',
                   xy=(e, n), xytext=(10, 10), textcoords='offset points',
                   fontsize=9, color='blue', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

    # Draw boundary box
    boundary_box = plt.Rectangle((east_min, north_min),
                                east_max - east_min,
                                north_max - north_min,
                                fill=False, edgecolor='blue', linewidth=2,
                                linestyle='--', label='Grid Boundary')
    ax.add_patch(boundary_box)

    # Add boundary text
    ax.text(0.02, 0.98, f'north_min = {north_min:.0f}\nnorth_max = {north_max:.0f}\n'
                        f'east_min = {east_min:.0f}\neast_max = {east_max:.0f}\n'
                        f'Grid size: {int(north_max-north_min)} × {int(east_max-east_min)}',
            transform=ax.transAxes, fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    # Formatting
    ax.set_xlabel('East (meters)', fontsize=12, weight='bold')
    ax.set_ylabel('North (meters)', fontsize=12, weight='bold')
    ax.set_title('2D Top-Down View of Obstacles\n(Color indicates height)',
                fontsize=14, weight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)

    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis,
                               norm=plt.Normalize(vmin=0, vmax=200))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, label='Height (meters)')

    # Add compass
    arrow_len = (east_max - east_min) * 0.1
    compass_e = east_max - (east_max - east_min) * 0.15
    compass_n = north_max - (north_max - north_min) * 0.15
    ax.arrow(compass_e, compass_n, 0, arrow_len, head_width=20, head_length=15,
            fc='red', ec='red', linewidth=2, zorder=15)
    ax.text(compass_e, compass_n + arrow_len + 30, 'N', fontsize=14,
           weight='bold', color='red', ha='center', zorder=15)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"2D visualization saved to: {save_path}")
    plt.close()


def visualize_3d(data, lat0, lon0, save_path='colliders_3d.png'):
    """
    Create 3D view of obstacles
    """
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Calculate boundaries
    north_min, north_max, east_min, east_max = calculate_boundaries(data)

    # Sample obstacles for 3D (plotting all can be slow)
    # Plot every Nth obstacle or all if dataset is small
    step = max(1, data.shape[0] // 500)  # Limit to ~500 obstacles

    for i in range(0, data.shape[0], step):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        # Define the 8 corners of the box
        n_min, n_max = north - d_north, north + d_north
        e_min, e_max = east - d_east, east + d_east
        z_min, z_max = 0, alt + d_alt  # Obstacles start from ground

        # Create the vertices of the box
        vertices = [
            [n_min, e_min, z_min], [n_max, e_min, z_min],
            [n_max, e_max, z_min], [n_min, e_max, z_min],  # Bottom face
            [n_min, e_min, z_max], [n_max, e_min, z_max],
            [n_max, e_max, z_max], [n_min, e_max, z_max]   # Top face
        ]

        # Define the 6 faces of the box
        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Right
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top
            [vertices[0], vertices[1], vertices[2], vertices[3]]   # Bottom
        ]

        # Color by height
        color = plt.cm.viridis(z_max / 200)

        # Add the box to the plot
        poly = Poly3DCollection(faces, facecolors=color, linewidths=0.1,
                               edgecolors='black', alpha=0.6)
        ax.add_collection3d(poly)

    # Plot home position
    ax.scatter([0], [0], [0], c='red', marker='*', s=500,
              label='Home', depthshade=False, zorder=100)

    # Plot boundary corners on ground
    boundary_points = [
        (north_min, east_min, 0, 'SW'),
        (north_max, east_min, 0, 'SE'),
        (north_min, east_max, 0, 'NW'),
        (north_max, east_max, 0, 'NE')
    ]

    for n, e, z, label in boundary_points:
        ax.scatter([n], [e], [z], c='blue', marker='o', s=200, depthshade=False)
        ax.text(n, e, z + 50, label, fontsize=9, color='blue', weight='bold')

    # Draw boundary box on ground
    boundary_corners = [
        [north_min, east_min, 0], [north_max, east_min, 0],
        [north_max, east_max, 0], [north_min, east_max, 0],
        [north_min, east_min, 0]  # Close the loop
    ]
    boundary_array = np.array(boundary_corners)
    ax.plot(boundary_array[:, 0], boundary_array[:, 1], boundary_array[:, 2],
           'b--', linewidth=2, label='Grid Boundary')

    # Formatting
    ax.set_xlabel('North (meters)', fontsize=11, weight='bold')
    ax.set_ylabel('East (meters)', fontsize=11, weight='bold')
    ax.set_zlabel('Altitude (meters)', fontsize=11, weight='bold')
    ax.set_title('3D View of Obstacles', fontsize=14, weight='bold')

    # Set equal aspect ratio
    max_range = max(north_max - north_min, east_max - east_min) / 2
    mid_n = (north_max + north_min) / 2
    mid_e = (east_max + east_min) / 2
    ax.set_xlim(mid_n - max_range, mid_n + max_range)
    ax.set_ylim(mid_e - max_range, mid_e + max_range)
    ax.set_zlim(0, 200)

    # Add info text
    info_text = f'Obstacles shown: {data.shape[0] // step} of {data.shape[0]}\n'
    info_text += f'north: [{north_min:.0f}, {north_max:.0f}]\n'
    info_text += f'east: [{east_min:.0f}, {east_max:.0f}]'
    ax.text2D(0.02, 0.98, info_text, transform=ax.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    ax.legend(loc='upper right', fontsize=10)
    ax.view_init(elev=30, azim=45)  # Set viewing angle

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"3D visualization saved to: {save_path}")
    plt.close()


def main():
    """
    Main function to generate both 2D and 3D visualizations
    """
    print("Loading colliders data...")
    data, lat0, lon0 = load_colliders('colliders.csv')

    print("\nGenerating 2D visualization...")
    visualize_2d(data, lat0, lon0, save_path='Logs/colliders_2d.png')

    print("\nGenerating 3D visualization...")
    visualize_3d(data, lat0, lon0, save_path='Logs/colliders_3d.png')

    print("\n" + "="*50)
    print("Visualization complete!")
    print("Files saved:")
    print("  - colliders_2d.png (top-down view)")
    print("  - colliders_3d.png (3D perspective)")
    print("="*50)


if __name__ == '__main__':
    main()
