import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def create_box(center, half_size):
    """Create vertices for a 3D box"""
    cx, cy, cz = center
    dx, dy, dz = half_size

    vertices = [
        [cx-dx, cy-dy, cz-dz], [cx+dx, cy-dy, cz-dz],
        [cx+dx, cy+dy, cz-dz], [cx-dx, cy+dy, cz-dz],
        [cx-dx, cy-dy, cz+dz], [cx+dx, cy-dy, cz+dz],
        [cx+dx, cy+dy, cz+dz], [cx-dx, cy+dy, cz+dz]
    ]

    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[7], vertices[6], vertices[2], vertices[3]],
        [vertices[0], vertices[3], vertices[7], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]]
    ]

    return faces

# Read obstacle data
data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)

# Configuration
TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

# Create figure
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot obstacles
obstacle_count = 0
for i, obstacle in enumerate(data):
    posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ = obstacle

    # Only show obstacles near the center and that affect flight
    if abs(posX) < 200 and abs(posY) < 200 and posZ + halfSizeZ >= TARGET_ALTITUDE:
        center = [posX, posY, posZ]
        half_size = [halfSizeX, halfSizeY, halfSizeZ]

        faces = create_box(center, half_size)

        # Color by height
        height_ratio = posZ / 100.0
        color = plt.cm.viridis(min(height_ratio, 1.0))

        poly = Poly3DCollection(faces, alpha=0.3, facecolor=color, edgecolor='black', linewidth=0.5)
        ax.add_collection3d(poly)
        obstacle_count += 1

# Mark start and goal
start_pos = [0, 0, 0]  # Map center
goal_pos = [10, 10, TARGET_ALTITUDE]

ax.scatter(*start_pos, color='green', s=100, marker='o', label='Start')
ax.scatter(*goal_pos, color='red', s=100, marker='*', label='Goal')

# Plot flight altitude plane
xx, yy = np.meshgrid(range(-200, 200, 50), range(-200, 200, 50))
zz = np.ones_like(xx) * TARGET_ALTITUDE
ax.plot_surface(xx, yy, zz, alpha=0.1, color='cyan')

# Labels and settings
ax.set_xlabel('North (m)')
ax.set_ylabel('East (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title(f'3D Obstacle Visualization\n(Flight Altitude: {TARGET_ALTITUDE}m, Showing {obstacle_count} obstacles)')
ax.legend()

# Set reasonable viewing limits
ax.set_xlim([-200, 200])
ax.set_ylim([-200, 200])
ax.set_zlim([0, 150])

# Better viewing angle
ax.view_init(elev=30, azim=45)

# Enable interactive rotation and zooming
ax.mouse_init()

print(f"\nDisplaying {obstacle_count} obstacles affecting flight at {TARGET_ALTITUDE}m altitude")
print("\n3D Interaction Controls:")
print("  - Left mouse button + drag: Rotate view")
print("  - Right mouse button + drag: Zoom in/out")
print("  - Middle mouse button + drag: Pan view")
print("  - Scroll wheel: Zoom in/out\n")

plt.tight_layout()
plt.show()
