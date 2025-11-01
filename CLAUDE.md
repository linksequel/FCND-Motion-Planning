# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the FCND (Flying Car Nanodegree) Motion Planning project, which implements autonomous drone path planning in an urban environment using the Udacity drone simulator. The drone must navigate from a start position to a goal position while avoiding obstacles defined in a 2.5D map.

## Installation (Industry Best Practice)

This project follows Python packaging best practices for production environments. Instead of using `sys.path.insert()` hacks, the project is installed as a proper Python package.

### Quick Setup

```bash
# Clone the repository (if not already done)
cd FCND-Motion-Planning

# Install the project in editable mode
pip install -e .

# Or install with development dependencies
pip install -e ".[dev]"
```

### What This Does

- **Editable mode (`-e`)**: Changes to source files are immediately reflected without reinstalling
- **Proper module imports**: All modules (`planning_utils`, `visualize`, etc.) can be imported from anywhere
- **No path hacks**: Eliminates the need for `sys.path.insert()` or `PYTHONPATH` manipulation
- **Production-ready**: Same approach used in professional Python projects and deployed services

### Alternative: Manual Dependencies

If you prefer not to install the package:

```bash
pip install numpy matplotlib udacidrone
```

Then run scripts from the project root directory.

## Running the Code

### Prerequisites
- Python 3.6+ environment (Python 3.6.3+ tested and working)
- Motion Planning simulator from [FCND-Simulator-Releases](https://github.com/udacity/FCND-Simulator-Releases/releases)

### Running the Motion Planner
```bash
python motion_planning.py
```

Optional arguments:
- `--port`: Port number (default: 5760)
- `--host`: Host address (default: '127.0.0.1')

Example with custom connection:
```bash
python motion_planning.py --host 127.0.0.1 --port 5760
```

### Testing with Backyard Flyer
To test the basic drone control setup:
```bash
python backyard_flyer_solution.py
```

## Architecture

### Core Files

**motion_planning.py** - Main motion planning script that extends the Udacity Drone class
- Implements a state machine with states: MANUAL, ARMING, TAKEOFF, PLANNING, WAYPOINT, LANDING, DISARMING
- The `plan_path()` method (line 114) is the core planning function that:
  1. Loads obstacle data from `colliders.csv`
  2. Creates a 2D grid representation of the environment
  3. Runs A* search to find a collision-free path
  4. Converts the path to waypoints in local ECEF coordinates [N, E, altitude, heading]
- State transitions are managed through callback functions registered to Mavlink messages

**planning_utils.py** - Path planning utilities
- `create_grid()`: Converts obstacle data into a 2D occupancy grid with configurable drone altitude and safety distance
- `Action` enum: Defines possible movements (NORTH, SOUTH, EAST, WEST) with associated costs
- `valid_actions()`: Returns valid actions from a node by checking grid boundaries and obstacles
- `a_star()`: Implements A* pathfinding algorithm using a priority queue
- `heuristic()`: Euclidean distance heuristic for A* search

**colliders.csv** - Environment obstacle map
- First line contains the global home position: `lat0 37.792480, lon0 -122.397450`
- Subsequent lines define obstacles as: posX, posY, posZ, halfSizeX, halfSizeY, halfSizeZ
- Loaded with `np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)`

### Key Architectural Patterns

**State Machine Pattern**: The drone operates as a finite state machine, transitioning through predefined states based on callbacks from the drone's sensor data and status updates.

**Callback Registration**: The system uses event-driven programming where callbacks are registered for specific message types (LOCAL_POSITION, LOCAL_VELOCITY, STATE) from the Mavlink connection.

**Grid-Based Planning**: The environment is discretized into a 2D grid where obstacles are marked as 1 and free space as 0. The A* algorithm searches this grid for collision-free paths.

**Coordinate Systems**:
- Global coordinates: Latitude/longitude from GPS
- Local coordinates: North-East-Down (NED) frame relative to home position
- Grid coordinates: Integer indices into the occupancy grid with north/east offsets

### Connection to Backyard Flyer

The motion planning implementation extends the basic waypoint navigation from `backyard_flyer_solution.py`. The key differences:
- Backyard Flyer uses predefined waypoints in `calculate_box()`
- Motion Planning adds a PLANNING state and `plan_path()` method that dynamically computes waypoints using A* search
- Motion Planning loads and processes obstacle data to ensure collision-free paths

## Key TODOs in the Code

The `motion_planning.py` file contains several TODO comments marking areas for improvement:
1. Line 122: Read lat0, lon0 from colliders.csv first line 第 122 行：从第一行读取 lat0、lon0 colliders.csv 
2. Line 124: Set home position to (lon0, lat0, 0) 第 124 行：将起始位置设置为 （lon0， lat0， 0） 
3. Line 126: Retrieve current global position 第 126 行：检索当前全局位置 
4. Line 128: Convert to current local position using global_to_local() 第128 行：使用 global_to_local（） 转换为当前本地位置 
5. Line 140: Convert start position to current position rather than map center  第 140 行：将起始位置转换为当前位置而不是地图中心 
6. Line 144: Adapt to set goal as latitude/longitude position and convert 第 144 行：适应将目标设置为纬度/经度位置并转换 
7. Line 147: Add diagonal motions with cost of sqrt(2) to A* implementation 第 147 行：将成本为 sqrt（2） 的对角线运动添加到 A* 实现中 
8. Line 151: Prune path to minimize number of waypoints (e.g., collinearity test or Bresenham) 第 151 行：修剪路径以尽量减少航路点数量（例如，共线性测试或 Bresenham） 
9. Line 158: Send waypoints to simulator for visualization 第 158 行：将航点发送到模拟器以进行可视化

## Extending the Planner

Common enhancements:
- Add diagonal actions to the Action enum with cost sqrt(2)
- Implement path pruning using collinearity test or ray tracing (Bresenham)
- Use global_to_local() to convert GPS coordinates to local grid positions
- Implement 3D path planning by varying altitude with waypoints
- Adjust deadbands in local_position_callback() for smoother flight
- Add heading commands to waypoints based on direction of travel
