import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import (a_star, heuristic, heuristic_diagonal, heuristic_manhattan,
                            heuristic_chebyshev, create_grid, validate_position,
                            prune_path_bresenham, a_star_multi_waypoints)
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

"""
Hints:
1.通过调整 grid_start 和 grid_goal 参数的值，您可以设置无人机的起点和终点。
"""
class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        #监控起飞状态，当达到目标高度时转换到航路点状态waypoint_transition;
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        #在航路点状态中，检查是否到达目标点，如果到达则前往下一个航路点或准备降落
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        #在降落状态中，当无人机接近地面时转换到解除武装状态
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        #控制状态转换流程：手动→武装→规划→起飞→航路点→降落→解除武装→手动
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE
        '''
        相当于存在三个坐标系：
        1.经纬度坐标系
        2.colliders.csv中的以home为中心的相对坐标系（local_position、north min的概念都是基于此坐标系） 符合地理方位（上北下南左西右东）
        3.网格坐标系grid 这里的grid（0，0）应该是(north_min, east_min) 抽象二维数组吗，跟地理方位无关了，行增加代表更北方，列增加代表更东方
        '''

        # DONE: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline()
        lat0, lon0 = first_line.replace('lat0', '').replace('lon0', '').replace(',', '').split()
        lat0, lon0 = float(lat0), float(lon0)

        # DONE: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # DONE: retrieve current global position
        # 无人机现在所处的位置（经纬度）
        global_position = self.global_position

        # DONE: convert to current local position using global_to_local()
        # 当前位置经纬度 -> 相对home的loca坐标（单位为米）
        relative_pos_from_home = global_to_local(global_position, self.global_home)

        print('home global:{0}, current position:{1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # 两者理论上应该非常接近（都表示无人机相对于home点的位置），但self.local_position是实时更新的传感器数据，而global_to_local()是基于GPS坐标的手动计算结果。
        print("should be closer param:", relative_pos_from_home - self.local_position)
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        # north_offset也是相对home，单位为米
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # DONE: convert start position to current position rather than map center
        grid_start = (int(relative_pos_from_home[0]) - north_offset, int(relative_pos_from_home[1]) - east_offset)

        # Set goal as some arbitrary position on the grid
        # DONE: adapt to set YOUR goal as latitude / longitude position and convert
        goal_lat = 37.793480  # Example latitude (about 100m north)
        goal_lon = -122.396450  # Example longitude (about 100m east)
        goal_global = [goal_lon, goal_lat, 0]
        goal_local = global_to_local(goal_global, self.global_home)
        grid_goal = (int(goal_local[0]) - north_offset, int(goal_local[1]) - east_offset)

        # Validate start and goal positions
        print("Grid shape:", grid.shape)
        print("Grid start:", grid_start)
        print("Grid goal:", grid_goal)

        # Validate start and goal positions using the abstracted method
        grid_start = validate_position(grid_start, grid, "start")
        grid_goal = validate_position(grid_goal, grid, "goal")

        # ============================================================
        # HW_1 Question 5: Multi-waypoint path planning with different heuristics
        # ============================================================

        # Define three fixed intermediate waypoints (in lon/lat coordinates)
        # IMPORTANT: Format is [longitude, latitude, altitude] to match global_to_local() requirements
        # These points are chosen to create an interesting path through the environment
        intermediate_waypoints_global = [
            [-122.397250, 37.792980, 0],  # Waypoint 1: slightly north and west
            [-122.396950, 37.793280, 0],  # Waypoint 2: further north and east
            [-122.396650, 37.793380, 0],  # Waypoint 3: even further north and east
        ]

        # Convert intermediate waypoints from global to grid coordinates
        intermediate_waypoints_grid = []
        for i, wp_global in enumerate(intermediate_waypoints_global):
            wp_local = global_to_local(wp_global, self.global_home)
            wp_grid = (int(wp_local[0]) - north_offset, int(wp_local[1]) - east_offset)
            wp_grid = validate_position(wp_grid, grid, f"intermediate_waypoint_{i+1}")
            intermediate_waypoints_grid.append(wp_grid)
            print(f"Intermediate waypoint {i+1}: Global (lon={wp_global[0]:.6f}, lat={wp_global[1]:.6f}) -> Grid {wp_grid}")

        # Choose whether to use multi-waypoint planning or standard A*
        USE_MULTI_WAYPOINT = True  # Set True to use multi-waypoint planning, False for standard A*

        # Choose heuristic function
        # Options: heuristic (Euclidean), heuristic_manhattan, heuristic_diagonal, heuristic_chebyshev
        chosen_heuristic = heuristic_diagonal  # Best for 8-directional movement
        print(f"Using heuristic: {chosen_heuristic.__name__}")

        # Run A* to find a path from start to goal
        # DONE: add diagonal motions with a cost of sqrt(2) to your A* implementation 在planning_utils.Action中实现
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)

        if USE_MULTI_WAYPOINT:
            print("\n=== Multi-Waypoint Path Planning ===")
            print(f"Will traverse {len(intermediate_waypoints_grid)} intermediate waypoints")
            # Note: a_star_multi_waypoints() prunes each segment internally to preserve waypoints
            # Setting prune_segments=True ensures intermediate waypoints are NOT removed
            path, _ = a_star_multi_waypoints(grid, chosen_heuristic, grid_start, grid_goal,
                                             intermediate_waypoints_grid, prune_segments=True)
            print('Final multi-waypoint path length:', len(path))
            print('⚠️ Path preserves all intermediate waypoints (not pruned globally)')
        else:
            print("\n=== Standard A* Path Planning ===")
            path, _ = a_star(grid, chosen_heuristic, grid_start, grid_goal)
            # DONE: prune path to minimize number of waypoints
            # ? (if you're feeling ambitious): Try a different approach altogether!
            print('Original path length:', len(path))
            path = prune_path_bresenham(grid, path)
            print('Pruned path length:', len(path))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # DONE: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
