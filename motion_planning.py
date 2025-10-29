import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
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

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline()
        lat0, lon0 = first_line.replace('lat0', '').replace('lon0', '').replace(',', '').split()
        lat0, lon0 = float(lat0), float(lon0)

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        global_position = self.global_position

        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_position[0]) - north_offset, int(local_position[1]) - east_offset)

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        # Example: Set goal to (lat, lon, alt) and convert to local coordinates
        # For now using an arbitrary position, but you can set goal_lat, goal_lon and convert
        goal_lat = 37.793480  # Example latitude (about 100m north)
        goal_lon = -122.396450  # Example longitude (about 100m east)
        goal_global = [goal_lon, goal_lat, 0]
        goal_local = global_to_local(goal_global, self.global_home)
        grid_goal = (int(goal_local[0]) - north_offset, int(goal_local[1]) - east_offset)

        # Validate start and goal positions
        print("Grid shape:", grid.shape)
        print("Grid start:", grid_start)
        print("Grid goal:", grid_goal)

        # Check if start is valid
        if grid_start[0] < 0 or grid_start[0] >= grid.shape[0] or grid_start[1] < 0 or grid_start[1] >= grid.shape[1]:
            print("WARNING: Start position is outside grid bounds!")
            grid_start = (int(grid.shape[0] / 2), int(grid.shape[1] / 2))
            print("Using grid center as start:", grid_start)
        elif grid[grid_start[0], grid_start[1]] == 1:
            print("WARNING: Start position is inside an obstacle!")
            # Find nearest free space
            for offset in range(1, 20):
                for dx in range(-offset, offset+1):
                    for dy in range(-offset, offset+1):
                        new_start = (grid_start[0] + dx, grid_start[1] + dy)
                        if (0 <= new_start[0] < grid.shape[0] and
                            0 <= new_start[1] < grid.shape[1] and
                            grid[new_start[0], new_start[1]] == 0):
                            grid_start = new_start
                            print("Adjusted start to:", grid_start)
                            break
                    if grid[grid_start[0], grid_start[1]] == 0:
                        break
                if grid[grid_start[0], grid_start[1]] == 0:
                    break

        # Check if goal is valid
        if grid_goal[0] < 0 or grid_goal[0] >= grid.shape[0] or grid_goal[1] < 0 or grid_goal[1] >= grid.shape[1]:
            print("WARNING: Goal position is outside grid bounds!")
            grid_goal = (int(grid.shape[0] / 2) + 10, int(grid.shape[1] / 2) + 10)
            print("Using adjusted goal:", grid_goal)
        elif grid[grid_goal[0], grid_goal[1]] == 1:
            print("WARNING: Goal position is inside an obstacle!")
            # Find nearest free space
            for offset in range(1, 20):
                for dx in range(-offset, offset+1):
                    for dy in range(-offset, offset+1):
                        new_goal = (grid_goal[0] + dx, grid_goal[1] + dy)
                        if (0 <= new_goal[0] < grid.shape[0] and
                            0 <= new_goal[1] < grid.shape[1] and
                            grid[new_goal[0], new_goal[1]] == 0):
                            grid_goal = new_goal
                            print("Adjusted goal to:", grid_goal)
                            break
                    if grid[grid_goal[0], grid_goal[1]] == 0:
                        break
                if grid[grid_goal[0], grid_goal[1]] == 0:
                    break

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        print('Original path length:', len(path))
        path = prune_path(path)
        print('Pruned path length:', len(path))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
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
