from enum import Enum
from queue import PriorityQueue
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        if Action.NORTH in valid_actions:
            valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        if Action.SOUTH in valid_actions:
            valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        if Action.WEST in valid_actions:
            valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        if Action.EAST in valid_actions:
            valid_actions.remove(Action.EAST)

    # Check diagonal actions
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        if Action.NORTH_WEST in valid_actions:
            valid_actions.remove(Action.NORTH_WEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        if Action.NORTH_EAST in valid_actions:
            valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        if Action.SOUTH_WEST in valid_actions:
            valid_actions.remove(Action.SOUTH_WEST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        if Action.SOUTH_EAST in valid_actions:
            valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions
'''
while 队列不为空:
    1. 取出f值最小的节点(current_node)
    2. 如果是目标节点 → 找到路径，结束
    3. 否则，遍历所有可行动作:
       - 计算邻居节点(next_node)
       - 计算总代价f = g(实际代价) + h(启发式代价)
       - 如果节点未访问过:
         * 标记为已访问
         * 记录分支信息{next_node: (cost, parent, action)}
         * 加入优先队列
'''
def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    # 格式: {节点: (代价, 父节点, 动作)}
    # 即: {node: (cost, parent_node, action)}
    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def validate_position(position, grid, position_name="position"):
    """
    Validate and adjust a grid position to ensure it's within bounds and not in an obstacle.

    Args:
        position: tuple (row, col) representing the position in grid coordinates
        grid: numpy array representing the obstacle grid
        position_name: string name for logging purposes (e.g., "start", "goal")

    Returns:
        tuple: validated and potentially adjusted position
    """
    validated_pos = position

    # Check if position is outside grid bounds
    if (position[0] < 0 or position[0] >= grid.shape[0] or
        position[1] < 0 or position[1] >= grid.shape[1]):
        print(f"WARNING: {position_name.capitalize()} position is outside grid bounds!")
        validated_pos = (int(grid.shape[0] / 2), int(grid.shape[1] / 2))
        print(f"Using grid center as {position_name}:", validated_pos)
        return validated_pos

    # Check if position is inside an obstacle
    if grid[position[0], position[1]] == 1:
        print(f"WARNING: {position_name.capitalize()} position is inside an obstacle!")
        # Find nearest free space
        for offset in range(1, 20):
            for dx in range(-offset, offset + 1):
                for dy in range(-offset, offset + 1):
                    new_pos = (position[0] + dx, position[1] + dy)
                    if (0 <= new_pos[0] < grid.shape[0] and
                        0 <= new_pos[1] < grid.shape[1] and
                        grid[new_pos[0], new_pos[1]] == 0):
                        validated_pos = new_pos
                        print(f"Adjusted {position_name} to:", validated_pos)
                        return validated_pos

    return validated_pos

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def iterative_astar(grid, h, start, goal, max_iterations=1000):
    """
    Iterative Deepening A* (IDA*) search algorithm.

    Uses DFS with an f-value threshold that increases iteratively.
    Space complexity: O(bd) - linear!
    Time complexity: O(b^m)

    Args:
        grid: 2D numpy array representing the environment (0=free, 1=obstacle)
        h: heuristic function h(position, goal_position)
        start: tuple (row, col) representing start position
        goal: tuple (row, col) representing goal position
        max_iterations: maximum number of iterations to prevent infinite loops (default: 1000)

    Returns:
        path: list of tuples representing the path from start to goal
        path_cost: total cost of the path
    """

    # Check if start or goal is on an obstacle
    if grid[start[0], start[1]] == 1:
        print('**********************')
        print('Start position is on an obstacle!')
        print('**********************')
        return [], 0

    if grid[goal[0], goal[1]] == 1:
        print('**********************')
        print('Goal position is on an obstacle!')
        print('**********************')
        return [], 0

    # Check if start equals goal
    if start == goal:
        print('Start equals goal, no path needed.')
        return [start], 0

    def dfs(node, g_cost, threshold, path, visited):
        """
        Depth-first search bounded by f-value threshold.

        Args:
            node: current node position (row, col)
            g_cost: cost from start to current node
            threshold: f-value threshold for this iteration
            path: current path from start to node
            visited: set of visited nodes in current DFS branch

        Returns:
            (found, min_cost, final_path) tuple where:
                found: True if goal is found
                min_cost: minimum f-value exceeding threshold (for next iteration)
                final_path: complete path if found, otherwise None
        """
        f_cost = g_cost + h(node, goal)

        # If f-value exceeds threshold, return the f-value for next iteration
        if f_cost > threshold:
            return False, f_cost, None

        # Goal found
        if node == goal:
            return True, f_cost, path

        # Track minimum f-value that exceeds threshold
        min_exceeded = float('inf')

        # Explore valid actions
        for action in valid_actions(grid, node):
            da = action.delta
            next_node = (node[0] + da[0], node[1] + da[1])

            # Skip if already in current path (avoid cycles in DFS)
            if next_node in visited:
                continue

            # Add to visited set and path
            visited.add(next_node)
            new_path = path + [next_node]

            # Recursive DFS call
            found, cost, result_path = dfs(
                next_node,
                g_cost + action.cost,
                threshold,
                new_path,
                visited
            )

            # Goal found - return immediately
            if found:
                return True, cost, result_path

            # Track minimum exceeded f-value for next iteration
            if cost < min_exceeded:
                min_exceeded = cost

            # Backtrack: remove from visited set for other branches
            visited.remove(next_node)

        return False, min_exceeded, None

    # Initialize threshold with heuristic from start
    threshold = h(start, goal)
    path = [start]

    iteration = 0
    prev_threshold = -1  # Track previous threshold to detect no progress

    while True:
        iteration += 1
        print(f'IDA* iteration {iteration}, threshold = {threshold:.2f}')

        # Check if maximum iterations exceeded
        if iteration > max_iterations:
            print('**********************')
            print(f'Failed to find a path! Maximum iterations ({max_iterations}) exceeded.')
            print('This likely means no path exists between start and goal.')
            print('**********************')
            return [], 0

        # Perform DFS with current threshold
        visited = {start}
        found, next_threshold, result_path = dfs(start, 0, threshold, path, visited)

        if found:
            print(f'Found a path in {iteration} iterations.')
            path_cost = next_threshold
            return result_path, path_cost

        # No path exists if threshold doesn't increase
        if next_threshold == float('inf'):
            print('**********************')
            print('Failed to find a path!')
            print('No nodes exceed the current threshold - goal is unreachable.')
            print('**********************')
            return [], 0

        # Check if threshold is not increasing (stuck in same state)
        if abs(next_threshold - prev_threshold) < 1e-6:
            print('**********************')
            print('Failed to find a path!')
            print('Threshold stopped increasing - goal may be unreachable.')
            print('**********************')
            return [], 0

        # Update threshold for next iteration
        prev_threshold = threshold
        threshold = next_threshold


def point(p):
    """Helper function to create a 3D point with z=1 for collinearity test"""
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    """
    Check if three points are collinear using the determinant method.
    Returns True if the points are collinear (within epsilon tolerance).
    """
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


def prune_path(path):
    """
    Prune path by removing collinear waypoints using the collinearity test.
    This reduces the number of waypoints while maintaining the same path.
    """
    if len(path) < 3:
        return path

    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])

        # If the 3 points are collinear, remove the middle point
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1

    return pruned_path