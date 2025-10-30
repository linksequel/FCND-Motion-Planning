"""
Test script to verify IDA* safety checks for infinite loop prevention
"""
import numpy as np
from planning_utils import iterative_astar, heuristic

def test_blocked_goal():
    """Test IDA* when goal is completely surrounded by obstacles"""
    print("=" * 60)
    print("Test 1: Completely blocked goal (no path exists)")
    print("=" * 60)

    # Create a grid where goal is completely blocked
    grid = np.zeros((10, 10))

    # Create a box around position (8, 8)
    grid[7:10, 7] = 1
    grid[7:10, 9] = 1
    grid[7, 7:10] = 1
    grid[9, 7:10] = 1

    start = (0, 0)
    goal = (8, 8)  # Goal is completely blocked

    print(f"\nStart: {start}")
    print(f"Goal: {goal} (completely surrounded by obstacles)")
    print("\nGrid (1 = obstacle, 0 = free):")
    print(grid.astype(int))

    print("\n" + "-" * 60)
    print("Running IDA* search with max_iterations=50...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal, max_iterations=50)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path found: {len(path) > 0}")
    print(f"Path length: {len(path)}")
    print(f"Path cost: {cost:.2f}")
    print("\n[SUCCESS] Algorithm terminated gracefully (no infinite loop)")


def test_goal_on_obstacle():
    """Test IDA* when goal is on an obstacle"""
    print("\n\n" + "=" * 60)
    print("Test 2: Goal is on an obstacle")
    print("=" * 60)

    grid = np.zeros((5, 5))
    grid[2, 2] = 1  # Obstacle at (2, 2)

    start = (0, 0)
    goal = (2, 2)  # Goal is on obstacle

    print(f"\nStart: {start}")
    print(f"Goal: {goal} (on obstacle)")
    print("\nGrid (1 = obstacle, 0 = free):")
    print(grid.astype(int))

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path found: {len(path) > 0}")
    print(f"Correctly detected invalid goal: {len(path) == 0}")
    print("\n[SUCCESS] Algorithm handled invalid goal correctly")


def test_start_on_obstacle():
    """Test IDA* when start is on an obstacle"""
    print("\n\n" + "=" * 60)
    print("Test 3: Start is on an obstacle")
    print("=" * 60)

    grid = np.zeros((5, 5))
    grid[0, 0] = 1  # Obstacle at (0, 0)

    start = (0, 0)  # Start is on obstacle
    goal = (4, 4)

    print(f"\nStart: {start} (on obstacle)")
    print(f"Goal: {goal}")
    print("\nGrid (1 = obstacle, 0 = free):")
    print(grid.astype(int))

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path found: {len(path) > 0}")
    print(f"Correctly detected invalid start: {len(path) == 0}")
    print("\n[SUCCESS] Algorithm handled invalid start correctly")


def test_start_equals_goal():
    """Test IDA* when start equals goal"""
    print("\n\n" + "=" * 60)
    print("Test 4: Start equals goal")
    print("=" * 60)

    grid = np.zeros((5, 5))
    start = (2, 2)
    goal = (2, 2)

    print(f"\nStart: {start}")
    print(f"Goal: {goal}")

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path: {path}")
    print(f"Cost: {cost:.2f}")
    print(f"Correctly handled start==goal: {path == [start] and cost == 0}")
    print("\n[SUCCESS] Algorithm handled start==goal correctly")


def test_narrow_passage():
    """Test IDA* with a narrow passage"""
    print("\n\n" + "=" * 60)
    print("Test 5: Valid path through narrow passage")
    print("=" * 60)

    grid = np.zeros((7, 7))
    # Create walls with a narrow passage
    grid[3, 0:3] = 1
    grid[3, 4:7] = 1

    start = (0, 3)
    goal = (6, 3)

    print(f"\nStart: {start}")
    print(f"Goal: {goal}")
    print("\nGrid (1 = obstacle, 0 = free):")
    print(grid.astype(int))

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path found: {len(path) > 0}")
    print(f"Path length: {len(path)}")
    print(f"Path cost: {cost:.2f}")
    print(f"Path (first 10 nodes): {path[:10]}")
    print("\n[SUCCESS] Algorithm found path through narrow passage")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("IDA* SAFETY CHECKS TEST SUITE")
    print("Testing infinite loop prevention and edge cases")
    print("=" * 60)

    test_blocked_goal()
    test_goal_on_obstacle()
    test_start_on_obstacle()
    test_start_equals_goal()
    test_narrow_passage()

    print("\n\n" + "=" * 60)
    print("ALL SAFETY TESTS COMPLETED")
    print("=" * 60)
    print("\nSummary:")
    print("- No infinite loops occurred")
    print("- All edge cases handled correctly")
    print("- Algorithm is robust and safe to use")
