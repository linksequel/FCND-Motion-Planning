"""
Simple test script for Iterative Deepening A* (IDA*) implementation
"""
import numpy as np
from planning_utils import iterative_astar, a_star, heuristic

def test_simple_path():
    """Test IDA* on a simple 5x5 grid"""
    print("=" * 60)
    print("Test: Simple 5x5 grid with obstacle")
    print("=" * 60)

    # Create a simple 5x5 grid
    grid = np.zeros((5, 5))

    # Add a small obstacle
    grid[2, 2] = 1

    start = (0, 0)
    goal = (4, 4)

    print(f"\nStart: {start}")
    print(f"Goal: {goal}")
    print("\nGrid (1 = obstacle, 0 = free):")
    print(grid.astype(int))

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path_ida, cost_ida = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "-" * 60)
    print("Running standard A* search for comparison...")
    print("-" * 60)
    path_astar, cost_astar = a_star(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"IDA* - Path length: {len(path_ida)}, Cost: {cost_ida:.2f}")
    print(f"A*   - Path length: {len(path_astar)}, Cost: {cost_astar:.2f}")
    print(f"\nIDA* Path: {path_ida}")
    print(f"A*   Path: {path_astar}")

    # Check if costs are equal
    if abs(cost_ida - cost_astar) < 0.01:
        print("\n[SUCCESS] Both algorithms found optimal path with same cost!")
    else:
        print("\n[WARNING] Costs differ between algorithms")

    return path_ida, path_astar


def test_straight_line():
    """Test IDA* with simple straight line path"""
    print("\n\n" + "=" * 60)
    print("Test: Straight line path (no obstacles)")
    print("=" * 60)

    # Empty 3x5 grid
    grid = np.zeros((3, 5))

    start = (1, 0)
    goal = (1, 4)

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
    print(f"Path length: {len(path)}, Cost: {cost:.2f}")
    print(f"Path: {path}")
    print(f"Expected cost: 4.0 (4 moves to the right)")

    if abs(cost - 4.0) < 0.01:
        print("[SUCCESS] Found optimal straight-line path!")
    else:
        print("[WARNING] Cost differs from expected")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("ITERATIVE DEEPENING A* (IDA*) SIMPLE TEST")
    print("=" * 60)

    test_simple_path()
    test_straight_line()

    print("\n\n" + "=" * 60)
    print("TESTS COMPLETED")
    print("=" * 60)
