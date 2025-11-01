"""
Test script for Iterative Deepening A* (IDA*) implementation
"""
import numpy as np
from planning_utils import iterative_astar, a_star, heuristic

def test_simple_grid():
    """Test IDA* on a simple 10x10 grid with some obstacles"""
    print("=" * 60)
    print("Test 1: Simple 10x10 grid")
    print("=" * 60)

    # Create a simple 10x10 grid
    grid = np.zeros((10, 10))

    # Add some obstacles
    grid[3:7, 5] = 1  # Vertical wall

    start = (0, 0)
    goal = (9, 9)

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
    print(f"\nIDA* Path (first 10 points): {path_ida[:10]}")
    print(f"A*   Path (first 10 points): {path_astar[:10]}")

    return path_ida, path_astar


def test_no_path():
    """Test IDA* when no path exists"""
    print("\n\n" + "=" * 60)
    print("Test 2: Grid with no path (blocked goal)")
    print("=" * 60)

    # Create a grid where goal is completely blocked
    grid = np.zeros((10, 10))

    # Create a box around the goal
    grid[7:10, 7] = 1
    grid[7:10, 9] = 1
    grid[7, 7:10] = 1
    grid[9, 7:10] = 1

    start = (0, 0)
    goal = (8, 8)  # Goal is blocked

    print(f"\nStart: {start}")
    print(f"Goal: {goal} (blocked)")
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


def test_diagonal_path():
    """Test IDA* with diagonal movements"""
    print("\n\n" + "=" * 60)
    print("Test 3: Simple diagonal path (no obstacles)")
    print("=" * 60)

    # Empty 5x5 grid
    grid = np.zeros((5, 5))

    start = (0, 0)
    goal = (4, 4)

    print(f"\nStart: {start}")
    print(f"Goal: {goal}")

    print("\n" + "-" * 60)
    print("Running IDA* search...")
    print("-" * 60)
    path, cost = iterative_astar(grid, heuristic, start, goal)

    print("\n" + "=" * 60)
    print("RESULTS:")
    print("=" * 60)
    print(f"Path length: {len(path)}, Cost: {cost:.2f}")
    print(f"Path: {path}")
    print(f"Expected: diagonal path with cost ≈ {4 * np.sqrt(2):.2f}")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("ITERATIVE DEEPENING A* (IDA*) TEST SUITE")
    print("=" * 60)

    test_simple_grid()
    test_no_path()
    test_diagonal_path()

    print("\n\n" + "=" * 60)
    print("ALL TESTS COMPLETED")
    print("=" * 60)
