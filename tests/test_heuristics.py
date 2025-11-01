"""
Test script to compare different heuristic functions for A* search.

This script demonstrates the differences between:
1. Euclidean distance (default)
2. Manhattan distance
3. Diagonal distance
4. Chebyshev distance

And also demonstrates multi-waypoint path planning.
"""

import numpy as np
import time
from planning_utils import (
    create_grid, a_star, a_star_multi_waypoints,
    heuristic, heuristic_manhattan, heuristic_diagonal, heuristic_chebyshev,
    validate_position, prune_path_bresenham
)


def test_heuristics():
    """Test different heuristic functions on the same start/goal positions."""

    print("=" * 80)
    print("Testing Different Heuristic Functions")
    print("=" * 80)

    # Load obstacle data
    print("\nLoading obstacle data from colliders.csv...")
    data = np.loadtxt('../colliders.csv', delimiter=',', dtype='float64', skiprows=2)

    # Create grid
    TARGET_ALTITUDE = 5
    SAFETY_DISTANCE = 5
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
    print(f"Grid created: {grid.shape[0]} x {grid.shape[1]}")
    print(f"North offset: {north_offset}, East offset: {east_offset}")

    # Define test start and goal positions (in grid coordinates)
    grid_start = (316, 445)  # Near the center
    grid_goal = (500, 600)   # Diagonal from start

    # Validate positions
    grid_start = validate_position(grid_start, grid, "start")
    grid_goal = validate_position(grid_goal, grid, "goal")

    print(f"\nStart: {grid_start}")
    print(f"Goal: {grid_goal}")

    # Test each heuristic
    heuristics = {
        "Euclidean": heuristic,
        "Manhattan": heuristic_manhattan,
        "Diagonal": heuristic_diagonal,
        "Chebyshev": heuristic_chebyshev
    }

    results = {}

    for name, h_func in heuristics.items():
        print("\n" + "-" * 80)
        print(f"Testing {name} heuristic...")
        print("-" * 80)

        start_time = time.time()
        path, cost = a_star(grid, h_func, grid_start, grid_goal)
        elapsed_time = time.time() - start_time

        if path:
            pruned_path = prune_path_bresenham(grid, path)
            results[name] = {
                'path_length': len(path),
                'pruned_length': len(pruned_path),
                'cost': cost,
                'time': elapsed_time
            }
            print(f"[SUCCESS] Path found!")
            print(f"  Original path length: {len(path)}")
            print(f"  Pruned path length: {len(pruned_path)}")
            print(f"  Path cost: {cost:.2f}")
            print(f"  Time: {elapsed_time:.4f} seconds")
        else:
            print(f"[FAILED] No path found")
            results[name] = None

    # Print comparison table
    print("\n" + "=" * 80)
    print("COMPARISON SUMMARY")
    print("=" * 80)
    print(f"{'Heuristic':<15} {'Path Length':<15} {'Pruned Length':<15} {'Cost':<10} {'Time (s)':<10}")
    print("-" * 80)

    for name, result in results.items():
        if result:
            print(f"{name:<15} {result['path_length']:<15} {result['pruned_length']:<15} "
                  f"{result['cost']:<10.2f} {result['time']:<10.4f}")
        else:
            print(f"{name:<15} {'N/A':<15} {'N/A':<15} {'N/A':<10} {'N/A':<10}")

if __name__ == "__main__":
    # Test different heuristics
    test_heuristics()

    print("\n" + "=" * 80)
    print("Testing Complete!")
    print("=" * 80)
