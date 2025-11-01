"""
Performance comparison between A* and IDA* on different grid sizes
"""
import numpy as np
import time
from planning_utils import a_star, iterative_astar, heuristic, heuristic_diagonal, heuristic_manhattan


def test_performance(grid_size, obstacle_density=0.1):
    """
    Test performance of both algorithms on a given grid size

    Args:
        grid_size: tuple (rows, cols)
        obstacle_density: fraction of cells that are obstacles (0.0 to 1.0)
    """
    rows, cols = grid_size
    print("\n" + "=" * 70)
    print(f"Grid Size: {rows}x{cols} = {rows*cols:,} cells")
    print(f"Obstacle Density: {obstacle_density*100:.0f}%")
    print("=" * 70)

    # Create grid with random obstacles
    np.random.seed(42)
    grid = np.random.choice([0, 1], size=grid_size, p=[1-obstacle_density, obstacle_density])

    # Set start and goal
    start = (5, 5)
    goal = (rows - 6, cols - 6)

    # Make sure start and goal are not obstacles
    grid[start[0], start[1]] = 0
    grid[goal[0], goal[1]] = 0

    # Clear a path to ensure solvability (straight line)
    for i in range(start[0], goal[0]):
        grid[i, start[1]] = 0
    for j in range(start[1], goal[1]):
        grid[goal[0], j] = 0

    # Test A*
    print("\n[A* Search]")
    start_time = time.time()
    path_astar, cost_astar = a_star(grid, heuristic, start, goal)
    time_astar = time.time() - start_time

    print(f"  Time: {time_astar:.3f}s")
    print(f"  Path length: {len(path_astar)}")
    print(f"  Cost: {cost_astar:.2f}")

    # Test IDA* (with timeout for large grids)
    print("\n[IDA* Search]")
    if rows * cols > 50000:
        print("  Skipped (grid too large - would take too long)")
        time_ida = float('inf')
        path_ida = []
        cost_ida = 0
    else:
        start_time = time.time()
        path_ida, cost_ida = iterative_astar(grid, heuristic, start, goal, max_iterations=200)
        time_ida = time.time() - start_time

        print(f"  Time: {time_ida:.3f}s")
        print(f"  Path length: {len(path_ida)}")
        print(f"  Cost: {cost_ida:.2f}")

    # Comparison
    print("\n" + "-" * 70)
    print("COMPARISON:")
    print("-" * 70)
    if time_ida != float('inf') and len(path_ida) > 0:
        speedup = time_ida / time_astar if time_astar > 0 else float('inf')
        print(f"  A* is {speedup:.1f}x faster than IDA* on this grid")
        print(f"  Both found {'the same' if abs(cost_astar - cost_ida) < 0.01 else 'different'} optimal path")
    else:
        print(f"  A* completed in {time_astar:.3f}s")
        print(f"  IDA* was skipped or failed to complete")


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("PERFORMANCE COMPARISON: A* vs IDA*")
    print("=" * 70)
    print("\nThis test demonstrates why IDA* becomes impractical on large grids")
    print("due to repeated re-exploration of the search space.")

    # Small grid - IDA* should be competitive
    test_performance((20, 20), obstacle_density=0.2)

    # Medium grid - IDA* starts to struggle
    test_performance((50, 50), obstacle_density=0.2)

    # Large grid - IDA* becomes very slow
    test_performance((100, 100), obstacle_density=0.2)

    # Very large grid (like colliders.csv) - IDA* is impractical
    print("\n\n" + "=" * 70)
    print("REAL-WORLD SCENARIO: colliders.csv")
    print("=" * 70)
    print("\nYour colliders.csv creates a 921x921 grid = 848,241 cells")
    print("\nObservations:")
    print("  - A*: ~100 steps to find path")
    print("  - IDA*: 167+ iterations, still searching...")
    print("\nConclusion:")
    print("  For large real-world maps like colliders.csv, use standard A*")
    print("  IDA* is only suitable for small grids or memory-constrained scenarios")

    print("\n\n" + "=" * 70)
    print("RECOMMENDATION")
    print("=" * 70)
    print("It automatically chooses the best algorithm based on grid size:")
    print("  - Large grids (>100k cells): A*")
    print("  - Small grids (<100k cells): IDA*")
