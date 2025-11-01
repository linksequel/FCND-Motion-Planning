# HW_1 问题5 解决方案

## 问题描述
问题5要求：
1. 为 A* 实现不同的启发式算法（当前版本使用欧几里得距离）
2. 设置三个固定点，无人机在到达目的地之前必须经过这些点
3. 重新实现 A* 搜索算法，以便遍历这三个点

## 实现内容

### 1. 实现不同的启发式算法

在 `planning_utils.py` 文件中实现了四种启发式算法：

#### 1.1 欧几里得距离 (Euclidean Distance)
```python
def heuristic(position, goal_position):
    """默认启发式：欧几里得距离（直线距离）"""
    return np.linalg.norm(np.array(position) - np.array(goal_position))
```
- **特点**: 直线距离，最常用的启发式
- **适用场景**: 通用场景，对于所有移动方式都是可采纳的（admissible）
- **公式**: √((x1-x2)² + (y1-y2)²)

#### 1.2 曼哈顿距离 (Manhattan Distance)
```python
def heuristic_manhattan(position, goal_position):
    """曼哈顿距离（L1范数）"""
    return abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])
```
- **特点**: 坐标差的绝对值之和
- **适用场景**: 仅4方向移动（上下左右）时最优
- **公式**: |x1-x2| + |y1-y2|
- **性能**: 测试显示这是最快的启发式（0.0031秒）

#### 1.3 对角距离 (Diagonal Distance)
```python
def heuristic_diagonal(position, goal_position):
    """对角距离：适合8方向移动"""
    dx = abs(position[0] - goal_position[0])
    dy = abs(position[1] - goal_position[1])
    return np.sqrt(2) * min(dx, dy) + abs(dx - dy)
```
- **特点**: 考虑对角线移动的成本
- **适用场景**: 8方向移动（包括对角线）时最优
- **公式**: √2 × min(dx, dy) + |dx - dy|
- **优势**: 对于本项目（支持8方向移动）来说，这是最合适的启发式

#### 1.4 切比雪夫距离 (Chebyshev Distance)
```python
def heuristic_chebyshev(position, goal_position):
    """切比雪夫距离（L∞范数）"""
    dx = abs(position[0] - goal_position[0])
    dy = abs(position[1] - goal_position[1])
    return max(dx, dy)
```
- **特点**: 坐标差的最大值
- **适用场景**: 对角线移动成本与直线相同时
- **公式**: max(|x1-x2|, |y1-y2|)

### 2. 实现多点路径规划

#### 2.1 核心函数: `a_star_multi_waypoints()`

在 `planning_utils.py` 中实现了 `a_star_multi_waypoints()` 函数：

```python
def a_star_multi_waypoints(grid, h, start, goal, intermediate_waypoints):
    """
    A* 搜索算法，遍历多个固定航点

    参数:
        grid: 环境的2D网格表示
        h: 启发式函数
        start: 起始位置
        goal: 最终目标位置
        intermediate_waypoints: 中间航点列表

    返回:
        path: 完整路径
        path_cost: 总成本
    """
```

**工作原理:**
1. 创建航点序列: start → wp1 → wp2 → wp3 → goal
2. 对每对连续航点运行 A* 搜索
3. 将所有路径段连接成完整路径
4. 避免在连接点重复节点

#### 2.2 在 motion_planning.py 中集成

在 `motion_planning.py` 的 `plan_path()` 方法中：

1. **定义三个固定中间点**（经纬度坐标）:
```python
intermediate_waypoints_global = [
    [37.792980, -122.397250, 0],  # Waypoint 1
    [37.793280, -122.396950, 0],  # Waypoint 2
    [37.793380, -122.396650, 0],  # Waypoint 3
]
```

2. **转换为网格坐标**:
```python
for i, wp_global in enumerate(intermediate_waypoints_global):
    wp_local = global_to_local(wp_global, self.global_home)
    wp_grid = (int(wp_local[0]) - north_offset, int(wp_local[1]) - east_offset)
    wp_grid = validate_position(wp_grid, grid, f"intermediate_waypoint_{i+1}")
    intermediate_waypoints_grid.append(wp_grid)
```

3. **执行多点路径规划**:
```python
if USE_MULTI_WAYPOINT:
    path, _ = a_star_multi_waypoints(grid, chosen_heuristic, grid_start,
                                     grid_goal, intermediate_waypoints_grid)
else:
    path, _ = a_star(grid, chosen_heuristic, grid_start, grid_goal)
```

### 3. 测试和验证

#### 3.1 测试脚本: `test_heuristics.py`

创建了完整的测试脚本来验证实现：

```bash
python test_heuristics.py
```

#### 3.2 测试结果

**不同启发式算法性能比较:**

| 启发式      | 路径长度 | 修剪后长度 | 成本    | 时间(秒) |
|------------|---------|-----------|---------|---------|
| Euclidean  | 179     | 3         | 239.72  | 0.1371  |
| Manhattan  | 179     | 3         | 239.72  | 0.0031  |
| Diagonal   | 179     | 3         | 239.72  | 0.0757  |
| Chebyshev  | 179     | 3         | 239.72  | 0.1657  |

**关键发现:**
1. 所有启发式都找到了相同质量的路径（相同成本）
2. 曼哈顿距离最快（0.0031秒），但它并不是理论上最优的
3. 对角距离启发式是8方向移动的理论最优选择

**多点路径规划测试:**

```
多点路径:
  - 原始路径长度: 321
  - 修剪后长度: 3
  - 总成本: 410.71
  - 时间: 0.0416秒

直接路径（无中间点）:
  - 路径长度: 279
  - 成本: 386.11
  - 时间: 0.0786秒

差异:
  - 额外步数: 42
  - 额外成本: 24.60
```

**结论:**
- 多点路径规划成功工作
- 经过三个中间点增加了约15%的路径长度
- 这是预期的，因为我们强制无人机访问指定位置

## 使用说明

### 切换启发式算法

在 `motion_planning.py` 第212行修改:

```python
# 可选项: heuristic, heuristic_manhattan, heuristic_diagonal, heuristic_chebyshev
chosen_heuristic = heuristic_diagonal  # 8方向移动的最佳选择
```

### 启用/禁用多点路径规划

在 `motion_planning.py` 第208行修改:

```python
USE_MULTI_WAYPOINT = True  # True: 多点规划, False: 标准A*
```

### 修改中间航点

在 `motion_planning.py` 第193-197行修改坐标:

```python
intermediate_waypoints_global = [
    [经度1, 纬度1, 0],  # ⚠️ 注意：先经度，后纬度
    [经度2, 纬度2, 0],
    [经度3, 纬度3, 0],
]
```

**⚠️ 重要提示**：格式必须是 `[经度, 纬度, 高度]`，不是 `[纬度, 经度, 高度]`

## 代码位置

### planning_utils.py
- 第214-219行: `heuristic()` - 欧几里得距离
- 第222-228行: `heuristic_manhattan()` - 曼哈顿距离
- 第231-243行: `heuristic_diagonal()` - 对角距离
- 第246-254行: `heuristic_chebyshev()` - 切比雪夫距离
- 第256-307行: `a_star_multi_waypoints()` - 多点路径规划

### motion_planning.py
- 第8-10行: 导入新函数
- 第186-227行: 多点路径规划实现和启发式选择

### test_heuristics.py
- 完整的测试脚本，包含性能比较

## 技术要点

### 启发式的可采纳性 (Admissibility)

所有实现的启发式都是**可采纳的**，即它们不会高估到达目标的实际成本，这保证了 A* 算法的最优性。

### 启发式的一致性 (Consistency)

对角距离启发式是**一致的**（也称为单调的），满足:
```
h(n) ≤ cost(n, n') + h(n')
```

这意味着估计成本满足三角不等式，可以提高 A* 的效率。

### 多点规划的复杂度

- **时间复杂度**: O(k × b^d)，其中 k 是航点数量
- **空间复杂度**: O(b^d)，与单次 A* 相同
- **实际性能**: 由于每段路径较短，总时间可能比预期更快

## 扩展建议

1. **动态航点顺序优化**
   - 当前实现按固定顺序访问航点
   - 可以实现旅行商问题（TSP）求解器来优化访问顺序

2. **3D路径规划**
   - 扩展到考虑不同的飞行高度
   - 在启发式中加入高度差异

3. **动态障碍物**
   - 添加时间维度
   - 实现时空 A* (Space-Time A*)

4. **其他搜索算法**
   - Theta*: 任意角度路径规划
   - Jump Point Search: 更快的网格搜索
   - RRT*: 基于采样的规划

## 参考资料

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. IEEE Transactions on Systems Science and Cybernetics.

2. Daniel, K., Nash, A., Koenig, S., & Felner, A. (2010). Theta*: Any-Angle Path Planning on Grids. Journal of Artificial Intelligence Research.

3. Patel, A. (2023). Introduction to A* - Red Blob Games. https://www.redblobgames.com/pathfinding/a-star/introduction.html
