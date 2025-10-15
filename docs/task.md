## 1. HW_1中的注意事项
完成规划器后，您可以对其进行测试。通过调整 **grid_start 和 grid_goal** 参数的值，您可以设置无人机的起点和终点。提醒：如果这些点位于文件 colliders.csv 记录的障碍物地图中，搜索算法可能会花费很长时间并无法输出路径。在这种情况下，您最好重置这些参数并再次运行更新后的程序。


## 2. 完成3的前置应该是完成仓库中的TODO？
1. Line 122: Read lat0, lon0 from colliders.csv first line 第 122 行：从第一行读取 lat0、lon0 colliders.csv 
2. Line 124: Set home position to (lon0, lat0, 0) 第 124 行：将起始位置设置为 （lon0， lat0， 0） 
3. Line 126: Retrieve current global position 第 126 行：检索当前全局位置 
4. Line 128: Convert to current local position using global_to_local() 第128 行：使用 global_to_local（） 转换为当前本地位置 
5. Line 140: Convert start position to current position rather than map center  第 140 行：将起始位置转换为当前位置而不是地图中心 
6. Line 144: Adapt to set goal as latitude/longitude position and convert 第 144 行：适应将目标设置为纬度/经度位置并转换 
7. Line 147: Add diagonal motions with cost of sqrt(2) to A* implementation 第 147 行：将成本为 sqrt（2） 的对角线运动添加到 A* 实现中 
8. Line 151: Prune path to minimize number of waypoints (e.g., collinearity test or Bresenham) 第 151 行：修剪路径以尽量减少航路点数量（例如，共线性测试或 Bresenham） 
9. Line 158: Send waypoints to simulator for visualization 第 158 行：将航点发送到模拟器以进行可视化

## 3. HW_1中的任务
### 问题 4（20 分）：实现迭代深化 A* 搜索算法
你需要在 planning_utils.py 中编写一个名为 iterative_astar(grid, h, start, goal) 的迭代深化 A* 搜索算法，以帮助无人机规划路线。迭代深化 A* 搜索算法的具体流程可在课程幻灯片中找到。

### 问题 5（20 分）：为 A* 实现不同的启发式算法，以及遍历 3 个固定点的 A* 搜索
在 planning_utils.py 中，当前版本的 A* 算法使用曼哈顿距离作为启发式算法。我们鼓励你提出一个有效的启发式算法，并实现它，然后观察规划路线的变化。

你还需要在 motion_planning.py 中设置三个固定点，无人机在到达目的地之前必须经过这些点。在此基础上，重新实现 A* 搜索算法，以便遍历这三个点。