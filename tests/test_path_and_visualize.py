# 对比可视化：原始路径 vs 修剪路径
import matplotlib.pyplot as plt
from planning_utils import prune_path

# 定义原始路径
original_path = [(316, 445), (317, 446), (318, 447), (319, 448), (320, 449), (321, 450),
                 (322, 451), (323, 452), (324, 453), (325, 454), (326, 455), (327, 456),
                 (328, 457), (329, 458), (330, 459), (331, 460), (332, 461), (333, 462),
                 (334, 463), (335, 464), (336, 465), (337, 466), (338, 467), (339, 468),
                 (340, 469), (341, 470), (342, 471), (343, 472), (344, 473), (345, 474),
                 (346, 475), (347, 476), (348, 477), (349, 478), (350, 479), (351, 480),
                 (352, 481), (353, 482), (354, 483), (355, 484), (356, 485), (357, 486),
                 (358, 487), (359, 488), (360, 489), (361, 490), (362, 491), (363, 492),
                 (364, 493), (365, 494), (366, 495), (367, 496), (368, 497), (369, 498),
                 (370, 499), (371, 500), (372, 501), (373, 502), (374, 503), (375, 504),
                 (376, 505), (377, 506), (378, 507), (379, 508), (380, 509), (381, 509),
                 (382, 510), (383, 510), (384, 511), (385, 511), (386, 512), (387, 512),
                 (388, 513), (389, 513), (390, 514), (391, 514), (392, 514), (393, 515),
                 (394, 515), (395, 516), (396, 516), (397, 517), (398, 517), (399, 518),
                 (400, 518), (401, 519), (402, 519), (403, 519), (404, 520), (405, 520),
                 (406, 521), (407, 521), (408, 522), (409, 522), (410, 523), (411, 523),
                 (412, 524), (413, 524), (414, 524), (415, 525), (416, 525), (417, 526),
                 (418, 526), (419, 527), (420, 527), (421, 528), (422, 528), (423, 528),
                 (424, 529)]

"""修建后的路径
[
(316, 445), (380, 509), (381, 509), (382, 510), (383, 510), (384, 511), (385, 511), 
(386, 512), (387, 512), (388, 513), (389, 513), (390, 514), (392, 514), (393, 515), 
(394, 515), (395, 516), (396, 516), (397, 517), (398, 517), (399, 518), (400, 518), 
(401, 519), (403, 519), (404, 520), (405, 520), (406, 521), (407, 521), (408, 522), 
(409, 522), (410, 523), (411, 523), (412, 524), (414, 524), (415, 525), (416, 525), 
(417, 526), (418, 526), (419, 527), (420, 527), (421, 528), (423, 528), (424, 529)
]
"""
pruned_path = prune_path(original_path)
print(pruned_path)

# 提取坐标（注意：第一个是 North，第二个是 East）
original_north = [p[0] for p in original_path]
original_east = [p[1] for p in original_path]
pruned_north = [p[0] for p in pruned_path]
pruned_east = [p[1] for p in pruned_path]

# 创建对比图
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

# 左图：原始路径（x轴=East, y轴=North）
ax1.plot(original_east, original_north, 'b-', linewidth=2, alpha=0.5, label='Path')
ax1.scatter(original_east, original_north, c='red', s=20, zorder=5, label=f'Waypoints ({len(original_path)})')
ax1.scatter(original_east[0], original_north[0], c='green', s=200, marker='o',
           edgecolors='black', linewidth=2, zorder=10, label='Start')
ax1.scatter(original_east[-1], original_north[-1], c='orange', s=200, marker='*',
           edgecolors='black', linewidth=2, zorder=10, label='Goal')
ax1.set_xlabel('East (grid units)', fontsize=12)
ax1.set_ylabel('North (grid units)', fontsize=12)
ax1.set_title('Original Path (未优化)', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best')
ax1.axis('equal')

# 右图：修剪后的路径（x轴=East, y轴=North）
ax2.plot(pruned_east, pruned_north, 'g-', linewidth=3, alpha=0.7, label='Pruned Path')
ax2.scatter(pruned_east, pruned_north, c='blue', s=100, zorder=5, marker='s',
           edgecolors='black', linewidth=1.5, label=f'Waypoints ({len(pruned_path)})')
ax2.scatter(pruned_east[0], pruned_north[0], c='green', s=200, marker='o',
           edgecolors='black', linewidth=2, zorder=10, label='Start')
ax2.scatter(pruned_east[-1], pruned_north[-1], c='orange', s=200, marker='*',
           edgecolors='black', linewidth=2, zorder=10, label='Goal')

# 在修剪后的路径上标注航点编号
for i, (north, east) in enumerate(pruned_path):
    ax2.annotate(f'{i}', (east, north), xytext=(5, 5), textcoords='offset points',
                fontsize=8, color='darkblue', fontweight='bold')

ax2.set_xlabel('East (grid units)', fontsize=12)
ax2.set_ylabel('North (grid units)', fontsize=12)
ax2.set_title('Pruned Path (优化后 - 移除共线点)', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best')
ax2.axis('equal')

plt.tight_layout()

# 保存到 Logs 目录
output_path = f'../Logs/path_comparison.png'
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"\n对比图已保存到: {output_path}")

# 显示统计信息
print(f"\n路径优化统计:")
print(f"{'='*50}")
print(f"原始航点数:   {len(original_path):>4} 个")
print(f"优化后航点数: {len(pruned_path):>4} 个")
print(f"减少航点数:   {len(original_path) - len(pruned_path):>4} 个")
print(f"优化比例:     {(1-len(pruned_path)/len(original_path))*100:>5.1f}%")
print(f"{'='*50}")

plt.show()