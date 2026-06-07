#!/usr/bin/env python3
"""
原理：随机采样 3D 空间中的点，用 IK 判断每个点是否可达，从而探出工作空间范围。

终端1: roslaunch star__moveit_config move_group.launch
终端2: python3 /home/hzr/arm_xingzai/arm/workspace_probe.py
"""

import math, random
import rospy
import moveit_commander

rospy.init_node('ws_probe')
g = moveit_commander.MoveGroupCommander('arm')
g.set_planning_time(0.15)        # IK 超时设短，加速
g.set_num_planning_attempts(1)

N = 1500
print(f"\n随机采样 {N} 次探测工作空间...\n")

pts = []
for i in range(N):
    # 在猜测的大范围内随机撒点
    x = random.uniform(-0.25, 0.25)
    y = random.uniform(-0.25, 0.25)
    z = random.uniform(-0.05, 0.35)

    g.set_position_target([x, y, z])
    if g.plan()[0]:          # IK 解出来了 → 该点可达
        pts.append((x, y, z))

    if (i+1) % 300 == 0:
        print(f"  {i+1}/{N}  已找到 {len(pts)} 个可达点")

# ── 统计结果 ──
xs = [p[0] for p in pts]
ys = [p[1] for p in pts]
zs = [p[2] for p in pts]

print(f"\n{'='*55}")
print(f"  {len(pts)} 个可达点 (占 {100*len(pts)/N:.0f}%)")
print(f"{'='*55}")
print(f"  X:  {min(xs):.3f}  ~  {max(xs):.3f} m")
print(f"  Y:  {min(ys):.3f}  ~  {max(ys):.3f} m")
print(f"  Z:  {min(zs):.3f}  ~  {max(zs):.3f} m")
print(f"{'='*55}")

# 各高度的可达半径
print("\n各高度层可达半径:")
for z0 in [0.02, 0.06, 0.10, 0.14, 0.18, 0.22, 0.26]:
    r_vals = [math.sqrt(p[0]**2+p[1]**2) for p in pts
              if abs(p[2] - z0) < 0.02]
    if r_vals:
        print(f"  z≈{z0:.2f}m → 半径 {min(r_vals):.3f} ~ {max(r_vals):.3f} m")
