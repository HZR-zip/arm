#!/usr/bin/env python3
"""简单规划测试 — 先关节空间, 再末端位置"""
import rospy, math
import moveit_commander

rospy.init_node('test_plan', anonymous=True)
group = moveit_commander.MoveGroupCommander('arm')

# 增加IK超时, 允许近似解
group.set_planning_time(10.0)
group.set_num_planning_attempts(10)
group.allow_replanning(True)

# ─── 测试1: 关节空间 (不需要IK, 必定成功) ───
print("\n=== 测试1: 关节空间 ===")
for name, joints in [
    ("home",  [0.0, 0.0, 0.0, 0.0]),
    ("姿态A", [0.5, 0.8, -0.8, 0.3]),
    ("姿态B", [1.0, -0.5, 0.5, -0.5]),
]:
    group.set_joint_value_target(joints)
    plan = group.plan()
    ok = "OK" if plan[0] else "FAIL"
    print(f"  {name} {joints} -> {ok}")

# ─── 测试2: 末端位置 ───
print("\n=== 测试2: 末端位置 ===")
group.set_position_target([0.08, 0.0, 0.10])
plan = group.plan()
print(f"  (0.08, 0.0, 0.10) -> {'OK' if plan[0] else 'FAIL'}")

group.set_position_target([0.0, 0.08, 0.10])
plan = group.plan()
print(f"  (0.0, 0.08, 0.10) -> {'OK' if plan[0] else 'FAIL'}")

group.set_position_target([0.05, 0.05, 0.12])
plan = group.plan()
print(f"  (0.05, 0.05, 0.12) -> {'OK' if plan[0] else 'FAIL'}")

# ─── 测试3: 当前位置附近 ───
print("\n=== 测试3: 当前位置微调 ===")
current = group.get_current_pose().pose
# 从当前位置稍微移动
target = type(current)()
target.position.x = current.position.x + 0.02
target.position.y = current.position.y
target.position.z = current.position.z + 0.02
target.orientation = current.orientation
group.set_pose_target(target)
plan = group.plan()
print(f"  微调 ({target.position.x:.2f},{target.position.y:.2f},{target.position.z:.2f}) -> {'OK' if plan[0] else 'FAIL'}")

# ─── 结果总结 ───
print(f"\n当前关节: {[round(j,2) for j in group.get_current_joint_values()]}")
p = current.position
print(f"当前末端: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
