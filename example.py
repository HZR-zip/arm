#!/usr/bin/env python3
"""
Star_ 机械臂控制示例 — 含碰撞检测

终端1: roslaunch star__moveit_config demo.launch
终端2: python3 /home/hzr/arm_xingzai/arm/example.py
"""

import time, math
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped


def show(group):
    j = group.get_current_joint_values()
    p = group.get_current_pose().pose.position
    print(f"  关节: j1={math.degrees(j[0]):.0f}°  j2={math.degrees(j[1]):.0f}°  "
          f"j3={math.degrees(j[2]):.0f}°  j4={math.degrees(j[3]):.0f}°")
    print(f"  末端: x={p.x:.3f}  y={p.y:.3f}  z={p.z:.3f}\n")


def add_box(scene, name, x, y, z, sx, sy, sz):
    """在场景中添加一个碰撞盒子"""
    box = PoseStamped()
    box.header.frame_id = "base_link"
    box.pose.position.x = x
    box.pose.position.y = y
    box.pose.position.z = z
    box.pose.orientation.w = 1.0
    scene.add_box(name, box, size=(sx, sy, sz))
    print(f"  添加碰撞体 '{name}': 位置({x},{y},{z}) 尺寸({sx},{sy},{sz})")


def main():
    rospy.init_node('example')
    g = moveit_commander.MoveGroupCommander('arm')
    scene = PlanningSceneInterface()

    g.set_planning_time(5.0)
    g.set_num_planning_attempts(10)

    print("\n" + "=" * 50)
    print("  Star_ 机械臂 — 碰撞检测示例")
    print("=" * 50 + "\n")

    # ── 1. 回 home ──
    print(">>> 回 home")
    g.set_joint_value_target([0, 0, 0, 0])
    g.go(wait=True)
    show(g)

    # ── 2. 无碰撞：自由运动 ──
    print(">>> 无碰撞：末端到前方")
    g.set_position_target([0.08, 0.0, 0.10])
    g.go(wait=True)
    show(g)

    # ── 3. 添加障碍物 ──
    print(">>> 在前方添加一个障碍物")
    add_box(scene, "obstacle", 0.08, 0.0, 0.10, 0.04, 0.04, 0.04)
    time.sleep(1)

    # ── 4. 有碰撞：绕开 ──
    print(">>> 有碰撞：末端到左侧（绕开障碍物）")
    g.set_position_target([0.0, 0.08, 0.10])
    g.go(wait=True)
    show(g)

    # ── 5. 有碰撞：硬穿（会失败或警告）──
    print(">>> 尝试穿过障碍物（应失败/无解）")
    g.set_position_target([0.08, 0.0, 0.10])
    plan = g.plan()
    if plan[0]:
        print("  ⚠ 居然穿过去了 — 可能需要加大障碍物")
    else:
        print("  ✅ 检测到碰撞，规划被阻止")

    # ── 6. 清理场景 ──
    scene.remove_world_object("obstacle")
    print("\n>>> 移除障碍物，回 home")
    g.set_joint_value_target([0, 0, 0, 0])
    g.go(wait=True)
    show(g)

    # ── 7. 自碰撞测试 ──
    print(">>> 自碰撞测试：折叠机械臂")
    g.set_joint_value_target([0, 1.5, -1.5, 1.5])
    plan = g.plan()
    print(f"  极端折叠: {'可执行' if plan[0] else '自碰撞，被阻止'}")
    show(g)

    print("=" * 50)
    print("  完成！碰撞检测已启用")
    print("=" * 50)


if __name__ == '__main__':
    main()
