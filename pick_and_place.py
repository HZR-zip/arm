#!/usr/bin/env python3
"""
Star_ 4-DOF Robot Arm - Pick & Place Demo

用法:
  roslaunch star__moveit_config demo.launch    # 终端1
  python3 /home/hzr/arm_xingzai/arm/pick_and_place.py   # 终端2

说明:
  - 4 轴臂不能独立控制末端姿态，RViz 中只能用位置箭头拖动 XYZ
  - 想调末端朝向用 joint_state_publisher_gui 的 j4 滑块
  - 关节角直接指定，关节空间规划 100% 可靠
  - 每次移动输出末端 XYZ，方便对照调整
"""

import sys, math
import rospy
import moveit_commander


def go_joint(group, angles, label=""):
    if label:
        print(f"\n  >>> {label}")
    group.clear_pose_targets()
    group.set_joint_value_target(angles)
    ok = group.go(wait=True)
    if ok:
        j = group.get_current_joint_values()
        p = group.get_current_pose().pose.position
        print(f"  j1={math.degrees(j[0]):.0f} j2={math.degrees(j[1]):.0f} "
              f"j3={math.degrees(j[2]):.0f} j4={math.degrees(j[3]):.0f}")
        print(f"  XYZ=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
    else:
        print(f"  FAILED")
    return ok


def main():
    rospy.init_node('demo', anonymous=True)
    group = moveit_commander.MoveGroupCommander('arm')
    group.set_planning_time(5.0)

    print("\n" + "=" * 55)
    print("  Star_ 4-DOF Arm - Pick & Place Demo")
    print("=" * 55)

    # ── 姿态定义 ──
    # j4 控制末端朝向: 负值=朝下, 正值=朝上
    # j2(肩)+j3(肘) 控制末端位置: 越大越前伸

    # 抓取: j2=52deg j3=38deg j4=-90deg → 前伸+末端朝下
    GRASP = [0.0, 0.90, 0.67, -1.57]

    # 抬升: 回收肩肘，保持末端朝下
    LIFT  = [0.0, 0.45, 0.45, -1.57]

    # 1. Home
    print("\n" + "-" * 55)
    print("  [1] Home")
    go_joint(group, [0, 0, 0, 0])

    # 2. Pre-grasp
    print("\n" + "-" * 55)
    print("  [2] Pre-grasp")
    go_joint(group, [0.0, 0.45, 0.45, -1.2], "pre-grasp")

    # 3. Grasp
    print("\n" + "-" * 55)
    print("  [3] Grasp")
    go_joint(group, GRASP, "grasp")

    # 4. Lift
    print("\n" + "-" * 55)
    print("  [4] Lift")
    go_joint(group, LIFT, "lift")

    # 5. Turn right
    print("\n" + "-" * 55)
    print("  [5] Turn right")
    go_joint(group, [0.6] + LIFT[1:], "turn half")
    go_joint(group, [1.2] + LIFT[1:], "turn full")

    # 6. Place
    print("\n" + "-" * 55)
    print("  [6] Place")
    go_joint(group, [1.2] + GRASP[1:], "place")

    # 7. Home
    print("\n" + "-" * 55)
    print("  [7] Home")
    go_joint(group, [0.0] + LIFT[1:], "retract")
    go_joint(group, [0, 0, 0, 0])

    print("\n" + "=" * 55)
    print("  Demo complete")
    print("=" * 55 + "\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
