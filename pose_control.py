#!/usr/bin/env python3
"""
Star_ 机械臂 — 自定义末端姿态控制

用法:
  python3 pose_control.py pos 0.08 0 0.10                # 只看位置
  python3 pose_control.py rpy 0.08 0 0.10 0 90 0         # 位置 + RPY角度(度)
  python3 pose_control.py quat 0.08 0 0.10 0 0 1 0       # 位置 + 四元数
  python3 pose_control.py demo                            # 演示各种姿态
"""

import sys, math
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler


def make_pose(x, y, z, roll_deg=0, pitch_deg=0, yaw_deg=0):
    """用 RPY 角度创建 Pose"""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)
    q = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def make_pose_quat(x, y, z, qx, qy, qz, qw):
    """用四元数创建 Pose"""
    pose = Pose()
    pose.position.x = x; pose.position.y = y; pose.position.z = z
    pose.orientation.x = qx; pose.orientation.y = qy
    pose.orientation.z = qz; pose.orientation.w = qw
    return pose


def show(group):
    j = group.get_current_joint_values()
    p = group.get_current_pose().pose
    print(f"  关节: j1={math.degrees(j[0]):.0f}° j2={math.degrees(j[1]):.0f}° "
          f"j3={math.degrees(j[2]):.0f}° j4={math.degrees(j[3]):.0f}°")
    print(f"  位置: x={p.position.x:.3f} y={p.position.y:.3f} z={p.position.z:.3f}")
    print(f"  姿态: qx={p.orientation.x:.3f} qy={p.orientation.y:.3f} "
          f"qz={p.orientation.z:.3f} qw={p.orientation.w:.3f}\n")


def go_pose(group, pose, label=""):
    if label:
        print(f">>> {label}")
    group.set_pose_target(pose)
    ok = group.go(wait=True)
    if ok:
        show(group)
    else:
        print(f"  ❌ 不可达，换个姿态试试\n")
    return ok


# ============================================================
#  预定义姿态模板 — 可以直接改这里
# ============================================================

def demo_defaults(group):
    """演示各种常用末端姿态"""

    # ── 朝向设置说明 ──
    # 末端 l4 的默认朝向取决于 URDF 中 link 的定义
    # RPY = (0,0,0) 即保持默认朝向，只移位置
    # RPY = (0,90,0) 即 pitch 向下 90°, 末端朝下(抓取姿态)
    # RPY = (90,0,0) 即 roll   转 90°

    examples = [
        # (标签,             x,    y,    z,   roll, pitch, yaw)
        ("home",             0.12, 0.01, 0.15, 0,    0,    0),
        ("正前上方",          0.10, 0.0,  0.18, 0,    0,    0),
        ("左侧",             0.0,  0.10, 0.12, 0,    0,    0),
        ("右侧",             0.0, -0.10, 0.12, 0,    0,    0),
        ("末端朝下(抓取)",     0.08, 0.0,  0.10, 0,   90,   0),
        ("末端朝右",          0.08, 0.0,  0.10, 0,    0,   90),
        ("末端倾斜45°",       0.06, 0.04, 0.12, 0,   45,   0),
        ("左侧+末端朝下",      0.0,  0.08, 0.08, 0,   90,   0),
    ]

    group.set_joint_value_target([0, 0, 0, 0])
    group.go(wait=True)

    for label, x, y, z, roll, pitch, yaw in examples:
        pose = make_pose(x, y, z, roll, pitch, yaw)
        go_pose(group, pose, label)


# ============================================================
#  命令行接口
# ============================================================

def cmd_pos(args):
    """python3 pose_control.py pos <x> <y> <z>"""
    x, y, z = float(args[0]), float(args[1]), float(args[2])
    g = moveit_commander.MoveGroupCommander('arm')
    pose = make_pose(x, y, z, 0, 0, 0)
    go_pose(g, pose, f"末端位置 ({x}, {y}, {z})")


def cmd_rpy(args):
    """python3 pose_control.py rpy <x> <y> <z> <roll> <pitch> <yaw>"""
    x, y, z = float(args[0]), float(args[1]), float(args[2])
    r, p, y = float(args[3]), float(args[4]), float(args[5])
    g = moveit_commander.MoveGroupCommander('arm')
    pose = make_pose(x, y, z, r, p, y)
    go_pose(g, pose, f"RPY ({x},{y},{z}) roll={r}° pitch={p}° yaw={y}°")


def cmd_quat(args):
    """python3 pose_control.py quat <x> <y> <z> <qx> <qy> <qz> <qw>"""
    x, y, z = float(args[0]), float(args[1]), float(args[2])
    qx, qy, qz, qw = float(args[3]), float(args[4]), float(args[5]), float(args[6])
    g = moveit_commander.MoveGroupCommander('arm')
    pose = make_pose_quat(x, y, z, qx, qy, qz, qw)
    go_pose(g, pose, f"Quat ({x},{y},{z}) [{qx},{qy},{qz},{qw}]")


def cmd_state():
    """查看当前姿态"""
    g = moveit_commander.MoveGroupCommander('arm')
    print("当前状态:")
    show(g)


def main():
    rospy.init_node('pose_ctrl')

    if len(sys.argv) < 2:
        print(__doc__)
        return

    cmd = sys.argv[1]
    args = sys.argv[2:]

    cmds = {
        'pos':   cmd_pos,
        'rpy':   cmd_rpy,
        'quat':  cmd_quat,
        'state': lambda _: cmd_state(),
        'demo':  lambda _: demo_defaults(
            moveit_commander.MoveGroupCommander('arm')),
    }

    if cmd in cmds:
        cmds[cmd](args)
    else:
        print(f"未知命令: {cmd}")
        print(__doc__)


if __name__ == '__main__':
    main()
