#!/usr/bin/env python3
"""
串口舵机驱动 — 作为 MoveIt FollowJointTrajectory action server

架构:
  MoveIt move_group
      ↓ FollowJointTrajectory action goal (轨迹点: positions, velocities)
  本节点 (servo_driver.py)
      ↓ 串口发送舵机指令
  真实舵机
      ↓ 读取位置反馈
  本节点 → publish /joint_states → MoveIt

用法:
  修改下方的串口参数和协议函数, 然后:
  python3 /home/hzr/arm_xingzai/arm/servo_driver.py
"""

import sys
import math
import time
import threading

import rospy
import actionlib
import serial
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

# ============================================================
#  硬件参数 — 根据你的机械臂修改
# ============================================================

SERIAL_PORT = "/dev/ttyUSB0"          # 串口设备
SERIAL_BAUD = 115200                    # 波特率
SERIAL_TIMEOUT = 0.1

JOINT_NAMES = ["j1", "j2", "j3", "j4"]  # 关节名 (与 URDF 一致)

# 角度偏移: 舵机编码值 = (angle_rad + offset) * encoder_per_rad
# 根据实际标定填入
ANGLE_OFFSETS = {
    "j1": 0.0,                          # j1 零位偏移 (rad)
    "j2": 0.0,
    "j3": 0.0,
    "j4": 0.0,
}

# 编码器分辨率: 每弧度对应编码值 (舵机通常是 0~1023 或 0~4096 对应 0~180deg)
# 例如: 舵机 0~180deg (0~pi rad), 编码 500~2500
#       encoder_per_rad = (2500-500) / pi ≈ 637
ENCODER_PER_RAD = {
    "j1": 637,                          # j1 舵机: 编码值/rad
    "j2": 637,
    "j3": 637,
    "j4": 637,
}

# 编码范围: 舵机允许的最小/最大编码值 (安全限位)
ENCODER_MIN = {
    "j1": 500,
    "j2": 500,
    "j3": 500,
    "j4": 500,
}
ENCODER_MAX = {
    "j1": 2500,
    "j2": 2500,
    "j3": 2500,
    "j4": 2500,
}

# 控制频率
CONTROL_RATE = 50   # Hz

# ============================================================
#  串口协议 — 根据你的控制协议文档修改下面的两个函数
# ============================================================

def build_command(joint_name, encoder_value):
    """
    构造发送给单个舵机的串口指令。
    返回 bytes。

    TODO: 按你的协议文档实现。

    示例 (常见舵机协议, 如 SCS/BUS 系列):
        # 位置指令格式: [0xFF, 0xFF, ID, len, WRITE, addr, pos_low, pos_high, checksum]
        servo_id = JOINT_IDS[joint_name]
        pos_low = encoder_value & 0xFF
        pos_high = (encoder_value >> 8) & 0xFF
        return bytes([0xFF, 0xFF, servo_id, 0x07, 0x03, 0x2A, pos_low, pos_high, ...])
    """
    # TODO: 替换为你的协议实现
    # 临时占位: 直接发送文本 "ID:value\n"
    joint_id = {"j1": 1, "j2": 2, "j3": 3, "j4": 4}
    sid = joint_id.get(joint_name, 1)
    return f"#{sid}:{encoder_value}\n".encode("ascii")


def parse_feedback(line):
    """
    解析串口返回的一行数据，返回 {joint_name: angle_rad} 或 None。

    TODO: 按你的协议文档实现。
    如果舵机不支持位置反馈，直接 return None (驱动会用指令值代替)。
    """
    # TODO: 替换为你的协议实现
    return None


# ============================================================
#  驱动逻辑 (一般不需要改)
# ============================================================

class ServoDriver:
    def __init__(self):
        self._lock = threading.Lock()
        self._current_positions = {jn: 0.0 for jn in JOINT_NAMES}  # rad

        # 打开串口
        try:
            self._ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD,
                                      timeout=SERIAL_TIMEOUT, write_timeout=1.0)
            rospy.loginfo(f"Serial {SERIAL_PORT} opened, baud={SERIAL_BAUD}")
        except serial.SerialException as e:
            rospy.logerr(f"Cannot open serial: {e}")
            sys.exit(1)

        # 关节状态发布
        self._js_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

        # FollowJointTrajectory action server
        self._as = actionlib.SimpleActionServer(
            "/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False)
        self._as.start()

        # 定时发布 joint_states
        self._timer = rospy.Timer(rospy.Duration(1.0 / CONTROL_RATE),
                                  self._publish_js)

        rospy.loginfo("Servo driver ready, waiting for MoveIt commands...")

    def _angle_to_encoder(self, joint_name, angle_rad):
        """角度 rad → 编码值，含偏移和限位"""
        offset = ANGLE_OFFSETS.get(joint_name, 0.0)
        epr = ENCODER_PER_RAD.get(joint_name, 637)
        enc = int((angle_rad + offset) * epr)
        lo = ENCODER_MIN.get(joint_name, 0)
        hi = ENCODER_MAX.get(joint_name, 4096)
        return max(lo, min(hi, enc))

    def _encoder_to_angle(self, joint_name, enc):
        """编码值 → 角度 rad"""
        offset = ANGLE_OFFSETS.get(joint_name, 0.0)
        epr = ENCODER_PER_RAD.get(joint_name, 637)
        return enc / epr - offset

    def _send_servo_command(self, joint_name, angle_rad):
        """发送单个舵机位置指令"""
        enc = self._angle_to_encoder(joint_name, angle_rad)
        cmd = build_command(joint_name, enc)
        with self._lock:
            self._ser.write(cmd)

    def _read_feedback(self):
        """读取串口返回, 更新当前位置"""
        with self._lock:
            if self._ser.in_waiting == 0:
                return
            try:
                line = self._ser.readline().decode("ascii", errors="ignore").strip()
                if not line:
                    return
                result = parse_feedback(line)
                if result:
                    self._current_positions.update(result)
            except Exception as e:
                rospy.logwarn_throttle(5, f"Serial read error: {e}")

    def _publish_js(self, event):
        """定时发布 /joint_states"""
        self._read_feedback()
        with self._lock:
            positions = [self._current_positions[jn] for jn in JOINT_NAMES]

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = JOINT_NAMES
        js.position = positions
        self._js_pub.publish(js)

    def _execute_cb(self, goal):
        """FollowJointTrajectory action 回调"""
        traj = goal.trajectory
        n_joints = len(traj.joint_names)
        if n_joints == 0:
            self._as.set_succeeded()
            return

        # 建立 joint_name → index 映射
        idx_map = {jn: i for i, jn in enumerate(traj.joint_names)}

        rospy.loginfo(f"Executing trajectory: {len(traj.points)} points")

        # 按时间戳执行轨迹点
        start_time = time.time()
        for pt in traj.points:
            # 检查是否被抢占
            if self._as.is_preempt_requested():
                rospy.loginfo("Trajectory preempted")
                self._as.set_preempted()
                return

            # 等待到该轨迹点的执行时间
            elapsed = time.time() - start_time
            wait_time = pt.time_from_start.to_sec() - elapsed
            if wait_time > 0:
                time.sleep(wait_time)

            # 发送每个关节的位置指令
            for jn in JOINT_NAMES:
                if jn in idx_map:
                    pos_idx = idx_map[jn]
                    if pos_idx < len(pt.positions):
                        angle = pt.positions[pos_idx]
                        self._current_positions[jn] = angle
                        self._send_servo_command(jn, angle)

            # 短暂等待舵机响应
            time.sleep(0.02)

        # 等待所有舵机到达目标位置
        settle_time = 0.3
        time.sleep(settle_time)

        rospy.loginfo("Trajectory complete")
        self._as.set_succeeded()

    def shutdown(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
            rospy.loginfo("Serial closed")


if __name__ == "__main__":
    rospy.init_node("servo_driver")
    driver = ServoDriver()
    rospy.on_shutdown(driver.shutdown)
    rospy.spin()
