#!/usr/bin/env python3
"""
串口舵机直控测试 — 不依赖 MoveIt, 先验证串口通信和舵机控制

用法:
  python3 /home/hzr/arm_xingzai/arm/star__moveit_config/scripts/servo_test.py

先修改下面的 SERIAL_PORT 和协议参数, 然后运行测试每个关节。
"""

import sys
import math
import serial

# ============================================================
#  串口参数
# ============================================================
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

# ============================================================
#  舵机参数
# ============================================================
# 关节名 → 舵机ID
JOINT_IDS = {"j1": 1, "j2": 2, "j3": 3, "j4": 4}

# 编码器: 每弧度对应编码值
ENCODER_PER_RAD = {"j1": 637, "j2": 637, "j3": 637, "j4": 637}

# 角度零位偏移 (rad)
OFFSETS = {"j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}


# ============================================================
#  ★ 协议函数 — 根据你的文档实现 ★
# ============================================================

def build_command(joint_name, encoder_value):
    """
    构造串口指令，返回 bytes。
    TODO: 替换为你的协议实现。
    """
    sid = JOINT_IDS[joint_name]
    # 示例: SCS/BUS 系列舵机协议
    # pos_low = encoder_value & 0xFF
    # pos_high = (encoder_value >> 8) & 0xFF
    # ... checksum ...
    # return bytes([0xFF, 0xFF, sid, ...])
    #
    # 下面是文本协议示例，请替换:
    return f"#{sid}:{encoder_value}\n".encode("ascii")


def read_response(ser):
    """读取串口返回，返回字符串或 None"""
    if ser.in_waiting > 0:
        return ser.readline().decode("ascii", errors="ignore").strip()
    return None


# ============================================================
#  测试逻辑
# ============================================================

def angle_to_enc(joint_name, deg):
    """角度(deg) → 编码值"""
    rad = math.radians(deg)
    return int((rad + OFFSETS[joint_name]) * ENCODER_PER_RAD[joint_name])


def test_joint(ser, joint_name, deg):
    """测试单个关节移动到指定角度"""
    enc = angle_to_enc(joint_name, deg)
    cmd = build_command(joint_name, enc)
    print(f"  {joint_name} → {deg:.0f}deg  (enc={enc})  cmd={cmd!r}")
    ser.write(cmd)
    # 读取返回
    resp = read_response(ser)
    if resp:
        print(f"    返回: {resp}")


def main():
    print("串口舵机测试\n")
    print(f"串口: {SERIAL_PORT} @ {SERIAL_BAUD}")
    print(f"舵机ID: {JOINT_IDS}")
    print()

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
        print(f"✓ 串口已打开\n")
    except serial.SerialException as e:
        print(f"✗ 无法打开串口: {e}")
        return 1

    # 依次测试各关节
    test_angles = [0, 45, 0, -45, 0]  # 来回摆动, 观察舵机是否响应

    for jn in ["j1", "j2", "j3", "j4"]:
        print(f"--- 测试 {jn} (ID={JOINT_IDS[jn]}) ---")
        for a in test_angles:
            test_joint(ser, jn, a)
            import time
            time.sleep(1.5)  # 等待舵机完成动作

    ser.close()
    print("\n✓ 测试完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
