# Star_ 机械臂 — MoveIt + RViz 仿真说明

## 1. 环境

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| IK 求解器 | Trac-IK |
| 规划器 | OMPL (默认 RRTConnect) |

## 2. 编译

```bash
cd /home/hzr/arm_xingzai/arm
source /opt/ros/noetic/setup.bash
catkin build
```

## 3. 启动

```bash
source /opt/ros/noetic/setup.bash
source /home/hzr/arm_xingzai/arm/devel/setup.bash
roslaunch star__moveit_config demo.launch
```

启动后出现 3 个进程：

| 窗口 | 说明 |
|------|------|
| **RViz** | 3D 可视化 + MotionPlanning 规划面板 |
| **Joint State Publisher GUI** | 拖动滑块手动调节关节 |
| **终端 (move_group)** | MoveIt 服务器日志 |

## 4. 工作空间 (Workspace)

> 通过随机采样 1500 个点 + Trac-IK 逆运动学求解探测得到，1115 个可达点。

```
坐标系: base_link
单位: 米

  X:  -0.249  ~  +0.249
  Y:  -0.249  ~  +0.249
  Z:  -0.050  ~  +0.346
```

各高度层的可达半径：

| 高度 Z | 可达半径 |
|--------|----------|
| 0.02 m | 0.08 ~ 0.29 m |
| 0.10 m | 0.03 ~ 0.29 m |
| 0.18 m | 0.02 ~ 0.27 m |
| 0.26 m | 0.03 ~ 0.23 m |

**建议安全范围**（保证高成功率）：

```
x: -0.20 ~ 0.20
y: -0.20 ~ 0.20
z:  0.02 ~ 0.28
```

## 5. 关节范围

| 关节 | 说明 | 角度范围 | 弧度 |
|------|------|----------|------|
| j1 | 底座旋转 | -180° ~ +180° | ±3.14 |
| j2 | 肩部 | -90° ~ +90° | ±1.57 |
| j3 | 肘部 | -90° ~ +90° | ±1.57 |
| j4 | 腕部 | -90° ~ +90° | ±1.57 |

## 6. RViz 操作

### 视角控制

| 操作 | 按键 |
|------|------|
| 旋转 | 鼠标左键拖动 |
| 平移 | 鼠标中键拖动 |
| 缩放 | 滚轮 |

### 运动规划

1. 左侧 **MotionPlanning** 面板 → **Planning** 标签
2. 勾选 **Query Goal State** → 拖动末端彩色球到目标位置
3. 点击 **Plan** → 看到绿色轨迹线表示规划成功
4. 点击 **Execute** → 机械臂沿轨迹运动

## 7. 命令行控制

### 末端位置控制

```bash
# 指定末端位置 (x, y, z)，保持默认朝向
python3 /home/hzr/arm_xingzai/arm/pose_control.py pos 0.08 0 0.10

# 指定位置 + RPY 朝向 (度)
python3 /home/hzr/arm_xingzai/arm/pose_control.py rpy 0.08 0 0.10 0 90 0
#                                                x    y z    r  p  y

# 指定位置 + 四元数朝向
python3 /home/hzr/arm_xingzai/arm/pose_control.py quat 0.08 0 0.10 0 0.707 0 0.707

# 查看当前姿态
python3 /home/hzr/arm_xingzai/arm/pose_control.py state

# 演示各种姿态
python3 /home/hzr/arm_xingzai/arm/pose_control.py demo
```

### 演示示例

```bash
# 关节空间 + 末端位置综合演示
python3 /home/hzr/arm_xingzai/arm/example.py
```

### 探测工作空间

```bash
python3 /home/hzr/arm_xingzai/arm/workspace_probe.py
```

## 8. Python API 速查

```python
import rospy, moveit_commander
from pose_control import make_pose

rospy.init_node('my_script')
g = moveit_commander.MoveGroupCommander('arm')
g.set_planning_time(5.0)

# ── 关节角度 ──
g.set_joint_value_target([0, 0, 0, 0])       # 回 home
g.set_joint_value_target([0, 0.8, -0.8, 0.3]) # 前伸
g.set_named_target('home')                     # 预设位姿
g.go()

# ── 末端位置 ──
g.set_position_target([0.08, 0, 0.10])         # 只看位置
pose = make_pose(0.08, 0, 0.10, 0, 90, 0)     # 位置 + RPY(末端朝下)
g.set_pose_target(pose)
g.go()

# ── 碰撞检测 ──
from moveit_commander import PlanningSceneInterface
scene = PlanningSceneInterface()
# add_box(scene, "name", x, y, z, sx, sy, sz)  # 添加障碍物
# scene.remove_world_object("name")              # 移除

# ── 查看状态 ──
print(g.get_current_joint_values())             # [j1, j2, j3, j4] (弧度)
print(g.get_current_pose().pose.position)       # (x, y, z)
```

## 9. 监控命令

```bash
# 实时关节角度
rostopic echo /joint_states/position

# 末端位置
rosrun tf tf_echo base_link l4

# 规划状态
rostopic echo /move_group/status
```

## 10. 常见问题

**Q: 启动时有红色 ERROR？**  
A: `planning_pipelines` 相关错误是 MoveIt 1 的已知瞬态信息，不影响功能。

**Q: 末端位置规划失败 (TIMED_OUT)？**  
A: 检查目标是否在工作空间内（参考第 4 节范围）。太远、太高、太靠近底座中心都可能失败。

**Q: 没有碰撞检测？**  
A: 已启用。相邻 link 禁用碰撞（物理上不可能碰撞），不相邻 link 会检测自碰撞。添加障碍物用 `PlanningSceneInterface.add_box()`。

**Q: RViz 看不到机械臂？**  
A: `Global Options → Fixed Frame` 设为 `base_link`。

## 11. 文件结构

```
arm/
├── src/
│   ├── star_/                       # URDF 描述包
│   │   ├── urdf/star_.urdf
│   │   ├── meshes/                  # STL 网格
│   │   └── launch/
│   └── star__moveit_config/         # MoveIt 配置包
│       ├── config/
│       │   ├── star_.srdf            # 语义描述(规划组/碰撞对)
│       │   ├── kinematics.yaml       # Trac-IK 运动学
│       │   ├── joint_limits.yaml
│       │   ├── ompl_planning.yaml    # OMPL 规划器
│       │   └── fake_controllers.yaml
│       └── launch/
│           ├── demo.launch           # ★ 主入口
│           ├── move_group.launch
│           ├── moveit_rviz.launch
│           └── moveit.rviz
├── example.py                       # 演示脚本
├── pose_control.py                  # 末端姿态控制
├── workspace_probe.py               # 工作空间探测
└── test_plan.py                     # 规划测试
```
