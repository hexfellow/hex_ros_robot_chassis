# hex_ros_robot_chassis
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 控制模式](#4-控制模式)
- [5. 参数说明](#5-参数说明)
- [6. 依赖关系](#6-依赖关系)
- [7. 快速使用](#7-快速使用)

---

## 1. 包的简介

这是 **HEXFELLOW** 底盘的 **ROS 驱动包**。

- **Trigger A3 LR1** 是一款三轮全向底盘，配备三个全向轮，实现平面内三自由度运动（前后、左右、旋转）。

本包通过 WebSocket 连接底盘控制器，将 ROS 控制指令转发给硬件，并发布底盘实时状态、里程计、关节状态和 TF 变换。
支持 **ROS 1** 和 **ROS 2**。



---

## 2. 包架构

```
hex_ros_robot_chassis/
├── config/                              # 参数配置
│   ├── ros1/
│   │   └── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 1 参数
│   └── ros2/
│       └── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 2 参数
├── launch/                              # 启动文件
│   ├── ros1/
│   │   └── trigger_a3_lr1.launch        #   A3 LR1 ROS 1 启动
│   └── ros2/
│       └── trigger_a3_lr1.launch.py     #   A3 LR1 ROS 2 启动
├── hex_ros_robot_chassis/               # 核心代码
│   ├── robot_trigger_a3_lr1.py          #   A3 LR1 主节点（控制循环 + ROS 接口）
│   └── utility/                         #   公共 DataInterface
│       ├── __init__.py                  #     根据 ROS_VERSION 自动选择 ROS1/ROS2 实现
│       ├── interface_base.py            #     抽象基类 ChassisInterfaceBase
│       ├── ros1_interface.py            #     ROS 1 DataInterface 实现
│       └── ros2_interface.py            #     ROS 2 DataInterface 实现
├── resource/                            # ament 资源文件
├── setup.py                             # Python 打包配置（1 个 entry_point）
├── setup.cfg                            # Python 打包配置
├── package.xml                          # ROS 包清单（双系统条件依赖）
└── README.md                            # 英文文档
```

### 接口层说明

`utility/` 模块提供统一的 `DataInterface`，根据 `ROS_VERSION` 环境变量自动选择 ROS 1 或 ROS 2 实现：

| 实现 | 文件 | 适用版本 |
|------|------|---------|
| `ros1_interface.DataInterface` | `ros1_interface.py` | ROS 1（Noetic） |
| `ros2_interface.DataInterface` | `ros2_interface.py` | ROS 2（Humble / Foxy） |

---

## 3. 话题接口

### Trigger A3 LR1

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | 底盘控制指令（VEL / MIT 模式） |
| 发布 | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | 底盘状态反馈（3 个轮子的位置、速度、力矩） |
| 发布 | `odom` | `nav_msgs/(msg/)Odometry` | 里程计（位置 x, y, yaw + 速度 vx, vy, omega） |
| 发布 | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 个轮关节状态（`joint_wheel1` ~ `joint_wheel3`） |
| 发布 | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link 坐标系变换 |

> 消息类型定义见 [hex_ros_msgs](https://github.com/hexfellow/hex_ros_msgs)

> 默认提供ros时间；如果您需要硬件时间戳，可以通过`chs_state.jnt.header.stamp`获取


---

## 4. 控制模式

底盘支持两种控制模式，通过 `chs_ctrl.ctrl_mode` 选择：

| 模式 | 值 |  说明 |
|------|-----|------|
| `VEL` | `2` |  速度模式：下发 (vx, vy, omega) 三自由度速度指令 |
| `MIT` | `1` |  力矩模式：下发每个电机的目标转速 + 最大电流 |
| `NONE` | `0` | 空操作，不执行任何控制 |

> 目前底盘未支持MIT；当您使用MIT模式时，将会向底盘设备下发目标速度(rad/s)+最大限制电流(A)

### MIT 模式使用警告
- 除非你知道什么是 MIT 模式，否则不要使用该模式
- 使用不当可能导致底盘剧烈运动甚至损坏设备

> 确保在安全区域内运行，并随时准备急停

---

## 5. 参数说明

### Trigger A3 LR1

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ctrl_rate` | 1000.0 | 主控制循环频率 [Hz] |
| `rate_state` | 500.0 | 状态发布频率（从 ctrl_rate 降采样）[Hz] |
| `robot_host` | 192.168.1.100 | 底盘控制器 IP 地址 |
| `robot_port` | 8439 | WebSocket 端口 |
| `robot_frame_id` | `base_link` | 状态消息中的坐标系 |
| `state_buffer_size` | 200 | 驱动状态缓冲区大小 |
| `enable_kcp` | `true` | 是否启用 KCP 传输协议 |

---

## 6. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0a4'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

---

## 7. 快速使用

### 1. 创建工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash --extend
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash --extend
```

### 4. 使用包

```shell
# ROS 2 — Trigger A3 LR1
ros2 launch hex_ros_robot_chassis trigger_a3_lr1.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439
```

> 将 `robot_host` 和 `robot_port` 替换为实际底盘控制器的 IP 和端口。

### 5. 控制底盘

#### Trigger A3 LR1 快速使用

通过 `ros2 topic pub` 可快速向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# VEL 模式 — 停止 
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# 目标速度最大电流限制 模式 — 三个电机同速 0.3，电流限制 2.0
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# 目标速度最大电流限制 模式 — 停止
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

```

通过`rostopic pub`可快速向底盘发送控制指令：

```bash
# VEL 模式 — 旋转 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# VEL 模式 — 停止
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# 目标速度最大电流限制模式 — 三个电机同速 0.3，电流限制 2.0
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# 目标速度最大电流限制模式 — 停止
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> 目前底盘未支持MIT；当您使用MIT模式时，将会向底盘设备下发目标速度(rad/s)+最大限制电流(A)
