# hex_ros_robot_chassis
[中文](README_CN.md) | **English**

## Table of Contents

- [1. Package Overview](#1-package-overview)
- [2. Package Architecture](#2-package-architecture)
- [3. Topic Interface](#3-topic-interface)
- [4. Control Modes](#4-control-modes)
- [5. Parameter Reference](#5-parameter-reference)
- [6. Dependencies](#6-dependencies)
- [7. Quick Start](#7-quick-start)

---

## 1. Package Overview

This is the **ROS Driver Package** for **HEXFELLOW** chassis.

- **Trigger A3 LR1** — a three-wheel omnidirectional chassis with three omni wheels, enabling 3-DoF planar motion (forward/backward, left/right, rotation).

The package connects to the chassis controller via WebSocket, forwards ROS control commands to the hardware, and publishes real-time chassis state, odometry, joint states, and TF transforms.
Supports **ROS 1** and **ROS 2**.

---

## 2. Package Architecture

```
hex_ros_robot_chassis/
├── config/                              # Parameter configuration
│   ├── ros1/
│   │   └── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 1 params
│   └── ros2/
│       └── trigger_a3_lr1_params.yaml   #   A3 LR1 ROS 2 params
├── launch/                              # Launch files
│   ├── ros1/
│   │   └── trigger_a3_lr1.launch        #   A3 LR1 ROS 1 launch
│   └── ros2/
│       └── trigger_a3_lr1.launch.py     #   A3 LR1 ROS 2 launch
├── hex_ros_robot_chassis/               # Core source
│   ├── robot_trigger_a3_lr1.py          #   A3 LR1 main node (control loop + ROS interface)
│   └── utility/                         #   Common DataInterface
│       ├── __init__.py                  #     Auto-selects ROS1/ROS2 impl via ROS_VERSION
│       ├── interface_base.py            #     Abstract base class ChassisInterfaceBase
│       ├── ros1_interface.py            #     ROS 1 DataInterface implementation
│       └── ros2_interface.py            #     ROS 2 DataInterface implementation
├── resource/                            # ament resource files
├── setup.py                             # Python packaging (1 entry_point)
├── setup.cfg                            # Python packaging config
├── package.xml                          # ROS package manifest (dual-system conditional deps)
└── README.md                            # English docs
```

### Interface Layer

`utility/` provides a unified `DataInterface` that automatically selects the ROS 1 or ROS 2 implementation based on the `ROS_VERSION` environment variable:

| Implementation | File | Compatible Versions |
|----------------|------|---------------------|
| `ros1_interface.DataInterface` | `ros1_interface.py` | ROS 1 (Noetic) |
| `ros2_interface.DataInterface` | `ros2_interface.py` | ROS 2 (Humble / Foxy) |

---

## 3. Topic Interface

### Trigger A3 LR1

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `chs_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboChsCtrlStamped` | Chassis control command (VEL / MIT modes) |
| Publish | `chs_state` | `hex_ros_msgs/(msg/)HexRosRoboChsStateStamped` | Chassis state feedback (3-wheel position, velocity, torque) |
| Publish | `odom` | `nav_msgs/(msg/)Odometry` | Odometry (position x, y, yaw + velocity vx, vy, omega) |
| Publish | `joint_states` | `sensor_msgs/(msg/)JointState` | 3 wheel joint states (`joint_wheel1` ~ `joint_wheel3`) |
| Publish | `/tf` | `tf2_msgs/(msg/)TFMessage` | odom → base_link transform |

> Message type definitions: [hex_ros_msgs](https://github.com/hexfellow/hex_ros_msgs)

> ROS timestamps are provided by default; for hardware timestamps, use `chs_state.jnt.header.stamp`.

---

## 4. Control Modes

The chassis supports two control modes, selected via `chs_ctrl.ctrl_mode`:

| Mode | Value | Description |
|------|-------|-------------|
| `VEL` | `2` | Velocity mode: send (vx, vy, omega) 3-DoF velocity commands |
| `MIT` | `1` | Torque mode: send target speed + max current per motor |
| `NONE` | `0` | No operation, no control executed |

> MIT mode is not currently supported by the **Trigger A3 lr**; when used, the driver will send target speed (rad/s) + max current limit (A) to the **Trigger A3 lr** device.

### MIT Mode Usage Warning
- Do not use MIT mode unless you understand what it does
- Improper use may cause violent chassis motion or equipment damage

> Always operate in a safe area and be ready for emergency stop.

---

## 5. Parameter Reference

### Trigger A3 LR1

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ctrl_rate` | 1000.0 | Main control loop rate [Hz] |
| `rate_state` | 500.0 | State publish rate (decimated from ctrl_rate) [Hz] |
| `robot_host` | 192.168.1.100 | Chassis controller IP address |
| `robot_port` | 8439 | WebSocket port |
| `robot_frame_id` | `base_link` | Frame ID in state message header |
| `state_buffer_size` | 200 | Driver state buffer size |
| `enable_kcp` | `true` | Enable KCP transport protocol |

---

## 6. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0a0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-util-runtime>=0.0.0,<0.1.0'
pip3 install 'hex-driver-robot>=0.1.0a4'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

---

## 7. Quick Start

### 1. Create a workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone the packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_robot_chassis.git
```

### 3. Build the packages

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash --extend
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash --extend
```

### 4. Launch

```shell
# ROS 2 — Trigger A3 LR1
ros2 launch hex_ros_robot_chassis trigger_a3_lr1.launch.py \
    robot_host:=192.168.1.100 robot_port:=8439
```

> Replace `robot_host` and `robot_port` with the actual chassis controller IP and port.

### 5. Send Control Commands

#### Quick Test with Trigger A3 LR1

Use `ros2 topic pub` to send control commands:

```bash
# VEL mode — rotate 0.3 m/s
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}'

# VEL mode — stop
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# Target speed + max current mode — 3 motors at 0.3, current limit 2.0
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

# Target speed + max current mode — stop
ros2 topic pub --once /chs_ctrl hex_ros_msgs/msg/HexRosRoboChsCtrlStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

Use `rostopic pub` for ROS 1:

```bash
# VEL mode — rotate 0.3 m/s
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}}"

# VEL mode — stop
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 2, jnt: {pos: [], vel: [], eff: [], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Target speed + max current mode — 3 motors at 0.3, current limit 2.0
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.3, 0.3, 0.3], eff: [2.0, 2.0, 2.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"

# Target speed + max current mode — stop
rostopic pub --once /chs_ctrl hex_ros_msgs/HexRosRoboChsCtrlStamped "{header: {stamp: 0, frame_id: 'base_link'}, chs_ctrl: {ctrl_mode: 1, jnt: {pos: [], vel: [0.0, 0.0, 0.0], eff: [0.0, 0.0, 0.0], kp: [], kd: [], lim_vel: [], lim_acc: []}, vel: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

> MIT mode is not currently supported by the **Trigger A3 lr**; when used, the driver will send target speed (rad/s) + max current limit (A) to the **Trigger A3 lr** device.
