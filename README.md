<div align="center">

# PiPER-ROS: Piper 机械臂 ROS 控制系统

一个基于 ROS Noetic + MoveIt 的 PiPER 机械臂控制框架，覆盖硬件接入、运动规划、Action/Service 控制接口与 EEF 挂载能力。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Framework: MoveIt!](https://img.shields.io/badge/Framework-MoveIt!-green.svg)](https://moveit.ros.org/)
[![Hardware: PiPER](https://img.shields.io/badge/Hardware-PiPER-red.svg)](https://www.agilex.ai/)

</div>

## 🤖 系统架构

### 硬件配置

| 项目 | 规格 | 说明 |
|---|---|---|
| 机械臂 | PiPER 6-DOF | 6 关节 + 可挂载末端执行器 |
| 主机 | Ubuntu 20.04 | 建议 >= 4 核 CPU, >= 8GB RAM |
| 通信 | USB-CAN | 推荐 `can0`，波特率 1000000 |

硬件连接示意：

```text
[24V 电源] ──► [PiPER 转接线] ──► [PiPER 机械臂]
                   │
            [Type-C / USB-CAN 到主机]
```

### 软件栈

- 操作系统：Ubuntu 20.04 LTS
- ROS 版本：ROS Noetic
- 规划执行：MoveIt
- 控制核心：`piper_tomato/src/piper_controller`
- 指令分发：`piper_tomato/src/piper_commander`
- ROS 接口：`piper_tomato/src/piper_interface`
- 消息定义：`piper_tomato/src/piper_msgs2`

---

## 📋 快速开始

### 1. 环境准备

#### 1.1 系统依赖

```bash
sudo apt update

# 基础工具
sudo apt install -y git build-essential cmake python3-dev curl

# ROS Noetic（如未安装）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# MoveIt / 控制器
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-controller-manager \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-joint-state-controller

# CAN 工具
sudo apt install -y can-utils ethtool

# 串口权限
sudo usermod -aG dialout $USER
```

#### 1.2 ROS 环境

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 2. 源码编译

仓库包含两个 catkin 工作区：`piper_ros` 与 `piper_tomato`。

```bash
cd /path/to/piper-ws

. ./ros_env/use-mamba-gcc.sh

cd piper_ros
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash

cd ../piper_tomato
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash

cd ..
ln -sf ../piper_tomato/build/compile_commands.json ./build/compile_commands.json
```

---

### 3. CAN 激活

```bash
# 推荐
source can-activate.sh

# 或手动
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

ip link show can0
```

---

### 4. 启动系统

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

若只想启动接口层（不拉起额外演示内容），可在 launch 参数中关闭相关开关。

---

## 🔌 ROS 接口

### Action

- `/move_arm`（`piper_msgs2/MoveArmAction`）
- `/simple_move_arm`（`piper_msgs2/SimpleMoveArmAction`）

### Service

- `/arm_config`（`piper_msgs2/ConfigArm`）
- `/arm_query`（`piper_msgs2/QueryArm`）

命令编号与字段说明见：

- `piper_tomato/PiPER 机械臂接口文档.md`

---

## 🧪 快速验证

### 查询当前关节

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

### 查询当前位姿

```bash
rosservice call /arm_query "command_type: 14
values: []"
```

### 设置姿态约束

```bash
rosservice call /arm_config "command_type: 10
point: {x: 0.0, y: 0.0, z: 0.0}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: []"
```

### Python 接口连通性测试

```bash
python piper_test.py
```

---

## ⚙️ EEF 与 TCP 偏移配置

配置文件：`piper_tomato/src/piper_interface/config/config.yaml`

```yaml
start:
  eef:
    enabled: true
    type: "servo_gripper"   # two_finger_gripper | servo_gripper
    name: "gripper"
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    tcp_offset:
      x: 0.0
      y: 0.0
      z: 0.1
      qx: 0.0
      qy: 0.0
      qz: 0.0
      qw: 1.0
```

说明：

- `enabled=false`：不挂载 EEF。
- `type=two_finger_gripper`：MoveIt 夹爪组方式。
- `type=servo_gripper`：串口舵机夹爪方式。
- `tcp_offset`：`flange -> tcp` 变换。

---

## 🗂️ 项目结构

```text
piper-ws/
├── README.md
├── can-activate.sh
├── piper_test.py
├── piper_ros/                 # 官方 ROS 相关包工作区
├── piper_tomato/              # 控制主工作区
│   ├── src/
│   │   ├── piper_interface/   # Action/Service 接口层
│   │   ├── piper_commander/   # 命令分发层
│   │   ├── piper_controller/  # 运动控制层
│   │   └── piper_msgs2/       # 消息与服务定义
│   └── PiPER 机械臂接口文档.md
├── piper_sdk/                 # Python SDK
└── ros_env/                   # 环境与工具脚本
```

---

## 🛠️ 常用脚本

| 脚本 | 功能 |
|---|---|
| `can-activate.sh` | 激活 CAN 设备 |
| `piper-start.sh` | 一键启动流程脚本 |
| `piper_test.py` | 接口快速验证 |
| `ros_env/source-piper.sh` | 快速加载环境 |

---

## 🔧 故障排查

### 1) CAN 未启动

```bash
ip link show can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2) 串口权限不足

```bash
groups $USER
sudo usermod -aG dialout $USER
```

重新登录后生效。

### 3) 服务/Action 不可见

```bash
source piper_tomato/devel/setup.bash
rosservice list | grep arm_
rostopic list | grep move_arm
```

### 4) 规划失败

- 检查目标位姿是否超出工作空间。
- 检查当前约束参数是否过严。
- 降低速度/加速度缩放参数后重试。

---

## 📚 参考资料

- ROS Noetic: http://wiki.ros.org/noetic
- MoveIt: https://moveit.ros.org/
- PiPER ROS: https://github.com/agilexrobotics/piper_ros
- PiPER SDK: https://github.com/agilexrobotics/piper_sdk

项目内文档：

- `piper_tomato/PiPER 机械臂接口文档.md`
- `piper_tomato/src/piper_interface/config/config.yaml`
- `piper_sdk/README.MD`

---

## 🙏 致谢

首先感谢松灵官方（AgileX Robotics）提供 PiPER 机械臂生态与开源基础：

- PiPER ROS: https://github.com/agilexrobotics/piper_ros
- PiPER SDK: https://github.com/agilexrobotics/piper_sdk

同时感谢以下优秀开源项目：

- trac-ik: https://github.com/HIRO-group/trac_ik
- serial: https://github.com/wjwwood/serial
- tl-optional: https://github.com/TartanLlama/optional

---

## 📝 许可证

MIT License，详见 `LICENSE`。

## 👥 贡献

欢迎提交 Issue / PR。
