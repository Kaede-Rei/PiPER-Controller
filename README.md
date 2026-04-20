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
| 主机 | Ubuntu 20.04 / 22.04 | 建议 >= 4 核 CPU, >= 8GB RAM |
| 通信 | USB-CAN | 推荐 `can0`，波特率 1000000 |

硬件连接示意：

```text
[24V 电源] ──► [PiPER 转接线] ──► [PiPER 机械臂]
                   │
            [Type-C / USB-CAN 到主机]
```

### 软件栈

- 操作系统：Ubuntu 20.04 LTS / Ubuntu 22.04 LTS
- ROS 版本：ROS Noetic
- 规划执行：MoveIt
- 控制核心：`piper_tomato/src/domain/piper_controller`
- 指令分发：`piper_tomato/src/service/piper_commander`
- ROS 接口：`piper_tomato/src/app/piper_interface`
- 任务接口：`piper_tomato/src/app/piper_task`
- GUI 工具：`piper_tomato/src/app/piper_gui`
- 消息定义：`piper_tomato/src/device/piper_msgs2`

---

## 📋 快速开始

### 1. 环境准备

#### 1.1 Ubuntu 20.04（原生 ROS Noetic）

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

# Python SDK (v0.2.20)
pip3 install piper_sdk
```

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 1.2 Ubuntu 22.04（通过 ros_env 在虚拟环境使用 ROS Noetic）

```bash
micromamba create -n ros_env -c conda-forge -c robostack-noetic \
  ros-noetic-desktop-full \
  ros-dev-tools \
  ros-noetic-moveit \
  ros-noetic-trac-ik-kinematics-plugin \
  ros-noetic-rosserial \
  ros-noetic-rosserial-python \
  compilers cxx-compiler c-compiler binutils sysroot_linux-64

micromamba activate ros_env
pip install python-can piper_sdk
```

---

### 2. 源码编译

仓库包含两个 catkin 工作区：`piper_ros` 与 `piper_tomato`。推荐使用下面的标准流程：

```bash
cd /path/to/piper-ws

# 先确保已 source 你的 ROS Noetic 环境
# 例如：source /opt/ros/noetic/setup.bash

cd piper_ros
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash

cd ../piper_tomato
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash
```

如需在 Ubuntu 22.04 上使用 `ros_env`，请先激活你自己的环境，再执行 `./ros_env/use-mamba-gcc.sh`；不要把编译产物或 `compile_commands.json` 软链接固定到仓库根目录。

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

#### 方式一：使用启动脚本（推荐）

```bash
# 基础启动
./piper-start.sh

# 启动并在中断时失能系统
./piper-start.sh --disable

# 启动并设置回零后等待时间（秒）
./piper-start.sh --delay 5

# 组合参数
./piper-start.sh --disable --delay 8

# 查看帮助
./piper-start.sh --help
```

启动脚本会自动：
- source ROS 环境（`piper_ros` 与 `piper_tomato`）
- 启动 `piper_interface` 节点
- 中断时执行回零操作
- 可选失能系统或设置延迟等待时间

#### 方式二：手动启动

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

若只想启动接口层（不拉起额外演示内容），可在 launch 参数中关闭相关开关。

### 5. 启动 GUI（可选）

`piper_gui` 已接入任务层 `PickTaskAction`，可用于图形化 ROI 选点并下发采摘任务：

```bash
source piper_tomato/devel/setup.bash
rosrun piper_gui piper_gui.py
```

说明：

- GUI 默认连接 `/pick_action`。
- GUI 启动前请确保 `piper_start.launch` 已正常运行。
- 或使用启动脚本直接启动：`./piper-start.sh`

### 6. 手眼采集与标定（hand-eye）

手眼工具位于 `hand-eye/`，数据默认读写到仓库根目录的 `picture/` 与 `outputs/`。

启动采集 GUI：

```bash
python ./hand-eye/hand_eye_capture_gui.py
```

命令行执行标定：

```bash
python ./hand-eye/pipper_hand_eye.py --min-index 1 --max-index 50
```

标定结果输出到：

```text
./outputs/handeye_时间戳/
```

包含 `best_result.json`、`ranked_candidates.json`、`sample_diagnostics.json`、`summary.txt`。

更多细节见：`hand-eye/README.md`。

---

## 🔌 ROS 接口

### Action

- `/move_arm`（`piper_msgs2/MoveArmAction`）
- `/simple_move_arm`（`piper_msgs2/SimpleMoveArmAction`）
- `/pick_action`（`piper_msgs2/PickTaskAction`）

补充：`piper_gui` 通过 `/pick_action` 进行任务组更新、任务写入与执行，不直接调用 `/move_arm` / `/simple_move_arm`。

### Service

- `/arm_config`（`piper_msgs2/ConfigArm`）
- `/arm_query`（`piper_msgs2/QueryArm`）
- `/eef_cmd`（`piper_msgs2/CommandEef`）

命令编号与字段说明见：

- `piper_tomato/PiPER 机械臂接口文档.md`

### 接口语义（当前实现）

- 接口层请求转换（Action Goal / Service Request -> `ArmCmdRequest`）使用 `tl::optional`，转换失败会直接返回无效请求。
- 命令执行结果中的当前状态（`current_pose` / `current_joints`）在分发层使用 `tl::optional`。
- `arm_interface` 会在返回 ROS 消息时做兜底：若结果中状态为空，则读取控制器当前状态并填充响应。
- `set_target_in_eef_frame` 会透传真实错误码（如 TF 变换失败），不再统一折叠为同一种失败原因。

### 代码层设计约定

- 类型命名采用无后缀风格（例如 `SearchReachablePose`、`ReachablePoseResult`、`AStarNode`），不再使用历史 `_t/_e`。
- `ArmCmdRequest.target` 继续使用 `variant + monostate` 表达目标类型分支。
- 除 `target` 外，输入转换与状态返回优先使用 `tl::optional` 表达“有值/无值”语义。
- 对外错误码优先保留真实来源，避免在接口层过度折叠错误原因。

---

## 🧪 快速验证

### 查询当前关节

```bash
rosservice call /arm_query "command_type: 12
values: []"
```

### 查询当前位姿

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

### 设置姿态约束

```bash
rosservice call /arm_config "command_type: 9
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

## ⚙️ 系统配置

### 启动参数

系统启动时从私有命名空间 `~start/...` 读取配置，对应文件为 [piper_tomato/src/app/piper_interface/config/config.yaml](piper_tomato/src/app/piper_interface/config/config.yaml)。

默认配置文件包含以下主要模块：

```yaml
start:
  arm_group_name: "arm"
  
  # 末端执行器配置
  eef:
    enabled: true
    type: "servo_gripper"   # two_finger_gripper | servo_gripper
    name: "gripper"
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200

  # Action 与 Service 接口
  arm_move_action:
    enabled: true
    name: "move_arm"
  simple_arm_move_action:
    enabled: true
    name: "simple_move_arm"
  arm_config_service:
    enabled: true
    name: "arm_config"
  arm_query_service:
    enabled: true
    name: "arm_query"
  pick_action:
    enabled: true
    name: "pick_action"
  eef_cmd_service:
    enabled: true
    name: "eef_cmd"

  # 轨迹参数（Decartes）
  decartes:
    vel_scale: 1.0
    acc_scale: 1.0
    eef_step: 0.01
    jump_threshold: 0.0
    min_success_rate: 0.8

  # 运动规划参数
  motion_planning:
    planning_time: 5.0
    planning_attempts: 10
    max_velocity_scaling_factor: 1.0
    max_acceleration_scaling_factor: 1.0
    planner_id: "RRTConnect"

  # 可达性搜索参数
  reachable_pose_search:
    step_deg: 5.0
    radius_deg: 60.0
```

### EEF 配置与 TCP 定义

末端执行器配置通过 `eef.*` 参数控制：

- `enabled=false`：不挂载 EEF，对应的 `/eef_cmd` Service 返回"未初始化"。
- `enabled=true` 且 `type=two_finger_gripper`：以 MoveIt 夹爪组方式集成。
- `enabled=true` 且 `type=servo_gripper`：以串口舵机夹爪方式集成。

TCP（Tool Center Point）配置说明：

- TCP 坐标关系统一从 URDF 定义，不再通过 `config.yaml` 配置 `tcp_offset`。
- 当前 `link_tcp` 固定在 `link6`，偏移为 `xyz=(0, 0.018, 0.13181)`、`rpy=(0, 0, 0)`。
- 对应文件：`piper_ros/src/piper_description/urdf/piper_description.urdf` 与 `piper_ros/src/piper_moveit/piper_with_gripper_moveit/config/gazebo_piper_description.urdf`。

---

## 🗂️ 项目结构

```text
piper-ws/
├── README.md
├── can-activate.sh
├── piper_test.py
├── hand-eye/                  # 手眼采集与标定工具（GUI + CLI）
├── picture/                   # 手眼数据集目录（按 index 存储）
├── outputs/                   # 手眼标定输出目录
├── piper_ros/                 # 官方 ROS 相关包工作区
├── piper_tomato/              # 控制主工作区
│   ├── src/
│   │   ├── app/
│   │   │   ├── piper_gui/       # 图形化任务下发工具（ROI + PickTaskAction）
│   │   │   ├── piper_interface/ # Action/Service 接口层
│   │   │   └── piper_task/      # 任务 Action（含 pick_action）
│   │   ├── service/
│   │   │   └── piper_commander/ # 命令分发层
│   │   ├── domain/
│   │   │   └── piper_controller/ # 运动控制层
│   │   └── device/
│   │       └── piper_msgs2/      # 消息与服务定义
│   └── PiPER 机械臂接口文档.md
├── piper_sdk/                 # Python SDK
└── ros_env/                   # 环境与工具脚本
```

---

## 🛠️ 常用脚本

| 脚本 | 功能 | 说明 |
|---|---|---|
| `can-activate.sh` | 激活 CAN 设备 | `sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up` |
| `piper-start.sh` | 一键启动系统 | 支持 `--disable` 和 `--delay` 参数；推荐方式 |
| `piper_test.py` | 接口快速验证 | 验证 Python SDK 连通性 |
| `hand-eye/hand_eye_capture_gui.py` | 手眼采集 GUI | 图形化手眼标定数据采集 |
| `hand-eye/pipper_hand_eye.py` | 手眼标定 CLI | 批量执行手眼标定计算 |
| `ros_env/source-piper.sh` | 快速加载环境 | Ubuntu 22.04 虚拟环境专用 |
| `piper_ros/can_activate.sh` | CAN 激活脚本 | 官方 ROS 包提供 |
| `piper_sdk/can_activate.sh` | CAN 激活脚本 | SDK 提供的替代脚本 |

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
- 降低速度/加速度缩放参数后重试（参考 `config.yaml` 中 `motion_planning` 配置）。
- 若返回 `TF_TRANSFORM_FAILED`，优先检查 TF 树与 URDF 中 `link_tcp` 定义。

### 5) 启动脚本问题

```bash
# 检查工作空间编译状态
ls piper_ros/devel/setup.bash piper_tomato/devel/setup.bash

# 查看脚本日志
./piper-start.sh 2>&1 | tail -20
```

### 6) 手眼标定数据不足

- 确保采集了足够的样本（推荐 >= 10 个）。
- 检查 `picture/` 目录中各 index 文件夹中是否都有 `board_detection.json` 和 `pose.json`。
- 查看 `outputs/` 中的 `sample_diagnostics.json` 了解标定质量。

### 7) 末端执行器不响应

```bash
# 检查串口连接
ls -la /dev/ttyACM* /dev/com-* 2>/dev/null

# 查看配置中的 serial_port 是否正确
grep serial_port piper_tomato/src/app/piper_interface/config/config.yaml
```

---

### 接口文档

详细的 ROS 接口说明、命令编号与字段定义见：

- [piper_tomato/PiPER 机械臂接口文档.md](piper_tomato/PiPER%20机械臂接口文档.md)

---

## 📚 参考资料

- ROS Noetic: http://wiki.ros.org/noetic
- MoveIt: https://moveit.ros.org/
- PiPER ROS: https://github.com/agilexrobotics/piper_ros
- PiPER SDK: https://github.com/agilexrobotics/piper_sdk

项目内文档：

- [piper_tomato/PiPER 机械臂接口文档.md](piper_tomato/PiPER%20机械臂接口文档.md) - 详细的 Action/Service 定义与命令编号
- [piper_tomato/src/app/piper_interface/config/config.yaml](piper_tomato/src/app/piper_interface/config/config.yaml) - 系统启动参数配置
- [piper_sdk/README.MD](piper_sdk/README.MD) - Python SDK 使用说明
- [hand-eye/README.md](hand-eye/README.md) - 手眼标定工具详细文档
- [piper_ros/README.MD](piper_ros/README.MD) - ROS 官方包文档

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
