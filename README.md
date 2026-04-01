<div align="center">

# PiPER-ROS: Piper 机械臂 ROS 控制系统

一个基于 ROS Noetic 和 MoveIt! 的松灵 PiPER 机械臂控制框架，集成了从硬件接口整合、运动规划到高层服务的全流程控制系统，应用于广东省农科院樱桃番茄采摘机器人

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Framework: MoveIt!](https://img.shields.io/badge/Framework-MoveIt!-green.svg)](https://moveit.ros.org/)
[![Hardware: PiPER](https://img.shields.io/badge/Hardware-PiPER%20-red.svg)](https://www.agilex.ai/page/690abe7b5e78cfa260412c92?mi=1&rn=PIPER+%E6%A0%87%E5%87%86%E7%89%88)

</div>

## 🤖 系统架构

### 硬件配置

|  |  |  |
|---|---|---|
| 机械臂 | PiPER 6-DOF 机械臂 | 6 个可控关节 + 末端执行器 |
| 主控 | Ubuntu 20.04 工控机/计算机 | >= 4 核 CPU, >= 8GB RAM |

硬件连接示意：

```
[24V 电源] ──► [PiPER 转接线] ──► [PiPER 机械臂]
                   │
            [Type-C to 工控机]
```

### 软件栈

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **核心框架**: Piper SDK + ROS 中间件层
- **Python SDK**: `piper_sdk` (Piper 官方 Python 库，支持 Protocol V1/V2)
- **控制器**: `joint_state_controller` + `joint_trajectory_controller`
- **规划器**: RRTConnect / CHOMP / STOMP / Pilz Industrial Motion Planner
- **业务服务**: `piper_service` (末端位姿控制和任务规划服务)
- **通信协议**: CAN 总线 (Piper 电机) + ROS 服务/话题

## 📋 快速开始

> ⚠️ 当前仓库存在两条链路：
>
> - 旧链路：`piper_service` + `/piper_server/*` 服务
> - 新链路（推荐）：`piper_tomato` + `piper_interface`（`/move_arm`、`/simple_move_arm`、`/arm_config`、`/arm_query`）
>
> 本 README 下方大部分示例仍是旧链路接口。若你正在开发 `piper_tomato`，请优先使用新链路接口。

### 1. 环境准备

#### 系统依赖

```bash
# Ubuntu 20.04 系统
sudo apt update

# 安装基础开发工具
sudo apt install git build-essential cmake python3-dev

# 安装 ROS Noetic（如果未安装）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装 ROS 控制器包
sudo apt install ros-noetic-controller-manager \
                 ros-noetic-joint-trajectory-controller \
                 ros-noetic-joint-state-controller \
                 ros-noetic-effort-controllers \
                 ros-noetic-position-controllers

# 安装 CAN 工具
sudo apt install can-utils ethtool

# 配置串口权限
sudo usermod -aG dialout $USER
sudo reboot  # 重启使权限生效
```

#### ROS 环境

```bash
# 在 ~/.bashrc 中添加 ROS 环境配置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Python SDK 安装

```bash
# 安装 python-can (版本 > 3.3.4)
pip3 install python-can>=3.3.4

# 安装 piper_sdk
pip3 install piper_sdk

# 查看 piper_sdk 信息
pip3 show piper_sdk
```

### 2. 源码获取与编译

```bash
# 创建工作空间
mkdir -p ~/piper_ws/src
cd ~/piper_ws/src

# 克隆项目
git clone https://github.com/Kaede-Rei/PiPER-Controller.git .

# 初始化工作空间
cd ~/piper_ws/src
catkin_init_workspace

# 编译
cd ~/piper_ws
catkin_make

# 配置环境
source devel/setup.bash
echo "source ~/piper_ws/devel/setup.bash" >> ~/.bashrc
```

### 3. 硬件连接

#### 电源连接

1. 机械臂供电: 24V 电源 → Piper 转接线 → 机械臂

#### 通信连接

1. USB-CAN 适配器: Type-C 线连接到工控机/计算机
2. 端口识别:

```bash
# 查看串口设备
ls -l /dev/ttyACM*  # 推荐接口（CAN 适配器常用）
ls -l /dev/ttyUSB*  # 备用接口

# 查看设备详细信息
dmesg | grep tty

# 查看 USB 设备
lsusb
```

#### 端口配置

编辑启动脚本 [piper-start.sh](piper-start.sh) 中的串口配置：

```bash
# 检查脚本中默认的串口设备配置（通常为 /dev/ttyACM0）
# 如需修改，编辑 piper_service 下的配置文件
```

### 4. 基础测试

#### 激活 CAN 设备

```bash
# 方法 1：使用提供的脚本（推荐）
source can-activate.sh

# 方法 2：手动激活（假设设备为 can0，波特率 1000000）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证
ip link show can0
```

#### 启动系统

```bash
# 新链路推荐启动（piper_tomato）
roslaunch piper_interface piper_start.launch

# 可选：只启动接口层（不自动拉起 MoveIt demo）
roslaunch piper_interface piper_start.launch start_moveit:=false

# 兼容旧链路启动脚本（仍可用，但接口命名不同）
# ./piper-start.sh

# 可选参数：
# ./piper-start.sh --disable  # 中断时失能机械臂
# ./piper-start.sh --delay 5  # 回零后延迟 5 秒
```

启动成功标志：

```
[INFO] [timestamp]: =====================================
[INFO] [timestamp]:    Piper 控制接口已启动
[INFO] [timestamp]: =====================================
[INFO] [timestamp]: 可用接口：
[INFO] [timestamp]:   * /move_arm (action)
[INFO] [timestamp]:   * /simple_move_arm (action)
[INFO] [timestamp]:   * /arm_config (service)
[INFO] [timestamp]:   * /arm_query (service)
[INFO] [timestamp]: =====================================
```

#### 测试服务

```bash
# 运行新链路测试脚本
python3 piper_test.py
```

### 新链路快速验证

#### 查询当前状态（`arm_query`）

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

```bash
rosservice call /arm_query "command_type: 14
values: []"
```

#### 设置约束（`arm_config`）

```bash
rosservice call /arm_config "command_type: 10
point: {x: 0.0, y: 0.0, z: 0.0}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: []"
```

## 📊 完整使用流程

### 阶段 1: 命令行快速控制

#### 末端位姿控制服务

服务名称: `/piper_server/eef_cmd`
服务类型: `piper_msgs_srvs::piper_cmd`

**常用命令**

|命令|功能|示例|
|---|---|---|
| zero | 回到零点（初始位置） | rosservice call /piper_server/eef_cmd "command: 'zero'" |
| goal_base | 基座坐标系下设置末端目标位姿 | command: 'goal_base', x: 0.3, y: 0.0, z: 0.5 |
| goal_eef | 末端坐标系下设置相对目标位姿 | command: 'goal_eef', x: 0.1 (末端沿x轴移动0.1米) |
| get_pose | 获取当前末端位姿（基座坐标系） | command: 'get_pose' |
| get_joints | 获取所有关节当前角度 | command: 'get_joints' |

**示例：设置末端位姿**

```bash
rosservice call /piper_server/eef_cmd "
command: 'goal_base'
x: 0.3
y: 0.0
z: 0.5
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
```

**示例：获取当前位姿**

```bash
rosservice call /piper_server/eef_cmd "
command: 'get_pose'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"

# 响应示例：
# success: True
# message: "当前末端位姿获取成功"
# cur_x: 0.3
# cur_y: 0.0
# cur_z: 0.5
# cur_roll: 0.0
# cur_pitch: 0.0
# cur_yaw: 0.0
```

#### 任务组规划服务

服务名称: `/piper_server/task_planner`
服务类型: `piper_msgs_srvs::piper_cmd`

**任务规划命令**

|命令|功能|参数说明|
|---|---|---|
| add_task | 添加任务到队列 | x,y,z,roll,pitch,yaw: 目标位姿, param1: 等待时长(秒) |
| clear_tasks | 清空任务队列 | 无需参数 |
| exe_all_tasks | 按顺序执行所有任务 | 无需参数 |

**示例：任务序列控制**

```bash
# 1. 清空任务队列
rosservice call /piper_server/task_planner "command: 'clear_tasks'"

# 2. 添加任务1：移动到位置 A 并等待 2 秒
rosservice call /piper_server/task_planner "
command: 'add_task'
x: 0.3
y: 0.1
z: 0.4
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '2.0'
param2: ''
param3: ''"

# 3. 添加任务2：移动到位置 B
rosservice call /piper_server/task_planner "
command: 'add_task'
x: 0.2
y: -0.1
z: 0.3
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: ''
param3: ''"

# 4. 执行所有任务
rosservice call /piper_server/task_planner "command: 'exe_all_tasks'"
```

### 阶段 2: Python 客户端编程

#### 安装依赖

```bash
pip3 install rospy
```

#### 示例代码：基础控制

```python
#!/usr/bin/env python3
import rospy
from piper_msgs_srvs.srv import piper_cmd, piper_cmdRequest

def move_to_pose(x, y, z, roll=0, pitch=0, yaw=0):
    """移动机械臂末端到指定位姿"""
    rospy.wait_for_service('/piper_server/eef_cmd')
    try:
        eef_cmd = rospy.ServiceProxy('/piper_server/eef_cmd', piper_cmd)
        req = piper_cmdRequest()
        req.command = "goal_base"
        req.x = x
        req.y = y
        req.z = z
        req.roll = roll
        req.pitch = pitch
        req.yaw = yaw
        
        resp = eef_cmd(req)
        if resp.success:
            rospy.loginfo(f"成功移动到位姿: ({x}, {y}, {z})")
        else:
            rospy.logerr(f"移动失败: {resp.message}")
        return resp
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def get_current_pose():
    """获取当前末端位姿"""
    rospy.wait_for_service('/piper_server/eef_cmd')
    try:
        eef_cmd = rospy.ServiceProxy('/piper_server/eef_cmd', piper_cmd)
        req = piper_cmdRequest()
        req.command = "get_pose"
        
        resp = eef_cmd(req)
        rospy.loginfo(f"当前位姿: x={resp.cur_x:.3f}, y={resp.cur_y:.3f}, z={resp.cur_z:.3f}")
        return resp
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == "__main__":
    rospy.init_node('piper_client_example')
    
    # 回到零点
    move_to_pose(0, 0, 0)
    rospy.sleep(2)
    
    # 移动到目标位姿
    move_to_pose(0.3, 0.0, 0.5)
    rospy.sleep(2)
    
    # 获取当前位姿
    get_current_pose()
```

#### 使用 Piper SDK 进行低级控制

```python
#!/usr/bin/env python3
from piper_sdk import PiperInterface

# 初始化接口 (使用 V2 协议)
piper = PiperInterface()

# 连接到机械臂
piper.connect("/dev/ttyACM0", protocol_version="v2")

# 归零
piper.go_to_zero()

# 设置关节角度（弧度）
piper.set_joint_angles([0.0, 0.523, 0.523, 0.0, 0.0, 0.0], speed=0.5)

# 获取当前关节角度
angles = piper.get_joint_angles()
print(f"当前关节角度: {angles}")

# 断开连接
piper.disconnect()
```

## 🛠️ 配置与定制

### 项目结构

```
PiPER-Controller/
├── piper_controller/                      # ROS 工作空间
│   ├── src/
│   │   ├── piper_controller/              # 机械臂控制节点
│   │   │   ├── include/
│   │   │   ├── src/
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── piper_msgs_srvs/               # 自定义消息和服务
│   │   │   ├── srv/
│   │   │   │   └── piper_cmd.srv          # 机械臂控制服务定义
│   │   │   ├── msg/
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   ├── piper_service/                 # 业务服务节点
│   │   │   ├── include/
│   │   │   ├── launch/
│   │   │   │   └── piper_service.launch
│   │   │   ├── src/
│   │   │   │   └── piper_service.cpp      # 服务器主程序
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   │
│   │   └── serial_driver/                 # 串口和 CAN 驱动
│   │       ├── include/
│   │       ├── src/
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   │
│   ├── devel/                             # 编译输出目录
│   ├── build/                             # 构建目录
│   └── CMakeLists.txt
│
├── piper_ros/                             # Piper 官方 ROS 包（源）
│   └── src/                               # 包含 Piper 描述文件、仿真等
│
├── piper_sdk/                             # Piper 官方 Python SDK
│   ├── base/                              # 基础类和接口
│   ├── protocol/                          # CAN 协议解析
│   ├── demo/                              # 示例代码
│   │   ├── V1/                            # Protocol V1 示例
│   │   └── V2/                            # Protocol V2 示例
│   ├── asserts/                           # 文档和说明
│   └── README.MD
│
├── scripts/                               # 工具脚本
│   └── zero.py                            # 回零脚本
│
├── Piper机械臂服务器客户端接口文档.md     # API 接口详细文档
├── README.md                              # 本文档
├── LICENSE                                # MIT 许可证
├── piper-start.sh                         # 启动脚本（含回零、失能参数）
├── can-activate.sh                        # CAN 设备激活脚本
├── can_find_and_config.sh                 # CAN 设备查找和配置脚本
└── test_service.sh                        # 服务测试脚本
```

### 常见配置

#### 串口设备配置

编辑 `piper_service` 的源代码中的运行参数，通常在启动脚本或 launch 文件中指定串口设备：

```bash
# 启动时指定串口设备
roslaunch piper_service piper_service.launch port:=/dev/ttyACM0
```

#### CAN 波特率配置

```bash
# 激活 CAN 设备时指定波特率（通常使用 1000000 bps）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 或通过脚本
source can-activate.sh can0 1000000
```

## 🛠️ 工具脚本快速参考

### 启动脚本

|脚本|功能|用法|
|---|---|---|
| piper-start.sh | 一键启动机械臂系统并处理中断 | ./piper-start.sh [--disable] [--delay 秒数] |
| can-activate.sh | 激活 USB-CAN 设备 | source can-activate.sh [device_name] [bitrate] |
| can_find_and_config.sh | 查找并配置 CAN 设备 | source can_find_and_config.sh |
| test_service.sh | 测试所有服务接口 | source test_service.sh |

### 启动示例

```bash
# 基础启动
./piper-start.sh

# 启动并在中断时失能机械臂
./piper-start.sh --disable

# 启动并在回零后延迟 5 秒
./piper-start.sh --delay 5

# 启动并配置回零延迟和失能
./piper-start.sh --disable --delay 8
```

## 📚 高级用法

### 多个 CAN 设备同时使用

当需要同时连接多个 USB-CAN 适配器（如机械臂和底盘）时：

```bash
# 1. 查找所有 CAN 设备的 USB 地址
bash find_all_can_port.sh

# 2. 记录各设备的 bus-info 信息（如 1-2:1.0，2-1:1.0）

# 3. 使用脚本配置指定设备
bash can_find_and_config.sh

# 4. 激活对应的 CAN 设备
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

### 自定义任务规划

扩展 `piper_service` 中的任务规划功能，实现复杂的机器人任务：

1. 编辑 [Piper机械臂服务器客户端接口文档.md](Piper机械臂服务器客户端接口文档.md) 了解服务格式
2. 修改 `piper_service/src/` 下的服务实现
3. 重新编译：`catkin_make`

### 使用 Piper SDK 进行低级控制

```python
from piper_sdk import PiperInterface
import time

# 创建接口
piper = PiperInterface()

# 连接（支持 V1 和 V2 协议）
piper.connect("/dev/ttyACM0", protocol_version="v2")

# 获取系统信息
info = piper.get_system_info()
print(f"Piper Version: {info.version}")

# 控制关节
piper.set_joint_angles(
    [0.0, 0.523, 0.523, 0.0, 0.0, 0.0],  # 关节角度（弧度）
    speed=1.0,                             # 速度缩放 (0.0~1.0)
    timeout=5.0                            # 超时时间（秒）
)

# 等待运动完成
time.sleep(3)

# 获取反馈
feedback = piper.get_joint_state()
print(f"当前关节角度: {feedback.angles}")
print(f"当前关节速度: {feedback.velocities}")
print(f"当前关节力矩: {feedback.torques}")

# 断开连接
piper.disconnect()
```

更多示例代码见 [`piper_sdk/demo/`](piper_sdk/demo/)

## 🔧 故障排查

### 串口连接问题

#### 问题：找不到串口设备

```bash
# 诊断所有 USB 设备
lsusb

# 查看串口设备
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*

# 查看设备权限
ls -l /dev/ttyACM0

# 查看内核消息
dmesg | grep tty
```

解决方案：

1. 确认用户在 `dialout` 组：

```bash
groups $USER
sudo usermod -aG dialout $USER
sudo reboot
```

2. 检查 USB 连接和驱动：

```bash
# 查看 USB 设备详细信息
lsusb -v | grep -A 10 "Piper\|CAN"
```

#### 问题：串口权限不足

错误消息：`PermissionError: [Errno 13] Permission denied: '/dev/ttyACM0'`

临时解决方案：

```bash
sudo chmod 666 /dev/ttyACM0
```

永久解决方案：

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

#### 问题：串口被其他程序占用

```bash
# 查看占用串口的进程
lsof /dev/ttyACM0

# 终止占用进程
kill -9 <PID>
```

### CAN 总线问题

#### 问题：CAN 设备无法激活

```bash
# 检查 CAN 设备是否存在
ifconfig -a | grep can

# 手动激活 CAN 设备
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证 CAN 设备状态
ip link show can0
```

#### 问题：CAN 通信超时

1. 检查硬件连接
2. 验证波特率配置（通常 1000000 bps）
3. 检查电源连接（24V）
4. 验证 USB-CAN 适配器驱动

### 硬件接口问题

#### 问题：机械臂无响应

诊断步骤：

1. 检查电源连接（24V 电源）
2. 确认 CAN 总线连接正常
3. 尝试使用 `piper_sdk` 的 demo 进行低级测试
4. 检查串口是否正确配置

```bash
# 测试串口通信
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 115200); print('串口连接成功:', s)"
```

#### 问题：关节不动或响应缓慢

解决方案：

1. 检查机械臂电源是否正常（LED 指示灯状态）
2. 尝试降低控制频率
3. 增加超时时间
4. 查看 ROS 日志了解详细错误信息

### 服务测试问题

#### 问题：服务无法调用

```bash
# 查看可用的 ROS 服务
rosservice list

# 调用服务进行测试
rosservice call /piper_server/eef_cmd "command: 'get_pose'"

# 查看服务类型
rosservice type /piper_server/eef_cmd

# 查看服务详细信息
rosservice args /piper_server/eef_cmd
```

#### 问题：任务执行失败

1. 确保机械臂已启动并处于正常状态
2. 检查目标位姿是否在工作空间内
3. 查看 ROS 日志了解错误信息：

```bash
# 查看 ROS 日志
rosparam get /rosdistro
rostopic echo /rosout
```

### 日志与调试

#### 启用详细日志

创建 `~/.ros/config/rosconsole.config`：

```
log4j.logger.ros=DEBUG
log4j.logger.ros.piper=DEBUG
```

#### 查看实时日志

```bash
# 查看所有日志
rostopic echo /rosout

# 查看 Piper 服务日志
rostopic echo /rosout | grep piper
```

#### 记录 rosbag

```bash
# 记录所有话题
rosbag record -a

# 记录特定话题
rosbag record /joint_states /tf /piper_server/feedback

# 回放 rosbag
rosbag play your_bag.bag
```

#### 使用 rqt 工具

```bash
# 启动 rqt 图形界面
rqt

# 常用插件：
# - rqt_graph: 查看节点和话题连接图
# - rqt_plot: 实时绘制话题数据
# - rqt_console: 查看日志消息
# - rqt_reconfigure: 动态调整参数
```

## 📖 参考文献与资源

### 官方文档

- **ROS Noetic**: [http://wiki.ros.org/noetic](http://wiki.ros.org/noetic)
- **松灵 PiPER 官网**: [https://www.agilex.ai/page/690abe7b5e78cfa260412c92?mi=1&rn=PIPER+标准版](https://www.agilex.ai/page/690abe7b5e78cfa260412c92?mi=1&rn=PIPER+%E6%A0%87%E5%87%86%E7%89%88)
- **PiPER ROS**: [https://github.com/agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros)
- **PiPER SDK**: [https://github.com/agilexrobotics/piper_sdk](https://github.com/agilexrobotics/piper_sdk)

### 本项目文档

- [PiPER 机械臂服务器客户端接口文档](Piper机械臂服务器客户端接口文档.md)
- [PiPER SDK README](piper_sdk/README.MD)
- [PiPER SDK 演示代码](piper_sdk/demo/)

## 📝 许可证

本项目采用 MIT License 开源，详见 [LICENSE](LICENSE) 文件

## 👥 贡献与支持

欢迎提交 Issues 和 Pull Requests！

## 📬 联系方式

如有问题或建议，请通过以下方式联系：

- GitHub Issues
- 提交讨论 (GitHub Discussions)

## 🙏 致谢

感谢以下项目和社区的支持：

- [ROS](http://www.ros.org/) 开源机器人操作系统
- [松灵](https://www.agilex.ai/) 提供优秀的 PiPER 机械臂产品和 SDK
- 所有贡献者和用户的反馈与支持
