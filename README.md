# PiPER-ROS: PiPER 机械臂 ROS 控制系统

<div align="center">

基于 **ROS Noetic + MoveIt + PiPER 机械臂 + Gemini335L 腕上相机** 的番茄采摘实验平台

本项目围绕“GUI 目标选择 → 任务流调度 → MoveIt 运动规划 → 末端执行器控制 → 腕上相机环境点云 → 采摘阶段碰撞策略”构建完整闭环，用于验证农业采摘机械臂的软件控制链、感知接入与任务级执行逻辑

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Framework: MoveIt](https://img.shields.io/badge/Framework-MoveIt-green.svg)](https://moveit.ros.org/)
[![Camera: Gemini335L](https://img.shields.io/badge/Camera-Orbbec%20Gemini335L-orange.svg)](https://www.orbbec.com/)
[![Arm: PiPER](https://img.shields.io/badge/Arm-PiPER-red.svg)](https://global.agilex.ai/)

</div>

---

## 1. 系统能力概览

- PiPER 机械臂 ROS 控制接入
- MoveIt 规划、执行与当前状态查询
- `/move_arm`、`/simple_move_arm`、`/pick_action`、`/arm_config`、`/arm_query`、`/eef_cmd` 接口
- 图形化 ROI 选点与采摘任务写入
- Gemini335L 腕上相机接入
- Depth → Color 对齐深度图发布
- LRM 单点距离发布，用于近距离目标深度 fallback
- 相机到法兰的手眼 TF 发布
- 点云生成、TF 变换、工作空间裁剪、VoxelGrid 降采样、SOR 离群点滤波
- MoveIt Octomap 点云输入开关与异步清图
- 采摘任务阶段机
- 接近/采摘/退出阶段 `link6` 临时 ACM 放行
- 取消任务、停止执行、回安全位流程

---

## 2. 推荐使用场景

当前版本适合：

- 实验室固定底座场景下的机械臂采摘实验
- 番茄目标点人工 ROI 选取
- 点云避障与采摘路径调试
- MoveIt 规划参数、笛卡尔路径参数调试
- 腕上相机手眼标定和深度目标估计验证
- 采摘动作任务流验证

当前版本不建议直接用于：

- 底盘移动过程中持续维护全局 Octomap
- 未经人工检查的长时间自主采摘
- 未做对象分割的复杂枝叶环境精细避障
- 将 LRM 单点距离当作全局深度补盲
- 允许整条机械臂或整个 Octomap 无条件碰撞

---

## 3. 硬件与软件栈

| 类别 | 当前配置 |
|---|---|
| 机械臂 | PiPER 6-DOF |
| 末端执行器 | 舵机夹爪 / 双指夹爪接口 |
| 相机 | Orbbec Gemini335L |
| 主机 | Ubuntu 20.04 / 22.04 |
| 通信 | USB-CAN，默认 `can0`，bitrate=1000000 |
| ROS | ROS Noetic |
| 规划 | MoveIt 1 |
| 点云处理 | PCL, pcl_ros, tf2_sensor_msgs |
| GUI | Python, PyQt5, OpenCV |
| 相机 SDK | pyorbbecsdk |

---

## 4. 项目结构

```text
piper-ws/
├── README.md
├── can-activate.sh
├── piper-start.sh
├── piper_test.py
├── hand-eye/                         # 手眼采集与标定工具
├── picture/                          # 手眼数据采集目录
├── outputs/                          # 手眼标定输出目录
├── piper_ros/                        # PiPER 官方 ROS 工作区
├── piper_tomato/                     # 本项目主工作区
│   ├── PiPER 机械臂接口文档.md
│   └── src/
│       ├── platform/
│       │   └── piper_msgs2/          # 自定义 Action/Service/Msg
│       ├── device/
│       │   └── piper_camera/         # Gemini335L 相机节点
│       ├── domain/
│       │   └── piper_controller/     # MoveIt 控制与运动规划封装
│       ├── service/
│       │   ├── piper_commander/      # 命令分发
│       │   └── piper_perception/     # 点云生成与 Octomap 输入控制
│       └── app/
│           ├── piper_interface/      # ROS Action/Service 接口层
│           ├── piper_task/           # 任务管理与采摘状态机
│           └── piper_gui/            # ROI 选点与任务下发 GUI
├── piper_sdk/                        # Python SDK
└── ros_env/                          # Ubuntu 22.04 Noetic 环境辅助脚本
```

---

## 5. 快速开始

### 5.1 环境准备

Ubuntu 20.04 原生 ROS Noetic：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-dev curl \
  can-utils ethtool \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-controller-manager \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-joint-state-controller \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-moveit-ros-perception

pip3 install piper_sdk
sudo usermod -aG dialout $USER
```

Ubuntu 22.04 使用 `ros_env` / micromamba 请参考 `ros_env/README.md`

### 5.2 编译

```bash
cd /path/to/piper-ws

cd piper_ros
catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash

cd ../piper_tomato
catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash
```

### 5.3 激活 CAN

```bash
source can-activate.sh
```

或手动：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip link show can0
```

### 5.4 启动系统（piper-start.sh 包含 can 激活、机械臂模型、感知接口、控制接口等全链路启动）

```bash
./piper-start.sh
```

可选参数：

```bash
./piper-start.sh --disable
./piper-start.sh --delay 5
./piper-start.sh --disable --delay 8
```

手动启动：

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

### 5.5 启动 GUI

```bash
source piper_tomato/devel/setup.bash
rosrun piper_gui piper_gui.py
```

GUI 典型流程：

1. 启动 `piper_start.launch`
2. 启动 `piper_gui.py`
3. 在图像上左键绘制 ROI，双击闭合
4. 检查目标坐标、深度来源与 TCP 坐标
5. 点击“写入 / 更新当前任务”
6. 点击“执行当前任务组”

---

## 6. 相机与感知链路

### 6.1 piper_camera

主要话题：

| Topic | Type | 说明 |
|---|---|---|
| `/piper/camera/orbbec/color/image_raw` | `sensor_msgs/Image` | 彩色图 |
| `/piper/camera/orbbec/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/piper/camera/orbbec/depth/image_raw` | `sensor_msgs/Image` | 原始深度，`32FC1`，单位 m |
| `/piper/camera/orbbec/depth/camera_info` | `sensor_msgs/CameraInfo` | 原始深度内参 |
| `/piper/camera/orbbec/depth_registered/image_raw` | `sensor_msgs/Image` | 对齐到彩色图的深度，`32FC1`，单位 m |
| `/piper/camera/orbbec/depth_registered/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度内参 |
| `/piper/camera/orbbec/lrm_distance` | `std_msgs/Float32` | LRM 单点测距，单位 m |

说明：

- `depth_registered` 是 `piper_perception` 的推荐输入
- LRM 是单点距离，不等价于整张深度图；当前更适合用于目标点深度 fallback
- 手眼 TF 由 `~hand_eye/T_cam_to_flange` 参数发布，默认父坐标系为 `link6`，子坐标系为 `eef_camera_color_optical_frame`

### 6.2 piper_perception

主要话题：

| Topic | Type | Frame | 说明 |
|---|---|---|---|
| `/piper/perception/cloud/raw` | `sensor_msgs/PointCloud2` | 相机 frame | 相机系原始点云 |
| `/piper/perception/cloud/base` | `sensor_msgs/PointCloud2` | `base_link` | 已转换到目标参考系 |
| `/piper/perception/cloud/filtered` | `sensor_msgs/PointCloud2` | `base_link` | 裁剪、降采样、滤波后的点云；当前作为 MoveIt Octomap 输入 |

处理流程：

```text
depth_registered + camera_info
  -> 反投影生成相机系点云
  -> TF 转换到 base_link
  -> PassThrough 工作空间裁剪
  -> VoxelGrid 降采样
  -> SOR 离群点滤波
  -> 发布 filtered
```

Octomap 控制服务：

```text
/piper/perception/set_octomap_enabled
```

类型：

```text
piper_msgs2/CommandOctomap
```

示例：

```bash
rosservice call /piper/perception/set_octomap_enabled "{enabled: false, clear_octomap: true}"
rosservice call /piper/perception/set_octomap_enabled "{enabled: true, clear_octomap: false}"
```

当前注意事项：

- Octomap 适合作为未知障碍兜底，不适合作为长期全局地图
- 腕上相机看不到的区域不会自动证明为空，因此历史体素可能残留
- 后续建议增加 `/piper/perception/cloud/moveit_input` 专用 topic，并加入保守的 invalid-depth clearing ray 或转向 Collision Object 管线

---

## 7. 采摘任务流与 ACM

当前 PICK 任务阶段：

```cpp
#define PICK_STAGE_TABLE \
    X(IDLE, "空闲") \
    X(START, "开始") \
    X(MOVE_TO_PRE_PICK, "移动到预采摘位") \
    X(APPROACH_PICK, "直线接近采摘位") \
    X(PICKING, "采摘中") \
    X(RETREAT_FROM_PICK, "直线退出采摘位") \
    X(MOVE_TO_PLACE, "移动到放置位") \
    X(PLACING, "放置中") \
    X(FINISH, "完成") \
    X(GO_HOME, "回到初始位") \
    X(CANCELED, "已取消") \
    X(FAILED, "失败")
```

当前 ACM 最小模型：

```text
仅在 APPROACH_PICK + PICKING + RETREAT_FROM_PICK 阶段允许 link6 碰撞
```

约束：

- 仅当 `use_pre_pick=true` 时启用 ACM
- `MOVE_TO_PRE_PICK` 阶段仍保持正常避障
- `RETREAT_FROM_PICK` 完成后立即禁止 `link6` 碰撞
- 取消、失败、任务结束时必须兜底禁止
- 当前是“允许/禁止”强语义，不是对象级语义放行

---

## 8. ROS 接口总览

### Action

| 名称 | 类型 | 说明 |
|---|---|---|
| `/move_arm` | `piper_msgs2/MoveArmAction` | 完整机械臂命令接口 |
| `/simple_move_arm` | `piper_msgs2/SimpleMoveArmAction` | 简化机械臂命令接口 |
| `/pick_action` | `piper_msgs2/PickTaskAction` | 任务组与采摘任务接口 |

### Service

| 名称 | 类型 | 说明 |
|---|---|---|
| `/arm_config` | `piper_msgs2/ConfigArm` | 约束配置 |
| `/arm_query` | `piper_msgs2/QueryArm` | 当前关节/位姿查询 |
| `/eef_cmd` | `piper_msgs2/CommandEef` | 末端执行器命令 |
| `/piper/perception/set_octomap_enabled` | `piper_msgs2/CommandOctomap` | Octomap 点云输入开关与异步清图 |

详细字段、命令编号与示例见：

```text
piper_tomato/PiPER 机械臂接口文档.md
```

---

## 9. 常用验证命令

### 查询机械臂状态

```bash
rosservice call /arm_query "command_type: 12
values: []"

rosservice call /arm_query "command_type: 13
values: []"
```

### 检查相机话题

```bash
rostopic list | grep /piper/camera/orbbec
rostopic hz /piper/camera/orbbec/depth_registered/image_raw
rostopic echo -n 1 /piper/camera/orbbec/lrm_distance
```

### 检查点云

```bash
rostopic hz /piper/perception/cloud/raw
rostopic hz /piper/perception/cloud/base
rostopic hz /piper/perception/cloud/filtered
```

### 检查相机 TF

```bash
rosrun tf tf_echo link6 eef_camera_color_optical_frame
rosrun tf tf_echo base_link eef_camera_color_optical_frame
```

### 控制 Octomap

```bash
rosservice call /piper/perception/set_octomap_enabled "{enabled: false, clear_octomap: true}"
rosservice call /piper/perception/set_octomap_enabled "{enabled: true, clear_octomap: false}"
```

---

## 10. 当前主要限制

### 点云与 Octomap

- 无效深度像素当前不会产生清理射线
- 相机视野外的历史点云不会自动消失
- Octomap 清图是整图操作，较大地图下耗时明显
- 腕上移动相机不适合长时间维护全局 Octomap

### LRM

- LRM 是单点距离，不是稠密深度图
- 适合目标点深度 fallback，不适合补完整环境点云

### ACM

- 当前 ACM 是最小模型，只对 `link6` 做阶段性允许/禁止
- 尚未实现对象级 ACM
- 尚未将目标区域从 Octomap 输入中剔除

### 语义感知

- 当前没有果实、叶片、枝条、支架的语义区分
- 后续应从 ROI 几何剔除、点云聚类、Collision Object 生命周期管理开始

---

## 11. 故障排查

### CAN 未启动

```bash
ip link show can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 相机无 TF

```bash
rosrun tf tf_echo link6 eef_camera_color_optical_frame
```

检查：

- `~hand_eye/T_cam_to_flange`
- `~hand_eye/flange_id`
- `color_frame_id`

### 点云没有发布

```bash
rostopic echo -n 1 /piper/camera/orbbec/depth_registered/image_raw/header
rostopic hz /piper/perception/cloud/raw
```

检查：

- `depth_registered` 是否发布
- `color` 与 `depth_registered` 尺寸是否一致
- 深度图编码是否为 `32FC1`
- TF 是否可从相机 frame 转到 `base_link`

### Octomap 历史点不消失

这是占据图机制限制，不是单纯 Bug优先处理：

- 缩小 `max_range`
- 缩小 perception 输入范围
- 采摘前 clear
- 非采摘阶段关闭输入
- 后续加入 invalid-depth clearing ray 或 Collision Object 管线

### ACM 未生效

检查：

- `use_pre_pick=true`
- 是否进入 `APPROACH_PICK`
- `allow()` 是否成功返回
- `RETREAT_FROM_PICK` 后是否成功 `disallow()`
- 当前碰撞 link 是否确实是 `link6`

---

## 12. 文档

- [PiPER 机械臂接口文档](piper_tomato/PiPER%20机械臂接口文档.md)
- [piper_interface 配置](piper_tomato/src/app/piper_interface/config/config.yaml)
- [hand-eye 手眼标定工具](hand-eye/README.md)
- [PiPER ROS 官方包](piper_ros/README.MD)
- [PiPER SDK](piper_sdk/README.MD)

---

## 13. 致谢

首先感谢松灵官方（AgileX Robotics）提供 PiPER 机械臂生态与开源基础：

- PiPER ROS: https://github.com/agilexrobotics/piper_ros
- PiPER SDK: https://github.com/agilexrobotics/piper_sdk

同时感谢以下优秀开源项目：

- trac-ik: https://github.com/HIRO-group/trac_ik
- serial: https://github.com/wjwwood/serial
- tl-optional: https://github.com/TartanLlama/optional
- tl-expected: https://github.com/TartanLlama/expected

---

## 14. 许可证

MIT License，详见 `LICENSE`

---

## 15. 贡献

欢迎提交 Issue / PR