# PiPER 机械臂接口文档

本文档说明 `piper_tomato` 当前对外 ROS 接口、任务层接口、相机/感知接口、Octomap 控制接口，以及采摘任务流中的 ACM 策略

适用范围：

```text
piper_tomato/src/app/piper_interface
piper_tomato/src/app/piper_task
piper_tomato/src/app/piper_gui
piper_tomato/src/service/piper_perception
piper_tomato/src/device/piper_camera
piper_tomato/src/platform/piper_msgs2
```

---

## 1. 使用前提

启动接口前应完成：

1. 编译 `piper_ros`
2. 编译 `piper_tomato`
3. 激活 CAN 设备
4. 确认 MoveIt 配置可启动
5. 如使用相机/点云避障，确认 Gemini335L 已连接
6. 如使用 GUI 采摘，确认 `/pick_action` 可用

启动：

```bash
source piper_ros/devel/setup.bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

或：

```bash
./piper-start.sh
```

---

## 2. 接口总览

### 2.1 Action

| 名称 | 类型 | 说明 |
|---|---|---|
| `/move_arm` | `piper_msgs2/MoveArmAction` | 完整机械臂控制接口 |
| `/simple_move_arm` | `piper_msgs2/SimpleMoveArmAction` | 简化机械臂控制接口 |
| `/pick_action` | `piper_msgs2/PickTaskAction` | 任务组/采摘任务接口 |

### 2.2 Service

| 名称 | 类型 | 说明 |
|---|---|---|
| `/arm_config` | `piper_msgs2/ConfigArm` | 约束配置 |
| `/arm_query` | `piper_msgs2/QueryArm` | 当前关节、当前位姿查询 |
| `/eef_cmd` | `piper_msgs2/CommandEef` | 末端执行器命令 |
| `/piper/perception/set_octomap_enabled` | `piper_msgs2/CommandOctomap` | MoveIt Octomap 点云输入开关与异步清图 |

### 2.3 Camera / Perception Topic

| 名称 | 类型 | 说明 |
|---|---|---|
| `/piper/camera/orbbec/color/image_raw` | `sensor_msgs/Image` | 彩色图 |
| `/piper/camera/orbbec/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/piper/camera/orbbec/depth/image_raw` | `sensor_msgs/Image` | 原始深度图，`32FC1`，单位 m |
| `/piper/camera/orbbec/depth/camera_info` | `sensor_msgs/CameraInfo` | 原始深度内参 |
| `/piper/camera/orbbec/depth_registered/image_raw` | `sensor_msgs/Image` | 对齐到彩色图的深度图，`32FC1`，单位 m |
| `/piper/camera/orbbec/depth_registered/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度内参 |
| `/piper/camera/orbbec/lrm_distance` | `std_msgs/Float32` | LRM 单点测距，单位 m |
| `/piper/perception/cloud/raw` | `sensor_msgs/PointCloud2` | 相机系原始点云 |
| `/piper/perception/cloud/base` | `sensor_msgs/PointCloud2` | 转到 `base_link` 的点云 |
| `/piper/perception/cloud/filtered` | `sensor_msgs/PointCloud2` | 工作空间裁剪和滤波后的点云 |

---

## 3. 启动参数

主要配置文件：

```text
piper_tomato/src/app/piper_interface/config/config.yaml
```

典型结构：

```yaml
start:
  arm_group_name: "arm"

  eef:
    enabled: true
    type: "servo_gripper"
    name: "gripper"
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200

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
```

说明：

- `eef.enabled=false` 时，末端执行器不会初始化
- `eef.type` 当前主要支持 `servo_gripper` 与 `two_finger_gripper`
- GUI 默认使用 `/pick_action`，而不是直接调用 `/move_arm`
- TCP 坐标系由 URDF 中的 `link_tcp` 定义

---

## 4. 机械臂命令编号

当前 `command_type` 使用 0-based 枚举值

### 4.1 ArmCmdType

| 编号 | 命令 | 说明 |
|:---|---|---|
| 0 | `HOME` | 回初始/安全位 |
| 1 | `MOVE_JOINTS` | 关节空间运动 |
| 2 | `MOVE_TARGET` | 移动到目标 |
| 3 | `MOVE_TARGET_IN_EEF_FRAME` | 末端坐标系相对目标 |
| 4 | `TELESCOPIC_END` | 末端伸缩 |
| 5 | `ROTATE_END` | 末端旋转 |
| 6 | `MOVE_LINE` | 直线运动 |
| 7 | `MOVE_BEZIER` | 贝塞尔轨迹 |
| 8 | `MOVE_DECARTES` | 多点笛卡尔轨迹 |
| 9 | `SET_ORIENTATION_CONSTRAINT` | 设置姿态约束 |
| 10 | `SET_POSITION_CONSTRAINT` | 设置位置约束 |
| 11 | `SET_JOINT_CONSTRAINT` | 设置关节约束 |
| 12 | `GET_CURRENT_JOINTS` | 查询当前关节 |
| 13 | `GET_CURRENT_POSE` | 查询当前位姿 |
| 14 | `MOVE_TO_ZERO` | 回零/安全位 |

### 4.2 EefCmdType

| 编号 | 命令 | 说明 |
|:---|---|---|
| 0 | `OPEN_GRIPPER` | 打开夹爪 |
| 1 | `CLOSE_GRIPPER` | 关闭夹爪 |
| 2 | `STOP_GRIPPER` | 停止末端动作 |

---

## 5. `/move_arm`

类型：

```text
piper_msgs2/MoveArmAction
```

主要字段：

```text
command_type
target_type
pose
point
quaternion
waypoints
joint_names
joints
values
```

`target_type`：

| 编号 | 名称 |
|:---|---|
| 0 | `TARGET_POSE` |
| 1 | `TARGET_POINT` |
| 2 | `TARGET_QUATERNION` |

执行约束：

- `MOVE_JOINTS`：`joints` 不能为空
- `MOVE_TARGET`：根据 `target_type` 读取 `pose` / `point` / `quaternion`
- `MOVE_TARGET_IN_EEF_FRAME`：目标会按末端坐标系相对变换处理
- `MOVE_LINE`：`waypoints.size() == 2`
- `MOVE_BEZIER`：`waypoints.size() == 3`
- `MOVE_DECARTES`：`waypoints.size() >= 1`
- `TELESCOPIC_END`、`ROTATE_END`：`values` 至少包含 1 个数值

返回字段：

```text
success
message
error_code
values
cur_pose
cur_joint
```

---

## 6. `/simple_move_arm`

类型：

```text
piper_msgs2/SimpleMoveArmAction
```

主要字段：

```text
command_type
target_type
x[]
y[]
z[]
roll[]
pitch[]
yaw[]
joint_names[]
joints[]
values[]
```

`target_type`：

| 编号 | 名称 |
|:---|---|
| 0 | `TARGET_POSE` |
| 1 | `TARGET_POINT` |
| 2 | `TARGET_ORIENTATION` |

语义：

- `TARGET_POSE`：要求 `x/y/z/roll/pitch/yaw` 长度一致
- `TARGET_POINT`：要求 `x/y/z` 长度一致
- `TARGET_ORIENTATION`：要求 `roll/pitch/yaw` 长度一致
- `MOVE_LINE`、`MOVE_BEZIER`、`MOVE_DECARTES` 的点数约束由分发层继续校验

---

## 7. `/arm_config`

类型：

```text
piper_msgs2/ConfigArm
```

Request：

```text
command_type
point
quaternion
joint_names[]
joints[]
values[]
```

Response：

```text
success
message
error_code
```

命令约束：

- `SET_ORIENTATION_CONSTRAINT(9)`：使用 `quaternion` 作为目标姿态
- `SET_POSITION_CONSTRAINT(10)`：使用 `point` 作为参考点，`values` 正好 3 个元素，对应 xyz 约束范围
- `SET_JOINT_CONSTRAINT(11)`：要求 `joint_names.size() == joints.size()`，且 `values.size() == joint_names.size() * 2`

---

## 8. `/arm_query`

类型：

```text
piper_msgs2/QueryArm
```

常用命令：

```bash
rosservice call /arm_query "command_type: 12
values: []"

rosservice call /arm_query "command_type: 13
values: []"
```

返回语义：

- 优先返回分发层提供的当前状态
- 若分发层没有填充当前状态，接口层回退读取控制器当前状态

---

## 9. `/eef_cmd`

类型：

```text
piper_msgs2/CommandEef
```

示例：

```bash
rosservice call /eef_cmd "command_type: 0
values: []"

rosservice call /eef_cmd "command_type: 1
values: []"

rosservice call /eef_cmd "command_type: 2
values: []"
```

说明：

- 末端未初始化时，返回“控制器未初始化”
- 末端类型不支持对应接口时，返回 `INVALID_INTERFACE`

---

## 10. `/pick_action`

类型：

```text
piper_msgs2/PickTaskAction
```

该接口由任务层提供，用于任务组配置、任务写入、任务执行和取消

### 10.1 主要请求类型

```text
UPDATE_TASK_GROUP_CONFIG
UPSERT_TASK
EXECUTE_TASK_GROUP
```

### 10.2 GUI 任务流

```text
绘制 ROI
  -> 计算目标像素
  -> depth_registered / LRM 获取目标深度
  -> 相机系目标点
  -> TF 转到 flange / tcp
  -> 通过 PickTaskAction 写入任务
  -> 执行任务组
```

### 10.3 PICK 阶段

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

### 10.4 `use_pre_pick`

`use_pre_pick=true` 时：

- 先规划到预采摘位
- 再笛卡尔直线接近最终采摘位
- 夹取后笛卡尔直线退出
- 这是启用阶段性 ACM 的推荐模式

`use_pre_pick=false` 时：

- `MOVE_TO_PRE_PICK` 阶段实际会直接规划到最终采摘位
- 不进入 `APPROACH_PICK` 与 `RETREAT_FROM_PICK`
- 不建议启用阶段性 ACM

---

## 11. 采摘阶段 ACM

当前最小模型：

```text
在 APPROACH_PICK、PICKING、RETREAT_FROM_PICK 阶段允许 link6 碰撞
离开这三个阶段后禁止 link6 碰撞
```

策略说明：

- 当前 ACM 是“允许/禁止”模型
- 只在 `use_pre_pick=true` 时启用
- `MOVE_TO_PRE_PICK` 正常避障
- `APPROACH_PICK` 前调用 `allow()`
- `RETREAT_FROM_PICK` 后调用 `disallow()`
- `CANCELED`、`FAILED`、`FINISH` 均应兜底调用 `disallow()`

限制：

- 当前只处理 `link6`
- 当前不是对象级 ACM
- 后续建议升级为 `pick_target_allow_region + gripper/link_tcp ACM`

---

## 12. `/piper/perception/set_octomap_enabled`

类型：

```text
piper_msgs2/CommandOctomap
```

语义：

```text
enabled: bool
clear_octomap: bool
---
success: bool
message: string
```

行为：

- `enabled=true`：允许发布 MoveIt 点云输入
- `enabled=false`：停止发布 MoveIt 点云输入
- `clear_octomap=true`：异步请求 MoveIt `/clear_octomap`
- 清图为异步行为，服务返回不代表 Octomap 已经完成清空

示例：

```bash
rosservice call /piper/perception/set_octomap_enabled "{enabled: false, clear_octomap: true}"
rosservice call /piper/perception/set_octomap_enabled "{enabled: true, clear_octomap: false}"
```

推荐使用策略：

```text
导航移动阶段：
  enabled=false
  必要时 clear_octomap=true

到达采摘点：
  clear_octomap=true
  等待当前局部点云进入
  enabled=true

采摘执行阶段：
  enabled=true

离开采摘点：
  enabled=false
  可选异步 clear
```

注意：

- 频繁调用 `/clear_octomap` 不是最佳方案
- 更推荐通过开关控制何时向 MoveIt 输入点云
- 后续应增加专用 `/piper/perception/cloud/moveit_input`

---

## 13. 相机与深度语义

### 13.1 `depth_registered`

`depth_registered` 是对齐到彩色图的深度图，推荐作为点云输入

```text
/piper/camera/orbbec/depth_registered/image_raw
/piper/camera/orbbec/depth_registered/camera_info
```

编码：

```text
32FC1
单位：m
```

### 13.2 LRM

LRM 发布：

```text
/piper/camera/orbbec/lrm_distance
```

类型：

```text
std_msgs/Float32
```

说明：

- LRM 是单点距离
- 不等价于整张深度图
- 当前主要用于 GUI 目标深度 fallback
- 不应直接用来填满整张点云

---

## 14. piper_perception 点云处理

当前流程：

```text
color + depth_registered + camera_info
  -> ApproximateTime 同步
  -> 反投影生成 raw cloud
  -> TF 转换到 base_link
  -> PassThrough 裁剪 x/y/z
  -> VoxelGrid 降采样
  -> SOR 离群点滤波
  -> 发布 filtered
```

主要参数示例：

```yaml
topics:
  color_image: /piper/camera/orbbec/color/image_raw
  depth_image: /piper/camera/orbbec/depth_registered/image_raw
  depth_info: /piper/camera/orbbec/depth_registered/camera_info

target_frame: base_link
pixel_stride: 4
frame_skip: 1

min_depth: 0.15
max_depth: 1.50

min_x: 0.0
max_x: 0.8
min_y: -0.5
max_y: 0.5
min_z: 0.0
max_z: 1.2

voxel_leaf: 0.015
sor_mean_k: 20
sor_stddev: 1.0
```

当前限制：

- 深度无效像素直接跳过，不会生成清理射线
- 因此相机看不到的历史 Octomap 点不会自动消失
- 后续应新增保守 invalid-depth clearing 或对象级 Collision Object 管线

---

## 15. 调用示例

### 15.1 末端坐标系相对移动

```python
import rospy
import actionlib
from piper_msgs2.msg import MoveArmAction, MoveArmGoal

rospy.init_node("move_arm_demo")
client = actionlib.SimpleActionClient("/move_arm", MoveArmAction)
client.wait_for_server()

goal = MoveArmGoal()
goal.command_type = MoveArmGoal.MOVE_TARGET_IN_EEF_FRAME
goal.target_type = MoveArmGoal.TARGET_POSE
goal.pose.position.x = 0.03
goal.pose.position.y = 0.0
goal.pose.position.z = 0.0
goal.pose.orientation.w = 1.0

client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
```

### 15.2 简化直线运动

```python
import rospy
import actionlib
from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal

rospy.init_node("simple_move_line_demo")
client = actionlib.SimpleActionClient("/simple_move_arm", SimpleMoveArmAction)
client.wait_for_server()

goal = SimpleMoveArmGoal()
goal.command_type = SimpleMoveArmGoal.MOVE_LINE
goal.target_type = SimpleMoveArmGoal.TARGET_POSE
goal.x = [0.30, 0.35]
goal.y = [0.00, 0.05]
goal.z = [0.35, 0.35]
goal.roll = [0.0, 0.0]
goal.pitch = [0.0, 0.0]
goal.yaw = [0.0, 0.0]

client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
```

### 15.3 启动 GUI

```bash
source piper_tomato/devel/setup.bash
rosrun piper_gui piper_gui.py
```

### 15.4 控制 Octomap

```bash
rosservice call /piper/perception/set_octomap_enabled "{enabled: false, clear_octomap: true}"
rosservice call /piper/perception/set_octomap_enabled "{enabled: true, clear_octomap: false}"
```

---

## 16. 错误码与排查

常见错误码：

| 错误码 | 说明 |
|---|---|
| `SUCCESS` | 成功 |
| `INVALID_PARAMETER` | 参数错误 |
| `INVALID_TARGET_TYPE` | 目标类型错误 |
| `INVALID_INTERFACE` | 接口不支持 |
| `TF_TRANSFORM_FAILED` | TF 变换失败 |
| `PLANNING_FAILED` | MoveIt 规划失败 |
| `EXECUTION_FAILED` | 执行失败 |
| `DESCARTES_PLANNING_FAILED` | 笛卡尔路径失败 |
| `EMPTY_WAYPOINTS` | 路径点为空 |
| `ASYNC_TASK_RUNNING` | 异步任务正在执行 |
| `CANCELLED` | 任务已取消 |

排查顺序：

1. `/arm_query` 是否能查询当前状态
2. TF 是否完整
3. 目标是否超出工作空间
4. MoveIt 是否已加载 robot model 和 planning scene
5. 相机点云是否发布
6. Octomap 输入是否启用
7. ACM 是否只在预期阶段启用
8. 取消任务后是否已恢复碰撞策略
