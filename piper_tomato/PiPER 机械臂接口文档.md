
# PiPER 机械臂接口文档

`piper_tomato/src/app/piper_interface` 当前对外控制接口说明（Action / Service）。

---

## 1. 使用前提

- 已完成 `piper_ros` 与 `piper_tomato` 编译。
- 已激活 CAN 设备（推荐 `can0`，`bitrate=1000000`）。
- 已加载并 source 当前工作空间：

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

接口节点由 `piper_start` 启动，参数均从私有命名空间 `~start/...` 读取，对应文件为 [piper_tomato/src/app/piper_interface/config/config.yaml](piper_tomato/src/app/piper_interface/config/config.yaml)。

默认接口名如下：

- `move_arm`
- `simple_move_arm`
- `arm_config`
- `arm_query`
- `pick_action`
- `eef_cmd`

当前实现约定：

- 接口层转换函数统一采用 `tl::optional` 语义：转换失败返回 `nullopt`，并中止请求处理。
- `ArmCmdResult.current_pose` 与 `ArmCmdResult.current_joints` 为 optional 字段，仅在需要时填充。
- ROS 接口层会在必要时做兜底填充，保证对外消息字段始终可读。
- 机械臂与末端执行器分开发送：`ArmCmdDispatcher` 负责机械臂命令，`EefCmdDispatcher` 负责末端执行器命令。
- 内部核心类型采用无后缀命名，例如 `SearchReachablePose`、`ReachablePoseResult`、`AStarNode`。

代码层设计约定：

- 类型名使用 PascalCase，无 `_t/_e` 后缀。
- 除 `target` 外，优先使用 `tl::optional`。
- `target` 保持 `variant + monostate`，用于多目标类型分派。
- 错误语义优先透传真实错误码，不在中间层无差别折叠。

---

## 2. 启动参数

`piper_start.cpp` 当前读取的参数如下：

```yaml
start:
  arm_group_name: "arm"
  eef:
    enabled: true
    type: "servo_gripper"   # two_finger_gripper | servo_gripper
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

- `eef.enabled=false` 时，末端执行器对象不会初始化；`/eef_cmd` 即使被创建，调用也会返回“控制器未初始化”。
- `eef.type` 当前支持 `two_finger_gripper` 与 `servo_gripper`。
- `eef_cmd_service.name` 默认是 `eef_cmd`，不是旧文档中的 `eef_command`。

---

## 3. 接口总览

### 3.1 Action

- `/move_arm`：`piper_msgs2/MoveArmAction`
- `/simple_move_arm`：`piper_msgs2/SimpleMoveArmAction`
- `/pick_action`：`piper_msgs2/PickTaskAction`（由 `piper_task` 模块提供）

### 3.2 Service

- `/arm_config`：`piper_msgs2/ConfigArm`
- `/arm_query`：`piper_msgs2/QueryArm`
- `/eef_cmd`：`piper_msgs2/CommandEef`

### 3.3 GUI（任务层）

- 可执行脚本：`piper_gui.py`（包：`piper_gui`）
- 默认动作接口：`/pick_action`（`piper_msgs2/PickTaskAction`）
- GUI 发送的核心请求类型：
  - `UPDATE_TASK_GROUP_CONFIG`
  - `UPSERT_TASK`
  - `EXECUTE_TASK_GROUP`

说明：GUI 不直接调用 `/move_arm` / `/simple_move_arm`，而是通过任务层 Action 统一管理“写入任务 / 执行任务组 / 取消执行”。

---

## 4. 命令编号（command_type）

当前代码使用的是 **0-based** 枚举值，不是从 1 开始。

### 4.1 机械臂命令（`ArmCmdType`）

- `0` HOME
- `1` MOVE_JOINTS
- `2` MOVE_TARGET
- `3` MOVE_TARGET_IN_EEF_FRAME
- `4` TELESCOPIC_END
- `5` ROTATE_END
- `6` MOVE_LINE
- `7` MOVE_BEZIER
- `8` MOVE_DECARTES
- `9` SET_ORIENTATION_CONSTRAINT
- `10` SET_POSITION_CONSTRAINT
- `11` SET_JOINT_CONSTRAINT
- `12` GET_CURRENT_JOINTS
- `13` GET_CURRENT_POSE
- `14` MOVE_TO_ZERO

### 4.2 末端命令（`EefCmdType`）

- `0` OPEN_GRIPPER
- `1` CLOSE_GRIPPER
- `2` STOP_GRIPPER

说明：

- `command_type` 超出范围会返回 `INVALID_PARAMETER`。
- 轨迹类命令会额外校验路径点数量和数据完整性。

---

## 5. Action 消息说明

### 5.1 `MoveArmAction`

`goal` 主要字段：

- `command_type`
- `target_type`
- `pose` / `point` / `quaternion`
- `waypoints`
- `joint_names`
- `joints`
- `values`

`target_type`：

- `0` `TARGET_POSE`
- `1` `TARGET_POINT`
- `2` `TARGET_QUATERNION`

执行语义：

- `MOVE_JOINTS`：`joints` 不能为空。
- `MOVE_TARGET` / `MOVE_TARGET_IN_EEF_FRAME`：`target` 不能为空。
- `TELESCOPIC_END`：`values` 需要 1 个元素。
- `ROTATE_END`：`values` 需要 1 个元素。
- `MOVE_LINE`：`waypoints.size() == 2`。
- `MOVE_BEZIER`：`waypoints.size() == 3`。
- `MOVE_DECARTES`：`waypoints.size() >= 1`。

返回字段：

- `success`
- `message`
- `error_code`
- `values`
- `cur_pose`
- `cur_joint`

反馈字段：

- `stage`
- `progress`
- `message`

### 5.2 `SimpleMoveArmAction`

`goal` 主要字段：

- `command_type`
- `target_type`
- `x[] / y[] / z[]`
- `roll[] / pitch[] / yaw[]`
- `joint_names`
- `joints`
- `values`

`target_type`：

- `0` `TARGET_POSE`
- `1` `TARGET_POINT`
- `2` `TARGET_ORIENTATION`

当前实现行为：

- `TARGET_POSE`：要求 `x/y/z` 与 `roll/pitch/yaw` 长度一致，且至少 1 个点。
- `TARGET_POINT`：要求 `x/y/z` 长度一致，且至少 1 个点。
- `TARGET_ORIENTATION`：要求 `roll/pitch/yaw` 长度一致，且至少 1 个点。
- 目标会先转换成 `req.target`，路径点会整体写入 `req.waypoints`。
- `MOVE_LINE` / `MOVE_BEZIER` / `MOVE_DECARTES` 的具体校验仍由分发层完成。

返回字段：

- `success`
- `message`
- `error_code`
- `values`
- `cur_joint`
- `cur_x`
- `cur_y`
- `cur_z`
- `cur_roll`
- `cur_pitch`
- `cur_yaw`

反馈字段：

- `stage`
- `progress`
- `message`

---

## 6. Service 消息说明

### 6.1 `/arm_config`（`piper_msgs2/ConfigArm`）

Request：

- `command_type`
- `point`
- `quaternion`
- `joint_names[]`
- `joints[]`
- `values[]`

Response：

- `success`
- `message`
- `error_code`

命令约束：

- `SET_ORIENTATION_CONSTRAINT(9)`：使用 `quaternion`，目标必须是 Quaternion。
- `SET_POSITION_CONSTRAINT(10)`：使用 `point`，`values` 必须正好 3 个元素（x/y/z 约束范围）。
- `SET_JOINT_CONSTRAINT(11)`：
  - `joint_names.size() == joints.size()`
  - `values.size() == joint_names.size() * 2`
  - 每个关节对应两个容差参数（下界、上界）。

### 6.2 `/arm_query`（`piper_msgs2/QueryArm`）

Request：

- `command_type`
- `values[]`

Response：

- `success`
- `message`
- `error_code`
- `cur_pose`
- `cur_joint[]`

命令约束：

- `GET_CURRENT_JOINTS(12)`：返回 `cur_joint[]`。
- `GET_CURRENT_POSE(13)`：返回 `cur_pose`。

返回语义：

- 优先返回分发层提供的当前状态。
- 若分发层没有填充 `current_pose/current_joints`，接口层会回退读取控制器当前状态。

### 6.3 `/eef_cmd`（`piper_msgs2/CommandEef`）

Request：

- `command_type`
- `values[]`

Response：

- `success`
- `message`
- `error_code`

命令约束：

- `OPEN_GRIPPER(0)`：需要末端支持 `JointEefInterface::open()`。
- `CLOSE_GRIPPER(1)`：需要末端支持 `JointEefInterface::close()`。
- `STOP_GRIPPER(2)`：直接调用 `EndEffector::stop()`。

说明：

- 如果末端执行器未初始化，服务返回“控制器未初始化”。
- 如果末端不支持关节式夹爪接口，`OPEN_GRIPPER` / `CLOSE_GRIPPER` 会返回 `INVALID_INTERFACE`。

---

## 7. 调用示例

### 7.1 查询当前关节

```bash
rosservice call /arm_query "command_type: 12
values: []"
```

### 7.2 查询当前位姿

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

### 7.3 设置姿态约束

```bash
rosservice call /arm_config "command_type: 9
point: {x: 0.0, y: 0.0, z: 0.0}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: []"
```

### 7.4 设置末端位置约束

```bash
rosservice call /arm_config "command_type: 10
point: {x: 0.30, y: 0.00, z: 0.35}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: [0.03, 0.03, 0.03]"
```

### 7.5 发送末端执行器命令

打开夹爪：

```bash
rosservice call /eef_cmd "command_type: 0
values: []"
```

关闭夹爪：

```bash
rosservice call /eef_cmd "command_type: 1
values: []"
```

停止夹爪：

```bash
rosservice call /eef_cmd "command_type: 2
values: []"
```

### 7.6 `move_arm`：末端坐标系相对移动

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

### 7.7 `simple_move_arm`：MOVE_LINE

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

### 7.8 `piper_gui`：图形化下发采摘任务

```bash
source piper_tomato/devel/setup.bash
rosrun piper_gui piper_gui.py
```

推荐启动顺序：

1. 启动 `roslaunch piper_interface piper_start.launch`（确保 `/pick_action` 可用）。
2. 启动 GUI。
3. 在图像上左键绘制 ROI，双击闭合区域。
4. 点击“写入 / 更新当前任务”或“执行当前任务组”。

当前 GUI 关键行为：

- 将 ROI 计算得到的目标点写为 `Point(link_tcp)`。
- 任务组默认名为 `gui_pick`，可在界面修改。
- 支持“仅更新任务组配置”“执行当前任务组”“取消当前执行”。

---

## 8. 错误码与排障

常见 `error_code`：

- `SUCCESS`：执行成功
- `INVALID_PARAMETER`：参数数量或类型错误
- `INVALID_TARGET_TYPE`：目标类型与字段不匹配
- `INVALID_INTERFACE`：末端执行器不支持对应接口
- `TF_TRANSFORM_FAILED`：坐标变换失败
- `PLANNING_FAILED` / `EXECUTION_FAILED`：MoveIt 规划或执行失败
- `DESCARTES_PLANNING_FAILED` / `EMPTY_WAYPOINTS`：笛卡尔路径失败
- `ASYNC_TASK_RUNNING` / `CANCELLED`：异步任务冲突或已取消

接口层返回策略：

- `MoveArmAction` / `SimpleMoveArmAction`：如果分发层没有填充当前状态，会回退读取控制器当前值后再填入结果。
- `QueryArm`：同样会回退读取控制器当前状态。
- `CommandEef`：只返回成功状态，不携带当前位姿或关节信息。

建议排查流程：

1. 先调用 `/arm_query` 确认当前状态可读。
2. 再检查命令对应参数数量是否满足约束。
3. 若是坐标相关失败，检查 TF 树、EEF 类型和 TCP 偏移配置。
4. 若是规划失败，先收紧目标范围并降低速度参数再尝试。
