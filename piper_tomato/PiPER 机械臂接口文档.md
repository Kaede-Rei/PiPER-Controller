<div align="center">

# PiPER 机械臂接口文档

`piper_tomato/src/piper_interface` 对外控制接口说明（Action / Service）

</div>

---

## 📌 1. 使用前提

- 已完成 `piper_ros` 与 `piper_tomato` 编译。
- 已激活 CAN 设备（推荐 `can0`，`bitrate=1000000`）。
- 已启动接口节点：

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_interface piper_start.launch
```

默认接口名由 `piper_tomato/src/piper_interface/config/config.yaml` 配置：

- `move_arm`
- `simple_move_arm`
- `arm_config`
- `arm_query`

实现约定（关键）：

- 接口层转换函数统一采用 `tl::optional` 返回语义：转换失败返回 `nullopt`，并中止请求处理。
- `ArmCmdResult.current_pose` 与 `ArmCmdResult.current_joints` 在命令分发层为 optional 字段，仅在需要时填充。
- ROS 响应阶段会做兜底填充，保证对外消息字段始终可读。
- 内部核心类型采用无后缀命名，例如：`SearchReachablePose`、`ReachablePoseResult`、`AStarNode`。

代码层设计约定：

- 命名：类型名使用 PascalCase，无 `_t/_e` 后缀。
- 可空语义：除 `target` 以外优先使用 `tl::optional`。
- 目标语义：`target` 保持 `variant + monostate`，用于多目标类型分派。
- 错误语义：优先透传真实错误码，不在中间层无差别折叠。

---

## 🔌 2. 接口总览

### 2.1 Action

- `/move_arm`：`piper_msgs2/MoveArmAction`
- `/simple_move_arm`：`piper_msgs2/SimpleMoveArmAction`

### 2.2 Service

- `/arm_config`：`piper_msgs2/ConfigArm`
- `/arm_query`：`piper_msgs2/QueryArm`

---

## 🧭 3. 命令编号（command_type）

### 3.1 运动命令（Action）

- `1` HOME
- `2` MOVE_JOINTS
- `3` MOVE_TARGET
- `4` MOVE_TARGET_IN_EEF_FRAME
- `5` TELESCOPIC_END
- `6` ROTATE_END
- `7` MOVE_LINE
- `8` MOVE_BEZIER
- `9` MOVE_DECARTES
- `15` MOVE_TO_ZERO

### 3.2 配置命令（/arm_config）

- `10` SET_ORIENTATION_CONSTRAINT
- `11` SET_POSITION_CONSTRAINT
- `12` SET_JOINT_CONSTRAINT

### 3.3 查询命令（/arm_query）

- `13` GET_CURRENT_JOINTS
- `14` GET_CURRENT_POSE

说明：

- `command_type` 必须在有效区间内，否则会返回失败。
- 轨迹类命令会校验路径点数量和数据完整性。

---

## 🧱 4. Action 消息说明

### 4.1 MoveArmAction

`target_type`：

- `0` TARGET_POSE
- `1` TARGET_POINT
- `2` TARGET_QUATERNION

关键字段：

- `command_type`
- `target_type`
- `pose / point / quaternion`
- `waypoints`（line / bezier / decartes 使用）
- `joint_names`、`joints`
- `values`

参数约束：

- `MOVE_JOINTS`：`joints` 不能为空。
- `MOVE_TARGET`、`MOVE_TARGET_IN_EEF_FRAME`：目标类型与字段必须匹配。
- `TELESCOPIC_END`：`values` 需 1 个元素（伸缩量）。
- `ROTATE_END`：`values` 需 1 个元素（旋转角）。
- `MOVE_LINE`：`waypoints.size() == 2`。
- `MOVE_BEZIER`：`waypoints.size() == 3`。
- `MOVE_DECARTES`：`waypoints.size() >= 1`。

### 4.2 SimpleMoveArmAction

`target_type`：

- `0` TARGET_POSE
- `1` TARGET_POINT
- `2` TARGET_ORIENTATION

数组字段：

- `x[] y[] z[]`
- `roll[] pitch[] yaw[]`

参数约束：

- `TARGET_POSE`：`x/y/z` 与 `roll/pitch/yaw` 各自长度一致，且至少 1 个点。
- `TARGET_POINT`：`x/y/z` 长度一致，且至少 1 个点。
- `TARGET_ORIENTATION`：`roll/pitch/yaw` 长度一致，且至少 1 个点。
- `MOVE_LINE`：需要 2 个点。
- `MOVE_BEZIER`：需要 3 个点。
- `MOVE_DECARTES`：需要至少 1 个点。

返回说明：

- `SimpleMoveArmResult.cur_x/cur_y/cur_z/cur_roll/cur_pitch/cur_yaw` 为当前末端状态。

---

## ⚙️ 5. Service 消息说明

### 5.1 /arm_config（piper_msgs2/ConfigArm）

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

- `SET_ORIENTATION_CONSTRAINT(10)`：使用 `quaternion`。
- `SET_POSITION_CONSTRAINT(11)`：使用 `point`，`values` 必须有 3 个元素（x/y/z 约束范围）。
- `SET_JOINT_CONSTRAINT(12)`：
  - `joint_names.size() == joints.size()`
  - `values.size() == joint_names.size() * 2`
  - 每个关节对应两个容差参数。

### 5.2 /arm_query（piper_msgs2/QueryArm）

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

- `GET_CURRENT_JOINTS(13)`：返回 `cur_joint[]`。
- `GET_CURRENT_POSE(14)`：返回 `cur_pose`。

返回语义说明：

- 查询命令会优先返回分发层的状态值。
- 若分发层状态为空（optional 未赋值），接口层会回退读取控制器当前状态填入响应。

---

## 🧪 6. 调用示例

### 6.1 查询当前关节

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

### 6.2 查询当前位姿

```bash
rosservice call /arm_query "command_type: 14
values: []"
```

### 6.3 设置姿态约束

```bash
rosservice call /arm_config "command_type: 10
point: {x: 0.0, y: 0.0, z: 0.0}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: []"
```

### 6.4 simple_move_arm：在末端坐标系移动

```python
import rospy
import actionlib
from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal

rospy.init_node("simple_move_demo")
client = actionlib.SimpleActionClient("/simple_move_arm", SimpleMoveArmAction)
client.wait_for_server()

goal = SimpleMoveArmGoal()
goal.command_type = SimpleMoveArmGoal.MOVE_TARGET_IN_EEF_FRAME
goal.target_type = SimpleMoveArmGoal.TARGET_POSE
goal.x = [0.03]
goal.y = [0.0]
goal.z = [0.0]
goal.roll = [0.0]
goal.pitch = [0.0]
goal.yaw = [0.0]
goal.joint_names = []
goal.joints = []
goal.values = []

client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
```

### 6.5 simple_move_arm：MOVE_LINE

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

---

## 🐞 7. 错误码与排障

常见 `error_code`：

- `SUCCESS`：执行成功
- `INVALID_PARAMETER`：参数数量或类型错误
- `INVALID_TARGET_TYPE`：目标类型与字段不匹配
- `TF_TRANSFORM_FAILED`：坐标变换失败
- `PLANNING_FAILED` / `EXECUTION_FAILED`：MoveIt 规划/执行失败
- `DESCARTES_PLANNING_FAILED` / `EMPTY_WAYPOINTS`：笛卡尔路径失败
- `ASYNC_TASK_RUNNING` / `CANCELLED`：异步任务冲突或已取消

错误透传说明：

- 末端坐标系目标设置路径中，`set_target_in_eef_frame` 会保留并返回真实错误码。
- 例如 TF 变换失败时直接返回 `TF_TRANSFORM_FAILED`，便于定位问题来源。

建议排查流程：

1. 先调用 `/arm_query` 确认当前状态可读。
2. 再检查命令对应参数数量是否满足约束。
3. 若是坐标相关失败，检查 TF 树和 EEF/TCP 配置。
4. 若是规划失败，先收紧目标范围并降低速度参数再尝试。
