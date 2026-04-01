[toc]

# Piper机械臂接口文档（piper_tomato 新接口）

> 本文档适用于 `piper_tomato/src/piper_interface`，不再使用旧版 `/piper_server/*` + `piper_cmd` 字符串协议。

## 1. 接口总览

### 1.1 Action 接口

- `/move_arm`，类型：`piper_msgs2/MoveArmAction`
- `/simple_move_arm`，类型：`piper_msgs2/SimpleMoveArmAction`

### 1.2 Service 接口

- `/arm_config`，类型：`piper_msgs2/ConfigArm`
- `/arm_query`，类型：`piper_msgs2/QueryArm`

---

## 2. 命令编号（`command_type`）

通用运动命令（Action）：

- `1` `HOME`
- `2` `MOVE_JOINTS`
- `3` `MOVE_TARGET`
- `4` `MOVE_TARGET_IN_EEF_FRAME`
- `5` `TELESCOPIC_END`
- `6` `ROTATE_END`
- `7` `MOVE_LINE`
- `8` `MOVE_BEZIER`
- `9` `MOVE_DECARTES`
- `15` `MOVE_TO_ZERO`

配置命令（`/arm_config`）：

- `10` `SET_ORIENTATION_CONSTRAINT`
- `11` `SET_POSITION_CONSTRAINT`
- `12` `SET_JOINT_CONSTRAINT`

查询命令（`/arm_query`）：

- `13` `GET_CURRENT_JOINTS`
- `14` `GET_CURRENT_POSE`

---

## 3. Action 消息格式

### 3.1 `MoveArmAction`

- 目标类型（`target_type`）：
	- `0` `TARGET_POSE`
	- `1` `TARGET_POINT`
	- `2` `TARGET_QUATERNION`

- 关键字段：
	- `command_type`
	- `target_type`
	- `pose / point / quaternion`
	- `waypoints`（用于 line / bezier / decartes）
	- `joint_names`、`joints`
	- `values`

### 3.2 `SimpleMoveArmAction`

- 目标类型（`target_type`）：
	- `0` `TARGET_POSE`
	- `1` `TARGET_POINT`
	- `2` `TARGET_ORIENTATION`

- 位姿字段改为数组：
	- `x[] y[] z[]`
	- `roll[] pitch[] yaw[]`

说明：

- `MOVE_LINE` 需要 2 个点。
- `MOVE_BEZIER` 需要 3 个点。
- `MOVE_DECARTES` 需要 >=1 个点。

---

## 4. Service 消息格式

### 4.1 `/arm_config` (`piper_msgs2/ConfigArm`)

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

### 4.2 `/arm_query` (`piper_msgs2/QueryArm`)

Request：

- `command_type`
- `values[]`

Response：

- `success`
- `message`
- `error_code`
- `cur_pose`
- `cur_joint[]`

---

## 5. 调用示例

### 5.1 查询当前关节

```bash
rosservice call /arm_query "command_type: 13
values: []"
```

### 5.2 查询当前位姿

```bash
rosservice call /arm_query "command_type: 14
values: []"
```

### 5.3 设置姿态约束

```bash
rosservice call /arm_config "command_type: 10
point: {x: 0.0, y: 0.0, z: 0.0}
quaternion: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_names: []
joints: []
values: []"
```

### 5.4 通过 `simple_move_arm` 在末端坐标系移动

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

