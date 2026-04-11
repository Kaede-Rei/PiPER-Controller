#!/usr/bin/env bash

# 启动 Piper 系统脚本，并在收到中断时执行回零（可选失能）
#
# 使用方法：
#   ./piper-start.sh [--disable] [--delay 秒数]
#   ./piper-start.sh [-d] [-t 秒数]
#
# 示例：
#   ./piper-start.sh
#   ./piper-start.sh --disable 或 ./piper-start.sh -d
#   ./piper-start.sh --delay 5 或 ./piper-start.sh -t 5
#   ./piper-start.sh --disable --delay 8
#
# 参数：
#   --disable, -d   中断时调用 /enable_srv 使系统失能，默认不失能
#   --delay, -t     回零后等待时间（秒），默认 2 秒

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# source ROS 环境
if [ -f "$SCRIPT_DIR/piper_ros/devel/setup.bash" ]; then
    source "$SCRIPT_DIR/piper_ros/devel/setup.bash"
else
    echo "未找到 PiPER ROS 工作空间，请先编译 ROS 包"
    exit 1
fi
if [ -f "$SCRIPT_DIR/piper_tomato/devel/setup.bash" ]; then
    source "$SCRIPT_DIR/piper_tomato/devel/setup.bash"
else
    echo "未找到 PiPER Tomato 工作空间，请先编译 ROS 包"
    exit 1
fi

# 默认参数
DISABLE_ON_EXIT=false
DELAY_SEC=2
SUCESS=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --disable|-d)
            DISABLE_ON_EXIT=true
            shift
            ;;
        --delay|-t)
            DELAY_SEC="$2"
            shift 2
            ;;
        --help|-h)
            grep -E '^#' "$0" | sed 's/^# //'
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 --help 查看用法"
            ;;
    esac
done

function cleanup() {
    if [ "$SUCESS" = false ]; then
        echo "PiPER 未成功启动，直接退出"
        exit 1
    fi

    echo "检测到中断，令机械臂回到零点 ..."
    python3 - <<'PY' || true
import sys

import actionlib
import rospy

from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal


def main() -> int:
    rospy.init_node("piper_exit_zero", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient("/simple_move_arm", SimpleMoveArmAction)

    if not client.wait_for_server(rospy.Duration(5.0)):
        print("[WARN] 未找到 /simple_move_arm action server")
        return 1

    goal = SimpleMoveArmGoal()
    goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
    goal.target_type = SimpleMoveArmGoal.TARGET_POSE
    goal.x = [0.0]
    goal.y = [0.0]
    goal.z = [0.0]
    goal.roll = [0.0]
    goal.pitch = [0.0]
    goal.yaw = [0.0]

    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(15.0)):
        client.cancel_goal()
        print("[WARN] 回零动作等待超时")
        return 1

    result = client.get_result()
    print(result)
    return 0


sys.exit(main())
PY

    # 等待自定义时间
    sleep "$DELAY_SEC"

    # 可选：失能
    if [[ "$DISABLE_ON_EXIT" == true ]]; then
        rosservice call /enable_srv "enable_request: false" || true
    fi

    echo "正在关闭 roslaunch ..."
    kill $ROSLAUNCH_PID 2>/dev/null || true
    wait $ROSLAUNCH_PID 2>/dev/null || true

    echo "退出完成"
}

trap cleanup SIGINT SIGTERM

echo "================ 启动 Piper 系统 ================"

# 激活 CAN
echo "[1/2] 配置 CAN 接口"
sudo "$SCRIPT_DIR/can-activate.sh"
echo "[CAN] 配置完成"

# 启动 ROS
echo "[2/2] 启动 ROS Launch"
setsid roslaunch piper_interface piper_start.launch &
ROSLAUNCH_PID=$!
SUCESS=true

wait $ROSLAUNCH_PID
