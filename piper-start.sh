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
SUCCESS=false
ROSLAUNCH_PID=""
CLEANING_UP=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --disable|-d)
            DISABLE_ON_EXIT=true
            shift
            ;;
        --delay|-t)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --delay 需要一个秒数参数"
                exit 1
            fi
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
            exit 1
            ;;
    esac
done

cleanup() {
    # 防止重复进入 cleanup
    if [[ "$CLEANING_UP" == true ]]; then
        return
    fi
    CLEANING_UP=true

    if [[ "$SUCCESS" == false ]]; then
        echo "PiPER 未成功启动，直接退出"
        exit 1
    fi

    echo "检测到中断，尝试令机械臂回到零点 ..."
    python3 - <<'PY' || echo "跳过回零动作（服务未就绪）"
import sys
import actionlib
import rospy
from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal

def main():
    rospy.init_node("piper_exit_zero_checker", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient("/simple_move_arm", SimpleMoveArmAction)

    if not client.wait_for_server(rospy.Duration(1.5)):
        return 1

    print("检测到服务，正在令机械臂回到零点...")
    goal = SimpleMoveArmGoal()
    goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(10.0)):
        client.cancel_goal()
        print("[WARN] 回零超时")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY

    sleep "$DELAY_SEC"

    if [[ "$DISABLE_ON_EXIT" == true ]]; then
        if rosservice list 2>/dev/null | grep -q "^/enable_srv$"; then
            echo "[ACTION] 正在使系统失能..."
            rosservice call /enable_srv "enable_request: false" >/dev/null 2>&1 || true
        fi
    fi

    if [[ -n "$ROSLAUNCH_PID" ]]; then
        echo "[ACTION] 正在关闭 roslaunch (PID: $ROSLAUNCH_PID)..."
        kill "$ROSLAUNCH_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$ROSLAUNCH_PID" 2>/dev/null || true
    fi

    echo "退出完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "================ 启动 Piper 系统 ================"

# 配置 CAN 接口
echo "[1/2] 配置 CAN 接口"
if [ -f "$SCRIPT_DIR/can-activate.sh" ]; then
    sudo "$SCRIPT_DIR/can-activate.sh" || echo "[WARN] CAN 配置脚本执行异常，尝试继续启动 ROS..."
else
    echo "[WARN] 未找到 can-activate.sh，跳过配置步骤"
fi

# 启动 ROS Launch
echo "[2/2] 启动 ROS Launch..."
roslaunch piper_interface piper_start.launch &
ROSLAUNCH_PID=$!

# 只要 roslaunch 成功拉起，cleanup 就可以接管退出流程
SUCCESS=true

# 等待子进程退出
wait "$ROSLAUNCH_PID" || true
