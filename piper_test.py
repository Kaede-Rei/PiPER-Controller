#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import copy
import shlex
import signal
import subprocess
from urllib.parse import urlparse

import rosgraph
import rospy
import actionlib

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

from piper_msgs2.srv import QueryArm, QueryArmRequest
from piper_msgs2.srv import ConfigArm, ConfigArmRequest
from piper_msgs2.msg import MoveArmAction, MoveArmGoal
from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal

# =========================
# roscore / 进程管理
# =========================


def is_local_master():
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    host = urlparse(master_uri).hostname
    return host in (None, "localhost", "127.0.0.1")


def wait_master_online(timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if rosgraph.is_master_online():
            return True
        time.sleep(0.2)
    return False


def start_roscore_if_needed():
    if rosgraph.is_master_online():
        print("[INFO] 检测到 roscore 已在线，复用现有 master")
        return None

    if not is_local_master():
        raise RuntimeError(
            "当前 ROS_MASTER_URI 不是本地地址，且 master 不在线，拒绝自动启动本地 roscore。"
        )

    print("[INFO] 未检测到 roscore，正在自动启动本地 roscore ...")
    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    if not wait_master_online(timeout=10.0):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            pass
        raise RuntimeError("roscore 启动失败或超时")

    print(f"[INFO] roscore 已启动，PID={proc.pid}")
    return proc


def stop_process_group(proc, name="process", timeout=10):
    if proc is None:
        return
    if proc.poll() is not None:
        return

    print(f"[INFO] 关闭 {name} (PID={proc.pid}) ...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


# =========================
# 数学工具
# =========================


def quat_norm(q):
    return math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)


def normalize_quaternion(q):
    n = quat_norm(q)
    if n < 1e-12:
        return Quaternion(0.0, 0.0, 0.0, 1.0)
    return Quaternion(q.x / n, q.y / n, q.z / n, q.w / n)


def quaternion_to_rpy(q):
    q = normalize_quaternion(q)
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# =========================
# Runner
# =========================


class Runner:
    def __init__(self):
        self.pass_items = []
        self.fail_items = []
        self.warn_items = []

        self.launch_proc = None
        self.move_client = None
        self.simple_client = None

        self.launch_cmd = os.environ.get(
            "TEST_LAUNCH_CMD", "roslaunch --wait piper_interface piper_start.launch"
        )

        self.query_srv_name = rospy.get_param("~arm_query_service", "/arm_query")
        self.config_srv_name = rospy.get_param("~arm_config_service", "/arm_config")
        self.move_arm_ns = rospy.get_param("~move_arm_action", "/move_arm")
        self.simple_move_arm_ns = rospy.get_param(
            "~simple_move_arm_action", "/simple_move_arm"
        )

        self.test_joint_name = rospy.get_param(
            "~test_joint_name", os.environ.get("TEST_JOINT_NAME", "joint1")
        )

        self.run_reset_tests = rospy.get_param("~run_reset_tests", True)
        self.run_end_tests = rospy.get_param("~run_end_tests", True)

        # 关键：为了“看全程”，每一步后停一下
        self.step_pause_sec = float(rospy.get_param("~step_pause_sec", 2.0))
        self.final_hold_forever = bool(rospy.get_param("~final_hold_forever", True))
        self.final_hold_sec = float(rospy.get_param("~final_hold_sec", 20.0))

        self.cur_pose = None
        self.cur_joints = []

    # ---------- 输出 ----------
    def info(self, msg):
        print(f"[INFO] {msg}")

    def ok(self, msg):
        print(f"[PASS] {msg}")
        self.pass_items.append(msg)

    def warn(self, msg):
        print(f"[WARN] {msg}")
        self.warn_items.append(msg)

    def fail(self, msg):
        print(f"[FAIL] {msg}")
        self.fail_items.append(msg)

    def summary(self):
        print("\n==================== 测试汇总 ====================")
        print(f"PASS 数量 : {len(self.pass_items)}")
        print(f"WARN 数量 : {len(self.warn_items)}")
        print(f"FAIL 数量 : {len(self.fail_items)}")

        if self.pass_items:
            print("\n[PASS 列表]")
            for x in self.pass_items:
                print(f"  - {x}")

        if self.warn_items:
            print("\n[WARN 列表]")
            for x in self.warn_items:
                print(f"  - {x}")

        if self.fail_items:
            print("\n[FAIL 列表]")
            for x in self.fail_items:
                print(f"  - {x}")

        print()
        return 0 if not self.fail_items else 1

    # ---------- 启停 launch ----------
    def start_launch(self):
        self.info(f"启动系统: {self.launch_cmd}")
        cmd = shlex.split(self.launch_cmd)
        self.launch_proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
        )
        self.info(f"roslaunch 已启动，PID={self.launch_proc.pid}")

    def stop_launch(self):
        stop_process_group(self.launch_proc, name="roslaunch", timeout=10)

    # ---------- 等待系统 ----------
    def wait_for_system(self):
        try:
            rospy.wait_for_service(self.query_srv_name, timeout=90.0)
            rospy.wait_for_service(self.config_srv_name, timeout=90.0)
            self.ok("QueryArm / ConfigArm 服务已上线")
        except Exception as e:
            self.fail(f"服务未正常上线: {e}")

        self.move_client = actionlib.SimpleActionClient(self.move_arm_ns, MoveArmAction)
        self.simple_client = actionlib.SimpleActionClient(
            self.simple_move_arm_ns, SimpleMoveArmAction
        )

        if self.move_client.wait_for_server(rospy.Duration(60.0)):
            self.ok("move_arm action 已上线")
        else:
            self.fail("move_arm action 未上线")

        if self.simple_client.wait_for_server(rospy.Duration(60.0)):
            self.ok("simple_move_arm action 已上线")
        else:
            self.fail("simple_move_arm action 未上线")

        try:
            rospy.wait_for_message("/joint_states", JointState, timeout=15.0)
            self.ok("/joint_states 已收到消息")
        except Exception as e:
            self.fail(f"/joint_states 未正常收到消息: {e}")

        try:
            hz_text = subprocess.check_output(
                [
                    "bash",
                    "-lc",
                    "timeout 3 rostopic hz /joint_states 2>/dev/null || true",
                ],
                text=True,
            ).strip()
            if hz_text:
                self.info("/joint_states hz 输出：")
                print(hz_text)
        except Exception:
            pass

    # ---------- service ----------
    def query_proxy(self):
        return rospy.ServiceProxy(self.query_srv_name, QueryArm)

    def config_proxy(self):
        return rospy.ServiceProxy(self.config_srv_name, ConfigArm)

    # ---------- 查询 ----------
    def check_query_current_joints(self):
        self.info("测试 QueryArm: GET_CURRENT_JOINTS")
        try:
            srv = self.query_proxy()
            req = QueryArmRequest()
            req.command_type = QueryArmRequest.GET_CURRENT_JOINTS
            req.values = []

            res = srv(req)
            print(res)

            if not res.success:
                self.fail(f"GET_CURRENT_JOINTS 返回 success=False: {res.message}")
                return

            if len(res.cur_joint) == 0:
                self.fail("GET_CURRENT_JOINTS 返回空关节数组")
                return

            self.cur_joints = list(res.cur_joint)
            self.ok(f"GET_CURRENT_JOINTS 成功，关节数={len(self.cur_joints)}")
        except Exception as e:
            self.fail(f"GET_CURRENT_JOINTS 调用异常: {e}")

    def check_query_current_pose(self):
        self.info("测试 QueryArm: GET_CURRENT_POSE")
        try:
            srv = self.query_proxy()
            req = QueryArmRequest()
            req.command_type = QueryArmRequest.GET_CURRENT_POSE
            req.values = []

            res = srv(req)
            print(res)

            if not res.success:
                self.fail(f"GET_CURRENT_POSE 返回 success=False: {res.message}")
                return

            q = res.cur_pose.orientation
            if quat_norm(q) < 1e-6:
                self.fail("GET_CURRENT_POSE 返回了无效四元数")
                return

            self.cur_pose = res.cur_pose
            self.ok("GET_CURRENT_POSE 成功")
        except Exception as e:
            self.fail(f"GET_CURRENT_POSE 调用异常: {e}")

    # ---------- 配置 ----------
    def check_config_orientation_constraint(self):
        self.info("测试 ConfigArm: SET_ORIENTATION_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_ORIENTATION_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.0, 0.0, 0.0)
            req.joint_names = []
            req.joints = []
            req.values = []

            res = srv(req)
            print(res)

            if res.success:
                self.ok("SET_ORIENTATION_CONSTRAINT 成功")
            else:
                self.fail(f"SET_ORIENTATION_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_ORIENTATION_CONSTRAINT 调用异常: {e}")

    def check_config_position_constraint(self):
        self.info("测试 ConfigArm: SET_POSITION_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_POSITION_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.30, 0.00, 0.20)
            req.joint_names = []
            req.joints = []
            req.values = [0.05, 0.05, 0.05]

            res = srv(req)
            print(res)

            if res.success:
                self.ok("SET_POSITION_CONSTRAINT 成功")
            else:
                self.fail(f"SET_POSITION_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_POSITION_CONSTRAINT 调用异常: {e}")

    def check_config_joint_constraint(self):
        self.info("测试 ConfigArm: SET_JOINT_CONSTRAINT")
        try:
            srv = self.config_proxy()
            req = ConfigArmRequest()
            req.command_type = ConfigArmRequest.SET_JOINT_CONSTRAINT
            req.quaternion = Quaternion(0.0, 0.0, 0.0, 1.0)
            req.point = Point(0.0, 0.0, 0.0)
            req.joint_names = [self.test_joint_name]
            req.joints = [0.0]
            req.values = [0.1, 0.1]

            res = srv(req)
            print(res)

            if res.success:
                self.ok(f"SET_JOINT_CONSTRAINT 成功（joint={self.test_joint_name}）")
            else:
                self.fail(f"SET_JOINT_CONSTRAINT 失败: {res.message}")
        except Exception as e:
            self.fail(f"SET_JOINT_CONSTRAINT 调用异常: {e}")

    # ---------- 辅助 ----------
    def _identity_pose(self):
        p = Pose()
        p.position.x = 0.0
        p.position.y = 0.0
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        return p

    def _copy_pose(self, pose):
        return copy.deepcopy(pose)

    def _pose_offset(self, base_pose, dx=0.0, dy=0.0, dz=0.0):
        p = self._copy_pose(base_pose)
        p.position.x += dx
        p.position.y += dy
        p.position.z += dz
        return p

    def _require_current_state(self):
        if self.cur_pose is None:
            self.warn("当前 pose 不可用，使用单位姿态兜底")
            self.cur_pose = self._identity_pose()

        if not self.cur_joints:
            self.warn("当前 joints 不可用，使用全零兜底")
            self.cur_joints = [0.0] * 6

    def _current_rpy_or_zero(self):
        if self.cur_pose is None:
            return (0.0, 0.0, 0.0)

        q = self.cur_pose.orientation
        if quat_norm(q) < 1e-6:
            self.warn("当前 pose 四元数无效，SimpleMoveArm 将使用 0/0/0 RPY 兜底")
            return (0.0, 0.0, 0.0)

        try:
            return quaternion_to_rpy(q)
        except Exception as e:
            self.warn(f"四元数转 RPY 失败，使用 0/0/0 兜底: {e}")
            return (0.0, 0.0, 0.0)

    def _visible_joint_target(self, base_joints, offsets):
        out = list(base_joints)
        for i, off in enumerate(offsets):
            if i < len(out):
                out[i] += off
        return out

    def _pause_watch(self, label):
        prompt = f"[INFO] {label} 完成，按 Enter 继续下一步..."
        try:
            # 交互终端下：等待用户回车
            if sys.stdin and sys.stdin.isatty():
                input(prompt)
            else:
                # 非交互环境下避免卡死
                self.warn(f"{label} 完成，但当前不是交互终端，自动继续")
        except EOFError:
            self.warn(f"{label} 完成，但 stdin 已关闭，自动继续")
        except KeyboardInterrupt:
            raise

    # ---------- action send ----------
    def send_move_arm(self, name, fill_goal_fn, timeout=30.0):
        self.info(f"测试 ArmMoveAction: {name}")
        try:
            goal = MoveArmGoal()
            fill_goal_fn(goal)

            def feedback_cb(fb):
                try:
                    self.info(
                        f"[{name}] stage={fb.stage} progress={fb.progress:.1f}% message={fb.message}"
                    )
                except Exception:
                    pass

            self.move_client.send_goal(goal, feedback_cb=feedback_cb)
            finished = self.move_client.wait_for_result(rospy.Duration(timeout))
            if not finished:
                self.move_client.cancel_goal()
                self.fail(f"{name} 超时")
                return False

            state = self.move_client.get_state()
            result = self.move_client.get_result()
            print(result)

            if result and result.success:
                self.ok(f"ArmMoveAction::{name} 成功")
                self._pause_watch(name)
                return True
            else:
                self.fail(f"ArmMoveAction::{name} 失败, state={state}, result={result}")
                return False
        except Exception as e:
            self.fail(f"ArmMoveAction::{name} 异常: {e}")
            return False

    def send_simple_move_arm(self, name, fill_goal_fn, timeout=30.0):
        self.info(f"测试 SimpleMoveArmAction: {name}")
        try:
            goal = SimpleMoveArmGoal()
            fill_goal_fn(goal)

            def feedback_cb(fb):
                try:
                    self.info(
                        f"[{name}] stage={fb.stage} progress={fb.progress:.1f}% message={fb.message}"
                    )
                except Exception:
                    pass

            self.simple_client.send_goal(goal, feedback_cb=feedback_cb)
            finished = self.simple_client.wait_for_result(rospy.Duration(timeout))
            if not finished:
                self.simple_client.cancel_goal()
                self.fail(f"{name} 超时")
                return False

            state = self.simple_client.get_state()
            result = self.simple_client.get_result()
            print(result)

            if result and result.success:
                self.ok(f"SimpleMoveArmAction::{name} 成功")
                self._pause_watch(name)
                return True
            else:
                self.fail(
                    f"SimpleMoveArmAction::{name} 失败, state={state}, result={result}"
                )
                return False
        except Exception as e:
            self.fail(f"SimpleMoveArmAction::{name} 异常: {e}")
            return False

    # ---------- MoveArm: 可见版 ----------
    def test_move_arm_all_visible(self):
        self.refresh_current_state()
        self._require_current_state()

        def dummy_point(g):
            g.target_type = MoveArmGoal.TARGET_POINT
            g.point = Point(0.0, 0.0, 0.0)

        if self.run_reset_tests:
            self.send_move_arm(
                "HOME",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.HOME),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                    setattr(g, "waypoints", []),
                ),
            )

        # 现取 joints，再做可见的小偏移
        self.refresh_current_state()
        self._require_current_state()
        joints_a = self._visible_joint_target(
            self.cur_joints, [0.20, 0.10, 0.00, 0.00, 0.00, 0.00]
        )
        joints_b = self._visible_joint_target(
            self.cur_joints, [-0.15, 0.15, 0.05, 0.00, 0.00, 0.00]
        )

        self.send_move_arm(
            "MOVE_JOINTS_A",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_JOINTS),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", joints_a),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        self.send_move_arm(
            "MOVE_JOINTS_B",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_JOINTS),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", joints_b),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        # 普通目标：每次现取当前 pose 再偏移
        pose_up = self.current_pose_plus(dz=0.02)
        self.send_move_arm(
            "MOVE_TARGET_UP",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_TARGET),
                setattr(g, "target_type", MoveArmGoal.TARGET_POSE),
                setattr(g, "pose", pose_up),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        pose_forward = self.current_pose_plus(dx=0.02)
        self.send_move_arm(
            "MOVE_TARGET_FORWARD",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_TARGET),
                setattr(g, "target_type", MoveArmGoal.TARGET_POSE),
                setattr(g, "pose", pose_forward),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        self.send_move_arm(
            "MOVE_TARGET_IN_EEF_FRAME_ZPLUS",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_TARGET_IN_EEF_FRAME),
                setattr(g, "target_type", MoveArmGoal.TARGET_POSE),
                setattr(g, "pose", self._pose_offset(self._identity_pose(), dz=0.01)),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", []),
            ),
        )

        if self.run_end_tests:
            self.send_move_arm(
                "TELESCOPIC_END_PLUS",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.TELESCOPIC_END),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.01]),
                    setattr(g, "waypoints", []),
                ),
            )

            self.send_move_arm(
                "ROTATE_END_PLUS",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.ROTATE_END),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.05]),
                    setattr(g, "waypoints", []),
                ),
            )

        # ===== 笛卡尔：参照 test_node，现取当前 pose，路径尽量温和 =====

        # line: 当前 pose -> y + 0.02
        line_start, line_end = self.current_pose_triplet(
            [
                (0.00, 0.00, 0.00),
                (0.00, 0.02, 0.00),
            ]
        )
        self.send_move_arm(
            "MOVE_LINE_VISIBLE",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_LINE),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [line_start, line_end]),
            ),
        )

        # bezier: 当前 pose 为起点，via/end 参照 test_node
        bezier_start, bezier_via, bezier_end = self.current_pose_triplet(
            [
                (0.00, 0.00, 0.00),
                (0.02, 0.03, 0.00),
                (0.04, 0.00, 0.00),
            ]
        )
        self.send_move_arm(
            "MOVE_BEZIER_VISIBLE",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_BEZIER),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [bezier_start, bezier_via, bezier_end]),
            ),
            timeout=45.0,
        )

        # decartes: 3 个连续小步 waypoint，不要跨太大
        d1, d2, d3 = self.current_pose_triplet(
            [
                (0.00, 0.008, 0.000),
                (0.005, 0.016, 0.000),
                (0.010, 0.024, 0.000),
            ]
        )
        self.send_move_arm(
            "MOVE_DECARTES_VISIBLE",
            lambda g: (
                setattr(g, "command_type", MoveArmGoal.MOVE_DECARTES),
                dummy_point(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
                setattr(g, "waypoints", [d1, d2, d3]),
            ),
            timeout=45.0,
        )

        if self.run_reset_tests:
            self.send_move_arm(
                "MOVE_TO_ZERO",
                lambda g: (
                    setattr(g, "command_type", MoveArmGoal.MOVE_TO_ZERO),
                    dummy_point(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                    setattr(g, "waypoints", []),
                ),
            )

    # ---------- SimpleMoveArm: 可见版 ----------
    def test_simple_move_arm_all_visible(self):
        self.refresh_current_state()
        self._require_current_state()

        def fill_dummy_arrays(g):
            g.x = [0.0]
            g.y = [0.0]
            g.z = [0.0]
            g.roll = [0.0]
            g.pitch = [0.0]
            g.yaw = [0.0]

        if self.run_reset_tests:
            self.send_simple_move_arm(
                "HOME",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.HOME),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                ),
            )

        self.refresh_current_state()
        self._require_current_state()
        joints_c = self._visible_joint_target(
            self.cur_joints, [0.10, 0.10, 0.08, 0.0, 0.0, 0.0]
        )

        self.send_simple_move_arm(
            "MOVE_JOINTS_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_JOINTS),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                fill_dummy_arrays(g),
                setattr(g, "joint_names", []),
                setattr(g, "joints", joints_c),
                setattr(g, "values", []),
            ),
        )

        x, y, z, r, p, yw = self.current_simple_pose_arrays([(0.02, 0.00, 0.00)])
        self.send_simple_move_arm(
            "MOVE_TARGET_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TARGET),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", x),
                setattr(g, "y", y),
                setattr(g, "z", z),
                setattr(g, "roll", r),
                setattr(g, "pitch", p),
                setattr(g, "yaw", yw),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        self.send_simple_move_arm(
            "MOVE_TARGET_IN_EEF_FRAME_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TARGET_IN_EEF_FRAME),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", [0.0]),
                setattr(g, "y", [0.0]),
                setattr(g, "z", [0.01]),
                setattr(g, "roll", [0.0]),
                setattr(g, "pitch", [0.0]),
                setattr(g, "yaw", [0.0]),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
        )

        if self.run_end_tests:
            self.send_simple_move_arm(
                "TELESCOPIC_END_VISIBLE",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.TELESCOPIC_END),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.01]),
                ),
            )

            self.send_simple_move_arm(
                "ROTATE_END_VISIBLE",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.ROTATE_END),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", [0.05]),
                ),
            )

        # ===== 笛卡尔 =====

        # MOVE_LINE: 必须 2 点
        x, y, z, r, p, yw = self.current_simple_pose_arrays(
            [
                (0.00, 0.00, 0.00),
                (0.00, 0.02, 0.00),
            ]
        )
        self.send_simple_move_arm(
            "MOVE_LINE_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_LINE),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", x),
                setattr(g, "y", y),
                setattr(g, "z", z),
                setattr(g, "roll", r),
                setattr(g, "pitch", p),
                setattr(g, "yaw", yw),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
            timeout=45.0,
        )

        # MOVE_BEZIER: 参照 test_node 的 via/end
        x, y, z, r, p, yw = self.current_simple_pose_arrays(
            [
                (0.00, 0.00, 0.00),
                (0.02, 0.03, 0.00),
                (0.04, 0.00, 0.00),
            ]
        )
        self.send_simple_move_arm(
            "MOVE_BEZIER_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_BEZIER),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", x),
                setattr(g, "y", y),
                setattr(g, "z", z),
                setattr(g, "roll", r),
                setattr(g, "pitch", p),
                setattr(g, "yaw", yw),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
            timeout=45.0,
        )

        # MOVE_DECARTES: 3 个连续小步
        x, y, z, r, p, yw = self.current_simple_pose_arrays(
            [
                (0.00, 0.008, 0.000),
                (0.005, 0.016, 0.000),
                (0.010, 0.024, 0.000),
            ]
        )
        self.send_simple_move_arm(
            "MOVE_DECARTES_VISIBLE",
            lambda g: (
                setattr(g, "command_type", SimpleMoveArmGoal.MOVE_DECARTES),
                setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POSE),
                setattr(g, "x", x),
                setattr(g, "y", y),
                setattr(g, "z", z),
                setattr(g, "roll", r),
                setattr(g, "pitch", p),
                setattr(g, "yaw", yw),
                setattr(g, "joint_names", []),
                setattr(g, "joints", []),
                setattr(g, "values", []),
            ),
            timeout=45.0,
        )

        if self.run_reset_tests:
            self.send_simple_move_arm(
                "MOVE_TO_ZERO",
                lambda g: (
                    setattr(g, "command_type", SimpleMoveArmGoal.MOVE_TO_ZERO),
                    setattr(g, "target_type", SimpleMoveArmGoal.TARGET_POINT),
                    fill_dummy_arrays(g),
                    setattr(g, "joint_names", []),
                    setattr(g, "joints", []),
                    setattr(g, "values", []),
                ),
            )

    def refresh_current_state(self):
        """每次真正发运动命令前，从服务端重新取一次当前 joints / pose。"""
        try:
            q_srv = self.query_proxy()

            q_req = QueryArmRequest()
            q_req.command_type = QueryArmRequest.GET_CURRENT_JOINTS
            q_req.values = []
            q_res = q_srv(q_req)
            if q_res.success and len(q_res.cur_joint) > 0:
                self.cur_joints = list(q_res.cur_joint)
            else:
                self.warn(f"刷新当前关节失败，沿用缓存值: {q_res.message}")

            p_req = QueryArmRequest()
            p_req.command_type = QueryArmRequest.GET_CURRENT_POSE
            p_req.values = []
            p_res = q_srv(p_req)
            if p_res.success and quat_norm(p_res.cur_pose.orientation) > 1e-6:
                self.cur_pose = p_res.cur_pose
            else:
                self.warn(f"刷新当前位姿失败，沿用缓存值: {p_res.message}")
        except Exception as e:
            self.warn(f"刷新当前状态异常，沿用缓存值: {e}")

    def current_pose_plus(self, dx=0.0, dy=0.0, dz=0.0):
        """现取当前 pose，再做偏移。"""
        self.refresh_current_state()
        self._require_current_state()
        return self._pose_offset(self.cur_pose, dx=dx, dy=dy, dz=dz)

    def current_pose_triplet(self, offsets):
        """
        offsets: [(dx,dy,dz), ...]
        返回基于同一个“当前 pose 基准”的多个 waypoint。
        """
        self.refresh_current_state()
        self._require_current_state()
        base = self._copy_pose(self.cur_pose)
        out = []
        for dx, dy, dz in offsets:
            out.append(self._pose_offset(base, dx=dx, dy=dy, dz=dz))
        return out

    def current_simple_pose_arrays(self, offsets):
        """
        offsets: [(dx,dy,dz), ...]
        现取当前 pose + 当前 rpy，生成 SimpleMoveArmGoal 所需数组。
        """
        self.refresh_current_state()
        self._require_current_state()

        base = self._copy_pose(self.cur_pose)
        r0, p0, y0 = self._current_rpy_or_zero()

        xs, ys, zs = [], [], []
        rs, ps, ysaw = [], [], []

        for dx, dy, dz in offsets:
            pose = self._pose_offset(base, dx=dx, dy=dy, dz=dz)
            xs.append(pose.position.x)
            ys.append(pose.position.y)
            zs.append(pose.position.z)
            rs.append(r0)
            ps.append(p0)
            ysaw.append(y0)

        return xs, ys, zs, rs, ps, ysaw

    # ---------- 主流程 ----------
    def run(self):
        self.start_launch()
        time.sleep(5.0)

        self.wait_for_system()

        self.check_query_current_joints()
        self.check_query_current_pose()

        self.test_move_arm_all_visible()
        self.test_simple_move_arm_all_visible()

        self.check_config_orientation_constraint()
        self.check_config_position_constraint()
        self.check_config_joint_constraint()

        code = self.summary()

        self.info("所有测试已结束。按 Enter 退出并关闭 roslaunch / roscore ...")
        try:
            if sys.stdin and sys.stdin.isatty():
                input()
            else:
                self.warn("当前不是交互终端，3 秒后自动退出")
                time.sleep(3.0)
        except EOFError:
            self.warn("stdin 已关闭，自动退出")
        except KeyboardInterrupt:
            pass

        return code


def main():
    roscore_proc = None
    runner = None

    try:
        roscore_proc = start_roscore_if_needed()

        rospy.init_node("piper_full_interface_tester", anonymous=True)

        runner = Runner()
        code = runner.run()
        sys.exit(code)

    finally:
        if runner is not None:
            runner.stop_launch()
        stop_process_group(roscore_proc, name="roscore", timeout=5)


if __name__ == "__main__":
    main()
