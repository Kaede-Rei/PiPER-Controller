#! /usr/bin/env python
import sys
import cv2
import numpy as np
from typing import Union, Any, Optional

# ================= 新增 ROS 相关导入 =================
import rospy
import actionlib
from piper_msgs2.msg import SimpleMoveArmAction, SimpleMoveArmGoal

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

from pyorbbecsdk import OBFormat, OBSensorType, OBAlignMode, OBPropertyID
from pyorbbecsdk import Pipeline, Config, VideoFrame

# ================= 导入 PyQt5 =================
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
)
from PyQt5.QtGui import (
    QImage,
    QPixmap,
    QPainter,
    QPen,
    QColor,
    QFont,
    QPolygonF,
    QBrush,
)
from PyQt5.QtCore import Qt, QPoint, QPointF, pyqtSignal, QThread

# ================= 全局变量和辅助函数 =================
ESC_KEY = 27
MIN_DEPTH = 100  # 100mm
MAX_DEPTH = 10000  # 10000mm
LRM_VALID_MIN = 1  # LRM最小有效距离1mm
LRM_VALID_MAX = 400  # LRM最大有效距离400mm (核心：0.4m以内强制用LRM)


def frame_to_bgr_image(frame: VideoFrame) -> Union[Optional[np.array], Any]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())
    image = np.zeros((height, width, 3), dtype=np.uint8)
    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif color_format == OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    elif color_format == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    else:
        return None
    return image


def hand_eye_calibration(x_cam, y_cam, z_cam):
    T_cam2end = np.array(
        [
            [0.103597, 0.960173, 0.259493, -0.059774],
            [-0.994337, 0.1062, 0.004006, 0.011284],
            [-0.023712, -0.258438, 0.965737, 0.107886],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    p_cam_homo = np.array([[x_cam], [y_cam], [z_cam], [1.0]], dtype=np.float64)
    p_end_homo = T_cam2end @ p_cam_homo
    return p_end_homo[0, 0], p_end_homo[1, 0], p_end_homo[2, 0]


def get_mask_center(mask):
    M = cv2.moments(mask.astype(np.uint8))
    if M["m00"] == 0:
        return None
    cx = float(M["m10"] / M["m00"])
    cy = float(M["m01"] / M["m00"])
    return (cx, cy)


def find_cutting_point(stem_mask, stem_center, original_image):
    binary_mask = (stem_mask > 0.5).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        epsilon = 0.001 * cv2.arcLength(largest_contour, True)
        largest_contour = cv2.approxPolyDP(largest_contour, epsilon, True)
        hull = cv2.convexHull(largest_contour, returnPoints=False)
        try:
            defects = cv2.convexityDefects(largest_contour, hull)
            if defects is not None:
                cutting_points = []
                for i in range(defects.shape[0]):
                    s, e, f, d = defects[i, 0]
                    far_point = tuple(largest_contour[f][0])
                    cutting_points.append(far_point)
                if cutting_points:
                    return min(
                        cutting_points,
                        key=lambda p: np.sqrt(
                            (p[0] - stem_center[0]) ** 2 + (p[1] - stem_center[1]) ** 2
                        ),
                    )
        except cv2.error:
            pass
    return stem_center


def depth_to_pointcloud(u, v, depth, depth_intrinsics):
    fx, fy = depth_intrinsics[0, 0], depth_intrinsics[1, 1]
    cx, cy = depth_intrinsics[0, 2], depth_intrinsics[1, 2]
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z


# ================= GUI 组件类 =================


class ImageDisplayWidget(QLabel):
    selection_changed = pyqtSignal(tuple, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)
        self._pixmap = None
        self._screen_points = []
        self._cutting_point_screen = None
        self._is_polygon_closed = False
        self.setStyleSheet("background-color: black;")
        self.setAlignment(Qt.AlignCenter)

    def set_image(self, cv_img):
        if cv_img is None:
            return
        self._latest_cv_image = cv_img
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        qt_img = QImage(rgb_img.data, w, h, ch * w, QImage.Format_RGB888)
        self._pixmap = QPixmap.fromImage(qt_img)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        if self._pixmap:
            scaled_pix = self._pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            x = (self.width() - scaled_pix.width()) // 2
            y = (self.height() - scaled_pix.height()) // 2
            painter.drawPixmap(x, y, scaled_pix)
            self._pixmap_offset = (x, y)
            self._scale_x = scaled_pix.width() / self._pixmap.width()
            self._scale_y = scaled_pix.height() / self._pixmap.height()

        if len(self._screen_points) > 0:
            pen = QPen(QColor(0, 255, 0), 3, Qt.SolidLine)
            painter.setPen(pen)
            for i in range(len(self._screen_points) - 1):
                painter.drawLine(self._screen_points[i], self._screen_points[i + 1])
            if len(self._screen_points) >= 3 or self._is_polygon_closed:
                if len(self._screen_points) >= 3:
                    painter.drawLine(self._screen_points[-1], self._screen_points[0])
                    brush = QBrush(QColor(0, 255, 0, 80))
                    painter.setBrush(brush)
                    painter.drawPolygon(QPolygonF(self._screen_points))
                    painter.setBrush(Qt.NoBrush)
            pen_point = QPen(QColor(255, 0, 0), 12, Qt.SolidLine)
            pen_point.setCapStyle(Qt.RoundCap)
            painter.setPen(pen_point)
            for p in self._screen_points:
                painter.drawPoint(p)

        if self._cutting_point_screen:
            pen_cut = QPen(QColor(255, 0, 255), 15, Qt.SolidLine)
            pen_cut.setCapStyle(Qt.RoundCap)
            painter.setPen(pen_cut)
            painter.drawPoint(self._cutting_point_screen)

    def _screen_to_img(self, pt):
        img_x = np.clip(
            (pt.x() - self._pixmap_offset[0]) / self._scale_x,
            0,
            self._pixmap.width() - 1,
        )
        img_y = np.clip(
            (pt.y() - self._pixmap_offset[1]) / self._scale_y,
            0,
            self._pixmap.height() - 1,
        )
        return (img_x, img_y)

    def _img_to_screen(self, x, y):
        if not hasattr(self, "_scale_x"):
            return QPoint()
        return QPoint(
            int(x * self._scale_x + self._pixmap_offset[0]),
            int(y * self._scale_y + self._pixmap_offset[1]),
        )

    def mousePressEvent(self, event):
        if self._is_polygon_closed:
            self.clear_selection()
        if event.button() == Qt.LeftButton:
            self._screen_points.append(event.pos())
            self._cutting_point_screen = None
            self.update()
            if len(self._screen_points) >= 3:
                self._update_mask_and_calculate()
        elif event.button() == Qt.RightButton:
            if len(self._screen_points) > 0:
                self._screen_points.pop()
                self._cutting_point_screen = None
                self._is_polygon_closed = False
                self.update()
                if len(self._screen_points) >= 3:
                    self._update_mask_and_calculate()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton and len(self._screen_points) >= 3:
            self._is_polygon_closed = True
            self.update()
            self._update_mask_and_calculate()

    def _update_mask_and_calculate(self):
        if len(self._screen_points) < 3 or not hasattr(self, "_latest_cv_image"):
            return
        img_pts = [self._screen_to_img(p) for p in self._screen_points]
        img_pts_np = np.array(img_pts, dtype=np.int32)
        h, w = self._latest_cv_image.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [img_pts_np], 255)
        stem_center = get_mask_center(mask)
        if stem_center is None:
            return
        cutting_point = find_cutting_point(mask, stem_center, self._latest_cv_image)
        if cutting_point is not None:
            self._cutting_point_screen = self._img_to_screen(
                cutting_point[0], cutting_point[1]
            )
            self.update()
            self.selection_changed.emit(cutting_point, mask)

    def clear_selection(self):
        self._screen_points = []
        self._cutting_point_screen = None
        self._is_polygon_closed = False
        self.update()


# ================= 工作线程 (相机采集) =================


class WorkerThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    log_signal = pyqtSignal(str)
    data_ready = pyqtSignal(object, object, tuple)
    intrinsics_ready = pyqtSignal(np.ndarray)
    lrm_data_ready = pyqtSignal(int, float)

    def __init__(self):
        super().__init__()
        self._running = False
        self.pipeline = None
        self._has_sent_intrinsics = False
        self.device = None

    def run(self):
        self._running = True
        self._has_sent_intrinsics = False

        try:
            self.pipeline = Pipeline()
            self.device = self.pipeline.get_device()
            config = Config()

            # 1. 配置 Color Stream
            profile_list = self.pipeline.get_stream_profile_list(
                OBSensorType.COLOR_SENSOR
            )
            try:
                # 彩色流使用1280x720，保持较高清晰度
                color_profile = profile_list.get_video_stream_profile(
                    1280, 720, OBFormat.RGB, 30
                )
            except:
                color_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(color_profile)

            # 2. 配置 Depth Stream 为 848x480 (默认128视差，最近工作距离0.34m)
            profile_list = self.pipeline.get_stream_profile_list(
                OBSensorType.DEPTH_SENSOR
            )
            try:
                depth_profile = profile_list.get_video_stream_profile(
                    848, 480, OBFormat.Y16, 30
                )
            except:
                depth_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(depth_profile)

            # 3. 开启对齐和同步
            config.set_align_mode(OBAlignMode.SW_MODE)
            self.pipeline.enable_frame_sync()

            # 4. 先启动 Pipeline
            self.pipeline.start(config)

            # 5. 开启 LRM 激光补盲模块 (覆盖 0.001m~0.4m，解决20~30cm盲区)
            try:
                self.device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, True)
                self.log_signal.emit("✅ LRM 激光补盲模块已开启 (0.001m~0.4m强制使用)")
            except Exception as e:
                self.log_signal.emit(f"⚠️  LRM 开启失败: {e}")

            self.log_signal.emit(
                f"✅ 相机启动成功 (深度分辨率: {depth_profile.get_width()}x{depth_profile.get_height()}, 视差: 128)"
            )
            self.log_signal.emit(f"⚠️  注意: 0.4m以内深度将优先使用LRM数据")

            # 6. 采集循环
            while self._running:
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if color_frame is None or depth_frame is None:
                    continue

                # 获取内参
                if not self._has_sent_intrinsics:
                    try:
                        stream_profile = depth_frame.get_stream_profile()
                        video_profile = stream_profile.as_video_stream_profile()
                        intrinsics = video_profile.get_intrinsic()
                        K = np.array(
                            [
                                [intrinsics.fx, 0, intrinsics.cx],
                                [0, intrinsics.fy, intrinsics.cy],
                                [0, 0, 1],
                            ],
                            dtype=np.float64,
                        )
                        self.intrinsics_ready.emit(K)
                        self.log_signal.emit(f"✅ 内参读取成功")
                        self._has_sent_intrinsics = True
                    except Exception as e:
                        pass

                # 图像处理
                color_image = frame_to_bgr_image(color_frame)
                if color_image is None:
                    continue

                # 深度数据预处理：中值滤波，去除噪声
                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_data = depth_data.reshape(
                    (depth_frame.get_height(), depth_frame.get_width())
                )
                depth_data = cv2.medianBlur(depth_data, 5)  # 中值滤波去椒盐
                depth_data = np.where(
                    (depth_data > MIN_DEPTH) & (depth_data < MAX_DEPTH), depth_data, 0
                )

                self.frame_ready.emit(color_image)
                self.data_ready.emit(
                    depth_data,
                    depth_frame.get_depth_scale(),
                    (color_frame.get_width(), color_frame.get_height()),
                )

                # 读取 LRM 数据
                if self.device:
                    try:
                        lrm_dist_mm = self.device.get_int_property(
                            OBPropertyID.OB_PROP_LDP_MEASURE_DISTANCE_INT
                        )
                        self.lrm_data_ready.emit(lrm_dist_mm, lrm_dist_mm / 1000.0)
                    except:
                        pass

        except Exception as e:
            self.log_signal.emit(f"❌ 工作线程错误: {e}")
        finally:
            if self.pipeline:
                self.pipeline.stop()

    def stop(self):
        self._running = False
        self.wait()


# ================= 主窗口 GUI (整合ROS) =================


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("番茄")
        self.setGeometry(100, 100, 1280, 720)

        self.current_depth_data = None
        self.current_depth_scale = None
        self.current_color_size = None
        self.current_lrm_mm = 0
        self.current_lrm_m = 0.0
        self.target_arm_coord = None  # 存储机械臂目标坐标
        self.current_cutting_pixel = None  # 存储当前切割点像素

        # ================= ROS 相关初始化 =================
        self.ros_available = False
        self.simple_move_client = None  # ROS Action 客户端
        self.init_ros()

        self.depth_intrinsics = np.array(
            [[612.287415, 0, 638.468506], [0, 612.246033, 359.933319], [0, 0, 1]],
            dtype=np.float32,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.init_ui()
        self.init_thread()

    def init_ros(self):
        """初始化ROS节点和 Action 客户端"""
        try:
            # 检查ROS Master是否运行
            if not rospy.is_shutdown():
                # 初始化ROS节点，disable_signals=True防止Qt和ROS信号冲突
                rospy.init_node(
                    "tomato_picking_gui", anonymous=True, disable_signals=True
                )

                # 等待 simple_move_arm action 可用（超时5秒）
                self.simple_move_client = actionlib.SimpleActionClient(
                    "/simple_move_arm", SimpleMoveArmAction
                )
                if not self.simple_move_client.wait_for_server(rospy.Duration(5.0)):
                    raise rospy.ROSException("/simple_move_arm action 不可用")

                self.ros_available = True
                print("[ROS] 初始化成功，已连接到 /simple_move_arm action")

        except rospy.ROSException as e:
            print(f"[ROS] 初始化失败: {e}")
            self.ros_available = False
        except Exception as e:
            print(f"[ROS] 未知错误: {e}")
            self.ros_available = False

    def flange_point_to_tcp_point(self, x_flange, y_flange, z_flange):
        pt_flange = PoseStamped()
        pt_flange.header.frame_id = "link6"
        pt_flange.header.stamp = rospy.Time(0)
        pt_flange.pose.position.x = x_flange
        pt_flange.pose.position.y = y_flange
        pt_flange.pose.position.z = z_flange
        pt_flange.pose.orientation.x = 0.0
        pt_flange.pose.orientation.y = 0.0
        pt_flange.pose.orientation.z = 0.0
        pt_flange.pose.orientation.w = 1.0

        pt_tcp = self.tf_buffer.transform(pt_flange, "link_tcp", rospy.Duration(0.2))
        return pt_tcp.pose.position.x, pt_tcp.pose.position.y, pt_tcp.pose.position.z

    def send_arm_command(self, x, y, z):
        if not self.ros_available:
            print("[ROS] 未连接到ROS，跳过发送指令")
            return False

        try:
            goal = SimpleMoveArmGoal()
            goal.command_type = SimpleMoveArmGoal.MOVE_TARGET_IN_EEF_FRAME
            goal.target_type = SimpleMoveArmGoal.TARGET_POSE

            goal.x = [x]
            goal.y = [y]
            goal.z = [z]
            goal.roll = [0.0]
            goal.pitch = [0.0]
            goal.yaw = [0.0]

            goal.joint_names = []
            goal.joints = []
            goal.values = []

            print(f"[ROS] 发送机械臂指令: x={x:.4f}, y={y:.4f}, z={z:.4f}")
            self.simple_move_client.send_goal(goal)
            finished = self.simple_move_client.wait_for_result(rospy.Duration(10.0))
            if not finished:
                self.simple_move_client.cancel_goal()
                print("[ROS] action 超时，已取消")
                return False

            result = self.simple_move_client.get_result()
            print(f"[ROS] action 返回: {result!r}")

            if result is not None and hasattr(result, "success"):
                print(f"[ROS] 业务成功标志: {result.success}")
                return bool(result.success)

            return False

        except Exception as e:
            print(f"[ROS] action 调用失败: {e}")
            return False

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        self.video_label = ImageDisplayWidget()
        self.video_label.selection_changed.connect(self.on_roi_selected)
        main_layout.addWidget(self.video_label, stretch=3)

        panel_layout = QVBoxLayout()
        main_layout.addLayout(panel_layout, stretch=1)

        self.info_label = QLabel(
            "系统初始化中...\n\n操作说明：\n1. 左键：添加轮廓点\n2. 右键：撤销上一个点\n3. 双击：闭合多边形"
        )
        self.info_label.setWordWrap(True)
        self.info_label.setFont(QFont("Arial", 10))
        panel_layout.addWidget(self.info_label)

        self.btn_reset = QPushButton("重置选择")
        self.btn_reset.clicked.connect(self.video_label.clear_selection)
        self.btn_reset.setMinimumHeight(40)
        self.btn_reset.setStyleSheet(
            "background-color: #3498db; color: white; font-size:14px; font-weight:bold;"
        )
        panel_layout.addWidget(self.btn_reset)

        # ================= 新增：发送机械臂指令按钮 =================
        self.btn_send_cmd = QPushButton("发送机械臂指令")
        self.btn_send_cmd.clicked.connect(self.manual_send_command)
        self.btn_send_cmd.setMinimumHeight(40)
        self.btn_send_cmd.setStyleSheet(
            "background-color: #27ae60; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_send_cmd.setEnabled(False)  # 默认禁用（有坐标后启用）
        panel_layout.addWidget(self.btn_send_cmd)

        panel_layout.addStretch()

    def manual_send_command(self):
        """手动触发发送机械臂指令"""
        if self.target_arm_coord:
            x, y, z = self.target_arm_coord
            self.send_arm_command(x, y, z)

    def init_thread(self):
        self.worker = WorkerThread()
        self.worker.frame_ready.connect(self.video_label.set_image)
        self.worker.data_ready.connect(self.receive_data)
        self.worker.log_signal.connect(lambda s: print(s))
        self.worker.intrinsics_ready.connect(self.on_intrinsics_received)
        self.worker.lrm_data_ready.connect(self.on_lrm_data_received)
        self.worker.start()

    def on_intrinsics_received(self, K):
        self.depth_intrinsics = K

    def receive_data(self, depth_data, scale, color_size):
        self.current_depth_data = depth_data
        self.current_depth_scale = scale
        self.current_color_size = color_size

    def on_lrm_data_received(self, dist_mm, dist_m):
        self.current_lrm_mm = dist_mm
        self.current_lrm_m = dist_m

    def get_valid_depth_around(self, depth_img, u, v, stem_mask, kernel_size=15):
        h, w = depth_img.shape
        if stem_mask.shape[:2] != (h, w):
            stem_mask = cv2.resize(stem_mask, (w, h), interpolation=cv2.INTER_NEAREST)

        half_k = kernel_size // 2
        start_x, end_x = max(0, u - half_k), min(w, u + half_k + 1)
        start_y, end_y = max(0, v - half_k), min(h, v + half_k + 1)

        depth_roi = depth_img[start_y:end_y, start_x:end_x]
        mask_roi = stem_mask[start_y:end_y, start_x:end_x]

        valid_mask = (mask_roi > 0) & (depth_roi > 0)
        valid_depths = depth_roi[valid_mask]

        if len(valid_depths) > 0:
            return np.median(valid_depths)

        full_mask_valid = (stem_mask > 0) & (depth_img > 0)
        full_valid_depths = depth_img[full_mask_valid]
        return np.median(full_valid_depths) if len(full_valid_depths) > 0 else 0

    def on_roi_selected(self, cutting_pixel, mask):
        if self.current_depth_data is None or mask is None:
            self.btn_send_cmd.setEnabled(False)
            return

        self.current_cutting_pixel = cutting_pixel
        u, v = cutting_pixel
        color_w, color_h = self.current_color_size
        depth_h, depth_w = self.current_depth_data.shape

        # 坐标映射
        du = int(np.clip(u * depth_w / color_w, 0, depth_w - 1))
        dv = int(np.clip(v * depth_h / color_h, 0, depth_h - 1))
        depth_mask = cv2.resize(
            mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST
        )
        depth_raw = self.get_valid_depth_around(
            self.current_depth_data, du, dv, depth_mask
        )

        # 0.4m以内强制使用LRM
        use_lrm = False
        if LRM_VALID_MIN <= self.current_lrm_mm <= LRM_VALID_MAX:
            depth_raw = self.current_lrm_mm
            use_lrm = True
        elif depth_raw == 0:
            self.info_label.setText(f"⚠️ 深度无效，请调整位置")
            self.target_arm_coord = None
            self.btn_send_cmd.setEnabled(False)
            return

        depth_m = depth_raw / 1000
        x_cam, y_cam, z_cam = depth_to_pointcloud(
            du, dv, depth_m, self.depth_intrinsics
        )
        # 进行手眼标定转换，得到法兰坐标
        x_flange, y_flange, z_flange = hand_eye_calibration(x_cam, y_cam, z_cam)

        # 将法兰坐标转换为TCP坐标
        x_arm, y_arm, z_arm = self.flange_point_to_tcp_point(
            x_flange, y_flange, z_flange
        )

        # 存储目标坐标并启用发送按钮
        self.target_arm_coord = (x_arm, y_arm, z_arm)
        self.btn_send_cmd.setEnabled(True)

        # ROS状态显示
        ros_status = "✅ ROS已连接" if self.ros_available else "❌ ROS未连接"
        lrm_note = " <font color='red'>(LRM激光补盲)</font>" if use_lrm else ""

        self.info_label.setText(f"""
        <h3>✅ 目标锁定{lrm_note}</h3>
        <p><b>ROS状态:</b> {ros_status}</p>
        <p><b>切割像素:</b> ({u}, {v})</p>
        <p><b>深度:</b> {depth_m:.3f} m</p>
        <hr>
        <p><b style="color: #9b59b6;">机械臂坐标:</b></p>
        <p><b>X:</b> {x_arm:.4f}</p>
        <p><b>Y:</b> {y_arm:.4f}</p>
        <p><b>Z:</b> {z_arm:.4f}</p>
        """)

        # 可选：自动发送指令（注释掉则仅手动发送）
        # self.send_arm_command(x_arm, y_arm, z_arm)

    def closeEvent(self, event):
        if hasattr(self, "worker"):
            self.worker.stop()
        # 关闭ROS节点
        if self.ros_available:
            rospy.signal_shutdown("GUI Closed")
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
