#! /usr/bin/env python
import sys
from dataclasses import dataclass
from typing import Any, Optional, Tuple, Union

import cv2
import numpy as np

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from piper_msgs2.msg import PickTaskAction, PickTaskGoal
from pyorbbecsdk import (
    OBAlignMode,
    OBFormat,
    OBPropertyID,
    OBSensorType,
    Config,
    Pipeline,
    VideoFrame,
)
from PyQt5.QtCore import Qt, QPoint, pyqtSignal, QThread
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class DepthConfig:
    min_mm: int = 100
    max_mm: int = 10000
    kernel_size: int = 15


@dataclass(frozen=True)
class LRMConfig:
    valid_min_mm: int = 1
    valid_max_mm: int = 400
    enable_property: int = OBPropertyID.OB_PROP_LDP_BOOL
    measure_property: int = OBPropertyID.OB_PROP_LDP_MEASURE_DISTANCE_INT


@dataclass(frozen=True)
class CameraConfig:
    color_width: int = 1280
    color_height: int = 720
    color_fps: int = 30
    depth_width: int = 848
    depth_height: int = 480
    depth_fps: int = 30
    align_mode: int = OBAlignMode.SW_MODE


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "tomato_picking_gui"
    action_name: str = "/pick_action"
    pick_group_name: str = "gui_pick"
    flange_frame: str = "link6"
    tcp_frame: str = "link_tcp"
    place_frame: str = "base_link"
    action_wait_sec: float = 5.0
    result_wait_sec: float = 10.0
    tf_wait_sec: float = 0.2


@dataclass(frozen=True)
class UiConfig:
    window_title: str = "番茄"
    window_x: int = 100
    window_y: int = 100
    window_w: int = 1280
    window_h: int = 720
    image_min_w: int = 640
    image_min_h: int = 480
    info_font_name: str = "Arial"
    info_font_size: int = 10


@dataclass(frozen=True)
class HandEyeConfig:
    # ^flange T_cam
    T_cam_to_flange: Tuple[Tuple[float, float, float, float], ...] = (
        (0.103597, 0.960173, 0.259493, -0.059774),
        (-0.994337, 0.1062, 0.004006, 0.011284),
        (-0.023712, -0.258438, 0.965737, 0.107886),
        (0.0, 0.0, 0.0, 1.0),
    )


@dataclass(frozen=True)
class IntrinsicsConfig:
    fx: float = 612.287415
    fy: float = 612.246033
    cx: float = 638.468506
    cy: float = 359.933319


DEPTH_CFG = DepthConfig()
LRM_CFG = LRMConfig()
CAMERA_CFG = CameraConfig()
ROS_CFG = RosConfig()
UI_CFG = UiConfig()
HAND_EYE_CFG = HandEyeConfig()
INTRINSICS_CFG = IntrinsicsConfig()

DEFAULT_DEPTH_INTRINSICS = np.array(
    [
        [INTRINSICS_CFG.fx, 0.0, INTRINSICS_CFG.cx],
        [0.0, INTRINSICS_CFG.fy, INTRINSICS_CFG.cy],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

T_CAM_TO_FLANGE = np.array(HAND_EYE_CFG.T_cam_to_flange, dtype=np.float64)


def frame_to_bgr_image(frame: VideoFrame) -> Union[Optional[np.ndarray], Any]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if color_format == OBFormat.RGB:
        image = np.resize(data, (height, width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if color_format == OBFormat.YUYV:
        image = np.resize(data, (height, width, 2))
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
    if color_format == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    return None


def cam_point_to_flange_point(
    x_cam: float, y_cam: float, z_cam: float
) -> Tuple[float, float, float]:
    p_cam_homo = np.array([[x_cam], [y_cam], [z_cam], [1.0]], dtype=np.float64)
    p_flange_homo = T_CAM_TO_FLANGE @ p_cam_homo
    return -p_flange_homo[0, 0], -p_flange_homo[1, 0], p_flange_homo[2, 0]


def get_mask_center(mask: np.ndarray) -> Optional[Tuple[float, float]]:
    M = cv2.moments(mask.astype(np.uint8))
    if M["m00"] == 0:
        return None
    cx = float(M["m10"] / M["m00"])
    cy = float(M["m01"] / M["m00"])
    return cx, cy


def find_cutting_point(
    stem_mask: np.ndarray,
    stem_center: Tuple[float, float],
    original_image: np.ndarray,
) -> Tuple[float, float]:
    del original_image

    binary_mask = (stem_mask > 0.5).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) == 0:
        return stem_center

    largest_contour = max(contours, key=cv2.contourArea)
    epsilon = 0.001 * cv2.arcLength(largest_contour, True)
    largest_contour = cv2.approxPolyDP(largest_contour, epsilon, True)
    hull = cv2.convexHull(largest_contour, returnPoints=False)

    try:
        defects = cv2.convexityDefects(largest_contour, hull)
        if defects is None:
            return stem_center

        cutting_points = []
        for i in range(defects.shape[0]):
            _, _, f, _ = defects[i, 0]
            far_point = tuple(largest_contour[f][0])
            cutting_points.append(far_point)

        if not cutting_points:
            return stem_center

        return min(
            cutting_points,
            key=lambda p: np.sqrt(
                (p[0] - stem_center[0]) ** 2 + (p[1] - stem_center[1]) ** 2
            ),
        )
    except cv2.error:
        return stem_center


def depth_to_pointcloud(
    u: int,
    v: int,
    depth_m: float,
    depth_intrinsics: np.ndarray,
) -> Tuple[float, float, float]:
    fx, fy = depth_intrinsics[0, 0], depth_intrinsics[1, 1]
    cx, cy = depth_intrinsics[0, 2], depth_intrinsics[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return x, y, z


class ImageDisplayWidget(QLabel):
    selection_changed = pyqtSignal(tuple, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(UI_CFG.image_min_w, UI_CFG.image_min_h)
        self.setStyleSheet("background-color: black;")
        self.setAlignment(Qt.AlignCenter)

        self._pixmap = None
        self._latest_cv_image = None
        self._screen_points = []
        self._cutting_point_screen = None
        self._is_polygon_closed = False

    def set_image(self, cv_img: np.ndarray) -> None:
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

    def _screen_to_img(self, pt: QPoint) -> Tuple[float, float]:
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
        return img_x, img_y

    def _img_to_screen(self, x: float, y: float) -> QPoint:
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
        elif event.button() == Qt.RightButton and len(self._screen_points) > 0:
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

    def _update_mask_and_calculate(self) -> None:
        if len(self._screen_points) < 3 or self._latest_cv_image is None:
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
        if cutting_point is None:
            return

        self._cutting_point_screen = self._img_to_screen(
            cutting_point[0], cutting_point[1]
        )
        self.update()
        self.selection_changed.emit(cutting_point, mask)

    def clear_selection(self) -> None:
        self._screen_points = []
        self._cutting_point_screen = None
        self._is_polygon_closed = False
        self.update()


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
        self.device = None
        self._has_sent_intrinsics = False

    def _select_color_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                CAMERA_CFG.color_width,
                CAMERA_CFG.color_height,
                OBFormat.RGB,
                CAMERA_CFG.color_fps,
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _select_depth_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                CAMERA_CFG.depth_width,
                CAMERA_CFG.depth_height,
                OBFormat.Y16,
                CAMERA_CFG.depth_fps,
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _emit_intrinsics_once(self, depth_frame) -> None:
        if self._has_sent_intrinsics:
            return
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
            self.log_signal.emit("✅ 内参读取成功")
            self._has_sent_intrinsics = True
        except Exception:
            pass

    def _read_lrm_data(self) -> None:
        if not self.device:
            return
        try:
            lrm_dist_mm = self.device.get_int_property(LRM_CFG.measure_property)
            self.lrm_data_ready.emit(lrm_dist_mm, lrm_dist_mm / 1000.0)
        except Exception:
            pass

    def run(self):
        self._running = True
        self._has_sent_intrinsics = False

        try:
            self.pipeline = Pipeline()
            self.device = self.pipeline.get_device()
            config = Config()

            color_profile = self._select_color_profile(
                self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            )
            depth_profile = self._select_depth_profile(
                self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            )

            config.enable_stream(color_profile)
            config.enable_stream(depth_profile)
            config.set_align_mode(CAMERA_CFG.align_mode)
            self.pipeline.enable_frame_sync()
            self.pipeline.start(config)

            try:
                self.device.set_bool_property(LRM_CFG.enable_property, True)
                self.log_signal.emit("✅ LRM 激光补盲模块已开启 (0.001m~0.4m强制使用)")
            except Exception as e:
                self.log_signal.emit(f"⚠️  LRM 开启失败: {e}")

            self.log_signal.emit(
                f"✅ 相机启动成功 (深度分辨率: {depth_profile.get_width()}x{depth_profile.get_height()}, 视差: 128)"
            )
            self.log_signal.emit("⚠️  注意: 0.4m以内深度将优先使用LRM数据")

            while self._running:
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame is None or depth_frame is None:
                    continue

                self._emit_intrinsics_once(depth_frame)

                color_image = frame_to_bgr_image(color_frame)
                if color_image is None:
                    continue

                depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                depth_data = depth_data.reshape(
                    (depth_frame.get_height(), depth_frame.get_width())
                )
                depth_data = cv2.medianBlur(depth_data, 5)
                depth_data = np.where(
                    (depth_data > DEPTH_CFG.min_mm) & (depth_data < DEPTH_CFG.max_mm),
                    depth_data,
                    0,
                )

                self.frame_ready.emit(color_image)
                self.data_ready.emit(
                    depth_data,
                    depth_frame.get_depth_scale(),
                    (color_frame.get_width(), color_frame.get_height()),
                )
                self._read_lrm_data()

        except Exception as e:
            self.log_signal.emit(f"❌ 工作线程错误: {e}")
        finally:
            if self.pipeline:
                self.pipeline.stop()

    def stop(self) -> None:
        self._running = False
        self.wait()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(UI_CFG.window_title)
        self.setGeometry(
            UI_CFG.window_x,
            UI_CFG.window_y,
            UI_CFG.window_w,
            UI_CFG.window_h,
        )

        self.current_depth_data = None
        self.current_depth_scale = None
        self.current_color_size = None
        self.current_lrm_mm = 0
        self.current_lrm_m = 0.0
        self.current_cutting_pixel = None

        self.target_tcp_coord = None
        self.last_cam_coord = None
        self.last_flange_coord = None

        self.ros_available = False
        self.pick_client = None
        self.depth_intrinsics = DEFAULT_DEPTH_INTRINSICS.copy()
        self.last_request_type = None

        # 先初始化 ROS
        self.init_ros()

        # 再创建 TF
        self.tf_buffer = None
        self.tf_listener = None
        if self.ros_available:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.init_ui()
        self.init_thread()

    def init_ros(self) -> None:
        try:
            if not rospy.is_shutdown():
                rospy.init_node(ROS_CFG.node_name, anonymous=True, disable_signals=True)
                self.pick_client = actionlib.SimpleActionClient(
                    ROS_CFG.action_name, PickTaskAction
                )
                if not self.pick_client.wait_for_server(
                    rospy.Duration(ROS_CFG.action_wait_sec)
                ):
                    raise rospy.ROSException(f"{ROS_CFG.action_name} action 不可用")

                self.ros_available = True
                print(f"[ROS] 初始化成功，已连接到 {ROS_CFG.action_name}")
        except rospy.ROSException as e:
            print(f"[ROS] 初始化失败: {e}")
            self.ros_available = False
        except Exception as e:
            print(f"[ROS] 未知错误: {e}")
            self.ros_available = False

    def flange_point_to_tcp_point(
        self,
        x_flange: float,
        y_flange: float,
        z_flange: float,
    ) -> Tuple[float, float, float]:
        if not self.ros_available or self.tf_buffer is None:
            raise RuntimeError("ROS/TF 未初始化，无法执行 flange -> tcp 变换")

        pt_flange = PoseStamped()
        pt_flange.header.frame_id = ROS_CFG.flange_frame
        pt_flange.header.stamp = rospy.Time(0)
        pt_flange.pose.position.x = x_flange
        pt_flange.pose.position.y = y_flange
        pt_flange.pose.position.z = z_flange
        pt_flange.pose.orientation.x = 0.0
        pt_flange.pose.orientation.y = 0.0
        pt_flange.pose.orientation.z = 0.0
        pt_flange.pose.orientation.w = 1.0

        pt_tcp = self.tf_buffer.transform(
            pt_flange, ROS_CFG.tcp_frame, rospy.Duration(ROS_CFG.tf_wait_sec)
        )
        return pt_tcp.pose.position.x, pt_tcp.pose.position.y, pt_tcp.pose.position.z

    def send_arm_command(self, x: float, y: float, z: float) -> bool:
        del x, y, z
        self.upsert_current_task()
        return True

    def init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        self.video_label = ImageDisplayWidget()
        self.video_label.selection_changed.connect(self.on_roi_selected)
        main_layout.addWidget(self.video_label, stretch=3)

        panel_layout = QHBoxLayout()
        main_layout.addLayout(panel_layout, stretch=2)

        info_column = QVBoxLayout()
        control_column = QVBoxLayout()
        panel_layout.addLayout(info_column, stretch=1)
        panel_layout.addLayout(control_column, stretch=1)

        info_group = QGroupBox("目标锁定 / 坐标信息")
        info_group_layout = QVBoxLayout(info_group)
        self.info_label = QLabel(
            "系统初始化中...\n\n操作说明：\n1. 左键：添加轮廓点\n2. 右键：撤销上一个点\n3. 双击：闭合多边形"
        )
        self.info_label.setWordWrap(True)
        self.info_label.setFont(QFont(UI_CFG.info_font_name, UI_CFG.info_font_size))
        self.info_label.setTextFormat(Qt.RichText)
        self.info_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        info_group_layout.addWidget(self.info_label)
        info_group.setMinimumWidth(300)
        info_column.addWidget(info_group)
        info_column.addStretch()

        self.task_status_label = QLabel("采摘任务状态：未发送")
        self.task_status_label.setWordWrap(True)
        self.task_status_label.setFont(
            QFont(UI_CFG.info_font_name, UI_CFG.info_font_size)
        )
        control_column.addWidget(self.task_status_label)

        task_group = QGroupBox("任务组配置")
        task_layout = QFormLayout(task_group)
        self.group_name_edit = QLineEdit(ROS_CFG.pick_group_name)
        self.task_id_edit = QLineEdit("1")
        self.task_desc_edit = QLineEdit("GUI采摘任务")
        self.task_type_box = QComboBox()
        self.task_type_box.addItem("PICK", PickTaskGoal.TASK_PICK)
        self.task_type_box.addItem("MOVE_ONLY", PickTaskGoal.TASK_MOVE_ONLY)
        self.retry_times_edit = QLineEdit("0")
        self.chk_use_eef = QCheckBox("执行夹爪动作")
        self.chk_use_eef.setChecked(True)
        self.chk_go_home_after_finish = QCheckBox("任务完成后回到初始位")
        self.chk_go_home_after_finish.setChecked(True)
        self.chk_go_safe_after_cancel = QCheckBox("取消后自动回安全位")
        self.chk_go_safe_after_cancel.setChecked(True)
        task_layout.addRow("任务组", self.group_name_edit)
        task_layout.addRow("任务 ID", self.task_id_edit)
        task_layout.addRow("任务类型", self.task_type_box)
        task_layout.addRow("描述", self.task_desc_edit)
        task_layout.addRow("重试次数", self.retry_times_edit)
        task_layout.addRow(self.chk_use_eef)
        task_layout.addRow(self.chk_go_home_after_finish)
        task_layout.addRow(self.chk_go_safe_after_cancel)
        control_column.addWidget(task_group)

        place_group = QGroupBox("放置区配置（base_link）")
        place_layout = QFormLayout(place_group)
        self.chk_use_place_pose = QCheckBox("启用放置动作")
        self.chk_use_place_pose.setChecked(True)
        self.place_target_type_box = QComboBox()
        self.place_target_type_box.addItem("Point(base_link)", PickTaskGoal.PLACE_TARGET_POINT)
        self.place_target_type_box.addItem("Pose(base_link)", PickTaskGoal.PLACE_TARGET_POSE)
        self.place_x_edit = QLineEdit("0.25")
        self.place_y_edit = QLineEdit("0.00")
        self.place_z_edit = QLineEdit("0.10")
        self.place_roll_edit = QLineEdit("0.00")
        self.place_pitch_edit = QLineEdit("0.00")
        self.place_yaw_edit = QLineEdit("0.00")
        place_layout.addRow(self.chk_use_place_pose)
        place_layout.addRow("放置目标类型", self.place_target_type_box)
        place_layout.addRow("X", self.place_x_edit)
        place_layout.addRow("Y", self.place_y_edit)
        place_layout.addRow("Z", self.place_z_edit)
        place_layout.addRow("Roll", self.place_roll_edit)
        place_layout.addRow("Pitch", self.place_pitch_edit)
        place_layout.addRow("Yaw", self.place_yaw_edit)
        control_column.addWidget(place_group)

        self.btn_reset = QPushButton("重置选择")
        self.btn_reset.clicked.connect(self.video_label.clear_selection)
        self.btn_reset.setMinimumHeight(40)
        self.btn_reset.setStyleSheet(
            "background-color: #3498db; color: white; font-size:14px; font-weight:bold;"
        )
        control_column.addWidget(self.btn_reset)

        self.btn_upsert_task = QPushButton("写入 / 更新当前任务")
        self.btn_upsert_task.clicked.connect(self.manual_send_command)
        self.btn_upsert_task.setMinimumHeight(40)
        self.btn_upsert_task.setStyleSheet(
            "background-color: #27ae60; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_upsert_task.setEnabled(False)
        control_column.addWidget(self.btn_upsert_task)

        self.btn_execute_group = QPushButton("执行当前任务组")
        self.btn_execute_group.clicked.connect(self.execute_current_group)
        self.btn_execute_group.setMinimumHeight(40)
        self.btn_execute_group.setStyleSheet(
            "background-color: #8e44ad; color: white; font-size:14px; font-weight:bold;"
        )
        self.btn_execute_group.setEnabled(self.ros_available)
        control_column.addWidget(self.btn_execute_group)

        self.btn_cancel_cmd = QPushButton("取消当前执行")
        self.btn_cancel_cmd.clicked.connect(self.cancel_pick_task)
        self.btn_cancel_cmd.setMinimumHeight(40)
        self.btn_cancel_cmd.setStyleSheet(
            "background-color: #c0392b; color: white; font-size:14px; font-weight:bold;"
        )
        control_column.addWidget(self.btn_cancel_cmd)

        control_column.addStretch()

    def init_thread(self) -> None:
        self.worker = WorkerThread()
        self.worker.frame_ready.connect(self.video_label.set_image)
        self.worker.data_ready.connect(self.receive_data)
        self.worker.log_signal.connect(lambda s: print(s))
        self.worker.intrinsics_ready.connect(self.on_intrinsics_received)
        self.worker.lrm_data_ready.connect(self.on_lrm_data_received)
        self.worker.start()

    def on_intrinsics_received(self, K: np.ndarray) -> None:
        self.depth_intrinsics = K

    def receive_data(self, depth_data, scale, color_size) -> None:
        self.current_depth_data = depth_data
        self.current_depth_scale = scale
        self.current_color_size = color_size

    def on_lrm_data_received(self, dist_mm: int, dist_m: float) -> None:
        self.current_lrm_mm = dist_mm
        self.current_lrm_m = dist_m

    def get_valid_depth_around(
        self,
        depth_img: np.ndarray,
        u: int,
        v: int,
        stem_mask: np.ndarray,
        kernel_size: int = DEPTH_CFG.kernel_size,
    ) -> float:
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
            return float(np.median(valid_depths))

        full_mask_valid = (stem_mask > 0) & (depth_img > 0)
        full_valid_depths = depth_img[full_mask_valid]
        return (
            float(np.median(full_valid_depths)) if len(full_valid_depths) > 0 else 0.0
        )

    def _get_depth_value_mm(self, depth_raw: float) -> Tuple[float, bool]:
        use_lrm = False
        if LRM_CFG.valid_min_mm <= self.current_lrm_mm <= LRM_CFG.valid_max_mm:
            depth_raw = float(self.current_lrm_mm)
            use_lrm = True
        return depth_raw, use_lrm

    def _update_info_label(
        self,
        u: float,
        v: float,
        depth_m: float,
        use_lrm: bool,
        cam_xyz: Tuple[float, float, float],
        flange_xyz: Tuple[float, float, float],
        tcp_xyz: Tuple[float, float, float],
    ) -> None:
        ros_status = "✅ ROS已连接" if self.ros_available else "❌ ROS未连接"
        lrm_note = " <font color='red'>(LRM激光补盲)</font>" if use_lrm else ""
        x_cam, y_cam, z_cam = cam_xyz
        x_flange, y_flange, z_flange = flange_xyz
        x_tcp, y_tcp, z_tcp = tcp_xyz

        self.info_label.setText(f"""
            <h3>✅ 目标锁定{lrm_note}</h3>
            <p><b>ROS状态:</b> {ros_status}</p>
            <p><b>切割像素:</b> ({u}, {v})</p>
            <p><b>深度:</b> {depth_m:.3f} m</p>
            <hr>
            <p><b>Camera坐标:</b></p>
            <p><b>X:</b> {x_cam:.4f}</p>
            <p><b>Y:</b> {y_cam:.4f}</p>
            <p><b>Z:</b> {z_cam:.4f}</p>
            <hr>
            <p><b>Flange坐标:</b></p>
            <p><b>X:</b> {x_flange:.4f}</p>
            <p><b>Y:</b> {y_flange:.4f}</p>
            <p><b>Z:</b> {z_flange:.4f}</p>
            <hr>
            <p><b style='color: #9b59b6;'>TCP相对坐标</b></p>
            <p style='color: #9b59b6;'>将以 Point(link_tcp) 写入任务层，并在设置任务时冻结到底座坐标系。</p>
            <p><b>X:</b> {x_tcp:.4f}</p>
            <p><b>Y:</b> {y_tcp:.4f}</p>
            <p><b>Z:</b> {z_tcp:.4f}</p>
            """)

    def manual_send_command(self) -> None:
        self.upsert_current_task()

    def _parse_float(self, edit: QLineEdit, default: float = 0.0) -> float:
        try:
            return float(edit.text().strip())
        except Exception:
            return default

    def _parse_int(self, edit: QLineEdit, default: int = 0) -> int:
        try:
            return int(edit.text().strip())
        except Exception:
            return default

    def _build_pose_stamped(
        self, frame_id: str, x: float, y: float, z: float
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now() if self.ros_available else rospy.Time(0)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        return pose

    def _fill_point_msg(self, point_msg, x: float, y: float, z: float) -> None:
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z

    def _fill_pose_msg(
        self,
        pose_msg,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

    def _send_goal(self, goal: PickTaskGoal) -> None:
        self.pick_client.send_goal(
            goal,
            done_cb=self._on_pick_done,
            active_cb=self._on_pick_active,
            feedback_cb=self._on_pick_feedback,
        )

    def upsert_current_task(self) -> None:
        if not self.ros_available or self.pick_client is None:
            self.task_status_label.setText("采摘任务状态：ROS 未连接")
            return

        if self.target_tcp_coord is None:
            self.task_status_label.setText("采摘任务状态：请先选择目标")
            return

        x, y, z = self.target_tcp_coord
        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.UPSERT_TASK
        goal.group_name = self.group_name_edit.text().strip() or ROS_CFG.pick_group_name
        goal.id = max(0, self._parse_int(self.task_id_edit, 1))
        goal.task_type = int(self.task_type_box.currentData())
        goal.description = self.task_desc_edit.text().strip() or "GUI采摘任务"
        goal.target_type = PickTaskGoal.TARGET_POINT
        self._fill_point_msg(goal.target_point, x, y, z)
        goal.target_frame_id = ROS_CFG.tcp_frame

        goal.use_place_pose = self.chk_use_place_pose.isChecked()
        goal.use_eef = self.chk_use_eef.isChecked()
        goal.go_home_after_finish = self.chk_go_home_after_finish.isChecked()
        goal.go_safe_after_cancel = self.chk_go_safe_after_cancel.isChecked()
        goal.retry_times = max(0, self._parse_int(self.retry_times_edit, 0))

        if goal.task_type == PickTaskGoal.TASK_PICK and goal.use_place_pose:
            place_target_type = int(self.place_target_type_box.currentData())
            goal.place_target_type = place_target_type
            goal.place_frame_id = ROS_CFG.place_frame

            px = self._parse_float(self.place_x_edit, 0.0)
            py = self._parse_float(self.place_y_edit, 0.0)
            pz = self._parse_float(self.place_z_edit, 0.0)
            proll = self._parse_float(self.place_roll_edit, 0.0)
            ppitch = self._parse_float(self.place_pitch_edit, 0.0)
            pyaw = self._parse_float(self.place_yaw_edit, 0.0)

            if place_target_type == PickTaskGoal.PLACE_TARGET_POSE:
                self._fill_pose_msg(goal.place_pose, px, py, pz, proll, ppitch, pyaw)
            else:
                goal.place_target_type = PickTaskGoal.PLACE_TARGET_POINT
                self._fill_point_msg(goal.place_point, px, py, pz)
        else:
            goal.use_place_pose = False
            goal.place_target_type = PickTaskGoal.PLACE_TARGET_NONE

        self.last_request_type = goal.request_type
        self.task_status_label.setText(
            f"采摘任务状态：正在写入任务组 '{goal.group_name}'，任务 ID={goal.id}，目标=({x:.3f}, {y:.3f}, {z:.3f})"
        )
        self._send_goal(goal)

    def execute_current_group(self) -> None:
        if not self.ros_available or self.pick_client is None:
            self.task_status_label.setText("采摘任务状态：ROS 未连接")
            return

        goal = PickTaskGoal()
        goal.request_type = PickTaskGoal.EXECUTE_TASK_GROUP
        goal.group_name = self.group_name_edit.text().strip() or ROS_CFG.pick_group_name
        self.last_request_type = goal.request_type
        self.task_status_label.setText(
            f"采摘任务状态：已发送任务组执行请求，任务组='{goal.group_name}'"
        )
        self._send_goal(goal)

    def send_pick_task(self) -> None:
        self.upsert_current_task()

    def cancel_pick_task(self) -> None:
        if self.ros_available and self.pick_client is not None:
            self.pick_client.cancel_goal()
            self.task_status_label.setText("采摘任务状态：已请求取消")

    def _on_pick_active(self) -> None:
        if self.last_request_type == PickTaskGoal.UPSERT_TASK:
            self.task_status_label.setText("采摘任务状态：写入任务中")
        else:
            self.task_status_label.setText("采摘任务状态：执行任务组中")

    def _on_pick_feedback(self, feedback) -> None:
        try:
            if self.last_request_type == PickTaskGoal.EXECUTE_TASK_GROUP:
                self.task_status_label.setText(
                    f"采摘任务状态：{feedback.stage_text} | step {feedback.current_step_index}/{feedback.total_steps}"
                )
        except Exception:
            pass

    def _on_pick_done(self, state, result) -> None:
        try:
            msg = getattr(result, "message", "")
            if getattr(result, "success", False):
                if self.last_request_type == PickTaskGoal.UPSERT_TASK:
                    self.task_status_label.setText(f"采摘任务状态：任务写入成功 - {msg}")
                else:
                    self.task_status_label.setText(f"采摘任务状态：任务组执行完成 - {msg}")
            elif getattr(result, "canceled", False):
                self.task_status_label.setText(f"采摘任务状态：已取消 - {msg}")
            else:
                if self.last_request_type == PickTaskGoal.UPSERT_TASK:
                    self.task_status_label.setText(f"采摘任务状态：任务写入失败 - {msg}")
                else:
                    self.task_status_label.setText(f"采摘任务状态：任务组执行失败 - {msg}")
        except Exception:
            self.task_status_label.setText(f"采摘任务状态：结束，state={state}")

    def on_roi_selected(self, cutting_pixel, mask) -> None:
        if (
            self.current_depth_data is None
            or self.current_color_size is None
            or mask is None
        ):
            self.btn_upsert_task.setEnabled(False)
            return

        self.current_cutting_pixel = cutting_pixel
        u, v = cutting_pixel
        color_w, color_h = self.current_color_size
        depth_h, depth_w = self.current_depth_data.shape

        du = int(np.clip(u * depth_w / color_w, 0, depth_w - 1))
        dv = int(np.clip(v * depth_h / color_h, 0, depth_h - 1))

        depth_mask = cv2.resize(
            mask, (depth_w, depth_h), interpolation=cv2.INTER_NEAREST
        )
        depth_raw = self.get_valid_depth_around(
            self.current_depth_data, du, dv, depth_mask
        )
        depth_raw, use_lrm = self._get_depth_value_mm(depth_raw)

        if depth_raw == 0:
            self.info_label.setText("⚠️ 深度无效，请调整位置")
            self.target_tcp_coord = None
            self.btn_upsert_task.setEnabled(False)
            return

        depth_m = depth_raw / 1000.0
        cam_xyz = depth_to_pointcloud(du, dv, depth_m, self.depth_intrinsics)
        flange_xyz = cam_point_to_flange_point(*cam_xyz)

        try:
            tcp_xyz = self.flange_point_to_tcp_point(*flange_xyz)
        except Exception as e:
            self.info_label.setText(f"⚠️ TF 变换失败: {e}")
            self.target_tcp_coord = None
            self.btn_upsert_task.setEnabled(False)
            return

        self.last_cam_coord = cam_xyz
        self.last_flange_coord = flange_xyz
        self.target_tcp_coord = tcp_xyz
        self.btn_upsert_task.setEnabled(True)

        self._update_info_label(u, v, depth_m, use_lrm, cam_xyz, flange_xyz, tcp_xyz)

    def closeEvent(self, event):
        if hasattr(self, "worker"):
            self.worker.stop()
        if self.ros_available:
            rospy.signal_shutdown("GUI Closed")
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
