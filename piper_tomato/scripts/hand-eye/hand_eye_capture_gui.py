from __future__ import annotations

import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QImage, QKeySequence, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QShortcut,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from pipper_hand_eye import (
    AUTO_SEARCH_FORMULAS,
    AUTO_SEARCH_METHODS,
    CAMERA_MATRIX,
    CONSTRAINT_MODES,
    CONSTRAINT_MODE_DISABLED,
    DATASET_ROOT,
    DEFAULT_ANNOTATED_NAME,
    DEFAULT_CAMERA_INFO_NAME,
    DEFAULT_DETECTION_NAME,
    DEFAULT_HAND_EYE_METHOD,
    DEFAULT_IMAGE_NAME,
    DEFAULT_POSE_NAME,
    DEFAULT_ROTATION_FORMULA,
    DEFAULT_Z_AXIS_COS_MIN,
    DIST_COEFFS,
    HAND_EYE_METHODS,
    MIN_MAX_DIST_IMPROVEMENT_M,
    MIN_RMS_IMPROVEMENT_M,
    MIN_VALID_SAMPLES,
    MAX_PRUNE_ROUNDS,
    OUTPUT_ROOT,
    ROBOT_POSE_FORMAT,
    ROTATION_FORMULA_CANDIDATES,
    SIGN_ANY,
    SIGN_OPTIONS,
    BoardConfig,
    HandEyeConfig,
    PhysicalConstraintConfig,
    RobotPose,
    calibrate_from_dataset,
    detect_board_pose,
    ensure_dir,
    list_available_dataset_indices,
    parse_excluded_indices,
    pose_to_dict,
    robot_pose_to_base_T_flange,
    save_calibration_outputs,
    write_json,
)

try:
    import rospy
    import tf2_ros
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

    HAS_ROS = True
except Exception:
    rospy = None
    tf2_ros = None
    CvBridge = None
    CameraInfo = object
    Image = object
    HAS_ROS = False


@dataclass(frozen=True)
class UiConfig:
    window_title: str = "Pipper Hand Eye 数据采集与标定"
    window_x: int = 100
    window_y: int = 100
    window_w: int = 1760
    window_h: int = 980
    image_min_w: int = 880
    image_min_h: int = 620
    info_font_name: str = "Arial"
    info_font_size: int = 10


@dataclass(frozen=True)
class RosCameraDefaults:
    color_topic: str = "/piper/camera/orbbec/color/image_raw"
    camera_info_topic: str = "/piper/camera/orbbec/color/camera_info"


@dataclass(frozen=True)
class RosDefaults:
    node_name: str = "handeye_capture_gui_paged"
    base_frame: str = "base_link"
    tool_frame: str = "link6"
    tf_timeout_sec: float = 0.30


UI_CFG = UiConfig()
ROS_CAMERA_DEFAULTS = RosCameraDefaults()
ROS_DEFAULTS = RosDefaults()


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = np.linalg.norm([qx, qy, qz, qw])
    if norm <= 1e-12:
        raise ValueError("四元数范数为 0")
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_euler_xyz_deg(R: np.ndarray) -> Tuple[float, float, float]:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return tuple(float(v) for v in np.degrees([rx, ry, rz]))


class VideoWidget(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(UI_CFG.image_min_w, UI_CFG.image_min_h)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: black;")
        self.setAlignment(Qt.AlignCenter)
        self._pixmap: Optional[QPixmap] = None

    def set_frame(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None:
            return
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        self._pixmap = QPixmap.fromImage(qimg)
        self._refresh()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self) -> None:
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.setPixmap(scaled)


class HandEyeCaptureWindow(QMainWindow):
    camera_info_updated = pyqtSignal(object)
    ros_image_received = pyqtSignal(object)
    log_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle(UI_CFG.window_title)
        self.setGeometry(
            UI_CFG.window_x, UI_CFG.window_y, UI_CFG.window_w, UI_CFG.window_h
        )

        self.current_frame: Optional[np.ndarray] = None
        self.current_frame_wall_time: str = ""
        self.last_detection_ok = False
        self.current_camera_info_payload: Optional[Dict[str, Any]] = None
        self.current_ros_camera_matrix: Optional[np.ndarray] = None
        self.current_ros_dist_coeffs: Optional[np.ndarray] = None
        self.current_camera_info_source: str = "manual_fallback"
        self.current_image_source: str = "no_stream"
        self.color_image_sub = None
        self.color_info_sub = None
        self.bridge = CvBridge() if HAS_ROS and CvBridge is not None else None
        self._closing = False

        self.ros_available = False
        self.tf_buffer = None
        self.tf_listener = None
        self._init_ros()

        self.video_label = VideoWidget()
        self.camera_info_updated.connect(self._apply_camera_info_payload)
        self.ros_image_received.connect(self._apply_ros_frame)
        self.log_message.connect(self._append_log)
        self.init_ui()
        self.refresh_available_indices()
        self._update_stream_status()
        if self.ros_available:
            self.log("当前使用 ROS 相机流")
        else:
            self.log(
                "ROS 未就绪：当前无法接收 color/image_raw，请先启动 roscore 与 piper_camera"
            )

        self._toggle_window_shortcut = QShortcut(QKeySequence("F11"), self)
        self._toggle_window_shortcut.activated.connect(self.toggle_window_state)

    def _init_ros(self) -> None:
        if not HAS_ROS:
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node(
                    ROS_DEFAULTS.node_name, anonymous=True, disable_signals=True
                )
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.color_info_sub = rospy.Subscriber(
                ROS_CAMERA_DEFAULTS.camera_info_topic,
                CameraInfo,
                self.on_color_camera_info,
                queue_size=1,
            )
            self.color_image_sub = rospy.Subscriber(
                ROS_CAMERA_DEFAULTS.color_topic,
                Image,
                self.on_ros_color_image,
                queue_size=1,
                buff_size=2**24,
            )
            self.ros_available = True
        except Exception:
            self.ros_available = False
            self.tf_buffer = None
            self.tf_listener = None
            self.color_info_sub = None
            self.color_image_sub = None

    def toggle_window_state(self) -> None:
        if self.isMaximized() or self.isFullScreen():
            self.showNormal()
        else:
            self.showMaximized()

    def _wrap_tab(self, widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        return scroll

    def _new_line_edit(self, text: str) -> QLineEdit:
        edit = QLineEdit(text)
        edit.setMinimumWidth(120)
        return edit

    def _page_help_text(self, index: int) -> str:
        page_name = (
            self.tabs.tabText(index) if hasattr(self, "tabs") and index >= 0 else ""
        )
        help_map = {
            "项目": "dataset_root：样本目录\noutput_root：标定结果输出目录\n当前保存索引：下一次采样写入的编号\n已存在索引：当前目录下已检测到的样本编号\n采集成功后索引自动 +1：保存成功后自动切换到下一个编号",
            "相机": "当前版本订阅 ROS 图像流\ncolor_topic：采样图像话题\ncamera_info_topic：相机模型话题\n当前后端：当前接收到的图像源\n要求：先启动 roscore 和 piper_camera，再启动本 GUI",
            "棋盘/内参": "board_cols / board_rows：棋盘内角点列数与行数\nsquare_size_m：单个方格边长，单位 m\n优先使用 /piper/camera/orbbec/color/camera_info 同步的相机模型\nfx fy cx cy 与 dist：仅作为离线 fallback 或手工覆盖",
            "ROS/位姿": "ROS：当前 ROS/T F 是否可用\n采样时自动从 TF 读取位姿：保存样本时直接读取当前 TF\nbase_frame / tool_frame：TF 查询使用的坐标系名称\ntf_timeout_sec：TF 查询超时\npose_format：位姿数据的语义格式\nrotation_formula：欧拉角转旋转矩阵的顺序\nx/y/z/rx/ry/rz：手动输入或回显的当前机械臂位姿",
            "标定": "min_index / max_index：参与标定的样本编号范围\nrotation_formula / handeye_method：旋转顺序与手眼方法，可选 AUTO\nexclude_indices：额外排除的样本编号\nmax_reprojection_error_px：角点重投影误差阈值\nauto_prune_outliers：是否自动剔除离群样本\nmin_valid_samples：最少有效样本数\nmax_prune_rounds：最多剔除轮数\nmin_rms_improvement_m / min_max_dist_improvement_m：继续剔除所需的最小改善量",
            "物理约束": "constraint_mode：DISABLED 为关闭，SOFT_PREFER 为排序偏好，HARD_FILTER 为严格过滤\nexpected_tx_sign / expected_ty_sign / expected_tz_sign：期望平移符号\nsign_tolerance_m：符号判断容差\nenable_tx/ty/tz_range：是否启用对应轴的平移范围约束\ntx/ty/tz min/max：对应轴允许范围，单位 m\nrequire_z_axis_same_direction：要求相机 z 轴与 flange z 轴同向\nz_axis_cos_min：z 轴夹角余弦阈值",
        }
        return help_map.get(page_name, "")

    def _update_page_help_text(self, index: int) -> None:
        if hasattr(self, "page_help_label"):
            self.page_help_label.setText(self._page_help_text(index))

    def init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        main_layout.addWidget(self.video_label, stretch=3)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, stretch=2)

        self.page_help_label = QLabel("")
        self.page_help_label.setWordWrap(True)
        self.page_help_label.setFont(
            QFont(UI_CFG.info_font_name, UI_CFG.info_font_size)
        )
        right_layout.addWidget(self.page_help_label)

        self.capture_status_label = QLabel("采集状态：等待 ROS 图像流")
        self.capture_status_label.setWordWrap(True)
        self.capture_status_label.setFont(
            QFont(UI_CFG.info_font_name, UI_CFG.info_font_size)
        )
        right_layout.addWidget(self.capture_status_label)

        self.tabs = QTabWidget()
        right_layout.addWidget(self.tabs, stretch=1)

        self._build_project_tab()
        self._build_camera_tab()
        self._build_board_tab()
        self._build_ros_pose_tab()
        self._build_calibration_tab()
        self._build_constraints_tab()
        self.tabs.currentChanged.connect(self._update_page_help_text)
        self._update_page_help_text(self.tabs.currentIndex())

        actions_group = QGroupBox("操作")
        actions_layout = QVBoxLayout(actions_group)
        self.capture_btn = QPushButton("拍照并保存当前样本")
        self.capture_btn.clicked.connect(self.capture_sample)
        self.calib_btn = QPushButton("用当前区间做手眼标定")
        self.calib_btn.clicked.connect(self.run_calibration)
        actions_layout.addWidget(self.capture_btn)
        actions_layout.addWidget(self.calib_btn)
        right_layout.addWidget(actions_group)

        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        right_layout.addWidget(log_group, stretch=1)

    def _build_project_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.dataset_root_edit = self._new_line_edit(str(DATASET_ROOT))
        self.output_root_edit = self._new_line_edit(str(OUTPUT_ROOT))
        self.current_index_edit = self._new_line_edit("1")
        self.available_indices_label = QLabel("无")
        self.auto_inc_checkbox = QCheckBox("采集成功后索引自动 +1")
        self.auto_inc_checkbox.setChecked(True)
        btn_browse_dataset = QPushButton("选择数据目录")
        btn_browse_output = QPushButton("选择输出目录")
        btn_refresh = QPushButton("刷新样本索引")
        btn_browse_dataset.clicked.connect(
            lambda: self._choose_dir(self.dataset_root_edit)
        )
        btn_browse_output.clicked.connect(
            lambda: self._choose_dir(self.output_root_edit)
        )
        btn_refresh.clicked.connect(self.refresh_available_indices)
        layout.addRow("dataset_root", self.dataset_root_edit)
        layout.addRow("output_root", self.output_root_edit)
        layout.addRow("当前保存索引", self.current_index_edit)
        layout.addRow("已存在索引", self.available_indices_label)
        layout.addRow(self.auto_inc_checkbox)
        layout.addRow(btn_browse_dataset)
        layout.addRow(btn_browse_output)
        layout.addRow(btn_refresh)
        self.tabs.addTab(self._wrap_tab(page), "项目")

    def _build_camera_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.camera_topic_label = QLabel(ROS_CAMERA_DEFAULTS.color_topic)
        self.camera_info_topic_label = QLabel(ROS_CAMERA_DEFAULTS.camera_info_topic)
        self.camera_backend_label = QLabel("ROS 未连接")
        self.camera_requirements_label = QLabel("请先启动 roscore 与 piper_camera")
        self.camera_requirements_label.setWordWrap(True)
        btn_update = QPushButton("刷新流状态")
        btn_update.clicked.connect(self._update_stream_status)
        layout.addRow("color_topic", self.camera_topic_label)
        layout.addRow("camera_info_topic", self.camera_info_topic_label)
        layout.addRow("当前后端", self.camera_backend_label)
        layout.addRow(self.camera_requirements_label)
        layout.addRow(btn_update)
        self.tabs.addTab(self._wrap_tab(page), "相机")

    def _build_board_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.board_cols_edit = self._new_line_edit("11")
        self.board_rows_edit = self._new_line_edit("8")
        self.square_size_edit = self._new_line_edit("0.02")
        self.camera_info_source_label = QLabel(self.current_camera_info_source)
        self.camera_info_resolution_label = QLabel("unknown")
        self.camera_info_dist_model_label = QLabel("unknown")
        self.btn_sync_camera_info = QPushButton("从 ROS color/camera_info 同步")
        self.btn_sync_camera_info.clicked.connect(self.sync_ros_camera_info_to_fields)
        self.btn_sync_camera_info.setEnabled(self.ros_available)
        self.fx_edit = self._new_line_edit(f"{float(CAMERA_MATRIX[0, 0]):.8f}")
        self.fy_edit = self._new_line_edit(f"{float(CAMERA_MATRIX[1, 1]):.8f}")
        self.cx_edit = self._new_line_edit(f"{float(CAMERA_MATRIX[0, 2]):.8f}")
        self.cy_edit = self._new_line_edit(f"{float(CAMERA_MATRIX[1, 2]):.8f}")
        self.dist0_edit = self._new_line_edit(f"{float(DIST_COEFFS[0]):.8f}")
        self.dist1_edit = self._new_line_edit(f"{float(DIST_COEFFS[1]):.8f}")
        self.dist2_edit = self._new_line_edit(f"{float(DIST_COEFFS[2]):.8f}")
        self.dist3_edit = self._new_line_edit(f"{float(DIST_COEFFS[3]):.8f}")
        self.dist4_edit = self._new_line_edit("0.00000000")
        layout.addRow("棋盘列数(内角点)", self.board_cols_edit)
        layout.addRow("棋盘行数(内角点)", self.board_rows_edit)
        layout.addRow("square_size_m", self.square_size_edit)
        layout.addRow("相机模型来源", self.camera_info_source_label)
        layout.addRow("当前分辨率", self.camera_info_resolution_label)
        layout.addRow("distortion_model", self.camera_info_dist_model_label)
        layout.addRow(self.btn_sync_camera_info)
        layout.addRow("fx", self.fx_edit)
        layout.addRow("fy", self.fy_edit)
        layout.addRow("cx", self.cx_edit)
        layout.addRow("cy", self.cy_edit)
        layout.addRow("dist[0]", self.dist0_edit)
        layout.addRow("dist[1]", self.dist1_edit)
        layout.addRow("dist[2]", self.dist2_edit)
        layout.addRow("dist[3]", self.dist3_edit)
        layout.addRow("dist[4]", self.dist4_edit)
        self.tabs.addTab(self._wrap_tab(page), "棋盘/内参")

    def _build_ros_pose_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.ros_status_label = QLabel("可用" if self.ros_available else "不可用")
        self.auto_tf_checkbox = QCheckBox("采样时自动从 TF 读取位姿")
        self.auto_tf_checkbox.setChecked(self.ros_available)
        self.base_frame_edit = self._new_line_edit(ROS_DEFAULTS.base_frame)
        self.tool_frame_edit = self._new_line_edit(ROS_DEFAULTS.tool_frame)
        self.tf_timeout_edit = self._new_line_edit(f"{ROS_DEFAULTS.tf_timeout_sec:.2f}")
        self.pose_format_box = QComboBox()
        self.pose_format_box.addItems(["BASE_T_FLANGE", "FLANGE_T_BASE"])
        self.pose_format_box.setCurrentText(ROBOT_POSE_FORMAT)
        self.pose_rotation_formula_box = QComboBox()
        self.pose_rotation_formula_box.addItems(ROTATION_FORMULA_CANDIDATES)
        self.pose_rotation_formula_box.setCurrentText(DEFAULT_ROTATION_FORMULA)
        self.x_edit = self._new_line_edit("0.0")
        self.y_edit = self._new_line_edit("0.0")
        self.z_edit = self._new_line_edit("0.0")
        self.rx_edit = self._new_line_edit("0.0")
        self.ry_edit = self._new_line_edit("0.0")
        self.rz_edit = self._new_line_edit("0.0")
        self.btn_sync_tf = QPushButton("读取当前 TF 到输入框")
        self.btn_sync_tf.clicked.connect(self.sync_tf_to_manual_fields)
        self.btn_sync_tf.setEnabled(self.ros_available)
        layout.addRow("ROS", self.ros_status_label)
        layout.addRow(self.auto_tf_checkbox)
        layout.addRow("base_frame", self.base_frame_edit)
        layout.addRow("tool_frame", self.tool_frame_edit)
        layout.addRow("tf_timeout_sec", self.tf_timeout_edit)
        layout.addRow("pose_format", self.pose_format_box)
        layout.addRow("姿态欧拉公式", self.pose_rotation_formula_box)
        layout.addRow("x_mm", self.x_edit)
        layout.addRow("y_mm", self.y_edit)
        layout.addRow("z_mm", self.z_edit)
        layout.addRow("rx_deg", self.rx_edit)
        layout.addRow("ry_deg", self.ry_edit)
        layout.addRow("rz_deg", self.rz_edit)
        layout.addRow(self.btn_sync_tf)
        self.tabs.addTab(self._wrap_tab(page), "ROS/位姿")

    def _build_calibration_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.min_index_edit = self._new_line_edit("1")
        self.max_index_edit = self._new_line_edit("1")
        self.auto_formula_box = QComboBox()
        self.auto_formula_box.addItems(["AUTO", *ROTATION_FORMULA_CANDIDATES])
        self.auto_formula_box.setCurrentText(
            "AUTO" if AUTO_SEARCH_FORMULAS else DEFAULT_ROTATION_FORMULA
        )
        self.method_box = QComboBox()
        self.method_box.addItems(["AUTO", *HAND_EYE_METHODS.keys()])
        self.method_box.setCurrentText(
            "AUTO" if AUTO_SEARCH_METHODS else DEFAULT_HAND_EYE_METHOD
        )
        self.exclude_indices_edit = self._new_line_edit("")
        self.reproj_threshold_edit = self._new_line_edit("0.50")
        self.auto_prune_checkbox = QCheckBox("自动剔除离群样本")
        self.auto_prune_checkbox.setChecked(True)
        self.min_valid_samples_edit = self._new_line_edit(str(MIN_VALID_SAMPLES))
        self.max_prune_rounds_edit = self._new_line_edit(str(MAX_PRUNE_ROUNDS))
        self.min_rms_improve_edit = self._new_line_edit(f"{MIN_RMS_IMPROVEMENT_M:.6f}")
        self.min_max_improve_edit = self._new_line_edit(
            f"{MIN_MAX_DIST_IMPROVEMENT_M:.6f}"
        )
        layout.addRow("min_index", self.min_index_edit)
        layout.addRow("max_index", self.max_index_edit)
        layout.addRow("旋转公式", self.auto_formula_box)
        layout.addRow("HandEye 方法", self.method_box)
        layout.addRow("排除索引(如 4,8,11)", self.exclude_indices_edit)
        layout.addRow("最大重投影误差(px)", self.reproj_threshold_edit)
        layout.addRow(self.auto_prune_checkbox)
        layout.addRow("最少有效样本", self.min_valid_samples_edit)
        layout.addRow("最大剔除轮数", self.max_prune_rounds_edit)
        layout.addRow("最小 RMS 改善(m)", self.min_rms_improve_edit)
        layout.addRow("最小最大距离改善(m)", self.min_max_improve_edit)
        self.tabs.addTab(self._wrap_tab(page), "标定")

    def _build_constraints_tab(self) -> None:
        page = QWidget()
        layout = QFormLayout(page)
        self.constraint_mode_box = QComboBox()
        self.constraint_mode_box.addItems(CONSTRAINT_MODES)
        self.constraint_mode_box.setCurrentText(CONSTRAINT_MODE_DISABLED)
        self.tx_sign_box = QComboBox()
        self.tx_sign_box.addItems(SIGN_OPTIONS)
        self.tx_sign_box.setCurrentText(SIGN_ANY)
        self.ty_sign_box = QComboBox()
        self.ty_sign_box.addItems(SIGN_OPTIONS)
        self.ty_sign_box.setCurrentText(SIGN_ANY)
        self.tz_sign_box = QComboBox()
        self.tz_sign_box.addItems(SIGN_OPTIONS)
        self.tz_sign_box.setCurrentText(SIGN_ANY)
        self.tx_min_edit = self._new_line_edit("")
        self.tx_max_edit = self._new_line_edit("")
        self.ty_min_edit = self._new_line_edit("")
        self.ty_max_edit = self._new_line_edit("")
        self.tz_min_edit = self._new_line_edit("")
        self.tz_max_edit = self._new_line_edit("")
        self.z_axis_same_direction_checkbox = QCheckBox("要求 cam z 与 flange z 同向")
        self.z_axis_same_direction_checkbox.setChecked(False)
        self.z_axis_cos_min_edit = self._new_line_edit(f"{DEFAULT_Z_AXIS_COS_MIN:.3f}")
        layout.addRow("constraint_mode", self.constraint_mode_box)
        layout.addRow("tx_sign", self.tx_sign_box)
        layout.addRow("ty_sign", self.ty_sign_box)
        layout.addRow("tz_sign", self.tz_sign_box)
        layout.addRow("tx_min_m", self.tx_min_edit)
        layout.addRow("tx_max_m", self.tx_max_edit)
        layout.addRow("ty_min_m", self.ty_min_edit)
        layout.addRow("ty_max_m", self.ty_max_edit)
        layout.addRow("tz_min_m", self.tz_min_edit)
        layout.addRow("tz_max_m", self.tz_max_edit)
        layout.addRow(self.z_axis_same_direction_checkbox)
        layout.addRow("z_axis_cos_min", self.z_axis_cos_min_edit)
        self.tabs.addTab(self._wrap_tab(page), "物理约束")

    def dataset_root(self) -> Path:
        return Path(self.dataset_root_edit.text().strip()).expanduser().resolve()

    def output_root(self) -> Path:
        return Path(self.output_root_edit.text().strip()).expanduser().resolve()

    def _choose_dir(self, line_edit: QLineEdit) -> None:
        chosen = QFileDialog.getExistingDirectory(
            self, "选择目录", line_edit.text().strip() or "."
        )
        if chosen:
            line_edit.setText(chosen)
            self.refresh_available_indices()

    def log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.appendPlainText(f"[{timestamp}] {message}")

    def _set_intrinsics_fields(
        self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray
    ) -> None:
        self.fx_edit.setText(f"{float(camera_matrix[0, 0]):.8f}")
        self.fy_edit.setText(f"{float(camera_matrix[1, 1]):.8f}")
        self.cx_edit.setText(f"{float(camera_matrix[0, 2]):.8f}")
        self.cy_edit.setText(f"{float(camera_matrix[1, 2]):.8f}")
        coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        padded = np.zeros(5, dtype=np.float64)
        padded[: min(5, coeffs.size)] = coeffs[:5]
        self.dist0_edit.setText(f"{float(padded[0]):.8f}")
        self.dist1_edit.setText(f"{float(padded[1]):.8f}")
        self.dist2_edit.setText(f"{float(padded[2]):.8f}")
        self.dist3_edit.setText(f"{float(padded[3]):.8f}")
        self.dist4_edit.setText(f"{float(padded[4]):.8f}")

    def _camera_info_payload_from_msg(self, msg: CameraInfo) -> Dict[str, Any]:
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        D = np.array(list(msg.D), dtype=np.float64).reshape(-1)
        return {
            "width": int(msg.width),
            "height": int(msg.height),
            "frame_id": str(msg.header.frame_id),
            "distortion_model": str(msg.distortion_model or "plumb_bob"),
            "K": K.tolist(),
            "D": D.tolist(),
            "R": list(msg.R),
            "P": list(msg.P),
            "source": "/piper/camera/orbbec/color/camera_info",
            "stream_name": "color",
        }

    def log(self, message: str) -> None:
        if self._closing:
            return
        self.log_message.emit(message)

    def _append_log(self, message: str) -> None:
        if self._closing:
            return
        if not hasattr(self, "log_text") or self.log_text is None:
            return
        timestamp = datetime.now().strftime("%H:%M:%S")
        try:
            self.log_text.appendPlainText(f"[{timestamp}] {message}")
        except RuntimeError:
            pass

    def _update_stream_status(self) -> None:
        if self.current_frame is not None:
            self.capture_status_label.setText(
                f"采集状态：ROS 图像在线，来源={self.current_image_source}"
            )
            self.camera_backend_label.setText(self.current_image_source)
            return
        if self.ros_available:
            self.capture_status_label.setText("采集状态：等待 ROS 图像流")
            self.camera_backend_label.setText("ROS 订阅中")
        else:
            self.capture_status_label.setText("采集状态：ROS 未连接")
            self.camera_backend_label.setText("ROS 未连接")

    def on_ros_color_image(self, msg: Image) -> None:
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.current_frame_wall_time = datetime.now().isoformat(
                timespec="milliseconds"
            )
            self.current_image_source = ROS_CAMERA_DEFAULTS.color_topic
            if not self._closing:
                self.ros_image_received.emit(frame)
        except Exception as exc:
            self.log(f"ROS 彩图解析失败: {exc}")

    def _apply_ros_frame(self, frame: np.ndarray) -> None:
        if self._closing:
            return
        try:
            self.current_frame = frame.copy()
            self.video_label.set_frame(frame)
            self._update_stream_status()
        except RuntimeError:
            pass

    def _apply_camera_info_payload(self, payload: Dict[str, Any]) -> None:
        if self._closing:
            return
        try:
            self.camera_info_source_label.setText(self.current_camera_info_source)
            self.camera_info_resolution_label.setText(
                f"{payload['width']} x {payload['height']} @ {payload['frame_id']}"
            )
            self.camera_info_dist_model_label.setText(payload["distortion_model"])
            if self.current_ros_camera_matrix is not None:
                dist = self.current_ros_dist_coeffs
                if dist is None or dist.size == 0:
                    dist = np.zeros(5, dtype=np.float64)
                self._set_intrinsics_fields(self.current_ros_camera_matrix, dist)
        except RuntimeError:
            pass

    def on_color_camera_info(self, msg: CameraInfo) -> None:
        try:
            payload = self._camera_info_payload_from_msg(msg)
            self.current_camera_info_payload = payload
            self.current_ros_camera_matrix = np.array(payload["K"], dtype=np.float64)
            self.current_ros_dist_coeffs = np.array(payload["D"], dtype=np.float64)
            self.current_camera_info_source = payload["source"]
            if not self._closing:
                self.camera_info_updated.emit(payload)
        except Exception as exc:
            self.log(f"color/camera_info 解析失败: {exc}")

    def sync_ros_camera_info_to_fields(self) -> None:
        if (
            self.current_camera_info_payload is None
            or self.current_ros_camera_matrix is None
        ):
            QMessageBox.warning(self, "提示", "当前尚未收到 ROS color/camera_info")
            return
        dist = self.current_ros_dist_coeffs
        if dist is None or dist.size == 0:
            dist = np.zeros(5, dtype=np.float64)
        self._set_intrinsics_fields(self.current_ros_camera_matrix, dist)
        self.log("已将 ROS color/camera_info 同步到棋盘/内参页面")

    def current_board_cfg(self) -> BoardConfig:
        camera_matrix = np.array(
            [
                [
                    float(self.fx_edit.text().strip()),
                    0.0,
                    float(self.cx_edit.text().strip()),
                ],
                [
                    0.0,
                    float(self.fy_edit.text().strip()),
                    float(self.cy_edit.text().strip()),
                ],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.array(
            [
                float(self.dist0_edit.text().strip()),
                float(self.dist1_edit.text().strip()),
                float(self.dist2_edit.text().strip()),
                float(self.dist3_edit.text().strip()),
                float(self.dist4_edit.text().strip()),
            ],
            dtype=np.float64,
        )
        return BoardConfig(
            chessboard_size=(
                int(self.board_cols_edit.text().strip()),
                int(self.board_rows_edit.text().strip()),
            ),
            square_size_m=float(self.square_size_edit.text().strip()),
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
        )

    def current_constraints_cfg(self) -> PhysicalConstraintConfig:
        def opt(edit: QLineEdit) -> Optional[float]:
            token = edit.text().strip()
            return None if token == "" else float(token)

        return PhysicalConstraintConfig(
            mode=self.constraint_mode_box.currentText(),
            tx_sign=self.tx_sign_box.currentText(),
            ty_sign=self.ty_sign_box.currentText(),
            tz_sign=self.tz_sign_box.currentText(),
            tx_min_m=opt(self.tx_min_edit),
            tx_max_m=opt(self.tx_max_edit),
            ty_min_m=opt(self.ty_min_edit),
            ty_max_m=opt(self.ty_max_edit),
            tz_min_m=opt(self.tz_min_edit),
            tz_max_m=opt(self.tz_max_edit),
            z_axis_same_direction=self.z_axis_same_direction_checkbox.isChecked(),
            z_axis_cos_min=float(self.z_axis_cos_min_edit.text().strip()),
        )

    def refresh_available_indices(self) -> None:
        dataset_root = self.dataset_root()
        indices = list_available_dataset_indices(dataset_root)
        if not indices:
            self.available_indices_label.setText("无")
            self.current_index_edit.setText("1")
            self.min_index_edit.setText("1")
            self.max_index_edit.setText("1")
            return

        self.available_indices_label.setText(
            ", ".join(str(i) for i in indices[:20])
            + (" ..." if len(indices) > 20 else "")
        )
        next_index = max(indices) + 1
        self.current_index_edit.setText(str(next_index))
        self.min_index_edit.setText(str(min(indices)))
        self.max_index_edit.setText(str(max(indices)))

    def _parse_pose(self) -> RobotPose:
        return RobotPose(
            x_mm=float(self.x_edit.text().strip()),
            y_mm=float(self.y_edit.text().strip()),
            z_mm=float(self.z_edit.text().strip()),
            rx_deg=float(self.rx_edit.text().strip()),
            ry_deg=float(self.ry_edit.text().strip()),
            rz_deg=float(self.rz_edit.text().strip()),
        )

    def _set_pose_fields(
        self,
        tx_m: float,
        ty_m: float,
        tz_m: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
    ) -> None:
        self.x_edit.setText(f"{tx_m * 1000.0:.3f}")
        self.y_edit.setText(f"{ty_m * 1000.0:.3f}")
        self.z_edit.setText(f"{tz_m * 1000.0:.3f}")
        self.rx_edit.setText(f"{rx_deg:.6f}")
        self.ry_edit.setText(f"{ry_deg:.6f}")
        self.rz_edit.setText(f"{rz_deg:.6f}")

    def _lookup_tf_transform(self) -> Dict[str, Any]:
        if not self.ros_available or self.tf_buffer is None:
            raise RuntimeError("ROS/TF 不可用")
        base_frame = self.base_frame_edit.text().strip() or ROS_DEFAULTS.base_frame
        tool_frame = self.tool_frame_edit.text().strip() or ROS_DEFAULTS.tool_frame
        tf_timeout = float(self.tf_timeout_edit.text().strip())
        tf_msg = self.tf_buffer.lookup_transform(
            base_frame,
            tool_frame,
            rospy.Time(0),
            rospy.Duration(tf_timeout),
        )
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        translation_m = (float(t.x), float(t.y), float(t.z))
        quaternion_xyzw = (float(q.x), float(q.y), float(q.z), float(q.w))
        R = quaternion_to_matrix(*quaternion_xyzw)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.array(translation_m, dtype=np.float64)
        rx_deg, ry_deg, rz_deg = matrix_to_euler_xyz_deg(R)
        return {
            "pose_source": "ros_tf",
            "pose_format_hint": "BASE_T_FLANGE",
            "base_frame": base_frame,
            "tool_frame": tool_frame,
            "translation_m": list(translation_m),
            "quaternion_xyzw": list(quaternion_xyzw),
            "transform_matrix": T.tolist(),
            "pose": {
                "x_mm": translation_m[0] * 1000.0,
                "y_mm": translation_m[1] * 1000.0,
                "z_mm": translation_m[2] * 1000.0,
                "rx_deg": rx_deg,
                "ry_deg": ry_deg,
                "rz_deg": rz_deg,
            },
            "ros_stamp_sec": (
                float(tf_msg.header.stamp.to_sec())
                if hasattr(tf_msg.header.stamp, "to_sec")
                else None
            ),
        }

    def sync_tf_to_manual_fields(self) -> None:
        try:
            payload = self._lookup_tf_transform()
            pose = payload["pose"]
            self._set_pose_fields(
                pose["x_mm"] / 1000.0,
                pose["y_mm"] / 1000.0,
                pose["z_mm"] / 1000.0,
                pose["rx_deg"],
                pose["ry_deg"],
                pose["rz_deg"],
            )
            self.pose_format_box.setCurrentText("BASE_T_FLANGE")
            self.log("已将当前 TF 回填到输入框")
        except Exception as exc:
            self.log(f"读取 TF 失败: {exc}")
            QMessageBox.warning(self, "TF 读取失败", str(exc))

    def _build_manual_pose_payload(self) -> Dict[str, Any]:
        pose = self._parse_pose()
        T = robot_pose_to_base_T_flange(
            pose,
            self.pose_format_box.currentText(),
            self.pose_rotation_formula_box.currentText(),
        )
        translation_m = [float(v) for v in T[:3, 3]]
        return {
            "pose_source": "manual_input",
            "pose_format_hint": self.pose_format_box.currentText(),
            "transform_matrix": T.tolist(),
            "translation_m": translation_m,
            "quaternion_xyzw": None,
            "pose": pose_to_dict(pose),
            "pose_rotation_formula": self.pose_rotation_formula_box.currentText(),
        }

    def capture_sample(self) -> None:
        try:
            if self.current_frame is None:
                raise ValueError("当前没有可用视频帧")

            sample_index = int(self.current_index_edit.text().strip())
            if sample_index <= 0:
                raise ValueError("样本索引必须为正整数")

            frame = self.current_frame.copy()
            board_cfg = self.current_board_cfg()
            if self.auto_tf_checkbox.isChecked():
                pose_payload = self._lookup_tf_transform()
                pose_note = "ROS TF"
                pose = RobotPose(**pose_payload["pose"])
                self._set_pose_fields(
                    pose.x_mm / 1000.0,
                    pose.y_mm / 1000.0,
                    pose.z_mm / 1000.0,
                    pose.rx_deg,
                    pose.ry_deg,
                    pose.rz_deg,
                )
            else:
                pose_payload = self._build_manual_pose_payload()
                pose_note = "手动输入"

            dataset_root = ensure_dir(self.dataset_root())
            sample_dir = ensure_dir(dataset_root / str(sample_index))

            image_path = sample_dir / DEFAULT_IMAGE_NAME
            pose_path = sample_dir / DEFAULT_POSE_NAME
            camera_info_path = sample_dir / DEFAULT_CAMERA_INFO_NAME
            detect_path = sample_dir / DEFAULT_DETECTION_NAME
            annotated_path = sample_dir / DEFAULT_ANNOTATED_NAME

            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError(f"保存图像失败: {image_path}")

            det = detect_board_pose(frame, board_cfg)
            if self.current_camera_info_payload is not None:
                camera_info_payload = dict(self.current_camera_info_payload)
            else:
                camera_info_payload = {
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                    "frame_id": "",
                    "distortion_model": "plumb_bob",
                    "K": board_cfg.camera_matrix.tolist(),
                    "D": board_cfg.dist_coeffs.tolist(),
                    "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    "P": [
                        float(board_cfg.camera_matrix[0, 0]),
                        0.0,
                        float(board_cfg.camera_matrix[0, 2]),
                        0.0,
                        0.0,
                        float(board_cfg.camera_matrix[1, 1]),
                        float(board_cfg.camera_matrix[1, 2]),
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0,
                    ],
                    "source": self.current_camera_info_source,
                    "stream_name": "color",
                }
            camera_info_payload.update(
                {
                    "saved_at": datetime.now().isoformat(timespec="milliseconds"),
                    "capture_backend": self.current_image_source,
                    "rotate_180": False,
                }
            )
            write_json(camera_info_path, camera_info_payload)

            pose_payload.update(
                {
                    "frame_requirement": f"默认要求为 ^base T_flange；位置单位 mm；姿态单位 deg；pose_rotation_formula={self.pose_rotation_formula_box.currentText()}",
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "image_capture_wall_time": self.current_frame_wall_time,
                    "camera_backend": self.current_image_source,
                    "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                    "camera_matrix": board_cfg.camera_matrix.tolist(),
                    "dist_coeffs": board_cfg.dist_coeffs.tolist(),
                    "camera_info_file": camera_info_path.name,
                    "camera_model_source": camera_info_payload.get(
                        "source", self.current_camera_info_source
                    ),
                    "rotate_180": False,
                    "capture_config": {
                        "color_topic": ROS_CAMERA_DEFAULTS.color_topic,
                        "camera_info_topic": ROS_CAMERA_DEFAULTS.camera_info_topic,
                        "source_mode": "ros_only",
                    },
                }
            )
            write_json(pose_path, pose_payload)

            detect_payload = {
                "ok": det.ok,
                "message": det.message,
                "corners_count": det.corners_count,
                "reprojection_error_px": det.reprojection_error_px,
                "image_size": det.image_size,
                "camera_matrix": board_cfg.camera_matrix.tolist(),
                "dist_coeffs": board_cfg.dist_coeffs.tolist(),
                "camera_info_file": DEFAULT_CAMERA_INFO_NAME,
                "camera_model_source": self.current_camera_info_source,
                "rvec": det.rvec.reshape(-1).tolist() if det.rvec is not None else None,
                "tvec_m": (
                    det.tvec.reshape(-1).tolist() if det.tvec is not None else None
                ),
                "T_board_to_cam": (
                    det.T_board_to_cam.tolist()
                    if det.T_board_to_cam is not None
                    else None
                ),
                "board_config": {
                    "chessboard_size": list(board_cfg.chessboard_size),
                    "square_size_m": board_cfg.square_size_m,
                },
            }
            write_json(detect_path, detect_payload)
            if det.annotated_image is not None:
                cv2.imwrite(str(annotated_path), det.annotated_image)

            self.last_detection_ok = det.ok
            if det.ok:
                self.log(
                    f"样本 {sample_index} 已保存，来源={pose_note}，棋盘格检测成功，重投影误差={det.reprojection_error_px:.4f}px"
                )
                self.capture_status_label.setText(
                    f"采集状态：样本 {sample_index} 保存成功，位姿来源={pose_note}"
                )
            else:
                self.log(f"样本 {sample_index} 已保存，但棋盘格检测失败：{det.message}")
                self.capture_status_label.setText(
                    f"采集状态：样本 {sample_index} 保存成功，但棋盘格检测失败"
                )

            self.refresh_available_indices()
            self.current_index_edit.setText(
                str(
                    sample_index + 1
                    if self.auto_inc_checkbox.isChecked()
                    else sample_index
                )
            )
        except Exception as exc:
            self.log(f"采集失败: {exc}")
            QMessageBox.critical(self, "错误", str(exc))

    def run_calibration(self) -> None:
        try:
            dataset_root = ensure_dir(self.dataset_root())
            output_root = ensure_dir(self.output_root())
            min_index = int(self.min_index_edit.text().strip())
            max_index = int(self.max_index_edit.text().strip())
            if min_index > max_index:
                raise ValueError("min_index 不能大于 max_index")

            board_cfg = self.current_board_cfg()
            constraints_cfg = self.current_constraints_cfg()
            cfg = HandEyeConfig(
                dataset_root=dataset_root,
                output_root=output_root,
                pose_format=self.pose_format_box.currentText(),
                rotation_formula=self.pose_rotation_formula_box.currentText(),
                board=board_cfg,
                max_reprojection_error_px=float(
                    self.reproj_threshold_edit.text().strip()
                ),
                auto_prune_outliers=self.auto_prune_checkbox.isChecked(),
                min_valid_samples=int(self.min_valid_samples_edit.text().strip()),
                max_prune_rounds=int(self.max_prune_rounds_edit.text().strip()),
                min_rms_improvement_m=float(self.min_rms_improve_edit.text().strip()),
                min_max_dist_improvement_m=float(
                    self.min_max_improve_edit.text().strip()
                ),
                excluded_indices=parse_excluded_indices(
                    self.exclude_indices_edit.text().strip()
                ),
                physical_constraints=constraints_cfg,
            )

            auto_formula = self.auto_formula_box.currentText() == "AUTO"
            auto_method = self.method_box.currentText() == "AUTO"
            rotation_formula = (
                self.auto_formula_box.currentText()
                if not auto_formula
                else DEFAULT_ROTATION_FORMULA
            )
            method_name = (
                self.method_box.currentText()
                if not auto_method
                else DEFAULT_HAND_EYE_METHOD
            )

            self.log(
                f"开始标定，区间 [{min_index}, {max_index}]，constraint_mode={constraints_cfg.mode}"
            )
            best, ranked, valid_records, invalid = calibrate_from_dataset(
                dataset_root=dataset_root,
                min_index=min_index,
                max_index=max_index,
                cfg=cfg,
                auto_search_formulas=auto_formula,
                auto_search_methods=auto_method,
                rotation_formula=rotation_formula,
                method_name=method_name,
                pose_format=self.pose_format_box.currentText(),
            )
            out_dir = save_calibration_outputs(
                best,
                ranked,
                dataset_root,
                min_index,
                max_index,
                output_root,
                invalid,
                cfg=cfg,
            )
            self.log(f"有效样本数: {len(valid_records)}")
            self.log(f"无效/缺失/过滤索引: {invalid}")
            self.log(f"自动剔除索引: {best.pruned_indices}")
            self.log(
                f"最优组合: method={best.method_name}, formula={best.rotation_formula}"
            )
            self.log(
                f"约束满足: {best.constraint_eval.passed}, 违例={best.constraint_eval.violations}"
            )
            self.log(f"平移 RMS={best.metrics.translation_rms_to_mean_m:.6f} m")
            self.log(f"最大平移误差={best.metrics.translation_max_to_mean_m:.6f} m")
            self.log("^flange T_cam =")
            self.log(
                np.array2string(best.T_cam_to_flange, precision=6, suppress_small=False)
            )
            self.log(f"结果已写入: {out_dir}")
            QMessageBox.information(self, "完成", f"标定完成\n输出目录:\n{out_dir}")
        except Exception as exc:
            self.log(f"标定失败: {exc}")
            QMessageBox.critical(self, "错误", str(exc))

    def closeEvent(self, event):
        self._closing = True
        if self.color_info_sub is not None:
            try:
                self.color_info_sub.unregister()
            except Exception:
                pass
            self.color_info_sub = None
        if self.color_image_sub is not None:
            try:
                self.color_image_sub.unregister()
            except Exception:
                pass
            self.color_image_sub = None
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = HandEyeCaptureWindow()
    window.show()
    QTimer.singleShot(0, window.showMaximized)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
