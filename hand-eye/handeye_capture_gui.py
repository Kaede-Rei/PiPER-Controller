from __future__ import annotations

import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
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
    QVBoxLayout,
    QWidget,
)

from pipper_hand_eye import (
    AUTO_SEARCH_FORMULAS,
    AUTO_SEARCH_METHODS,
    CAMERA_MATRIX,
    DATASET_ROOT,
    DEFAULT_ANNOTATED_NAME,
    DEFAULT_DETECTION_NAME,
    DEFAULT_HAND_EYE_METHOD,
    DEFAULT_IMAGE_NAME,
    DEFAULT_POSE_NAME,
    DEFAULT_ROTATION_FORMULA,
    DIST_COEFFS,
    HAND_EYE_METHODS,
    OUTPUT_ROOT,
    ROBOT_POSE_FORMAT,
    ROTATION_FORMULA_CANDIDATES,
    BoardConfig,
    HandEyeConfig,
    RobotPose,
    calibrate_from_dataset,
    detect_board_pose,
    ensure_dir,
    list_available_dataset_indices,
    pose_to_dict,
    save_calibration_outputs,
    write_json,
)

try:
    from pyorbbecsdk import (
        Config,
        OBAlignMode,
        OBFormat,
        OBSensorType,
        Pipeline,
        VideoFrame,
    )

    HAS_ORBBEC = True
except Exception:
    HAS_ORBBEC = False
    Config = None
    OBAlignMode = None
    OBFormat = None
    OBSensorType = None
    Pipeline = None
    VideoFrame = object

try:
    import rospy
    import tf2_ros

    HAS_ROS = True
except Exception:
    rospy = None
    tf2_ros = None
    HAS_ROS = False


@dataclass(frozen=True)
class UiConfig:
    window_title: str = "HandEye 数据采集与标定"
    window_x: int = 100
    window_y: int = 100
    window_w: int = 1680
    window_h: int = 980
    image_min_w: int = 800
    image_min_h: int = 600
    info_font_name: str = "Arial"
    info_font_size: int = 10


@dataclass(frozen=True)
class CaptureConfig:
    color_width: int = 1280
    color_height: int = 720
    color_fps: int = 30
    cv_device_index: int = 0
    rotate_180: bool = False


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "handeye_capture_gui"
    base_frame: str = "base_link"
    tool_frame: str = "link6"
    tf_timeout_sec: float = 0.30


UI_CFG = UiConfig()
CAPTURE_CFG = CaptureConfig()
ROS_CFG = RosConfig()


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


class CameraWorker(QThread):
    frame_ready = pyqtSignal(object)
    status = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self.pipeline = None
        self.cap = None
        self.latest_frame: Optional[np.ndarray] = None
        self.backend_name = "uninitialized"

    def run(self):
        self._running = True
        if HAS_ORBBEC:
            try:
                self._run_orbbec()
                return
            except Exception as exc:
                self.status.emit(f"Orbbec 打开失败，回退到 OpenCV: {exc}")
        self._run_opencv()

    def _run_orbbec(self):
        self.backend_name = "OrbbecSDK"
        self.pipeline = Pipeline()
        config = Config()
        profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            profile = profile_list.get_video_stream_profile(
                CAPTURE_CFG.color_width,
                CAPTURE_CFG.color_height,
                OBFormat.RGB,
                CAPTURE_CFG.color_fps,
            )
        except Exception:
            profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(profile)
        if OBAlignMode is not None:
            config.set_align_mode(OBAlignMode.SW_MODE)
        self.pipeline.start(config)
        self.status.emit("✅ 相机已打开: OrbbecSDK")

        while self._running:
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            image = self._frame_to_bgr(color_frame)
            if image is None:
                continue
            if CAPTURE_CFG.rotate_180:
                image = cv2.rotate(image, cv2.ROTATE_180)
            self.latest_frame = image
            self.frame_ready.emit(image)

    def _frame_to_bgr(self, frame: VideoFrame) -> Optional[np.ndarray]:
        width = frame.get_width()
        height = frame.get_height()
        fmt = frame.get_format()
        data = np.asanyarray(frame.get_data())
        if fmt == OBFormat.RGB:
            image = np.resize(data, (height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if fmt == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if fmt == OBFormat.YUYV:
            image = np.resize(data, (height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
        return None

    def _run_opencv(self):
        self.backend_name = f"OpenCV({CAPTURE_CFG.cv_device_index})"
        self.cap = cv2.VideoCapture(CAPTURE_CFG.cv_device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_CFG.color_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_CFG.color_height)
        self.cap.set(cv2.CAP_PROP_FPS, CAPTURE_CFG.color_fps)
        if not self.cap.isOpened():
            self.status.emit("❌ OpenCV 相机打开失败")
            return
        self.status.emit(f"✅ 相机已打开: {self.backend_name}")
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                continue
            if CAPTURE_CFG.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            self.latest_frame = frame
            self.frame_ready.emit(frame)

    def stop(self) -> None:
        self._running = False
        self.wait(2000)
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        if self.cap is not None:
            self.cap.release()


class HandEyeCaptureWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(UI_CFG.window_title)
        self.setGeometry(UI_CFG.window_x, UI_CFG.window_y, UI_CFG.window_w, UI_CFG.window_h)

        self.board_cfg = BoardConfig(
            camera_matrix=CAMERA_MATRIX.copy(), dist_coeffs=DIST_COEFFS.copy()
        )
        self.current_frame: Optional[np.ndarray] = None
        self.current_frame_wall_time: str = ""
        self.last_detection_ok = False

        self.ros_available = False
        self.tf_buffer = None
        self.tf_listener = None
        self._init_ros()

        self.video_label = VideoWidget()
        self.worker = CameraWorker()
        self.worker.frame_ready.connect(self.on_frame_ready)
        self.worker.status.connect(self.log)

        self.init_ui()
        self.refresh_available_indices()
        self.worker.start()

        self._toggle_window_shortcut = QShortcut(QKeySequence("F11"), self)
        self._toggle_window_shortcut.activated.connect(self.toggle_window_state)

    def _init_ros(self) -> None:
        if not HAS_ROS:
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node(ROS_CFG.node_name, anonymous=True, disable_signals=True)
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.ros_available = True
        except Exception:
            self.ros_available = False
            self.tf_buffer = None
            self.tf_listener = None

    def toggle_window_state(self) -> None:
        if self.isMaximized() or self.isFullScreen():
            self.showNormal()
        else:
            self.showMaximized()

    def init_ui(self) -> None:
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        main_layout.addWidget(self.video_label, stretch=3)

        self.side_scroll = QScrollArea()
        self.side_scroll.setWidgetResizable(True)
        self.side_panel = QWidget()
        panel_layout = QHBoxLayout(self.side_panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        self.side_scroll.setWidget(self.side_panel)
        main_layout.addWidget(self.side_scroll, stretch=2)

        info_column = QVBoxLayout()
        control_column = QVBoxLayout()
        panel_layout.addLayout(info_column, stretch=1)
        panel_layout.addLayout(control_column, stretch=1)

        info_group = QGroupBox("状态 / 说明")
        info_group_layout = QVBoxLayout(info_group)
        self.info_label = QLabel(
            "系统初始化中...\n\n"
            "说明：\n"
            "1. GUI 启动后会自动打开相机并在左侧显示视频。\n"
            "2. 默认优先在采样瞬间自动读取 ^base T_flange。\n"
            "3. pose.json 会同时保存 4x4 矩阵、平移、四元数和旧版欧拉角字段。\n"
            "4. F11 可在普通窗口和最大化之间切换。\n"
            "5. 标定时会输出每样本诊断信息，并支持自动剔除离群样本。"
        )
        self.info_label.setWordWrap(True)
        self.info_label.setFont(QFont(UI_CFG.info_font_name, UI_CFG.info_font_size))
        self.info_label.setTextFormat(Qt.RichText)
        self.info_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        info_group_layout.addWidget(self.info_label)
        info_group.setMinimumWidth(320)
        info_column.addWidget(info_group)

        self.capture_status_label = QLabel("采集状态：等待相机")
        self.capture_status_label.setWordWrap(True)
        self.capture_status_label.setFont(QFont(UI_CFG.info_font_name, UI_CFG.info_font_size))
        info_column.addWidget(self.capture_status_label)

        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        info_column.addWidget(log_group)
        info_column.addStretch()

        dataset_group = QGroupBox("数据目录与索引")
        dataset_form = QFormLayout(dataset_group)
        self.dataset_root_edit = QLineEdit(str(DATASET_ROOT))
        self.output_root_edit = QLineEdit(str(OUTPUT_ROOT))
        self.current_index_edit = QLineEdit("1")
        self.available_indices_label = QLabel("无")
        btn_browse_dataset = QPushButton("选择数据目录")
        btn_browse_output = QPushButton("选择输出目录")
        btn_refresh = QPushButton("刷新样本索引")
        btn_browse_dataset.clicked.connect(lambda: self._choose_dir(self.dataset_root_edit))
        btn_browse_output.clicked.connect(lambda: self._choose_dir(self.output_root_edit))
        btn_refresh.clicked.connect(self.refresh_available_indices)
        dataset_form.addRow("dataset_root", self.dataset_root_edit)
        dataset_form.addRow("output_root", self.output_root_edit)
        dataset_form.addRow("当前保存索引", self.current_index_edit)
        dataset_form.addRow("已存在索引", self.available_indices_label)
        dataset_form.addRow(btn_browse_dataset)
        dataset_form.addRow(btn_browse_output)
        dataset_form.addRow(btn_refresh)
        control_column.addWidget(dataset_group)

        ros_group = QGroupBox("ROS TF 采样")
        ros_form = QFormLayout(ros_group)
        self.ros_status_label = QLabel("✅ 可用" if self.ros_available else "❌ 不可用")
        self.auto_tf_checkbox = QCheckBox("采样时自动从 TF 读取位姿")
        self.auto_tf_checkbox.setChecked(self.ros_available)
        self.base_frame_edit = QLineEdit(ROS_CFG.base_frame)
        self.tool_frame_edit = QLineEdit(ROS_CFG.tool_frame)
        self.btn_sync_tf = QPushButton("读取当前 TF 到输入框")
        self.btn_sync_tf.clicked.connect(self.sync_tf_to_manual_fields)
        self.btn_sync_tf.setEnabled(self.ros_available)
        ros_form.addRow("ROS", self.ros_status_label)
        ros_form.addRow(self.auto_tf_checkbox)
        ros_form.addRow("base_frame", self.base_frame_edit)
        ros_form.addRow("tool_frame", self.tool_frame_edit)
        ros_form.addRow(self.btn_sync_tf)
        control_column.addWidget(ros_group)

        pose_group = QGroupBox("机械臂当前位姿输入 / 回显")
        pose_form = QFormLayout(pose_group)
        self.pose_notice = QLabel(
            "默认要求为 ^base T_flange。\n"
            "若勾选“采样时自动从 TF 读取位姿”，下面的数值主要用于回显与兼容旧版数据格式。\n"
            "位置单位 mm；姿态单位 deg。"
        )
        self.pose_notice.setWordWrap(True)
        self.pose_format_box = QComboBox()
        self.pose_format_box.addItems(["BASE_T_FLANGE", "FLANGE_T_BASE"])
        self.pose_format_box.setCurrentText(ROBOT_POSE_FORMAT)
        self.x_edit = QLineEdit("0.0")
        self.y_edit = QLineEdit("0.0")
        self.z_edit = QLineEdit("0.0")
        self.rx_edit = QLineEdit("0.0")
        self.ry_edit = QLineEdit("0.0")
        self.rz_edit = QLineEdit("0.0")
        pose_form.addRow(self.pose_notice)
        pose_form.addRow("pose_format", self.pose_format_box)
        pose_form.addRow("x_mm", self.x_edit)
        pose_form.addRow("y_mm", self.y_edit)
        pose_form.addRow("z_mm", self.z_edit)
        pose_form.addRow("rx_deg", self.rx_edit)
        pose_form.addRow("ry_deg", self.ry_edit)
        pose_form.addRow("rz_deg", self.rz_edit)
        control_column.addWidget(pose_group)

        capture_group = QGroupBox("样本采集")
        capture_layout = QVBoxLayout(capture_group)
        self.auto_inc_checkbox = QCheckBox("采集成功后索引自动 +1")
        self.auto_inc_checkbox.setChecked(True)
        self.capture_btn = QPushButton("拍照并保存当前样本")
        self.capture_btn.clicked.connect(self.capture_sample)
        capture_layout.addWidget(self.auto_inc_checkbox)
        capture_layout.addWidget(self.capture_btn)
        control_column.addWidget(capture_group)

        calib_group = QGroupBox("手眼标定")
        calib_form = QFormLayout(calib_group)
        self.min_index_edit = QLineEdit("1")
        self.max_index_edit = QLineEdit("1")
        self.auto_formula_box = QComboBox()
        self.auto_formula_box.addItems(["AUTO", *ROTATION_FORMULA_CANDIDATES])
        self.auto_formula_box.setCurrentText("AUTO" if AUTO_SEARCH_FORMULAS else DEFAULT_ROTATION_FORMULA)
        self.method_box = QComboBox()
        self.method_box.addItems(["AUTO", *HAND_EYE_METHODS.keys()])
        self.method_box.setCurrentText("AUTO" if AUTO_SEARCH_METHODS else DEFAULT_HAND_EYE_METHOD)
        self.exclude_indices_edit = QLineEdit("")
        self.reproj_threshold_edit = QLineEdit("0.50")
        self.auto_prune_checkbox = QCheckBox("自动剔除离群样本")
        self.auto_prune_checkbox.setChecked(True)
        self.calib_btn = QPushButton("用当前区间做手眼标定")
        self.calib_btn.clicked.connect(self.run_calibration)
        calib_form.addRow("min_index", self.min_index_edit)
        calib_form.addRow("max_index", self.max_index_edit)
        calib_form.addRow("旋转公式", self.auto_formula_box)
        calib_form.addRow("HandEye 方法", self.method_box)
        calib_form.addRow("排除索引(如 4,8,11)", self.exclude_indices_edit)
        calib_form.addRow("最大重投影误差(px)", self.reproj_threshold_edit)
        calib_form.addRow(self.auto_prune_checkbox)
        calib_form.addRow(self.calib_btn)
        control_column.addWidget(calib_group)

        control_column.addStretch()

    def dataset_root(self) -> Path:
        return Path(self.dataset_root_edit.text().strip()).expanduser().resolve()

    def output_root(self) -> Path:
        return Path(self.output_root_edit.text().strip()).expanduser().resolve()

    def _choose_dir(self, line_edit: QLineEdit) -> None:
        chosen = QFileDialog.getExistingDirectory(self, "选择目录", line_edit.text().strip() or ".")
        if chosen:
            line_edit.setText(chosen)
            self.refresh_available_indices()

    def log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.appendPlainText(f"[{timestamp}] {message}")

    def on_frame_ready(self, frame: np.ndarray) -> None:
        self.current_frame = frame.copy()
        self.current_frame_wall_time = datetime.now().isoformat(timespec="milliseconds")
        self.video_label.set_frame(frame)
        ros_note = " | ROS在线" if self.ros_available else " | ROS离线"
        self.capture_status_label.setText(f"采集状态：相机在线，后端={self.worker.backend_name}{ros_note}")

    def refresh_available_indices(self) -> None:
        dataset_root = self.dataset_root()
        indices = list_available_dataset_indices(dataset_root)
        if not indices:
            self.available_indices_label.setText("无")
            self.current_index_edit.setText("1")
            self.min_index_edit.setText("1")
            self.max_index_edit.setText("1")
            return

        self.available_indices_label.setText(", ".join(str(i) for i in indices[:20]) + (" ..." if len(indices) > 20 else ""))
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

    def _set_pose_fields(self, tx_m: float, ty_m: float, tz_m: float, rx_deg: float, ry_deg: float, rz_deg: float) -> None:
        self.x_edit.setText(f"{tx_m * 1000.0:.3f}")
        self.y_edit.setText(f"{ty_m * 1000.0:.3f}")
        self.z_edit.setText(f"{tz_m * 1000.0:.3f}")
        self.rx_edit.setText(f"{rx_deg:.6f}")
        self.ry_edit.setText(f"{ry_deg:.6f}")
        self.rz_edit.setText(f"{rz_deg:.6f}")

    def _lookup_tf_transform(self) -> Dict[str, Any]:
        if not self.ros_available or self.tf_buffer is None:
            raise RuntimeError("ROS/TF 不可用")
        base_frame = self.base_frame_edit.text().strip() or ROS_CFG.base_frame
        tool_frame = self.tool_frame_edit.text().strip() or ROS_CFG.tool_frame
        tf_msg = self.tf_buffer.lookup_transform(
            base_frame,
            tool_frame,
            rospy.Time(0),
            rospy.Duration(ROS_CFG.tf_timeout_sec),
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
            "ros_stamp_sec": float(tf_msg.header.stamp.to_sec()) if hasattr(tf_msg.header.stamp, "to_sec") else None,
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
        # 这里保留旧格式兼容；新格式仍会保存 4x4 矩阵
        from pipper_hand_eye import robot_pose_to_base_T_flange

        T = robot_pose_to_base_T_flange(
            pose,
            self.pose_format_box.currentText(),
            DEFAULT_ROTATION_FORMULA,
        )
        translation_m = [float(v) for v in T[:3, 3]]
        return {
            "pose_source": "manual_input",
            "pose_format_hint": self.pose_format_box.currentText(),
            "transform_matrix": T.tolist(),
            "translation_m": translation_m,
            "quaternion_xyzw": None,
            "pose": pose_to_dict(pose),
        }

    def capture_sample(self) -> None:
        try:
            if self.current_frame is None:
                raise ValueError("当前没有可用视频帧")

            sample_index = int(self.current_index_edit.text().strip())
            if sample_index <= 0:
                raise ValueError("样本索引必须为正整数")

            frame = self.current_frame.copy()
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
                pose = self._parse_pose()

            dataset_root = ensure_dir(self.dataset_root())
            sample_dir = ensure_dir(dataset_root / str(sample_index))

            image_path = sample_dir / DEFAULT_IMAGE_NAME
            pose_path = sample_dir / DEFAULT_POSE_NAME
            detect_path = sample_dir / DEFAULT_DETECTION_NAME
            annotated_path = sample_dir / DEFAULT_ANNOTATED_NAME

            if not cv2.imwrite(str(image_path), frame):
                raise RuntimeError(f"保存图像失败: {image_path}")

            det = detect_board_pose(frame, self.board_cfg)
            pose_payload.update(
                {
                    "frame_requirement": "默认要求为 ^base T_flange；单位 x/y/z=mm, rx/ry/rz=deg",
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "image_capture_wall_time": self.current_frame_wall_time,
                    "camera_backend": self.worker.backend_name,
                    "image_size": [int(frame.shape[1]), int(frame.shape[0])],
                    "camera_matrix": CAMERA_MATRIX.tolist(),
                    "dist_coeffs": DIST_COEFFS.tolist(),
                    "rotate_180": CAPTURE_CFG.rotate_180,
                }
            )
            write_json(pose_path, pose_payload)

            detect_payload = {
                "ok": det.ok,
                "message": det.message,
                "corners_count": det.corners_count,
                "reprojection_error_px": det.reprojection_error_px,
                "image_size": det.image_size,
                "camera_matrix": CAMERA_MATRIX.tolist(),
                "dist_coeffs": DIST_COEFFS.tolist(),
                "rvec": det.rvec.reshape(-1).tolist() if det.rvec is not None else None,
                "tvec_m": det.tvec.reshape(-1).tolist() if det.tvec is not None else None,
                "T_board_to_cam": det.T_board_to_cam.tolist() if det.T_board_to_cam is not None else None,
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

            if self.auto_inc_checkbox.isChecked():
                self.current_index_edit.setText(str(sample_index + 1))
            self.refresh_available_indices()
            self.current_index_edit.setText(str(sample_index + 1 if self.auto_inc_checkbox.isChecked() else sample_index))
        except Exception as exc:
            self.log(f"采集失败: {exc}")
            QMessageBox.critical(self, "错误", str(exc))

    def run_calibration(self) -> None:
        try:
            from pipper_hand_eye import parse_excluded_indices

            dataset_root = ensure_dir(self.dataset_root())
            output_root = ensure_dir(self.output_root())
            min_index = int(self.min_index_edit.text().strip())
            max_index = int(self.max_index_edit.text().strip())
            if min_index > max_index:
                raise ValueError("min_index 不能大于 max_index")

            cfg = HandEyeConfig(
                dataset_root=dataset_root,
                output_root=output_root,
                pose_format=self.pose_format_box.currentText(),
                rotation_formula=DEFAULT_ROTATION_FORMULA,
                board=self.board_cfg,
                max_reprojection_error_px=float(self.reproj_threshold_edit.text().strip()),
                auto_prune_outliers=self.auto_prune_checkbox.isChecked(),
                excluded_indices=parse_excluded_indices(self.exclude_indices_edit.text().strip()),
            )

            auto_formula = self.auto_formula_box.currentText() == "AUTO"
            auto_method = self.method_box.currentText() == "AUTO"
            rotation_formula = self.auto_formula_box.currentText() if not auto_formula else DEFAULT_ROTATION_FORMULA
            method_name = self.method_box.currentText() if not auto_method else DEFAULT_HAND_EYE_METHOD

            self.log(f"开始标定，使用连续区间 [{min_index}, {max_index}]")
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
            out_dir = save_calibration_outputs(best, ranked, dataset_root, min_index, max_index, output_root, invalid)
            self.log(f"有效样本数: {len(valid_records)}")
            self.log(f"无效/缺失/过滤索引: {invalid}")
            self.log(f"自动剔除索引: {best.pruned_indices}")
            self.log(f"最优组合: method={best.method_name}, formula={best.rotation_formula}")
            self.log(f"平移 RMS={best.metrics.translation_rms_to_mean_m:.6f} m")
            self.log(f"最大平移误差={best.metrics.translation_max_to_mean_m:.6f} m")
            self.log("^flange T_cam =")
            self.log(np.array2string(best.T_cam_to_flange, precision=6, suppress_small=False))
            self.log(f"结果已写入: {out_dir}")
            QMessageBox.information(self, "完成", f"标定完成\n输出目录:\n{out_dir}")
        except Exception as exc:
            self.log(f"标定失败: {exc}")
            QMessageBox.critical(self, "错误", str(exc))

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = HandEyeCaptureWindow()
    window.show()
    QTimer.singleShot(0, window.showMaximized)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
