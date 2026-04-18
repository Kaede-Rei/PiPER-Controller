from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ============================
# 顶部统一配置区
# ============================
DATASET_ROOT = Path("./picture")
OUTPUT_ROOT = Path("./outputs")

DEFAULT_IMAGE_NAME = "image.jpg"
DEFAULT_POSE_NAME = "pose.json"
DEFAULT_DETECTION_NAME = "board_detection.json"
DEFAULT_ANNOTATED_NAME = "annotated.jpg"

CAMERA_MATRIX = np.array(
    [
        [612.28741455, 0.0, 638.46850586],
        [0.0, 612.24603271, 399.93331909],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DIST_COEFFS = np.array([-0.02141041, 0.01469840, 0.0, 0.0], dtype=np.float64)

CHESSBOARD_SIZE = (11, 8)  # 内角点 (列, 行)
SQUARE_SIZE_M = 0.02

ROBOT_POSE_FORMAT = "BASE_T_FLANGE"
ROTATION_FORMULA_CANDIDATES = [
    "RzRyRx",
    "RxRyRz",
    "RyRxRz",
    "RzRxRy",
    "RxRzRy",
    "RyRzRx",
]
DEFAULT_ROTATION_FORMULA = "RzRyRx"

AUTO_SEARCH_FORMULAS = True
AUTO_SEARCH_METHODS = True
DEFAULT_HAND_EYE_METHOD = "TSAI"

# 棋盘格检测参数
FIND_CORNERS_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_FAST_CHECK
)
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    1e-3,
)
SUBPIX_WIN = (11, 11)

# 质量控制参数
MAX_REPROJECTION_ERROR_PX = 0.50
AUTO_PRUNE_OUTLIERS = True
MIN_VALID_SAMPLES = 6
MAX_PRUNE_ROUNDS = 3
MIN_RMS_IMPROVEMENT_M = 0.003
MIN_MAX_DIST_IMPROVEMENT_M = 0.005

# 物理约束参数
CONSTRAINT_MODE_DISABLED = "DISABLED"
CONSTRAINT_MODE_SOFT = "SOFT_PREFER"
CONSTRAINT_MODE_HARD = "HARD_FILTER"
CONSTRAINT_MODES = [
    CONSTRAINT_MODE_DISABLED,
    CONSTRAINT_MODE_SOFT,
    CONSTRAINT_MODE_HARD,
]
SIGN_ANY = "ANY"
SIGN_POSITIVE = "POSITIVE"
SIGN_NEGATIVE = "NEGATIVE"
SIGN_OPTIONS = [SIGN_ANY, SIGN_POSITIVE, SIGN_NEGATIVE]
DEFAULT_SIGN_CONSTRAINT = SIGN_ANY
DEFAULT_Z_AXIS_COS_MIN = 0.0

HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ============================
# 数据结构
# ============================
@dataclass(frozen=True)
class BoardConfig:
    chessboard_size: Tuple[int, int] = CHESSBOARD_SIZE
    square_size_m: float = SQUARE_SIZE_M
    camera_matrix: np.ndarray = field(default_factory=lambda: CAMERA_MATRIX.copy())
    dist_coeffs: np.ndarray = field(default_factory=lambda: DIST_COEFFS.copy())


@dataclass(frozen=True)
class RobotPose:
    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float


@dataclass(frozen=True)
class PhysicalConstraintConfig:
    mode: str = CONSTRAINT_MODE_DISABLED
    tx_sign: str = DEFAULT_SIGN_CONSTRAINT
    ty_sign: str = DEFAULT_SIGN_CONSTRAINT
    tz_sign: str = DEFAULT_SIGN_CONSTRAINT
    tx_min_m: Optional[float] = None
    tx_max_m: Optional[float] = None
    ty_min_m: Optional[float] = None
    ty_max_m: Optional[float] = None
    tz_min_m: Optional[float] = None
    tz_max_m: Optional[float] = None
    z_axis_same_direction: bool = False
    z_axis_cos_min: float = DEFAULT_Z_AXIS_COS_MIN

    @property
    def enabled(self) -> bool:
        return self.mode != CONSTRAINT_MODE_DISABLED


@dataclass(frozen=True)
class HandEyeConfig:
    dataset_root: Path = DATASET_ROOT
    output_root: Path = OUTPUT_ROOT
    pose_format: str = ROBOT_POSE_FORMAT
    rotation_formula: str = DEFAULT_ROTATION_FORMULA
    board: BoardConfig = field(default_factory=BoardConfig)
    max_reprojection_error_px: Optional[float] = MAX_REPROJECTION_ERROR_PX
    auto_prune_outliers: bool = AUTO_PRUNE_OUTLIERS
    min_valid_samples: int = MIN_VALID_SAMPLES
    max_prune_rounds: int = MAX_PRUNE_ROUNDS
    min_rms_improvement_m: float = MIN_RMS_IMPROVEMENT_M
    min_max_dist_improvement_m: float = MIN_MAX_DIST_IMPROVEMENT_M
    excluded_indices: Tuple[int, ...] = ()
    physical_constraints: PhysicalConstraintConfig = field(
        default_factory=PhysicalConstraintConfig
    )


@dataclass
class BoardDetection:
    ok: bool
    message: str
    corners_count: int = 0
    reprojection_error_px: Optional[float] = None
    image_size: Optional[Tuple[int, int]] = None
    rvec: Optional[np.ndarray] = None
    tvec: Optional[np.ndarray] = None
    T_board_to_cam: Optional[np.ndarray] = None
    corners: Optional[np.ndarray] = None
    annotated_image: Optional[np.ndarray] = None


@dataclass
class LoadedPose:
    T_base_to_flange: np.ndarray
    pose_format: str
    source: str
    robot_pose: Optional[RobotPose] = None
    translation_m: Optional[Tuple[float, float, float]] = None
    quaternion_xyzw: Optional[Tuple[float, float, float, float]] = None


@dataclass
class SampleRecord:
    index: int
    image_path: Path
    pose_path: Path
    robot_pose: Optional[RobotPose]
    T_base_to_flange: np.ndarray
    detection: BoardDetection
    pose_source: str = "unknown"
    pose_format: str = ROBOT_POSE_FORMAT
    excluded_reason: Optional[str] = None


@dataclass(frozen=True)
class CalibrationMetrics:
    translation_mean_m: Tuple[float, float, float]
    translation_rms_to_mean_m: float
    translation_max_to_mean_m: float
    rotation_max_to_first_deg: float
    valid_count: int


@dataclass
class ConstraintEvaluation:
    mode: str
    passed: bool
    hard_failed: bool
    penalty: float
    violations: List[str] = field(default_factory=list)
    translation_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    z_axis_in_flange: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    z_axis_cos_to_flange_z: float = 1.0


@dataclass
class CalibrationCandidate:
    method_name: str
    rotation_formula: str
    pose_format: str
    T_cam_to_flange: np.ndarray
    metrics: CalibrationMetrics
    sample_indices: List[int]
    board_to_base_translations_m: List[Tuple[float, float, float]]
    board_to_base_rotations_deg_to_first: List[float]
    sample_diagnostics: List[Dict[str, Any]] = field(default_factory=list)
    pruned_indices: List[int] = field(default_factory=list)
    constraint_eval: ConstraintEvaluation = field(
        default_factory=lambda: ConstraintEvaluation(
            mode=CONSTRAINT_MODE_DISABLED,
            passed=True,
            hard_failed=False,
            penalty=0.0,
        )
    )


# ============================
# 基础工具
# ============================
def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    ensure_dir(path.parent)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def pose_to_dict(pose: RobotPose) -> Dict[str, float]:
    return {
        "x_mm": float(pose.x_mm),
        "y_mm": float(pose.y_mm),
        "z_mm": float(pose.z_mm),
        "rx_deg": float(pose.rx_deg),
        "ry_deg": float(pose.ry_deg),
        "rz_deg": float(pose.rz_deg),
    }


def dict_to_pose(payload: Dict[str, Any]) -> RobotPose:
    src = payload.get("pose", payload)
    return RobotPose(
        x_mm=float(src["x_mm"]),
        y_mm=float(src["y_mm"]),
        z_mm=float(src["z_mm"]),
        rx_deg=float(src["rx_deg"]),
        ry_deg=float(src["ry_deg"]),
        rz_deg=float(src["rz_deg"]),
    )


def list_available_dataset_indices(dataset_root: Path) -> List[int]:
    if not dataset_root.exists():
        return []
    result: List[int] = []
    for p in dataset_root.iterdir():
        if p.is_dir() and p.name.isdigit():
            result.append(int(p.name))
    return sorted(result)


def matrix_to_tuple_literal(matrix: np.ndarray) -> str:
    rows = []
    for row in matrix:
        rows.append("(" + ", ".join(f"{float(v):.6f}" for v in row) + ")")
    return "(\n    " + ",\n    ".join(rows) + ",\n)"


def _parse_optional_float(raw: str) -> Optional[float]:
    token = raw.strip()
    if token == "":
        return None
    return float(token)


def parse_excluded_indices(raw: str) -> Tuple[int, ...]:
    if not raw.strip():
        return ()
    result: List[int] = []
    for chunk in raw.split(","):
        token = chunk.strip()
        if not token:
            continue
        if "-" in token:
            left, right = token.split("-", 1)
            a, b = int(left), int(right)
            lo, hi = min(a, b), max(a, b)
            result.extend(range(lo, hi + 1))
        else:
            result.append(int(token))
    return tuple(sorted(set(result)))


def physical_constraints_to_jsonable(cfg: PhysicalConstraintConfig) -> Dict[str, Any]:
    return asdict(cfg)


# ============================
# 几何与变换
# ============================
def _rot_x(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_y(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rot_z(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def rotation_matrix_from_euler(
    rx_deg: float, ry_deg: float, rz_deg: float, formula: str
) -> np.ndarray:
    rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
    mats = {"Rx": _rot_x(rx), "Ry": _rot_y(ry), "Rz": _rot_z(rz)}
    tokens = [formula[i : i + 2] for i in range(0, len(formula), 2)]
    if sorted(tokens) != ["Rx", "Ry", "Rz"]:
        raise ValueError(f"非法旋转公式: {formula}")
    R = np.eye(3, dtype=np.float64)
    for token in tokens:
        R = R @ mats[token]
    return R


def quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("非法四元数：范数为 0")
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


def make_transform(R: np.ndarray, t_xyz_m: Sequence[float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t_xyz_m, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -(R.T @ t)
    return T_inv


def robot_pose_to_base_T_flange(
    pose: RobotPose, pose_format: str, formula: str
) -> np.ndarray:
    R = rotation_matrix_from_euler(pose.rx_deg, pose.ry_deg, pose.rz_deg, formula)
    t = np.array([pose.x_mm, pose.y_mm, pose.z_mm], dtype=np.float64) / 1000.0
    T = make_transform(R, t)
    if pose_format == "BASE_T_FLANGE":
        return T
    if pose_format == "FLANGE_T_BASE":
        return invert_transform(T)
    raise ValueError(f"未知 pose_format: {pose_format}")


def rotation_angle_deg(R: np.ndarray) -> float:
    trace = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(trace)))


def load_pose_json(
    path: Path,
    default_pose_format: str,
    default_formula: str,
) -> LoadedPose:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)

    pose_format = (
        payload.get("pose_format_hint")
        or payload.get("pose_format")
        or default_pose_format
    )
    source = str(
        payload.get("pose_source") or payload.get("camera_backend") or "unknown"
    )

    matrix_payload = payload.get("transform_matrix")
    if matrix_payload is None:
        matrix_payload = payload.get("T_base_to_flange")
    if matrix_payload is None:
        matrix_payload = payload.get("base_T_flange")

    if matrix_payload is not None:
        T = np.asarray(matrix_payload, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError(f"pose.json 中的 transform_matrix 不是 4x4: {path}")
        translation = tuple(float(v) for v in T[:3, 3])
        quat = payload.get("quaternion_xyzw")
        quat_tuple = (
            tuple(float(v) for v in quat[:4])
            if isinstance(quat, (list, tuple)) and len(quat) >= 4
            else None
        )
        return LoadedPose(
            T_base_to_flange=T,
            pose_format=pose_format,
            source=source,
            robot_pose=None,
            translation_m=translation,
            quaternion_xyzw=quat_tuple,
        )

    translation = payload.get("translation_m")
    quat = payload.get("quaternion_xyzw")
    if (
        isinstance(translation, (list, tuple))
        and len(translation) >= 3
        and isinstance(quat, (list, tuple))
        and len(quat) >= 4
    ):
        tx, ty, tz = [float(v) for v in translation[:3]]
        qx, qy, qz, qw = [float(v) for v in quat[:4]]
        T = make_transform(quaternion_to_rotation_matrix(qx, qy, qz, qw), [tx, ty, tz])
        return LoadedPose(
            T_base_to_flange=T,
            pose_format=pose_format,
            source=source,
            robot_pose=None,
            translation_m=(tx, ty, tz),
            quaternion_xyzw=(qx, qy, qz, qw),
        )

    pose = dict_to_pose(payload)
    T = robot_pose_to_base_T_flange(pose, pose_format, default_formula)
    return LoadedPose(
        T_base_to_flange=T,
        pose_format=pose_format,
        source=source or "legacy_pose",
        robot_pose=pose,
        translation_m=tuple(float(v) for v in T[:3, 3]),
        quaternion_xyzw=None,
    )


# ============================
# 棋盘格检测
# ============================
def _build_board_object_points(board_cfg: BoardConfig) -> np.ndarray:
    cols, rows = board_cfg.chessboard_size
    obj = np.zeros((cols * rows, 3), dtype=np.float64)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= board_cfg.square_size_m
    return obj


def detect_board_pose(image_bgr: np.ndarray, board_cfg: BoardConfig) -> BoardDetection:
    if image_bgr is None:
        return BoardDetection(ok=False, message="空图像")

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, board_cfg.chessboard_size, FIND_CORNERS_FLAGS
    )
    if not found:
        return BoardDetection(
            ok=False,
            message="未检测到棋盘格角点",
            image_size=(image_bgr.shape[1], image_bgr.shape[0]),
            annotated_image=image_bgr.copy(),
        )

    corners = cv2.cornerSubPix(gray, corners, SUBPIX_WIN, (-1, -1), SUBPIX_CRITERIA)
    objp = _build_board_object_points(board_cfg)
    ok, rvec, tvec = cv2.solvePnP(
        objp, corners, board_cfg.camera_matrix, board_cfg.dist_coeffs
    )
    if not ok:
        return BoardDetection(
            ok=False, message="solvePnP 失败", corners_count=len(corners)
        )

    reproj, _ = cv2.projectPoints(
        objp, rvec, tvec, board_cfg.camera_matrix, board_cfg.dist_coeffs
    )
    reproj = reproj.reshape(-1, 2)
    corners_2d = corners.reshape(-1, 2)
    err = float(np.sqrt(np.mean(np.sum((reproj - corners_2d) ** 2, axis=1))))

    R_board_to_cam, _ = cv2.Rodrigues(rvec)
    T_board_to_cam = make_transform(R_board_to_cam, tvec.reshape(3))

    annotated = image_bgr.copy()
    cv2.drawChessboardCorners(annotated, board_cfg.chessboard_size, corners, True)
    try:
        cv2.drawFrameAxes(
            annotated,
            board_cfg.camera_matrix,
            board_cfg.dist_coeffs,
            rvec,
            tvec,
            board_cfg.square_size_m * 2.0,
            2,
        )
    except Exception:
        pass

    return BoardDetection(
        ok=True,
        message="ok",
        corners_count=len(corners),
        reprojection_error_px=err,
        image_size=(image_bgr.shape[1], image_bgr.shape[0]),
        rvec=rvec.reshape(3, 1),
        tvec=tvec.reshape(3, 1),
        T_board_to_cam=T_board_to_cam,
        corners=corners,
        annotated_image=annotated,
    )


# ============================
# 样本加载与标定
# ============================
def load_sample_record(
    dataset_root: Path,
    index: int,
    cfg: HandEyeConfig,
    pose_format_override: Optional[str] = None,
    rotation_formula: Optional[str] = None,
) -> Optional[SampleRecord]:
    if index in set(cfg.excluded_indices):
        return None

    sample_dir = dataset_root / str(index)
    image_path = sample_dir / DEFAULT_IMAGE_NAME
    pose_path = sample_dir / DEFAULT_POSE_NAME
    if not image_path.exists() or not pose_path.exists():
        return None

    image = cv2.imread(str(image_path))
    if image is None:
        return None

    formula = rotation_formula or cfg.rotation_formula
    loaded_pose = load_pose_json(
        pose_path, pose_format_override or cfg.pose_format, formula
    )
    detection = detect_board_pose(image, cfg.board)
    if not detection.ok:
        return None

    if (
        cfg.max_reprojection_error_px is not None
        and detection.reprojection_error_px is not None
        and detection.reprojection_error_px > cfg.max_reprojection_error_px
    ):
        return None

    return SampleRecord(
        index=index,
        image_path=image_path,
        pose_path=pose_path,
        robot_pose=loaded_pose.robot_pose,
        T_base_to_flange=loaded_pose.T_base_to_flange,
        detection=detection,
        pose_source=loaded_pose.source,
        pose_format=loaded_pose.pose_format,
    )


def _match_sign(value: float, sign_rule: str) -> bool:
    eps = 1e-12
    if sign_rule == SIGN_ANY:
        return True
    if sign_rule == SIGN_POSITIVE:
        return value > eps
    if sign_rule == SIGN_NEGATIVE:
        return value < -eps
    raise ValueError(f"未知 sign_rule: {sign_rule}")


def _range_penalty(
    axis_name: str,
    value: float,
    min_v: Optional[float],
    max_v: Optional[float],
    violations: List[str],
) -> float:
    penalty = 0.0
    if min_v is not None and value < min_v:
        delta = min_v - value
        violations.append(f"{axis_name}={value:.6f} < {min_v:.6f}")
        penalty += float(delta * 1000.0)
    if max_v is not None and value > max_v:
        delta = value - max_v
        violations.append(f"{axis_name}={value:.6f} > {max_v:.6f}")
        penalty += float(delta * 1000.0)
    return penalty


def evaluate_candidate_constraints(
    T_cam_to_flange: np.ndarray,
    cfg: PhysicalConstraintConfig,
) -> ConstraintEvaluation:
    translation = T_cam_to_flange[:3, 3].reshape(3)
    z_axis = T_cam_to_flange[:3, 2].reshape(3)
    z_axis_norm = float(np.linalg.norm(z_axis))
    if z_axis_norm <= 1e-12:
        z_axis_cos = -1.0
    else:
        z_axis_cos = float(z_axis[2] / z_axis_norm)

    if not cfg.enabled:
        return ConstraintEvaluation(
            mode=cfg.mode,
            passed=True,
            hard_failed=False,
            penalty=0.0,
            violations=[],
            translation_m=(float(translation[0]), float(translation[1]), float(translation[2])),
            z_axis_in_flange=(float(z_axis[0]), float(z_axis[1]), float(z_axis[2])),
            z_axis_cos_to_flange_z=z_axis_cos,
        )

    violations: List[str] = []
    penalty = 0.0
    tx, ty, tz = [float(v) for v in translation]

    for axis_name, value, sign_rule in (
        ("tx", tx, cfg.tx_sign),
        ("ty", ty, cfg.ty_sign),
        ("tz", tz, cfg.tz_sign),
    ):
        if not _match_sign(value, sign_rule):
            violations.append(f"{axis_name} 符号不满足 {sign_rule}: {value:.6f}")
            penalty += abs(value) * 1000.0 + 1.0

    penalty += _range_penalty("tx", tx, cfg.tx_min_m, cfg.tx_max_m, violations)
    penalty += _range_penalty("ty", ty, cfg.ty_min_m, cfg.ty_max_m, violations)
    penalty += _range_penalty("tz", tz, cfg.tz_min_m, cfg.tz_max_m, violations)

    if cfg.z_axis_same_direction and z_axis_cos < cfg.z_axis_cos_min:
        violations.append(
            f"cam z 与 flange z 夹角约束失败: cos={z_axis_cos:.6f} < {cfg.z_axis_cos_min:.6f}"
        )
        penalty += float((cfg.z_axis_cos_min - z_axis_cos) * 1000.0 + 10.0)

    passed = len(violations) == 0
    hard_failed = cfg.mode == CONSTRAINT_MODE_HARD and not passed
    return ConstraintEvaluation(
        mode=cfg.mode,
        passed=passed,
        hard_failed=hard_failed,
        penalty=float(penalty),
        violations=violations,
        translation_m=(tx, ty, tz),
        z_axis_in_flange=(float(z_axis[0]), float(z_axis[1]), float(z_axis[2])),
        z_axis_cos_to_flange_z=z_axis_cos,
    )


def _compute_candidate_metrics(
    records: Sequence[SampleRecord], T_cam_to_flange: np.ndarray
) -> Tuple[
    CalibrationMetrics,
    List[Tuple[float, float, float]],
    List[float],
    List[Dict[str, Any]],
]:
    translations: List[np.ndarray] = []
    board_rotations: List[np.ndarray] = []

    for record in records:
        T_base_to_board = (
            record.T_base_to_flange @ T_cam_to_flange @ record.detection.T_board_to_cam
        )
        translations.append(T_base_to_board[:3, 3].copy())
        board_rotations.append(T_base_to_board[:3, :3].copy())

    trans_stack = np.vstack(translations)
    trans_mean = np.mean(trans_stack, axis=0)
    dists = np.linalg.norm(trans_stack - trans_mean[None, :], axis=1)
    rms = float(np.sqrt(np.mean(np.square(dists))))
    max_dist = float(np.max(dists))

    first_R = board_rotations[0]
    rot_angles = [rotation_angle_deg(first_R.T @ R) for R in board_rotations]
    rot_max = float(np.max(rot_angles)) if rot_angles else 0.0

    metrics = CalibrationMetrics(
        translation_mean_m=(
            float(trans_mean[0]),
            float(trans_mean[1]),
            float(trans_mean[2]),
        ),
        translation_rms_to_mean_m=rms,
        translation_max_to_mean_m=max_dist,
        rotation_max_to_first_deg=rot_max,
        valid_count=len(records),
    )

    trans_list = [(float(t[0]), float(t[1]), float(t[2])) for t in translations]
    rot_list = [float(v) for v in rot_angles]
    diagnostics: List[Dict[str, Any]] = []
    for record, t_vec, dist_m, rot_deg in zip(records, translations, dists, rot_angles):
        diagnostics.append(
            {
                "index": record.index,
                "pose_source": record.pose_source,
                "reprojection_error_px": (
                    None
                    if record.detection.reprojection_error_px is None
                    else float(record.detection.reprojection_error_px)
                ),
                "board_to_base_translation_m": [float(v) for v in t_vec],
                "translation_deviation_to_mean_m": float(dist_m),
                "translation_deviation_to_mean_mm": float(dist_m * 1000.0),
                "rotation_deviation_to_first_deg": float(rot_deg),
            }
        )
    diagnostics.sort(
        key=lambda x: (
            x["translation_deviation_to_mean_m"],
            x["rotation_deviation_to_first_deg"],
        ),
        reverse=True,
    )
    return metrics, trans_list, rot_list, diagnostics


def _calibrate_one_combo(
    records: Sequence[SampleRecord],
    method_name: str,
    rotation_formula: str,
    pose_format: str,
    physical_constraints: PhysicalConstraintConfig,
) -> CalibrationCandidate:
    R_gripper2base = [r.T_base_to_flange[:3, :3] for r in records]
    t_gripper2base = [r.T_base_to_flange[:3, 3].reshape(3, 1) for r in records]
    R_target2cam = [r.detection.T_board_to_cam[:3, :3] for r in records]
    t_target2cam = [r.detection.T_board_to_cam[:3, 3].reshape(3, 1) for r in records]

    method_flag = HAND_EYE_METHODS[method_name]
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=method_flag,
    )
    T_cam_to_flange = make_transform(R_cam2gripper, t_cam2gripper.reshape(3))
    metrics, translations, rot_list, diagnostics = _compute_candidate_metrics(
        records, T_cam_to_flange
    )
    constraint_eval = evaluate_candidate_constraints(T_cam_to_flange, physical_constraints)
    return CalibrationCandidate(
        method_name=method_name,
        rotation_formula=rotation_formula,
        pose_format=pose_format,
        T_cam_to_flange=T_cam_to_flange,
        metrics=metrics,
        sample_indices=[r.index for r in records],
        board_to_base_translations_m=translations,
        board_to_base_rotations_deg_to_first=rot_list,
        sample_diagnostics=diagnostics,
        pruned_indices=[],
        constraint_eval=constraint_eval,
    )


def _rank_key(candidate: CalibrationCandidate) -> Tuple[float, float, float, float, float]:
    failed_flag = 0.0 if candidate.constraint_eval.passed else 1.0
    return (
        failed_flag,
        float(candidate.constraint_eval.penalty),
        candidate.metrics.translation_rms_to_mean_m,
        candidate.metrics.translation_max_to_mean_m,
        candidate.metrics.rotation_max_to_first_deg,
    )


def _candidate_improved(
    old: CalibrationCandidate,
    new: CalibrationCandidate,
    cfg: HandEyeConfig,
) -> bool:
    if _rank_key(new) >= _rank_key(old):
        return False
    rms_gain = (
        old.metrics.translation_rms_to_mean_m - new.metrics.translation_rms_to_mean_m
    )
    max_gain = (
        old.metrics.translation_max_to_mean_m - new.metrics.translation_max_to_mean_m
    )
    passed_upgrade = (
        not old.constraint_eval.passed and new.constraint_eval.passed
    )
    penalty_gain = old.constraint_eval.penalty - new.constraint_eval.penalty
    return (
        passed_upgrade
        or penalty_gain > 1e-9
        or rms_gain >= cfg.min_rms_improvement_m
        or max_gain >= cfg.min_max_dist_improvement_m
    )


def _auto_prune_candidate(
    records: Sequence[SampleRecord],
    method_name: str,
    rotation_formula: str,
    pose_format: str,
    cfg: HandEyeConfig,
) -> CalibrationCandidate:
    current_records = list(records)
    best_candidate = _calibrate_one_combo(
        current_records,
        method_name,
        rotation_formula,
        pose_format,
        cfg.physical_constraints,
    )
    pruned: List[int] = []

    if cfg.physical_constraints.mode == CONSTRAINT_MODE_HARD and best_candidate.constraint_eval.hard_failed:
        raise ValueError(
            f"候选 {method_name}/{rotation_formula} 不满足硬约束: {best_candidate.constraint_eval.violations}"
        )

    if not cfg.auto_prune_outliers:
        best_candidate.pruned_indices = pruned
        return best_candidate

    for _ in range(cfg.max_prune_rounds):
        if len(current_records) <= cfg.min_valid_samples:
            break
        if not best_candidate.sample_diagnostics:
            break

        worst_index = int(best_candidate.sample_diagnostics[0]["index"])
        next_records = [r for r in current_records if r.index != worst_index]
        if len(next_records) < cfg.min_valid_samples:
            break
        try:
            next_candidate = _calibrate_one_combo(
                next_records,
                method_name,
                rotation_formula,
                pose_format,
                cfg.physical_constraints,
            )
        except cv2.error:
            break
        except ValueError:
            break
        if (
            cfg.physical_constraints.mode == CONSTRAINT_MODE_HARD
            and next_candidate.constraint_eval.hard_failed
        ):
            break
        if not _candidate_improved(best_candidate, next_candidate, cfg):
            break

        pruned.append(worst_index)
        current_records = next_records
        best_candidate = next_candidate

    best_candidate.pruned_indices = pruned
    return best_candidate


def calibrate_from_dataset(
    dataset_root: Path,
    min_index: int,
    max_index: int,
    cfg: HandEyeConfig,
    auto_search_formulas: bool = AUTO_SEARCH_FORMULAS,
    auto_search_methods: bool = AUTO_SEARCH_METHODS,
    rotation_formula: str = DEFAULT_ROTATION_FORMULA,
    method_name: str = DEFAULT_HAND_EYE_METHOD,
    pose_format: Optional[str] = None,
) -> Tuple[
    CalibrationCandidate, List[CalibrationCandidate], List[SampleRecord], List[int]
]:
    if min_index > max_index:
        raise ValueError("min_index 不能大于 max_index")

    formula_candidates = (
        ROTATION_FORMULA_CANDIDATES if auto_search_formulas else [rotation_formula]
    )
    method_candidates = (
        list(HAND_EYE_METHODS.keys()) if auto_search_methods else [method_name]
    )
    pose_format_final = pose_format or cfg.pose_format

    best_candidate: Optional[CalibrationCandidate] = None
    ranked: List[CalibrationCandidate] = []
    best_records: List[SampleRecord] = []
    missing_or_invalid: List[int] = []
    hard_fail_messages: List[str] = []

    for formula in formula_candidates:
        records: List[SampleRecord] = []
        invalid_local: List[int] = []
        excluded_set = set(cfg.excluded_indices)
        for idx in range(min_index, max_index + 1):
            if idx in excluded_set:
                invalid_local.append(idx)
                continue
            record = load_sample_record(
                dataset_root,
                idx,
                cfg,
                pose_format_override=pose_format_final,
                rotation_formula=formula,
            )
            if record is None:
                invalid_local.append(idx)
                continue
            records.append(record)

        if len(records) < max(
            3, cfg.min_valid_samples if cfg.auto_prune_outliers else 3
        ):
            continue

        for m in method_candidates:
            try:
                candidate = _auto_prune_candidate(
                    records, m, formula, pose_format_final, cfg
                )
                ranked.append(candidate)
                if best_candidate is None or _rank_key(candidate) < _rank_key(
                    best_candidate
                ):
                    best_candidate = candidate
                    best_records = [
                        r for r in records if r.index in set(candidate.sample_indices)
                    ]
                    missing_or_invalid = invalid_local
            except cv2.error:
                continue
            except ValueError as exc:
                hard_fail_messages.append(str(exc))
                continue

    if best_candidate is None:
        suffix = ""
        if cfg.physical_constraints.mode == CONSTRAINT_MODE_HARD and hard_fail_messages:
            suffix = "；当前硬约束过于严格，所有候选都被过滤"
        raise ValueError(
            "在当前数据集上无法得到有效 hand-eye 结果，请检查样本数量、棋盘格检测和位姿定义" + suffix
        )

    ranked.sort(key=_rank_key)
    return best_candidate, ranked, best_records, missing_or_invalid


# ============================
# 输出结果
# ============================
def candidate_to_jsonable(candidate: CalibrationCandidate) -> Dict[str, Any]:
    return {
        "method_name": candidate.method_name,
        "rotation_formula": candidate.rotation_formula,
        "pose_format": candidate.pose_format,
        "sample_indices": candidate.sample_indices,
        "pruned_indices": candidate.pruned_indices,
        "T_cam_to_flange": candidate.T_cam_to_flange.tolist(),
        "T_cam_to_flange_tuple_literal": matrix_to_tuple_literal(
            candidate.T_cam_to_flange
        ),
        "metrics": asdict(candidate.metrics),
        "board_to_base_translations_m": candidate.board_to_base_translations_m,
        "board_to_base_rotations_deg_to_first": candidate.board_to_base_rotations_deg_to_first,
        "sample_diagnostics": candidate.sample_diagnostics,
        "constraint_eval": asdict(candidate.constraint_eval),
    }


def save_calibration_outputs(
    best: CalibrationCandidate,
    ranked: Sequence[CalibrationCandidate],
    dataset_root: Path,
    min_index: int,
    max_index: int,
    output_root: Path,
    invalid_indices: Optional[Sequence[int]] = None,
    cfg: Optional[HandEyeConfig] = None,
) -> Path:
    ts = datetime.now().strftime("handeye_%Y%m%d_%H%M%S")
    out_dir = ensure_dir(output_root / ts)

    summary = {
        "dataset_root": str(dataset_root),
        "index_range": {"min": min_index, "max": max_index},
        "invalid_or_missing_indices": list(invalid_indices or []),
        "config": {
            "pose_format": None if cfg is None else cfg.pose_format,
            "rotation_formula": None if cfg is None else cfg.rotation_formula,
            "max_reprojection_error_px": None if cfg is None else cfg.max_reprojection_error_px,
            "auto_prune_outliers": None if cfg is None else cfg.auto_prune_outliers,
            "min_valid_samples": None if cfg is None else cfg.min_valid_samples,
            "max_prune_rounds": None if cfg is None else cfg.max_prune_rounds,
            "min_rms_improvement_m": None if cfg is None else cfg.min_rms_improvement_m,
            "min_max_dist_improvement_m": None if cfg is None else cfg.min_max_dist_improvement_m,
            "physical_constraints": None
            if cfg is None
            else physical_constraints_to_jsonable(cfg.physical_constraints),
        },
        "best": candidate_to_jsonable(best),
        "ranked_count": len(ranked),
    }
    write_json(out_dir / "best_result.json", summary)
    write_json(
        out_dir / "ranked_candidates.json",
        {"candidates": [candidate_to_jsonable(c) for c in ranked]},
    )
    write_json(
        out_dir / "sample_diagnostics.json",
        {
            "best_method": best.method_name,
            "best_rotation_formula": best.rotation_formula,
            "pruned_indices": best.pruned_indices,
            "constraint_eval": asdict(best.constraint_eval),
            "samples": best.sample_diagnostics,
        },
    )

    lines = [
        f"dataset_root: {dataset_root}",
        f"index_range: [{min_index}, {max_index}]",
        f"invalid_or_missing_indices: {list(invalid_indices or [])}",
        f"best_method: {best.method_name}",
        f"best_rotation_formula: {best.rotation_formula}",
        f"pose_format: {best.pose_format}",
        f"valid_count: {best.metrics.valid_count}",
        f"pruned_indices: {best.pruned_indices}",
        f"translation_rms_to_mean_m: {best.metrics.translation_rms_to_mean_m:.6f}",
        f"translation_max_to_mean_m: {best.metrics.translation_max_to_mean_m:.6f}",
        f"rotation_max_to_first_deg: {best.metrics.rotation_max_to_first_deg:.6f}",
        f"constraint_mode: {best.constraint_eval.mode}",
        f"constraint_passed: {best.constraint_eval.passed}",
        f"constraint_penalty: {best.constraint_eval.penalty:.6f}",
        f"constraint_violations: {best.constraint_eval.violations}",
        "",
        "^flange T_cam =",
        np.array2string(best.T_cam_to_flange, precision=6, suppress_small=False),
        "",
        "适合直接粘贴到 Python 常量里的 tuple 形式:",
        matrix_to_tuple_literal(best.T_cam_to_flange),
    ]
    (out_dir / "summary.txt").write_text("\n".join(lines), encoding="utf-8")
    return out_dir


# ============================
# 命令行入口
# ============================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="基于 ./picture/<index>/ 的手眼标定脚本（支持可选物理约束）"
    )
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--min-index", type=int, required=True)
    parser.add_argument("--max-index", type=int, required=True)
    parser.add_argument(
        "--pose-format",
        choices=["BASE_T_FLANGE", "FLANGE_T_BASE"],
        default=ROBOT_POSE_FORMAT,
    )
    parser.add_argument(
        "--rotation-formula",
        choices=ROTATION_FORMULA_CANDIDATES,
        default=DEFAULT_ROTATION_FORMULA,
    )
    parser.add_argument(
        "--method",
        choices=list(HAND_EYE_METHODS.keys()),
        default=DEFAULT_HAND_EYE_METHOD,
    )
    parser.add_argument(
        "--auto-formulas", action="store_true", default=AUTO_SEARCH_FORMULAS
    )
    parser.add_argument(
        "--auto-methods", action="store_true", default=AUTO_SEARCH_METHODS
    )
    parser.add_argument(
        "--max-reproj-error-px", type=float, default=MAX_REPROJECTION_ERROR_PX
    )
    parser.add_argument("--disable-auto-prune", action="store_true")
    parser.add_argument("--exclude-indices", type=str, default="")
    parser.add_argument(
        "--constraint-mode",
        choices=CONSTRAINT_MODES,
        default=CONSTRAINT_MODE_DISABLED,
    )
    parser.add_argument("--tx-sign", choices=SIGN_OPTIONS, default=SIGN_ANY)
    parser.add_argument("--ty-sign", choices=SIGN_OPTIONS, default=SIGN_ANY)
    parser.add_argument("--tz-sign", choices=SIGN_OPTIONS, default=SIGN_ANY)
    parser.add_argument("--tx-min-m", type=float, default=None)
    parser.add_argument("--tx-max-m", type=float, default=None)
    parser.add_argument("--ty-min-m", type=float, default=None)
    parser.add_argument("--ty-max-m", type=float, default=None)
    parser.add_argument("--tz-min-m", type=float, default=None)
    parser.add_argument("--tz-max-m", type=float, default=None)
    parser.add_argument("--z-axis-same-direction", action="store_true")
    parser.add_argument("--z-axis-cos-min", type=float, default=DEFAULT_Z_AXIS_COS_MIN)
    args = parser.parse_args()

    cfg = HandEyeConfig(
        dataset_root=args.dataset_root,
        output_root=args.output_root,
        pose_format=args.pose_format,
        rotation_formula=args.rotation_formula,
        board=BoardConfig(
            camera_matrix=CAMERA_MATRIX.copy(), dist_coeffs=DIST_COEFFS.copy()
        ),
        max_reprojection_error_px=args.max_reproj_error_px,
        auto_prune_outliers=not args.disable_auto_prune,
        excluded_indices=parse_excluded_indices(args.exclude_indices),
        physical_constraints=PhysicalConstraintConfig(
            mode=args.constraint_mode,
            tx_sign=args.tx_sign,
            ty_sign=args.ty_sign,
            tz_sign=args.tz_sign,
            tx_min_m=args.tx_min_m,
            tx_max_m=args.tx_max_m,
            ty_min_m=args.ty_min_m,
            ty_max_m=args.ty_max_m,
            tz_min_m=args.tz_min_m,
            tz_max_m=args.tz_max_m,
            z_axis_same_direction=args.z_axis_same_direction,
            z_axis_cos_min=args.z_axis_cos_min,
        ),
    )
    best, ranked, records, invalid = calibrate_from_dataset(
        dataset_root=args.dataset_root,
        min_index=args.min_index,
        max_index=args.max_index,
        cfg=cfg,
        auto_search_formulas=args.auto_formulas,
        auto_search_methods=args.auto_methods,
        rotation_formula=args.rotation_formula,
        method_name=args.method,
        pose_format=args.pose_format,
    )
    out_dir = save_calibration_outputs(
        best,
        ranked,
        args.dataset_root,
        args.min_index,
        args.max_index,
        args.output_root,
        invalid,
        cfg=cfg,
    )

    print(f"有效样本数: {len(records)}")
    print(f"无效/缺失样本索引: {invalid}")
    print(f"最佳组合: method={best.method_name}, formula={best.rotation_formula}")
    print(f"被自动剔除的样本: {best.pruned_indices}")
    print(f"约束模式: {best.constraint_eval.mode}")
    print(f"约束是否满足: {best.constraint_eval.passed}")
    print(f"约束违例: {best.constraint_eval.violations}")
    print(f"平移 RMS: {best.metrics.translation_rms_to_mean_m:.6f} m")
    print(f"最大平移误差: {best.metrics.translation_max_to_mean_m:.6f} m")
    print("^flange T_cam =")
    print(np.array2string(best.T_cam_to_flange, precision=6, suppress_small=False))
    print(f"输出目录: {out_dir}")


if __name__ == "__main__":
    main()
