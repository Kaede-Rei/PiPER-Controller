from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ============================
# 顶部统一配置区
# ============================
# 数据目录
DATASET_ROOT = Path("./picture")
OUTPUT_ROOT = Path("./outputs")

# 数据文件名
DEFAULT_IMAGE_NAME = "image.jpg"
DEFAULT_POSE_NAME = "pose.json"
DEFAULT_DETECTION_NAME = "board_detection.json"
DEFAULT_ANNOTATED_NAME = "annotated.jpg"

# 相机内参
CAMERA_MATRIX = np.array(
    [
        [611.8829, 0.0, 639.6838],
        [0.0, 612.0766, 363.2091],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

# 这里按 OpenCV 顺序传入，允许 4/5/8 项
DIST_COEFFS = np.array([-0.02141041, 0.01469840, 0.0, 0.0], dtype=np.float64)

# 棋盘格参数
CHESSBOARD_SIZE = (11, 8)  # 内角点 (列, 行)
SQUARE_SIZE_M = 0.02

# 机器人位姿的语义：
# - "BASE_T_FLANGE": 输入的是 ^base T_flange
# - "FLANGE_T_BASE": 输入的是 ^flange T_base
ROBOT_POSE_FORMAT = "BASE_T_FLANGE"

# 欧拉角公式候选。
# 名字按最终矩阵乘法顺序写，例如 "RzRyRx" 表示 R = Rz @ Ry @ Rx。
ROTATION_FORMULA_CANDIDATES = [
    "RzRyRx",
    "RxRyRz",
    "RyRxRz",
    "RzRxRy",
    "RxRzRy",
    "RyRzRx",
]
DEFAULT_ROTATION_FORMULA = "RzRyRx"

# 是否自动搜索欧拉角公式与 hand-eye 方法
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

# OpenCV hand-eye 方法
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
class HandEyeConfig:
    dataset_root: Path = DATASET_ROOT
    output_root: Path = OUTPUT_ROOT
    pose_format: str = ROBOT_POSE_FORMAT
    rotation_formula: str = DEFAULT_ROTATION_FORMULA
    board: BoardConfig = field(default_factory=BoardConfig)


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


@dataclass(frozen=True)
class CalibrationMetrics:
    translation_mean_m: Tuple[float, float, float]
    translation_rms_to_mean_m: float
    translation_max_to_mean_m: float
    rotation_max_to_first_deg: float
    valid_count: int


@dataclass
class SampleRecord:
    index: int
    image_path: Path
    pose_path: Path
    robot_pose: RobotPose
    T_base_to_flange: np.ndarray
    detection: BoardDetection


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


def load_pose_json(path: Path) -> Tuple[RobotPose, Optional[str]]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    pose = dict_to_pose(payload)
    pose_format = payload.get("pose_format_hint")
    return pose, pose_format


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


def make_transform(R: np.ndarray, t_xyz_m: Sequence[float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
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
    sample_dir = dataset_root / str(index)
    image_path = sample_dir / DEFAULT_IMAGE_NAME
    pose_path = sample_dir / DEFAULT_POSE_NAME
    if not image_path.exists() or not pose_path.exists():
        return None

    image = cv2.imread(str(image_path))
    if image is None:
        return None

    pose, pose_format_from_file = load_pose_json(pose_path)
    pose_format = pose_format_override or pose_format_from_file or cfg.pose_format
    formula = rotation_formula or cfg.rotation_formula
    T_base_to_flange = robot_pose_to_base_T_flange(pose, pose_format, formula)
    detection = detect_board_pose(image, cfg.board)
    if not detection.ok:
        return None

    return SampleRecord(
        index=index,
        image_path=image_path,
        pose_path=pose_path,
        robot_pose=pose,
        T_base_to_flange=T_base_to_flange,
        detection=detection,
    )


def _compute_candidate_metrics(
    records: Sequence[SampleRecord], T_cam_to_flange: np.ndarray
) -> Tuple[CalibrationMetrics, List[Tuple[float, float, float]], List[float]]:
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
    return metrics, trans_list, rot_list


def _calibrate_one_combo(
    records: Sequence[SampleRecord],
    method_name: str,
    rotation_formula: str,
    pose_format: str,
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
    metrics, translations, rot_list = _compute_candidate_metrics(
        records, T_cam_to_flange
    )
    return CalibrationCandidate(
        method_name=method_name,
        rotation_formula=rotation_formula,
        pose_format=pose_format,
        T_cam_to_flange=T_cam_to_flange,
        metrics=metrics,
        sample_indices=[r.index for r in records],
        board_to_base_translations_m=translations,
        board_to_base_rotations_deg_to_first=rot_list,
    )


def _rank_key(candidate: CalibrationCandidate) -> Tuple[float, float, float]:
    return (
        candidate.metrics.translation_rms_to_mean_m,
        candidate.metrics.translation_max_to_mean_m,
        candidate.metrics.rotation_max_to_first_deg,
    )


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

    for formula in formula_candidates:
        records: List[SampleRecord] = []
        invalid_local: List[int] = []
        for idx in range(min_index, max_index + 1):
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

        if len(records) < 3:
            continue

        for m in method_candidates:
            try:
                candidate = _calibrate_one_combo(records, m, formula, pose_format_final)
                ranked.append(candidate)
                if best_candidate is None or _rank_key(candidate) < _rank_key(
                    best_candidate
                ):
                    best_candidate = candidate
                    best_records = list(records)
                    missing_or_invalid = invalid_local
            except cv2.error:
                continue

    if best_candidate is None:
        raise ValueError(
            "在当前数据集上无法得到有效 hand-eye 结果，请检查样本数量、棋盘格检测和位姿定义"
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
        "T_cam_to_flange": candidate.T_cam_to_flange.tolist(),
        "T_cam_to_flange_tuple_literal": matrix_to_tuple_literal(
            candidate.T_cam_to_flange
        ),
        "metrics": asdict(candidate.metrics),
        "board_to_base_translations_m": candidate.board_to_base_translations_m,
        "board_to_base_rotations_deg_to_first": candidate.board_to_base_rotations_deg_to_first,
    }


def save_calibration_outputs(
    best: CalibrationCandidate,
    ranked: Sequence[CalibrationCandidate],
    dataset_root: Path,
    min_index: int,
    max_index: int,
    output_root: Path,
    invalid_indices: Optional[Sequence[int]] = None,
) -> Path:
    ts = datetime.now().strftime("handeye_%Y%m%d_%H%M%S")
    out_dir = ensure_dir(output_root / ts)

    summary = {
        "dataset_root": str(dataset_root),
        "index_range": {"min": min_index, "max": max_index},
        "invalid_or_missing_indices": list(invalid_indices or []),
        "best": candidate_to_jsonable(best),
        "ranked_count": len(ranked),
    }
    write_json(out_dir / "best_result.json", summary)
    write_json(
        out_dir / "ranked_candidates.json",
        {"candidates": [candidate_to_jsonable(c) for c in ranked]},
    )

    lines = [
        f"dataset_root: {dataset_root}",
        f"index_range: [{min_index}, {max_index}]",
        f"invalid_or_missing_indices: {list(invalid_indices or [])}",
        f"best_method: {best.method_name}",
        f"best_rotation_formula: {best.rotation_formula}",
        f"pose_format: {best.pose_format}",
        f"valid_count: {best.metrics.valid_count}",
        f"translation_rms_to_mean_m: {best.metrics.translation_rms_to_mean_m:.6f}",
        f"translation_max_to_mean_m: {best.metrics.translation_max_to_mean_m:.6f}",
        f"rotation_max_to_first_deg: {best.metrics.rotation_max_to_first_deg:.6f}",
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
        description="基于 ./picture/<index>/ 的手眼标定脚本"
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
    args = parser.parse_args()

    cfg = HandEyeConfig(
        dataset_root=args.dataset_root,
        output_root=args.output_root,
        pose_format=args.pose_format,
        rotation_formula=args.rotation_formula,
        board=BoardConfig(
            camera_matrix=CAMERA_MATRIX.copy(), dist_coeffs=DIST_COEFFS.copy()
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
    )

    print(f"有效样本数: {len(records)}")
    print(f"无效/缺失样本索引: {invalid}")
    print(f"最佳组合: method={best.method_name}, formula={best.rotation_formula}")
    print(f"平移 RMS: {best.metrics.translation_rms_to_mean_m:.6f} m")
    print(f"最大平移误差: {best.metrics.translation_max_to_mean_m:.6f} m")
    print("^flange T_cam =")
    print(np.array2string(best.T_cam_to_flange, precision=6, suppress_small=False))
    print(f"输出目录: {out_dir}")


if __name__ == "__main__":
    main()
