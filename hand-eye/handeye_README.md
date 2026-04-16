# HandEye 采集与标定说明

## 文件
- `pipper_HandEye.py`：重构后的手眼标定核心脚本
- `handeye_capture_gui.py`：采集与标定 GUI

## GUI 启动
```bash
python handeye_capture_gui.py
```

GUI 特性：
- 启动后自动打开相机，左侧显示视频
- 启动后自动最大化
- `F11` 可切换最大化/普通窗口
- 右侧输入当前机械臂位姿
- 位姿语义默认要求为 `^base T_flange`
- 数据集选择为连续区间 `[min_index, max_index]`

## 采集目录结构
每个样本存放在：
```text
./picture/<index>/
```
目录下包含：
- `image.jpg`
- `pose.json`
- `board_detection.json`
- `annotated.jpg`

## 命令行标定
```bash
python pipper_HandEye.py --min-index 1 --max-index 50
```

可选项：
```bash
python pipper_HandEye.py \
  --dataset-root ./picture \
  --output-root ./outputs \
  --min-index 1 \
  --max-index 50 \
  --pose-format BASE_T_FLANGE \
  --rotation-formula RzRyRx \
  --method TSAI
```

输出写入：
```text
./outputs/handeye_时间戳/
```
包含：
- `best_result.json`
- `ranked_candidates.json`
- `summary.txt`

## 需要重点确认
最关键的顶部常量在 `pipper_HandEye.py` 开头：
- `ROBOT_POSE_FORMAT`
- `DEFAULT_ROTATION_FORMULA`
- `AUTO_SEARCH_FORMULAS`
- `AUTO_SEARCH_METHODS`
