# HandEye 采集与标定说明

## 文件
- `pipper_hand_eye.py`：重构后的手眼标定核心脚本
- `handeye_capture_gui.py`：采集与标定 GUI

## GUI 启动（在工作区根目录执行）
```bash
python ./hand-eye/hand_eye_capture_gui.py
```

GUI 特性：
- 启动后自动打开相机，左侧显示视频
- 启动后自动最大化
- `F11` 可切换最大化/普通窗口
- 右侧输入当前机械臂位姿
- 位姿语义默认要求为 `^base T_flange`
- 数据集选择为连续区间 `[min_index, max_index]`

## GUI 使用说明

### 1) 基本流程
1. 打开 GUI 后，先在 **项目** 页确认 `dataset_root`（默认 `./picture`）和 `output_root`（默认 `./outputs`）。
2. 在 **相机** 页确认分辨率/帧率，必要时点“应用并重启相机”。
3. 在 **ROS/位姿** 页设置位姿来源：
   - 勾选“采样时自动从 TF 读取位姿”：采样时自动写入 TF 位姿。
   - 不勾选：使用下方手动输入的 `x/y/z/rx/ry/rz`。
4. 画面与参数确认后，点击“拍照并保存当前样本”。
5. 重复采样，积累足够样本后，在 **标定** 页设置 `min_index/max_index`，点击“用当前区间做手眼标定”。

### 2) 各分页重点
- **项目**：选择数据目录与输出目录，管理当前保存索引，可一键刷新已存在样本编号。
- **相机**：可切换 Orbbec/OpenCV、设置分辨率与帧率、设置画面旋转 180°。
- **棋盘/内参**：配置棋盘内角点数量、方格尺寸、相机内参和畸变参数。
- **ROS/位姿**：可读取 TF 回填到输入框，也可手动输入位姿。
- **标定**：配置样本区间、方法/公式（支持 AUTO）、离群剔除参数。
- **物理约束**：设置平移符号/范围与 z 轴方向约束（可关闭）。

### 3) 采样后会写入什么
每次“拍照并保存当前样本”会在 `./picture/<index>/` 下写入：
- `image.jpg`：原始图像
- `pose.json`：机械臂位姿（TF 或手动输入）
- `board_detection.json`：棋盘检测与 PnP 结果
- `annotated.jpg`：棋盘角点可视化（检测成功时）

### 4) 标定结果怎么看
- GUI 标定完成后会弹窗提示输出目录，默认为 `./outputs/handeye_时间戳/`。
- 重点看：
  - `best_result.json`：最优结果（含 `^flange T_cam`）
  - `summary.txt`：可直接阅读的摘要
  - `sample_diagnostics.json`：逐样本诊断
  - `ranked_candidates.json`：候选结果排序

### 5) 快捷键
- `F11`：切换最大化/普通窗口

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
python ./hand-eye/pipper_hand_eye.py --min-index 1 --max-index 50
```

可选项：
```bash
python ./hand-eye/pipper_hand_eye.py \
  --dataset-root ./picture \
  --output-root ./outputs \
  --min-index 1 \
  --max-index 50 \
  --pose-format BASE_T_FLANGE \
  --rotation-formula RzRyRx \
  --method TSAI
```

常用补充参数：
- `--exclude-indices 3,8,13`：排除指定样本
- `--max-reproj-error-px 0.5`：重投影误差阈值（像素）
- `--disable-auto-prune`：关闭自动剔除异常样本
- `--auto-formulas --auto-methods`：自动搜索旋转公式与标定方法组合

输出写入：
```text
./outputs/handeye_时间戳/
```
包含：
- `best_result.json`
- `ranked_candidates.json`
- `sample_diagnostics.json`
- `summary.txt`

## 需要重点确认
最关键的顶部常量在 `pipper_hand_eye.py` 开头：
- `ROBOT_POSE_FORMAT`
- `DEFAULT_ROTATION_FORMULA`
- `AUTO_SEARCH_FORMULAS`
- `AUTO_SEARCH_METHODS`
