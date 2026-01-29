# 标定结果验收验证脚本方案

## 功能概述

开发一个独立脚本，从S3下载所有设备的标定结果，验证是否符合验收标准，并生成Excel报告。

## 功能需求

1. **从S3获取所有设备列表**
   - 列出S3 bucket中所有包含 `v1/results` 的设备
   - 使用 `s3sdk` 的 `list_folders` 方法

2. **下载和分析流程**
   - 对每个设备：
     - 下载 `{device_id}/v1/results/` 目录到临时目录
     - 分析该设备的所有内外参文件
     - 验证是否符合验收标准
     - 分析完成后删除临时下载目录

3. **验收标准验证**
   - **内参验证**：
     - Projection: fx[620-660], fy[620-660], cx[920-1000], cy[500-580]
     - Distortion: k1[0.01-0.07], k2[-0.06-0.06], p1[-0.06-0.06], p2[-0.04-0.04]
   - **外参验证**：
     - Reprojection error < 1.5 (每个相机)
     - Gyroscope error mean/median < 0.015 (每个IMU)
     - Accelerometer error mean/median < 0.12 (每个IMU)

4. **Excel报告生成**
   - 创建Excel文件，包含两个sheet：
     - **"Passed"**: 通过验收的设备列表
     - **"Failed"**: 未通过验收的设备列表
   - 每个设备记录详细信息（设备ID、标定类型、失败原因等）

## 文件结构

```
scripts/
├── calibration_validation.py          # 主脚本文件
└── calibration_validation_plan.md    # 本方案文档
```

## 技术实现方案

### 1. 主脚本结构

**文件**: `scripts/calibration_validation.py`

**主要类/函数**:
- `CalibrationValidator`: 验证器类
  - `validate_intrinsic()`: 验证内参
  - `validate_extrinsic()`: 验证外参
- `S3ResultsAnalyzer`: S3结果分析器类
  - `list_all_devices()`: 列出所有设备
  - `download_and_analyze_device()`: 下载并分析单个设备
  - `analyze_all_devices()`: 分析所有设备
- `ExcelReporter`: Excel报告生成器类
  - `create_report()`: 创建Excel报告
  - `add_passed_device()`: 添加通过的设备
  - `add_failed_device()`: 添加失败的设备

### 2. 依赖模块

- `s3sdk`: S3 SDK（已存在）
- `intrinsic_result_parser`: 内参结果解析（已存在）
- `extrinsic_result_parser`: 外参结果解析（已存在）
- `openpyxl` 或 `pandas`: Excel文件生成

### 3. 工作流程

```
1. 初始化S3 SDK
2. 列出所有设备（通过list_folders查找包含v1/results的设备）
3. 对每个设备：
   a. 创建临时下载目录
   b. 下载 {device_id}/v1/results/ 目录
   c. 查找并解析所有内外参结果文件
   d. 验证每个标定结果是否符合标准
   e. 记录验证结果（通过/失败及原因）
   f. 删除临时下载目录
4. 生成Excel报告
   - Sheet "Passed": 通过的设备
   - Sheet "Failed": 失败的设备（包含失败原因）
```

### 4. 结果文件识别

**内参文件模式**:
- `*-intrinsic-results-cam.txt` (单相机或双相机)
- 可能在不同子目录中：
  - `imus_intrinsic/` (IMU内参，不验证)
  - `imus_cam_lr_front_extrinsic/` (前相机内参)
  - `imus_cam_lr_eye_extrinsic/` (眼相机内参)
  - `imus_cam_l_extrinsic/` (左相机内参)
  - `imus_cam_r_extrinsic/` (右相机内参)

**外参文件模式**:
- `*-results-imucam.txt` (外参结果)
- 在对应的子目录中：
  - `imus_cam_lr_front_extrinsic/`
  - `imus_cam_lr_eye_extrinsic/`
  - `imus_cam_l_extrinsic/`
  - `imus_cam_r_extrinsic/`

### 5. Excel报告格式

**Sheet "Passed"**:
| Device ID | Intrinsic Types | Extrinsic Types | All Passed |
|-----------|----------------|----------------|------------|
| device001 | front, eye, left, right | front, eye, left, right | Yes |

**Sheet "Failed"**:
| Device ID | Failed Type | Failed Item | Reason | Details |
|-----------|-------------|-------------|--------|---------|
| device002 | Intrinsic | front | fx out of range | fx=610.0 (expected 620-660) |
| device002 | Extrinsic | front | reprojection error too high | cam0_reproj=2.1 (expected <1.5) |

### 6. 命令行参数

```bash
python calibration_validation.py \
  --config s3_config.json \
  --output validation_report.xlsx \
  --temp-dir /tmp/calibration_validation \
  [--device-id DEVICE_ID]  # 可选：只验证指定设备
```

## 实现细节

### 验证逻辑

1. **内参验证**:
   - 解析内参结果文件
   - 检查每个相机的projection和distortion参数
   - 验证是否在有效范围内
   - 记录失败的参数和原因

2. **外参验证**:
   - 解析外参结果文件
   - 检查reprojection error
   - 检查所有IMU的gyroscope和accelerometer errors
   - 验证mean和median是否都满足要求
   - 记录失败的项和原因

### 错误处理

- 下载失败：记录为失败，原因"Download failed"
- 文件不存在：记录为失败，原因"Result file not found"
- 解析失败：记录为失败，原因"Parse error"
- 验证失败：记录具体失败原因

### 性能优化

- 使用临时目录，分析完立即删除
- 可以添加进度显示
- 可以支持多线程（可选）

## 输出示例

### Excel文件结构

**Passed Sheet**:
```
Device ID    | Intrinsic Types              | Extrinsic Types              | All Passed
device001    | front, eye, left, right     | front, eye, left, right     | Yes
device003    | front, eye                  | front, eye                  | Yes
```

**Failed Sheet**:
```
Device ID    | Failed Type   | Failed Item | Reason                          | Details
device002    | Intrinsic     | front       | fx out of range                | fx=610.0 (expected 620-660)
device002    | Extrinsic     | front       | reprojection error too high    | cam0_reproj=2.1 (expected <1.5)
device004    | Intrinsic     | eye         | Result file not found          | No eye intrinsic result file
```

## 注意事项

1. **临时目录管理**: 确保分析完成后删除临时文件，避免磁盘空间问题
2. **S3连接**: 处理网络错误和超时
3. **文件格式**: 确保能处理各种可能的结果文件格式
4. **并发**: 如果设备很多，考虑添加进度条和日志
5. **Excel格式**: 使用openpyxl或pandas确保Excel格式正确

## 依赖安装

```bash
pip install openpyxl  # 或使用 pandas
# s3sdk 已存在，无需额外安装
```
