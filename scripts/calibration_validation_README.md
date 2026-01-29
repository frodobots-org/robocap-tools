# 标定结果验收验证脚本使用说明

## 功能

从S3下载所有设备的标定结果，验证是否符合验收标准，并生成Excel报告。

## 安装依赖

```bash
pip install openpyxl
```

## 使用方法

### 基本用法（验证所有设备）

```bash
python scripts/calibration_validation.py \
  --config s3sdk/s3_config.json \
  --output validation_report.xlsx
```

### 验证单个设备

```bash
python scripts/calibration_validation.py \
  --config s3sdk/s3_config.json \
  --output validation_report.xlsx \
  --device-id device001
```

### 验证设备列表（从文件读取）

```bash
python scripts/calibration_validation.py \
  --config s3sdk/s3_config.json \
  --output validation_report.xlsx \
  --device-list ids.txt
```

文件格式（`ids.txt`）：
```
3ccac2ac83ff9eb7
28895e5d054f4a0f
6147733576e45bbc
8abb713cf25a1a62
# 支持注释行（以 # 开头）
d715e08e73870ac3
```

### 指定临时目录

```bash
python scripts/calibration_validation.py \
  --config s3sdk/s3_config.json \
  --output validation_report.xlsx \
  --temp-dir /tmp/my_temp_dir
```

## 命令行参数

- `--config`: S3配置文件路径（默认: `s3_config.json`）
- `--output`: 输出Excel文件路径（默认: `calibration_validation_report.xlsx`）
- `--temp-dir`: 临时下载目录（默认: 系统临时目录）
- `--device-id`: 可选，只验证指定设备（不能与 `--device-list` 同时使用）
- `--device-list`: 可选，从文件读取设备ID列表（每行一个ID，支持 `#` 注释，不能与 `--device-id` 同时使用）

**注意**: 如果不提供 `--device-id` 或 `--device-list`，将验证S3中所有设备

## 输出说明

### Excel报告结构

生成的Excel文件包含两个sheet：

#### 1. "Passed" Sheet
包含通过验收的设备列表：
- **Device ID**: 设备ID
- **Intrinsic Types**: 通过的内参类型（front, eye, left, right）
- **Extrinsic Types**: 通过的外参类型（front, eye, left, right）
- **All Passed**: 是否全部通过

#### 2. "Failed" Sheet
包含未通过验收的设备列表：
- **Device ID**: 设备ID
- **Failed Type**: 失败类型（intrinsic 或 extrinsic）
- **Failed Item**: 失败的标定项（front, eye, left, right）
- **Reason**: 失败原因
- **Details**: 详细失败信息

## 验收标准

### 内参标准

**Projection参数**:
- fx: [620.0, 660.0]
- fy: [620.0, 660.0]
- cx: [920.0, 1000.0]
- cy: [500.0, 580.0]

**Distortion参数**:
- k1: [0.01, 0.07]
- k2: [-0.06, 0.06]
- p1: [-0.06, 0.06]
- p2: [-0.04, 0.04]

### 外参标准

- **Reprojection error**: < 1.5 像素（每个相机）
- **Gyroscope error**: mean 和 median 都 < 0.015 rad/s（每个IMU）
- **Accelerometer error**: mean 和 median 都 < 0.12 m/s²（每个IMU）

## 工作流程

1. 从S3列出所有包含 `v1/results` 目录的设备
2. 对每个设备：
   - 下载 `{device_id}/v1/results/` 目录到临时目录
   - 查找并解析内外参结果文件
   - 验证是否符合验收标准
   - 记录验证结果
   - 删除临时下载目录
3. 生成Excel报告

## 注意事项

1. **S3配置**: 确保S3配置文件正确，包含有效的访问凭证
2. **网络连接**: 需要稳定的网络连接来下载S3文件
3. **磁盘空间**: 临时目录需要足够的空间存储下载的文件
4. **处理时间**: 如果设备很多，处理可能需要较长时间

## 示例输出

```
Temporary directory: /tmp/calibration_validation_xxxxx
S3 SDK initialized with bucket: my-bucket
Listing all devices from S3...
Found 10 devices

================================================================================
Starting validation for 10 device(s)
================================================================================

[1/10] Analyzing device: device001
  Downloading device001/v1/results/...
  Results: 8 passed, 0 failed
  Cleaned up temporary directory for device001

[2/10] Analyzing device: device002
  Downloading device002/v1/results/...
  Results: 6 passed, 2 failed
  Cleaned up temporary directory for device002

...

================================================================================
Generating Excel report...
================================================================================

Excel report saved to: validation_report.xlsx

================================================================================
Validation Summary
================================================================================
Total devices analyzed: 10
Total passed validations: 75
Total failed validations: 5
Report saved to: validation_report.xlsx
================================================================================
```
