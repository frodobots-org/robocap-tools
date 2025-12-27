# 批量标定工具使用说明

## 概述

`batch_calibration_manager.py` 是一个自动化批量标定工具，可以自动发现并标定所有设备的所有标定任务。

## 功能特性

- **自动发现设备**: 自动扫描 `/data/` 目录，发现所有需要标定的设备
- **批量标定**: 按顺序执行每个设备的9个标定任务
- **结果记录**: 将标定结果记录到CSV文件
- **S3上传**: 自动将标定成功的YAML文件上传到S3

## 标定任务列表

每个设备需要执行以下9个标定任务：

1. **data5**: IMU内参标定（3个IMU：imu0, imu1, imu2）
2. **data6**: 左右前摄像头内参标定
3. **data6**: 左右前摄像头外参标定（包含IMU）
4. **data7**: 左右眼摄像头内参标定
5. **data7**: 左右眼摄像头外参标定（包含IMU）
6. **data8**: 左摄像头内参标定
7. **data8**: 左摄像头外参标定（包含IMU）
8. **data9**: 右摄像头内参标定
9. **data9**: 右摄像头外参标定（包含IMU）

## 使用方法

### 基本用法（自动发现所有设备）

```bash
python scripts/batch_calibration_manager.py
```

### 指定特定设备

```bash
python scripts/batch_calibration_manager.py --device-ids faf2a598869ccfc8 device2 device3
```

### 指定CSV输出路径

```bash
python scripts/batch_calibration_manager.py --csv-file /path/to/results.csv
```

### 指定S3配置文件

```bash
python scripts/batch_calibration_manager.py --s3-config /path/to/s3_config.json
```

### 完整参数示例

```bash
python scripts/batch_calibration_manager.py \
    --device-ids faf2a598869ccfc8 \
    --data-root /data \
    --scripts-dir /path/to/scripts \
    --csv-file /tmp/calibration_results.csv \
    --s3-config /path/to/s3_config.json
```

## 参数说明

- `--device-ids`: 要标定的设备ID列表（可选，如果不指定则自动发现）
- `--data-root`: 数据根目录（默认: `/data`）
- `--scripts-dir`: 脚本目录路径（默认: 当前脚本所在目录）
- `--csv-file`: CSV结果文件路径（默认: `/tmp/calibration_results.csv`）
- `--s3-config`: S3配置文件路径（可选，默认使用 `s3sdk/s3_config.json`）

## CSV结果文件格式

CSV文件包含以下列：

- `device_id`: 设备ID
- `timestamp`: 标定时间戳
- `imu_intrinsic`: IMU内参标定结果（成功/失败）
- `cam_lr_front_intrinsic`: 左右前摄像头内参标定结果
- `cam_lr_front_extrinsic`: 左右前摄像头外参标定结果
- `cam_lr_eye_intrinsic`: 左右眼摄像头内参标定结果
- `cam_lr_eye_extrinsic`: 左右眼摄像头外参标定结果
- `cam_l_intrinsic`: 左摄像头内参标定结果
- `cam_l_extrinsic`: 左摄像头外参标定结果
- `cam_r_intrinsic`: 右摄像头内参标定结果
- `cam_r_extrinsic`: 右摄像头外参标定结果

## S3上传路径

标定成功的YAML文件会自动上传到S3，路径格式为：

- IMU内参: `{device_id}/v1/results/imus_intrinsic/{filename}`
- 左右前摄像头: `{device_id}/v1/results/imus_cam_lr_front_extrinsic/{filename}`
- 左右眼摄像头: `{device_id}/v1/results/imus_cam_lr_eye_extrinsic/{filename}`
- 左摄像头: `{device_id}/v1/results/imus_cam_l_extrinsic/{filename}`
- 右摄像头: `{device_id}/v1/results/imus_cam_r_extrinsic/{filename}`

## 目录结构要求

工具期望的数据目录结构：

```
/data/
  {device_id}/
    v1/
      data5/          # IMU内参数据
      data6/          # 左右前摄像头数据
      data7/          # 左右眼摄像头数据
      data8/          # 左摄像头数据
      data9/          # 右摄像头数据
      results/        # 标定结果输出目录
        imus_intrinsic/
        imus_cam_lr_front_extrinsic/
        imus_cam_lr_eye_extrinsic/
        imus_cam_l_extrinsic/
        imus_cam_r_extrinsic/
```

## 注意事项

1. 确保所有标定脚本（`calib_*.py`）都在 `scripts/` 目录下
2. 确保S3配置文件存在且有效（如果启用S3上传）
3. 标定过程可能需要较长时间，请耐心等待
4. 如果某个任务失败，工具会继续执行其他任务
5. CSV文件会自动创建，如果设备已存在记录，会更新该记录

## 错误处理

- 如果数据目录不存在，对应任务会标记为失败
- 如果脚本执行失败，会记录错误信息
- 如果S3上传失败，会在控制台输出警告，但不会影响标定流程

## 示例输出

```
================================================================================
开始批量标定 - 共 1 个设备
================================================================================

################################################################################
设备 1/1: faf2a598869ccfc8
################################################################################

================================================================================
开始标定设备: faf2a598869ccfc8
================================================================================

================================================================================
任务: imu_intrinsic
================================================================================
[执行] imu_intrinsic - 设备: faf2a598869ccfc8
[成功] imu_intrinsic - 生成了 3 个文件
✓ imu_intrinsic 标定成功

...

================================================================================
设备 faf2a598869ccfc8 标定总结
================================================================================
总任务数: 9
成功: 9
失败: 0
================================================================================
```

