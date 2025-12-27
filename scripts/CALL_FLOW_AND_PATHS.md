# 批量标定系统 - 调用逻辑与路径说明

## 一、整体调用流程

```
batch_calibration_manager.py (主入口)
    │
    ├─> BatchCalibrationManager.discover_devices()
    │   └─> 扫描 /data/ 目录，发现所有设备
    │
    ├─> BatchCalibrationManager.calibrate_devices()
    │   │
    │   └─> 对每个设备循环:
    │       │
    │       └─> DeviceCalibrationManager.calibrate_all()
    │           │
    │           └─> 按顺序执行9个标定任务:
    │               │
    │               ├─> CalibrationExecutor.execute_task()
    │               │   │
    │               │   └─> 调用对应的 calib_*.py 脚本
    │               │       │
    │               │       ├─> calib_imus_intrinsic.py (任务1)
    │               │       │   ├─> create_rosbag_from_db.py
    │               │       │   │   └─> 生成: imus_intrinsic.bag
    │               │       │   ├─> rosbag play (60x速度)
    │               │       │   ├─> roslaunch imu_mid_0.launch
    │               │       │   ├─> roslaunch imu_right_1.launch
    │               │       │   ├─> roslaunch imu_left_2.launch
    │               │       │   └─> extract_imu_params.py
    │               │       │       └─> 生成: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml
    │               │       │
    │               │       ├─> calib_cams_intrinsic.py --mode front (任务2)
    │               │       │   ├─> create_rosbag_from_db.py
    │               │       │   │   └─> 生成: cam_lr_front_intrinsic.bag
    │               │       │   └─> kalibr_calibrate_cameras
    │               │       │       └─> 生成: cam_lr_front_intrinsic-camchain.yaml
    │               │       │
    │               │       ├─> calib_cams_imu_extrinsic.py --mode front (任务3)
    │               │       │   ├─> create_rosbag_from_db.py
    │               │       │   │   └─> 生成: imus_cam_lr_front_extrinsic.bag
    │               │       │   └─> kalibr_calibrate_imu_camera
    │               │       │       └─> 生成: imus-cam_lr_front_extrinsic-camchain.yaml
    │               │       │
    │               │       └─> ... (其他6个任务类似)
    │               │
    │               └─> S3Uploader.upload_calibration_file()
    │                   └─> 上传YAML文件到S3
    │
    └─> ResultRecorder.record_device_result()
        └─> 记录结果到CSV文件
```

## 二、详细调用逻辑

### 2.1 主入口：batch_calibration_manager.py

**调用方式：**
```bash
python scripts/batch_calibration_manager.py [参数]
```

**主要步骤：**
1. 初始化 `BatchCalibrationManager`
2. 调用 `discover_devices()` 自动发现设备（或使用指定的设备ID列表）
3. 对每个设备调用 `calibrate_devices()`
4. 生成最终统计报告

### 2.2 设备标定：DeviceCalibrationManager

**主要方法：** `calibrate_all()`

**执行流程：**
1. 设置设备ID到 `robocap_env`
2. 按顺序执行9个标定任务（见下表）
3. 每个任务执行后检查输出文件
4. 如果成功，上传文件到S3
5. 记录结果到CSV

### 2.3 标定任务执行：CalibrationExecutor

**主要方法：** `execute_task(task)`

**执行流程：**
1. 构建命令：`python calib_*.py --device-id {device_id} [其他参数]`
2. 执行子进程
3. 等待完成
4. 检查输出文件是否存在
5. 返回执行结果

### 2.4 具体标定脚本流程

#### 任务1：IMU内参标定 (calib_imus_intrinsic.py)

```
1. create_rosbag_from_db.py
   ├─ 输入: /data/{device_id}/v1/data5/
   │   ├─ IMU0.db (或 IMUWriter_dev0_*.db)
   │   ├─ IMU1.db (或 IMUWriter_dev1_*.db)
   │   └─ IMU2.db (或 IMUWriter_dev2_*.db)
   └─ 输出: /data/{device_id}/v1/results/imus_intrinsic/imus_intrinsic.bag

2. rosbag play (60x速度，后台运行)

3. 对每个IMU (0, 1, 2):
   ├─ roslaunch imu_mid_0.launch (或 imu_right_1.launch, imu_left_2.launch)
   ├─ 等待标定完成
   └─ extract_imu_params.py
       └─ 输出: /data/{device_id}/v1/results/imus_intrinsic/imu_mid_0.yaml
               /data/{device_id}/v1/results/imus_intrinsic/imu_right_1.yaml
               /data/{device_id}/v1/results/imus_intrinsic/imu_left_2.yaml
```

#### 任务2：左右前摄像头内参 (calib_cams_intrinsic.py --mode front)

```
1. create_rosbag_from_db.py
   ├─ 输入: /data/{device_id}/v1/data6/
   │   ├─ IMU0.db, IMU1.db, IMU2.db
   │   ├─ left-front.mp4
   │   └─ right-front.mp4
   └─ 输出: /data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/cam_lr_front_intrinsic.bag

2. kalibr_calibrate_cameras
   └─ 输出: /data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/cam_lr_front_intrinsic-camchain.yaml
```

#### 任务3：左右前摄像头外参 (calib_cams_imu_extrinsic.py --mode front)

```
1. create_rosbag_from_db.py
   ├─ 输入: /data/{device_id}/v1/data6/
   │   ├─ IMU0.db, IMU1.db, IMU2.db
   │   ├─ left-front.mp4
   │   └─ right-front.mp4
   └─ 输出: /data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/imus_cam_lr_front_extrinsic.bag

2. kalibr_calibrate_imu_camera
   ├─ 需要: cam_lr_front_intrinsic-camchain.yaml (任务2的输出)
   ├─ 需要: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml (任务1的输出)
   └─ 输出: /data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/imus-cam_lr_front_extrinsic-camchain.yaml
```

#### 任务4-9：类似流程（不同data目录和模式）

## 三、输入路径

### 3.1 数据输入目录（用户提供）

所有输入数据位于 `/data/{device_id}/v1/` 下：

```
/data/{device_id}/v1/
├─ data5/                    # IMU内参数据
│   ├─ IMU0.db              # 新格式
│   ├─ IMU1.db
│   ├─ IMU2.db
│   └─ 或 IMUWriter_dev*_session*_segment*.db  # 旧格式
│
├─ data6/                    # 左右前摄像头数据
│   ├─ IMU0.db, IMU1.db, IMU2.db
│   ├─ left-front.mp4
│   └─ right-front.mp4
│
├─ data7/                    # 左右眼摄像头数据
│   ├─ IMU0.db, IMU1.db, IMU2.db
│   ├─ left-eye.mp4
│   └─ right-eye.mp4
│
├─ data8/                    # 左摄像头数据
│   ├─ IMU0.db, IMU1.db, IMU2.db
│   └─ left.mp4
│
└─ data9/                    # 右摄像头数据
    ├─ IMU0.db, IMU1.db, IMU2.db
    └─ right.mp4
```

### 3.2 配置文件路径

- **S3配置**: `s3sdk/s3_config.json` (可选)
- **AprilTag配置**: `/robocap-tools/config/april_6x6.yaml`
- **Launch文件**: `/robocap-tools/launch/imu_mid_0.launch` 等

## 四、中间文件路径

### 4.1 Rosbag文件（中间文件）

所有rosbag文件生成在对应的results目录下：

```
/data/{device_id}/v1/results/
├─ imus_intrinsic/
│   └─ imus_intrinsic.bag                    # 任务1中间文件
│
├─ imus_cam_lr_front_extrinsic/
│   ├─ cam_lr_front_intrinsic.bag            # 任务2中间文件
│   └─ imus_cam_lr_front_extrinsic.bag      # 任务3中间文件
│
├─ imus_cam_lr_eye_extrinsic/
│   ├─ cam_lr_eye_intrinsic.bag              # 任务4中间文件
│   └─ imus_cam_lr_eye_extrinsic.bag        # 任务5中间文件
│
├─ imus_cam_l_extrinsic/
│   ├─ cam_l_intrinsic.bag                   # 任务6中间文件
│   └─ imus_cam_l_extrinsic.bag              # 任务7中间文件
│
└─ imus_cam_r_extrinsic/
    ├─ cam_r_intrinsic.bag                   # 任务8中间文件
    └─ imus_cam_r_extrinsic.bag              # 任务9中间文件
```

### 4.2 日志文件（中间文件）

每个rosbag生成时，会在同一目录下生成对应的日志文件：

```
/data/{device_id}/v1/results/imus_intrinsic/
├─ imus_intrinsic.bag
├─ IMU0.log (或 IMUWriter_dev0_*.log)      # IMU数据日志
├─ IMU1.log
└─ IMU2.log

/data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/
├─ cam_lr_front_intrinsic.bag
├─ left-front.log                           # 视频数据日志
└─ right-front.log
```

### 4.3 Kalibr临时文件（中间文件）

Kalibr标定过程中会在输出目录生成临时文件：

```
/data/{device_id}/v1/results/imus_cam_lr_front_extrinsic/
├─ cam_lr_front_intrinsic-camchain.yaml      # 最终输出
├─ report-cam-lr-front-intrinsic-*.pdf      # Kalibr报告
└─ results-cam-lr-front-intrinsic-*.txt     # Kalibr结果文本
```

## 五、最终输出文件路径

### 5.1 标定结果YAML文件（最终输出）

```
/data/{device_id}/v1/results/
├─ imus_intrinsic/
│   ├─ imu_mid_0.yaml                        # 任务1输出
│   ├─ imu_right_1.yaml                      # 任务1输出
│   └─ imu_left_2.yaml                       # 任务1输出
│
├─ imus_cam_lr_front_extrinsic/
│   ├─ cam_lr_front_intrinsic-camchain.yaml  # 任务2输出
│   └─ imus-cam_lr_front_extrinsic-camchain.yaml  # 任务3输出
│
├─ imus_cam_lr_eye_extrinsic/
│   ├─ cam_lr_eye_intrinsic-camchain.yaml    # 任务4输出
│   └─ imus-cam_lr_eye_extrinsic-camchain.yaml  # 任务5输出
│
├─ imus_cam_l_extrinsic/
│   ├─ cam_l_intrinsic-camchain.yaml         # 任务6输出
│   └─ imus-cam_l_extrinsic-camchain.yaml    # 任务7输出
│
└─ imus_cam_r_extrinsic/
    ├─ cam_r_intrinsic-camchain.yaml         # 任务8输出
    └─ imus-cam_r_extrinsic-camchain.yaml    # 任务9输出
```

### 5.2 CSV结果文件（最终输出）

**默认路径：** `/tmp/calibration_results.csv`

**格式：**
```csv
device_id,timestamp,imu_intrinsic,cam_lr_front_intrinsic,cam_lr_front_extrinsic,cam_lr_eye_intrinsic,cam_lr_eye_extrinsic,cam_l_intrinsic,cam_l_extrinsic,cam_r_intrinsic,cam_r_extrinsic
faf2a598869ccfc8,2024-01-01 12:00:00,成功,成功,成功,成功,成功,成功,成功,成功,成功
```

### 5.3 S3上传路径（最终输出）

标定成功的YAML文件会自动上传到S3：

```
S3 Bucket: {bucket_name}/
├─ {device_id}/v1/results/imus_intrinsic/
│   ├─ imu_mid_0.yaml
│   ├─ imu_right_1.yaml
│   └─ imu_left_2.yaml
│
├─ {device_id}/v1/results/imus_cam_lr_front_extrinsic/
│   ├─ cam_lr_front_intrinsic-camchain.yaml
│   └─ imus-cam_lr_front_extrinsic-camchain.yaml
│
├─ {device_id}/v1/results/imus_cam_lr_eye_extrinsic/
│   ├─ cam_lr_eye_intrinsic-camchain.yaml
│   └─ imus-cam_lr_eye_extrinsic-camchain.yaml
│
├─ {device_id}/v1/results/imus_cam_l_extrinsic/
│   ├─ cam_l_intrinsic-camchain.yaml
│   └─ imus-cam_l_extrinsic-camchain.yaml
│
└─ {device_id}/v1/results/imus_cam_r_extrinsic/
    ├─ cam_r_intrinsic-camchain.yaml
    └─ imus-cam_r_extrinsic-camchain.yaml
```

## 六、文件依赖关系

### 6.1 任务依赖关系

```
任务1 (IMU内参)
  └─> 无依赖

任务2 (左右前摄像头内参)
  └─> 无依赖

任务3 (左右前摄像头外参)
  ├─> 依赖任务1的输出: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml
  └─> 依赖任务2的输出: cam_lr_front_intrinsic-camchain.yaml

任务4 (左右眼摄像头内参)
  └─> 无依赖

任务5 (左右眼摄像头外参)
  ├─> 依赖任务1的输出: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml
  └─> 依赖任务4的输出: cam_lr_eye_intrinsic-camchain.yaml

任务6 (左摄像头内参)
  └─> 无依赖

任务7 (左摄像头外参)
  ├─> 依赖任务1的输出: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml
  └─> 依赖任务6的输出: cam_l_intrinsic-camchain.yaml

任务8 (右摄像头内参)
  └─> 无依赖

任务9 (右摄像头外参)
  ├─> 依赖任务1的输出: imu_mid_0.yaml, imu_right_1.yaml, imu_left_2.yaml
  └─> 依赖任务8的输出: cam_r_intrinsic-camchain.yaml
```

### 6.2 文件生成顺序

1. **Rosbag文件** → 由 `create_rosbag_from_db.py` 生成
2. **内参YAML文件** → 由 `kalibr_calibrate_cameras` 或 `extract_imu_params.py` 生成
3. **外参YAML文件** → 由 `kalibr_calibrate_imu_camera` 生成（需要内参文件）

## 七、文件清理建议

### 7.1 可删除的中间文件

- **Rosbag文件**: 标定完成后可以删除（占用空间大）
- **日志文件**: `.log` 文件可以删除
- **Kalibr临时文件**: `report-*.pdf`, `results-*.txt` 可以删除

### 7.2 需要保留的文件

- **所有YAML文件**: 标定结果，必须保留
- **CSV结果文件**: 标定记录，建议保留

## 八、路径变量说明

所有路径都通过 `robocap_env.py` 中的变量定义：

```python
DATASET_ROOT_DIR = "/data"
ROBOCAP_DEVICE_ID = os.environ.get("ROBOCAP_DEVICE_ID", "default_device")

# 输入目录
DATASET_IMUS_INTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data5"
DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data6"
# ... 其他输入目录

# 输出目录
OUTPUT_IMUS_INTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_intrinsic"
OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_cam_lr_front_extrinsic"
# ... 其他输出目录

# Rosbag文件路径
ROSBAG_FILE_IMUS_INTRINSIC = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imus_intrinsic.bag"
# ... 其他rosbag路径

# YAML文件路径
YAML_FILE_IMU_MID_0 = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imu_mid_0.yaml"
# ... 其他YAML路径
```

## 九、总结

### 输入
- **数据目录**: `/data/{device_id}/v1/data{5-9}/`
- **配置文件**: S3配置、AprilTag配置、Launch文件

### 中间文件
- **Rosbag文件**: 每个任务生成1-2个rosbag文件
- **日志文件**: 每个数据源生成对应的日志文件
- **Kalibr临时文件**: 标定过程中的临时文件

### 最终输出
- **YAML文件**: 9个标定任务共生成约12个YAML文件
- **CSV文件**: 1个CSV文件记录所有设备的标定结果
- **S3文件**: 标定成功的YAML文件自动上传到S3

### 文件数量统计（每个设备）
- **输入文件**: 约15-20个（3个IMU数据库文件 + 2-4个视频文件 × 5个data目录）
- **中间文件**: 约20-30个（rosbag + 日志 + Kalibr临时文件）
- **最终输出**: 约12个YAML文件 + 1个CSV记录

