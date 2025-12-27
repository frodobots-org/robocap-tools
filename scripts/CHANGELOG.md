# 批量标定系统更新日志

## 最新更新（2024）

### 新增功能

1. **实时日志输出**
   - 所有子进程的输出现在实时显示在终端上
   - 使用多线程实现非阻塞的日志读取
   - 确保用户可以看到标定过程的实时进度

2. **超时控制**
   - 单个设备的标定总超时时间设置为2小时（7200秒）
   - 每个标定任务也有2小时超时保护
   - 超时后自动终止进程并标记为失败

3. **自动清理中间文件**
   - 标定完成后自动删除 `.log` 文件
   - 标定完成后自动删除 `.bag` 文件
   - 保留所有YAML、PDF、TXT文件用于上传

4. **Kalibr报告文件上传**
   - 自动查找并上传 `report-*.pdf` 文件（Kalibr生成的PDF报告）
   - 自动查找并上传 `results-*.txt` 文件（Kalibr生成的文本结果）
   - 这些文件会跟随YAML文件一起上传到S3

### 修改的文件

1. **calibration_executor.py**
   - 修改超时时间从1小时改为2小时
   - 实现实时日志输出（使用多线程）
   - 修改 `_check_output_files()` 方法，返回PDF和TXT文件

2. **device_calibration_manager.py**
   - 添加 `_cleanup_intermediate_files()` 方法
   - 在标定完成后自动清理.log和.bag文件
   - 添加设备总超时时间检查（2小时）

3. **s3_uploader.py**
   - 无需修改，因为PDF和TXT文件已经包含在output_files中

### 使用说明

#### 实时日志输出

运行批量标定时，所有子进程的输出会实时显示：

```bash
python scripts/batch_calibration_manager.py
```

输出示例：
```
[执行] imu_intrinsic - 设备: faf2a598869ccfc8
[命令] python scripts/calib_imus_intrinsic.py --device-id faf2a598869ccfc8
[超时] 2小时
--------------------------------------------------------------------------------
Step 1: Creating rosbag from database files...
Input directory: /data/faf2a598869ccfc8/v1/data5
Output rosbag: /data/faf2a598869ccfc8/v1/results/imus_intrinsic/imus_intrinsic.bag
...
```

#### 文件清理

标定完成后会自动清理：

```
[清理] 已删除日志文件: IMU0.log
[清理] 已删除rosbag文件: imus_intrinsic.bag
[清理] 共删除 2 个中间文件
```

#### 文件上传

所有相关文件（YAML、PDF、TXT）都会上传到S3：

```
[上传] /data/.../imu_mid_0.yaml -> s3://bucket/.../imu_mid_0.yaml
[成功] 上传完成: ...
[上传] /data/.../report-cam-lr-front-intrinsic-2024-01-01.pdf -> s3://bucket/.../report-cam-lr-front-intrinsic-2024-01-01.pdf
[成功] 上传完成: ...
```

### 超时设置

- **单个任务超时**: 2小时（7200秒）
- **单个设备总超时**: 2小时（7200秒）
- 如果超时，进程会被终止，任务标记为失败

### 保留的文件

标定完成后，以下文件会被保留：

- ✅ 所有 `.yaml` 文件（标定结果）
- ✅ 所有 `report-*.pdf` 文件（Kalibr报告）
- ✅ 所有 `results-*.txt` 文件（Kalibr文本结果）

### 删除的文件

标定完成后，以下文件会被自动删除：

- ❌ 所有 `.log` 文件（日志文件）
- ❌ 所有 `.bag` 文件（Rosbag文件）

### 注意事项

1. **日志输出**: 如果子进程输出很多，终端可能会滚动很快，建议使用 `tee` 命令保存日志：
   ```bash
   python scripts/batch_calibration_manager.py 2>&1 | tee calibration.log
   ```

2. **超时处理**: 如果标定任务超过2小时，会被自动终止。如果经常超时，可能需要：
   - 检查数据质量
   - 优化标定参数
   - 增加超时时间（修改代码中的 `timeout_seconds`）

3. **文件清理**: 清理操作在标定成功后执行。如果标定失败，中间文件不会被删除，方便调试。

4. **S3上传**: 确保S3配置正确，否则上传会失败但不会影响标定流程。

