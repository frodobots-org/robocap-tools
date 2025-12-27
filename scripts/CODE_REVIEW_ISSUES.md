# 代码审查发现的问题

## 问题1: S3上传器初始化逻辑不合理

**位置**: `batch_calibration_manager.py:43`

**问题**: 
```python
self.s3_uploader = S3Uploader(s3_config_file) if s3_config_file else None
```

如果 `s3_config_file` 为 `None`，应该尝试使用默认配置，而不是直接设为 `None`。

**修复**: 应该改为：
```python
self.s3_uploader = S3Uploader(s3_config_file)
```

因为 `S3Uploader` 内部已经处理了 `None` 的情况，会尝试使用默认配置。

## 问题2: 外参标定的期望输出文件不正确

**位置**: `calibration_task.py` 中所有外参任务的 `expected_output_files`

**问题**: 
外参标定任务（CAM_LR_FRONT_EXTRINSIC等）的 `expected_output_files` 只列出了内参文件（如 `cam_lr_front_intrinsic-camchain.yaml`），但外参标定应该生成外参文件（如 `imus-cam_lr_front_extrinsic-camchain.yaml`）。

**影响**: 
- 如果外参标定成功但只检查内参文件，可能会误判为失败
- 或者外参文件不会被检查，导致即使生成了也不会被上传

**需要确认**: 
需要查看 `calib_cams_imu_extrinsic.py` 实际生成什么文件。

## 问题3: 清理逻辑只在成功时执行

**位置**: `device_calibration_manager.py:_execute_single_task()`

**问题**: 
清理中间文件（.log和.bag）只在标定成功时执行。如果标定失败，这些文件不会被清理。

**考虑**: 
- 如果用户希望"标定完成后"清理，可能意味着无论成功失败都要清理
- 但如果标定失败，保留中间文件可能有助于调试

**建议**: 
可以添加一个参数控制是否在失败时也清理，或者默认只在成功时清理（当前行为）。

## 问题4: 实时日志输出的线程安全问题

**位置**: `calibration_executor.py:read_output()`

**问题**: 
如果进程快速结束，`read_output()` 线程可能还在读取，导致竞态条件。

**当前处理**: 
使用 `daemon=True` 和 `timeout_occurred` 事件来处理，但可能不够完善。

**建议**: 
当前实现基本合理，但可以改进错误处理。

## 问题5: 超时处理中的进程变量作用域

**位置**: `calibration_executor.py:173-180`

**问题**: 
在 `except subprocess.TimeoutExpired` 块中，使用 `if 'process' in locals()` 检查，但如果超时发生在 `process` 定义之前，可能会有问题。

**当前处理**: 
使用 `locals()` 检查，这是合理的。

## 问题6: 缺少对PDF/TXT文件存在性的验证

**位置**: `calibration_executor.py:_check_output_files()`

**问题**: 
PDF和TXT文件可能不存在（如果Kalibr没有生成），但代码会尝试上传所有找到的文件，这是合理的。但如果期望这些文件必须存在，应该添加验证。

**当前处理**: 
只查找存在的文件，不强制要求，这是合理的。

