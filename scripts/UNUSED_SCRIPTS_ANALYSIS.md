# 未使用脚本分析报告

## 脚本使用情况分析

### ✅ 正在使用的脚本（核心功能）

#### 1. 批量标定系统
- **batch_calibration_manager.py** ✅ 主入口脚本
- **device_calibration_manager.py** ✅ 被batch_calibration_manager.py使用
- **calibration_executor.py** ✅ 被device_calibration_manager.py使用
- **calibration_task.py** ✅ 被calibration_executor.py使用
- **result_recorder.py** ✅ 被device_calibration_manager.py使用
- **s3_uploader.py** ✅ 被device_calibration_manager.py使用

#### 2. 标定脚本（直接调用）
- **calib_imus_intrinsic.py** ✅ 主入口脚本，被calibration_executor.py调用
- **calib_cams_intrinsic.py** ✅ 主入口脚本，被calibration_executor.py调用
- **calib_cams_imu_extrinsic.py** ✅ 主入口脚本，被calibration_executor.py调用

#### 3. 公共工具模块
- **calibration_common.py** ✅ 被所有标定脚本使用
- **rosbag_creator_helper.py** ✅ 被标定脚本使用
- **kalibr_executor.py** ✅ 被标定脚本使用
- **rosbag_helper.py** ✅ 被标定脚本和imu_calibration_helper.py使用
- **imu_calibration_helper.py** ✅ 被calib_imus_intrinsic.py使用

#### 4. 核心功能模块
- **create_rosbag_from_db.py** ✅ 被所有标定脚本调用（通过RosbagCreator）
- **robocap_env.py** ✅ 被所有脚本使用
- **calibration_result_handler.py** ✅ 被所有标定脚本使用

#### 5. 被调用的辅助脚本
- **extract_imu_params.py** ✅ 被imu_calibration_helper.py调用（在calib_imus_intrinsic.py流程中）

---

### ⚠️ 可能未使用的脚本（需要确认）

#### 1. merge_imu_data.py
**状态**: ⚠️ **可能未使用**
- **功能**: 合并单个数据库文件的gyro和acc数据到CSV
- **是否有main函数**: ✅ 有
- **是否被其他脚本导入**: ❌ 未被导入
- **是否被调用**: ❌ 未被调用
- **分析**: 
  - 这是一个独立的工具脚本，用于将IMU数据库文件转换为CSV
  - 但在当前的标定流程中，`create_rosbag_from_db.py`直接处理数据库文件，不需要先转换为CSV
  - **建议**: 如果不再需要将IMU数据转换为CSV格式，可以删除

#### 2. merge_multi_imu_data.py
**状态**: ⚠️ **可能未使用**
- **功能**: 合并多个数据库文件的gyro和acc数据到CSV（支持多设备）
- **是否有main函数**: ✅ 有
- **是否被其他脚本导入**: ❌ 未被导入
- **是否被调用**: ❌ 未被调用
- **分析**:
  - 这是`merge_imu_data.py`的增强版，支持处理多个IMU设备
  - 同样，在当前标定流程中不需要CSV中间格式
  - **建议**: 如果不再需要CSV格式输出，可以删除

#### 3. example_calibration_callback.py
**状态**: ⚠️ **示例脚本，可能不需要**
- **功能**: 标定回调函数的示例实现
- **是否有main函数**: ✅ 有（但只是示例）
- **是否被其他脚本导入**: ❌ 未被导入
- **是否被调用**: ❌ 未被调用
- **分析**:
  - 这是一个示例文件，展示如何使用回调函数
  - 用户可以根据这个示例创建自己的回调函数
  - **建议**: 保留作为文档/示例，或者移到examples目录

---

### 📋 脚本分类总结

#### 核心脚本（必须保留）
1. batch_calibration_manager.py - 批量标定主入口
2. device_calibration_manager.py - 设备标定管理器
3. calibration_executor.py - 标定执行器
4. calibration_task.py - 标定任务定义
5. result_recorder.py - 结果记录器
6. s3_uploader.py - S3上传器
7. calib_imus_intrinsic.py - IMU内参标定
8. calib_cams_intrinsic.py - 相机内参标定
9. calib_cams_imu_extrinsic.py - 相机-IMU外参标定
10. create_rosbag_from_db.py - Rosbag创建
11. robocap_env.py - 环境配置
12. calibration_result_handler.py - 结果处理器
13. extract_imu_params.py - IMU参数提取（被调用）
14. calibration_common.py - 公共工具
15. rosbag_creator_helper.py - Rosbag创建器
16. kalibr_executor.py - Kalibr执行器
17. rosbag_helper.py - Rosbag辅助工具
18. imu_calibration_helper.py - IMU标定辅助

#### 可能未使用的脚本
1. **merge_imu_data.py** - 单文件IMU数据合并到CSV（可能已废弃）
2. **merge_multi_imu_data.py** - 多文件IMU数据合并到CSV（可能已废弃）

#### 示例/文档脚本
1. **example_calibration_callback.py** - 回调函数示例（建议保留或移到examples目录）

---

## 详细分析

### merge_imu_data.py 和 merge_multi_imu_data.py

**功能**: 将IMU数据库文件（.db）中的gyro和acc数据合并并输出为CSV文件

**当前使用情况**:
- ❌ 未被任何脚本导入
- ❌ 未被任何脚本调用
- ❌ 不在标定流程中

**历史用途推测**:
- 可能是早期用于数据分析和处理的工具
- 现在`create_rosbag_from_db.py`直接处理数据库文件，不需要CSV中间格式

**建议**:
- 如果不再需要CSV格式的IMU数据，可以删除这两个脚本
- 如果将来可能需要CSV格式进行分析，可以保留

### example_calibration_callback.py

**功能**: 展示如何使用标定回调函数

**当前使用情况**:
- ❌ 未被任何脚本导入
- ❌ 未被任何脚本调用
- ✅ 有main函数（但只是演示）

**建议**:
- **选项1**: 保留作为文档/示例（推荐）
- **选项2**: 移到`examples/`目录
- **选项3**: 如果不需要示例，可以删除

---

## 建议操作

### 可以安全删除的脚本

1. **merge_imu_data.py** 
   - 原因: 功能已被`create_rosbag_from_db.py`替代，不再需要CSV中间格式
   - 风险: 低（未被引用）

2. **merge_multi_imu_data.py**
   - 原因: 同上，功能已被替代
   - 风险: 低（未被引用）

### 建议保留的脚本

1. **example_calibration_callback.py**
   - 原因: 作为文档/示例，帮助用户理解如何使用回调函数
   - 建议: 保留，或移到examples目录

---

## 检查命令

如果你想确认这些脚本是否真的未被使用，可以运行：

```bash
# 检查merge_imu_data.py的引用
grep -r "merge_imu_data" scripts/ --exclude="merge_imu_data.py"

# 检查merge_multi_imu_data.py的引用
grep -r "merge_multi_imu_data" scripts/ --exclude="merge_multi_imu_data.py"

# 检查example_calibration_callback.py的引用
grep -r "example_calibration_callback" scripts/ --exclude="example_calibration_callback.py"
```

---

## 总结

**可以删除的脚本**:
1. `merge_imu_data.py` - 未被使用，功能已替代
2. `merge_multi_imu_data.py` - 未被使用，功能已替代

**建议保留的脚本**:
1. `example_calibration_callback.py` - 作为示例/文档

**删除前请确认**:
- 是否还有其他外部脚本或工具依赖这些文件
- 是否将来可能需要CSV格式的IMU数据
- 是否在CI/CD或其他自动化流程中使用

