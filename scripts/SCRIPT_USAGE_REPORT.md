# 脚本使用情况完整报告

## 📊 脚本统计

**总脚本数**: 21个Python脚本 + 2个Shell脚本 = 23个脚本文件

---

## ✅ 正在使用的脚本（18个）

### 批量标定系统（6个）
1. ✅ **batch_calibration_manager.py** - 批量标定主入口
2. ✅ **device_calibration_manager.py** - 设备标定管理器
3. ✅ **calibration_executor.py** - 标定执行器
4. ✅ **calibration_task.py** - 标定任务定义
5. ✅ **result_recorder.py** - 结果记录器（CSV）
6. ✅ **s3_uploader.py** - S3上传器

### 标定脚本（3个）
7. ✅ **calib_imus_intrinsic.py** - IMU内参标定
8. ✅ **calib_cams_intrinsic.py** - 相机内参标定
9. ✅ **calib_cams_imu_extrinsic.py** - 相机-IMU外参标定

### 公共工具模块（5个）
10. ✅ **calibration_common.py** - 公共工具函数和类
11. ✅ **rosbag_creator_helper.py** - Rosbag创建器
12. ✅ **kalibr_executor.py** - Kalibr执行器
13. ✅ **rosbag_helper.py** - Rosbag辅助工具
14. ✅ **imu_calibration_helper.py** - IMU标定辅助

### 核心功能模块（4个）
15. ✅ **create_rosbag_from_db.py** - Rosbag创建（核心功能）
16. ✅ **robocap_env.py** - 环境配置（被所有脚本使用）
17. ✅ **calibration_result_handler.py** - 结果处理器
18. ✅ **extract_imu_params.py** - IMU参数提取（被imu_calibration_helper.py调用）

---

## ⚠️ 可能未使用的脚本（3个）

### 1. merge_imu_data.py
**文件大小**: ~311行  
**功能**: 将单个IMU数据库文件的gyro和acc数据合并输出为CSV文件  
**使用情况**:
- ❌ 未被任何脚本导入
- ❌ 未被任何脚本调用
- ❌ 不在任何标定流程中
- ✅ 有独立的main函数（可作为独立工具使用）

**分析**:
- 这个脚本的功能是：`数据库文件(.db) → CSV文件`
- 但在当前标定流程中，`create_rosbag_from_db.py`直接处理数据库文件生成rosbag，不需要CSV中间格式
- `create_rosbag_from_db.py`中有`merge_imu_data()`函数，但那是内部函数，不是调用这个脚本

**建议**: 
- 🔴 **可以删除**（如果不再需要CSV格式的IMU数据）
- 或者保留作为独立的数据分析工具

---

### 2. merge_multi_imu_data.py
**文件大小**: ~444行  
**功能**: 将多个IMU数据库文件（支持多设备）的gyro和acc数据合并输出为CSV文件  
**使用情况**:
- ❌ 未被任何脚本导入
- ❌ 未被任何脚本调用
- ❌ 不在任何标定流程中
- ✅ 有独立的main函数（可作为独立工具使用）

**分析**:
- 这是`merge_imu_data.py`的增强版，支持处理多个IMU设备
- 同样，在当前标定流程中不需要CSV中间格式
- `create_rosbag_from_db.py`已经实现了类似的功能（直接处理多个数据库文件）

**建议**: 
- 🔴 **可以删除**（如果不再需要CSV格式的IMU数据）
- 或者保留作为独立的数据分析工具

---

### 3. example_calibration_callback.py
**文件大小**: ~152行  
**功能**: 标定回调函数的示例实现，展示如何处理标定结果并上传到S3  
**使用情况**:
- ❌ 未被任何脚本导入
- ❌ 未被任何脚本调用
- ✅ 有main函数（但只是演示用途）
- ✅ 包含完整的示例代码和注释

**分析**:
- 这是一个**示例/文档**文件
- 帮助用户理解如何实现回调函数
- 展示了如何处理标定结果和上传到S3

**建议**: 
- 🟡 **建议保留**（作为文档/示例）
- 或者移到`examples/`目录
- 如果确定不需要示例，可以删除

---

## 📝 Shell脚本（2个）

### 1. 01_ros_env.sh
**状态**: ❓ **需要确认用途**
- 可能是ROS环境设置脚本
- 需要检查是否在Docker容器或启动脚本中使用

### 2. getmp4_basetime.sh
**状态**: ❓ **可能已废弃**
- 功能：获取MP4文件的基础时间戳
- 但在`create_rosbag_from_db.py`中已经用Python实现了`get_mp4_base_timestamp()`函数
- 可能已被Python实现替代

---

## 🗑️ 删除建议总结

### 可以安全删除的脚本（2个）

1. **merge_imu_data.py**
   - ✅ 未被引用
   - ✅ 功能已被替代
   - ⚠️ 删除前确认：是否还有其他外部工具依赖

2. **merge_multi_imu_data.py**
   - ✅ 未被引用
   - ✅ 功能已被替代
   - ⚠️ 删除前确认：是否还有其他外部工具依赖

### 建议保留的脚本（1个）

1. **example_calibration_callback.py**
   - 🟡 作为示例/文档保留
   - 或者移到`examples/`目录

### 需要确认的脚本（2个）

1. **01_ros_env.sh** - 需要确认是否在Docker/启动脚本中使用
2. **getmp4_basetime.sh** - 可能已被Python实现替代

---

## 📋 详细检查结果

### merge_imu_data.py 引用检查
```bash
# 在scripts目录中搜索（排除自身）
grep -r "merge_imu_data" scripts/ --exclude="merge_imu_data.py"
```
**结果**: 
- 只在`create_rosbag_from_db.py`的注释中提到（"from merge_imu_data.py"）
- 没有实际的导入或调用

### merge_multi_imu_data.py 引用检查
```bash
grep -r "merge_multi_imu_data" scripts/ --exclude="merge_multi_imu_data.py"
```
**结果**: 
- 只在文档中提到
- 没有实际的导入或调用

### example_calibration_callback.py 引用检查
```bash
grep -r "example_calibration_callback" scripts/ --exclude="example_calibration_callback.py"
```
**结果**: 
- 只在文档中提到
- 没有实际的导入或调用

---

## 🎯 最终建议

### 立即可以删除
1. ✅ `merge_imu_data.py` - 功能已替代，未被使用
2. ✅ `merge_multi_imu_data.py` - 功能已替代，未被使用

### 建议保留（作为示例）
1. 🟡 `example_calibration_callback.py` - 保留作为文档/示例

### 需要进一步确认
1. ❓ `01_ros_env.sh` - 检查是否在Docker/启动流程中使用
2. ❓ `getmp4_basetime.sh` - 检查是否还在使用

---

## 📊 删除后的影响

### 删除 merge_imu_data.py 和 merge_multi_imu_data.py
- ✅ **无影响** - 未被任何脚本引用
- ✅ **功能完整** - `create_rosbag_from_db.py`已实现相同功能
- ⚠️ **唯一风险** - 如果有外部工具或脚本依赖这些文件

### 删除 example_calibration_callback.py
- ✅ **无功能影响** - 只是示例文件
- ⚠️ **文档缺失** - 用户可能失去一个有用的示例

---

## ✅ 检查清单

删除前请确认：

- [ ] 是否有CI/CD流程使用这些脚本？
- [ ] 是否有外部工具或脚本依赖这些文件？
- [ ] 是否有文档或README引用这些脚本？
- [ ] 是否将来可能需要CSV格式的IMU数据？
- [ ] 是否有其他项目或仓库依赖这些脚本？

---

## 📝 总结

**可以安全删除**: 2个脚本
- `merge_imu_data.py`
- `merge_multi_imu_data.py`

**建议保留**: 1个脚本
- `example_calibration_callback.py`（作为示例）

**需要确认**: 2个Shell脚本
- `01_ros_env.sh`
- `getmp4_basetime.sh`

