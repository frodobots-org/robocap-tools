# 代码修改方案

## 概述

根据 `API_DOCUMENTATION.md` 文档，需要修改客户端代码 `scripts/results_report.py`，使其发送的请求符合 API 规范，特别是添加 `errcode` 和 `errmsg` 字段的支持，以及处理标定失败的情况。

## 设计代码范围

### 修改客户端代码

#### 结果报告模块
- **文件路径**: `scripts/results_report.py` (修改)
- **修改内容**:
  1. 在 `report_intrinsic_calibration` 方法中：
     - 添加 `errcode` 和 `errmsg` 参数
     - 当无法解析结果文件时（返回空字典），自动设置 `errcode = 1`
     - 对于单相机内参（left/right），当 `errcode ≠ 0` 时，calibration 字段可选（使用默认值）
     - 对于双相机内参（eye/front），`errcode` 和 `errmsg` 放在顶层，不在 cam0/cam1 内部
     - 即使失败，双相机模式仍需要提供 cam0 和 cam1 对象（使用默认值）
  
  2. 在 `report_extrinsic_calibration` 方法中：
     - 添加 `errcode` 和 `errmsg` 参数
     - 当无法解析结果文件时（返回空字典），自动设置 `errcode = 1`
     - 当 `errcode ≠ 0` 时，所有字段可选，缺失字段使用默认值 999.0
     - 对于 eye/front，需要 `cam1_reprojection_error`；对于 left/right，不需要

## 详细修改内容

### 客户端代码修改详情

#### 1. report_intrinsic_calibration 方法修改

**新增参数**:
- `errcode: int = 0` - 错误代码（0 = 成功，非零 = 失败）
- `errmsg: Optional[str] = None` - 错误消息

**逻辑修改**:
1. **失败检测**:
   - 如果 `params_dict` 为空（无法解析结果文件），自动设置 `errcode = 1`
   - 如果传入的 `errcode ≠ 0`，使用传入的值

2. **单相机模式（left/right）**:
   - **成功情况（errcode = 0）**: 所有字段必需（projection, projection_range, distortion, distortion_range）
   - **失败情况（errcode ≠ 0）**: 
     - calibration 字段可选
     - 如果存在部分数据，使用这些数据；否则不发送这些字段，让服务器使用默认值 [0.0, 0.0, 0.0, 0.0]
     - 始终发送 `errcode` 和 `errmsg`

3. **双相机模式（eye/front）**:
   - **成功情况（errcode = 0）**: cam0 和 cam1 的所有字段必需
   - **失败情况（errcode ≠ 0）**: 
     - cam0 和 cam1 对象始终必需（根据 API 文档）
     - 如果数据缺失，使用默认值 [0.0, 0.0, 0.0, 0.0]
     - `errcode` 和 `errmsg` 放在**顶层**，不在 cam0/cam1 内部

#### 2. report_extrinsic_calibration 方法修改

**新增参数**:
- `errcode: int = 0` - 错误代码（0 = 成功，非零 = 失败）
- `errmsg: Optional[str] = None` - 错误消息

**逻辑修改**:
1. **失败检测**:
   - 如果 `errors_dict` 为空（无法解析结果文件），自动设置 `errcode = 1`
   - 如果传入的 `errcode ≠ 0`，使用传入的值

2. **成功情况（errcode = 0）**:
   - 所有必需字段必须存在
   - 单相机（left/right）: 需要 `cam0_reprojection_error` 和所有 12 个 IMU 字段
   - 双相机（eye/front）: 需要 `cam0_reprojection_error`, `cam1_reprojection_error` 和所有 12 个 IMU 字段

3. **失败情况（errcode ≠ 0）**:
   - 所有字段可选
   - 缺失字段使用默认值 `999.0`
   - 对于 left/right，不发送 `cam1_reprojection_error`（API 不使用）
   - 对于 eye/front，如果缺失 `cam1_reprojection_error`，使用默认值 `999.0`

#### 3. 错误处理增强

- 在异常捕获中，尝试向 API 报告失败状态
- 使用适当的默认值构建失败请求
- 确保即使解析失败也能向服务器报告

## API 规范要点（参考）

### 单相机内参 (left/right)
- **成功**: 所有 calibration 字段必需
- **失败**: calibration 字段可选，缺失时服务器使用 [0.0, 0.0, 0.0, 0.0]

### 双相机内参 (eye/front)
- **成功**: cam0 和 cam1 的所有字段必需
- **失败**: cam0 和 cam1 对象始终必需，但字段可以使用默认值
- **重要**: errcode 和 errmsg 仅在顶层，不在 cam0/cam1 内部

### 外参 (所有类型)
- **成功**: 根据 cam_group 类型要求相应字段
- **失败**: 所有字段可选，缺失时使用默认值 999.0
- **注意**: left/right 不使用 cam1_reprojection_error

## 文件结构

```
robocap-tools/
├── scripts/
│   └── results_report.py          # 修改：添加 errcode/errmsg 支持
└── API_DOCUMENTATION.md           # API 文档（参考）
```

## 额外修改

### device_calibration_manager.py 修改

**修改内容**:
1. **`_execute_single_task` 方法**:
   - 即使标定失败（超时或异常退出），也会调用 `_report_calibration_results` 报告失败状态
   - 传递 `calibration_success` 和 `error_message` 参数

2. **`_report_calibration_results` 方法**:
   - 新增参数：`calibration_success` 和 `error_message`
   - 根据标定成功/失败状态设置 `errcode` 和 `errmsg`
   - 即使输出目录不存在（标定失败），也会尝试报告失败状态

### results_report.py 增强验证

**新增验证逻辑**:
1. **内参验证**:
   - 检查所有必需字段是否存在
   - 验证数组字段长度是否为 4
   - 验证字段类型是否正确
   - 如果验证失败，自动设置 `errcode = 1`

2. **外参验证**:
   - 检查 reprojection error 字段是否存在（根据单/双相机类型）
   - 检查所有 IMU 错误字段是否存在
   - 验证字段类型是否为数值类型
   - 如果验证失败，自动设置 `errcode = 1`

## 失败处理逻辑

### 失败情况 1: 标定执行失败（超时或异常退出）
- **检测位置**: `device_calibration_manager.py` 的 `_execute_single_task`
- **处理方式**: 
  - 设置 `calibration_success = False`
  - 提取 `error_message` 从 `execution_result`
  - 调用 `_report_calibration_results` 传递失败状态
  - 即使输出目录不存在，也会尝试报告失败

### 失败情况 2: 无法解析结果文件
- **检测位置**: `results_report.py` 的解析函数返回空字典
- **处理方式**: 
  - 自动设置 `errcode = 1`
  - 使用默认值构建请求（内参: [0.0, 0.0, 0.0, 0.0]，外参: 999.0）
  - 向 API 报告失败状态

### 失败情况 3: 解析到字段但字段不正确
- **检测位置**: `results_report.py` 的字段验证逻辑
- **处理方式**: 
  - 验证字段存在性、类型、长度
  - 如果验证失败，设置 `errcode = 1`
  - 使用默认值或部分可用数据构建请求
  - 向 API 报告失败状态

## 注意事项

1. **向后兼容**: 新增的 `errcode` 和 `errmsg` 参数有默认值，不影响现有调用
2. **自动失败检测**: 
   - 标定执行失败时自动报告
   - 无法解析结果文件时自动设置 `errcode = 1`
   - 字段验证失败时自动设置 `errcode = 1`
3. **默认值**: 严格按照 API 文档使用默认值（内参: [0.0, 0.0, 0.0, 0.0]，外参: 999.0）
4. **字段位置**: 双相机内参的 errcode/errmsg 必须在顶层，不在 cam0/cam1 内部
5. **错误处理**: 即使发生异常，也尝试向 API 报告失败状态
6. **始终报告**: 无论标定成功或失败，都会向 API 报告状态