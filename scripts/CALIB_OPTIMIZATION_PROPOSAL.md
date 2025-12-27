# Calib脚本优化方案

## 当前问题分析

### 1. 代码重复严重

**重复的函数：**
- `run_command()` - 三个脚本中都有，但实现略有不同
- `get_config_for_mode()` - cam_intrinsic 和 cam_imu_extrinsic 中都有
- `create_rosbag()` - 逻辑相似，但实现细节不同
- 参数解析逻辑 - 三个脚本都有类似的 argparse 代码
- 导入和初始化逻辑 - 都有相同的导入和错误处理

**重复的模式：**
- 都使用 `robocap_env` 获取配置
- 都使用 `CalibrationResultHandler` 处理结果
- 都有类似的错误处理和日志输出

### 2. 代码结构问题

- **函数式编程**：所有逻辑都是函数，没有封装
- **缺乏抽象**：没有基类或接口，难以复用
- **配置分散**：配置逻辑分散在各个函数中
- **错误处理不统一**：每个脚本的错误处理方式略有不同

### 3. 可维护性问题

- **修改困难**：如果要修改公共逻辑，需要改三个文件
- **测试困难**：函数式结构难以进行单元测试
- **扩展困难**：添加新功能需要修改多个地方

## 优化方案

### 方案1：提取公共工具类（推荐）

**优点：**
- 改动最小，风险低
- 保持现有脚本结构
- 逐步重构，不影响现有功能

**实现：**
1. 创建 `calibration_utils.py` - 包含所有公共函数
2. 创建 `rosbag_creator.py` - 统一的rosbag创建逻辑
3. 创建 `kalibr_executor.py` - 统一的kalibr执行逻辑
4. 修改现有脚本，使用公共工具类

### 方案2：面向对象重构（彻底）

**优点：**
- 代码结构清晰
- 易于扩展和维护
- 符合OOP设计原则

**实现：**
1. 创建基类 `BaseCalibrationScript`
2. 创建子类：
   - `IMUIntrinsicCalibration`
   - `CameraIntrinsicCalibration`
   - `CameraIMUExtrinsicCalibration`
3. 每个子类实现特定的标定逻辑

### 方案3：混合方案（平衡）

**优点：**
- 兼顾代码复用和灵活性
- 保持脚本的独立性
- 逐步改进

**实现：**
1. 提取公共工具类（方案1）
2. 创建基类封装通用逻辑（方案2的部分）
3. 脚本继承基类，但保持独立入口

## 推荐方案：方案1 + 部分方案2

### 第一步：提取公共工具类

创建以下文件：

1. **`calibration_common.py`** - 公共工具函数
   - `run_command()` - 统一的命令执行
   - `get_config_for_mode()` - 统一的配置获取
   - `setup_robocap_env()` - 统一的环境设置
   - `parse_common_args()` - 统一的参数解析

2. **`rosbag_creator.py`** - Rosbag创建器类
   ```python
   class RosbagCreator:
       def create(self, input_dir, output_bag, **kwargs) -> bool
   ```

3. **`kalibr_executor.py`** - Kalibr执行器类
   ```python
   class KalibrExecutor:
       def run_intrinsic_calibration(...) -> bool
       def run_extrinsic_calibration(...) -> bool
   ```

### 第二步：重构现有脚本

每个脚本简化为：
```python
from calibration_common import *
from rosbag_creator import RosbagCreator
from kalibr_executor import KalibrExecutor

def main():
    # 解析参数
    args = parse_common_args()
    
    # 创建rosbag
    creator = RosbagCreator()
    creator.create(...)
    
    # 执行标定
    executor = KalibrExecutor()
    executor.run_xxx(...)
```

## 具体优化点

### 1. 统一 `run_command()` 函数

**当前问题：**
- `calib_imus_intrinsic.py` 中的 `run_command()` 支持 background 模式
- `calib_cams_intrinsic.py` 和 `calib_cams_imu_extrinsic.py` 中的返回 tuple

**优化：**
```python
def run_command(
    cmd: List[str], 
    timeout: Optional[int] = None,
    cwd: Optional[str] = None,
    background: bool = False,
    capture_output: bool = True
) -> Union[subprocess.Popen, Tuple[int, str, str]]:
    """统一的命令执行函数"""
    ...
```

### 2. 统一 `create_rosbag()` 逻辑

**当前问题：**
- 三个脚本中都有类似的rosbag创建逻辑
- 参数略有不同（video_rate等）

**优化：**
```python
class RosbagCreator:
    def create(
        self,
        input_dir: str,
        output_bag: str,
        imu_src_rate: int = 200,
        video_rate: Optional[float] = None,
        **kwargs
    ) -> bool:
        """统一的rosbag创建逻辑"""
        ...
```

### 3. 统一配置获取

**当前问题：**
- `get_config_for_mode()` 在两个脚本中重复
- 返回值的结构略有不同

**优化：**
```python
@dataclass
class CalibrationConfig:
    input_dir: str
    output_rosbag: str
    camchain_file: Optional[str] = None
    imu_yaml_files: Optional[List[str]] = None

def get_config_for_mode(mode: str, calibration_type: str) -> CalibrationConfig:
    """统一的配置获取"""
    ...
```

### 4. 统一参数解析

**当前问题：**
- 三个脚本都有类似的 argparse 代码
- `--device-id` 参数处理重复

**优化：**
```python
def parse_common_args(description: str) -> argparse.Namespace:
    """统一的参数解析"""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--device-id', ...)
    parser.add_argument('--callback', ...)
    return parser.parse_args()
```

### 5. 统一错误处理和日志

**当前问题：**
- 错误处理方式不统一
- 日志输出格式不一致

**优化：**
```python
class CalibrationLogger:
    @staticmethod
    def step(step_num: int, description: str):
        """统一的步骤日志"""
        print("=" * 80)
        print(f"Step {step_num}: {description}")
        print("=" * 80)
    
    @staticmethod
    def error(message: str):
        """统一的错误日志"""
        print(f"✗ {message}")
    
    @staticmethod
    def success(message: str):
        """统一的成功日志"""
        print(f"✓ {message}")
```

## 优化后的代码结构

```
scripts/
├── calibration_common.py      # 公共工具函数
├── rosbag_creator.py          # Rosbag创建器
├── kalibr_executor.py         # Kalibr执行器
├── calibration_logger.py     # 统一日志
├── calib_imus_intrinsic.py    # 简化后的IMU内参脚本
├── calib_cams_intrinsic.py    # 简化后的相机内参脚本
└── calib_cams_imu_extrinsic.py # 简化后的外参脚本
```

## 预期效果

### 代码量减少
- **当前**：三个脚本共约 1500+ 行
- **优化后**：公共代码约 300 行，每个脚本约 200 行
- **减少**：约 40% 代码量

### 可维护性提升
- 公共逻辑集中管理
- 修改一处，所有脚本受益
- 更容易添加新功能

### 可测试性提升
- 工具类可以独立测试
- 脚本逻辑更清晰，易于测试

## 实施建议

1. **第一阶段**：提取 `calibration_common.py`，统一公共函数
2. **第二阶段**：创建 `RosbagCreator` 和 `KalibrExecutor` 类
3. **第三阶段**：重构现有脚本，使用新的工具类
4. **第四阶段**：添加单元测试

## 风险评估

- **低风险**：提取公共函数，不影响现有功能
- **中风险**：重构脚本逻辑，需要充分测试
- **建议**：先在测试环境验证，再应用到生产环境

