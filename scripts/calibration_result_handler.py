#!/usr/bin/env python3
"""
calibration_result_handler.py
接收和处理标定结果的类
"""

from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum
from datetime import datetime


class CalibrationType(Enum):
    """标定类型枚举"""
    IMU_INTRINSIC = "imu_intrinsic"
    CAMERA_INTRINSIC = "camera_intrinsic"
    CAMERA_IMU_EXTRINSIC = "camera_imu_extrinsic"


class CalibrationStatus(Enum):
    """标定状态枚举"""
    SUCCESS = "success"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class IMUCalibrationResult:
    """单个IMU标定结果"""
    imu_number: int  # 0, 1, or 2
    status: CalibrationStatus
    yaml_file: Optional[str] = None  # 输出YAML文件路径
    error_message: Optional[str] = None


@dataclass
class CameraCalibrationResult:
    """单个相机标定结果"""
    mode: str  # 'front', 'eye', 'left', 'right'
    status: CalibrationStatus
    rosbag_file: Optional[str] = None  # 生成的rosbag文件路径
    camchain_file: Optional[str] = None  # 相机链文件路径（仅外参标定）
    error_message: Optional[str] = None


@dataclass
class CalibrationResult:
    """完整的标定结果"""
    calibration_type: CalibrationType
    device_id: str
    timestamp: datetime
    imu_results: Optional[List[IMUCalibrationResult]] = None  # IMU内参标定结果
    camera_results: Optional[List[CameraCalibrationResult]] = None  # 相机标定结果
    overall_success: bool = False


class CalibrationResultHandler:
    """标定结果处理器"""
    
    def __init__(self, callback: Optional[Callable[[CalibrationResult], None]] = None):
        """
        初始化结果处理器
        
        Args:
            callback: 回调函数，接收 CalibrationResult 对象
        """
        self.callback = callback
        self.results: List[CalibrationResult] = []
    
    def handle_imu_intrinsic_result(
        self,
        device_id: str,
        imu_results: List[IMUCalibrationResult],
        callback: Optional[Callable[[CalibrationResult], None]] = None
    ) -> CalibrationResult:
        """
        处理IMU内参标定结果
        
        Args:
            device_id: 设备ID
            imu_results: IMU标定结果列表（3个：imu0, imu1, imu2）
            callback: 可选的回调函数（如果提供，会覆盖初始化时的回调）
            
        Returns:
            CalibrationResult 对象
        """
        overall_success = all(result.status == CalibrationStatus.SUCCESS for result in imu_results)
        
        result = CalibrationResult(
            calibration_type=CalibrationType.IMU_INTRINSIC,
            device_id=device_id,
            timestamp=datetime.now(),
            imu_results=imu_results,
            overall_success=overall_success
        )
        
        self.results.append(result)
        
        # 调用回调函数
        cb = callback if callback is not None else self.callback
        if cb:
            cb(result)
        
        return result
    
    def handle_camera_intrinsic_result(
        self,
        device_id: str,
        mode: str,
        status: CalibrationStatus,
        rosbag_file: Optional[str] = None,
        error_message: Optional[str] = None,
        callback: Optional[Callable[[CalibrationResult], None]] = None
    ) -> CalibrationResult:
        """
        处理单个相机内参标定结果
        
        Args:
            device_id: 设备ID
            mode: 标定模式 ('front', 'eye', 'left', 'right')
            status: 标定状态
            rosbag_file: rosbag文件路径
            error_message: 错误信息（如果失败）
            callback: 可选的回调函数
            
        Returns:
            CalibrationResult 对象
        """
        camera_result = CameraCalibrationResult(
            mode=mode,
            status=status,
            rosbag_file=rosbag_file,
            error_message=error_message
        )
        
        # 查找是否已有该设备ID和类型的标定结果
        existing_result = None
        for r in self.results:
            if (r.device_id == device_id and 
                r.calibration_type == CalibrationType.CAMERA_INTRINSIC):
                existing_result = r
                break
        
        if existing_result is None:
            # 创建新的结果对象
            result = CalibrationResult(
                calibration_type=CalibrationType.CAMERA_INTRINSIC,
                device_id=device_id,
                timestamp=datetime.now(),
                camera_results=[camera_result],
                overall_success=(status == CalibrationStatus.SUCCESS)
            )
            self.results.append(result)
        else:
            # 添加到现有结果
            if existing_result.camera_results is None:
                existing_result.camera_results = []
            existing_result.camera_results.append(camera_result)
            # 更新整体成功状态
            existing_result.overall_success = all(
                r.status == CalibrationStatus.SUCCESS 
                for r in existing_result.camera_results
            )
            result = existing_result
        
        # 调用回调函数
        cb = callback if callback is not None else self.callback
        if cb:
            cb(result)
        
        return result
    
    def handle_camera_imu_extrinsic_result(
        self,
        device_id: str,
        mode: str,
        status: CalibrationStatus,
        rosbag_file: Optional[str] = None,
        camchain_file: Optional[str] = None,
        error_message: Optional[str] = None,
        callback: Optional[Callable[[CalibrationResult], None]] = None
    ) -> CalibrationResult:
        """
        处理单个相机-IMU外参标定结果
        
        Args:
            device_id: 设备ID
            mode: 标定模式 ('front', 'eye', 'left', 'right')
            status: 标定状态
            rosbag_file: rosbag文件路径
            camchain_file: 相机链文件路径
            error_message: 错误信息（如果失败）
            callback: 可选的回调函数
            
        Returns:
            CalibrationResult 对象
        """
        camera_result = CameraCalibrationResult(
            mode=mode,
            status=status,
            rosbag_file=rosbag_file,
            camchain_file=camchain_file,
            error_message=error_message
        )
        
        # 查找是否已有该设备ID和类型的标定结果
        existing_result = None
        for r in self.results:
            if (r.device_id == device_id and 
                r.calibration_type == CalibrationType.CAMERA_IMU_EXTRINSIC):
                existing_result = r
                break
        
        if existing_result is None:
            # 创建新的结果对象
            result = CalibrationResult(
                calibration_type=CalibrationType.CAMERA_IMU_EXTRINSIC,
                device_id=device_id,
                timestamp=datetime.now(),
                camera_results=[camera_result],
                overall_success=(status == CalibrationStatus.SUCCESS)
            )
            self.results.append(result)
        else:
            # 添加到现有结果
            if existing_result.camera_results is None:
                existing_result.camera_results = []
            existing_result.camera_results.append(camera_result)
            # 更新整体成功状态
            existing_result.overall_success = all(
                r.status == CalibrationStatus.SUCCESS 
                for r in existing_result.camera_results
            )
            result = existing_result
        
        # 调用回调函数
        cb = callback if callback is not None else self.callback
        if cb:
            cb(result)
        
        return result
    
    def get_all_results(self) -> List[CalibrationResult]:
        """获取所有标定结果"""
        return self.results
    
    def get_results_by_type(self, calibration_type: CalibrationType) -> List[CalibrationResult]:
        """根据类型获取标定结果"""
        return [r for r in self.results if r.calibration_type == calibration_type]
    
    def get_results_by_device(self, device_id: str) -> List[CalibrationResult]:
        """根据设备ID获取标定结果"""
        return [r for r in self.results if r.device_id == device_id]
    
    def clear_results(self):
        """清空所有结果"""
        self.results.clear()


# 示例回调函数
def example_callback(result: CalibrationResult):
    """
    示例回调函数，展示如何处理标定结果
    
    Args:
        result: 标定结果对象
    """
    print(f"\n{'='*80}")
    print(f"标定结果回调 - {result.calibration_type.value}")
    print(f"{'='*80}")
    print(f"设备ID: {result.device_id}")
    print(f"时间戳: {result.timestamp}")
    print(f"整体状态: {'成功' if result.overall_success else '失败'}")
    
    if result.calibration_type == CalibrationType.IMU_INTRINSIC:
        print(f"\nIMU标定结果:")
        if result.imu_results:
            for imu_result in result.imu_results:
                status_str = "成功" if imu_result.status == CalibrationStatus.SUCCESS else "失败"
                print(f"  IMU{imu_result.imu_number}: {status_str}")
                if imu_result.yaml_file:
                    print(f"    YAML文件: {imu_result.yaml_file}")
                if imu_result.error_message:
                    print(f"    错误: {imu_result.error_message}")
    
    elif result.calibration_type in [CalibrationType.CAMERA_INTRINSIC, CalibrationType.CAMERA_IMU_EXTRINSIC]:
        print(f"\n相机标定结果:")
        if result.camera_results:
            for camera_result in result.camera_results:
                status_str = "成功" if camera_result.status == CalibrationStatus.SUCCESS else "失败"
                print(f"  {camera_result.mode}: {status_str}")
                if camera_result.rosbag_file:
                    print(f"    Rosbag文件: {camera_result.rosbag_file}")
                if camera_result.camchain_file:
                    print(f"    Camchain文件: {camera_result.camchain_file}")
                if camera_result.error_message:
                    print(f"    错误: {camera_result.error_message}")
    
    print(f"{'='*80}\n")
    
    # 这里可以添加上传到S3的逻辑
    # if result.overall_success:
    #     upload_to_s3(result)


if __name__ == "__main__":
    # 测试代码
    handler = CalibrationResultHandler(callback=example_callback)
    
    # 测试IMU内参标定结果
    imu_results = [
        IMUCalibrationResult(imu_number=0, status=CalibrationStatus.SUCCESS, yaml_file="/tmp/output/imu_mid_0.yaml"),
        IMUCalibrationResult(imu_number=1, status=CalibrationStatus.SUCCESS, yaml_file="/tmp/output/imu_right_1.yaml"),
        IMUCalibrationResult(imu_number=2, status=CalibrationStatus.FAILED, error_message="Timeout")
    ]
    handler.handle_imu_intrinsic_result("test_device", imu_results)
    
    # 测试相机内参标定结果
    handler.handle_camera_intrinsic_result(
        "test_device", "front", CalibrationStatus.SUCCESS,
        rosbag_file="/tmp/output/cam_lr_front_intrinsic.bag"
    )

