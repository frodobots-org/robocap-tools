#!/usr/bin/env python3
"""
calibration_result_handler.py
Class for receiving and processing calibration results
"""

from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum
from datetime import datetime


class CalibrationType(Enum):
    """Calibration type enumeration"""
    IMU_INTRINSIC = "imu_intrinsic"
    CAMERA_INTRINSIC = "camera_intrinsic"
    CAMERA_IMU_EXTRINSIC = "camera_imu_extrinsic"


class CalibrationStatus(Enum):
    """Calibration status enumeration"""
    SUCCESS = "success"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class IMUCalibrationResult:
    """Single IMU calibration result"""
    imu_number: int  # 0, 1, or 2
    status: CalibrationStatus
    yaml_file: Optional[str] = None  # Output YAML file path
    error_message: Optional[str] = None


@dataclass
class CameraCalibrationResult:
    """Single camera calibration result"""
    mode: str  # 'front', 'eye', 'left', 'right'
    status: CalibrationStatus
    rosbag_file: Optional[str] = None  # Generated rosbag file path
    camchain_file: Optional[str] = None  # Camera chain file path (extrinsic calibration only)
    error_message: Optional[str] = None


@dataclass
class CalibrationResult:
    """Complete calibration result"""
    calibration_type: CalibrationType
    device_id: str
    timestamp: datetime
    imu_results: Optional[List[IMUCalibrationResult]] = None  # IMU intrinsic calibration results
    camera_results: Optional[List[CameraCalibrationResult]] = None  # Camera calibration results
    overall_success: bool = False


class CalibrationResultHandler:
    """Calibration result handler"""
    
    def __init__(self, callback: Optional[Callable[[CalibrationResult], None]] = None):
        """
        Initialize result handler
        
        Args:
            callback: Callback function that receives CalibrationResult object
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
        Handle IMU intrinsic calibration result
        
        Args:
            device_id: Device ID
            imu_results: IMU calibration result list (3 items: imu0, imu1, imu2)
            callback: Optional callback function (if provided, overrides the callback set during initialization)
            
        Returns:
            CalibrationResult object
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
        
        # Call callback function
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
        Handle single camera intrinsic calibration result
        
        Args:
            device_id: Device ID
            mode: Calibration mode ('front', 'eye', 'left', 'right')
            status: Calibration status
            rosbag_file: rosbag file path
            error_message: Error message (if failed)
            callback: Optional callback function
            
        Returns:
            CalibrationResult object
        """
        camera_result = CameraCalibrationResult(
            mode=mode,
            status=status,
            rosbag_file=rosbag_file,
            error_message=error_message
        )
        
        # Check if result already exists for this device ID and type
        existing_result = None
        for r in self.results:
            if (r.device_id == device_id and 
                r.calibration_type == CalibrationType.CAMERA_INTRINSIC):
                existing_result = r
                break
        
        if existing_result is None:
            # Create new result object
            result = CalibrationResult(
                calibration_type=CalibrationType.CAMERA_INTRINSIC,
                device_id=device_id,
                timestamp=datetime.now(),
                camera_results=[camera_result],
                overall_success=(status == CalibrationStatus.SUCCESS)
            )
            self.results.append(result)
        else:
            # Add to existing result
            if existing_result.camera_results is None:
                existing_result.camera_results = []
            existing_result.camera_results.append(camera_result)
            # Update overall success status
            existing_result.overall_success = all(
                r.status == CalibrationStatus.SUCCESS 
                for r in existing_result.camera_results
            )
            result = existing_result
        
        # Call callback function
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
        Handle single camera-IMU extrinsic calibration result
        
        Args:
            device_id: Device ID
            mode: Calibration mode ('front', 'eye', 'left', 'right')
            status: Calibration status
            rosbag_file: rosbag file path
            camchain_file: Camera chain file path
            error_message: Error message (if failed)
            callback: Optional callback function
            
        Returns:
            CalibrationResult object
        """
        camera_result = CameraCalibrationResult(
            mode=mode,
            status=status,
            rosbag_file=rosbag_file,
            camchain_file=camchain_file,
            error_message=error_message
        )
        
        # Check if result already exists for this device ID and type
        existing_result = None
        for r in self.results:
            if (r.device_id == device_id and 
                r.calibration_type == CalibrationType.CAMERA_IMU_EXTRINSIC):
                existing_result = r
                break
        
        if existing_result is None:
            # Create new result object
            result = CalibrationResult(
                calibration_type=CalibrationType.CAMERA_IMU_EXTRINSIC,
                device_id=device_id,
                timestamp=datetime.now(),
                camera_results=[camera_result],
                overall_success=(status == CalibrationStatus.SUCCESS)
            )
            self.results.append(result)
        else:
            # Add to existing result
            if existing_result.camera_results is None:
                existing_result.camera_results = []
            existing_result.camera_results.append(camera_result)
            # Update overall success status
            existing_result.overall_success = all(
                r.status == CalibrationStatus.SUCCESS 
                for r in existing_result.camera_results
            )
            result = existing_result
        
        # Call callback function
        cb = callback if callback is not None else self.callback
        if cb:
            cb(result)
        
        return result
    
    def get_all_results(self) -> List[CalibrationResult]:
        """Get all calibration results"""
        return self.results
    
    def get_results_by_type(self, calibration_type: CalibrationType) -> List[CalibrationResult]:
        """Get calibration results by type"""
        return [r for r in self.results if r.calibration_type == calibration_type]
    
    def get_results_by_device(self, device_id: str) -> List[CalibrationResult]:
        """Get calibration results by device ID"""
        return [r for r in self.results if r.device_id == device_id]
    
    def clear_results(self):
        """Clear all results"""
        self.results.clear()


# Example callback function
def example_callback(result: CalibrationResult):
    """
    Example callback function demonstrating how to handle calibration results
    
    Args:
        result: Calibration result object
    """
    print(f"\n{'='*80}")
    print(f"Calibration result callback - {result.calibration_type.value}")
    print(f"{'='*80}")
    print(f"Device ID: {result.device_id}")
    print(f"Timestamp: {result.timestamp}")
    print(f"Overall status: {'Success' if result.overall_success else 'Failed'}")
    
    if result.calibration_type == CalibrationType.IMU_INTRINSIC:
        print(f"\nIMU calibration results:")
        if result.imu_results:
            for imu_result in result.imu_results:
                status_str = "Success" if imu_result.status == CalibrationStatus.SUCCESS else "Failed"
                print(f"  IMU{imu_result.imu_number}: {status_str}")
                if imu_result.yaml_file:
                    print(f"    YAML file: {imu_result.yaml_file}")
                if imu_result.error_message:
                    print(f"    Error: {imu_result.error_message}")
    
    elif result.calibration_type in [CalibrationType.CAMERA_INTRINSIC, CalibrationType.CAMERA_IMU_EXTRINSIC]:
        print(f"\nCamera calibration results:")
        if result.camera_results:
            for camera_result in result.camera_results:
                status_str = "Success" if camera_result.status == CalibrationStatus.SUCCESS else "Failed"
                print(f"  {camera_result.mode}: {status_str}")
                if camera_result.rosbag_file:
                    print(f"    Rosbag file: {camera_result.rosbag_file}")
                if camera_result.camchain_file:
                    print(f"    Camchain file: {camera_result.camchain_file}")
                if camera_result.error_message:
                    print(f"    Error: {camera_result.error_message}")
    
    print(f"{'='*80}\n")
    
    # Here you can add logic to upload to S3
    # if result.overall_success:
    #     upload_to_s3(result)


if __name__ == "__main__":
    # Test code
    handler = CalibrationResultHandler(callback=example_callback)
    
    # Test IMU intrinsic calibration result
    imu_results = [
        IMUCalibrationResult(imu_number=0, status=CalibrationStatus.SUCCESS, yaml_file="/tmp/output/imu_mid_0.yaml"),
        IMUCalibrationResult(imu_number=1, status=CalibrationStatus.SUCCESS, yaml_file="/tmp/output/imu_right_1.yaml"),
        IMUCalibrationResult(imu_number=2, status=CalibrationStatus.FAILED, error_message="Timeout")
    ]
    handler.handle_imu_intrinsic_result("test_device", imu_results)
    
    # Test camera intrinsic calibration result
    handler.handle_camera_intrinsic_result(
        "test_device", "front", CalibrationStatus.SUCCESS,
        rosbag_file="/tmp/output/cam_lr_front_intrinsic.bag"
    )

