#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Results report module
Responsible for reporting calibration results to the calibration database API
"""

import os
import sys
import requests
from typing import Dict, Optional
from pathlib import Path

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from calibration_task import CalibrationTaskType
from intrinsic_result_parser import get_intrinsic_calibration_params
from extrinsic_result_parser import get_extrinsic_imu_errors


class ResultsReporter:
    """Results reporter - report calibration results to API"""
    
    def __init__(self, api_base_url: Optional[str] = None, enabled: bool = True):
        """
        Initialize results reporter
        
        Args:
            api_base_url: API base URL (e.g., 'http://127.0.0.1:6001')
                         If None, will try to get from environment variable CALIBRATION_API_URL
            enabled: Whether reporting is enabled (default True)
        """
        self.enabled = enabled
        
        if api_base_url:
            self.api_base_url = api_base_url.rstrip('/')
        else:
            # Try to get from environment variable
            self.api_base_url = os.environ.get('CALIBRATION_API_URL', 'http://127.0.0.1:6001').rstrip('/')
        
        if not self.enabled:
            print("[Results Report] Reporting is disabled")
    
    def report_intrinsic_calibration(
        self,
        device_id: str,
        task_type: CalibrationTaskType,
        output_dir: str,
        errcode: int = 0,
        errmsg: Optional[str] = None
    ) -> bool:
        """
        Report intrinsic calibration results to API
        
        Args:
            device_id: Device ID
            task_type: Calibration task type
            output_dir: Output directory containing result files
            errcode: Error code (0 = success, non-zero = failure)
            errmsg: Error message (optional)
            
        Returns:
            Whether reporting was successful
        """
        if not self.enabled:
            return False
        
        try:
            # Get intrinsic parameters from result file
            params_dict = get_intrinsic_calibration_params(output_dir, task_type.value)
            
            # Map task type to API endpoint
            endpoint_map = {
                CalibrationTaskType.CAM_L_INTRINSIC: ('left', None),
                CalibrationTaskType.CAM_R_INTRINSIC: ('right', None),
                CalibrationTaskType.CAM_LR_EYE_INTRINSIC: ('eye', 'dual'),
                CalibrationTaskType.CAM_LR_FRONT_INTRINSIC: ('front', 'dual'),
            }
            
            if task_type not in endpoint_map:
                print(f"[Results Report] Error: Unsupported intrinsic task type: {task_type.value}")
                return False
            
            cam_type, camera_mode = endpoint_map[task_type]
            
            # Validate parsed parameters
            validation_failed = False
            validation_error = None
            
            if params_dict:
                # Check if all required fields exist and are valid
                if camera_mode == 'dual':
                    # Dual camera: need cam0 and cam1
                    if 'cam0' not in params_dict or 'cam1' not in params_dict:
                        validation_failed = True
                        validation_error = "Missing cam0 or cam1 in parsed parameters"
                    else:
                        # Validate each camera's fields
                        for cam_name in ['cam0', 'cam1']:
                            cam_data = params_dict[cam_name]
                            required_fields = ['projection', 'projection_range', 'distortion', 'distortion_range']
                            for field in required_fields:
                                if field not in cam_data:
                                    validation_failed = True
                                    validation_error = f"Missing {field} in {cam_name}"
                                    break
                                elif not isinstance(cam_data[field], list) or len(cam_data[field]) != 4:
                                    validation_failed = True
                                    validation_error = f"Invalid {field} in {cam_name}: expected list of 4 elements"
                                    break
                            if validation_failed:
                                break
                else:
                    # Single camera: need cam0
                    if 'cam0' not in params_dict:
                        validation_failed = True
                        validation_error = "Missing cam0 in parsed parameters"
                    else:
                        cam_data = params_dict['cam0']
                        required_fields = ['projection', 'projection_range', 'distortion', 'distortion_range']
                        for field in required_fields:
                            if field not in cam_data:
                                validation_failed = True
                                validation_error = f"Missing {field} in cam0"
                                break
                            elif not isinstance(cam_data[field], list) or len(cam_data[field]) != 4:
                                validation_failed = True
                                validation_error = f"Invalid {field} in cam0: expected list of 4 elements"
                                break
            
            # Determine if calibration failed (errcode != 0, no parameters found, or validation failed)
            if errcode != 0 or not params_dict or validation_failed:
                if not params_dict:
                    errcode = 1
                    errmsg = errmsg or f"No intrinsic parameters found for {task_type.value}"
                    print(f"[Results Report] Warning: {errmsg}")
                elif validation_failed:
                    errcode = 1
                    errmsg = errmsg or validation_error or "Invalid intrinsic parameters"
                    print(f"[Results Report] Warning: {errmsg}")
            
            # Build request data based on camera mode
            if camera_mode == 'dual':
                # Dual camera mode (eye/front)
                # According to API doc: cam0 and cam1 are always required, even when errcode != 0
                if errcode == 0:
                    # Success case: require all fields
                    if 'cam0' not in params_dict or 'cam1' not in params_dict:
                        print(f"[Results Report] Error: Missing cam0 or cam1 parameters for {task_type.value}")
                        return False
                    
                    request_data = {
                        'cam0': {
                            'projection': params_dict['cam0']['projection'],
                            'projection_range': params_dict['cam0']['projection_range'],
                            'distortion': params_dict['cam0']['distortion'],
                            'distortion_range': params_dict['cam0']['distortion_range']
                        },
                        'cam1': {
                            'projection': params_dict['cam1']['projection'],
                            'projection_range': params_dict['cam1']['projection_range'],
                            'distortion': params_dict['cam1']['distortion'],
                            'distortion_range': params_dict['cam1']['distortion_range']
                        },
                        'errcode': errcode,
                        'errmsg': errmsg
                    }
                else:
                    # Failure case: still need cam0 and cam1, but use defaults if missing
                    default_array = [0.0, 0.0, 0.0, 0.0]
                    cam0_data = params_dict.get('cam0', {}) if params_dict else {}
                    cam1_data = params_dict.get('cam1', {}) if params_dict else {}
                    
                    request_data = {
                        'cam0': {
                            'projection': cam0_data.get('projection', default_array),
                            'projection_range': cam0_data.get('projection_range', default_array),
                            'distortion': cam0_data.get('distortion', default_array),
                            'distortion_range': cam0_data.get('distortion_range', default_array)
                        },
                        'cam1': {
                            'projection': cam1_data.get('projection', default_array),
                            'projection_range': cam1_data.get('projection_range', default_array),
                            'distortion': cam1_data.get('distortion', default_array),
                            'distortion_range': cam1_data.get('distortion_range', default_array)
                        },
                        'errcode': errcode,
                        'errmsg': errmsg
                    }
                
                endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
            else:
                # Single camera mode (left/right)
                if errcode == 0:
                    # Success case: require all fields
                    if 'cam0' not in params_dict:
                        print(f"[Results Report] Error: Missing cam0 parameters for {task_type.value}")
                        return False
                    
                    request_data = {
                        'projection': params_dict['cam0']['projection'],
                        'projection_range': params_dict['cam0']['projection_range'],
                        'distortion': params_dict['cam0']['distortion'],
                        'distortion_range': params_dict['cam0']['distortion_range'],
                        'errcode': errcode,
                        'errmsg': errmsg
                    }
                else:
                    # Failure case: fields are optional, use defaults if missing
                    default_array = [0.0, 0.0, 0.0, 0.0]
                    cam0_data = params_dict.get('cam0', {}) if params_dict else {}
                    
                    request_data = {
                        'errcode': errcode,
                        'errmsg': errmsg
                    }
                    
                    # Add calibration fields only if available, otherwise let server use defaults
                    if cam0_data:
                        request_data['projection'] = cam0_data.get('projection', default_array)
                        request_data['projection_range'] = cam0_data.get('projection_range', default_array)
                        request_data['distortion'] = cam0_data.get('distortion', default_array)
                        request_data['distortion_range'] = cam0_data.get('distortion_range', default_array)
                
                endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
            
            # Send HTTP POST request
            return self._send_intrinsic_request(endpoint, request_data, device_id, cam_type)
            
        except Exception as e:
            print(f"[Results Report] Error reporting intrinsic calibration for {task_type.value}: {e}")
            # Try to report failure to API
            try:
                endpoint_map = {
                    CalibrationTaskType.CAM_L_INTRINSIC: ('left', None),
                    CalibrationTaskType.CAM_R_INTRINSIC: ('right', None),
                    CalibrationTaskType.CAM_LR_EYE_INTRINSIC: ('eye', 'dual'),
                    CalibrationTaskType.CAM_LR_FRONT_INTRINSIC: ('front', 'dual'),
                }
                if task_type in endpoint_map:
                    cam_type, camera_mode = endpoint_map[task_type]
                    if camera_mode == 'dual':
                        endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
                        request_data = {
                            'cam0': {
                                'projection': [0.0, 0.0, 0.0, 0.0],
                                'projection_range': [0.0, 0.0, 0.0, 0.0],
                                'distortion': [0.0, 0.0, 0.0, 0.0],
                                'distortion_range': [0.0, 0.0, 0.0, 0.0]
                            },
                            'cam1': {
                                'projection': [0.0, 0.0, 0.0, 0.0],
                                'projection_range': [0.0, 0.0, 0.0, 0.0],
                                'distortion': [0.0, 0.0, 0.0, 0.0],
                                'distortion_range': [0.0, 0.0, 0.0, 0.0]
                            },
                            'errcode': 1,
                            'errmsg': str(e)
                        }
                    else:
                        endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
                        request_data = {
                            'errcode': 1,
                            'errmsg': str(e)
                        }
                    self._send_intrinsic_request(endpoint, request_data, device_id, cam_type)
            except:
                pass
            return False
    
    def report_extrinsic_calibration(
        self,
        device_id: str,
        task_type: CalibrationTaskType,
        output_dir: str,
        errcode: int = 0,
        errmsg: Optional[str] = None
    ) -> bool:
        """
        Report extrinsic calibration results to API
        
        Args:
            device_id: Device ID
            task_type: Calibration task type
            output_dir: Output directory containing result files
            errcode: Error code (0 = success, non-zero = failure)
            errmsg: Error message (optional)
            
        Returns:
            Whether reporting was successful
        """
        if not self.enabled:
            return False
        
        try:
            # Get extrinsic errors from result file
            errors_dict = get_extrinsic_imu_errors(output_dir, task_type.value)
            
            # Map task type to API cam_group
            cam_group_map = {
                CalibrationTaskType.CAM_L_EXTRINSIC: 'left',
                CalibrationTaskType.CAM_R_EXTRINSIC: 'right',
                CalibrationTaskType.CAM_LR_EYE_EXTRINSIC: 'eye',
                CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC: 'front',
            }
            
            if task_type not in cam_group_map:
                print(f"[Results Report] Error: Unsupported extrinsic task type: {task_type.value}")
                return False
            
            cam_group = cam_group_map[task_type]
            
            # Validate parsed errors
            validation_failed = False
            validation_error = None
            
            if errors_dict:
                # Check required fields based on camera type
                if is_dual_camera:
                    # Dual camera: need cam0 and cam1 reprojection errors
                    if 'cam0_reprojection_error' not in errors_dict:
                        validation_failed = True
                        validation_error = "Missing cam0_reprojection_error"
                    elif 'cam1_reprojection_error' not in errors_dict:
                        validation_failed = True
                        validation_error = "Missing cam1_reprojection_error"
                else:
                    # Single camera: need cam0 reprojection error
                    if 'cam0_reprojection_error' not in errors_dict:
                        validation_failed = True
                        validation_error = "Missing cam0_reprojection_error"
                
                # Check IMU errors (imu0, imu1, imu2)
                if not validation_failed:
                    for imu_num in [0, 1, 2]:
                        required_fields = [
                            f'imu{imu_num}_gyroscope_error_mean',
                            f'imu{imu_num}_gyroscope_error_median',
                            f'imu{imu_num}_accelerometer_error_mean',
                            f'imu{imu_num}_accelerometer_error_median'
                        ]
                        for field in required_fields:
                            if field not in errors_dict:
                                validation_failed = True
                                validation_error = f"Missing {field}"
                                break
                            elif not isinstance(errors_dict[field], (int, float)):
                                validation_failed = True
                                validation_error = f"Invalid {field}: expected numeric value"
                                break
                        if validation_failed:
                            break
            
            # Determine if calibration failed (errcode != 0, no errors found, or validation failed)
            if errcode != 0 or not errors_dict or validation_failed:
                if not errors_dict:
                    errcode = 1
                    errmsg = errmsg or f"No extrinsic errors found for {task_type.value}"
                    print(f"[Results Report] Warning: {errmsg}")
                elif validation_failed:
                    errcode = 1
                    errmsg = errmsg or validation_error or "Invalid extrinsic errors"
                    print(f"[Results Report] Warning: {errmsg}")
            
            # Check if dual camera (eye/front) or single camera (left/right)
            is_dual_camera = cam_group in ['eye', 'front']
            
            # Default value for missing numeric fields when errcode != 0
            default_error_value = 999.0
            
            # Build request data
            request_data = {
                'errcode': errcode,
                'errmsg': errmsg
            }
            
            if errcode == 0:
                # Success case: require all fields
                # Add reprojection errors
                if 'cam0_reprojection_error' not in errors_dict:
                    print(f"[Results Report] Error: Missing cam0_reprojection_error for {task_type.value}")
                    return False
                request_data['cam0_reprojection_error'] = errors_dict['cam0_reprojection_error']
                
                if is_dual_camera:
                    if 'cam1_reprojection_error' not in errors_dict:
                        print(f"[Results Report] Error: Missing cam1_reprojection_error for {task_type.value}")
                        return False
                    request_data['cam1_reprojection_error'] = errors_dict['cam1_reprojection_error']
                
                # Add IMU errors (imu0, imu1, imu2)
                for imu_num in [0, 1, 2]:
                    imu_key = f'imu{imu_num}'
                    
                    # Check required fields
                    required_fields = [
                        f'{imu_key}_gyroscope_error_mean',
                        f'{imu_key}_gyroscope_error_median',
                        f'{imu_key}_accelerometer_error_mean',
                        f'{imu_key}_accelerometer_error_median'
                    ]
                    
                    for field in required_fields:
                        if field not in errors_dict:
                            print(f"[Results Report] Error: Missing {field} for {task_type.value}")
                            return False
                        request_data[field] = errors_dict[field]
            else:
                # Failure case: fields are optional, use defaults if missing
                request_data['cam0_reprojection_error'] = errors_dict.get('cam0_reprojection_error', default_error_value)
                
                if is_dual_camera:
                    request_data['cam1_reprojection_error'] = errors_dict.get('cam1_reprojection_error', default_error_value)
                
                # Add IMU errors (imu0, imu1, imu2) with defaults
                for imu_num in [0, 1, 2]:
                    imu_key = f'imu{imu_num}'
                    request_data[f'{imu_key}_gyroscope_error_mean'] = errors_dict.get(f'{imu_key}_gyroscope_error_mean', default_error_value)
                    request_data[f'{imu_key}_gyroscope_error_median'] = errors_dict.get(f'{imu_key}_gyroscope_error_median', default_error_value)
                    request_data[f'{imu_key}_accelerometer_error_mean'] = errors_dict.get(f'{imu_key}_accelerometer_error_mean', default_error_value)
                    request_data[f'{imu_key}_accelerometer_error_median'] = errors_dict.get(f'{imu_key}_accelerometer_error_median', default_error_value)
            
            endpoint = f"{self.api_base_url}/{device_id}/api/v1/extrinsic/{cam_group}"
            
            # Send HTTP POST request
            return self._send_extrinsic_request(endpoint, request_data, device_id, cam_group)
            
        except Exception as e:
            print(f"[Results Report] Error reporting extrinsic calibration for {task_type.value}: {e}")
            # Try to report failure to API
            try:
                cam_group_map = {
                    CalibrationTaskType.CAM_L_EXTRINSIC: 'left',
                    CalibrationTaskType.CAM_R_EXTRINSIC: 'right',
                    CalibrationTaskType.CAM_LR_EYE_EXTRINSIC: 'eye',
                    CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC: 'front',
                }
                if task_type in cam_group_map:
                    cam_group = cam_group_map[task_type]
                    is_dual_camera = cam_group in ['eye', 'front']
                    endpoint = f"{self.api_base_url}/{device_id}/api/v1/extrinsic/{cam_group}"
                    request_data = {
                        'cam0_reprojection_error': 999.0,
                        'imu0_gyroscope_error_mean': 999.0,
                        'imu0_gyroscope_error_median': 999.0,
                        'imu0_accelerometer_error_mean': 999.0,
                        'imu0_accelerometer_error_median': 999.0,
                        'imu1_gyroscope_error_mean': 999.0,
                        'imu1_gyroscope_error_median': 999.0,
                        'imu1_accelerometer_error_mean': 999.0,
                        'imu1_accelerometer_error_median': 999.0,
                        'imu2_gyroscope_error_mean': 999.0,
                        'imu2_gyroscope_error_median': 999.0,
                        'imu2_accelerometer_error_mean': 999.0,
                        'imu2_accelerometer_error_median': 999.0,
                        'errcode': 1,
                        'errmsg': str(e)
                    }
                    if is_dual_camera:
                        request_data['cam1_reprojection_error'] = 999.0
                    self._send_extrinsic_request(endpoint, request_data, device_id, cam_group)
            except:
                pass
            return False
    
    def _send_intrinsic_request(
        self,
        endpoint: str,
        data: Dict,
        device_id: str,
        cam_type: str
    ) -> bool:
        """
        Send intrinsic calibration data to API
        
        Args:
            endpoint: API endpoint URL
            data: Request data dictionary
            device_id: Device ID (for logging)
            cam_type: Camera type (for logging)
            
        Returns:
            Whether request was successful
        """
        try:
            print(f"[Results Report] Reporting intrinsic calibration: {device_id}/{cam_type}")
            print(f"[Results Report] Endpoint: {endpoint}")
            
            response = requests.post(
                endpoint,
                json=data,
                headers={'Content-Type': 'application/json'},
                timeout=30
            )
            
            if response.status_code == 201:
                result = response.json()
                is_passed = result.get('is_passed', False)
                status_text = "PASSED" if is_passed else "FAILED"
                print(f"[Results Report] ✓ Intrinsic calibration reported successfully ({status_text})")
                return True
            else:
                print(f"[Results Report] ✗ Failed to report intrinsic calibration: HTTP {response.status_code}")
                try:
                    error_msg = response.json().get('error', response.text)
                    print(f"[Results Report] Error message: {error_msg}")
                except:
                    print(f"[Results Report] Response: {response.text[:200]}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"[Results Report] ✗ Network error reporting intrinsic calibration: {e}")
            return False
        except Exception as e:
            print(f"[Results Report] ✗ Error reporting intrinsic calibration: {e}")
            return False
    
    def _send_extrinsic_request(
        self,
        endpoint: str,
        data: Dict,
        device_id: str,
        cam_group: str
    ) -> bool:
        """
        Send extrinsic calibration data to API
        
        Args:
            endpoint: API endpoint URL
            data: Request data dictionary
            device_id: Device ID (for logging)
            cam_group: Camera group (for logging)
            
        Returns:
            Whether request was successful
        """
        try:
            print(f"[Results Report] Reporting extrinsic calibration: {device_id}/{cam_group}")
            print(f"[Results Report] Endpoint: {endpoint}")
            
            response = requests.post(
                endpoint,
                json=data,
                headers={'Content-Type': 'application/json'},
                timeout=30
            )
            
            if response.status_code == 201:
                result = response.json()
                is_passed = result.get('is_passed', False)
                status_text = "PASSED" if is_passed else "FAILED"
                print(f"[Results Report] ✓ Extrinsic calibration reported successfully ({status_text})")
                return True
            else:
                print(f"[Results Report] ✗ Failed to report extrinsic calibration: HTTP {response.status_code}")
                try:
                    error_msg = response.json().get('error', response.text)
                    print(f"[Results Report] Error message: {error_msg}")
                except:
                    print(f"[Results Report] Response: {response.text[:200]}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"[Results Report] ✗ Network error reporting extrinsic calibration: {e}")
            return False
        except Exception as e:
            print(f"[Results Report] ✗ Error reporting extrinsic calibration: {e}")
            return False

