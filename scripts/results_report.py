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
        output_dir: str
    ) -> bool:
        """
        Report intrinsic calibration results to API
        
        Args:
            device_id: Device ID
            task_type: Calibration task type
            output_dir: Output directory containing result files
            
        Returns:
            Whether reporting was successful
        """
        if not self.enabled:
            return False
        
        try:
            # Get intrinsic parameters from result file
            params_dict = get_intrinsic_calibration_params(output_dir, task_type.value)
            
            if not params_dict:
                print(f"[Results Report] Warning: No intrinsic parameters found for {task_type.value}")
                return False
            
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
            
            # Build request data based on camera mode
            if camera_mode == 'dual':
                # Dual camera mode (eye/front)
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
                    }
                }
                
                endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
            else:
                # Single camera mode (left/right)
                if 'cam0' not in params_dict:
                    print(f"[Results Report] Error: Missing cam0 parameters for {task_type.value}")
                    return False
                
                request_data = {
                    'projection': params_dict['cam0']['projection'],
                    'projection_range': params_dict['cam0']['projection_range'],
                    'distortion': params_dict['cam0']['distortion'],
                    'distortion_range': params_dict['cam0']['distortion_range']
                }
                
                endpoint = f"{self.api_base_url}/{device_id}/api/v1/intrinsic/{cam_type}"
            
            # Send HTTP POST request
            return self._send_intrinsic_request(endpoint, request_data, device_id, cam_type)
            
        except Exception as e:
            print(f"[Results Report] Error reporting intrinsic calibration for {task_type.value}: {e}")
            return False
    
    def report_extrinsic_calibration(
        self,
        device_id: str,
        task_type: CalibrationTaskType,
        output_dir: str
    ) -> bool:
        """
        Report extrinsic calibration results to API
        
        Args:
            device_id: Device ID
            task_type: Calibration task type
            output_dir: Output directory containing result files
            
        Returns:
            Whether reporting was successful
        """
        if not self.enabled:
            return False
        
        try:
            # Get extrinsic errors from result file
            errors_dict = get_extrinsic_imu_errors(output_dir, task_type.value)
            
            if not errors_dict:
                print(f"[Results Report] Warning: No extrinsic errors found for {task_type.value}")
                return False
            
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
            
            # Check if dual camera (eye/front) or single camera (left/right)
            is_dual_camera = cam_group in ['eye', 'front']
            
            # Build request data
            request_data = {}
            
            # Add reprojection errors
            if 'cam0_reprojection_error' in errors_dict:
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
            
            endpoint = f"{self.api_base_url}/{device_id}/api/v1/extrinsic/{cam_group}"
            
            # Send HTTP POST request
            return self._send_extrinsic_request(endpoint, request_data, device_id, cam_group)
            
        except Exception as e:
            print(f"[Results Report] Error reporting extrinsic calibration for {task_type.value}: {e}")
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

