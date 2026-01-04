#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S3 uploader module
Responsible for uploading calibration result files to S3
"""

import os
import sys
from typing import List, Optional
from pathlib import Path

# Add s3sdk to path
project_root = Path(__file__).parent.parent
s3sdk_path = project_root / "s3sdk"
if str(s3sdk_path) not in sys.path:
    sys.path.insert(0, str(s3sdk_path))

try:
    # Try to import from s3sdk directory
    s3sdk_sys_path = str(s3sdk_path)
    if s3sdk_sys_path not in sys.path:
        sys.path.insert(0, s3sdk_sys_path)
    
    from s3_sdk import S3SDK, S3Config
    from config_reader import load_s3_config
except ImportError as e:
    print(f"Warning: Could not import S3 SDK: {e}")
    S3SDK = None
    S3Config = None
    load_s3_config = None


class S3Uploader:
    """S3 uploader - upload calibration result files to S3"""
    
    def __init__(self, s3_config_file: Optional[str] = None):
        """
        Initialize S3 uploader
        
        Args:
            s3_config_file: S3 configuration file path (optional, default uses s3sdk/s3_config.json)
        """
        self.s3_sdk = None
        
        if S3SDK is None:
            print("Warning: S3 SDK not available, upload functionality disabled")
            return
        
        try:
            if s3_config_file is None:
                # Default configuration file path
                default_config = project_root / "s3sdk" / "s3_config.json"
                if default_config.exists():
                    s3_config_file = str(default_config)
                else:
                    print(f"Warning: S3 config file not found at {default_config}")
                    return
            
            config = load_s3_config(s3_config_file)
            self.s3_sdk = S3SDK(config)
            print(f"S3 Uploader initialized with bucket: {config.bucket_name}")
        except Exception as e:
            print(f"Warning: Failed to initialize S3 uploader: {e}")
            self.s3_sdk = None
    
    def upload_calibration_file(
        self,
        local_file_path: str,
        device_id: str,
        task_type: str,
        filename: Optional[str] = None
    ) -> bool:
        """
        Upload calibration result file to S3
        
        Args:
            local_file_path: Local file path
            device_id: Device ID
            task_type: Task type (used to determine S3 path)
            filename: Filename in S3 (optional, default uses local filename)
            
        Returns:
            Whether upload was successful
        """
        if self.s3_sdk is None:
            print("S3 SDK not available, skipping upload")
            return False
        
        if not os.path.exists(local_file_path):
            print(f"File not found: {local_file_path}")
            return False
        
        # Build S3 path
        if filename is None:
            filename = os.path.basename(local_file_path)
        
        # Determine S3 path based on task type
        s3_folder = self._get_s3_folder(device_id, task_type)
        s3_path = f"{s3_folder}/{filename}"
        
        try:
            print(f"[Upload] {local_file_path} -> s3://{self.s3_sdk.config.bucket_name}/{s3_path}")
            success = self.s3_sdk.upload_file(local_file_path, s3_path)
            if success:
                print(f"[Success] Upload completed: {s3_path}")
            else:
                print(f"[Failed] Upload failed: {s3_path}")
            return success
        except Exception as e:
            print(f"[Error] Upload exception: {e}")
            return False
    
    def _get_s3_folder(self, device_id: str, task_type: str) -> str:
        """
        Get S3 folder path based on task type
        
        Args:
            device_id: Device ID
            task_type: Task type
            
        Returns:
            S3 folder path
        """
        # Mapping from task type to S3 path
        type_to_folder = {
            "imu_intrinsic": f"{device_id}/v1/results/imus_intrinsic",
            "cam_lr_front_intrinsic": f"{device_id}/v1/results/imus_cam_lr_front_extrinsic",
            "cam_lr_front_extrinsic": f"{device_id}/v1/results/imus_cam_lr_front_extrinsic",
            "cam_lr_eye_intrinsic": f"{device_id}/v1/results/imus_cam_lr_eye_extrinsic",
            "cam_lr_eye_extrinsic": f"{device_id}/v1/results/imus_cam_lr_eye_extrinsic",
            "cam_l_intrinsic": f"{device_id}/v1/results/imus_cam_l_extrinsic",
            "cam_l_extrinsic": f"{device_id}/v1/results/imus_cam_l_extrinsic",
            "cam_r_intrinsic": f"{device_id}/v1/results/imus_cam_r_extrinsic",
            "cam_r_extrinsic": f"{device_id}/v1/results/imus_cam_r_extrinsic",
        }
        
        return type_to_folder.get(task_type, f"{device_id}/v1/results/unknown")
    
    def upload_multiple_files(
        self,
        file_paths: List[str],
        device_id: str,
        task_type: str
    ) -> List[bool]:
        """
        Batch upload files
        
        Args:
            file_paths: List of local file paths
            device_id: Device ID
            task_type: Task type
            
        Returns:
            List of upload results for each file
        """
        results = []
        for file_path in file_paths:
            success = self.upload_calibration_file(file_path, device_id, task_type)
            results.append(success)
        return results

