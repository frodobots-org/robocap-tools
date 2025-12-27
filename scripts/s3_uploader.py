#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S3上传器模块
负责将标定结果文件上传到S3
"""

import os
import sys
from typing import List, Optional
from pathlib import Path

# 添加s3sdk到路径
project_root = Path(__file__).parent.parent
s3sdk_path = project_root / "s3sdk"
if str(s3sdk_path) not in sys.path:
    sys.path.insert(0, str(s3sdk_path))

try:
    # 尝试从s3sdk目录导入
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
    """S3上传器 - 上传标定结果文件到S3"""
    
    def __init__(self, s3_config_file: Optional[str] = None):
        """
        初始化S3上传器
        
        Args:
            s3_config_file: S3配置文件路径（可选，默认使用s3sdk/s3_config.json）
        """
        self.s3_sdk = None
        
        if S3SDK is None:
            print("Warning: S3 SDK not available, upload functionality disabled")
            return
        
        try:
            if s3_config_file is None:
                # 默认配置文件路径
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
        上传标定结果文件到S3
        
        Args:
            local_file_path: 本地文件路径
            device_id: 设备ID
            task_type: 任务类型（用于确定S3路径）
            filename: S3中的文件名（可选，默认使用本地文件名）
            
        Returns:
            是否上传成功
        """
        if self.s3_sdk is None:
            print("S3 SDK not available, skipping upload")
            return False
        
        if not os.path.exists(local_file_path):
            print(f"File not found: {local_file_path}")
            return False
        
        # 构建S3路径
        if filename is None:
            filename = os.path.basename(local_file_path)
        
        # 根据任务类型确定S3路径
        s3_folder = self._get_s3_folder(device_id, task_type)
        s3_path = f"{s3_folder}/{filename}"
        
        try:
            print(f"[上传] {local_file_path} -> s3://{self.s3_sdk.config.bucket_name}/{s3_path}")
            success = self.s3_sdk.upload_file(local_file_path, s3_path)
            if success:
                print(f"[成功] 上传完成: {s3_path}")
            else:
                print(f"[失败] 上传失败: {s3_path}")
            return success
        except Exception as e:
            print(f"[错误] 上传异常: {e}")
            return False
    
    def _get_s3_folder(self, device_id: str, task_type: str) -> str:
        """
        根据任务类型获取S3文件夹路径
        
        Args:
            device_id: 设备ID
            task_type: 任务类型
            
        Returns:
            S3文件夹路径
        """
        # 任务类型到S3路径的映射
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
        批量上传文件
        
        Args:
            file_paths: 本地文件路径列表
            device_id: 设备ID
            task_type: 任务类型
            
        Returns:
            每个文件的上传结果列表
        """
        results = []
        for file_path in file_paths:
            success = self.upload_calibration_file(file_path, device_id, task_type)
            results.append(success)
        return results

