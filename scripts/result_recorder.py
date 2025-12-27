#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
结果记录器模块
负责将标定结果记录到CSV文件
"""

import os
import csv
from typing import Dict, Optional
from datetime import datetime
from pathlib import Path

from calibration_task import CalibrationTaskType, get_all_task_types
from extrinsic_result_parser import get_extrinsic_reprojection_errors


class ResultRecorder:
    """结果记录器 - 将标定结果记录到CSV文件"""
    
    CSV_HEADERS = [
        "device_id",
        "timestamp",
        "imu_intrinsic",
        "cam_lr_front_intrinsic",
        "cam_lr_front_extrinsic",
        "cam_lr_front_extrinsic_cam0_reproj_error",
        "cam_lr_front_extrinsic_cam1_reproj_error",
        "cam_lr_eye_intrinsic",
        "cam_lr_eye_extrinsic",
        "cam_lr_eye_extrinsic_cam0_reproj_error",
        "cam_lr_eye_extrinsic_cam1_reproj_error",
        "cam_l_intrinsic",
        "cam_l_extrinsic",
        "cam_l_extrinsic_cam0_reproj_error",
        "cam_r_intrinsic",
        "cam_r_extrinsic",
        "cam_r_extrinsic_cam0_reproj_error"
    ]
    
    def __init__(self, csv_file_path: str):
        """
        初始化结果记录器
        
        Args:
            csv_file_path: CSV文件路径
        """
        self.csv_file_path = csv_file_path
        self._ensure_csv_exists()
    
    def _ensure_csv_exists(self):
        """确保CSV文件存在，如果不存在则创建并写入表头"""
        if not os.path.exists(self.csv_file_path):
            # 确保目录存在
            csv_dir = os.path.dirname(self.csv_file_path)
            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)
            
            # 创建CSV文件并写入表头
            with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(self.CSV_HEADERS)
    
    def record_device_result(
        self,
        device_id: str,
        results: Dict[CalibrationTaskType, bool],
        reprojection_errors: Optional[Dict[CalibrationTaskType, Dict[str, float]]] = None
    ) -> None:
        """
        记录设备的所有标定结果
        
        Args:
            device_id: 设备ID
            results: 任务类型到成功状态的映射
            reprojection_errors: 外参标定任务的reprojection error值
                                 格式: {task_type: {'cam0': value, 'cam1': value}}
        """
        # 读取现有数据
        existing_data = self._read_existing_data()
        
        # 检查是否已存在该设备记录
        device_index = None
        for i, row in enumerate(existing_data):
            if len(row) > 0 and row[0] == device_id:
                device_index = i
                break
        
        # 构建新行数据
        new_row = [device_id, datetime.now().strftime('%Y-%m-%d %H:%M:%S')]
        
        # 按顺序添加每个任务的结果
        task_types = get_all_task_types()
        for task_type in task_types:
            success = results.get(task_type, False)
            new_row.append("成功" if success else "失败")
            
            # 如果是外参标定任务，添加reprojection error值
            if task_type == CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC:
                # 双摄像头任务：记录cam0和cam1
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                cam1_error = errors.get('cam1', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
                new_row.append(f"{cam1_error:.6f}" if cam1_error else "")
            elif task_type == CalibrationTaskType.CAM_LR_EYE_EXTRINSIC:
                # 双摄像头任务：记录cam0和cam1
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                cam1_error = errors.get('cam1', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
                new_row.append(f"{cam1_error:.6f}" if cam1_error else "")
            elif task_type == CalibrationTaskType.CAM_L_EXTRINSIC:
                # 单摄像头任务：只记录cam0
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
            elif task_type == CalibrationTaskType.CAM_R_EXTRINSIC:
                # 单摄像头任务：只记录cam0
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
        
        # 更新或添加记录
        if device_index is not None:
            existing_data[device_index] = new_row
        else:
            existing_data.append(new_row)
        
        # 写回CSV文件
        self._write_data(existing_data)
    
    def _read_existing_data(self) -> list:
        """读取现有CSV数据"""
        if not os.path.exists(self.csv_file_path):
            return []
        
        data = []
        with open(self.csv_file_path, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            # 跳过表头
            next(reader, None)
            for row in reader:
                data.append(row)
        
        return data
    
    def _write_data(self, data: list):
        """写入数据到CSV文件"""
        with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(self.CSV_HEADERS)
            writer.writerows(data)
    
    def get_device_result(self, device_id: str) -> Optional[Dict[CalibrationTaskType, bool]]:
        """
        获取设备的标定结果
        
        Args:
            device_id: 设备ID
            
        Returns:
            任务类型到成功状态的映射，如果设备不存在则返回None
        """
        existing_data = self._read_existing_data()
        
        for row in existing_data:
            if len(row) > 0 and row[0] == device_id:
                # 解析结果
                results = {}
                task_types = get_all_task_types()
                
                # 跳过device_id和timestamp（前2列）
                for i, task_type in enumerate(task_types):
                    if i + 2 < len(row):
                        status_str = row[i + 2]
                        results[task_type] = (status_str == "成功")
                
                return results
        
        return None

