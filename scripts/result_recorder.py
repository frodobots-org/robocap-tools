#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Result recorder module
Responsible for recording calibration results to CSV file
"""

import os
import csv
from typing import Dict, Optional
from datetime import datetime
from pathlib import Path

from calibration_task import CalibrationTaskType, get_all_task_types
from extrinsic_result_parser import get_extrinsic_reprojection_errors


class ResultRecorder:
    """Result recorder - record calibration results to CSV file"""
    
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
        Initialize result recorder
        
        Args:
            csv_file_path: CSV file path
        """
        self.csv_file_path = csv_file_path
        self._ensure_csv_exists()
    
    def _ensure_csv_exists(self):
        """Ensure CSV file exists, create and write header if it doesn't exist"""
        if not os.path.exists(self.csv_file_path):
            # Ensure directory exists
            csv_dir = os.path.dirname(self.csv_file_path)
            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)
            
            # Create CSV file and write header
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
        Record all calibration results for a device
        
        Args:
            device_id: Device ID
            results: Mapping from task type to success status
            reprojection_errors: Reprojection error values for extrinsic calibration tasks
                                 Format: {task_type: {'cam0': value, 'cam1': value}}
        """
        # Read existing data
        existing_data = self._read_existing_data()
        
        # Check if device record already exists
        device_index = None
        for i, row in enumerate(existing_data):
            if len(row) > 0 and row[0] == device_id:
                device_index = i
                break
        
        # Build new row data
        new_row = [device_id, datetime.now().strftime('%Y-%m-%d %H:%M:%S')]
        
        # Add results for each task in order
        task_types = get_all_task_types()
        for task_type in task_types:
            success = results.get(task_type, False)
            new_row.append("Success" if success else "Failed")
            
            # If it's an extrinsic calibration task, add reprojection error values
            if task_type == CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC:
                # Dual camera task: record cam0 and cam1
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                cam1_error = errors.get('cam1', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
                new_row.append(f"{cam1_error:.6f}" if cam1_error else "")
            elif task_type == CalibrationTaskType.CAM_LR_EYE_EXTRINSIC:
                # Dual camera task: record cam0 and cam1
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                cam1_error = errors.get('cam1', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
                new_row.append(f"{cam1_error:.6f}" if cam1_error else "")
            elif task_type == CalibrationTaskType.CAM_L_EXTRINSIC:
                # Single camera task: only record cam0
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
            elif task_type == CalibrationTaskType.CAM_R_EXTRINSIC:
                # Single camera task: only record cam0
                errors = {}
                if reprojection_errors and task_type in reprojection_errors:
                    errors = reprojection_errors[task_type]
                cam0_error = errors.get('cam0', '')
                new_row.append(f"{cam0_error:.6f}" if cam0_error else "")
        
        # Update or add record
        if device_index is not None:
            existing_data[device_index] = new_row
        else:
            existing_data.append(new_row)
        
        # Write back to CSV file
        self._write_data(existing_data)
    
    def _read_existing_data(self) -> list:
        """Read existing CSV data"""
        if not os.path.exists(self.csv_file_path):
            return []
        
        data = []
        with open(self.csv_file_path, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            # Skip header
            next(reader, None)
            for row in reader:
                data.append(row)
        
        return data
    
    def _write_data(self, data: list):
        """Write data to CSV file"""
        with open(self.csv_file_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(self.CSV_HEADERS)
            writer.writerows(data)
    
    def get_device_result(self, device_id: str) -> Optional[Dict[CalibrationTaskType, bool]]:
        """
        Get calibration results for a device
        
        Args:
            device_id: Device ID
            
        Returns:
            Mapping from task type to success status, or None if device doesn't exist
        """
        existing_data = self._read_existing_data()
        
        for row in existing_data:
            if len(row) > 0 and row[0] == device_id:
                # Parse results
                results = {}
                task_types = get_all_task_types()
                
                # Skip device_id and timestamp (first 2 columns)
                for i, task_type in enumerate(task_types):
                    if i + 2 < len(row):
                        status_str = row[i + 2]
                        results[task_type] = (status_str == "Success")
                
                return results
        
        return None

