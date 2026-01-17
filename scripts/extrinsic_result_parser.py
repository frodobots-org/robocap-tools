#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extrinsic calibration result parser
Parse Kalibr-generated result files, extract reprojection error and other information
"""

import os
import re
from typing import Dict, Optional, List
from pathlib import Path


def parse_extrinsic_result_file(result_file_path: str) -> Dict[str, float]:
    """
    Parse extrinsic calibration result file, extract reprojection error mean value for each camera
    
    Args:
        result_file_path: Result file path (.txt file)
        
    Returns:
        Dictionary with camera names (e.g., 'cam0', 'cam1') as keys and reprojection error mean values as values
        Returns empty dictionary if parsing fails
    """
    if not os.path.exists(result_file_path):
        return {}
    
    reprojection_errors = {}
    
    try:
        with open(result_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # Match format: Reprojection error (cam0) [px]:     mean 0.6064611271920474, ...
        # or: Reprojection error (cam1) [px]:     mean 0.6047673621605816, ...
        pattern = r'Reprojection error\s+\(cam(\d+)\)\s+\[px\]:\s+mean\s+([\d.]+)'
        
        matches = re.findall(pattern, content)
        
        for cam_num, mean_value in matches:
            cam_name = f'cam{cam_num}'
            try:
                reprojection_errors[cam_name] = float(mean_value)
            except ValueError:
                continue
                
    except Exception as e:
        print(f"Warning: Failed to parse result file {result_file_path}: {e}")
        return {}
    
    return reprojection_errors


def find_extrinsic_result_file(output_dir: str, task_type: str) -> Optional[str]:
    """
    Find extrinsic calibration result file
    
    Args:
        output_dir: Output directory
        task_type: Task type (e.g., 'cam_lr_front_extrinsic')
        
    Returns:
        Result file path, or None if not found
    """
    if not os.path.exists(output_dir):
        return None
    
    import glob
    
    # Try multiple filename patterns
    patterns = [
        f"*{task_type}*results-imucam.txt",
        f"*-results-imucam.txt",
        f"*{task_type}*results*.txt",
        f"*results*.txt",
        f"results-*.txt",
        f"*-results-*.txt"
    ]
    
    for pattern in patterns:
        files = glob.glob(os.path.join(output_dir, pattern))
        if files:
            # Prefer files containing task type name
            for f in files:
                if task_type in os.path.basename(f):
                    return f
            # If none found containing task type, return the first one
            return files[0]
    
    return None


def get_extrinsic_reprojection_errors(
    output_dir: str,
    task_type: str
) -> Dict[str, float]:
    """
    Get reprojection error values for extrinsic calibration
    
    Args:
        output_dir: Output directory
        task_type: Task type
        
    Returns:
        Dictionary with camera names (e.g., 'cam0', 'cam1') as keys and reprojection error mean values as values
    """
    result_file = find_extrinsic_result_file(output_dir, task_type)
    
    if result_file:
        return parse_extrinsic_result_file(result_file)
    
    return {}


def parse_extrinsic_imu_errors(result_file_path: str) -> Dict[str, float]:
    """
    Parse extrinsic calibration result file, extract IMU errors from Residuals section
    
    Args:
        result_file_path: Result file path (e.g., *-results-imucam.txt)
        
    Returns:
        Dictionary with error keys:
        - 'cam0_reprojection_error': float (mean value in px)
        - 'cam1_reprojection_error': float (mean value in px, optional)
        - 'imu0_gyroscope_error_mean': float (mean value in rad/s)
        - 'imu0_gyroscope_error_median': float (median value in rad/s)
        - 'imu0_accelerometer_error_mean': float (mean value in m/s²)
        - 'imu0_accelerometer_error_median': float (median value in m/s²)
        - ... (same for imu1 and imu2)
        Returns empty dictionary if parsing fails
    """
    if not os.path.exists(result_file_path):
        return {}
    
    errors = {}
    
    try:
        with open(result_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Find Residuals section
        residuals_start = content.find('Residuals')
        if residuals_start == -1:
            return {}
        
        # Extract content after Residuals section
        residuals_content = content[residuals_start:]
        
        # Pattern for reprojection error: Reprojection error (cam0) [px]:     mean 0.832, median 0.685, std: 0.596
        reproj_pattern = r'Reprojection error\s+\(cam(\d+)\)\s+\[px\]:\s+mean\s+([\d.]+),\s+median\s+([\d.]+)'
        reproj_matches = re.finditer(reproj_pattern, residuals_content)
        
        for match in reproj_matches:
            cam_num = match.group(1)
            mean_value = float(match.group(2))
            errors[f'cam{cam_num}_reprojection_error'] = mean_value
        
        # Pattern for gyroscope error: Gyroscope error (imu0) [rad/s]:     mean 0.01226, median 0.01087, std: 0.00727
        gyro_pattern = r'Gyroscope error\s+\(imu(\d+)\)\s+\[rad/s\]:\s+mean\s+([\d.]+),\s+median\s+([\d.]+)'
        gyro_matches = re.finditer(gyro_pattern, residuals_content)
        
        for match in gyro_matches:
            imu_num = match.group(1)
            mean_value = float(match.group(2))
            median_value = float(match.group(3))
            errors[f'imu{imu_num}_gyroscope_error_mean'] = mean_value
            errors[f'imu{imu_num}_gyroscope_error_median'] = median_value
        
        # Pattern for accelerometer error: Accelerometer error (imu0) [m/s^2]: mean 0.0903, median 0.0750, std: 0.0723
        accel_pattern = r'Accelerometer error\s+\(imu(\d+)\)\s+\[m/s\^2\]:\s+mean\s+([\d.]+),\s+median\s+([\d.]+)'
        accel_matches = re.finditer(accel_pattern, residuals_content)
        
        for match in accel_matches:
            imu_num = match.group(1)
            mean_value = float(match.group(2))
            median_value = float(match.group(3))
            errors[f'imu{imu_num}_accelerometer_error_mean'] = mean_value
            errors[f'imu{imu_num}_accelerometer_error_median'] = median_value
                
    except Exception as e:
        print(f"Warning: Failed to parse IMU errors from result file {result_file_path}: {e}")
        return {}
    
    return errors


def get_extrinsic_imu_errors(
    output_dir: str,
    task_type: str
) -> Dict[str, float]:
    """
    Get all extrinsic calibration errors (reprojection errors and IMU errors)
    
    Args:
        output_dir: Output directory
        task_type: Task type
        
    Returns:
        Dictionary with all error values (reprojection errors and IMU errors)
    """
    result_file = find_extrinsic_result_file(output_dir, task_type)
    
    if result_file:
        return parse_extrinsic_imu_errors(result_file)
    
    return {}
