#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外参标定结果解析器
解析Kalibr生成的结果文件，提取reprojection error等信息
"""

import os
import re
from typing import Dict, Optional, List
from pathlib import Path


def parse_extrinsic_result_file(result_file_path: str) -> Dict[str, float]:
    """
    解析外参标定结果文件，提取每个摄像头的reprojection error mean值
    
    Args:
        result_file_path: 结果文件路径（.txt文件）
        
    Returns:
        字典，键为摄像头名称（如'cam0', 'cam1'），值为reprojection error mean值
        如果解析失败，返回空字典
    """
    if not os.path.exists(result_file_path):
        return {}
    
    reprojection_errors = {}
    
    try:
        with open(result_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # 匹配格式：Reprojection error (cam0) [px]:     mean 0.6064611271920474, ...
        # 或：Reprojection error (cam1) [px]:     mean 0.6047673621605816, ...
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
    查找外参标定结果文件
    
    Args:
        output_dir: 输出目录
        task_type: 任务类型（如'cam_lr_front_extrinsic'）
        
    Returns:
        结果文件路径，如果未找到则返回None
    """
    if not os.path.exists(output_dir):
        return None
    
    import glob
    
    # 尝试多种文件名模式
    patterns = [
        f"*{task_type}*results*.txt",
        f"*results*.txt",
        f"results-*.txt",
        f"*-results-*.txt"
    ]
    
    for pattern in patterns:
        files = glob.glob(os.path.join(output_dir, pattern))
        if files:
            # 优先选择包含任务类型名称的文件
            for f in files:
                if task_type in os.path.basename(f):
                    return f
            # 如果没有找到包含任务类型的，返回第一个
            return files[0]
    
    return None


def get_extrinsic_reprojection_errors(
    output_dir: str,
    task_type: str
) -> Dict[str, float]:
    """
    获取外参标定的reprojection error值
    
    Args:
        output_dir: 输出目录
        task_type: 任务类型
        
    Returns:
        字典，键为摄像头名称（如'cam0', 'cam1'），值为reprojection error mean值
    """
    result_file = find_extrinsic_result_file(output_dir, task_type)
    
    if result_file:
        return parse_extrinsic_result_file(result_file)
    
    return {}

