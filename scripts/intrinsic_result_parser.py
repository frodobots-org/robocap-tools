#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Intrinsic calibration result parser
Parse Kalibr-generated intrinsic calibration result files, extract projection and distortion parameters
"""

import os
import re
from typing import Dict, Optional, List
from pathlib import Path


def parse_intrinsic_result_file(result_file_path: str) -> Dict[str, Dict]:
    """
    Parse intrinsic calibration result file, extract projection and distortion parameters for each camera
    
    Args:
        result_file_path: Result file path (e.g., *-intrinsic-results-cam.txt)
        
    Returns:
        Dictionary with camera names (e.g., 'cam0', 'cam1') as keys and parameter dictionaries as values
        Each parameter dictionary contains:
        - 'projection': [fx, fy, cx, cy]
        - 'projection_range': [fx_range, fy_range, cx_range, cy_range]
        - 'distortion': [k1, k2, p1, p2]
        - 'distortion_range': [k1_range, k2_range, p1_range, p2_range]
        Returns empty dictionary if parsing fails
    """
    if not os.path.exists(result_file_path):
        return {}
    
    camera_params = {}
    
    try:
        with open(result_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Find Camera-system parameters section
        if 'Camera-system parameters:' not in content:
            return {}
        
        # Pattern to match camera block:
        # cam0 (/cam0_left/image_raw):
        #     type: ...
        #     distortion: [ ... ] +- [ ... ]
        #     projection: [ ... ] +- [ ... ]
        
        # Extract all camera blocks - match cam0/1 lines and then extract distortion and projection
        # Use a more flexible pattern that allows for intermediate lines between distortion and projection
        cam_blocks = re.finditer(r'cam(\d+)\s*\([^)]+\):', content)
        
        matches = []
        for cam_match in cam_blocks:
            cam_num = cam_match.group(1)
            cam_start = cam_match.end()
            
            # Find the next camera or end of section
            next_cam = content.find(f'cam{int(cam_num)+1}' if int(cam_num) < 9 else 'Target configuration', cam_start)
            if next_cam == -1:
                next_cam = len(content)
            
            cam_block = content[cam_start:next_cam]
            
            # Extract distortion line
            distortion_match = re.search(r'distortion:\s*\[([^\]]+)\]\s+\+-\s+\[([^\]]+)\]', cam_block)
            # Extract projection line
            projection_match = re.search(r'projection:\s*\[([^\]]+)\]\s+\+-\s+\[([^\]]+)\]', cam_block)
            
            if distortion_match and projection_match:
                matches.append((cam_num, distortion_match, projection_match))
        
        for cam_num, distortion_match, projection_match in matches:
            cam_name = f'cam{cam_num}'
            
            # Parse distortion values and ranges
            distortion_str = distortion_match.group(1)
            distortion_range_str = distortion_match.group(2)
            
            # Parse projection values and ranges
            projection_str = projection_match.group(1)
            projection_range_str = projection_match.group(2)
            
            try:
                # Parse arrays (handle spaces and commas)
                distortion = [float(x.strip()) for x in re.split(r'[\s,]+', distortion_str.strip()) if x.strip()]
                distortion_range = [float(x.strip()) for x in re.split(r'[\s,]+', distortion_range_str.strip()) if x.strip()]
                projection = [float(x.strip()) for x in re.split(r'[\s,]+', projection_str.strip()) if x.strip()]
                projection_range = [float(x.strip()) for x in re.split(r'[\s,]+', projection_range_str.strip()) if x.strip()]
                
                if len(distortion) == 4 and len(distortion_range) == 4 and len(projection) == 4 and len(projection_range) == 4:
                    camera_params[cam_name] = {
                        'projection': projection,
                        'projection_range': projection_range,
                        'distortion': distortion,
                        'distortion_range': distortion_range
                    }
                else:
                    print(f"Warning: Invalid parameter length in camera {cam_name}")
                    
            except (ValueError, IndexError) as e:
                print(f"Warning: Failed to parse parameters for {cam_name}: {e}")
                continue
                
    except Exception as e:
        print(f"Warning: Failed to parse intrinsic result file {result_file_path}: {e}")
        return {}
    
    return camera_params


def find_intrinsic_result_file(output_dir: str, task_type: str) -> Optional[str]:
    """
    Find intrinsic calibration result file
    
    Args:
        output_dir: Output directory
        task_type: Task type (e.g., 'cam_l_intrinsic', 'cam_lr_eye_intrinsic')
        
    Returns:
        Result file path, or None if not found
    """
    if not os.path.exists(output_dir):
        return None
    
    import glob
    
    # Try multiple filename patterns
    patterns = [
        f"*-intrinsic-results-cam.txt",
        f"*{task_type}*results*.txt",
        f"*intrinsic*results*.txt",
    ]
    
    # Map task_type to strict matching patterns (avoid partial matches)
    task_type_strict_patterns = {
        'cam_lr_front_intrinsic': ['cam_lr_front_intrinsic', 'cam-lr-front-intrinsic'],
        'cam_lr_eye_intrinsic': ['cam_lr_eye_intrinsic', 'cam-lr-eye-intrinsic'],
        'cam_l_intrinsic': ['cam_l_intrinsic', 'cam-l-intrinsic'],  # Must NOT match cam_lr_*
        'cam_r_intrinsic': ['cam_r_intrinsic', 'cam-r-intrinsic'],  # Must NOT match cam_lr_*
    }
    
    strict_patterns = task_type_strict_patterns.get(task_type, [task_type])
    
    for pattern in patterns:
        files = glob.glob(os.path.join(output_dir, pattern))
        if files:
            # Filter out extrinsic files - only keep intrinsic files
            intrinsic_files = [f for f in files if 'intrinsic' in os.path.basename(f).lower() and 'extrinsic' not in os.path.basename(f).lower()]
            
            if not intrinsic_files:
                continue
            
            # Strict matching: file must match exact task type pattern
            for f in intrinsic_files:
                basename = os.path.basename(f).lower()
                
                # Check if file matches any strict pattern
                for strict_pattern in strict_patterns:
                    strict_lower = strict_pattern.lower()
                    if strict_lower in basename:
                        # Additional validation: reject partial matches
                        # For single camera (cam_l_intrinsic, cam_r_intrinsic), reject dual camera files
                        if task_type == 'cam_l_intrinsic':
                            # Reject if contains cam_lr_ (dual camera)
                            if 'cam_lr_' in basename:
                                continue
                        elif task_type == 'cam_r_intrinsic':
                            # Reject if contains cam_lr_ (dual camera)
                            if 'cam_lr_' in basename:
                                continue
                        elif task_type == 'cam_lr_front_intrinsic':
                            # Reject single camera files
                            if ('cam_l_intrinsic' in basename and 'cam_lr_' not in basename) or \
                               ('cam_r_intrinsic' in basename and 'cam_lr_' not in basename):
                                continue
                            # Reject eye files
                            if 'eye_intrinsic' in basename and 'front_intrinsic' not in basename:
                                continue
                        elif task_type == 'cam_lr_eye_intrinsic':
                            # Reject single camera files
                            if ('cam_l_intrinsic' in basename and 'cam_lr_' not in basename) or \
                               ('cam_r_intrinsic' in basename and 'cam_lr_' not in basename):
                                continue
                            # Reject front files
                            if 'front_intrinsic' in basename and 'eye_intrinsic' not in basename:
                                continue
                        
                        return f
            
            # Don't return wrong file - better to return None if no exact match
            # return intrinsic_files[0]  # Removed
    
    return None


def get_intrinsic_calibration_params(
    output_dir: str,
    task_type: str
) -> Dict[str, Dict]:
    """
    Get intrinsic calibration parameters
    
    Args:
        output_dir: Output directory
        task_type: Task type
        
    Returns:
        Dictionary with camera names as keys and parameter dictionaries as values
    """
    result_file = find_intrinsic_result_file(output_dir, task_type)
    
    if result_file:
        return parse_intrinsic_result_file(result_file)
    
    return {}

