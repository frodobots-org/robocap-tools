#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosbag creator helper class
Unified encapsulation of rosbag creation logic
"""

import os
import sys
from typing import Optional

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from calibration_common import CommandExecutor, CalibrationLogger


class RosbagCreator:
    """Rosbag creator - unified encapsulation of rosbag creation logic"""
    
    def __init__(self, script_dir: Optional[str] = None):
        """
        Initialize Rosbag creator
        
        Args:
            script_dir: Script directory path (default uses directory where current script is located)
        """
        if script_dir is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
        self.script_dir = script_dir
        self.create_rosbag_script = os.path.join(self.script_dir, "create_rosbag_from_db.py")
    
    def create(
        self,
        input_dir: str,
        output_bag: str,
        imu_src_rate: int = 200,
        video_rate: Optional[float] = None,
        trim_start_seconds: Optional[float] = None,
        trim_end_seconds: Optional[float] = None
    ) -> bool:
        """
        Create rosbag file
        
        Args:
            input_dir: Input directory path
            output_bag: Output rosbag file path
            imu_src_rate: IMU source sampling rate (Hz, default 200)
            video_rate: Video frame rate (Hz, None means use original frame rate)
            trim_start_seconds: Trim start time (seconds, None means no trimming)
            trim_end_seconds: Trim end time (seconds, None means no trimming)
            
        Returns:
            Whether successful
        """
        CalibrationLogger.step(1, "Creating rosbag from database files...")
        CalibrationLogger.info(f"Input directory: {input_dir}")
        CalibrationLogger.info(f"Output rosbag: {output_bag}")
        
        if video_rate is not None:
            CalibrationLogger.info(f"Video frame rate: {video_rate} fps")
        else:
            CalibrationLogger.info(f"Video frame rate: Original (no downsampling)")
        
        CalibrationLogger.info(f"IMU source rate: {imu_src_rate} Hz")
        
        # Ensure output directory exists
        output_dir = os.path.dirname(output_bag)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
            CalibrationLogger.info(f"Created output directory: {output_dir}")
        
        # Check input directory
        if not os.path.isdir(input_dir):
            CalibrationLogger.error(f"Input directory does not exist: {input_dir}")
            return False
        
        # Check if script exists
        if not os.path.exists(self.create_rosbag_script):
            CalibrationLogger.error(f"Script not found: {self.create_rosbag_script}")
            return False
        
        # Build command
        cmd = [
            sys.executable,
            self.create_rosbag_script,
            input_dir,
            "-ir_src", str(imu_src_rate),
            "-o", output_bag
        ]
        
        # If video frame rate is specified, add parameter
        if video_rate is not None:
            cmd.extend(["-vr", str(video_rate)])
        
        CalibrationLogger.info(f"\nRunning command: {' '.join(cmd)}")
        
        # Execute command
        result = CommandExecutor.run(cmd, shell=False, check=False, capture_output=True)
        
        if isinstance(result, tuple):
            returncode, stdout, stderr = result
            
            if returncode != 0:
                CalibrationLogger.error(f"Failed to create rosbag (return code: {returncode})")
                if stderr:
                    CalibrationLogger.error(f"Stderr: {stderr}")
                if stdout:
                    CalibrationLogger.info(f"Stdout: {stdout}")
                return False
            
            if stdout:
                CalibrationLogger.info(stdout)
        
        # Verify if rosbag was created successfully
        if not os.path.exists(output_bag):
            CalibrationLogger.error(f"Rosbag file was not created: {output_bag}")
            return False
        
        # If trimming start/end time is needed, use rosbag tool
        if trim_start_seconds is not None or trim_end_seconds is not None:
            import rosbag
            import tempfile
            
            CalibrationLogger.info(f"\nTrimming rosbag: start={trim_start_seconds}s, end={trim_end_seconds}s")
            
            # Read original rosbag to get time range
            with rosbag.Bag(output_bag, 'r') as bag:
                start_time = bag.get_start_time()
                end_time = bag.get_end_time()
                duration = end_time - start_time
                
                # Calculate trimmed time range
                if trim_start_seconds is not None:
                    new_start = start_time + trim_start_seconds
                else:
                    new_start = start_time
                
                if trim_end_seconds is not None:
                    new_end = end_time - trim_end_seconds
                else:
                    new_end = end_time
                
                if new_start >= new_end:
                    CalibrationLogger.error(f"Invalid trim range: start={new_start}, end={new_end}")
                    return False
                
                CalibrationLogger.info(f"Original: {start_time:.2f} - {end_time:.2f} (duration: {duration:.2f}s)")
                CalibrationLogger.info(f"Trimmed: {new_start:.2f} - {new_end:.2f} (duration: {new_end - new_start:.2f}s)")
                
                # Create temporary file
                temp_bag = output_bag + ".tmp"
                
                # Write trimmed data
                with rosbag.Bag(temp_bag, 'w') as outbag:
                    for topic, msg, t in bag.read_messages():
                        if new_start <= t.to_sec() <= new_end:
                            outbag.write(topic, msg, t)
            
            # Replace original file
            import shutil
            shutil.move(temp_bag, output_bag)
            CalibrationLogger.success(f"Rosbag trimmed successfully")
        
        CalibrationLogger.success(f"Rosbag created successfully: {output_bag}")
        return True

