#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kalibr executor
Unified encapsulation of kalibr calibration command execution logic
"""

import os
import sys
import time
from typing import List, Optional

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import robocap_env
except ImportError:
    robocap_env = None

from calibration_common import CommandExecutor, CalibrationLogger


class KalibrExecutor:
    """Kalibr executor - unified encapsulation of kalibr calibration command execution"""
    
    def __init__(self):
        """Initialize Kalibr executor"""
        self.target_file = robocap_env.TARGET_FILE if robocap_env else None
        self.setup_file = robocap_env.KALIBR_SETUP_FILE if robocap_env else None
    
    def run_intrinsic_calibration(
        self,
        rosbag_path: str,
        camera_topics: List[str],
        timeout: int = 1800
    ) -> bool:
        """
        Run kalibr camera intrinsic calibration
        
        Args:
            rosbag_path: rosbag file path
            camera_topics: Camera topic list
            timeout: Timeout in seconds (default 1800)
            
        Returns:
            Whether successful
        """
        CalibrationLogger.step(2, "Running kalibr camera intrinsic calibration...")
        CalibrationLogger.info(f"Rosbag: {rosbag_path}")
        CalibrationLogger.info(f"Camera topics: {camera_topics}")
        CalibrationLogger.info(f"Model: pinhole-equi")
        CalibrationLogger.info(f"Target: {self.target_file}")
        CalibrationLogger.info(f"Timeout: {timeout} seconds")
        
        if not camera_topics:
            CalibrationLogger.error("No camera topics found in rosbag")
            return False
        
        # Verify files exist
        if not self._verify_files(rosbag_path, check_camchain=False):
            return False
        
        # Build kalibr command
        models = ['pinhole-equi'] * len(camera_topics)
        kalibr_cmd = [
            'rosrun kalibr kalibr_calibrate_cameras',
            '--bag', rosbag_path,
            '--topics'] + camera_topics + [
            '--models'] + models + [
            '--target', self.target_file,
            '--dont-show-report'
        ]
        
        return self._execute_kalibr_command(kalibr_cmd, timeout)
    
    def run_extrinsic_calibration(
        self,
        rosbag_path: str,
        camchain_file: str,
        imu_yaml_files: List[str],
        imu_models: Optional[List[str]] = None,
        timeout: int = 3600
    ) -> bool:
        """
        Run kalibr camera-IMU extrinsic calibration
        
        Args:
            rosbag_path: rosbag file path
            camchain_file: camchain file path
            imu_yaml_files: IMU YAML file list
            imu_models: IMU model list (optional, default uses 'calibrated')
            timeout: Timeout in seconds (default 3600)
            
        Returns:
            Whether successful
        """
        CalibrationLogger.step(2, "Running kalibr IMU-camera extrinsic calibration...")
        CalibrationLogger.info(f"Rosbag: {rosbag_path}")
        CalibrationLogger.info(f"Camchain file: {camchain_file}")
        CalibrationLogger.info(f"IMU YAML files: {imu_yaml_files}")
        
        # Determine IMU models
        if imu_models:
            final_imu_models = imu_models
        else:
            final_imu_models = ['calibrated'] * len(imu_yaml_files)
        
        CalibrationLogger.info(f"IMU models: {final_imu_models}")
        CalibrationLogger.info(f"Target: {self.target_file}")
        CalibrationLogger.info(f"Bag time range: 5-65 seconds")
        CalibrationLogger.info(f"Timeout: {timeout} seconds")
        
        # Verify files exist
        if not self._verify_files(rosbag_path, camchain_file=camchain_file):
            return False
        
        # Verify IMU YAML files
        if not self._verify_imu_files(imu_yaml_files):
            return False
        
        # Build kalibr command
        kalibr_cmd = [
            'rosrun kalibr kalibr_calibrate_imu_camera',
            '--perform-synchronization',
            '--bag', rosbag_path,
            '--timeoffset-padding', '0.1',
            '--cam', camchain_file,
            '--imu'] + imu_yaml_files + [
            '--imu-models'
        ] + final_imu_models + [
            '--target', self.target_file,
            '--dont-show-report'
        ]
        
        return self._execute_kalibr_command(kalibr_cmd, timeout)
    
    def _verify_files(
        self,
        rosbag_path: str,
        camchain_file: Optional[str] = None,
        check_camchain: bool = True
    ) -> bool:
        """Verify if required files exist"""
        if not os.path.exists(rosbag_path):
            CalibrationLogger.error(f"Rosbag file not found: {rosbag_path}")
            return False
        
        if check_camchain and camchain_file:
            if not os.path.exists(camchain_file):
                CalibrationLogger.error(f"Camchain file not found: {camchain_file}")
                CalibrationLogger.info("Please run camera intrinsic calibration first.")
                return False
        
        if not os.path.exists(self.target_file):
            CalibrationLogger.error(f"Kalibr target file not found: {self.target_file}")
            return False
        
        if not os.path.exists(self.setup_file):
            CalibrationLogger.error(f"Kalibr setup file not found: {self.setup_file}")
            return False
        
        return True
    
    def _verify_imu_files(self, imu_yaml_files: List[str]) -> bool:
        """Verify if IMU YAML files exist"""
        CalibrationLogger.section("Verifying IMU YAML files...")
        
        all_exist = True
        for imu_file in imu_yaml_files:
            if os.path.exists(imu_file):
                CalibrationLogger.success(f"Found: {imu_file}")
            else:
                CalibrationLogger.error(f"Missing: {imu_file}")
                all_exist = False
        
        if not all_exist:
            CalibrationLogger.error("Some IMU YAML files are missing. Please run IMU intrinsic calibration first.")
            return False
        
        return True
    
    def _execute_kalibr_command(self, kalibr_cmd: List[str], timeout: int) -> bool:
        """Execute kalibr command (real-time log output)"""
        import subprocess
        import threading
        import sys
        
        env_cmd = (
            f"source {self.setup_file} && "
            f"{' '.join(kalibr_cmd)}"
        )
        
        CalibrationLogger.info(f"\nRunning command: {env_cmd}")
        CalibrationLogger.info("-" * 80)
        
        # Execute command (real-time output)
        cmd = ['/bin/bash', '-c', env_cmd]
        start_time = time.time()
        
        # Use Popen for real-time output
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Read and output logs in real-time
        stdout_lines = []
        
        def read_output():
            """Thread function to read process output"""
            try:
                for line in process.stdout:
                    line = line.rstrip()
                    print(line)  # Real-time output to terminal
                    stdout_lines.append(line)
                    sys.stdout.flush()  # Ensure immediate output
            except Exception:
                pass
        
        # Start thread to read output
        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()
        
        # Wait for process to complete or timeout
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            CalibrationLogger.error(f"Kalibr calibration timed out after {timeout} seconds")
            return False
        
        # Wait for output thread to complete
        output_thread.join(timeout=1)
        
        elapsed_time = time.time() - start_time
        stdout = '\n'.join(stdout_lines)
        
        CalibrationLogger.info(f"\nKalibr calibration completed in {elapsed_time:.1f} seconds")
        
        if returncode != 0:
            CalibrationLogger.error(f"Kalibr calibration failed (return code: {returncode})")
            return False
        
        CalibrationLogger.success("Kalibr calibration completed successfully")
        return True

