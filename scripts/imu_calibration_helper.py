#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU标定辅助类
封装IMU内参标定的特定逻辑
"""

import os
import sys
import subprocess
import time
from typing import Optional, Tuple

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import robocap_env
except ImportError:
    robocap_env = None

from calibration_common import CommandExecutor, CalibrationLogger
from rosbag_helper import RosbagHelper


class IMUCalibrationHelper:
    """IMU标定辅助类 - 封装IMU内参标定的特定逻辑"""
    
    def __init__(self):
        """初始化IMU标定辅助类"""
        self.setup_file = robocap_env.IMU_UTILS_SETUP_FILE if robocap_env else None
        self.data_dir = "/catkin_ws_imu/src/imu_utils/data"
    
    def run_roslaunch(self, imu_num: int, launch_file: str) -> Optional[subprocess.Popen]:
        """
        运行roslaunch进程
        
        Args:
            imu_num: IMU编号（0, 1, 2）
            launch_file: launch文件路径
            
        Returns:
            subprocess.Popen对象或None
        """
        if not os.path.exists(launch_file):
            CalibrationLogger.warning(f"Launch file does not exist: {launch_file}")
            return None
        
        launch_file_name = os.path.basename(launch_file)
        cmd = (
            f"cd /catkin_ws_imu && "
            f"source {self.setup_file} && "
            f"roslaunch imu_utils {launch_file_name}"
        )
        
        CalibrationLogger.info(f"Starting roslaunch for imu{imu_num}...")
        CalibrationLogger.info(f"  Launch file: {launch_file}")
        CalibrationLogger.info(f"  Command: {cmd}")
        
        log_file_path = f"/tmp/roslaunch_imu{imu_num}.log"
        with open(log_file_path, 'w') as log_file:
            process = subprocess.Popen(
                ["/bin/bash", "-c", cmd],
                stdout=log_file,
                stderr=subprocess.STDOUT
            )
        CalibrationLogger.info(f"  Started (PID: {process.pid}, log: {log_file_path})")
        
        return process
    
    def wait_for_completion(self, process: subprocess.Popen, imu_num: int, timeout: int = 180) -> bool:
        """
        等待roslaunch进程完成
        
        Args:
            process: subprocess.Popen对象
            imu_num: IMU编号
            timeout: 超时时间（秒，默认180）
            
        Returns:
            是否成功完成
        """
        if process is None:
            return False
        
        CalibrationLogger.section(f"Waiting for roslaunch process (imu{imu_num}) to complete...")
        CalibrationLogger.info(f"Timeout: {timeout} seconds")
        
        start_time = time.time()
        time.sleep(3)  # 给进程一些启动时间
        
        # 检查初始状态
        CalibrationLogger.info("\nInitial process status:")
        status = "running" if process.poll() is None else f"exited (code: {process.returncode})"
        CalibrationLogger.info(f"  imu{imu_num} (PID: {process.pid}): {status}")
        
        # 检查日志文件
        log_file = f"/tmp/roslaunch_imu{imu_num}.log"
        if os.path.exists(log_file):
            try:
                with open(log_file, 'r') as f:
                    log_content = f.read()
                    if log_content:
                        CalibrationLogger.info(f"    Log file size: {len(log_content)} bytes")
                        if "error" in log_content.lower() or "Error" in log_content:
                            lines = log_content.split('\n')
                            error_lines = [l for l in lines if "error" in l.lower() or "Error" in l]
                            if error_lines:
                                CalibrationLogger.warning(f"    Recent errors:")
                                for err_line in error_lines[-3:]:
                                    CalibrationLogger.info(f"      {err_line[:100]}")
            except Exception as e:
                CalibrationLogger.warning(f"    Could not read log file: {e}")
        
        # 等待进程完成
        while True:
            if process.poll() is not None:
                return_code = process.returncode
                if return_code == 0:
                    CalibrationLogger.success(f"Roslaunch process (imu{imu_num}) completed successfully!")
                    return True
                else:
                    CalibrationLogger.error(f"Roslaunch process (imu{imu_num}) failed with return code {return_code}")
                    return False
            
            elapsed = time.time() - start_time
            if elapsed > timeout:
                CalibrationLogger.error(f"Timeout after {timeout} seconds")
                CalibrationLogger.info(f"Stopping roslaunch process (imu{imu_num})...")
                
                if process.poll() is None:
                    CalibrationLogger.info(f"  Terminating imu{imu_num} process (PID: {process.pid})...")
                    try:
                        process.terminate()
                        time.sleep(2)
                        if process.poll() is None:
                            process.kill()
                            CalibrationLogger.info(f"  Force killed imu{imu_num} process")
                    except Exception as e:
                        CalibrationLogger.error(f"  Error stopping imu{imu_num} process: {e}")
                
                return False
            
            time.sleep(2)
            elapsed_int = int(elapsed)
            print(f"Waiting... ({elapsed_int}s / {timeout}s elapsed)", end='\r')
        
        print()
    
    def extract_parameters(self, imu_num: int) -> Tuple[bool, Optional[str]]:
        """
        提取IMU参数
        
        Args:
            imu_num: IMU编号（0, 1, 2）
            
        Returns:
            (是否成功, 输出YAML文件路径)
        """
        CalibrationLogger.section(f"Extracting IMU parameters for imu{imu_num}...")
        
        input_file = os.path.join(self.data_dir, f"imu{imu_num}_imu_param.yaml")
        
        # 获取输出文件路径
        if robocap_env:
            if imu_num == 0:
                output_file = robocap_env.YAML_FILE_IMU_MID_0
            elif imu_num == 1:
                output_file = robocap_env.YAML_FILE_IMU_RIGHT_1
            elif imu_num == 2:
                output_file = robocap_env.YAML_FILE_IMU_LEFT_2
            else:
                CalibrationLogger.error(f"Invalid IMU number: {imu_num}. Must be 0, 1, or 2.")
                return False, None
        else:
            CalibrationLogger.error("robocap_env not available")
            return False, None
        
        # 确保输出目录存在
        output_dir = os.path.dirname(output_file)
        os.makedirs(output_dir, exist_ok=True)
        
        # 检查输入文件
        if not os.path.exists(input_file):
            CalibrationLogger.error(f"Input file does not exist: {input_file}")
            return False, None
        
        # 执行提取命令
        extract_script = os.path.join(script_dir, "extract_imu_params.py")
        cmd = [
            sys.executable,
            extract_script,
            "-i", input_file,
            "-o", output_file
        ]
        
        CalibrationLogger.info(f"Running: {' '.join(cmd)}")
        CalibrationLogger.info(f"  Input: {input_file}")
        CalibrationLogger.info(f"  Output: {output_file}")
        
        result = CommandExecutor.run(cmd, shell=False, check=False, capture_output=True)
        
        if isinstance(result, tuple):
            returncode, stdout, stderr = result
            
            if returncode != 0:
                CalibrationLogger.error(f"Failed to extract parameters for imu{imu_num}")
                if stderr:
                    CalibrationLogger.error(f"Stderr: {stderr}")
                return False, None
        
        # 验证输出文件
        if os.path.exists(output_file):
            CalibrationLogger.success(f"Output file created: {output_file}")
            return True, output_file
        else:
            CalibrationLogger.error(f"Expected output file not found: {output_file}")
            return False, None
    
    def process_single_imu(
        self,
        imu_num: int,
        launch_file: str,
        bag_file: str
    ) -> Tuple[bool, Optional[str], Optional[str]]:
        """
        处理单个IMU的完整流程
        
        Args:
            imu_num: IMU编号
            launch_file: launch文件路径
            bag_file: rosbag文件路径
            
        Returns:
            (是否成功, YAML文件路径, 错误信息)
        """
        CalibrationLogger.section(f"Processing IMU {imu_num}")
        
        # 1. 播放rosbag
        rosbag_process = RosbagHelper.play_rosbag(bag_file, rate=60.0)
        
        # 2. 运行roslaunch
        roslaunch_process = self.run_roslaunch(imu_num, launch_file)
        
        if roslaunch_process is None:
            RosbagHelper.stop_rosbag(rosbag_process)
            return False, None, "Failed to start roslaunch process"
        
        # 3. 等待完成
        success = self.wait_for_completion(roslaunch_process, imu_num)
        
        # 4. 停止rosbag
        RosbagHelper.stop_rosbag(rosbag_process)
        
        if not success:
            error_msg = f"Roslaunch process (imu{imu_num}) failed or timed out"
            CalibrationLogger.error(f"{error_msg}!")
            CalibrationLogger.info("Cannot proceed with parameter extraction.")
            return False, None, error_msg
        
        # 5. 提取参数
        success, yaml_file = self.extract_parameters(imu_num)
        
        if success:
            CalibrationLogger.success(f"IMU {imu_num} processing completed successfully!")
            return True, yaml_file, None
        else:
            error_msg = f"Failed to extract parameters for imu{imu_num}"
            CalibrationLogger.error(error_msg)
            return False, None, error_msg

