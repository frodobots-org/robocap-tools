#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kalibr执行器
统一封装kalibr标定命令执行逻辑
"""

import os
import sys
import time
from typing import List, Optional

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import robocap_env
except ImportError:
    robocap_env = None

from calibration_common import CommandExecutor, CalibrationLogger


class KalibrExecutor:
    """Kalibr执行器 - 统一封装kalibr标定命令执行"""
    
    def __init__(self):
        """初始化Kalibr执行器"""
        self.target_file = robocap_env.TARGET_FILE if robocap_env else None
        self.setup_file = robocap_env.KALIBR_SETUP_FILE if robocap_env else None
    
    def run_intrinsic_calibration(
        self,
        rosbag_path: str,
        camera_topics: List[str],
        timeout: int = 600
    ) -> bool:
        """
        运行kalibr相机内参标定
        
        Args:
            rosbag_path: rosbag文件路径
            camera_topics: 相机topic列表
            timeout: 超时时间（秒，默认600）
            
        Returns:
            是否成功
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
        
        # 验证文件存在
        if not self._verify_files(rosbag_path, check_camchain=False):
            return False
        
        # 构建kalibr命令
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
        timeout: int = 1800
    ) -> bool:
        """
        运行kalibr相机-IMU外参标定
        
        Args:
            rosbag_path: rosbag文件路径
            camchain_file: camchain文件路径
            imu_yaml_files: IMU YAML文件列表
            imu_models: IMU模型列表（可选，默认使用'calibrated'）
            timeout: 超时时间（秒，默认1800）
            
        Returns:
            是否成功
        """
        CalibrationLogger.step(2, "Running kalibr IMU-camera extrinsic calibration...")
        CalibrationLogger.info(f"Rosbag: {rosbag_path}")
        CalibrationLogger.info(f"Camchain file: {camchain_file}")
        CalibrationLogger.info(f"IMU YAML files: {imu_yaml_files}")
        
        # 确定IMU模型
        if imu_models:
            final_imu_models = imu_models
        else:
            final_imu_models = ['calibrated'] * len(imu_yaml_files)
        
        CalibrationLogger.info(f"IMU models: {final_imu_models}")
        CalibrationLogger.info(f"Target: {self.target_file}")
        CalibrationLogger.info(f"Bag time range: 5-65 seconds")
        CalibrationLogger.info(f"Timeout: {timeout} seconds")
        
        # 验证文件存在
        if not self._verify_files(rosbag_path, camchain_file=camchain_file):
            return False
        
        # 验证IMU YAML文件
        if not self._verify_imu_files(imu_yaml_files):
            return False
        
        # 构建kalibr命令
        kalibr_cmd = [
            'rosrun kalibr kalibr_calibrate_imu_camera',
            '--perform-synchronization',
            '--bag', rosbag_path,
            '--bag-from-to', '5 65',
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
        """验证必需文件是否存在"""
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
        """验证IMU YAML文件是否存在"""
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
        """执行kalibr命令（实时输出日志）"""
        import subprocess
        import threading
        import sys
        
        env_cmd = (
            f"source {self.setup_file} && "
            f"{' '.join(kalibr_cmd)}"
        )
        
        CalibrationLogger.info(f"\nRunning command: {env_cmd}")
        CalibrationLogger.info("-" * 80)
        
        # 执行命令（实时输出）
        cmd = ['/bin/bash', '-c', env_cmd]
        start_time = time.time()
        
        # 使用Popen实时输出
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # 实时读取并输出日志
        stdout_lines = []
        
        def read_output():
            """读取进程输出的线程函数"""
            try:
                for line in process.stdout:
                    line = line.rstrip()
                    print(line)  # 实时输出到终端
                    stdout_lines.append(line)
                    sys.stdout.flush()  # 确保立即输出
            except Exception:
                pass
        
        # 启动读取输出的线程
        output_thread = threading.Thread(target=read_output, daemon=True)
        output_thread.start()
        
        # 等待进程完成或超时
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
        
        # 等待输出线程完成
        output_thread.join(timeout=1)
        
        elapsed_time = time.time() - start_time
        stdout = '\n'.join(stdout_lines)
        
        CalibrationLogger.info(f"\nKalibr calibration completed in {elapsed_time:.1f} seconds")
        
        if returncode != 0:
            CalibrationLogger.error(f"Kalibr calibration failed (return code: {returncode})")
            return False
        
        CalibrationLogger.success("Kalibr calibration completed successfully")
        return True

