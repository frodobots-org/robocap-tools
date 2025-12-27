#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定公共工具模块
提供所有标定脚本共用的工具函数和类
"""

import os
import sys
import subprocess
import argparse
import importlib.util
from typing import List, Optional, Tuple, Union, Callable
from dataclasses import dataclass

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import robocap_env
except ImportError:
    robocap_env = None

try:
    from calibration_result_handler import (
        CalibrationResultHandler,
        CalibrationStatus
    )
except ImportError:
    CalibrationResultHandler = None
    CalibrationStatus = None


@dataclass
class CalibrationConfig:
    """标定配置数据类"""
    input_dir: str
    output_rosbag: str
    camchain_file: Optional[str] = None
    imu_yaml_files: Optional[List[str]] = None


class CommandExecutor:
    """命令执行器 - 统一的命令执行接口"""
    
    @staticmethod
    def run(
        cmd: Union[str, List[str]],
        shell: bool = True,
        check: bool = True,
        background: bool = False,
        timeout: Optional[int] = None,
        cwd: Optional[str] = None,
        capture_output: bool = True
    ) -> Union[subprocess.Popen, Tuple[int, str, str], subprocess.CompletedProcess]:
        """
        执行命令
        
        Args:
            cmd: 命令字符串或列表
            shell: 是否使用shell
            check: 是否检查返回码
            background: 是否后台运行
            timeout: 超时时间（秒）
            cwd: 工作目录
            capture_output: 是否捕获输出
            
        Returns:
            - background=True: subprocess.Popen
            - background=False, capture_output=True: (returncode, stdout, stderr)
            - background=False, capture_output=False: CompletedProcess
        """
        if background:
            return subprocess.Popen(
                cmd,
                shell=shell,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        elif capture_output:
            try:
                result = subprocess.run(
                    cmd,
                    shell=shell,
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                    cwd=cwd,
                    check=check
                )
                return result.returncode, result.stdout, result.stderr
            except subprocess.TimeoutExpired:
                return -1, "", f"Command timed out after {timeout} seconds"
            except subprocess.CalledProcessError as e:
                return e.returncode, e.stdout or "", e.stderr or ""
            except Exception as e:
                return -1, "", str(e)
        else:
            return subprocess.run(
                cmd,
                shell=shell,
                check=check,
                timeout=timeout,
                cwd=cwd
            )


class CalibrationLogger:
    """标定日志记录器 - 统一的日志输出格式"""
    
    @staticmethod
    def step(step_num: int, description: str):
        """输出步骤日志"""
        print("=" * 80)
        print(f"Step {step_num}: {description}")
        print("=" * 80)
    
    @staticmethod
    def info(message: str):
        """输出信息日志"""
        print(message)
    
    @staticmethod
    def success(message: str):
        """输出成功日志"""
        print(f"✓ {message}")
    
    @staticmethod
    def error(message: str):
        """输出错误日志"""
        print(f"✗ {message}")
    
    @staticmethod
    def warning(message: str):
        """输出警告日志"""
        print(f"⚠ {message}")
    
    @staticmethod
    def section(title: str):
        """输出章节标题"""
        print("\n" + "=" * 80)
        print(title)
        print("=" * 80)


class ConfigHelper:
    """配置助手 - 统一的配置获取"""
    
    @staticmethod
    def get_camera_intrinsic_config(mode: str) -> Optional[CalibrationConfig]:
        """
        获取相机内参标定配置
        
        Args:
            mode: 'front', 'eye', 'left', 'right'
            
        Returns:
            CalibrationConfig 或 None
        """
        if robocap_env is None:
            return None
        
        if mode == 'front':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC
            )
        elif mode == 'eye':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_LR_EYE_INTRINSIC
            )
        elif mode == 'left':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_L_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_L_INTRINSIC
            )
        elif mode == 'right':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_R_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_R_INTRINSIC
            )
        return None
    
    @staticmethod
    def get_camera_extrinsic_config(mode: str) -> Optional[CalibrationConfig]:
        """
        获取相机外参标定配置
        
        Args:
            mode: 'front', 'eye', 'left', 'right'
            
        Returns:
            CalibrationConfig 或 None
        """
        if robocap_env is None:
            return None
        
        if mode == 'front':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC,
                camchain_file=robocap_env.CAMCHAIN_FILE_LR_FRONT,
                imu_yaml_files=[
                    robocap_env.YAML_FILE_IMU_MID_0,
                    robocap_env.YAML_FILE_IMU_RIGHT_1,
                    robocap_env.YAML_FILE_IMU_LEFT_2
                ]
            )
        elif mode == 'eye':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC,
                camchain_file=robocap_env.CAMCHAIN_FILE_LR_EYE,
                imu_yaml_files=[
                    robocap_env.YAML_FILE_IMU_MID_0,
                    robocap_env.YAML_FILE_IMU_RIGHT_1,
                    robocap_env.YAML_FILE_IMU_LEFT_2
                ]
            )
        elif mode == 'left':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_L_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC,
                camchain_file=robocap_env.CAMCHAIN_FILE_L,
                imu_yaml_files=[
                    robocap_env.YAML_FILE_IMU_MID_0,
                    robocap_env.YAML_FILE_IMU_RIGHT_1,
                    robocap_env.YAML_FILE_IMU_LEFT_2
                ]
            )
        elif mode == 'right':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_IMUS_CAM_R_EXTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC,
                camchain_file=robocap_env.CAMCHAIN_FILE_R,
                imu_yaml_files=[
                    robocap_env.YAML_FILE_IMU_MID_0,
                    robocap_env.YAML_FILE_IMU_RIGHT_1,
                    robocap_env.YAML_FILE_IMU_LEFT_2
                ]
            )
        return None
    
    @staticmethod
    def get_imu_intrinsic_config() -> Optional[CalibrationConfig]:
        """
        获取IMU内参标定配置
        
        Returns:
            CalibrationConfig 或 None
        """
        if robocap_env is None:
            return None
        
        return CalibrationConfig(
            input_dir=robocap_env.DATASET_IMUS_INTRINSIC_DIR,
            output_rosbag=robocap_env.ROSBAG_FILE_IMUS_INTRINSIC
        )


class ArgumentParserHelper:
    """参数解析助手 - 统一的参数解析"""
    
    @staticmethod
    def add_common_args(parser: argparse.ArgumentParser):
        """添加通用参数"""
        parser.add_argument(
            '--device-id',
            type=str,
            default=None,
            help='Robocap device ID (e.g., faf2a598869ccfc8). If not specified, uses ROBOCAP_DEVICE_ID environment variable or default value.'
        )
        parser.add_argument(
            '--callback',
            type=str,
            default=None,
            help='Path to Python module containing callback function for calibration results (optional)'
        )
    
    @staticmethod
    def setup_device_id(args: argparse.Namespace) -> str:
        """
        设置设备ID
        
        Args:
            args: 解析后的参数对象
            
        Returns:
            设备ID字符串
        """
        if args.device_id:
            if robocap_env:
                robocap_env.set_device_id(args.device_id)
            print(f"Device ID set to: {args.device_id}")
            return args.device_id
        else:
            device_id = os.environ.get("ROBOCAP_DEVICE_ID", "")
            if robocap_env and not device_id:
                device_id = robocap_env.ROBOCAP_DEVICE_ID
            print(f"Using device ID: {device_id} (from environment variable or default)")
            return device_id
    
    @staticmethod
    def load_callback(callback_path: Optional[str]) -> Optional[Callable]:
        """
        加载回调函数
        
        Args:
            callback_path: 回调模块路径
            
        Returns:
            回调函数或None
        """
        if not callback_path:
            return None
        
        try:
            spec = importlib.util.spec_from_file_location("callback_module", callback_path)
            callback_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(callback_module)
            if hasattr(callback_module, 'calibration_callback'):
                callback_func = callback_module.calibration_callback
                print(f"Loaded callback function from: {callback_path}")
                return callback_func
            else:
                print(f"Warning: No 'calibration_callback' function found in {callback_path}")
                return None
        except Exception as e:
            print(f"Warning: Failed to load callback function: {e}")
            return None
    
    @staticmethod
    def create_result_handler(callback_func: Optional[Callable] = None) -> Optional[CalibrationResultHandler]:
        """
        创建结果处理器
        
        Args:
            callback_func: 回调函数
            
        Returns:
            CalibrationResultHandler 或 None
        """
        if CalibrationResultHandler is None:
            return None
        return CalibrationResultHandler(callback=callback_func)

