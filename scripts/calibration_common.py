#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration common utilities module
Provides common utility functions and classes for all calibration scripts
"""

import os
import sys
import subprocess
import argparse
import importlib.util
from typing import List, Optional, Tuple, Union, Callable
from dataclasses import dataclass

# Add script directory to path
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
    """Calibration configuration data class"""
    input_dir: str
    output_rosbag: str
    camchain_file: Optional[str] = None
    imu_yaml_files: Optional[List[str]] = None


class CommandExecutor:
    """Command executor - unified command execution interface"""
    
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
        Execute command
        
        Args:
            cmd: Command string or list
            shell: Whether to use shell
            check: Whether to check return code
            background: Whether to run in background
            timeout: Timeout in seconds
            cwd: Working directory
            capture_output: Whether to capture output
            
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
    """Calibration logger - unified log output format"""
    
    @staticmethod
    def step(step_num: int, description: str):
        """Output step log"""
        print("=" * 80)
        print(f"Step {step_num}: {description}")
        print("=" * 80)
    
    @staticmethod
    def info(message: str):
        """Output info log"""
        print(message)
    
    @staticmethod
    def success(message: str):
        """Output success log"""
        print(f"✓ {message}")
    
    @staticmethod
    def error(message: str):
        """Output error log"""
        print(f"✗ {message}")
    
    @staticmethod
    def warning(message: str):
        """Output warning log"""
        print(f"⚠ {message}")
    
    @staticmethod
    def section(title: str):
        """Output section title"""
        print("\n" + "=" * 80)
        print(title)
        print("=" * 80)


class ConfigHelper:
    """Config helper - unified configuration retrieval"""
    
    @staticmethod
    def get_camera_intrinsic_config(mode: str) -> Optional[CalibrationConfig]:
        """
        Get camera intrinsic calibration configuration
        
        Args:
            mode: 'front', 'eye', 'left', 'right'
            
        Returns:
            CalibrationConfig or None
        """
        if robocap_env is None:
            return None
        
        if mode == 'front':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_CAM_LR_FRONT_INTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC
            )
        elif mode == 'eye':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_CAM_LR_EYE_INTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_LR_EYE_INTRINSIC
            )
        elif mode == 'left':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_CAM_L_EYE_INTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_L_INTRINSIC
            )
        elif mode == 'right':
            return CalibrationConfig(
                input_dir=robocap_env.DATASET_CAM_R_EYE_INTRINSIC_DIR,
                output_rosbag=robocap_env.ROSBAG_FILE_CAM_R_INTRINSIC
            )
        return None
    
    @staticmethod
    def get_camera_extrinsic_config(mode: str) -> Optional[CalibrationConfig]:
        """
        Get camera extrinsic calibration configuration
        
        Args:
            mode: 'front', 'eye', 'left', 'right'
            
        Returns:
            CalibrationConfig or None
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
        Get IMU intrinsic calibration configuration
        
        Returns:
            CalibrationConfig or None
        """
        if robocap_env is None:
            return None
        
        return CalibrationConfig(
            input_dir=robocap_env.DATASET_IMUS_INTRINSIC_DIR,
            output_rosbag=robocap_env.ROSBAG_FILE_IMUS_INTRINSIC
        )


class ArgumentParserHelper:
    """Argument parser helper - unified argument parsing"""
    
    @staticmethod
    def add_common_args(parser: argparse.ArgumentParser):
        """Add common arguments"""
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
        parser.add_argument(
            '--api-url',
            type=str,
            default=None,
            help='API base URL for result reporting (e.g. http://host:6002). If set, calibration result is reported to the server. Can also use CALIBRATION_API_URL env var.'
        )
        parser.add_argument(
            '--disable-api-report',
            action='store_true',
            help='Disable reporting calibration result to API even when --api-url or CALIBRATION_API_URL is set'
        )
    
    @staticmethod
    def setup_device_id(args: argparse.Namespace) -> str:
        """
        Setup device ID
        
        Args:
            args: Parsed arguments object
            
        Returns:
            Device ID string
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
        Load callback function
        
        Args:
            callback_path: Callback module path
            
        Returns:
            Callback function or None
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
        Create result handler
        
        Args:
            callback_func: Callback function
            
        Returns:
            CalibrationResultHandler or None
        """
        if CalibrationResultHandler is None:
            return None
        return CalibrationResultHandler(callback=callback_func)

