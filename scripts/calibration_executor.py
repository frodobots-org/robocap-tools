#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定执行器模块
负责执行单个标定任务
"""

import os
import sys
import subprocess
from typing import Optional, Dict, Any
from pathlib import Path

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from calibration_task import CalibrationTask, CalibrationTaskType
from calibration_result_handler import CalibrationResultHandler


class CalibrationExecutor:
    """标定执行器 - 执行单个标定任务"""
    
    def __init__(self, scripts_dir: str, device_id: str):
        """
        初始化标定执行器
        
        Args:
            scripts_dir: 脚本目录路径
            device_id: 设备ID
        """
        self.scripts_dir = scripts_dir
        self.device_id = device_id
        self.result_handler = None
    
    def set_result_handler(self, result_handler: CalibrationResultHandler):
        """设置结果处理器"""
        self.result_handler = result_handler
    
    def execute_task(
        self,
        task: CalibrationTask,
        callback=None
    ) -> Dict[str, Any]:
        """
        执行标定任务
        
        Args:
            task: 标定任务配置
            callback: 回调函数（可选）
            
        Returns:
            执行结果字典，包含：
            - success: bool - 是否成功
            - output_files: List[str] - 生成的输出文件列表
            - error_message: Optional[str] - 错误信息
        """
        script_path = os.path.join(self.scripts_dir, task.script_name)
        
        if not os.path.exists(script_path):
            return {
                'success': False,
                'output_files': [],
                'error_message': f"Script not found: {script_path}"
            }
        
        # 构建命令
        cmd = [sys.executable, script_path]
        cmd.extend(['--device-id', self.device_id])
        cmd.extend(task.script_args)
        
        # 如果有回调，添加回调参数
        if callback:
            # 这里需要将回调函数序列化或通过其他方式传递
            # 暂时先不传递回调，通过result_handler处理
            pass
        
        # 如果有result_handler，添加回调参数
        if self.result_handler and callback:
            # 通过环境变量或其他方式传递回调信息
            pass
        
        try:
            # 执行脚本
            print(f"[执行] {task.task_type.value} - 设备: {self.device_id}")
            print(f"[命令] {' '.join(cmd)}")
            print(f"[超时] 3小时")
            print("-" * 80)
            
            # 实时输出日志到终端
            import time
            import threading
            
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,  # 行缓冲
                universal_newlines=True
            )
            
            # 实时读取并输出日志
            stdout_lines = []
            timeout_seconds = 10800 # 3小时超时
            timeout_occurred = threading.Event()
            
            def read_output():
                """读取进程输出的线程函数"""
                try:
                    for line in process.stdout:
                        if timeout_occurred.is_set():
                            break
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
            start_time = time.time()
            while process.poll() is None:
                elapsed = time.time() - start_time
                if elapsed > timeout_seconds:
                    timeout_occurred.set()
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    raise subprocess.TimeoutExpired(cmd, timeout_seconds)
                time.sleep(0.1)  # 短暂休眠
            
            # 等待输出线程完成
            output_thread.join(timeout=1)
            
            returncode = process.returncode
            
            stdout = '\n'.join(stdout_lines)
            
            if returncode != 0:
                error_msg = f"Script failed with return code {returncode}\n"
                error_msg += f"STDOUT: {stdout}"
                return {
                    'success': False,
                    'output_files': [],
                    'error_message': error_msg
                }
            
            # 检查输出文件是否存在
            output_files = self._check_output_files(task)
            
            if not output_files:
                return {
                    'success': False,
                    'output_files': [],
                    'error_message': "Expected output files not found after execution"
                }
            
            print(f"[成功] {task.task_type.value} - 生成了 {len(output_files)} 个文件")
            return {
                'success': True,
                'output_files': output_files,
                'error_message': None
            }
            
        except subprocess.TimeoutExpired:
            # 超时后终止进程
            if 'process' in locals():
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
            return {
                'success': False,
                'output_files': [],
                'error_message': "Script execution timed out after 3 hours"
            }
        except Exception as e:
            return {
                'success': False,
                'output_files': [],
                'error_message': f"Exception during execution: {str(e)}"
            }
    
    def _check_output_files(self, task: CalibrationTask) -> list:
        """
        检查输出文件是否存在，包括YAML文件和相关的PDF/TXT文件
        
        Args:
            task: 标定任务配置
            
        Returns:
            存在的输出文件路径列表（包括YAML、PDF、TXT）
        """
        # 动态导入robocap_env，因为device_id可能已经设置
        import robocap_env
        import glob
        
        # 根据任务类型确定输出目录
        output_dir = None
        if task.task_type == CalibrationTaskType.IMU_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_INTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_LR_FRONT_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_LR_EYE_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_LR_EYE_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_L_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_L_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_R_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR
        elif task.task_type == CalibrationTaskType.CAM_R_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR
        
        if not output_dir or not os.path.exists(output_dir):
            return []
        
        found_files = []
        
        # 检查期望的输出文件（YAML文件）
        for expected_file in task.expected_output_files:
            file_path = os.path.join(output_dir, expected_file)
            if os.path.exists(file_path):
                found_files.append(file_path)
        
        # 根据任务类型，查找所有相关的YAML文件
        # 对于外参标定任务，还需要查找内参标定生成的YAML文件（如果存在）
        if task.task_type == CalibrationTaskType.IMU_INTRINSIC:
            # IMU内参：查找所有imu_*.yaml文件
            imu_yaml_files = glob.glob(os.path.join(output_dir, "imu_*.yaml"))
            found_files.extend(imu_yaml_files)
        elif task.task_type in [
            CalibrationTaskType.CAM_LR_FRONT_INTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_INTRINSIC,
            CalibrationTaskType.CAM_L_INTRINSIC,
            CalibrationTaskType.CAM_R_INTRINSIC
        ]:
            # 内参标定：查找所有camchain.yaml文件
            camchain_files = glob.glob(os.path.join(output_dir, "*intrinsic-camchain.yaml"))
            found_files.extend(camchain_files)
        elif task.task_type in [
            CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
            CalibrationTaskType.CAM_L_EXTRINSIC,
            CalibrationTaskType.CAM_R_EXTRINSIC
        ]:
            # 外参标定：查找所有相关YAML文件
            # 1. 外参camchain文件：*-camchain-imucam.yaml 或 imus-cam_*-camchain-imucam.yaml
            extrinsic_camchain_patterns = [
                "*-camchain-imucam.yaml",
                "imus-cam_*-camchain-imucam.yaml"
            ]
            for pattern in extrinsic_camchain_patterns:
                files = glob.glob(os.path.join(output_dir, pattern))
                found_files.extend(files)
            # 2. IMU YAML文件：*-imu.yaml 或 imus-cam_*-imu.yaml
            imu_yaml_patterns = [
                "*-imu.yaml",
                "imus-cam_*-imu.yaml"
            ]
            for pattern in imu_yaml_patterns:
                files = glob.glob(os.path.join(output_dir, pattern))
                found_files.extend(files)
            # 3. 内参camchain文件（如果存在）：*-intrinsic-camchain.yaml
            intrinsic_camchain_files = glob.glob(os.path.join(output_dir, "*intrinsic-camchain.yaml"))
            found_files.extend(intrinsic_camchain_files)
        
        # 查找相关的PDF和TXT文件（Kalibr生成的报告文件）
        # 支持多种文件名模式：
        # - report-*.pdf, results-*.txt (通用模式)
        # - *-report-*.pdf, *-results-*.txt (带前缀的模式，如cam_lr_front_intrinsic-report-cam.pdf)
        # - *report*.pdf, *results*.txt (包含report/results的文件)
        
        # PDF文件
        pdf_patterns = [
            "report-*.pdf",
            "*-report-*.pdf",
            "*report*.pdf"
        ]
        for pattern in pdf_patterns:
            pdf_files = glob.glob(os.path.join(output_dir, pattern))
            found_files.extend(pdf_files)
        
        # TXT文件
        txt_patterns = [
            "results-*.txt",
            "*-results-*.txt",
            "*results*.txt"
        ]
        for pattern in txt_patterns:
            txt_files = glob.glob(os.path.join(output_dir, pattern))
            found_files.extend(txt_files)
        
        # 去重并过滤掉.log和.bag文件（这些文件不应该上传）
        found_files = list(set(found_files))
        found_files = [f for f in found_files if not f.endswith('.log') and not f.endswith('.bag')]
        
        return found_files

