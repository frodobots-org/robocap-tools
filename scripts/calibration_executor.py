#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration executor module
Responsible for executing individual calibration tasks
"""

import os
import sys
import subprocess
from typing import Optional, Dict, Any
from pathlib import Path

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from calibration_task import CalibrationTask, CalibrationTaskType
from calibration_result_handler import CalibrationResultHandler


class CalibrationExecutor:
    """Calibration executor - execute individual calibration tasks"""
    
    def __init__(self, scripts_dir: str, device_id: str):
        """
        Initialize calibration executor
        
        Args:
            scripts_dir: Script directory path
            device_id: Device ID
        """
        self.scripts_dir = scripts_dir
        self.device_id = device_id
        self.result_handler = None
    
    def set_result_handler(self, result_handler: CalibrationResultHandler):
        """Set result handler"""
        self.result_handler = result_handler
    
    def execute_task(
        self,
        task: CalibrationTask,
        callback=None
    ) -> Dict[str, Any]:
        """
        Execute calibration task
        
        Args:
            task: Calibration task configuration
            callback: Callback function (optional)
            
        Returns:
            Execution result dictionary containing:
            - success: bool - Whether successful
            - output_files: List[str] - List of generated output files
            - error_message: Optional[str] - Error message
        """
        script_path = os.path.join(self.scripts_dir, task.script_name)
        
        if not os.path.exists(script_path):
            return {
                'success': False,
                'output_files': [],
                'error_message': f"Script not found: {script_path}"
            }
        
        # Build command
        cmd = [sys.executable, script_path]
        cmd.extend(['--device-id', self.device_id])
        cmd.extend(task.script_args)
        
        # If callback exists, add callback parameter
        if callback:
            # Need to serialize callback function or pass through other means
            # For now, skip passing callback, handle through result_handler
            pass
        
        # If result_handler exists, add callback parameter
        if self.result_handler and callback:
            # Pass callback information through environment variables or other means
            pass
        
        try:
            # Execute script
            print(f"[Execute] {task.task_type.value} - Device: {self.device_id}")
            print(f"[Command] {' '.join(cmd)}")
            print(f"[Timeout] 3 hours")
            print("-" * 80)
            
            # Real-time log output to terminal
            import time
            import threading
            
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,  # Line buffering
                universal_newlines=True
            )
            
            # Read and output logs in real-time
            stdout_lines = []
            timeout_seconds = 10800 # 3 hour timeout
            timeout_occurred = threading.Event()
            
            def read_output():
                """Thread function to read process output"""
                try:
                    for line in process.stdout:
                        if timeout_occurred.is_set():
                            break
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
                time.sleep(0.1)  # Brief sleep
            
            # Wait for output thread to complete
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
            
            # Check if output files exist
            output_files = self._check_output_files(task)
            
            if not output_files:
                return {
                    'success': False,
                    'output_files': [],
                    'error_message': "Expected output files not found after execution"
                }
            
            print(f"[Success] {task.task_type.value} - Generated {len(output_files)} files")
            return {
                'success': True,
                'output_files': output_files,
                'error_message': None
            }
            
        except subprocess.TimeoutExpired:
            # Terminate process after timeout
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
        Check if output files exist, including YAML files and related PDF/TXT files
        
        Args:
            task: Calibration task configuration
            
        Returns:
            List of existing output file paths (including YAML, PDF, TXT)
        """
        # Dynamically import robocap_env, as device_id may have been set
        import robocap_env
        import glob
        
        # Determine output directory based on task type
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
        
        # Check expected output files (YAML files)
        for expected_file in task.expected_output_files:
            file_path = os.path.join(output_dir, expected_file)
            if os.path.exists(file_path):
                found_files.append(file_path)
        
        # Based on task type, find all related YAML files
        # For extrinsic calibration tasks, also find YAML files generated by intrinsic calibration (if they exist)
        if task.task_type == CalibrationTaskType.IMU_INTRINSIC:
            # IMU intrinsic: find all imu_*.yaml files
            imu_yaml_files = glob.glob(os.path.join(output_dir, "imu_*.yaml"))
            found_files.extend(imu_yaml_files)
        elif task.task_type in [
            CalibrationTaskType.CAM_LR_FRONT_INTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_INTRINSIC,
            CalibrationTaskType.CAM_L_INTRINSIC,
            CalibrationTaskType.CAM_R_INTRINSIC
        ]:
            # Intrinsic calibration: find all camchain.yaml files
            camchain_files = glob.glob(os.path.join(output_dir, "*intrinsic-camchain.yaml"))
            found_files.extend(camchain_files)
        elif task.task_type in [
            CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
            CalibrationTaskType.CAM_L_EXTRINSIC,
            CalibrationTaskType.CAM_R_EXTRINSIC
        ]:
            # Extrinsic calibration: find all related YAML files
            # 1. Extrinsic camchain files: *-camchain-imucam.yaml or imus-cam_*-camchain-imucam.yaml
            extrinsic_camchain_patterns = [
                "*-camchain-imucam.yaml",
                "imus-cam_*-camchain-imucam.yaml"
            ]
            for pattern in extrinsic_camchain_patterns:
                files = glob.glob(os.path.join(output_dir, pattern))
                found_files.extend(files)
            # 2. IMU YAML files: *-imu.yaml or imus-cam_*-imu.yaml
            imu_yaml_patterns = [
                "*-imu.yaml",
                "imus-cam_*-imu.yaml"
            ]
            for pattern in imu_yaml_patterns:
                files = glob.glob(os.path.join(output_dir, pattern))
                found_files.extend(files)
            # 3. Intrinsic camchain files (if they exist): *-intrinsic-camchain.yaml
            intrinsic_camchain_files = glob.glob(os.path.join(output_dir, "*intrinsic-camchain.yaml"))
            found_files.extend(intrinsic_camchain_files)
        
        # Find related PDF and TXT files (Kalibr-generated report files)
        # Support multiple filename patterns:
        # - report-*.pdf, results-*.txt (generic pattern)
        # - *-report-*.pdf, *-results-*.txt (prefixed pattern, e.g., cam_lr_front_intrinsic-report-cam.pdf)
        # - *report*.pdf, *results*.txt (files containing report/results)
        
        # PDF files
        pdf_patterns = [
            "report-*.pdf",
            "*-report-*.pdf",
            "*report*.pdf"
        ]
        for pattern in pdf_patterns:
            pdf_files = glob.glob(os.path.join(output_dir, pattern))
            found_files.extend(pdf_files)
        
        # TXT files
        txt_patterns = [
            "results-*.txt",
            "*-results-*.txt",
            "*results*.txt"
        ]
        for pattern in txt_patterns:
            txt_files = glob.glob(os.path.join(output_dir, pattern))
            found_files.extend(txt_files)
        
        # Remove duplicates and filter out .log and .bag files (these files should not be uploaded)
        found_files = list(set(found_files))
        found_files = [f for f in found_files if not f.endswith('.log') and not f.endswith('.bag')]
        
        return found_files

