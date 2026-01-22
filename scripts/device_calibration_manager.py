#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Device calibration manager module
Responsible for managing all calibration tasks for a single device
"""

import os
import sys
from typing import Dict, Optional
from pathlib import Path

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import robocap_env
from calibration_task import (
    CalibrationTaskType,
    get_all_task_types,
    get_task_by_type
)
from calibration_executor import CalibrationExecutor
from result_recorder import ResultRecorder
from s3_uploader import S3Uploader
from calibration_result_handler import CalibrationResultHandler
from results_report import ResultsReporter


class DeviceCalibrationManager:
    """Device calibration manager - manage all calibration tasks for a single device"""
    
    def __init__(
        self,
        device_id: str,
        scripts_dir: str,
        result_recorder: Optional[ResultRecorder] = None,
        s3_uploader: Optional[S3Uploader] = None,
        results_reporter: Optional[ResultsReporter] = None
    ):
        """
        Initialize device calibration manager
        
        Args:
            device_id: Device ID
            scripts_dir: Script directory path
            result_recorder: Result recorder (optional)
            s3_uploader: S3 uploader (optional)
            results_reporter: Results reporter for API reporting (optional)
        """
        self.device_id = device_id
        self.scripts_dir = scripts_dir
        self.result_recorder = result_recorder
        self.s3_uploader = s3_uploader
        self.results_reporter = results_reporter
        self.executor = CalibrationExecutor(scripts_dir, device_id)
        
        # Set device ID
        robocap_env.set_device_id(device_id)
    
    def calibrate_all(self) -> Dict[CalibrationTaskType, bool]:
        """
        Execute all calibration tasks (total timeout per device: 3 hours)
        
        Returns:
            Mapping from task type to success status
        """
        import time
        start_time = time.time()
        timeout_seconds = 10800  # 3 hours
        
        print(f"\n{'='*80}")
        print(f"Start calibrating device: {self.device_id}")
        print(f"[Timeout] Total timeout per device: 3 hours")
        print(f"{'='*80}\n")
        
        results = {}
        task_types = get_all_task_types()
        
        for task_type in task_types:
            # Check total timeout
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout_seconds:
                print(f"\n[Timeout] Device {self.device_id} calibration total time exceeded 3 hours, stopping execution")
                # Mark remaining tasks as failed
                remaining_tasks = [t for t in task_types if t not in results]
                for remaining_task in remaining_tasks:
                    results[remaining_task] = False
                break
            
            print(f"\n{'='*80}")
            print(f"Task: {task_type.value}")
            print(f"[Elapsed time] {elapsed_time:.1f}s / {timeout_seconds}s")
            print(f"{'='*80}")
            
            task = get_task_by_type(task_type)
            success = self._execute_single_task(task_type, task)
            results[task_type] = success
            
            if success:
                print(f"✓ {task_type.value} calibration successful")
            else:
                print(f"✗ {task_type.value} calibration failed")
        
        # Collect reprojection error values for all extrinsic calibration tasks
        reprojection_errors = {}
        for task_type in [
            CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
            CalibrationTaskType.CAM_L_EXTRINSIC,
            CalibrationTaskType.CAM_R_EXTRINSIC
        ]:
            if results.get(task_type, False):  # Only parse if successful
                task = get_task_by_type(task_type)
                # Get output directory
                import robocap_env
                output_dir = None
                if task_type == CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC:
                    output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
                elif task_type == CalibrationTaskType.CAM_LR_EYE_EXTRINSIC:
                    output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
                elif task_type == CalibrationTaskType.CAM_L_EXTRINSIC:
                    output_dir = robocap_env.OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
                elif task_type == CalibrationTaskType.CAM_R_EXTRINSIC:
                    output_dir = robocap_env.OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR

                if output_dir:
                    from extrinsic_result_parser import get_extrinsic_reprojection_errors
                    errors = get_extrinsic_reprojection_errors(output_dir, task_type.value)
                    if errors:
                        reprojection_errors[task_type] = errors

        # Record results to CSV
        if self.result_recorder:
            self.result_recorder.record_device_result(
                self.device_id,
                results,
                reprojection_errors if reprojection_errors else None
            )
        
        # Print summary
        total_time = time.time() - start_time
        print(f"\n[Total time] {total_time:.1f}s ({total_time/60:.1f} minutes)")
        self._print_summary(results)
        
        return results
    
    def _execute_single_task(
        self,
        task_type: CalibrationTaskType,
        task
    ) -> bool:
        """
        Execute single calibration task
        
        Args:
            task_type: Task type
            task: Task configuration
            
        Returns:
            Whether successful
        """
        # Check if data directory exists
        data_dir = os.path.join("/data", self.device_id, "v1", task.data_dir)
        if not os.path.exists(data_dir):
            print(f"Data directory does not exist: {data_dir}")
            return False
        
        # Execute calibration task
        execution_result = self.executor.execute_task(task)
        
        # Clean up intermediate files (.log and .bag files) to save space, regardless of success or failure
        self._cleanup_intermediate_files(task_type, task)
        
        # Determine calibration success status
        calibration_success = execution_result['success']
        error_message = execution_result.get('error_message', None)
        
        if not calibration_success:
            print(f"Execution failed: {error_message or 'Unknown error'}")
        
        # Upload files to S3 (including YAML, PDF, TXT files) - only if successful
        if calibration_success and self.s3_uploader and execution_result['output_files']:
            print(f"\n[Upload] Starting upload of {len(execution_result['output_files'])} files to S3...")
            for file_path in execution_result['output_files']:
                filename = os.path.basename(file_path)
                self.s3_uploader.upload_calibration_file(
                    file_path,
                    self.device_id,
                    task_type.value,
                    filename
                )
        
        # Report results to API database (always report, even if failed)
        if self.results_reporter:
            self._report_calibration_results(task_type, calibration_success, error_message)
        
        return calibration_success
    
    def _report_calibration_results(
        self, 
        task_type: CalibrationTaskType, 
        calibration_success: bool = True,
        error_message: Optional[str] = None
    ) -> None:
        """
        Report calibration results to API database
        
        Args:
            task_type: Calibration task type
            calibration_success: Whether calibration was successful
            error_message: Error message if calibration failed
        """
        # Get output directory based on task type
        output_dir = None
        if task_type == CalibrationTaskType.CAM_LR_FRONT_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_LR_EYE_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_LR_EYE_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_L_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_L_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_R_INTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR
        elif task_type == CalibrationTaskType.CAM_R_EXTRINSIC:
            output_dir = robocap_env.OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR
        
        # Determine errcode and errmsg
        errcode = 0 if calibration_success else 1
        errmsg = error_message if not calibration_success else None
        
        # Report based on task type
        if task_type in [
            CalibrationTaskType.CAM_L_INTRINSIC,
            CalibrationTaskType.CAM_R_INTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_INTRINSIC,
            CalibrationTaskType.CAM_LR_FRONT_INTRINSIC
        ]:
            # Report intrinsic calibration
            # output_dir may be None if calibration failed, but we still try to report
            if output_dir and os.path.exists(output_dir):
                self.results_reporter.report_intrinsic_calibration(
                    self.device_id,
                    task_type,
                    output_dir,
                    errcode=errcode,
                    errmsg=errmsg
                )
            else:
                # No output directory (calibration failed before creating output)
                # Still report failure to API
                self.results_reporter.report_intrinsic_calibration(
                    self.device_id,
                    task_type,
                    output_dir or "",  # Pass empty string if None
                    errcode=errcode,
                    errmsg=errmsg or "Calibration execution failed"
                )
        elif task_type in [
            CalibrationTaskType.CAM_L_EXTRINSIC,
            CalibrationTaskType.CAM_R_EXTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
            CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC
        ]:
            # Report extrinsic calibration
            # output_dir may be None if calibration failed, but we still try to report
            if output_dir and os.path.exists(output_dir):
                self.results_reporter.report_extrinsic_calibration(
                    self.device_id,
                    task_type,
                    output_dir,
                    errcode=errcode,
                    errmsg=errmsg
                )
            else:
                # No output directory (calibration failed before creating output)
                # Still report failure to API
                self.results_reporter.report_extrinsic_calibration(
                    self.device_id,
                    task_type,
                    output_dir or "",  # Pass empty string if None
                    errcode=errcode,
                    errmsg=errmsg or "Calibration execution failed"
                )
        # Note: IMU_INTRINSIC is not reported as there's no API endpoint for it
    
    def _cleanup_intermediate_files(
        self,
        task_type: CalibrationTaskType,
        task
    ) -> None:
        """
        Clean up intermediate files (.log and .bag files)
        
        Args:
            task_type: Task type
            task: Task configuration
        """
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
            return
        
        deleted_count = 0
        
        # Delete .log files
        log_files = glob.glob(os.path.join(output_dir, "*.log"))
        for log_file in log_files:
            try:
                os.remove(log_file)
                deleted_count += 1
                print(f"[Cleanup] Deleted log file: {os.path.basename(log_file)}")
            except Exception as e:
                print(f"[Warning] Failed to delete log file {log_file}: {e}")
        
        # Delete .bag files
        bag_files = glob.glob(os.path.join(output_dir, "*.bag"))
        for bag_file in bag_files:
            try:
                os.remove(bag_file)
                deleted_count += 1
                print(f"[Cleanup] Deleted rosbag file: {os.path.basename(bag_file)}")
            except Exception as e:
                print(f"[Warning] Failed to delete rosbag file {bag_file}: {e}")
        
        if deleted_count > 0:
            print(f"[Cleanup] Deleted {deleted_count} intermediate files in total")
    
    def _print_summary(self, results: Dict[CalibrationTaskType, bool]):
        """Print calibration result summary"""
        print(f"\n{'='*80}")
        print(f"Device {self.device_id} calibration summary")
        print(f"{'='*80}")
        
        total = len(results)
        success_count = sum(1 for v in results.values() if v)
        failed_count = total - success_count
        
        print(f"Total tasks: {total}")
        print(f"Success: {success_count}")
        print(f"Failed: {failed_count}")
        
        if failed_count > 0:
            print("\nFailed tasks:")
            for task_type, success in results.items():
                if not success:
                    print(f"  - {task_type.value}")
        
        print(f"{'='*80}\n")

