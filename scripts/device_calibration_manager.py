#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备标定管理器模块
负责管理单个设备的所有标定任务
"""

import os
import sys
from typing import Dict, Optional
from pathlib import Path

# 添加脚本目录到路径
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


class DeviceCalibrationManager:
    """设备标定管理器 - 管理单个设备的所有标定任务"""
    
    def __init__(
        self,
        device_id: str,
        scripts_dir: str,
        result_recorder: Optional[ResultRecorder] = None,
        s3_uploader: Optional[S3Uploader] = None
    ):
        """
        初始化设备标定管理器
        
        Args:
            device_id: 设备ID
            scripts_dir: 脚本目录路径
            result_recorder: 结果记录器（可选）
            s3_uploader: S3上传器（可选）
        """
        self.device_id = device_id
        self.scripts_dir = scripts_dir
        self.result_recorder = result_recorder
        self.s3_uploader = s3_uploader
        self.executor = CalibrationExecutor(scripts_dir, device_id)
        
        # 设置设备ID
        robocap_env.set_device_id(device_id)
    
    def calibrate_all(self) -> Dict[CalibrationTaskType, bool]:
        """
        执行所有标定任务（单个设备总超时时间：2小时）
        
        Returns:
            任务类型到成功状态的映射
        """
        import time
        start_time = time.time()
        timeout_seconds = 7200  # 2小时
        
        print(f"\n{'='*80}")
        print(f"开始标定设备: {self.device_id}")
        print(f"[超时设置] 单个设备总超时时间: 2小时")
        print(f"{'='*80}\n")
        
        results = {}
        task_types = get_all_task_types()
        
        for task_type in task_types:
            # 检查总超时时间
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout_seconds:
                print(f"\n[超时] 设备 {self.device_id} 标定总时间超过2小时，停止执行")
                # 将剩余任务标记为失败
                remaining_tasks = [t for t in task_types if t not in results]
                for remaining_task in remaining_tasks:
                    results[remaining_task] = False
                break
            
            print(f"\n{'='*80}")
            print(f"任务: {task_type.value}")
            print(f"[已用时间] {elapsed_time:.1f}秒 / {timeout_seconds}秒")
            print(f"{'='*80}")
            
            task = get_task_by_type(task_type)
            success = self._execute_single_task(task_type, task)
            results[task_type] = success
            
            if success:
                print(f"✓ {task_type.value} 标定成功")
            else:
                print(f"✗ {task_type.value} 标定失败")
        
        # 收集所有外参标定任务的reprojection error值
        reprojection_errors = {}
        for task_type in [
            CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
            CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
            CalibrationTaskType.CAM_L_EXTRINSIC,
            CalibrationTaskType.CAM_R_EXTRINSIC
        ]:
            if results.get(task_type, False):  # 只有成功时才解析
                task = get_task_by_type(task_type)
                # 获取输出目录
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
        
        # 记录结果到CSV
        if self.result_recorder:
            self.result_recorder.record_device_result(
                self.device_id,
                results,
                reprojection_errors if reprojection_errors else None
            )
        
        # 打印总结
        total_time = time.time() - start_time
        print(f"\n[总耗时] {total_time:.1f}秒 ({total_time/60:.1f}分钟)")
        self._print_summary(results)
        
        return results
    
    def _execute_single_task(
        self,
        task_type: CalibrationTaskType,
        task
    ) -> bool:
        """
        执行单个标定任务
        
        Args:
            task_type: 任务类型
            task: 任务配置
            
        Returns:
            是否成功
        """
        # 检查数据目录是否存在
        data_dir = os.path.join("/data", self.device_id, "v1", task.data_dir)
        if not os.path.exists(data_dir):
            print(f"数据目录不存在: {data_dir}")
            return False
        
        # 执行标定任务
        execution_result = self.executor.execute_task(task)
        
        # 无论成功或失败，都清理中间文件（.log和.bag文件）以节省空间
        self._cleanup_intermediate_files(task_type, task)
        
        if not execution_result['success']:
            print(f"执行失败: {execution_result.get('error_message', 'Unknown error')}")
            return False
        
        # 上传文件到S3（包括YAML、PDF、TXT文件）
        if self.s3_uploader and execution_result['output_files']:
            print(f"\n[上传] 开始上传 {len(execution_result['output_files'])} 个文件到S3...")
            for file_path in execution_result['output_files']:
                filename = os.path.basename(file_path)
                self.s3_uploader.upload_calibration_file(
                    file_path,
                    self.device_id,
                    task_type.value,
                    filename
                )
        
        return True
    
    def _cleanup_intermediate_files(
        self,
        task_type: CalibrationTaskType,
        task
    ) -> None:
        """
        清理中间文件（.log和.bag文件）
        
        Args:
            task_type: 任务类型
            task: 任务配置
        """
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
            return
        
        deleted_count = 0
        
        # 删除.log文件
        log_files = glob.glob(os.path.join(output_dir, "*.log"))
        for log_file in log_files:
            try:
                os.remove(log_file)
                deleted_count += 1
                print(f"[清理] 已删除日志文件: {os.path.basename(log_file)}")
            except Exception as e:
                print(f"[警告] 删除日志文件失败 {log_file}: {e}")
        
        # 删除.bag文件
        bag_files = glob.glob(os.path.join(output_dir, "*.bag"))
        for bag_file in bag_files:
            try:
                os.remove(bag_file)
                deleted_count += 1
                print(f"[清理] 已删除rosbag文件: {os.path.basename(bag_file)}")
            except Exception as e:
                print(f"[警告] 删除rosbag文件失败 {bag_file}: {e}")
        
        if deleted_count > 0:
            print(f"[清理] 共删除 {deleted_count} 个中间文件")
    
    def _print_summary(self, results: Dict[CalibrationTaskType, bool]):
        """打印标定结果总结"""
        print(f"\n{'='*80}")
        print(f"设备 {self.device_id} 标定总结")
        print(f"{'='*80}")
        
        total = len(results)
        success_count = sum(1 for v in results.values() if v)
        failed_count = total - success_count
        
        print(f"总任务数: {total}")
        print(f"成功: {success_count}")
        print(f"失败: {failed_count}")
        
        if failed_count > 0:
            print("\n失败的任务:")
            for task_type, success in results.items():
                if not success:
                    print(f"  - {task_type.value}")
        
        print(f"{'='*80}\n")

