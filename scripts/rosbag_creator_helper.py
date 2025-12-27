#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosbag创建辅助类
统一封装rosbag创建逻辑
"""

import os
import sys
from typing import Optional

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from calibration_common import CommandExecutor, CalibrationLogger


class RosbagCreator:
    """Rosbag创建器 - 统一封装rosbag创建逻辑"""
    
    def __init__(self, script_dir: Optional[str] = None):
        """
        初始化Rosbag创建器
        
        Args:
            script_dir: 脚本目录路径（默认使用当前脚本所在目录）
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
        创建rosbag文件
        
        Args:
            input_dir: 输入目录路径
            output_bag: 输出rosbag文件路径
            imu_src_rate: IMU源采样率（Hz，默认200）
            video_rate: 视频帧率（Hz，None表示使用原始帧率）
            trim_start_seconds: 去除开始时间（秒，None表示不裁剪）
            trim_end_seconds: 去除结束时间（秒，None表示不裁剪）
            
        Returns:
            是否成功
        """
        CalibrationLogger.step(1, "Creating rosbag from database files...")
        CalibrationLogger.info(f"Input directory: {input_dir}")
        CalibrationLogger.info(f"Output rosbag: {output_bag}")
        
        if video_rate is not None:
            CalibrationLogger.info(f"Video frame rate: {video_rate} fps")
        else:
            CalibrationLogger.info(f"Video frame rate: Original (no downsampling)")
        
        CalibrationLogger.info(f"IMU source rate: {imu_src_rate} Hz")
        
        # 确保输出目录存在
        output_dir = os.path.dirname(output_bag)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
            CalibrationLogger.info(f"Created output directory: {output_dir}")
        
        # 检查输入目录
        if not os.path.isdir(input_dir):
            CalibrationLogger.error(f"Input directory does not exist: {input_dir}")
            return False
        
        # 检查脚本是否存在
        if not os.path.exists(self.create_rosbag_script):
            CalibrationLogger.error(f"Script not found: {self.create_rosbag_script}")
            return False
        
        # 构建命令
        cmd = [
            sys.executable,
            self.create_rosbag_script,
            input_dir,
            "-ir_src", str(imu_src_rate),
            "-o", output_bag
        ]
        
        # 如果指定了视频帧率，添加参数
        if video_rate is not None:
            cmd.extend(["-vr", str(video_rate)])
        
        CalibrationLogger.info(f"\nRunning command: {' '.join(cmd)}")
        
        # 执行命令
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
        
        # 验证rosbag是否创建成功
        if not os.path.exists(output_bag):
            CalibrationLogger.error(f"Rosbag file was not created: {output_bag}")
            return False
        
        # 如果需要裁剪前后时间，使用rosbag工具
        if trim_start_seconds is not None or trim_end_seconds is not None:
            import rosbag
            import tempfile
            
            CalibrationLogger.info(f"\nTrimming rosbag: start={trim_start_seconds}s, end={trim_end_seconds}s")
            
            # 读取原始rosbag获取时间范围
            with rosbag.Bag(output_bag, 'r') as bag:
                start_time = bag.get_start_time()
                end_time = bag.get_end_time()
                duration = end_time - start_time
                
                # 计算裁剪后的时间范围
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
                
                # 创建临时文件
                temp_bag = output_bag + ".tmp"
                
                # 写入裁剪后的数据
                with rosbag.Bag(temp_bag, 'w') as outbag:
                    for topic, msg, t in bag.read_messages():
                        if new_start <= t.to_sec() <= new_end:
                            outbag.write(topic, msg, t)
            
            # 替换原文件
            import shutil
            shutil.move(temp_bag, output_bag)
            CalibrationLogger.success(f"Rosbag trimmed successfully")
        
        CalibrationLogger.success(f"Rosbag created successfully: {output_bag}")
        return True

