#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import rospy
from sensor_msgs.msg import Imu, Image
import sqlite3
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import glob
import argparse
import re
import subprocess
import sys
from typing import List, Tuple, Optional, Dict, Any, Generator
from dataclasses import dataclass
from datetime import datetime

# Add script directory to path to ensure robocap_env can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import environment variables
try:
    import robocap_env
except ImportError:
    print("Warning: Could not import robocap_env, using default topic names")
    robocap_env = None

# IMU scaling factors (from merge_imu_data.py)
IN_ANGLVEL_SCALES = 0.000266316
IN_ACC_SCALES = 0.001197101

# Camera timestamp offset (nanoseconds)
# This offset will be added to all camera timestamps
CAMERA_TIMESTAMP_OFFSET_NS = 0


# ============================================================================
# 数据类 (Data Classes)
# ============================================================================

@dataclass
class IMUFileInfo:
    """IMU文件信息"""
    path: str
    device_number: int
    segment_number: Optional[int]
    topic_name: str


@dataclass
class VideoFileInfo:
    """视频文件信息"""
    path: str
    device_number: int
    position: str
    topic_name: str


@dataclass
class RosbagConfig:
    """Rosbag创建配置"""
    input_dir: str
    output_bag: Optional[str] = None
    imu_src_rate: int = 200
    imu_dst_rate: Optional[int] = None
    video_rate: Optional[float] = None
    min_timestamp_ns: Optional[int] = None
    max_timestamp_ns: Optional[int] = None
    camera_timestamp_offset_ns: int = 0
    log_dir: Optional[str] = None
    
    def __post_init__(self):
        """自动推断缺失的参数"""
        if self.output_bag is None:
            self.output_bag = os.path.join(self.input_dir, "output.bag")
        if self.log_dir is None:
            self.log_dir = self.input_dir


# ============================================================================
# 工具类 (Utility Classes)
# ============================================================================

class FileFinder:
    """文件查找器 - 查找和识别IMU数据库文件和视频文件"""
    
    def __init__(self, input_dir: str):
        """
        初始化文件查找器
        
        Args:
            input_dir: 输入目录路径
        """
        self.input_dir = input_dir
        if not os.path.isdir(input_dir):
            raise ValueError(f"Directory does not exist: {input_dir}")
    
    def find_imu_files(self) -> List[str]:
        """
        查找IMU数据库文件（支持新旧两种格式）
        
        Returns:
            IMU数据库文件路径列表（已排序）
        """
        db_files = []
        
        # 新格式: IMU0.db, IMU1.db, IMU2.db
        new_format_db = []
        for i in range(10):
            new_format_db.extend(glob.glob(os.path.join(self.input_dir, f'IMU{i}.db')))
            new_format_db.extend(glob.glob(os.path.join(self.input_dir, f'imu{i}.db')))
        new_format_db = sorted(set(new_format_db))
        
        if new_format_db:
            db_files = sorted(set(new_format_db))
        else:
            # 旧格式: IMUWriter_dev*.db
            db_files = sorted(glob.glob(os.path.join(self.input_dir, 'IMUWriter_dev*.db')))
            if not db_files:
                db_files = sorted(glob.glob(os.path.join(self.input_dir, 'imuwriter_dev*.db')))
                db_files.extend(sorted(glob.glob(os.path.join(self.input_dir, 'IMUWRITER_DEV*.db'))))
                db_files = sorted(set(db_files))
        
        return db_files
    
    def find_video_files(self) -> List[str]:
        """
        查找视频文件（支持新旧两种格式）
        
        Returns:
            视频文件路径列表（已排序）
        """
        # 新格式: left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4
        new_format_video = sorted(glob.glob(os.path.join(self.input_dir, 'left*.mp4')))
        new_format_video.extend(sorted(glob.glob(os.path.join(self.input_dir, 'right*.mp4'))))
        
        if new_format_video:
            return sorted(set(new_format_video))
        
        # 旧格式: video_dev*.mp4
        video_files = sorted(glob.glob(os.path.join(self.input_dir, 'video_dev*.mp4')))
        if not video_files:
            video_files = sorted(glob.glob(os.path.join(self.input_dir, 'VIDEO_DEV*.mp4')))
            video_files.extend(sorted(glob.glob(os.path.join(self.input_dir, 'Video_Dev*.mp4'))))
            video_files = sorted(set(video_files))
        
        return video_files
    
    def group_imu_files_by_device(self) -> Dict[int, List[str]]:
        """
        按设备号分组IMU文件
        
        Returns:
            {设备号: [文件路径列表]} 字典，文件已按segment排序
        """
        db_files = self.find_imu_files()
        grouped: Dict[int, List[Tuple[int, str]]] = {}
        
        for db_file in db_files:
            dev_number = FileParser._get_imu_dev_number(db_file)
            segment_number = FileParser._get_imu_segment_number(db_file)
            
            if dev_number is None:
                continue
            
            if segment_number is None:
                segment_number = 0
            
            if dev_number not in grouped:
                grouped[dev_number] = []
            grouped[dev_number].append((segment_number, db_file))
        
        # 按segment排序
        result: Dict[int, List[str]] = {}
        for dev_number, file_list in grouped.items():
            file_list.sort(key=lambda x: x[0])
            result[dev_number] = [filepath for _, filepath in file_list]
        
        return result


class FileParser:
    """文件解析器 - 从文件名提取元数据"""
    
    def __init__(self, robocap_env_module=None):
        """
        初始化文件解析器
        
        Args:
            robocap_env_module: robocap_env模块（可选，用于获取topic名称）
        """
        self.robocap_env = robocap_env_module
    
    @staticmethod
    def _get_imu_dev_number(db_file: str) -> Optional[int]:
        """提取IMU设备号"""
        basename = os.path.basename(db_file)
        # 新格式: IMU0.db, IMU1.db, IMU2.db
        match = re.match(r'IMU(\d+)\.db$', basename, re.IGNORECASE)
        if match:
            return int(match.group(1))
        # 旧格式: IMUWriter_dev0_session6_segment1
        match = re.search(r'dev(\d+)', basename)
        if match:
            return int(match.group(1))
        return None
    
    @staticmethod
    def _get_imu_segment_number(db_file: str) -> Optional[int]:
        """提取IMU segment号"""
        basename = os.path.basename(db_file)
        # 新格式没有segment号
        if re.match(r'IMU\d+\.db$', basename, re.IGNORECASE):
            return None
        # 旧格式: 提取segment号
        match = re.search(r'segment(\d+)', basename)
        if match:
            return int(match.group(1))
        return None
    
    def parse_imu_file(self, file_path: str) -> Optional[IMUFileInfo]:
        """
        解析IMU文件
        
        Args:
            file_path: IMU数据库文件路径
            
        Returns:
            IMUFileInfo对象或None（解析失败）
        """
        device_number = self._get_imu_dev_number(file_path)
        if device_number is None:
            return None
        
        segment_number = self._get_imu_segment_number(file_path)
        topic_name = self.get_imu_topic_name(device_number)
        
        return IMUFileInfo(
            path=file_path,
            device_number=device_number,
            segment_number=segment_number,
            topic_name=topic_name
        )
    
    def parse_video_file(self, file_path: str) -> Optional[VideoFileInfo]:
        """
        解析视频文件
        
        Args:
            file_path: 视频文件路径
            
        Returns:
            VideoFileInfo对象或None（解析失败）
        """
        basename = os.path.basename(file_path)
        name_without_ext = os.path.splitext(basename)[0]
        
        # 旧格式: video_dev{N}_session{S}_segment{G}_{position}
        dev_match = re.search(r'video_dev(\d+)', basename)
        if dev_match:
            device_number = int(dev_match.group(1))
            # 提取position
            parts = name_without_ext.split('_')
            dev_index = -1
            for i, part in enumerate(parts):
                if part.startswith('dev') and part[3:].isdigit():
                    dev_index = i
                    break
            if dev_index != -1 and len(parts) > dev_index + 1:
                position = parts[-1]
            else:
                position = 'unknown'
        else:
            # 新格式: left-front.mp4, right-front.mp4等
            device_number = 0
            position = name_without_ext
        
        topic_name = self.get_video_topic_name(device_number, position)
        
        return VideoFileInfo(
            path=file_path,
            device_number=device_number,
            position=position,
            topic_name=topic_name
        )
    
    def get_imu_topic_name(self, device_number: int) -> str:
        """
        获取IMU topic名称
        
        Args:
            device_number: 设备号
            
        Returns:
            ROS topic名称
        """
        if self.robocap_env is not None:
            return self.robocap_env.get_imu_topic_name(device_number)
        else:
            # 默认topic名称
            if device_number == 0:
                return '/imu_mid_0'
            elif device_number == 1:
                return '/imu_right_1'
            elif device_number == 2:
                return '/imu_left_2'
            else:
                return f'/imu_{device_number}'
    
    def get_video_topic_name(self, device_number: int, position: str) -> str:
        """
        获取视频topic名称
        
        Args:
            device_number: 设备号
            position: 位置字符串
            
        Returns:
            ROS topic名称
        """
        if self.robocap_env is not None:
            return self.robocap_env.get_video_topic_name(device_number, position)
        else:
            position_clean = position.replace('-', '_')
            return f"/cam{device_number}_{position_clean}/image_raw"


# ============================================================================
# 处理器类 (Processor Classes)
# ============================================================================

class IMUDataLoader:
    """IMU数据加载器 - 从数据库文件加载IMU数据"""
    
    def __init__(self):
        """初始化IMU数据加载器"""
        pass
    
    def load_gyro_data(self, db_path: str) -> List[Tuple[int, int, int, int, int, int]]:
        """
        加载陀螺仪数据
        
        Args:
            db_path: 数据库文件路径
            
        Returns:
            [(id, imuid_, x, y, z, timestamp), ...] 列表，按时间戳排序
        """
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        try:
            cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM gyro_data ORDER BY timestamp ASC")
            rows = cursor.fetchall()
            result = []
            for row in rows:
                result.append((
                    int(row[0]),
                    int(row[1]),
                    int(row[2]),
                    int(row[3]),
                    int(row[4]),
                    int(row[5])
                ))
            conn.close()
            return result
        except sqlite3.Error as e:
            print(f"Error: Failed to read gyro_data from {db_path}: {e}")
            conn.close()
            return []
        except (ValueError, TypeError) as e:
            print(f"Error: Failed to convert gyro_data values from {db_path}: {e}")
            conn.close()
            return []
    
    def load_acc_data(self, db_path: str) -> List[Tuple[int, int, int, int, int, int]]:
        """
        加载加速度计数据
        
        Args:
            db_path: 数据库文件路径
            
        Returns:
            [(id, imuid_, x, y, z, timestamp), ...] 列表，按时间戳排序
        """
        conn = sqlite3.connect(db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        try:
            cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM acc_data ORDER BY timestamp ASC")
            rows = cursor.fetchall()
            result = []
            for row in rows:
                result.append((
                    int(row[0]),
                    int(row[1]),
                    int(row[2]),
                    int(row[3]),
                    int(row[4]),
                    int(row[5])
                ))
            conn.close()
            return result
        except sqlite3.Error as e:
            print(f"Error: Failed to read acc_data from {db_path}: {e}")
            conn.close()
            return []
        except (ValueError, TypeError) as e:
            print(f"Error: Failed to convert acc_data values from {db_path}: {e}")
            conn.close()
            return []


class IMUDataProcessor:
    """IMU数据处理器 - 处理、合并、下采样IMU数据"""
    
    def __init__(
        self,
        anglvel_scale: float = IN_ANGLVEL_SCALES,
        acc_scale: float = IN_ACC_SCALES
    ):
        """
        初始化IMU数据处理器
        
        Args:
            anglvel_scale: 角速度缩放因子
            acc_scale: 加速度缩放因子
        """
        self.anglvel_scale = anglvel_scale
        self.acc_scale = acc_scale
    
    def _find_best_match(
        self,
        gyro_timestamp: int,
        acc_data: List[Tuple],
        sample_interval_ns: int,
        start_index: int = 0
    ) -> Tuple[Optional[Tuple], int]:
        """
        找到与gyro时间戳最匹配的acc数据
        
        Args:
            gyro_timestamp: 陀螺仪时间戳（纳秒）
            acc_data: 加速度计数据列表
            sample_interval_ns: 采样间隔（纳秒）
            start_index: 起始搜索索引
            
        Returns:
            (最佳匹配的acc数据, 下一个起始索引)
        """
        if not acc_data or start_index >= len(acc_data):
            return None, start_index
        
        left = start_index
        right = len(acc_data) - 1
        best_match = None
        min_diff = None
        best_index = start_index
        
        # 二分搜索
        while left <= right:
            mid = (left + right) // 2
            acc_timestamp = acc_data[mid][5]
            diff = abs(acc_timestamp - gyro_timestamp)
            
            if min_diff is None or diff < min_diff:
                min_diff = diff
                best_match = acc_data[mid]
                best_index = mid
            
            if acc_timestamp < gyro_timestamp:
                left = mid + 1
            else:
                right = mid - 1
        
        # 检查最佳匹配周围的邻居
        search_range = min(10, len(acc_data))
        start_check = max(start_index, best_index - search_range // 2)
        end_check = min(len(acc_data), best_index + search_range // 2 + 1)
        
        for i in range(start_check, end_check):
            acc_timestamp = acc_data[i][5]
            diff = abs(acc_timestamp - gyro_timestamp)
            if min_diff is None or diff < min_diff:
                min_diff = diff
                best_match = acc_data[i]
                best_index = i
        
        # 只有在采样间隔内才返回匹配
        if best_match is not None and min_diff is not None and min_diff <= sample_interval_ns:
            next_index = max(0, best_index - 1)
            return best_match, next_index
        
        return None, best_index
    
    def merge_data(
        self,
        gyro_data: List[Tuple],
        acc_data: List[Tuple],
        sampling_rate: int
    ) -> List[Tuple[int, float, float, float, float, float, float]]:
        """
        合并陀螺仪和加速度计数据
        
        Args:
            gyro_data: 陀螺仪数据列表
            acc_data: 加速度计数据列表
            sampling_rate: 采样率（Hz）
            
        Returns:
            [(timestamp_ns, omega_x, omega_y, omega_z, alpha_x, alpha_y, alpha_z), ...]
        """
        if not gyro_data:
            return []
        
        sample_interval_ns = 1000000000 // sampling_rate
        merged_data = []
        acc_index = 0
        
        for gyro in gyro_data:
            gyro_timestamp = gyro[5]
            gyro_x, gyro_y, gyro_z = gyro[2], gyro[3], gyro[4]
            
            # 找到最佳匹配的acc数据
            best_acc, next_index = self._find_best_match(
                gyro_timestamp, acc_data, sample_interval_ns, acc_index
            )
            
            if best_acc:
                acc_x, acc_y, acc_z = best_acc[2], best_acc[3], best_acc[4]
                
                # 应用缩放因子
                omega_x = float(gyro_x) * self.anglvel_scale
                omega_y = float(gyro_y) * self.anglvel_scale
                omega_z = float(gyro_z) * self.anglvel_scale
                alpha_x = float(acc_x) * self.acc_scale
                alpha_y = float(acc_y) * self.acc_scale
                alpha_z = float(acc_z) * self.acc_scale
                
                merged_data.append((
                    gyro_timestamp,
                    omega_x, omega_y, omega_z,
                    alpha_x, alpha_y, alpha_z
                ))
                acc_index = next_index
        
        return merged_data
    
    def downsample(
        self,
        merged_data: List[Tuple],
        src_rate: int,
        dst_rate: int
    ) -> List[Tuple]:
        """
        下采样IMU数据
        
        Args:
            merged_data: 合并后的数据
            src_rate: 源采样率（Hz）
            dst_rate: 目标采样率（Hz）
            
        Returns:
            下采样后的数据列表
        """
        if dst_rate >= src_rate:
            print(f"Warning: Target sampling rate {dst_rate} >= source sampling rate {src_rate}, skipping downsampling")
            return merged_data
        
        if not merged_data:
            return []
        
        dst_interval_ns = 1000000000 // dst_rate
        downsampled = []
        last_timestamp_ns = None
        
        for row in merged_data:
            timestamp_ns = row[0]
            
            if last_timestamp_ns is None or (timestamp_ns - last_timestamp_ns) >= dst_interval_ns:
                downsampled.append(row)
                last_timestamp_ns = timestamp_ns
        
        return downsampled
    
    def get_timestamp_range(
        self,
        db_files: List[str],
        sampling_rate: int,
        loader: IMUDataLoader
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        获取IMU文件的时间戳范围
        
        Args:
            db_files: 数据库文件路径列表
            sampling_rate: 采样率（Hz）
            loader: IMUDataLoader实例
            
        Returns:
            (min_timestamp_ns, max_timestamp_ns) 或 (None, None)
        """
        if not db_files:
            return None, None
        
        # 加载并合并所有文件的数据
        all_gyro_data = []
        all_acc_data = []
        
        for db_file in db_files:
            gyro_data = loader.load_gyro_data(db_file)
            acc_data = loader.load_acc_data(db_file)
            
            if gyro_data:
                all_gyro_data.extend(gyro_data)
            if acc_data:
                all_acc_data.extend(acc_data)
        
        if not all_gyro_data or not all_acc_data:
            return None, None
        
        # 按时间戳排序
        all_gyro_data.sort(key=lambda x: x[5])
        all_acc_data.sort(key=lambda x: x[5])
        
        # 合并数据
        merged_data = self.merge_data(all_gyro_data, all_acc_data, sampling_rate)
        
        if not merged_data:
            return None, None
        
        # 返回最小和最大时间戳
        timestamps = [row[0] for row in merged_data]
        return min(timestamps), max(timestamps)


class VideoDataProcessor:
    """视频数据处理器 - 处理视频文件"""
    
    def __init__(self, timestamp_offset_ns: int = 0):
        """
        初始化视频数据处理器
        
        Args:
            timestamp_offset_ns: 时间戳偏移量（纳秒）
        """
        self.timestamp_offset_ns = timestamp_offset_ns
        self.bridge = CvBridge()
    
    def get_base_timestamp(self, video_file: str) -> Optional[int]:
        """
        从MP4文件元数据获取基础时间戳
        
        Args:
            video_file: 视频文件路径
            
        Returns:
            基础时间戳（纳秒）或None
        """
        try:
            cmd = [
                'ffprobe', '-v', 'error', '-show_format',
                '-i', video_file
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            
            for line in result.stdout.split('\n'):
                if line.startswith('TAG:comment='):
                    comment_value = line.split('=', 1)[1].strip()
                    try:
                        timestamp_us = int(comment_value)
                        timestamp_ns = timestamp_us * 1000
                        return timestamp_ns
                    except ValueError as e:
                        print(f"Error: Failed to convert comment field to integer: {e}")
                        return None
            
            print(f"Error: TAG:comment field not found in {video_file}")
            return None
        except subprocess.CalledProcessError as e:
            print(f"Error: ffprobe execution failed: {e}")
            return None
        except FileNotFoundError:
            print("Error: ffprobe command not found, please ensure ffmpeg is installed")
            return None
    
    def get_time_base(self, video_file: str) -> Optional[Tuple[int, int]]:
        """
        获取视频时间基准
        
        Args:
            video_file: 视频文件路径
            
        Returns:
            (numerator, denominator) 或 None
        """
        try:
            cmd = [
                'ffprobe', '-v', 'error',
                '-select_streams', 'v:0',
                '-show_entries', 'stream=time_base',
                '-of', 'csv=p=0',
                video_file
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            
            if result.stdout.strip():
                time_base_str = result.stdout.strip()
                parts = time_base_str.split('/')
                if len(parts) == 2:
                    num = int(parts[0])
                    den = int(parts[1])
                    return (num, den)
            return None
        except (subprocess.CalledProcessError, ValueError, FileNotFoundError):
            return None
    
    def get_all_frame_pts(self, video_file: str) -> List[int]:
        """
        获取所有帧的PTS时间戳
        
        Args:
            video_file: 视频文件路径
            
        Returns:
            PTS时间戳列表
        """
        try:
            cmd = [
                'ffprobe', '-v', 'error',
                '-select_streams', 'v:0',
                '-show_entries', 'frame=pkt_pts',
                '-of', 'csv=p=0',
                video_file
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            
            if result.stdout.strip():
                pts_list = []
                for line in result.stdout.strip().split('\n'):
                    if line.strip() and line.strip() != 'N/A':
                        try:
                            pts = int(line.strip())
                            pts_list.append(pts)
                        except ValueError:
                            continue
                return pts_list
            return []
        except (subprocess.CalledProcessError, ValueError, FileNotFoundError) as e:
            print(f"  Warning: Cannot get frame PTS using ffprobe, will use FPS-based calculation: {e}")
            return []
    
    def get_timestamp_range(self, video_file: str) -> Tuple[Optional[int], Optional[int]]:
        """
        获取视频文件的时间戳范围
        
        Args:
            video_file: 视频文件路径
            
        Returns:
            (min_timestamp_ns, max_timestamp_ns) 或 (None, None)
        """
        base_timestamp_ns = self.get_base_timestamp(video_file)
        if base_timestamp_ns is None:
            return None, None
        
        time_base = self.get_time_base(video_file)
        frame_pts_list = self.get_all_frame_pts(video_file)
        
        if not frame_pts_list:
            # 使用FPS估算
            cap = cv2.VideoCapture(video_file)
            if not cap.isOpened():
                return None, None
            fps = cap.get(cv2.CAP_PROP_FPS)
            fps = fps if fps > 0 else 30
            frame_count_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            cap.release()
            
            if frame_count_total == 0:
                return None, None
            
            frame_interval_ns = int(1e9 / fps)
            min_timestamp_ns = base_timestamp_ns + self.timestamp_offset_ns
            max_timestamp_ns = base_timestamp_ns + (frame_count_total - 1) * frame_interval_ns + self.timestamp_offset_ns
            return min_timestamp_ns, max_timestamp_ns
        
        # 使用PTS计算
        if time_base is None:
            return None, None
        
        time_base_num, time_base_den = time_base
        if time_base_den <= 0:
            return None, None
        
        first_pts = frame_pts_list[0]
        last_pts = frame_pts_list[-1]
        
        first_pts_us = (first_pts * time_base_num * 1000000) // time_base_den
        last_pts_us = (last_pts * time_base_num * 1000000) // time_base_den
        
        min_timestamp_ns = base_timestamp_ns + first_pts_us * 1000 + self.timestamp_offset_ns
        max_timestamp_ns = base_timestamp_ns + last_pts_us * 1000 + self.timestamp_offset_ns
        
        return min_timestamp_ns, max_timestamp_ns
    
    def process_frames(
        self,
        video_file: str,
        target_fps: Optional[float] = None,
        min_timestamp_ns: Optional[int] = None,
        max_timestamp_ns: Optional[int] = None
    ) -> Generator[Tuple[np.ndarray, int, rospy.Time], None, None]:
        """
        处理视频帧（生成器）
        
        Args:
            video_file: 视频文件路径
            target_fps: 目标帧率（None表示使用原始帧率）
            min_timestamp_ns: 最小时间戳过滤
            max_timestamp_ns: 最大时间戳过滤
            
        Yields:
            (frame, timestamp_ns, ros_timestamp) 元组
        """
        base_timestamp_ns = self.get_base_timestamp(video_file)
        if base_timestamp_ns is None:
            return
        
        time_base = self.get_time_base(video_file)
        frame_pts_list = self.get_all_frame_pts(video_file)
        use_pts = (time_base is not None and len(frame_pts_list) > 0)
        
        cap = cv2.VideoCapture(video_file)
        if not cap.isOpened():
            return
        
        original_fps = cap.get(cv2.CAP_PROP_FPS)
        original_fps = original_fps if original_fps > 0 else 30
        frame_count_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        
        fps = target_fps if target_fps is not None and target_fps > 0 else original_fps
        
        if target_fps is not None and target_fps > 0 and target_fps < original_fps:
            frame_skip = int(original_fps / target_fps)
        else:
            frame_skip = 1
        
        frame_count = 0
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # 帧跳过逻辑
            if frame_count % frame_skip != 0:
                frame_count += 1
                continue
            
            # 计算时间戳
            if use_pts and frame_count < len(frame_pts_list):
                pts = frame_pts_list[frame_count]
                time_base_num, time_base_den = time_base
                if time_base_den > 0:
                    pts_us = (pts * time_base_num * 1000000) // time_base_den
                    pts_ns = pts_us * 1000
                else:
                    frame_interval_ns = int(1e9 / original_fps)
                    pts_ns = frame_count * frame_interval_ns
                current_timestamp_ns = base_timestamp_ns + pts_ns + self.timestamp_offset_ns
            else:
                frame_interval_ns = int(1e9 / original_fps)
                current_timestamp_ns = base_timestamp_ns + frame_count * frame_interval_ns + self.timestamp_offset_ns
            
            # 时间戳过滤
            if min_timestamp_ns is not None and current_timestamp_ns < min_timestamp_ns:
                frame_count += 1
                continue
            if max_timestamp_ns is not None and current_timestamp_ns > max_timestamp_ns:
                frame_count += 1
                continue
            
            timestamp = rospy.Time.from_sec(current_timestamp_ns / 1e9)
            yield (frame, current_timestamp_ns, timestamp)
            
            frame_count += 1
        
        cap.release()


class TimestampAligner:
    """时间戳对齐器 - 对齐所有数据源的时间戳"""
    
    def __init__(self):
        """初始化时间戳对齐器"""
        pass
    
    def scan_imu_timestamp_ranges(
        self,
        imu_files_by_device: Dict[int, List[str]],
        imu_processor: IMUDataProcessor,
        imu_loader: IMUDataLoader,
        sampling_rate: int
    ) -> Dict[int, Tuple[Optional[int], Optional[int]]]:
        """
        扫描IMU文件的时间戳范围
        
        Args:
            imu_files_by_device: 按设备分组的IMU文件
            imu_processor: IMU数据处理器实例
            imu_loader: IMU数据加载器实例
            sampling_rate: 采样率
            
        Returns:
            {设备号: (min_ts, max_ts)} 字典
        """
        ranges = {}
        for dev_number, db_files in imu_files_by_device.items():
            min_ts, max_ts = imu_processor.get_timestamp_range(db_files, sampling_rate, imu_loader)
            ranges[dev_number] = (min_ts, max_ts)
        return ranges
    
    def scan_video_timestamp_ranges(
        self,
        video_files: List[str],
        video_processor: VideoDataProcessor
    ) -> Dict[str, Tuple[Optional[int], Optional[int]]]:
        """
        扫描视频文件的时间戳范围
        
        Args:
            video_files: 视频文件路径列表
            video_processor: 视频数据处理器实例
            
        Returns:
            {文件路径: (min_ts, max_ts)} 字典
        """
        ranges = {}
        for video_file in video_files:
            min_ts, max_ts = video_processor.get_timestamp_range(video_file)
            ranges[video_file] = (min_ts, max_ts)
        return ranges
    
    def calculate_aligned_range(
        self,
        timestamp_ranges: List[Tuple[Optional[int], Optional[int]]]
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        计算对齐的时间戳范围
        
        Args:
            timestamp_ranges: 时间戳范围列表
            
        Returns:
            (aligned_min_ts, aligned_max_ts) 对齐后的时间范围
        """
        valid_ranges = [r for r in timestamp_ranges if r[0] is not None and r[1] is not None]
        
        if not valid_ranges:
            return None, None
        
        min_timestamps = [r[0] for r in valid_ranges]
        max_timestamps = [r[1] for r in valid_ranges]
        
        # 最小时间戳：所有文件中最晚的开始时间
        aligned_min_ts = max(min_timestamps)
        # 最大时间戳：所有文件中最早的结束时间
        aligned_max_ts = min(max_timestamps)
        
        return aligned_min_ts, aligned_max_ts
    
    @staticmethod
    def is_in_range(
        timestamp_ns: int,
        min_timestamp_ns: Optional[int],
        max_timestamp_ns: Optional[int]
    ) -> bool:
        """
        检查时间戳是否在范围内
        
        Args:
            timestamp_ns: 要检查的时间戳
            min_timestamp_ns: 最小时间戳
            max_timestamp_ns: 最大时间戳
            
        Returns:
            是否在范围内
        """
        if min_timestamp_ns is not None and timestamp_ns < min_timestamp_ns:
            return False
        if max_timestamp_ns is not None and timestamp_ns > max_timestamp_ns:
            return False
        return True


class LogWriter:
    """日志写入器 - 写入处理日志"""
    
    def __init__(self, log_dir: str):
        """
        初始化日志写入器
        
        Args:
            log_dir: 日志文件输出目录
        """
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
    
    def write_imu_log(
        self,
        file_info: IMUFileInfo,
        db_files: List[str],
        merged_data: List[Tuple],
        config: RosbagConfig,
        stats: Dict[str, Any]
    ) -> str:
        """
        写入IMU处理日志
        
        Args:
            file_info: IMU文件信息
            db_files: 数据库文件路径列表
            merged_data: 合并后的数据
            config: 处理配置
            stats: 统计信息字典
            
        Returns:
            日志文件路径
        """
        if len(db_files) == 1:
            db_basename = os.path.basename(db_files[0])
            db_name_without_ext = os.path.splitext(db_basename)[0]
            log_file_path = os.path.join(self.log_dir, f"{db_name_without_ext}.log")
        else:
            log_file_path = os.path.join(self.log_dir, f"IMUWriter_dev{file_info.device_number}_merged.log")
        
        with open(log_file_path, 'w', encoding='utf-8') as log_file:
            log_file.write("="*80 + "\n")
            log_file.write("IMU Database Timestamp Log\n")
            log_file.write(f"Generation time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            if len(db_files) == 1:
                log_file.write(f"IMU database file: {os.path.basename(db_files[0])}\n")
                log_file.write(f"Full path: {db_files[0]}\n")
            else:
                log_file.write(f"IMU database files ({len(db_files)} segments merged):\n")
                for i, db_file in enumerate(db_files, 1):
                    log_file.write(f"  Segment {i}: {os.path.basename(db_file)}\n")
                    log_file.write(f"    Full path: {db_file}\n")
            
            log_file.write(f"Topic name: {file_info.topic_name}\n")
            log_file.write(f"Dev number: {file_info.device_number}\n")
            log_file.write(f"Total records (after merging): {len(merged_data)}\n")
            
            if config.min_timestamp_ns is not None:
                log_file.write(f"Min timestamp filter: {config.min_timestamp_ns} ns ({config.min_timestamp_ns / 1e9:.9f} sec)\n")
            if config.max_timestamp_ns is not None:
                log_file.write(f"Max timestamp filter: {config.max_timestamp_ns} ns ({config.max_timestamp_ns / 1e9:.9f} sec)\n")
            
            log_file.write("-"*80 + "\n")
            log_file.write(f"{'Record#':>10} {'Timestamp(ns)':>20} {'Timestamp(sec)':>25} {'ROS Timestamp':>25}\n")
            log_file.write("-"*80 + "\n")
            
            record_index = 0
            for row in merged_data:
                timestamp_ns = row[0]
                
                # 时间戳过滤
                if not TimestampAligner.is_in_range(timestamp_ns, config.min_timestamp_ns, config.max_timestamp_ns):
                    continue
                
                timestamp = rospy.Time.from_sec(timestamp_ns / 1e9)
                log_file.write(f"{record_index:>10} {timestamp_ns:>20} {timestamp_ns / 1e9:>25.9f} {timestamp.secs}:{timestamp.nsecs:>25}\n")
                record_index += 1
            
            log_file.write("-"*80 + "\n")
            log_file.write(f"Actual processed records: {record_index}\n")
            log_file.write(f"Expected total records: {len(merged_data)}\n")
            if record_index != len(merged_data):
                log_file.write("Warning: Actual processed records != expected total records!\n")
            log_file.write("="*80 + "\n")
        
        return log_file_path
    
    def write_video_log(
        self,
        file_info: VideoFileInfo,
        video_file: str,
        config: RosbagConfig,
        stats: Dict[str, Any]
    ) -> str:
        """
        写入视频处理日志
        
        Args:
            file_info: 视频文件信息
            video_file: 视频文件路径
            config: 处理配置
            stats: 统计信息字典
            
        Returns:
            日志文件路径
        """
        video_basename = os.path.basename(video_file)
        video_name_without_ext = os.path.splitext(video_basename)[0]
        log_file_path = os.path.join(self.log_dir, f"{video_name_without_ext}.log")
        
        with open(log_file_path, 'w', encoding='utf-8') as log_file:
            log_file.write("="*80 + "\n")
            log_file.write("Video File Timestamp Log\n")
            log_file.write(f"Generation time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            log_file.write(f"Video file: {os.path.basename(video_file)}\n")
            log_file.write(f"Full path: {video_file}\n")
            log_file.write(f"Topic name: {file_info.topic_name}\n")
            log_file.write("-"*80 + "\n")
            log_file.write(f"{'Frame#':>10} {'Timestamp(ns)':>20} {'Timestamp(sec)':>25} {'ROS Timestamp':>25}\n")
            log_file.write("-"*80 + "\n")
            
            # 这里只写头部，实际帧数据在process_video时写入
            log_file.write("="*80 + "\n")
        
        return log_file_path


class RosbagWriter:
    """ROS Bag写入器 - 写入ROS bag文件"""
    
    def __init__(self, bag_path: str):
        """
        初始化ROS Bag写入器
        
        Args:
            bag_path: rosbag文件路径
        """
        self.bag_path = bag_path
        self.bag = None
        self.topics = []
        
        # 确保输出目录存在
        output_dir = os.path.dirname(bag_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
    
    def __enter__(self):
        """上下文管理器入口"""
        self.bag = rosbag.Bag(self.bag_path, 'w')
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        if self.bag is not None:
            self.bag.close()
    
    def write_imu_message(
        self,
        topic_name: str,
        timestamp: rospy.Time,
        omega_x: float,
        omega_y: float,
        omega_z: float,
        alpha_x: float,
        alpha_y: float,
        alpha_z: float,
        frame_id: str
    ) -> None:
        """
        写入IMU消息
        
        Args:
            topic_name: topic名称
            timestamp: ROS时间戳
            omega_x/y/z: 角速度
            alpha_x/y/z: 加速度
            frame_id: 坐标系ID
        """
        if self.bag is None:
            raise RuntimeError("Bag file is not open")
        
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = frame_id
        imu_msg.angular_velocity.x = omega_x
        imu_msg.angular_velocity.y = omega_y
        imu_msg.angular_velocity.z = omega_z
        imu_msg.linear_acceleration.x = alpha_x
        imu_msg.linear_acceleration.y = alpha_y
        imu_msg.linear_acceleration.z = alpha_z
        imu_msg.orientation.w = 1.0
        
        self.bag.write(topic_name, imu_msg, timestamp)
        
        if topic_name not in self.topics:
            self.topics.append(topic_name)
    
    def write_image_message(
        self,
        topic_name: str,
        timestamp: rospy.Time,
        frame: np.ndarray,
        frame_id: str
    ) -> None:
        """
        写入图像消息
        
        Args:
            topic_name: topic名称
            timestamp: ROS时间戳
            frame: OpenCV图像帧
            frame_id: 坐标系ID
        """
        if self.bag is None:
            raise RuntimeError("Bag file is not open")
        
        bridge = CvBridge()
        try:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = frame_id
            
            self.bag.write(topic_name, image_msg, timestamp)
            
            if topic_name not in self.topics:
                self.topics.append(topic_name)
        except Exception as e:
            print(f"Error converting frame: {e}")
            raise
    
    def get_topics(self) -> List[str]:
        """
        获取所有已写入的topic
        
        Returns:
            topic名称列表
        """
        return self.topics.copy()


# ============================================================================
# 保留原有函数以保持兼容性（后续会逐步替换）
# ============================================================================

def get_mp4_base_timestamp(mp4_file: str) -> Optional[int]:
    """
    Read TAG:comment field from MP4 file metadata to get base timestamp (in microseconds)
    Returns: base timestamp in nanoseconds, or None if failed
    """
    try:
        # Use ffprobe to read metadata
        cmd = [
            'ffprobe', '-v', 'error', '-show_format',
            '-i', mp4_file
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        # Find TAG:comment field
        for line in result.stdout.split('\n'):
            if line.startswith('TAG:comment='):
                comment_value = line.split('=', 1)[1].strip()
                try:
                    # Convert to uint64, unit is microseconds, convert to nanoseconds
                    timestamp_us = int(comment_value)
                    timestamp_ns = timestamp_us * 1000  # microseconds to nanoseconds
                    return timestamp_ns
                except ValueError as e:
                    print("Error: Failed to convert comment field to integer: {}".format(e))
                    return None
        
        print("Error: TAG:comment field not found in {}".format(mp4_file))
        return None
    except subprocess.CalledProcessError as e:
        print("Error: ffprobe execution failed: {}".format(e))
        return None
    except FileNotFoundError:
        print("Error: ffprobe command not found, please ensure ffmpeg is installed")
        return None

def load_gyro_data(db_path: str) -> List[Tuple]:
    """
    Load gyro data from database
    Returns: [(id, imuid_, x, y, z, timestamp), ...]
    timestamp is 64-bit integer (nanoseconds)
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM gyro_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        result = []
        for row in rows:
            result.append((
                int(row[0]),
                int(row[1]),
                int(row[2]),
                int(row[3]),
                int(row[4]),
                int(row[5])
            ))
        conn.close()
        return result
    except sqlite3.Error as e:
        print("Error: Failed to read gyro_data from {}: {}".format(db_path, e))
        conn.close()
        return []
    except (ValueError, TypeError) as e:
        print("Error: Failed to convert gyro_data values from {}: {}".format(db_path, e))
        conn.close()
        return []

def load_acc_data(db_path: str) -> List[Tuple]:
    """
    Load acc data from database
    Returns: [(id, imuid_, x, y, z, timestamp), ...]
    timestamp is 64-bit integer (nanoseconds)
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM acc_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        result = []
        for row in rows:
            result.append((
                int(row[0]),
                int(row[1]),
                int(row[2]),
                int(row[3]),
                int(row[4]),
                int(row[5])
            ))
        conn.close()
        return result
    except sqlite3.Error as e:
        print("Error: Failed to read acc_data from {}: {}".format(db_path, e))
        conn.close()
        return []
    except (ValueError, TypeError) as e:
        print("Error: Failed to convert acc_data values from {}: {}".format(db_path, e))
        conn.close()
        return []

def find_best_match(gyro_timestamp: int, acc_data: List[Tuple], sample_interval_ns: int, start_index: int = 0) -> Tuple[Optional[Tuple], int]:
    """
    Find the best matching acc data for a given gyro timestamp
    Uses binary search for efficiency
    """
    if not acc_data or start_index >= len(acc_data):
        return None, start_index
    
    left = start_index
    right = len(acc_data) - 1
    best_match = None
    min_diff = None
    best_index = start_index
    
    while left <= right:
        mid = (left + right) // 2
        acc_timestamp = acc_data[mid][5]
        diff = abs(acc_timestamp - gyro_timestamp)
        
        if min_diff is None or diff < min_diff:
            min_diff = diff
            best_match = acc_data[mid]
            best_index = mid
        
        if acc_timestamp < gyro_timestamp:
            left = mid + 1
        else:
            right = mid - 1
    
    # Check neighbors around the best match
    search_range = min(10, len(acc_data))
    start_check = max(start_index, best_index - search_range // 2)
    end_check = min(len(acc_data), best_index + search_range // 2 + 1)
    
    for i in range(start_check, end_check):
        acc_timestamp = acc_data[i][5]
        diff = abs(acc_timestamp - gyro_timestamp)
        if min_diff is None or diff < min_diff:
            min_diff = diff
            best_match = acc_data[i]
            best_index = i
    
    if best_match is not None and min_diff is not None and min_diff <= sample_interval_ns:
        next_index = max(0, best_index - 1)
        return best_match, next_index
    
    return None, best_index

def merge_imu_data(gyro_data: List[Tuple], acc_data: List[Tuple], sampling_rate: int = 200) -> List[Tuple]:
    """
    Merge gyro and acc data
    Returns: [(timestamp, omega_x, omega_y, omega_z, alpha_x, alpha_y, alpha_z), ...]
    """
    if not gyro_data:
        return []
    
    sample_interval_ns = 1000000000 // sampling_rate
    merged_data = []
    acc_index = 0
    
    for gyro in gyro_data:
        gyro_timestamp = gyro[5]
        gyro_x, gyro_y, gyro_z = gyro[2], gyro[3], gyro[4]
        
        best_acc, next_index = find_best_match(gyro_timestamp, acc_data, sample_interval_ns, acc_index)
        
        if best_acc:
            acc_x, acc_y, acc_z = best_acc[2], best_acc[3], best_acc[4]
            
            # Apply scaling factors
            omega_x = float(gyro_x) * IN_ANGLVEL_SCALES
            omega_y = float(gyro_y) * IN_ANGLVEL_SCALES
            omega_z = float(gyro_z) * IN_ANGLVEL_SCALES
            alpha_x = float(acc_x) * IN_ACC_SCALES
            alpha_y = float(acc_y) * IN_ACC_SCALES
            alpha_z = float(acc_z) * IN_ACC_SCALES
            
            merged_data.append((
                gyro_timestamp,
                omega_x, omega_y, omega_z,
                alpha_x, alpha_y, alpha_z
            ))
            acc_index = next_index
    
    return merged_data

def downsample_imu_data(merged_data: List[Tuple], src_rate: int, dst_rate: int) -> List[Tuple]:
    """
    Downsample IMU data
    Args:
        merged_data: List of merged IMU data
        src_rate: Source sampling rate (Hz)
        dst_rate: Target sampling rate (Hz)
    Returns: Downsampled data list
    """
    if dst_rate >= src_rate:
        print("Warning: Target sampling rate {} >= source sampling rate {}, skipping downsampling".format(dst_rate, src_rate))
        return merged_data
    
    if not merged_data:
        return []
    
    # Calculate target sampling interval (nanoseconds)
    dst_interval_ns = 1000000000 // dst_rate
    
    downsampled = []
    last_timestamp_ns = None
    
    for row in merged_data:
        timestamp_ns = row[0]
        
        # Keep if it's the first frame, or time interval reaches target sampling interval
        if last_timestamp_ns is None or (timestamp_ns - last_timestamp_ns) >= dst_interval_ns:
            downsampled.append(row)
            last_timestamp_ns = timestamp_ns
    
    return downsampled

def get_imu_dev_number(db_file: str) -> Optional[int]:
    """
    Extract dev number from IMU database filename
    Supports both formats:
    - New format: IMU0.db, IMU1.db, IMU2.db -> 0, 1, 2
    - Old format: IMUWriter_dev0_session6_segment1 -> 0
    """
    basename = os.path.basename(db_file)
    # Try new format first: IMU0.db, IMU1.db, IMU2.db
    match = re.match(r'IMU(\d+)\.db$', basename, re.IGNORECASE)
    if match:
        return int(match.group(1))
    # Try old format: IMUWriter_dev0_session6_segment1
    match = re.search(r'dev(\d+)', basename)
    if match:
        return int(match.group(1))
    return None

def get_imu_segment_number(db_file: str) -> Optional[int]:
    """
    Extract segment number from IMU database filename
    Supports both formats:
    - New format: IMU0.db, IMU1.db, IMU2.db -> None (no segments)
    - Old format: IMUWriter_dev0_session6_segment1 -> 1
    """
    basename = os.path.basename(db_file)
    # New format has no segment number
    if re.match(r'IMU\d+\.db$', basename, re.IGNORECASE):
        return None
    # Old format: extract segment number
    match = re.search(r'segment(\d+)', basename)
    if match:
        return int(match.group(1))
    return None

def group_imu_files_by_dev(db_files: List[str]) -> Dict[int, List[str]]:
    """
    Group IMU database files by device number and sort by segment number.
    
    Args:
        db_files: List of database file paths
        
    Returns:
        Dictionary mapping device numbers to sorted list of file paths (sorted by segment number)
    """
    grouped: Dict[int, List[Tuple[int, str]]] = {}  # dev -> [(segment, filepath), ...]
    
    for db_file in db_files:
        dev_number = get_imu_dev_number(db_file)
        segment_number = get_imu_segment_number(db_file)
        
        if dev_number is None:
            continue
        
        # Use segment number for sorting, default to 0 if not found
        if segment_number is None:
            segment_number = 0
        
        if dev_number not in grouped:
            grouped[dev_number] = []
        grouped[dev_number].append((segment_number, db_file))
    
    # Sort by segment number for each device
    result: Dict[int, List[str]] = {}
    for dev_number, file_list in grouped.items():
        file_list.sort(key=lambda x: x[0])  # Sort by segment number
        result[dev_number] = [filepath for _, filepath in file_list]
    
    return result

def get_imu_topic_name(dev_number: int) -> str:
    """
    Return IMU topic name based on dev number.
    Uses robocap_env for topic definitions.
    """
    if robocap_env is not None:
        return robocap_env.get_imu_topic_name(dev_number)
    else:
        # Fallback to default topic names if robocap_env is not available
        if dev_number == 0:
            return '/imu_mid_0'
        elif dev_number == 1:
            return '/imu_right_1'
        elif dev_number == 2:
            return '/imu_left_2'
        else:
            return '/imu_{}'.format(dev_number)

def get_video_dev_number(video_file: str) -> Optional[int]:
    """
    Extract dev number from video filename
    Supports both formats:
    - New format: left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4 -> None (no dev number, use 0)
    - Old format: video_dev1_xx_left-front.mp4 -> 1
    """
    basename = os.path.basename(video_file)
    # Try old format first: video_dev{N}
    match = re.search(r'video_dev(\d+)', basename)
    if match:
        return int(match.group(1))
    # New format: no dev number in filename, default to 0
    # Files like left-front.mp4, right-front.mp4, etc. don't have dev number
    return 0

def get_video_topic_name(video_file: str) -> Optional[str]:
    """
    Generate topic name from video filename.
    Uses robocap_env for topic definitions.
    Supports both formats:
    - New format: left-front.mp4 -> /cam0_left_front/image_raw, right-front.mp4 -> /cam0_right_front/image_raw, etc.
    - Old format: video_dev{N}_session{S}_segment{G}_{position}.mp4
    Examples: 
    - left-front.mp4 -> /cam0_left_front/image_raw
    - right-eye.mp4 -> /cam0_right_eye/image_raw
    - left.mp4 -> /cam0_left/image_raw
    - video_dev0_session11_segment1_right-eye.mp4 -> /cam0_right_eye/image_raw
    """
    basename = os.path.basename(video_file)
    name_without_ext = os.path.splitext(basename)[0]
    
    # Try old format first (video_dev{N}_session{S}_segment{G}_{position})
    if robocap_env is not None:
        topic_name = robocap_env.get_video_topic_name_from_file(video_file)
        if topic_name is not None:
            return topic_name
    
    # New format: simple filenames like left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4
    # These don't have dev number, use dev 0
    # Position is the filename without extension
    dev_number = 0
    position = name_without_ext
    
    if robocap_env is not None:
        return robocap_env.get_video_topic_name(dev_number, position)
    else:
        # Fallback implementation
        position_clean = position.replace('-', '_')
        return f"/cam{dev_number}_{position_clean}/image_raw"

def get_video_time_base(video_file: str) -> Optional[Tuple[int, int]]:
    """
    Get video time base
    Returns: (numerator, denominator) or None
    """
    try:
        cmd = [
            'ffprobe', '-v', 'error',
            '-select_streams', 'v:0',
            '-show_entries', 'stream=time_base',
            '-of', 'csv=p=0',
            video_file
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        if result.stdout.strip():
            time_base_str = result.stdout.strip()
            # Format is usually "1/10000" or similar
            parts = time_base_str.split('/')
            if len(parts) == 2:
                num = int(parts[0])
                den = int(parts[1])
                return (num, den)
        return None
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError):
        return None

def get_all_frame_pts(video_file: str) -> List[int]:
    """
    Get all frame PTS timestamps (in time base units) using ffprobe
    Returns: [pts0, pts1, ...] or empty list
    """
    try:
        cmd = [
            'ffprobe', '-v', 'error',
            '-select_streams', 'v:0',
            '-show_entries', 'frame=pkt_pts',
            '-of', 'csv=p=0',
            video_file
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        if result.stdout.strip():
            pts_list = []
            for line in result.stdout.strip().split('\n'):
                if line.strip() and line.strip() != 'N/A':
                    try:
                        pts = int(line.strip())
                        pts_list.append(pts)
                    except ValueError:
                        continue
            return pts_list
        return []
    except (subprocess.CalledProcessError, ValueError, FileNotFoundError) as e:
        print("  Warning: Cannot get frame PTS using ffprobe, will use FPS-based calculation: {}".format(e))
        return []

def get_imu_timestamp_range(db_files: List[str], imu_src_rate: int) -> Tuple[Optional[int], Optional[int]]:
    """
    Get timestamp range of IMU database file(s)
    If multiple files are provided, they will be merged first.
    Returns: (min_timestamp_ns, max_timestamp_ns) or (None, None)
    """
    if not db_files:
        return None, None
    
    # Load and merge data from all files
    all_gyro_data = []
    all_acc_data = []
    
    for db_file in db_files:
        gyro_data = load_gyro_data(db_file)
        acc_data = load_acc_data(db_file)
        
        if gyro_data:
            all_gyro_data.extend(gyro_data)
        if acc_data:
            all_acc_data.extend(acc_data)
    
    if not all_gyro_data or not all_acc_data:
        return None, None
    
    # Sort combined data by timestamp
    all_gyro_data.sort(key=lambda x: x[5])
    all_acc_data.sort(key=lambda x: x[5])
    
    merged_data = merge_imu_data(all_gyro_data, all_acc_data, imu_src_rate)
    
    if not merged_data:
        return None, None
    
    # Return min and max timestamps
    timestamps = [row[0] for row in merged_data]
    return min(timestamps), max(timestamps)

def get_video_timestamp_range(video_file: str) -> Tuple[Optional[int], Optional[int]]:
    """
    Get timestamp range of video file
    Returns: (min_timestamp_ns, max_timestamp_ns) or (None, None)
    """
    # Get base timestamp
    base_timestamp_ns = get_mp4_base_timestamp(video_file)
    if base_timestamp_ns is None:
        return None, None
    
    # Get time base and all frame PTS
    time_base = get_video_time_base(video_file)
    frame_pts_list = get_all_frame_pts(video_file)
    
    if not frame_pts_list:
        # If PTS cannot be obtained, estimate using FPS
        cap = cv2.VideoCapture(video_file)
        if not cap.isOpened():
            return None, None
        fps = cap.get(cv2.CAP_PROP_FPS)
        fps = fps if fps > 0 else 30
        frame_count_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        
        if frame_count_total == 0:
            return None, None
        
        # Estimate timestamp range
        frame_interval_ns = int(1e9 / fps)
        min_timestamp_ns = base_timestamp_ns + CAMERA_TIMESTAMP_OFFSET_NS
        max_timestamp_ns = base_timestamp_ns + (frame_count_total - 1) * frame_interval_ns + CAMERA_TIMESTAMP_OFFSET_NS
        return min_timestamp_ns, max_timestamp_ns
    
    # Calculate timestamp range using PTS
    if time_base is None:
        return None, None
    
    time_base_num, time_base_den = time_base
    if time_base_den <= 0:
        return None, None
    
    # Calculate timestamps of first and last frames
    first_pts = frame_pts_list[0]
    last_pts = frame_pts_list[-1]
    
    first_pts_us = (first_pts * time_base_num * 1000000) // time_base_den
    last_pts_us = (last_pts * time_base_num * 1000000) // time_base_den
    
    min_timestamp_ns = base_timestamp_ns + first_pts_us * 1000 + CAMERA_TIMESTAMP_OFFSET_NS
    max_timestamp_ns = base_timestamp_ns + last_pts_us * 1000 + CAMERA_TIMESTAMP_OFFSET_NS
    
    return min_timestamp_ns, max_timestamp_ns

def process_imu_db_to_bag(bag, db_files: List[str], input_dir: str, imu_src_rate: int, imu_dst_rate: Optional[int] = None, 
                          min_timestamp_ns: Optional[int] = None, max_timestamp_ns: Optional[int] = None):
    """
    Process one or multiple IMU database files (for the same device) and write to rosbag.
    If multiple files are provided, they will be merged by concatenating gyro_data and acc_data,
    then sorted by timestamp before processing.
    
    Args:
        bag: ROS bag object
        db_files: List of database file paths (should be for the same device, sorted by segment)
        input_dir: Input directory for log files
        imu_src_rate: IMU source sampling rate
        imu_dst_rate: Optional IMU target sampling rate
        min_timestamp_ns: Optional minimum timestamp filter
        max_timestamp_ns: Optional maximum timestamp filter
        
    Returns: (topic_name, first_timestamp)
    """
    if not db_files:
        return None, None
    
    dev_number = get_imu_dev_number(db_files[0])
    if dev_number is None:
        print("Warning: Cannot extract dev number from filename {}, skipping".format(db_files[0]))
        return None, None
    
    topic_name = get_imu_topic_name(dev_number)
    
    # Create log file name based on device number
    if len(db_files) == 1:
        db_basename = os.path.basename(db_files[0])
        db_name_without_ext = os.path.splitext(db_basename)[0]
        log_file_path = os.path.join(input_dir, "{}.log".format(db_name_without_ext))
        print("Processing IMU database: {} -> topic {}".format(os.path.basename(db_files[0]), topic_name))
    else:
        # Multiple segments: create combined log file
        log_file_path = os.path.join(input_dir, "IMUWriter_dev{}_merged.log".format(dev_number))
        print("Processing IMU database ({} segments): -> topic {}".format(len(db_files), topic_name))
        for i, db_file in enumerate(db_files, 1):
            print("  Segment {}: {}".format(i, os.path.basename(db_file)))
    
    # Load and merge data from all segment files
    all_gyro_data = []
    all_acc_data = []
    
    for db_file in db_files:
        gyro_data = load_gyro_data(db_file)
        acc_data = load_acc_data(db_file)
        
        if gyro_data:
            all_gyro_data.extend(gyro_data)
        if acc_data:
            all_acc_data.extend(acc_data)
    
    if not all_gyro_data:
        print("Warning: No gyro data found in any database files, skipping")
        return None, None
    if not all_acc_data:
        print("Warning: No acc data found in any database files, skipping")
        return None, None
    
    # Sort combined data by timestamp (should already be sorted, but ensure it)
    all_gyro_data.sort(key=lambda x: x[5])  # Sort by timestamp (index 5)
    all_acc_data.sort(key=lambda x: x[5])  # Sort by timestamp (index 5)
    
    if len(db_files) > 1:
        print("  Merged {} gyro samples and {} acc samples from {} segment files".format(
            len(all_gyro_data), len(all_acc_data), len(db_files)))
    
    # Merge data (using source sampling rate)
    merged_data = merge_imu_data(all_gyro_data, all_acc_data, imu_src_rate)
    
    if not merged_data:
        print("Warning: Cannot merge IMU data from device {}, skipping".format(dev_number))
        return None, None
    
    # Downsample if target sampling rate is specified
    original_count = len(merged_data)
    if imu_dst_rate is not None:
        merged_data = downsample_imu_data(merged_data, imu_src_rate, imu_dst_rate)
        print("  IMU data downsampled: {} Hz -> {} Hz ({} -> {} records)".format(
            imu_src_rate, imu_dst_rate, original_count, len(merged_data)))
    
    first_timestamp = None
    record_index = 0
    
    # Open log file and write
    with open(log_file_path, 'w', encoding='utf-8') as log_file:
        # Write log file header
        log_file.write("="*80 + "\n")
        log_file.write("IMU Database Timestamp Log\n")
        log_file.write("Generation time: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        if len(db_files) == 1:
            log_file.write("IMU database file: {}\n".format(os.path.basename(db_files[0])))
            log_file.write("Full path: {}\n".format(db_files[0]))
        else:
            log_file.write("IMU database files ({} segments merged):\n".format(len(db_files)))
            for i, db_file in enumerate(db_files, 1):
                log_file.write("  Segment {}: {}\n".format(i, os.path.basename(db_file)))
                log_file.write("    Full path: {}\n".format(db_file))
        log_file.write("Topic name: {}\n".format(topic_name))
        log_file.write("Dev number: {}\n".format(dev_number))
        log_file.write("Total records (after merging): {}\n".format(len(merged_data)))
        if min_timestamp_ns is not None:
            log_file.write("Min timestamp filter: {} ns ({:.9f} sec)\n".format(min_timestamp_ns, min_timestamp_ns / 1e9))
        if max_timestamp_ns is not None:
            log_file.write("Max timestamp filter: {} ns ({:.9f} sec)\n".format(max_timestamp_ns, max_timestamp_ns / 1e9))
        log_file.write("-"*80 + "\n")
        log_file.write("{:>10} {:>20} {:>25} {:>25}\n".format(
            "Record#", "Timestamp(ns)", "Timestamp(sec)", "ROS Timestamp"
        ))
        log_file.write("-"*80 + "\n")
        
        # Write to rosbag (filter data outside timestamp range)
        filtered_count = 0
        last_timestamp_ns = None
        interval_warning_count = 0
        
        # Calculate expected interval (nanoseconds)
        if imu_dst_rate is not None:
            expected_interval_ns = int(1e9 / imu_dst_rate)
        else:
            expected_interval_ns = int(1e9 / imu_src_rate)
        tolerance_ns = 1000000  # 1ms tolerance (nanoseconds)
        max_expected_interval_ns = expected_interval_ns + tolerance_ns
        
        for row in merged_data:
            timestamp_ns = row[0]
            
            # Timestamp range filtering
            if min_timestamp_ns is not None and timestamp_ns < min_timestamp_ns:
                filtered_count += 1
                continue
            if max_timestamp_ns is not None and timestamp_ns > max_timestamp_ns:
                filtered_count += 1
                continue
            
            # Check time interval
            if last_timestamp_ns is not None:
                interval_ns = timestamp_ns - last_timestamp_ns
                if interval_ns > max_expected_interval_ns:
                    interval_ms = interval_ns / 1e6
                    expected_ms = expected_interval_ns / 1e6
                    warning_msg = "Warning: Time interval {:.3f} ms between records {} and {} exceeds expected {:.3f} ms (1ms tolerance)".format(
                        interval_ms, record_index - 1, record_index, expected_ms)
                    print("  {}".format(warning_msg))
                    log_file.write("  # {}\n".format(warning_msg))
                    interval_warning_count += 1
            
            timestamp = rospy.Time.from_sec(timestamp_ns / 1e9)
            
            if first_timestamp is None:
                first_timestamp = timestamp
            
            # Write to log
            log_file.write("{:>10} {:>20} {:>25.9f} {:>25}\n".format(
                record_index,
                timestamp_ns,
                timestamp_ns / 1e9,
                "{}:{}".format(timestamp.secs, timestamp.nsecs)
            ))
            log_file.flush()  # Ensure immediate write
            
            last_timestamp_ns = timestamp_ns
            
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = "imu{}_link".format(dev_number)
            
            # Fill angular velocity and linear acceleration
            imu_msg.angular_velocity.x = row[1]  # omega_x
            imu_msg.angular_velocity.y = row[2]  # omega_y
            imu_msg.angular_velocity.z = row[3]  # omega_z
            imu_msg.linear_acceleration.x = row[4]  # alpha_x
            imu_msg.linear_acceleration.y = row[5]  # alpha_y
            imu_msg.linear_acceleration.z = row[6]  # alpha_z
            imu_msg.orientation.w = 1.0
            
            bag.write(topic_name, imu_msg, timestamp)
            record_index += 1
        
        if filtered_count > 0:
            print("  Filtered {} records outside timestamp range".format(filtered_count))
            log_file.write("Filtered {} records outside timestamp range\n".format(filtered_count))
        if interval_warning_count > 0:
            print("  Found {} time interval warnings".format(interval_warning_count))
            log_file.write("Found {} time interval warnings\n".format(interval_warning_count))
        
        # Record actual processed record count in log
        log_file.write("-"*80 + "\n")
        log_file.write("Actual processed records: {}\n".format(record_index))
        log_file.write("Expected total records: {}\n".format(len(merged_data)))
        if record_index != len(merged_data):
            log_file.write("Warning: Actual processed records != expected total records!\n")
        log_file.write("="*80 + "\n")
    
    print("  IMU data writing completed, {} records total".format(len(merged_data)))
    print("  Log file saved to: {}".format(log_file_path))
    
    return topic_name, first_timestamp

def process_video_to_bag(bag, video_file: str, input_dir: str, target_fps: Optional[float] = None,
                         min_timestamp_ns: Optional[int] = None, max_timestamp_ns: Optional[int] = None):
    """
    Process single video file and write to rosbag
    """
    topic_name = get_video_topic_name(video_file)
    if topic_name is None:
        print("Warning: Cannot generate topic name from filename {}, skipping".format(video_file))
        return
    
    print("Processing video: {} -> topic {}".format(os.path.basename(video_file), topic_name))
    
    # Create independent log file
    # Filename format: video_dev{n}_xx_left-front_timestamp.log (based on original filename)
    video_basename = os.path.basename(video_file)
    video_name_without_ext = os.path.splitext(video_basename)[0]  # Remove .mp4 extension
    log_file_path = os.path.join(input_dir, "{}.log".format(video_name_without_ext))
    
    # Get base timestamp
    base_timestamp_ns = get_mp4_base_timestamp(video_file)
    if base_timestamp_ns is None:
        print("Error: Cannot get base timestamp from {}, skipping".format(video_file))
        return
    
    print("  Base timestamp: {} ns ({} us)".format(base_timestamp_ns, base_timestamp_ns // 1000))
    
    # Get time base and all frame PTS
    time_base = get_video_time_base(video_file)
    frame_pts_list = get_all_frame_pts(video_file)
    
    bridge = CvBridge()
    cap = cv2.VideoCapture(video_file)
    if not cap.isOpened():
        print("Error: Cannot open video file {}".format(video_file))
        return
    
    # Get video properties
    original_fps = cap.get(cv2.CAP_PROP_FPS)
    original_fps = original_fps if original_fps > 0 else 30
    frame_count_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Determine actual FPS to use (use target_fps if specified, otherwise use original FPS)
    fps = target_fps if target_fps is not None and target_fps > 0 else original_fps
    
    if target_fps is not None and target_fps > 0:
        print("  Original video FPS: {:.2f}, Target FPS: {:.2f}, Total frames: {}".format(original_fps, target_fps, frame_count_total))
        # Calculate frame skipping interval
        if target_fps >= original_fps:
            print("  Warning: Target FPS {} >= Original FPS {}, no frame skipping".format(target_fps, original_fps))
            frame_skip = 1
        else:
            frame_skip = int(original_fps / target_fps)
            print("  Frame skip interval: keep 1 frame every {} frames".format(frame_skip))
    else:
        print("  Video FPS: {:.2f}, Total frames: {}".format(original_fps, frame_count_total))
        frame_skip = 1
    
    # If time base and PTS list are successfully obtained, use them; otherwise use FPS-based calculation
    use_pts = (time_base is not None and len(frame_pts_list) > 0)
    if use_pts:
        time_base_num, time_base_den = time_base
        time_base_sec = time_base_num / time_base_den if time_base_den > 0 else 0
        print("  Using ffmpeg PTS timestamps")
        print("  Time base: {}/{} = {:.10f} sec/unit".format(time_base_num, time_base_den, time_base_sec))
        if len(frame_pts_list) > 1:
            pts_diff = frame_pts_list[1] - frame_pts_list[0]
            frame_interval_sec = pts_diff * time_base_sec
            frame_interval_ms = frame_interval_sec * 1000
            print("  First frame PTS difference: {}, corresponding time interval: {:.3f} ms".format(pts_diff, frame_interval_ms))
    else:
        print("  Using FPS-based timestamp calculation")
    
    # Open log file and write
    with open(log_file_path, 'w', encoding='utf-8') as log_file:
        # Write log file header
        log_file.write("="*80 + "\n")
        log_file.write("Video File Timestamp Log\n")
        log_file.write("Generation time: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        log_file.write("Video file: {}\n".format(os.path.basename(video_file)))
        log_file.write("Full path: {}\n".format(video_file))
        log_file.write("Topic name: {}\n".format(topic_name))
        log_file.write("Base timestamp: {} ns ({} us)\n".format(base_timestamp_ns, base_timestamp_ns // 1000))
        log_file.write("Original video FPS: {:.2f}\n".format(original_fps))
        if target_fps is not None and target_fps > 0:
            log_file.write("Target FPS: {:.2f}\n".format(target_fps))
            log_file.write("Frame skip interval: keep 1 frame every {} frames\n".format(frame_skip))
        log_file.write("Total frames: {}\n".format(frame_count_total))
        log_file.write("Timestamp method: {}\n".format("ffmpeg PTS" if use_pts else "FPS-based calculation"))
        if min_timestamp_ns is not None:
            log_file.write("Min timestamp filter: {} ns ({:.9f} sec)\n".format(min_timestamp_ns, min_timestamp_ns / 1e9))
        if max_timestamp_ns is not None:
            log_file.write("Max timestamp filter: {} ns ({:.9f} sec)\n".format(max_timestamp_ns, max_timestamp_ns / 1e9))
        log_file.write("-"*80 + "\n")
        log_file.write("{:>10} {:>20} {:>25} {:>25} {:>15}\n".format(
            "Frame#", "Timestamp(ns)", "Timestamp(sec)", "ROS Timestamp", "Relative(ms)"
        ))
        log_file.write("-"*80 + "\n")
        
        frame_count = 0
        output_frame_count = 0
        filtered_by_timestamp_count = 0
        interval_warning_count = 0
        first_frame_timestamp_ns = None
        last_output_timestamp_ns = None
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            
            # Frame skipping logic: skip if not target frame
            if frame_count % frame_skip != 0:
                frame_count += 1
                continue
            
            # Calculate current frame timestamp
            # Note: If frame skipping is performed, timestamp should be based on original frame PTS, not output frame count
            if use_pts and frame_count < len(frame_pts_list):
                # Use PTS timestamp from ffmpeg (using original frame index frame_count)
                pts = frame_pts_list[frame_count]
                # Convert PTS from time base units to nanoseconds
                # PTS timestamp = PTS value * (time_base.num / time_base.den) seconds
                # Convert to nanoseconds: PTS * (num/den) * 1e9
                time_base_num, time_base_den = time_base
                if time_base_den > 0:
                    # Use integer arithmetic to avoid precision loss
                    # pts_ns = pts * time_base_num * 1e9 / time_base_den
                    # To avoid floating point errors, calculate microseconds first then convert to nanoseconds
                    pts_us = (pts * time_base_num * 1000000) // time_base_den
                    pts_ns = pts_us * 1000  # microseconds to nanoseconds
                else:
                    # If time_base is invalid, fall back to original FPS-based calculation
                    frame_interval_ns = int(1e9 / original_fps)
                    pts_ns = frame_count * frame_interval_ns
                current_timestamp_ns = base_timestamp_ns + pts_ns + CAMERA_TIMESTAMP_OFFSET_NS
            else:
                # Use original FPS-based calculation (fallback)
                # Even with frame skipping, timestamp should be based on original frame position
                frame_interval_ns = int(1e9 / original_fps)
                current_timestamp_ns = base_timestamp_ns + frame_count * frame_interval_ns + CAMERA_TIMESTAMP_OFFSET_NS
            
            # Timestamp range filtering
            if min_timestamp_ns is not None and current_timestamp_ns < min_timestamp_ns:
                filtered_by_timestamp_count += 1
                frame_count += 1
                continue
            if max_timestamp_ns is not None and current_timestamp_ns > max_timestamp_ns:
                filtered_by_timestamp_count += 1
                frame_count += 1
                continue
            
            timestamp = rospy.Time.from_sec(current_timestamp_ns / 1e9)
            
            # Record first frame timestamp
            if first_frame_timestamp_ns is None:
                first_frame_timestamp_ns = current_timestamp_ns
                last_output_timestamp_ns = current_timestamp_ns
            else:
                # Check time interval (only for output frames)
                # Calculate expected interval (based on actual FPS used)
                if target_fps is not None and target_fps > 0:
                    expected_interval_ns = int(1e9 / target_fps)
                else:
                    expected_interval_ns = int(1e9 / original_fps)
                tolerance_ns = 1000000  # 1ms tolerance (nanoseconds)
                max_expected_interval_ns = expected_interval_ns + tolerance_ns
                
                interval_ns = current_timestamp_ns - last_output_timestamp_ns
                if interval_ns > max_expected_interval_ns:
                    interval_ms = interval_ns / 1e6
                    expected_ms = expected_interval_ns / 1e6
                    warning_msg = "Warning: Time interval {:.3f} ms between frames {} and {} exceeds expected {:.3f} ms (1ms tolerance)".format(
                        interval_ms, output_frame_count - 1, output_frame_count, expected_ms)
                    print("  {}".format(warning_msg))
                    log_file.write("  # {}\n".format(warning_msg))
                    interval_warning_count += 1
            
            # Write to log (write before processing frame to ensure all frames are recorded)
            relative_time_ms = (current_timestamp_ns - first_frame_timestamp_ns) / 1e6 if first_frame_timestamp_ns is not None else 0.0
            log_file.write("{:>10} {:>20} {:>25.9f} {:>25} {:>15.3f}".format(
                output_frame_count,  # Use output frame count
                current_timestamp_ns,
                current_timestamp_ns / 1e9,
                "{}:{}".format(timestamp.secs, timestamp.nsecs),
                relative_time_ms
            ))
            
            last_output_timestamp_ns = current_timestamp_ns
            
            # Try to process and write to rosbag
            frame_written = False
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header.stamp = timestamp
                
                # Extract frame_id from topic name
                # Example: /cam1_left_front/image_raw -> cam1_left_front
                frame_id = topic_name.replace('/image_raw', '').replace('/', '')
                image_msg.header.frame_id = frame_id
                
                bag.write(topic_name, image_msg, timestamp)
                frame_written = True
            except Exception as e:
                print("  Error converting frame {}: {}".format(frame_count, e))
                # Mark this frame as processing failed in log
                log_file.write(" [Processing failed: {}]".format(str(e)[:30]))
            
            # Complete log line
            log_file.write("\n")
            log_file.flush()  # Ensure immediate write
            
            output_frame_count += 1
            frame_count += 1
        
        # Record actual processed frame count in log
        log_file.write("-"*80 + "\n")
        log_file.write("Actual processed frames: {}\n".format(output_frame_count))
        log_file.write("Original total frames: {}\n".format(frame_count_total))
        if filtered_by_timestamp_count > 0:
            log_file.write("Frames filtered by timestamp range: {}\n".format(filtered_by_timestamp_count))
        if interval_warning_count > 0:
            log_file.write("Found {} time interval warnings\n".format(interval_warning_count))
        if target_fps is not None and target_fps > 0:
            expected_output_frames = frame_count_total // frame_skip
            log_file.write("Expected output frames (after skipping): {}\n".format(expected_output_frames))
            if output_frame_count != expected_output_frames:
                log_file.write("Warning: Actual output frames != expected output frames!\n")
        log_file.write("="*80 + "\n")
    
    cap.release()
    warning_summary = []
    if filtered_by_timestamp_count > 0:
        warning_summary.append("filtered {} frames".format(filtered_by_timestamp_count))
    if interval_warning_count > 0:
        warning_summary.append("{} interval warnings".format(interval_warning_count))
    
    if warning_summary:
        print("  Video data writing completed, {} frames total (original {} frames, {})".format(
            output_frame_count, frame_count_total, ", ".join(warning_summary)))
    else:
        print("  Video data writing completed, {} frames total (original {} frames)".format(output_frame_count, frame_count_total))
    print("  Log file saved to: {}".format(log_file_path))

def main():
    """
    Main function
    """
    parser = argparse.ArgumentParser(description="Create rosbag from IMU database (.db) and video (.mp4) files")
    parser.add_argument("input_dir", type=str, help="Input directory path containing IMU database and video files")
    parser.add_argument("-vr", "--video_rate", type=float, default=None,
                        help="Video target frame rate (Hz), if specified then frame skipping will be performed, otherwise use original frame rate")
    parser.add_argument("-ir_src", "--imu_rate_src", type=int, required=True,
                        help="IMU source sampling rate (Hz), must be specified")
    parser.add_argument("-ir_dst", "--imu_rate_dst", type=int, default=None,
                        help="IMU target sampling rate (Hz), if specified then downsample to this rate, otherwise use all data")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output rosbag file path (supports directory hierarchy), if not specified then output to input_dir/output.bag")
    args = parser.parse_args()
    
    input_dir = args.input_dir
    if not os.path.isdir(input_dir):
        print("Error: Directory '{}' does not exist".format(input_dir))
        return
    
    # Find files - support both old and new naming formats
    # Old format: IMUWriter_dev*.db, video_dev*.mp4
    # New format: IMU0.db, IMU1.db, IMU2.db, and simple video names like left-front.mp4, right-front.mp4, etc.
    db_files = []
    video_files = []
    
    # Find IMU database files - try new format first, then old format
    # New format: IMU0.db, IMU1.db, IMU2.db
    new_format_db = []
    for i in range(10):  # Check IMU0.db through IMU9.db
        new_format_db.extend(glob.glob(os.path.join(input_dir, f'IMU{i}.db')))
        new_format_db.extend(glob.glob(os.path.join(input_dir, f'imu{i}.db')))
    new_format_db = sorted(set(new_format_db))  # Remove duplicates and sort
    if new_format_db:
        db_files = sorted(set(new_format_db))
    else:
        # Try old format
        db_files = sorted(glob.glob(os.path.join(input_dir, 'IMUWriter_dev*.db')))
        if not db_files:
            db_files = sorted(glob.glob(os.path.join(input_dir, 'imuwriter_dev*.db')))
            db_files.extend(sorted(glob.glob(os.path.join(input_dir, 'IMUWRITER_DEV*.db'))))
            db_files = sorted(set(db_files))
    
    # Find video files - try new format first (simple names), then old format
    # New format: left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4
    new_format_video = sorted(glob.glob(os.path.join(input_dir, 'left*.mp4')))
    new_format_video.extend(sorted(glob.glob(os.path.join(input_dir, 'right*.mp4'))))
    if new_format_video:
        video_files = sorted(set(new_format_video))
    else:
        # Try old format
        video_files = sorted(glob.glob(os.path.join(input_dir, 'video_dev*.mp4')))
        if not video_files:
            video_files = sorted(glob.glob(os.path.join(input_dir, 'VIDEO_DEV*.mp4')))
            video_files.extend(sorted(glob.glob(os.path.join(input_dir, 'Video_Dev*.mp4'))))
            video_files = sorted(set(video_files))
    
    if not db_files and not video_files:
        print("Error: No IMU database files or video files found in '{}'".format(input_dir))
        print("Hint: Please ensure file naming follows one of these formats:")
        print("  Old format:")
        print("    - IMU files: IMUWriter_dev{N}_session{S}_segment{G}.db")
        print("    - Video files: video_dev{N}_session{S}_segment{G}_{position}.mp4")
        print("  New format:")
        print("    - IMU files: IMU0.db, IMU1.db, IMU2.db")
        print("    - Video files: left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4")
        return
    
    print("Files found in directory '{}':".format(input_dir))
    if db_files:
        print("  - IMU database files ({}):".format(len(db_files)))
        for f in db_files:
            print("    * {}".format(os.path.basename(f)))
    else:
        print("  - IMU database files: not found")
    
    if video_files:
        print("  - Video files ({}):".format(len(video_files)))
        for f in video_files:
            print("    * {}".format(os.path.basename(f)))
    else:
        print("  - Video files: not found")
    
    # Hint messages
    if db_files and not video_files:
        print("\nHint: Only IMU files found, will process IMU data only")
    elif video_files and not db_files:
        print("\nHint: Only video files found, will process video data only")
    elif db_files and video_files:
        print("\nHint: Both IMU and video files found, will process all data")
    
    # Set output bag file
    if args.output:
        output_bag_file = args.output
        # Ensure output directory exists
        output_dir = os.path.dirname(output_bag_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)
            print("Created output directory: {}".format(output_dir))
    else:
        output_bag_file = os.path.join(input_dir, 'output.bag')
    
    # Validate parameters
    if args.imu_rate_src <= 0:
        print("Error: IMU source sampling rate must be greater than 0")
        return
    
    if args.imu_rate_dst is not None and args.imu_rate_dst <= 0:
        print("Error: IMU target sampling rate must be greater than 0")
        return
    
    if args.imu_rate_dst is not None and args.imu_rate_dst >= args.imu_rate_src:
        print("Warning: IMU target sampling rate {} >= source sampling rate {}, will not downsample".format(
            args.imu_rate_dst, args.imu_rate_src))
        args.imu_rate_dst = None
    
    if args.video_rate is not None and args.video_rate <= 0:
        print("Error: Video target frame rate must be greater than 0")
        return
    
    print("\n" + "="*80)
    print("Processing parameters:")
    print("  Input directory: {}".format(input_dir))
    print("  Output file: {}".format(output_bag_file))
    print("  IMU source sampling rate: {} Hz".format(args.imu_rate_src))
    if args.imu_rate_dst:
        print("  IMU target sampling rate: {} Hz (downsampling)".format(args.imu_rate_dst))
    else:
        print("  IMU target sampling rate: No downsampling (use all data)")
    if args.video_rate:
        print("  Video target frame rate: {} Hz (frame skipping)".format(args.video_rate))
    else:
        print("  Video target frame rate: Use original frame rate")
    print("="*80)
    
    # Scan all files to find timestamp range
    print("\nScanning timestamp range of all files...")
    all_min_timestamps = []
    all_max_timestamps = []
    
    # Scan IMU files (grouped by device)
    imu_files_by_dev = group_imu_files_by_dev(db_files)
    for dev_number in sorted(imu_files_by_dev.keys()):
        db_files_for_dev = imu_files_by_dev[dev_number]
        min_ts, max_ts = get_imu_timestamp_range(db_files_for_dev, args.imu_rate_src)
        if min_ts is not None and max_ts is not None:
            all_min_timestamps.append(min_ts)
            all_max_timestamps.append(max_ts)
            if len(db_files_for_dev) == 1:
                print("  IMU device {}: {} ns - {} ns".format(
                    dev_number, min_ts, max_ts))
            else:
                print("  IMU device {} ({} segments): {} ns - {} ns".format(
                    dev_number, len(db_files_for_dev), min_ts, max_ts))
    
    # Scan video files
    for video_file in video_files:
        min_ts, max_ts = get_video_timestamp_range(video_file)
        if min_ts is not None and max_ts is not None:
            all_min_timestamps.append(min_ts)
            all_max_timestamps.append(max_ts)
            print("  Video {}: {} ns - {} ns".format(
                os.path.basename(video_file), min_ts, max_ts))
    
    # Determine aligned timestamp range
    if all_min_timestamps and all_max_timestamps:
        # Min timestamp: maximum of all files' first frame (latest start time)
        min_timestamp_ns = max(all_min_timestamps)
        # Max timestamp: minimum of all files' last frame (earliest end time)
        max_timestamp_ns = min(all_max_timestamps)
        
        print("\nTimestamp alignment range:")
        print("  Min timestamp (latest start): {} ns ({:.9f} sec)".format(
            min_timestamp_ns, min_timestamp_ns / 1e9))
        print("  Max timestamp (earliest end): {} ns ({:.9f} sec)".format(
            max_timestamp_ns, max_timestamp_ns / 1e9))
        print("  Aligned duration: {:.3f} seconds".format((max_timestamp_ns - min_timestamp_ns) / 1e9))
        
        if min_timestamp_ns >= max_timestamp_ns:
            print("Warning: Min timestamp >= Max timestamp, may have no overlapping data!")
    else:
        print("Warning: Cannot get timestamp range, will not perform timestamp filtering")
        min_timestamp_ns = None
        max_timestamp_ns = None
    
    print("\nStarting processing... Output will be written to '{}'".format(output_bag_file))
    print("Each file will generate an independent log file (.log format)")
    
    with rosbag.Bag(output_bag_file, 'w') as bag:
        all_topics = []
        
        # Process IMU database files
        if db_files:
            print("\n=== Processing IMU Data ===")
            # Group files by device number and sort by segment
            imu_files_by_dev = group_imu_files_by_dev(db_files)
            
            for dev_number in sorted(imu_files_by_dev.keys()):
                db_files_for_dev = imu_files_by_dev[dev_number]
                try:
                    if len(db_files_for_dev) > 1:
                        print("\nDevice {}: Found {} segment files, will merge them".format(
                            dev_number, len(db_files_for_dev)))
                    topic_name, first_timestamp = process_imu_db_to_bag(
                        bag, db_files_for_dev, input_dir, args.imu_rate_src, args.imu_rate_dst,
                        min_timestamp_ns, max_timestamp_ns)
                    if topic_name and topic_name not in all_topics:
                        all_topics.append(topic_name)
                except Exception as e:
                    print("Error: Failed to process IMU device {}: {}".format(dev_number, e))
                    import traceback
                    traceback.print_exc()
                    continue
        
        # Process video files
        if video_files:
            print("\n=== Processing Video Data ===")
            for video_file in video_files:
                try:
                    process_video_to_bag(bag, video_file, input_dir, args.video_rate,
                                        min_timestamp_ns, max_timestamp_ns)
                    topic_name = get_video_topic_name(video_file)
                    if topic_name and topic_name not in all_topics:
                        all_topics.append(topic_name)
                except Exception as e:
                    print("Error: Failed to process video file {}: {}".format(os.path.basename(video_file), e))
                    import traceback
                    traceback.print_exc()
                    continue
    
    print("\n" + "="*80)
    print("Processing completed!")
    print("="*80)
    print("ROS bag file: {}".format(output_bag_file))
    
    if all_topics:
        print("\nContains the following topics ({} total):".format(len(all_topics)))
        for topic in sorted(all_topics):
            print("  - {}".format(topic))
    else:
        print("\nWarning: No topics were successfully processed, rosbag may be empty")
    
    print("\nTimestamp logs for each file have been saved as independent .log files")
    
    # Statistics
    if db_files:
        print("\nIMU file processing statistics:")
        print("  - Found {} IMU database files".format(len(db_files)))
    if video_files:
        print("\nVideo file processing statistics:")
        print("  - Found {} video files".format(len(video_files)))

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Unhandled exception occurred: {}".format(e))
        import traceback
        traceback.print_exc()

