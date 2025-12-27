#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量标定管理器模块
负责管理多个设备的批量标定
"""

import os
import sys
import argparse
from typing import List, Optional
from pathlib import Path

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from device_calibration_manager import DeviceCalibrationManager
from result_recorder import ResultRecorder
from s3_uploader import S3Uploader


class BatchCalibrationManager:
    """批量标定管理器 - 管理多个设备的批量标定"""
    
    def __init__(
        self,
        scripts_dir: str,
        csv_file_path: str = "/tmp/calibration_results.csv",
        s3_config_file: Optional[str] = None
    ):
        """
        初始化批量标定管理器
        
        Args:
            scripts_dir: 脚本目录路径
            csv_file_path: CSV结果文件路径
            s3_config_file: S3配置文件路径（可选）
        """
        self.scripts_dir = scripts_dir
        self.result_recorder = ResultRecorder(csv_file_path)
        # S3Uploader内部会处理None情况，尝试使用默认配置
        self.s3_uploader = S3Uploader(s3_config_file)
    
    def discover_devices(self, data_root: str = "/data") -> List[str]:
        """
        自动发现所有设备ID
        
        Args:
            data_root: 数据根目录（默认/data）
            
        Returns:
            设备ID列表
        """
        device_ids = []
        
        if not os.path.exists(data_root):
            print(f"数据根目录不存在: {data_root}")
            return device_ids
        
        # 扫描/data目录下的所有子目录
        for item in os.listdir(data_root):
            item_path = os.path.join(data_root, item)
            
            # 检查是否是目录
            if not os.path.isdir(item_path):
                continue
            
            # 检查是否包含v1/data5目录（至少要有data5才认为是有效设备）
            v1_data5_path = os.path.join(item_path, "v1", "data5")
            if os.path.exists(v1_data5_path):
                device_ids.append(item)
        
        return sorted(device_ids)
    
    def calibrate_devices(
        self,
        device_ids: Optional[List[str]] = None,
        auto_discover: bool = True
    ) -> dict:
        """
        标定多个设备
        
        Args:
            device_ids: 设备ID列表（可选，如果为None且auto_discover=True则自动发现）
            auto_discover: 是否自动发现设备（默认True）
            
        Returns:
            所有设备的标定结果字典
        """
        # 确定要标定的设备列表
        if device_ids is None:
            if auto_discover:
                print("自动发现设备...")
                device_ids = self.discover_devices()
                if not device_ids:
                    print("未发现任何设备")
                    return {}
                print(f"发现 {len(device_ids)} 个设备: {', '.join(device_ids)}")
            else:
                print("错误: 未指定设备ID且未启用自动发现")
                return {}
        
        # 执行批量标定
        all_results = {}
        
        print(f"\n{'='*80}")
        print(f"开始批量标定 - 共 {len(device_ids)} 个设备")
        print(f"{'='*80}\n")
        
        for i, device_id in enumerate(device_ids, 1):
            print(f"\n{'#'*80}")
            print(f"设备 {i}/{len(device_ids)}: {device_id}")
            print(f"{'#'*80}\n")
            
            try:
                manager = DeviceCalibrationManager(
                    device_id=device_id,
                    scripts_dir=self.scripts_dir,
                    result_recorder=self.result_recorder,
                    s3_uploader=self.s3_uploader
                )
                
                results = manager.calibrate_all()
                all_results[device_id] = results
                
            except Exception as e:
                print(f"设备 {device_id} 标定过程中发生异常: {e}")
                import traceback
                traceback.print_exc()
                all_results[device_id] = {}
        
        # 打印最终总结
        self._print_final_summary(all_results)
        
        return all_results
    
    def _print_final_summary(self, all_results: dict):
        """打印最终总结"""
        print(f"\n{'='*80}")
        print("批量标定最终总结")
        print(f"{'='*80}\n")
        
        total_devices = len(all_results)
        print(f"总设备数: {total_devices}")
        
        if total_devices == 0:
            return
        
        # 统计每个任务的总体成功率
        from calibration_task import get_all_task_types
        
        task_types = get_all_task_types()
        task_stats = {}
        
        for task_type in task_types:
            success_count = 0
            for device_id, results in all_results.items():
                if task_type in results and results[task_type]:
                    success_count += 1
            task_stats[task_type] = (success_count, total_devices)
        
        print("\n各任务成功率:")
        for task_type, (success, total) in task_stats.items():
            percentage = (success / total * 100) if total > 0 else 0
            print(f"  {task_type.value}: {success}/{total} ({percentage:.1f}%)")
        
        print(f"\n结果已保存到: {self.result_recorder.csv_file_path}")
        print(f"{'='*80}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="批量标定工具 - 自动标定所有设备的所有标定任务"
    )
    
    parser.add_argument(
        "--device-ids",
        type=str,
        nargs="+",
        help="要标定的设备ID列表（如果不指定则自动发现）"
    )
    
    parser.add_argument(
        "--data-root",
        type=str,
        default="/data",
        help="数据根目录（默认: /data）"
    )
    
    parser.add_argument(
        "--scripts-dir",
        type=str,
        default=None,
        help="脚本目录路径（默认: 当前脚本所在目录）"
    )
    
    parser.add_argument(
        "--csv-file",
        type=str,
        default="/tmp/calibration_results.csv",
        help="CSV结果文件路径（默认: /tmp/calibration_results.csv）"
    )
    
    parser.add_argument(
        "--s3-config",
        type=str,
        default=None,
        help="S3配置文件路径（可选）"
    )
    
    args = parser.parse_args()
    
    # 确定脚本目录
    if args.scripts_dir is None:
        args.scripts_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 创建批量标定管理器
    manager = BatchCalibrationManager(
        scripts_dir=args.scripts_dir,
        csv_file_path=args.csv_file,
        s3_config_file=args.s3_config
    )
    
    # 执行批量标定
    results = manager.calibrate_devices(
        device_ids=args.device_ids,
        auto_discover=(args.device_ids is None)
    )
    
    # 返回退出码
    if results:
        # 检查是否有失败的设备
        has_failure = False
        for device_id, device_results in results.items():
            if not device_results or not all(device_results.values()):
                has_failure = True
                break
        
        return 0 if not has_failure else 1
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())

