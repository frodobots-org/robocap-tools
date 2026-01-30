#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batch calibration manager module
Responsible for managing batch calibration of multiple devices
"""

import os
import sys
import argparse
from typing import List, Optional
from pathlib import Path

# Add script directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from device_calibration_manager import DeviceCalibrationManager
from result_recorder import ResultRecorder
from s3_uploader import S3Uploader
from results_report import ResultsReporter
from calibration_task import CalibrationTaskType, get_all_task_types


class BatchCalibrationManager:
    """Batch calibration manager - manage batch calibration of multiple devices"""
    
    def __init__(
        self,
        scripts_dir: str,
        csv_file_path: str = "/tmp/calibration_results.csv",
        s3_config_file: Optional[str] = None,
        api_base_url: Optional[str] = None,
        enable_api_reporting: bool = True
    ):
        """
        Initialize batch calibration manager
        
        Args:
            scripts_dir: Script directory path
            csv_file_path: CSV result file path
            s3_config_file: S3 configuration file path (optional)
            api_base_url: API base URL for results reporting (optional, can also use CALIBRATION_API_URL env var)
            enable_api_reporting: Whether to enable API reporting (default True)
        """
        self.scripts_dir = scripts_dir
        self.result_recorder = ResultRecorder(csv_file_path)
        # S3Uploader internally handles None case, try to use default configuration
        self.s3_uploader = S3Uploader(s3_config_file)
        # ResultsReporter for API reporting
        self.results_reporter = ResultsReporter(api_base_url=api_base_url, enabled=enable_api_reporting) if enable_api_reporting else None
    
    def discover_devices(self, data_root: str = "/data") -> List[str]:
        """
        Automatically discover all device IDs
        
        Args:
            data_root: Data root directory (default /data)
            
        Returns:
            List of device IDs
        """
        device_ids = []
        
        if not os.path.exists(data_root):
            print(f"Data root directory does not exist: {data_root}")
            return device_ids
        
        # Scan all subdirectories under /data directory
        for item in os.listdir(data_root):
            item_path = os.path.join(data_root, item)
            
            # Check if it's a directory
            if not os.path.isdir(item_path):
                continue
            
            # Check if it contains v1/data5 directory (IMU intrinsic data, required for all calibrations)
            # Note: data5 is IMU intrinsic, data1-4 are camera intrinsics, data6-9 are camera-IMU extrinsics
            v1_data5_path = os.path.join(item_path, "v1", "data5")
            if os.path.exists(v1_data5_path):
                device_ids.append(item)
        
        return sorted(device_ids)
    
    def calibrate_devices(
        self,
        device_ids: Optional[List[str]] = None,
        auto_discover: bool = True,
        selected_tasks: Optional[List['CalibrationTaskType']] = None
    ) -> dict:
        """
        Calibrate multiple devices

        Args:
            device_ids: List of device IDs (optional, if None and auto_discover=True then auto-discover)
            auto_discover: Whether to auto-discover devices (default True)
            selected_tasks: List of specific tasks to run (optional, if None run all tasks)

        Returns:
            Dictionary of calibration results for all devices
        """
        # Determine list of devices to calibrate
        if device_ids is None:
            if auto_discover:
                print("Auto-discovering devices...")
                device_ids = self.discover_devices()
                if not device_ids:
                    print("No devices found")
                    return {}
                print(f"Found {len(device_ids)} devices: {', '.join(device_ids)}")
            else:
                print("Error: Device ID not specified and auto-discovery not enabled")
                return {}
        
        # Execute batch calibration
        all_results = {}
        
        print(f"\n{'='*80}")
        print(f"Start batch calibration - {len(device_ids)} devices total")
        print(f"{'='*80}\n")
        
        for i, device_id in enumerate(device_ids, 1):
            print(f"\n{'#'*80}")
            print(f"Device {i}/{len(device_ids)}: {device_id}")
            print(f"{'#'*80}\n")
            
            try:
                manager = DeviceCalibrationManager(
                    device_id=device_id,
                    scripts_dir=self.scripts_dir,
                    result_recorder=self.result_recorder,
                    s3_uploader=self.s3_uploader,
                    results_reporter=self.results_reporter
                )

                results = manager.calibrate_all(selected_tasks=selected_tasks)
                all_results[device_id] = results
                
            except Exception as e:
                print(f"Exception occurred during calibration of device {device_id}: {e}")
                import traceback
                traceback.print_exc()
                all_results[device_id] = {}
        
        # Print final summary
        self._print_final_summary(all_results)
        
        return all_results
    
    def _print_final_summary(self, all_results: dict):
        """Print final summary"""
        print(f"\n{'='*80}")
        print("Batch calibration final summary")
        print(f"{'='*80}\n")
        
        total_devices = len(all_results)
        print(f"Total devices: {total_devices}")
        
        if total_devices == 0:
            return
        
        # Calculate overall success rate for each task
        from calibration_task import get_all_task_types
        
        task_types = get_all_task_types()
        task_stats = {}
        
        for task_type in task_types:
            success_count = 0
            for device_id, results in all_results.items():
                if task_type in results and results[task_type]:
                    success_count += 1
            task_stats[task_type] = (success_count, total_devices)
        
        print("\nTask success rates:")
        for task_type, (success, total) in task_stats.items():
            percentage = (success / total * 100) if total > 0 else 0
            print(f"  {task_type.value}: {success}/{total} ({percentage:.1f}%)")
        
        print(f"\nResults saved to: {self.result_recorder.csv_file_path}")
        print(f"{'='*80}\n")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Batch calibration tool - automatically calibrate all calibration tasks for all devices"
    )
    
    parser.add_argument(
        "--device-ids",
        type=str,
        nargs="+",
        help="List of device IDs to calibrate (if not specified, auto-discover)"
    )
    
    parser.add_argument(
        "--data-root",
        type=str,
        default="/data",
        help="Data root directory (default: /data)"
    )
    
    parser.add_argument(
        "--scripts-dir",
        type=str,
        default=None,
        help="Script directory path (default: directory where current script is located)"
    )
    
    parser.add_argument(
        "--csv-file",
        type=str,
        default="/tmp/calibration_results.csv",
        help="CSV result file path (default: /tmp/calibration_results.csv)"
    )
    
    parser.add_argument(
        "--s3-config",
        type=str,
        default=None,
        help="S3 configuration file path (optional)"
    )
    
    parser.add_argument(
        "--api-url",
        type=str,
        default=None,
        help="API base URL for results reporting (optional, can also use CALIBRATION_API_URL env var, default: http://127.0.0.1:6001)"
    )
    
    parser.add_argument(
        "--disable-api-reporting",
        action="store_true",
        help="Disable API results reporting"
    )

    # Get valid task names for choices
    valid_tasks = [t.value for t in CalibrationTaskType]
    parser.add_argument(
        "--tasks",
        type=str,
        nargs="+",
        choices=valid_tasks,
        help=f"List of specific tasks to run (default: all). Valid tasks: {', '.join(valid_tasks)}"
    )

    args = parser.parse_args()
    
    # Determine script directory
    if args.scripts_dir is None:
        args.scripts_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Create batch calibration manager
    manager = BatchCalibrationManager(
        scripts_dir=args.scripts_dir,
        csv_file_path=args.csv_file,
        s3_config_file=args.s3_config,
        api_base_url=args.api_url,
        enable_api_reporting=not args.disable_api_reporting
    )
    
    # Parse selected tasks
    selected_tasks = None
    if args.tasks:
        selected_tasks = [CalibrationTaskType(t) for t in args.tasks]
        print(f"Selected tasks: {', '.join(t.value for t in selected_tasks)}")

    # Execute batch calibration
    results = manager.calibrate_devices(
        device_ids=args.device_ids,
        auto_discover=(args.device_ids is None),
        selected_tasks=selected_tasks
    )
    
    # Return exit code
    if results:
        # Check if there are any failed devices
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

