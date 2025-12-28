#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera-IMU extrinsic calibration script.
Creates rosbag from database files and runs kalibr IMU-camera calibration.
"""

import os
import sys
import argparse

# Add script directory to path to ensure robocap_env can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import environment variables
try:
    import robocap_env
except ImportError:
    print("Error: Could not import robocap_env")
    sys.exit(1)

# Import calibration common utilities
from calibration_common import (
    CalibrationLogger,
    ConfigHelper,
    ArgumentParserHelper
)
from rosbag_creator_helper import RosbagCreator
from kalibr_executor import KalibrExecutor

# Import calibration result handler
try:
    from calibration_result_handler import CalibrationStatus
except ImportError:
    CalibrationStatus = None


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Camera-IMU extrinsic calibration script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s front                    # Calibrate front cameras with all 3 IMUs
  %(prog)s eye --device-id faf2a598869ccfc8  # Calibrate eye cameras with device ID
  %(prog)s left                     # Calibrate left camera with all 3 IMUs (uses ROBOCAP_DEVICE_ID env var)
  %(prog)s right --device-id faf2a598869ccfc8  # Calibrate right camera with device ID
  %(prog)s front --imu-models calibrated calibrated calibrated  # Specify IMU models
        """
    )
    parser.add_argument(
        'mode',
        choices=['front', 'eye', 'left', 'right'],
        help='Calibration mode: front (left+right front), eye (left+right eye), left, or right'
    )
    parser.add_argument(
        '--imu-rate',
        type=int,
        default=200,
        help='IMU source sampling rate (Hz), default: 200'
    )
    parser.add_argument(
        '--imu-models',
        nargs='+',
        default=None,
        help='IMU models (e.g., calibrated scale-misalignment). If omitted, uses default. Number must match number of IMU YAML files.'
    )
    parser.add_argument(
        '--timeout',
        type=int,
        default=3600,
        help='Kalibr calibration timeout in seconds (default: 3600 = 60 minutes)'
    )
    ArgumentParserHelper.add_common_args(parser)
    
    args = parser.parse_args()
    
    # Setup device ID
    device_id = ArgumentParserHelper.setup_device_id(args)
    
    # Load callback
    callback_func = ArgumentParserHelper.load_callback(args.callback)
    
    # Create result handler
    result_handler = ArgumentParserHelper.create_result_handler(callback_func)
    
    # Print header
    CalibrationLogger.section("Camera-IMU Extrinsic Calibration")
    CalibrationLogger.info(f"Mode: {args.mode}")
    CalibrationLogger.info(f"IMU source rate: {args.imu_rate} Hz")
    CalibrationLogger.info(f"Kalibr timeout: {args.timeout} seconds")
    if args.imu_models:
        CalibrationLogger.info(f"IMU models: {args.imu_models}")
    
    # Get configuration
    config = ConfigHelper.get_camera_extrinsic_config(args.mode)
    
    if config is None or config.camchain_file is None or config.imu_yaml_files is None:
        CalibrationLogger.error(f"Invalid mode: {args.mode}")
        return 1
    
    # Verify input directory
    if not os.path.isdir(config.input_dir):
        CalibrationLogger.error(f"Input directory does not exist: {config.input_dir}")
        return 1
    
    # Verify IMU models count matches IMU YAML files count (if provided)
    if args.imu_models and len(args.imu_models) != len(config.imu_yaml_files):
        CalibrationLogger.error(
            f"Number of IMU models ({len(args.imu_models)}) does not match "
            f"number of IMU YAML files ({len(config.imu_yaml_files)})"
        )
        return 1
    
    # Step 1: Create rosbag
    rosbag_creator = RosbagCreator(script_dir)
    if not rosbag_creator.create(
        config.input_dir,
        config.output_rosbag,
        imu_src_rate=args.imu_rate,
        video_rate=None  # 外参标定使用原始帧率
    ):
        CalibrationLogger.error("Failed to create rosbag. Aborting.")
        return 1
    
    # Step 2: Run kalibr extrinsic calibration
    kalibr_executor = KalibrExecutor()
    calibration_success = kalibr_executor.run_extrinsic_calibration(
        config.output_rosbag,
        config.camchain_file,
        config.imu_yaml_files,
        imu_models=args.imu_models,
        timeout=args.timeout
    )
    
    # Send result via callback
    if result_handler is not None:
        status = CalibrationStatus.SUCCESS if calibration_success else CalibrationStatus.FAILED
        error_message = None if calibration_success else "Kalibr extrinsic calibration failed"
        result_handler.handle_camera_imu_extrinsic_result(
            device_id,
            args.mode,
            status,
            rosbag_file=config.output_rosbag if calibration_success else None,
            camchain_file=config.camchain_file if calibration_success else None,
            error_message=error_message
        )
    
    if not calibration_success:
        CalibrationLogger.error("Kalibr calibration failed. Aborting.")
        return 1
    
    # Print summary
    CalibrationLogger.section("Camera-IMU Extrinsic Calibration Completed Successfully!")
    CalibrationLogger.info(f"Mode: {args.mode}")
    CalibrationLogger.info(f"Rosbag: {config.output_rosbag}")
    CalibrationLogger.info(f"Camchain file: {config.camchain_file}")
    CalibrationLogger.info(f"IMU YAML files: {', '.join(config.imu_yaml_files)}")
    
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\nUnhandled exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
