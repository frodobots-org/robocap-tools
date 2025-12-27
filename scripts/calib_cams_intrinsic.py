#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera intrinsic calibration script.
Creates rosbag from database files and runs kalibr calibration.
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
from rosbag_helper import RosbagHelper

# Import calibration result handler
try:
    from calibration_result_handler import CalibrationStatus
except ImportError:
    CalibrationStatus = None


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Camera intrinsic calibration script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s front                              # Calibrate front cameras (left + right)
  %(prog)s eye --device-id faf2a598869ccfc8    # Calibrate eye cameras with device ID
  %(prog)s left                                # Calibrate left camera (uses ROBOCAP_DEVICE_ID env var)
  %(prog)s right --device-id faf2a598869ccfc8   # Calibrate right camera with device ID
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
        '--timeout',
        type=int,
        default=600,
        help='Kalibr calibration timeout in seconds (default: 600 = 10 minutes)'
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
    CalibrationLogger.section("Camera Intrinsic Calibration")
    CalibrationLogger.info(f"Mode: {args.mode}")
    CalibrationLogger.info(f"IMU source rate: {args.imu_rate} Hz")
    CalibrationLogger.info(f"Kalibr timeout: {args.timeout} seconds")
    
    # Get configuration
    config = ConfigHelper.get_camera_intrinsic_config(args.mode)
    
    if config is None:
        CalibrationLogger.error(f"Invalid mode: {args.mode}")
        return 1
    
    # Verify input directory
    if not os.path.isdir(config.input_dir):
        CalibrationLogger.error(f"Input directory does not exist: {config.input_dir}")
        return 1
    
    # Step 1: Create rosbag
    rosbag_creator = RosbagCreator(script_dir)
    if not rosbag_creator.create(
        config.input_dir,
        config.output_rosbag,
        imu_src_rate=args.imu_rate,
        video_rate=5.0  # 相机内参标定使用5fps
    ):
        CalibrationLogger.error("Failed to create rosbag. Aborting.")
        return 1
    
    # Step 2: Extract camera topics from rosbag
    CalibrationLogger.section("Extracting camera topics from rosbag...")
    camera_topics = RosbagHelper.get_camera_topics(config.output_rosbag)
    
    if not camera_topics:
        CalibrationLogger.error("No camera topics found in rosbag")
        return 1
    
    CalibrationLogger.info(f"Found {len(camera_topics)} camera topic(s):")
    for topic in camera_topics:
        CalibrationLogger.info(f"  - {topic}")
    
    # Step 3: Run kalibr calibration
    kalibr_executor = KalibrExecutor()
    calibration_success = kalibr_executor.run_intrinsic_calibration(
        config.output_rosbag,
        camera_topics,
        timeout=args.timeout
    )
    
    # Send result via callback
    if result_handler is not None:
        status = CalibrationStatus.SUCCESS if calibration_success else CalibrationStatus.FAILED
        error_message = None if calibration_success else "Kalibr calibration failed"
        result_handler.handle_camera_intrinsic_result(
            device_id,
            args.mode,
            status,
            rosbag_file=config.output_rosbag if calibration_success else None,
            error_message=error_message
        )
    
    if not calibration_success:
        CalibrationLogger.error("Kalibr calibration failed. Aborting.")
        return 1
    
    # Print summary
    CalibrationLogger.section("Camera Intrinsic Calibration Completed Successfully!")
    CalibrationLogger.info(f"Mode: {args.mode}")
    CalibrationLogger.info(f"Rosbag: {config.output_rosbag}")
    CalibrationLogger.info(f"Camera topics: {', '.join(camera_topics)}")
    
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
