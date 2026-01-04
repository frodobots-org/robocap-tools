#!/usr/bin/env python3
"""
calib_imus_intrinsic.py
Calibrate IMU intrinsic parameters by:
1. Creating rosbag from database files
2. Playing rosbag at 60x speed
3. Running three roslaunch processes for imu0, imu1, imu2
4. Extracting IMU parameters to YAML files
"""

import os
import sys
import argparse
import time

# Add script directory to path to ensure robocap_env can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import environment variables
import robocap_env

# Import calibration common utilities
from calibration_common import (
    CalibrationLogger,
    ConfigHelper,
    ArgumentParserHelper
)
from rosbag_creator_helper import RosbagCreator
from imu_calibration_helper import IMUCalibrationHelper

# Import calibration result handler
try:
    from calibration_result_handler import (
        CalibrationResultHandler,
        IMUCalibrationResult,
        CalibrationStatus
    )
except ImportError:
    CalibrationResultHandler = None
    IMUCalibrationResult = None
    CalibrationStatus = None


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="IMU intrinsic calibration script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --device-id faf2a598869ccfc8  # Calibrate IMUs with device ID
  %(prog)s                                # Use ROBOCAP_DEVICE_ID environment variable or default
        """
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
    CalibrationLogger.section("IMU Intrinsic Calibration")
    CalibrationLogger.info("Processing IMUs sequentially (one at a time)")
    
    try:
        # Get configuration
        config = ConfigHelper.get_imu_intrinsic_config()
        
        if config is None:
            CalibrationLogger.error("Failed to get IMU intrinsic configuration")
            return 1
        
        # Verify input directory
        if not os.path.isdir(config.input_dir):
            CalibrationLogger.error(f"Input directory does not exist: {config.input_dir}")
            return 1
        
        # Step 1: Create rosbag from database files
        rosbag_creator = RosbagCreator(script_dir)
        if not rosbag_creator.create(
            config.input_dir,
            config.output_rosbag,
            imu_src_rate=200,
            video_rate=None,
            trim_start_seconds=300,  # Trim first 5 minutes
            trim_end_seconds=300     # Trim last 5 minutes
        ):
            CalibrationLogger.error("Failed to create rosbag. Aborting.")
            return 1
        
        # Step 2: Process each IMU sequentially
        launch_files = [
            robocap_env.LAUNCH_FILE_IMU_MID_0,
            robocap_env.LAUNCH_FILE_IMU_RIGHT_1,
            robocap_env.LAUNCH_FILE_IMU_LEFT_2
        ]
        imu_numbers = [0, 1, 2]
        
        imu_helper = IMUCalibrationHelper()
        imu_calibration_results = []
        
        for imu_num, launch_file in zip(imu_numbers, launch_files):
            success, yaml_file, error_message = imu_helper.process_single_imu(
                imu_num,
                launch_file,
                config.output_rosbag
            )
            
            # Create IMU calibration result
            if CalibrationResultHandler is not None and IMUCalibrationResult is not None:
                status = CalibrationStatus.SUCCESS if success else CalibrationStatus.FAILED
                imu_result = IMUCalibrationResult(
                    imu_number=imu_num,
                    status=status,
                    yaml_file=yaml_file,
                    error_message=error_message
                )
                imu_calibration_results.append(imu_result)
            
            # Small delay between IMUs
            if imu_num < 2:  # Not the last one
                CalibrationLogger.info("\n" + "-" * 80)
                CalibrationLogger.info("Waiting 2 seconds before processing next IMU...")
                time.sleep(2)
        
        # Summary
        CalibrationLogger.section("Calibration Summary")
        if imu_calibration_results:
            for imu_result in imu_calibration_results:
                status = "✓ Success" if imu_result.status == CalibrationStatus.SUCCESS else "✗ Failed"
                CalibrationLogger.info(f"  IMU {imu_result.imu_number}: {status}")
        else:
            # Fallback if result handler is not available
            for imu_num in imu_numbers:
                CalibrationLogger.info(f"  IMU {imu_num}: (status unknown)")
        
        # Send results via callback
        if result_handler is not None and imu_calibration_results:
            result_handler.handle_imu_intrinsic_result(device_id, imu_calibration_results)
        
        # Check if all succeeded
        all_success = all(
            r.status == CalibrationStatus.SUCCESS for r in imu_calibration_results
        ) if imu_calibration_results else False
        
        if all_success:
            CalibrationLogger.success("All IMUs calibrated successfully!")
            return 0
        else:
            failed = [
                r.imu_number for r in imu_calibration_results 
                if r.status != CalibrationStatus.SUCCESS
            ] if imu_calibration_results else []
            CalibrationLogger.error(f"Some IMUs failed: {', '.join(map(str, failed))}")
            return 1
        
    except KeyboardInterrupt:
        CalibrationLogger.error("\n\nInterrupted by user")
        return 1
    except Exception as e:
        CalibrationLogger.error(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
