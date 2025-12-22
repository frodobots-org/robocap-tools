#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera-IMU extrinsic calibration script.
Creates rosbag from database files and runs kalibr IMU-camera calibration.
"""

import os
import sys
import argparse
import subprocess
import rosbag
import time
from typing import List, Optional

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

# Kalibr target file
KALIBR_TARGET_FILE = robocap_env.TARGET_FILE

# IMU YAML files
IMU_YAML_FILES = [
    robocap_env.YAML_FILE_IMU_MID_0,
    robocap_env.YAML_FILE_IMU_RIGHT_1,
    robocap_env.YAML_FILE_IMU_LEFT_2,
]

def run_command(cmd: List[str], timeout: Optional[int] = None, cwd: Optional[str] = None) -> tuple:
    """
    Run a shell command and return (returncode, stdout, stderr).
    
    Args:
        cmd: Command as list of strings
        timeout: Timeout in seconds (None for no timeout)
        cwd: Working directory
        
    Returns:
        (returncode, stdout, stderr)
    """
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=cwd
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", f"Command timed out after {timeout} seconds"
    except Exception as e:
        return -1, "", str(e)

def get_config_for_mode(mode: str) -> tuple:
    """
    Get input directory, output rosbag path, and camchain file based on mode.
    
    Args:
        mode: One of 'front', 'eye', 'left', 'right'
        
    Returns:
        (input_dir, output_rosbag_path, camchain_file) or (None, None, None) if invalid mode
    """
    if mode == 'front':
        input_dir = robocap_env.DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC
        camchain_file = robocap_env.CAMCHAIN_FILE_LR_FRONT
    elif mode == 'eye':
        input_dir = robocap_env.DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC
        camchain_file = robocap_env.CAMCHAIN_FILE_LR_EYE
    elif mode == 'left':
        input_dir = robocap_env.DATASET_IMUS_CAM_L_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC
        camchain_file = robocap_env.CAMCHAIN_FILE_L
    elif mode == 'right':
        input_dir = robocap_env.DATASET_IMUS_CAM_R_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC
        camchain_file = robocap_env.CAMCHAIN_FILE_R
    else:
        return None, None, None
    
    return input_dir, output_rosbag, camchain_file

def create_rosbag(input_dir: str, output_rosbag: str, imu_src_rate: int = 500) -> bool:
    """
    Create rosbag from database files using create_rosbag_from_db.py.
    Note: Video fps is not downsampled (no -vr parameter).
    
    Args:
        input_dir: Input directory containing IMU database and video files
        output_rosbag: Output rosbag file path
        imu_src_rate: IMU source sampling rate (Hz)
        
    Returns:
        True if successful, False otherwise
    """
    print("=" * 80)
    print("Step 1: Creating rosbag from database files...")
    print("=" * 80)
    print(f"Input directory: {input_dir}")
    print(f"Output rosbag: {output_rosbag}")
    print(f"Video frame rate: Original (no downsampling)")
    print(f"IMU source rate: {imu_src_rate} Hz")
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_rosbag)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
        print(f"Created output directory: {output_dir}")
    
    # Get path to create_rosbag_from_db.py script
    create_rosbag_script = os.path.join(script_dir, "create_rosbag_from_db.py")
    
    if not os.path.exists(create_rosbag_script):
        print(f"Error: Script not found: {create_rosbag_script}")
        return False
    
    # Build command (no -vr parameter for video rate, use original fps)
    cmd = [
        sys.executable,
        create_rosbag_script,
        input_dir,
        "-ir_src", str(imu_src_rate),  # IMU source rate
        "-o", output_rosbag  # Output rosbag path
    ]
    
    print(f"\nRunning command: {' '.join(cmd)}")
    
    returncode, stdout, stderr = run_command(cmd)
    
    if returncode != 0:
        print(f"Error: Failed to create rosbag (return code: {returncode})")
        if stderr:
            print(f"Stderr: {stderr}")
        if stdout:
            print(f"Stdout: {stdout}")
        return False
    
    if stdout:
        print(stdout)
    
    # Verify rosbag was created
    if not os.path.exists(output_rosbag):
        print(f"Error: Rosbag file was not created: {output_rosbag}")
        return False
    
    print(f"\n✓ Rosbag created successfully: {output_rosbag}")
    return True

def verify_imu_yaml_files(imu_yaml_files: List[str]) -> bool:
    """
    Verify that all IMU YAML files exist.
    
    Args:
        imu_yaml_files: List of IMU YAML file paths
        
    Returns:
        True if all files exist, False otherwise
    """
    print("\n" + "=" * 80)
    print("Verifying IMU YAML files...")
    print("=" * 80)
    
    all_exist = True
    for imu_file in imu_yaml_files:
        if os.path.exists(imu_file):
            print(f"✓ Found: {imu_file}")
        else:
            print(f"✗ Missing: {imu_file}")
            all_exist = False
    
    if not all_exist:
        print("\nError: Some IMU YAML files are missing. Please run IMU intrinsic calibration first.")
        return False
    
    return True

def run_kalibr_extrinsic_calibration(
    rosbag_path: str,
    camchain_file: str,
    imu_yaml_files: List[str],
    imu_models: Optional[List[str]] = None,
    timeout: int = 600
) -> bool:
    """
    Run kalibr IMU-camera extrinsic calibration.
    
    Args:
        rosbag_path: Path to rosbag file
        camchain_file: Path to camera chain YAML file
        imu_yaml_files: List of IMU YAML file paths
        imu_models: Optional list of IMU models (can be omitted)
        timeout: Timeout in seconds (default: 900 = 15 minutes)
        
    Returns:
        True if successful, False otherwise
    """
    print("=" * 80)
    print("Step 2: Running kalibr IMU-camera extrinsic calibration...")
    print("=" * 80)
    print(f"Rosbag: {rosbag_path}")
    print(f"Camchain file: {camchain_file}")
    print(f"IMU YAML files: {imu_yaml_files}")
    # Determine IMU models to use
    if imu_models:
        final_imu_models = imu_models
    else:
        # Default: use 'calibrated' for all IMUs
        final_imu_models = ['calibrated'] * len(imu_yaml_files)
    print(f"IMU models: {final_imu_models}")
    print(f"Target: {KALIBR_TARGET_FILE}")
    print(f"Bag time range: 0-60 seconds (first 60 seconds only)")
    print(f"Timeout: {timeout} seconds (15 minutes)")
    
    # Verify files exist
    if not os.path.exists(rosbag_path):
        print(f"Error: Rosbag file not found: {rosbag_path}")
        return False
    
    if not os.path.exists(camchain_file):
        print(f"Error: Camchain file not found: {camchain_file}")
        print("Please run camera intrinsic calibration first.")
        return False
    
    if not os.path.exists(KALIBR_TARGET_FILE):
        print(f"Error: Kalibr target file not found: {KALIBR_TARGET_FILE}")
        return False
    
    # Source kalibr setup file
    kalibr_setup = robocap_env.KALIBR_SETUP_FILE
    
    if not os.path.exists(kalibr_setup):
        print(f"Error: Kalibr setup file not found: {kalibr_setup}")
        return False
    
    # Build kalibr command
    # Format: kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] 
    #        --imu [imu0.yaml ... imuN.yaml] --imu-models [MODEL0 ... MODELN] --target [target.yaml]
    kalibr_cmd = [
        'rosrun kalibr kalibr_calibrate_imu_camera',
        '--perform-synchronization',
        '--bag', rosbag_path,
        '--bag-from-to', '5 65',
        '--timeoffset-padding', '0.1',
        '--cam', camchain_file,
        '--imu'] + imu_yaml_files + [
        '--imu-models',
    ]
    
    # Add IMU models (use provided models or default to 'calibrated' for each IMU)
    if imu_models:
        kalibr_cmd.extend(imu_models)
    else:
        # Default: use 'calibrated' for all IMUs
        kalibr_cmd.extend(['calibrated'] * len(imu_yaml_files))
    
    kalibr_cmd.extend([
        '--target', KALIBR_TARGET_FILE,
        '--dont-show-report'
    ])
    
    # Build full command with source
    full_cmd = f"source {kalibr_setup} && {' '.join(kalibr_cmd)}"
    
    print(f"\nRunning command: {full_cmd}")
    print("-" * 80)
    
    # Run command using bash
    cmd = ['/bin/bash', '-c', full_cmd]
    
    start_time = time.time()
    returncode, stdout, stderr = run_command(cmd, timeout=timeout)
    elapsed_time = time.time() - start_time
    
    print(f"\nKalibr calibration completed in {elapsed_time:.1f} seconds")
    
    if returncode != 0:
        print(f"✗ Kalibr calibration failed (return code: {returncode})")
        if stderr:
            print(f"\nStderr:\n{stderr}")
        if stdout:
            print(f"\nStdout:\n{stdout}")
        return False
    
    print("✓ Kalibr calibration completed successfully")
    if stdout:
        print(f"\nOutput:\n{stdout}")
    
    return True

def main():
    """
    Main function.
    """
    parser = argparse.ArgumentParser(
        description="Camera-IMU extrinsic calibration script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s front                    # Calibrate front cameras with all 3 IMUs
  %(prog)s eye                      # Calibrate eye cameras with all 3 IMUs
  %(prog)s left                     # Calibrate left camera with all 3 IMUs
  %(prog)s right                    # Calibrate right camera with all 3 IMUs
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
        default=500,
        help='IMU source sampling rate (Hz), default: 500'
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
        default=1800,
        help='Kalibr calibration timeout in seconds (default: 900 = 15 minutes)'
    )
    
    args = parser.parse_args()
    
    print("=" * 80)
    print("Camera-IMU Extrinsic Calibration")
    print("=" * 80)
    print(f"Mode: {args.mode}")
    print(f"IMU source rate: {args.imu_rate} Hz")
    print(f"Kalibr timeout: {args.timeout} seconds")
    if args.imu_models:
        print(f"IMU models: {args.imu_models}")
    print("=" * 80)
    
    # Get configuration for mode
    input_dir, output_rosbag, camchain_file = get_config_for_mode(args.mode)
    
    if input_dir is None or output_rosbag is None or camchain_file is None:
        print(f"Error: Invalid mode: {args.mode}")
        return 1
    
    # Verify input directory exists
    if not os.path.isdir(input_dir):
        print(f"Error: Input directory does not exist: {input_dir}")
        return 1
    
    # Verify IMU YAML files exist
    if not verify_imu_yaml_files(IMU_YAML_FILES):
        return 1
    
    # Verify IMU models count matches IMU YAML files count (if provided)
    if args.imu_models and len(args.imu_models) != len(IMU_YAML_FILES):
        print(f"Error: Number of IMU models ({len(args.imu_models)}) does not match "
              f"number of IMU YAML files ({len(IMU_YAML_FILES)})")
        return 1
    
    # Step 1: Create rosbag
    if not create_rosbag(input_dir, output_rosbag, args.imu_rate):
        print("\n✗ Failed to create rosbag. Aborting.")
        return 1
    
    # Step 2: Run kalibr extrinsic calibration
    if not run_kalibr_extrinsic_calibration(
        output_rosbag,
        camchain_file,
        IMU_YAML_FILES,
        args.imu_models,
        args.timeout
    ):
        print("\n✗ Kalibr calibration failed. Aborting.")
        return 1
    
    print("\n" + "=" * 80)
    print("Camera-IMU Extrinsic Calibration Completed Successfully!")
    print("=" * 80)
    print(f"Mode: {args.mode}")
    print(f"Rosbag: {output_rosbag}")
    print(f"Camchain file: {camchain_file}")
    print(f"IMU YAML files: {', '.join(IMU_YAML_FILES)}")
    print("=" * 80)
    
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

