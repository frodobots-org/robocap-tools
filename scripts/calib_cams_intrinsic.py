#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera intrinsic calibration script.
Creates rosbag from database files and runs kalibr calibration.
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
    Get input directory and output rosbag path based on mode.
    
    Args:
        mode: One of 'front', 'eye', 'left', 'right'
        
    Returns:
        (input_dir, output_rosbag_path) or (None, None) if invalid mode
    """
    if mode == 'front':
        input_dir = robocap_env.DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC
    elif mode == 'eye':
        input_dir = robocap_env.DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_CAM_LR_EYE_INTRINSIC
    elif mode == 'left':
        input_dir = robocap_env.DATASET_IMUS_CAM_L_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_CAM_L_INTRINSIC
    elif mode == 'right':
        input_dir = robocap_env.DATASET_IMUS_CAM_R_EXTRINSIC_DIR
        output_rosbag = robocap_env.ROSBAG_FILE_CAM_R_INTRINSIC
    else:
        return None, None
    
    return input_dir, output_rosbag

def create_rosbag(input_dir: str, output_rosbag: str, imu_src_rate: int = 200) -> bool:
    """
    Create rosbag from database files using create_rosbag_from_db.py.
    
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
    print(f"Video frame rate: 5 fps")
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
    
    # Build command
    cmd = [
        sys.executable,
        create_rosbag_script,
        input_dir,
        "-vr", "5.0",  # Video rate: 5 fps
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

def get_camera_topics_from_rosbag(rosbag_path: str) -> List[str]:
    """
    Extract camera topics (ending with /image_raw) from rosbag.
    
    Args:
        rosbag_path: Path to rosbag file
        
    Returns:
        List of camera topic names
    """
    camera_topics = []
    
    try:
        bag = rosbag.Bag(rosbag_path, 'r')
        topics = bag.get_type_and_topic_info()[1].keys()
        
        for topic in topics:
            if topic.endswith('/image_raw'):
                camera_topics.append(topic)
        
        bag.close()
    except Exception as e:
        print(f"Error: Failed to read rosbag topics: {e}")
        return []
    
    return sorted(camera_topics)

def run_kalibr_calibration(rosbag_path: str, camera_topics: List[str], timeout: int = 600) -> bool:
    """
    Run kalibr camera intrinsic calibration.
    
    Args:
        rosbag_path: Path to rosbag file
        camera_topics: List of camera topic names
        timeout: Timeout in seconds (default: 600 = 10 minutes)
        
    Returns:
        True if successful, False otherwise
    """
    print("=" * 80)
    print("Step 2: Running kalibr camera intrinsic calibration...")
    print("=" * 80)
    print(f"Rosbag: {rosbag_path}")
    print(f"Camera topics: {camera_topics}")
    print(f"Model: pinhole-equi")
    print(f"Target: {KALIBR_TARGET_FILE}")
    print(f"Timeout: {timeout} seconds (10 minutes)")
    
    if not camera_topics:
        print("Error: No camera topics found in rosbag")
        return False
    
    # Verify target file exists
    if not os.path.exists(KALIBR_TARGET_FILE):
        print(f"Error: Kalibr target file not found: {KALIBR_TARGET_FILE}")
        return False
    
    # Source kalibr setup file
    kalibr_setup = robocap_env.KALIBR_SETUP_FILE
    
    if not os.path.exists(kalibr_setup):
        print(f"Error: Kalibr setup file not found: {kalibr_setup}")
        return False
    
    # Build kalibr command
    # Format: kalibr_calibrate_cameras --bag [filename.bag] --topics [TOPIC_0 ... TOPIC_N] 
    #        --models [MODEL_0 ... MODEL_N] --target [target.yaml]
    models = ['pinhole-equi'] * len(camera_topics)
    
    kalibr_cmd = [
        'rosrun kalibr kalibr_calibrate_cameras',
        '--bag', rosbag_path,
        '--topics'] + camera_topics + [
        '--models'] + models + [
        '--target', KALIBR_TARGET_FILE,
        '--dont-show-report'
    ]
    
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
        description="Camera intrinsic calibration script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s front   # Calibrate front cameras (left + right)
  %(prog)s eye     # Calibrate eye cameras (left + right)
  %(prog)s left    # Calibrate left camera
  %(prog)s right   # Calibrate right camera
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
    
    args = parser.parse_args()
    
    print("=" * 80)
    print("Camera Intrinsic Calibration")
    print("=" * 80)
    print(f"Mode: {args.mode}")
    print(f"IMU source rate: {args.imu_rate} Hz")
    print(f"Kalibr timeout: {args.timeout} seconds")
    print("=" * 80)
    
    # Get configuration for mode
    input_dir, output_rosbag = get_config_for_mode(args.mode)
    
    if input_dir is None or output_rosbag is None:
        print(f"Error: Invalid mode: {args.mode}")
        return 1
    
    # Verify input directory exists
    if not os.path.isdir(input_dir):
        print(f"Error: Input directory does not exist: {input_dir}")
        return 1
    
    # Step 1: Create rosbag
    if not create_rosbag(input_dir, output_rosbag, args.imu_rate):
        print("\n✗ Failed to create rosbag. Aborting.")
        return 1
    
    # Step 2: Extract camera topics from rosbag
    print("\n" + "=" * 80)
    print("Extracting camera topics from rosbag...")
    print("=" * 80)
    camera_topics = get_camera_topics_from_rosbag(output_rosbag)
    
    if not camera_topics:
        print("Error: No camera topics found in rosbag")
        return 1
    
    print(f"Found {len(camera_topics)} camera topic(s):")
    for topic in camera_topics:
        print(f"  - {topic}")
    
    # Step 3: Run kalibr calibration
    if not run_kalibr_calibration(output_rosbag, camera_topics, args.timeout):
        print("\n✗ Kalibr calibration failed. Aborting.")
        return 1
    
    print("\n" + "=" * 80)
    print("Camera Intrinsic Calibration Completed Successfully!")
    print("=" * 80)
    print(f"Mode: {args.mode}")
    print(f"Rosbag: {output_rosbag}")
    print(f"Camera topics: {', '.join(camera_topics)}")
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

