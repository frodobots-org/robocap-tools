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
import subprocess
import time
import sys
import signal
from pathlib import Path

# Add script directory to path to ensure robocap_env can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import environment variables
import robocap_env


def run_command(cmd, shell=True, check=True, background=False):
    """
    Run a shell command.
    
    Args:
        cmd: Command string or list
        shell: Whether to use shell
        check: Whether to check return code
        background: Whether to run in background
        
    Returns:
        subprocess.Popen or CompletedProcess
    """
    if background:
        return subprocess.Popen(
            cmd,
            shell=shell,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
    else:
        return subprocess.run(
            cmd,
            shell=shell,
            check=check
        )


def create_rosbag_from_db():
    """
    Create rosbag file from database files using create_rosbag_from_db.py.
    """
    print("=" * 80)
    print("Step 1: Creating rosbag from database files...")
    print("=" * 80)
    
    input_dir = robocap_env.DATASET_IMUS_INTRINSIC_DIR
    output_bag = robocap_env.ROSBAG_FILE_IMUS_INTRINSIC
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_bag)
    os.makedirs(output_dir, exist_ok=True)
    
    # Check if input directory exists
    if not os.path.isdir(input_dir):
        print(f"Error: Input directory does not exist: {input_dir}")
        sys.exit(1)
    
    # IMU source rate is typically 200 Hz (from merge_multi_imu_data.py usage)
    # We'll use 200 Hz as source rate and optionally downsample if needed
    # Get the script directory to use relative paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cmd = [
        sys.executable,
        os.path.join(script_dir, "create_rosbag_from_db.py"),
        input_dir,
        "-ir_src", "200",
        "-o", output_bag
    ]
    
    print(f"Input directory: {input_dir}")
    print(f"Output rosbag: {output_bag}")
    print(f"Running: {' '.join(cmd)}")
    
    try:
        subprocess.run(cmd, check=True)
        print(f"Rosbag created successfully: {output_bag}")
    except subprocess.CalledProcessError as e:
        print(f"Error: Failed to create rosbag: {e}")
        sys.exit(1)


def play_rosbag():
    """
    Play rosbag at 60x speed in background without logging.
    """
    print("\n" + "=" * 80)
    print("Step 2: Playing rosbag at 60x speed...")
    print("=" * 80)
    
    bag_file = robocap_env.ROSBAG_FILE_IMUS_INTRINSIC
    
    if not os.path.exists(bag_file):
        print(f"Error: Rosbag file does not exist: {bag_file}")
        sys.exit(1)
    
    # Play rosbag at 60x speed, no logging, in background
    # Use preexec_fn to create a new process group for easier cleanup
    cmd = f"rosbag play -r 60 {bag_file}"
    
    print(f"Playing rosbag: {bag_file}")
    print(f"Rate: 60x")
    print("Running in background...")
    
    # Create process with new process group
    try:
        import signal
        process = subprocess.Popen(
            ["/bin/bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # Create new process group
        )
    except AttributeError:
        # os.setsid might not be available on all systems, fallback to regular Popen
        process = subprocess.Popen(
            ["/bin/bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
    
    print(f"Rosbag playback started (PID: {process.pid})")
    
    return process


def stop_rosbag_completely(rosbag_process):
    """
    Completely stop rosbag play process and all related processes.
    
    Args:
        rosbag_process: subprocess.Popen object of rosbag play
    """
    if rosbag_process is None:
        return
    
    print("\nStopping rosbag playback completely...")
    
    try:
        import signal
        
        # Try to kill the entire process group if we created one
        try:
            if rosbag_process.poll() is None:
                pgid = os.getpgid(rosbag_process.pid)
                print(f"  Killing process group {pgid} (PID: {rosbag_process.pid})...")
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(2)
                
                # Check if process group still exists
                try:
                    os.getpgid(rosbag_process.pid)
                    # Still exists, force kill
                    print(f"  Force killing process group {pgid}...")
                    os.killpg(pgid, signal.SIGKILL)
                    time.sleep(1)
                except ProcessLookupError:
                    # Process group already gone, good
                    pass
        except (ProcessLookupError, AttributeError, OSError):
            # Fallback to regular process termination
            if rosbag_process.poll() is None:
                print(f"  Terminating rosbag process (PID: {rosbag_process.pid})...")
                rosbag_process.terminate()
                time.sleep(2)
                
                # If still running, force kill
                if rosbag_process.poll() is None:
                    print(f"  Force killing rosbag process (PID: {rosbag_process.pid})...")
                    rosbag_process.kill()
                    time.sleep(1)
    except Exception as e:
        print(f"  Warning: Error stopping rosbag process: {e}")
    
    # Also kill any remaining rosbag processes using pkill
    # This ensures all rosbag play processes are stopped, including child processes
    try:
        print("  Killing all remaining rosbag play processes...")
        subprocess.run(
            ["pkill", "-9", "-f", "rosbag play"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3
        )
        time.sleep(1)
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
        # pkill might not be available or might fail, try alternative
        pass
    
    # Verify all rosbag processes are stopped
    try:
        result = subprocess.run(
            ["pgrep", "-f", "rosbag play"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=2
        )
        if result.returncode == 0 and result.stdout.strip():
            # There are still rosbag processes running
            pids = result.stdout.decode().strip().split('\n')
            print(f"  Warning: Found {len(pids)} remaining rosbag process(es), force killing...")
            for pid in pids:
                try:
                    pid_int = int(pid.strip())
                    os.kill(pid_int, signal.SIGKILL)  # SIGKILL
                except (ValueError, ProcessLookupError, PermissionError, OSError):
                    pass
            time.sleep(1)
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
        # pgrep might not be available, that's okay
        pass
    
    # Final check
    try:
        result = subprocess.run(
            ["pgrep", "-f", "rosbag play"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=1
        )
        if result.returncode == 0 and result.stdout.strip():
            print(f"  ⚠ Warning: Some rosbag processes may still be running")
        else:
            print("  ✓ All rosbag processes stopped")
    except Exception:
        print("  ✓ Rosbag process stopped")


def run_roslaunch_single_imu(imu_num: int, launch_file: str):
    """
    Run a single roslaunch process for one IMU.
    
    Args:
        imu_num: IMU number (0, 1, or 2)
        launch_file: Path to launch file
        
    Returns:
        subprocess.Popen object
    """
    setup_file = robocap_env.IMU_UTILS_SETUP_FILE
    
    if not os.path.exists(launch_file):
        print(f"Warning: Launch file does not exist: {launch_file}")
        return None
    
    # Change to catkin_ws_imu directory, source setup.bash, and run roslaunch
    # Use the full path to the launch file
    launch_file_name = os.path.basename(launch_file)
    # Use bash -c to properly execute source command
    # Note: roslaunch needs ROS environment, so we must source setup.bash
    cmd = (
        f"cd /catkin_ws_imu && "
        f"source {setup_file} && "
        f"roslaunch imu_utils {launch_file_name}"
    )
    
    print(f"Starting roslaunch for imu{imu_num}...")
    print(f"  Launch file: {launch_file}")
    print(f"  Command: {cmd}")
    
    # Use bash explicitly to ensure source command works
    # Redirect stderr to a log file for debugging
    log_file_path = f"/tmp/roslaunch_imu{imu_num}.log"
    with open(log_file_path, 'w') as log_file:
        process = subprocess.Popen(
            ["/bin/bash", "-c", cmd],
            stdout=log_file,
            stderr=subprocess.STDOUT
        )
    print(f"  Started (PID: {process.pid}, log: {log_file_path})")
    
    return process


def wait_for_roslaunch_completion(process, imu_num: int, timeout=180):
    """
    Wait for a single roslaunch process to complete.
    
    Args:
        process: subprocess.Popen object
        imu_num: IMU number for logging
        timeout: Maximum time to wait in seconds (default: 180 = 3 minutes)
        
    Returns:
        bool: True if process completed successfully, False otherwise
    """
    if process is None:
        return False
    
    print("\n" + "=" * 80)
    print(f"Waiting for roslaunch process (imu{imu_num}) to complete...")
    print(f"Timeout: {timeout} seconds (3 minutes)")
    print("=" * 80)
    
    start_time = time.time()
    
    # Give process a moment to start
    time.sleep(3)
    
    # Check initial status
    print("\nInitial process status:")
    status = "running" if process.poll() is None else f"exited (code: {process.returncode})"
    print(f"  imu{imu_num} (PID: {process.pid}): {status}")
    
    # Check if roslaunch log file exists and has content
    log_file = f"/tmp/roslaunch_imu{imu_num}.log"
    if os.path.exists(log_file):
        try:
            with open(log_file, 'r') as f:
                log_content = f.read()
                if log_content:
                    print(f"    Log file size: {len(log_content)} bytes")
                    # Show last few lines if there are errors
                    if "error" in log_content.lower() or "Error" in log_content:
                        lines = log_content.split('\n')
                        error_lines = [l for l in lines if "error" in l.lower() or "Error" in l]
                        if error_lines:
                            print(f"    Recent errors:")
                            for err_line in error_lines[-3:]:
                                print(f"      {err_line[:100]}")
        except Exception as e:
            print(f"    Could not read log file: {e}")
    
    while True:
        if process.poll() is not None:
            # Process completed
            return_code = process.returncode
            if return_code == 0:
                print(f"\n✓ Roslaunch process (imu{imu_num}) completed successfully!")
                return True
            else:
                print(f"\n✗ Roslaunch process (imu{imu_num}) failed with return code {return_code}")
                return False
        
        elapsed = time.time() - start_time
        if elapsed > timeout:
            print(f"\n✗ Timeout after {timeout} seconds (3 minutes)")
            print(f"Stopping roslaunch process (imu{imu_num})...")
            
            # Stop the process
            if process.poll() is None:
                print(f"  Terminating imu{imu_num} process (PID: {process.pid})...")
                try:
                    process.terminate()
                    # Wait a bit for graceful termination
                    time.sleep(2)
                    if process.poll() is None:
                        # Force kill if still running
                        process.kill()
                        print(f"  Force killed imu{imu_num} process")
                except Exception as e:
                    print(f"  Error stopping imu{imu_num} process: {e}")
            
            print("Process stopped due to timeout")
            return False
        
        time.sleep(2)
        elapsed_int = int(elapsed)
        print(f"Waiting... ({elapsed_int}s / {timeout}s elapsed)", end='\r')
    
    print()


def extract_imu_params(imu_num: int):
    """
    Extract IMU parameters using extract_imu_params.py for a single IMU.
    
    Args:
        imu_num: IMU number (0, 1, or 2)
    """
    print("\n" + "=" * 80)
    print(f"Extracting IMU parameters for imu{imu_num}...")
    print("=" * 80)
    
    input_dir = "/catkin_ws_imu/src/imu_utils/data"
    
    # Input file: imu{n}_imu_param.yaml in the data directory
    input_file = os.path.join(input_dir, f"imu{imu_num}_imu_param.yaml")
    
    # Output file: use environment variable based on IMU number
    if imu_num == 0:
        output_file = robocap_env.YAML_FILE_IMU_MID_0
    elif imu_num == 1:
        output_file = robocap_env.YAML_FILE_IMU_RIGHT_1
    elif imu_num == 2:
        output_file = robocap_env.YAML_FILE_IMU_LEFT_2
    else:
        raise ValueError(f"Invalid IMU number: {imu_num}. Must be 0, 1, or 2.")
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_file)
    os.makedirs(output_dir, exist_ok=True)
    
    # Check if input file exists
    if not os.path.exists(input_file):
        print(f"  ✗ Error: Input file does not exist: {input_file}")
        raise FileNotFoundError(f"Input file not found: {input_file}")
    
    # Get the script directory to use relative paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    cmd = [
        sys.executable,
        os.path.join(script_dir, "extract_imu_params.py"),
        "-i", input_file,
        "-o", output_file
    ]
    
    print(f"Running: {' '.join(cmd)}")
    print(f"  Input: {input_file}")
    print(f"  Output: {output_file}")
    
    try:
        subprocess.run(cmd, check=True)
        
        # Check if output file was created
        if os.path.exists(output_file):
            print(f"  ✓ Output file created: {output_file}")
        else:
            print(f"  ✗ Warning: Expected output file not found: {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"  ✗ Error: Failed to extract parameters for imu{imu_num}: {e}")
        raise


def process_single_imu(imu_num: int, launch_file: str):
    """
    Process a single IMU: play rosbag, run roslaunch, wait, stop rosbag, extract parameters.
    
    Args:
        imu_num: IMU number (0, 1, or 2)
        launch_file: Path to launch file
        
    Returns:
        bool: True if successful, False otherwise
    """
    print("\n" + "=" * 80)
    print(f"Processing IMU {imu_num}")
    print("=" * 80)
    
    # Step 1: Play rosbag at 60x speed in background
    rosbag_process = play_rosbag()
    
    # Step 2: Run roslaunch process
    roslaunch_process = run_roslaunch_single_imu(imu_num, launch_file)
    
    if roslaunch_process is None:
        # Stop rosbag if launch failed
        stop_rosbag_completely(rosbag_process)
        return False
    
    # Step 3: Wait for roslaunch process to complete
    success = wait_for_roslaunch_completion(roslaunch_process, imu_num)
    
    # Step 4: Stop rosbag playback completely (always stop after each IMU)
    stop_rosbag_completely(rosbag_process)
    
    if not success:
        print(f"\n✗ Roslaunch process (imu{imu_num}) failed or timed out!")
        print("Cannot proceed with parameter extraction.")
        return False
    
    # Step 5: Extract IMU parameters
    try:
        extract_imu_params(imu_num)
        print(f"\n✓ IMU {imu_num} processing completed successfully!")
        return True
    except Exception as e:
        print(f"\n✗ Failed to extract parameters for imu{imu_num}: {e}")
        return False


def main():
    """Main function."""
    print("=" * 80)
    print("IMU Intrinsic Calibration")
    print("Processing IMUs sequentially (one at a time)")
    print("=" * 80)
    
    try:
        # Step 1: Create rosbag from database files
        create_rosbag_from_db()
        
        # Step 2: Process each IMU sequentially
        launch_files = [
            robocap_env.LAUNCH_FILE_IMU_MID_0,
            robocap_env.LAUNCH_FILE_IMU_RIGHT_1,
            robocap_env.LAUNCH_FILE_IMU_LEFT_2
        ]
        imu_numbers = [0, 1, 2]
        
        results = []
        for imu_num, launch_file in zip(imu_numbers, launch_files):
            success = process_single_imu(imu_num, launch_file)
            results.append((imu_num, success))
            
            # Small delay between IMUs
            if imu_num < 2:  # Not the last one
                print("\n" + "-" * 80)
                print(f"Waiting 2 seconds before processing next IMU...")
                time.sleep(2)
        
        # Summary
        print("\n" + "=" * 80)
        print("Calibration Summary")
        print("=" * 80)
        for imu_num, success in results:
            status = "✓ Success" if success else "✗ Failed"
            print(f"  IMU {imu_num}: {status}")
        
        all_success = all(success for _, success in results)
        if all_success:
            print("\n✓ All IMUs calibrated successfully!")
        else:
            failed = [imu_num for imu_num, success in results if not success]
            print(f"\n✗ Some IMUs failed: {', '.join(map(str, failed))}")
            sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

