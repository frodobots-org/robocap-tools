#!/usr/bin/env python3
"""
Background calibration runner
Downloads data from S3, runs calibration, uploads results
"""

import os
import sys
import threading
import logging
from datetime import datetime

# Add paths for imports
sys.path.insert(0, "/robocap-tools/scripts")
sys.path.insert(0, "/robocap-tools/s3sdk")

import database

# Set up logging
LOG_DIR = "/data/logs"
os.makedirs(LOG_DIR, exist_ok=True)


def setup_job_logger(job_id: int, device_id: str) -> logging.Logger:
    """Create a logger that writes to a job-specific log file"""
    timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
    log_file = f"{LOG_DIR}/job_{job_id}_{device_id}_{timestamp}.log"

    logger = logging.getLogger(f"job_{job_id}")
    logger.setLevel(logging.DEBUG)

    # Clear existing handlers
    logger.handlers = []

    # File handler
    fh = logging.FileHandler(log_file)
    fh.setLevel(logging.DEBUG)

    # Console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)

    # Format
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)

    logger.info(f"Log file: {log_file}")
    return logger

# S3 config path (can be overridden via environment)
S3_CONFIG_PATH = os.environ.get("S3_CONFIG_PATH", "/robocap-tools/s3sdk/s3_config.json")


def get_s3_sdk():
    """Initialize S3 SDK from config file"""
    from s3_sdk import S3SDK
    from config_reader import load_s3_config

    if not os.path.exists(S3_CONFIG_PATH):
        raise FileNotFoundError(f"S3 config not found: {S3_CONFIG_PATH}")

    config = load_s3_config(S3_CONFIG_PATH)
    return S3SDK(config)


def get_s3_uploader():
    """Initialize S3 uploader for result upload"""
    from s3_uploader import S3Uploader

    if not os.path.exists(S3_CONFIG_PATH):
        return None

    return S3Uploader(S3_CONFIG_PATH)


def download_device_data(device_id: str, logger=None) -> bool:
    """
    Download device data from S3 to local /data directory.

    Downloads: s3://{bucket}/{device_id}/v1/ -> /data/{device_id}/v1/
    """
    log = logger.info if logger else print

    log(f"[S3] Downloading data for device {device_id}...")

    try:
        sdk = get_s3_sdk()

        s3_path = f"{device_id}/v1/"
        local_path = f"/data/{device_id}/v1/"

        # Check if data exists in S3 (use list_files for recursive check)
        files = sdk.list_files(s3_path)
        if not files:
            log(f"[S3] No data found in S3 at {s3_path}")
            return False
        log(f"[S3] Found {len(files)} files in S3")

        # Create local directory
        os.makedirs(local_path, exist_ok=True)

        # Download all data
        results = sdk.download_folder(s3_path, local_path)

        success_count = sum(1 for _, _, success in results if success)
        total_count = len(results)

        log(f"[S3] Downloaded {success_count}/{total_count} files")

        return success_count > 0

    except Exception as e:
        if logger:
            logger.error(f"[S3] Download failed: {e}")
        else:
            print(f"[S3] Download failed: {e}")
        return False


def run_calibration(job_id: int, device_id: str):
    """
    Run calibration for a device in a background thread.
    Downloads data from S3, runs calibration, uploads results.
    """
    thread = threading.Thread(
        target=_run_calibration_thread,
        args=(job_id, device_id),
        daemon=True
    )
    thread.start()
    return thread


def _run_calibration_thread(job_id: int, device_id: str):
    """Background thread that runs calibration"""
    import subprocess

    logger = setup_job_logger(job_id, device_id)

    try:
        logger.info(f"Starting calibration for device {device_id}")

        # Step 1: Download data from S3
        database.update_job(job_id, current_task="downloading_from_s3")

        if not download_device_data(device_id, logger):
            database.update_job(
                job_id,
                status="failed",
                error_message="Failed to download data from S3",
                completed=True
            )
            return

        # Step 2: Run calibration as subprocess to capture all output
        database.update_job(job_id, current_task="running_calibration")

        # Build the calibration command
        cmd = f"""
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
source /catkin_ws_imu/devel/setup.bash
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
cd /robocap-tools/scripts
python3 -c "
import sys
sys.path.insert(0, '/robocap-tools/scripts')
sys.path.insert(0, '/robocap-tools/s3sdk')
from device_calibration_manager import DeviceCalibrationManager
from s3_uploader import S3Uploader

s3_uploader = S3Uploader('{S3_CONFIG_PATH}')
manager = DeviceCalibrationManager(
    device_id='{device_id}',
    scripts_dir='/robocap-tools/scripts',
    result_recorder=None,
    s3_uploader=s3_uploader
)
results = manager.calibrate_all()
success = sum(1 for v in results.values() if v)
total = len(results)
print(f'CALIBRATION_RESULT:{{success}}/{{total}}')
failed = [k.value for k, v in results.items() if not v]
if failed:
    print(f'FAILED_TASKS:{{\\",\\".join(failed)}}')
"
"""
        logger.info("Running calibration subprocess...")

        process = subprocess.Popen(
            cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        # Stream output to logger
        calibration_result = None
        failed_tasks = None

        for line in process.stdout:
            line = line.rstrip()
            logger.info(line)

            # Parse result markers
            if line.startswith("CALIBRATION_RESULT:"):
                calibration_result = line.split(":", 1)[1]
            elif line.startswith("FAILED_TASKS:"):
                failed_tasks = line.split(":", 1)[1]

        return_code = process.wait()
        logger.info(f"Calibration process exited with code {return_code}")

        # Update job status based on results
        if return_code == 0 and calibration_result:
            success, total = calibration_result.split("/")
            if success == total:
                database.update_job(
                    job_id,
                    status="completed",
                    current_task=None,
                    completed=True
                )
                logger.info(f"Calibration completed successfully: {calibration_result}")
            else:
                database.update_job(
                    job_id,
                    status="completed_with_errors",
                    current_task=None,
                    error_message=f"Failed tasks: {failed_tasks}" if failed_tasks else None,
                    completed=True
                )
                logger.info(f"Calibration completed with errors: {calibration_result}")
        else:
            database.update_job(
                job_id,
                status="failed",
                error_message=f"Process exited with code {return_code}",
                completed=True
            )
            logger.error(f"Calibration failed with exit code {return_code}")

    except Exception as e:
        error_msg = str(e)
        logger.error(f"Error running calibration: {error_msg}")
        import traceback
        logger.error(traceback.format_exc())
        database.update_job(
            job_id,
            status="failed",
            error_message=error_msg,
            completed=True
        )
