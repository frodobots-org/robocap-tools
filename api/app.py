#!/usr/bin/env python3
"""
Calibration API Server
HTTP endpoints for triggering and monitoring calibration jobs
"""

import os
from flask import Flask, request, jsonify

import database
import calibration_runner

app = Flask(__name__)


@app.route("/health", methods=["GET"])
def health():
    """Health check endpoint"""
    return jsonify({"status": "ok"})


@app.route("/calibrate", methods=["POST"])
def start_calibration():
    """
    Start a calibration job for a device.

    Request body: {"device_id": "abc123"}
    Returns: {"job_id": 1, "status": "running"}
    """
    data = request.get_json()

    if not data or "device_id" not in data:
        return jsonify({"error": "device_id is required"}), 400

    device_id = data["device_id"].strip()

    if not device_id:
        return jsonify({"error": "device_id cannot be empty"}), 400

    # Check if a job is already running
    if database.has_running_job():
        current = database.get_current_job()
        return jsonify({
            "error": "A calibration job is already running",
            "current_job": current
        }), 409

    # Create job and start calibration (data will be downloaded from S3)
    job_id = database.create_job(device_id)
    calibration_runner.run_calibration(job_id, device_id)

    return jsonify({
        "job_id": job_id,
        "device_id": device_id,
        "status": "running"
    }), 202


@app.route("/status", methods=["GET"])
def get_status():
    """
    Get current server status.

    Returns: {"current_job": {...} or null, "recent_jobs": [...]}
    """
    current = database.get_current_job()
    recent = database.get_recent_jobs(limit=10)

    return jsonify({
        "current_job": current,
        "recent_jobs": recent
    })


@app.route("/jobs/<int:job_id>", methods=["GET"])
def get_job(job_id: int):
    """
    Get specific job status.

    Returns: {"job_id": 1, "device_id": "...", "status": "...", ...}
    """
    job = database.get_job(job_id)

    if not job:
        return jsonify({"error": "Job not found"}), 404

    return jsonify(job)


@app.route("/jobs/<int:job_id>/logs", methods=["GET"])
def get_job_logs(job_id: int):
    """
    Get logs for a specific job.

    Returns: Plain text log content
    """
    import glob

    job = database.get_job(job_id)
    if not job:
        return jsonify({"error": "Job not found"}), 404

    # Find log file for this job
    log_pattern = f"/data/logs/job_{job_id}_*.log"
    log_files = glob.glob(log_pattern)

    if not log_files:
        return jsonify({"error": "No log file found for this job"}), 404

    # Return the most recent log file
    log_file = sorted(log_files)[-1]

    with open(log_file, "r") as f:
        content = f.read()

    return content, 200, {"Content-Type": "text/plain"}


@app.route("/logs", methods=["GET"])
def list_logs():
    """
    List all available log files.
    """
    import glob

    log_files = glob.glob("/data/logs/*.log")
    logs = []

    for log_file in sorted(log_files, reverse=True):
        filename = os.path.basename(log_file)
        size = os.path.getsize(log_file)
        logs.append({"filename": filename, "size": size})

    return jsonify({"logs": logs})


if __name__ == "__main__":
    # Initialize database
    database.create_tables()

    # Run server
    port = int(os.environ.get("PORT", 8080))
    app.run(host="0.0.0.0", port=port, debug=False)
