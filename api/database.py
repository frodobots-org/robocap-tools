#!/usr/bin/env python3
"""
SQLite database helper for calibration job tracking
"""

import sqlite3
import threading
from datetime import datetime
from typing import Optional, List, Dict, Any

DB_PATH = "/data/calibration_jobs.db"
_local = threading.local()


def get_connection() -> sqlite3.Connection:
    """Get thread-local database connection"""
    if not hasattr(_local, "connection"):
        _local.connection = sqlite3.connect(DB_PATH, check_same_thread=False)
        _local.connection.row_factory = sqlite3.Row
    return _local.connection


def create_tables():
    """Create database tables if they don't exist"""
    conn = get_connection()
    conn.execute("""
        CREATE TABLE IF NOT EXISTS jobs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            status TEXT NOT NULL DEFAULT 'pending',
            current_task TEXT,
            started_at TEXT,
            completed_at TEXT,
            error_message TEXT,
            created_at TEXT NOT NULL
        )
    """)
    conn.commit()


def create_job(device_id: str) -> int:
    """Create a new calibration job"""
    conn = get_connection()
    cursor = conn.execute(
        """
        INSERT INTO jobs (device_id, status, created_at, started_at)
        VALUES (?, 'running', ?, ?)
        """,
        (device_id, datetime.utcnow().isoformat(), datetime.utcnow().isoformat())
    )
    conn.commit()
    return cursor.lastrowid


def update_job(
    job_id: int,
    status: Optional[str] = None,
    current_task: Optional[str] = None,
    error_message: Optional[str] = None,
    completed: bool = False
):
    """Update job status"""
    conn = get_connection()
    updates = []
    params = []

    if status is not None:
        updates.append("status = ?")
        params.append(status)

    if current_task is not None:
        updates.append("current_task = ?")
        params.append(current_task)

    if error_message is not None:
        updates.append("error_message = ?")
        params.append(error_message)

    if completed:
        updates.append("completed_at = ?")
        params.append(datetime.utcnow().isoformat())

    if updates:
        params.append(job_id)
        conn.execute(
            f"UPDATE jobs SET {', '.join(updates)} WHERE id = ?",
            params
        )
        conn.commit()


def get_job(job_id: int) -> Optional[Dict[str, Any]]:
    """Get job by ID"""
    conn = get_connection()
    cursor = conn.execute("SELECT * FROM jobs WHERE id = ?", (job_id,))
    row = cursor.fetchone()
    return dict(row) if row else None


def get_current_job() -> Optional[Dict[str, Any]]:
    """Get currently running job"""
    conn = get_connection()
    cursor = conn.execute(
        "SELECT * FROM jobs WHERE status = 'running' ORDER BY id DESC LIMIT 1"
    )
    row = cursor.fetchone()
    return dict(row) if row else None


def get_recent_jobs(limit: int = 10) -> List[Dict[str, Any]]:
    """Get recent jobs"""
    conn = get_connection()
    cursor = conn.execute(
        "SELECT * FROM jobs ORDER BY id DESC LIMIT ?",
        (limit,)
    )
    return [dict(row) for row in cursor.fetchall()]


def has_running_job() -> bool:
    """Check if there's a running job"""
    return get_current_job() is not None
