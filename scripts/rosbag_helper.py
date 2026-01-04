#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rosbag helper utilities
Provides rosbag-related helper functions
"""

import os
import rosbag
from typing import List

from calibration_common import CalibrationLogger


class RosbagHelper:
    """Rosbag helper utility class"""
    
    @staticmethod
    def get_camera_topics(rosbag_path: str) -> List[str]:
        """
        Extract camera topics from rosbag (ending with /image_raw)
        
        Args:
            rosbag_path: rosbag file path
            
        Returns:
            List of camera topics
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
            CalibrationLogger.error(f"Failed to read rosbag topics: {e}")
            return []
        
        return sorted(camera_topics)
    
    @staticmethod
    def play_rosbag(bag_file: str, rate: float = 60.0) -> 'subprocess.Popen':
        """
        Play rosbag (run in background)
        
        Args:
            bag_file: rosbag file path
            rate: Playback rate (default 60x)
            
        Returns:
            subprocess.Popen object
        """
        import subprocess
        
        CalibrationLogger.step(2, f"Playing rosbag at {rate}x speed...")
        CalibrationLogger.info(f"Playing rosbag: {bag_file}")
        CalibrationLogger.info(f"Rate: {rate}x")
        CalibrationLogger.info("Running in background...")
        
        cmd = f"rosbag play -r {rate} {bag_file}"
        
        try:
            import signal
            process = subprocess.Popen(
                ["/bin/bash", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
        except AttributeError:
            process = subprocess.Popen(
                ["/bin/bash", "-c", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        
        CalibrationLogger.info(f"Rosbag playback started (PID: {process.pid})")
        return process
    
    @staticmethod
    def stop_rosbag(rosbag_process) -> None:
        """
        Completely stop rosbag playback process
        
        Args:
            rosbag_process: subprocess.Popen object
        """
        if rosbag_process is None:
            return
        
        CalibrationLogger.info("\nStopping rosbag playback completely...")
        
        try:
            import signal
            import time
            import subprocess
            
            # Try to terminate process group
            try:
                if rosbag_process.poll() is None:
                    pgid = os.getpgid(rosbag_process.pid)
                    CalibrationLogger.info(f"  Killing process group {pgid} (PID: {rosbag_process.pid})...")
                    os.killpg(pgid, signal.SIGTERM)
                    time.sleep(2)
                    
                    try:
                        os.getpgid(rosbag_process.pid)
                        CalibrationLogger.info(f"  Force killing process group {pgid}...")
                        os.killpg(pgid, signal.SIGKILL)
                        time.sleep(1)
                    except ProcessLookupError:
                        pass
            except (ProcessLookupError, AttributeError, OSError):
                if rosbag_process.poll() is None:
                    CalibrationLogger.info(f"  Terminating rosbag process (PID: {rosbag_process.pid})...")
                    rosbag_process.terminate()
                    time.sleep(2)
                    
                    if rosbag_process.poll() is None:
                        CalibrationLogger.info(f"  Force killing rosbag process (PID: {rosbag_process.pid})...")
                        rosbag_process.kill()
                        time.sleep(1)
        except Exception as e:
            CalibrationLogger.warning(f"Error stopping rosbag process: {e}")
        
        # Use pkill to clean up all rosbag processes
        try:
            CalibrationLogger.info("  Killing all remaining rosbag play processes...")
            subprocess.run(
                ["pkill", "-9", "-f", "rosbag play"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            time.sleep(1)
        except Exception:
            pass
        
        # Final verification
        try:
            result = subprocess.run(
                ["pgrep", "-f", "rosbag play"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=1
            )
            if result.returncode == 0 and result.stdout.strip():
                CalibrationLogger.warning("Some rosbag processes may still be running")
            else:
                CalibrationLogger.success("All rosbag processes stopped")
        except Exception:
            CalibrationLogger.success("Rosbag process stopped")

