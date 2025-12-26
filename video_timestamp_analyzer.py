#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplified Video Analyzer - Checks only inter-file continuity
TAG:comment unit is microseconds (μs)
"""

import os
import sys
import re
import cv2
import subprocess
import json
from datetime import datetime
import argparse

class SimpleVideoAnalyzer:
    def __init__(self, expected_fps=30, max_gap_ms=40, max_file_gap_ms=100):
        self.expected_fps = expected_fps
        self.max_gap_ms = max_gap_ms
        self.max_file_gap_ms = max_file_gap_ms
        self.expected_interval_ms = 1000.0 / expected_fps
        
        self.results = []
        self.segment_files = []
        
        # Cross-file continuity check
        self.prev_file_end_time_us = None
        self.prev_filename = None
        self.continuity_errors = []
    
    def extract_segment_number(self, filename):
        """Extract segment number from filename"""
        match = re.search(r'segment(\d+)', filename, re.IGNORECASE)
        if match:
            return int(match.group(1))
        
        match = re.search(r'_(\d+)_right-', filename, re.IGNORECASE)
        if match:
            return int(match.group(1))
        
        numbers = re.findall(r'\d+', filename)
        if numbers:
            return int(numbers[-1])
        
        return None
    
    def find_segment_files(self, directory):
        """Find and sort segment files"""
        print(f"Scanning directory: {directory}")
        
        segment_files = []
        
        for filename in os.listdir(directory):
            if not filename.lower().endswith('.mp4'):
                continue
            
            segment_num = self.extract_segment_number(filename)
            if segment_num is not None:
                filepath = os.path.join(directory, filename)
                segment_files.append((segment_num, filepath, filename))
        
        if not segment_files:
            print("No video files found")
            return False
        
        segment_files.sort(key=lambda x: x[0])
        
        print(f"Found {len(segment_files)} video files:")
        for seg_num, _, filename in segment_files:
            print(f"  [{seg_num:3d}] {filename}")
        
        self.segment_files = segment_files
        return True
    
    def run_command(self, cmd, timeout=5):
        """Command execution function compatible with Python 3.6"""
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                universal_newlines=True
            )
            return result
        except Exception as e:
            return None
    
    def get_timestamp_from_file(self, filepath):
        """Get file start timestamp - unit is microseconds (μs)"""
        try:
            cmd = [
                'ffprobe', '-v', 'quiet',
                '-print_format', 'json',
                '-show_format',
                filepath
            ]
            result = self.run_command(cmd, timeout=3)
            
            if result and result.returncode == 0:
                data = json.loads(result.stdout)
                if 'format' in data and 'tags' in data['format']:
                    tags = data['format']['tags']
                    for tag_key in ['comment', 'Comment', 'COMMENT']:
                        if tag_key in tags:
                            value = tags[tag_key]
                            if isinstance(value, str) and value.isdigit():
                                timestamp_us = int(value)  # Microseconds unit
                                return timestamp_us, True
            
            return 0, False
            
        except Exception as e:
            return 0, False
    
    def analyze_video_frames(self, filepath, seg_num, filename):
        """Analyze video frame intervals and continuity"""
        print(f"\n{'='*60}")
        print(f"Analyzing [{seg_num}] {filename}")
        print(f"{'='*60}")
        
        # Get file start timestamp (microseconds unit)
        timestamp_us, has_timestamp = self.get_timestamp_from_file(filepath)
        
        if has_timestamp:
            print(f"Timestamp: {timestamp_us} μs")
        else:
            print(f"No TAG:comment timestamp found")
            timestamp_us = seg_num * 10000000  # Estimate
        
        # 🔍 Check continuity with previous file
        if self.prev_file_end_time_us is not None:
            # Calculate inter-file gap (microseconds → milliseconds)
            file_gap_us = timestamp_us - self.prev_file_end_time_us
            file_gap_ms = file_gap_us / 1000.0
            
            print(f"\nContinuity check:")
            print(f"  Previous file end: {self.prev_file_end_time_us} μs")
            print(f"  Current file start: {timestamp_us} μs")
            print(f"  Inter-file gap: {file_gap_ms:.1f} ms")
            
            # Check continuity
            if abs(file_gap_ms) > self.max_file_gap_ms:
                if file_gap_ms > 0:
                    error_type = 'Gap between files'
                else:
                    error_type = 'Time overlap between files'
                
                continuity_error = {
                    'type': error_type,
                    'from_file': self.prev_filename,
                    'to_file': filename,
                    'gap_ms': round(file_gap_ms, 1)
                }
                
                print(f"  ⚠️  {error_type}: {abs(file_gap_ms):.1f}ms")
                self.continuity_errors.append(continuity_error)
            else:
                print(f"  ✅ Good continuity")
        
        # Analyze video frames
        result = {
            'segment': seg_num,
            'filename': filename,
            'has_timestamp': has_timestamp,
            'start_time_us': timestamp_us,
            'total_frames': 0,
            'reported_fps': 0,
            'actual_fps': 0,
            'avg_interval_ms': 0,
            'error_count': 0,
            'end_time_us': None,
            'status': 'PENDING'
        }
        
        try:
            cap = cv2.VideoCapture(filepath)
            if not cap.isOpened():
                print(f"Cannot open video file")
                result['status'] = 'ERROR'
                return result
            
            # Get video information
            reported_fps = cap.get(cv2.CAP_PROP_FPS)
            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            
            result['total_frames'] = frame_count
            result['reported_fps'] = reported_fps
            
            if reported_fps > 0:
                file_duration = frame_count / reported_fps
                # Calculate file end time (microseconds)
                result['end_time_us'] = timestamp_us + int(file_duration * 1000000)
                
                print(f"\nVideo information:")
                print(f"  Reported FPS: {reported_fps:.2f} FPS")
                print(f"  Total frames: {frame_count:,}")
                print(f"  Duration: {file_duration:.2f} seconds")
            
            if frame_count == 0:
                print("Warning: Video has 0 frames")
                result['status'] = 'ERROR'
                return result
            
            # Analyze frame intervals
            print(f"\nAnalyzing frame intervals...")
            frame_intervals = []
            prev_time_ms = None
            frame_idx = 0
            error_count = 0
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                current_time_ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                
                if prev_time_ms is not None and current_time_ms > 0:
                    interval_ms = current_time_ms - prev_time_ms
                    frame_intervals.append(interval_ms)
                    
                    # Detect anomalies
                    if interval_ms > self.max_gap_ms:
                        error_count += 1
                
                prev_time_ms = current_time_ms
                frame_idx += 1
                
                # Show progress
                if frame_idx % 100 == 0:
                    progress = (frame_idx / frame_count) * 100
                    sys.stdout.write(f"  Progress: {frame_idx}/{frame_count} frames ({progress:.1f}%)\r")
                    sys.stdout.flush()
            
            cap.release()
            print(f"  Progress: {frame_count}/{frame_count} frames (100.0%)")
            
            # Update previous file information
            if result['end_time_us']:
                self.prev_file_end_time_us = result['end_time_us']
            self.prev_filename = filename
            
            # Calculate statistics
            if frame_intervals:
                result['avg_interval_ms'] = sum(frame_intervals) / len(frame_intervals)
                result['actual_fps'] = 1000.0 / result['avg_interval_ms'] if result['avg_interval_ms'] > 0 else 0
                result['error_count'] = error_count
                
                # Quality assessment
                analyzed_frames = max(frame_count - 1, 1)
                error_rate = (error_count / analyzed_frames) * 100
                
                if error_count == 0:
                    result['status'] = 'PASS'
                elif error_rate < 1.0:
                    result['status'] = 'WARNING'
                else:
                    result['status'] = 'FAIL'
                
                print(f"\nFrame rate analysis results:")
                print(f"  Actual FPS: {result['actual_fps']:.2f} FPS")
                print(f"  Average frame interval: {result['avg_interval_ms']:.2f} ms")
                print(f"  Anomalous frames: {error_count} ({error_rate:.2f}%)")
                print(f"  Status: {result['status']}")
            
            else:
                result['status'] = 'ERROR'
            
            return result
            
        except Exception as e:
            print(f"Video analysis error: {e}")
            result['status'] = f'ERROR'
            return result
    
    def analyze_all_segments(self, directory):
        """Analyze all segment files"""
        print(f"{'='*60}")
        print(f"Video Analysis - Inter-file Continuity Check")
        print(f"Directory: {directory}")
        print(f"Expected FPS: {self.expected_fps} FPS")
        print(f"Intra-frame anomaly threshold: {self.max_gap_ms} ms")
        print(f"Inter-file anomaly threshold: ±{self.max_file_gap_ms} ms")
        print(f"{'='*60}")
        
        if not self.find_segment_files(directory):
            return False
        
        print(f"\nStarting analysis of {len(self.segment_files)} video files...")
        
        # Reset
        self.prev_file_end_time_us = None
        self.prev_filename = None
        self.continuity_errors = []
        
        for seg_num, filepath, filename in self.segment_files:
            result = self.analyze_video_frames(filepath, seg_num, filename)
            self.results.append(result)
        
        self.generate_report()
        return True
    
    def generate_report(self):
        """Generate analysis report"""
        print(f"\n{'='*60}")
        print(f"Analysis Report")
        print(f"{'='*60}")
        
        if not self.results:
            print("No analysis results")
            return
        
        # Display inter-file continuity results
        if self.continuity_errors:
            print(f"\nInter-file continuity anomalies:")
            print(f"{'-'*60}")
            for error in self.continuity_errors:
                gap_abs = abs(error['gap_ms'])
                print(f"  {error['from_file']} → {error['to_file']}")
                print(f"    Type: {error['type']}, Gap: {gap_abs:.1f} ms")
        else:
            print(f"\nAll files have good continuity")
        
        # File timeline
        print(f"\nTimeline (unit: microseconds):")
        print(f"{'-'*80}")
        print(f"{'File':6} | {'Start(μs)':12} | {'Duration(s)':8} | {'End(μs)':12} | {'Gap(ms)':10}")
        print(f"{'-'*80}")
        
        prev_end_us = None
        for result in sorted(self.results, key=lambda x: x['segment']):
            seg = result['segment']
            start_us = result['start_time_us']
            duration = result.get('file_duration_sec', 0)
            end_us = result.get('end_time_us', 0)
            
            # Calculate gap with previous file
            interval = "N/A"
            if prev_end_us is not None and end_us > 0:
                gap_us = start_us - prev_end_us
                interval_ms = gap_us / 1000.0
                interval = f"{interval_ms:+.1f}"
            
            print(f"{seg:6} | {start_us:12} | {duration:8.2f} | {end_us:12} | {interval:10}")
            
            if end_us > 0:
                prev_end_us = end_us
        
        # Summary
        print(f"\nOverall statistics:")
        total_files = len(self.results)
        total_frames = sum(r.get('total_frames', 0) for r in self.results)
        total_errors = sum(r.get('error_count', 0) for r in self.results)
        
        print(f"  Files analyzed: {total_files}")
        print(f"  Total frames: {total_frames:,}")
        print(f"  Total anomalous frames: {total_errors}")

def main():
    parser = argparse.ArgumentParser(description='Video Analysis - Inter-file Continuity Check')
    parser.add_argument('directory', help='Directory containing video files')
    parser.add_argument('--fps', type=float, default=30.0, 
                       help='Expected FPS (default: 30 FPS)')
    parser.add_argument('--max-gap', type=float, default=40.0,
                       help='Maximum allowed frame gap (milliseconds) (default: 40ms)')
    parser.add_argument('--max-file-gap', type=float, default=100.0,
                       help='Maximum allowed inter-file gap (milliseconds) (default: 100ms)')
    
    args = parser.parse_args()
    
    if not os.path.isdir(args.directory):
        print(f"Error: Directory does not exist - {args.directory}")
        return 1
    
    print(f"Python version: {sys.version}")
    
    # Create analyzer and run
    analyzer = SimpleVideoAnalyzer(
        expected_fps=args.fps,
        max_gap_ms=args.max_gap,
        max_file_gap_ms=args.max_file_gap
    )
    
    success = analyzer.analyze_all_segments(args.directory)
    
    if success:
        print(f"\nAnalysis completed!")
        return 0
    else:
        print(f"\nAnalysis failed!")
        return 1

if __name__ == "__main__":
    # Check OpenCV
    try:
        import cv2
        print(f"OpenCV version: {cv2.__version__}")
    except ImportError:
        print("OpenCV is required")
        print("Please run: sudo apt install python3-opencv")
        sys.exit(1)
    
    sys.exit(main())
