#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete IMU Database Time Interval Analysis Tool
Includes intra-file and inter-file continuity checks and packet loss reporting
Timestamp unit: nanoseconds (ns)
Filename format: IMUWriter_dev0_session10_segment{number}.db
"""

import os
import sys
import re
import sqlite3
import numpy as np
from datetime import datetime
from collections import defaultdict, Counter
import argparse

class IMUAnalyzer:
    def __init__(self, expected_freq=500, max_interval_ms=2.1, max_file_gap_ms=50, log_to_file=True):
        """
        Initialize analyzer
        
        Parameters:
            expected_freq: Expected sampling frequency (Hz)
            max_interval_ms: Maximum allowed intra-file time interval (ms)
            max_file_gap_ms: Maximum allowed inter-file gap (ms)
            log_to_file: Whether to save logs to file
        """
        self.expected_freq = expected_freq
        self.expected_interval_ns = int(1_000_000_000 / expected_freq)  # 2,000,000 ns = 2ms
        self.max_interval_ns = int(max_interval_ms * 1_000_000)  # 2,100,000 ns
        self.max_file_gap_ns = int(max_file_gap_ms * 1_000_000)  # Allowed inter-file gap
        self.log_to_file = log_to_file
        self.log_file = None
        
        # Analysis results
        self.results = []
        self.all_intervals = []
        self.file_gaps = []  # Inter-file intervals
        
        # Global statistics - separate statistics for accelerometer and gyroscope
        self.total_acc_packets = 0
        self.total_gyro_packets = 0
        self.total_acc_missing = 0
        self.total_gyro_missing = 0
        self.total_inter_file_missing = 0  # Inter-file packet loss
        
        self.imu_files = []
    
    def log(self, message, end="\n"):
        """Log message to file and terminal"""
        print(message, end=end)
        if self.log_file:
            self.log_file.write(f"{message}{end}")
            self.log_file.flush()
    
    def setup_logging(self, directory):
        """Set up log file"""
        if not self.log_to_file:
            return
        
        # Create log filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"imu_analysis_log_{timestamp}.txt"
        log_path = os.path.join(directory, log_filename)
        
        try:
            self.log_file = open(log_path, 'w', encoding='utf-8')
            self.log(f"📝 IMU Analysis Log - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            self.log(f"Analysis directory: {directory}")
            self.log(f"Log file: {log_path}")
            self.log(f"{'='*70}")
            return log_path
        except Exception as e:
            self.log(f"❌ Failed to create log file: {e}")
            self.log_file = None
    
    def close_logging(self):
        """Close log file"""
        if self.log_file:
            self.log(f"\n{'='*70}")
            self.log(f"✅ Analysis Complete - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            self.log_file.close()
            self.log_file = None
    
    def extract_segment_number(self, filename):
        """Extract segment number from filename"""
        match = re.search(r'segment(\d+)', filename, re.IGNORECASE)
        if match:
            return int(match.group(1))
        
        numbers = re.findall(r'\d+', filename)
        if numbers:
            return int(numbers[-1])
        
        return None
    
    def find_imu_files(self, directory):
        """Find and sort IMU database files"""
        self.log(f"📂 Scanning directory: {directory}")
        
        imu_files = []
        
        for filename in os.listdir(directory):
            if not filename.lower().endswith('.db'):
                continue
            
            segment_num = self.extract_segment_number(filename)
            if segment_num is not None:
                filepath = os.path.join(directory, filename)
                imu_files.append((segment_num, filepath, filename))
                self.log(f"  Found: [{segment_num:3d}] {filename}")
        
        if not imu_files:
            self.log("❌ No database files found")
            return False
        
        imu_files.sort(key=lambda x: x[0])
        
        self.log(f"\n✅ Found {len(imu_files)} database files:")
        for seg_num, _, filename in imu_files:
            self.log(f"  [{seg_num:3d}] {filename}")
        
        self.imu_files = imu_files
        return True
    
    def analyze_database(self, filepath, filename, seg_num, prev_end_time=None):
        """Analyze single database file - includes inter-file continuity check"""
        self.log(f"\n{'='*70}")
        self.log(f"📊 Analyzing [{seg_num}] {filename}")
        self.log(f"{'='*70}")
        
        result = {
            'segment': seg_num,
            'filename': filename,
            'acc_packets': 0,
            'gyro_packets': 0,
            'acc_timestamps': [],
            'gyro_timestamps': [],
            'acc_start_ns': None,
            'acc_end_ns': None,
            'gyro_start_ns': None,
            'gyro_end_ns': None,
            'acc_intervals_ns': [],
            'gyro_intervals_ns': [],
            'acc_missing': 0,
            'gyro_missing': 0,
            'inter_file_gap_ns': None,
            'inter_file_missing': 0,
            'acc_actual_freq': 0,
            'gyro_actual_freq': 0,
            'acc_duration_ns': 0,
            'gyro_duration_ns': 0,
            'status': 'PENDING'
        }
        
        try:
            conn = sqlite3.connect(filepath)
            cursor = conn.cursor()
            
            # Get table structure information
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = cursor.fetchall()
            table_names = [t[0] for t in tables]
            self.log(f"  Database tables: {table_names}")
            
            # 🔍 Step 1: Get start and end times of accelerometer table (for inter-file check)
            current_start_time = None
            current_end_time = None
            acc_table_found = False
            acc_table_name = None
            
            for table_name in table_names:
                if 'acc' in table_name.lower():
                    acc_table_name = table_name
                    break
            
            if acc_table_name:
                # Get timestamp column
                cursor.execute(f"PRAGMA table_info({acc_table_name});")
                columns = cursor.fetchall()
                column_names = [col[1] for col in columns]
                
                # Find timestamp column
                time_column = None
                for col_name in column_names:
                    if any(keyword in col_name.lower() for keyword in ['time', 'timestamp', 'ts']):
                        time_column = col_name
                        break
                
                if not time_column and column_names:
                    time_column = column_names[0]
                
                if time_column:
                    # Get min and max timestamps of accelerometer table (start and end times)
                    cursor.execute(f"SELECT MIN({time_column}), MAX({time_column}) FROM {acc_table_name};")
                    time_range_row = cursor.fetchone()
                    if time_range_row and time_range_row[0] and time_range_row[1]:
                        current_start_time = time_range_row[0]  # Start time
                        current_end_time = time_range_row[1]    # End time
                        result['acc_start_ns'] = current_start_time
                        result['acc_end_ns'] = current_end_time  # Save end time
                        acc_table_found = True
                        self.log(f"  📍 Accelerometer table time range: {current_start_time:,} -> {current_end_time:,} ns")
            
            # 🔍 Step 2: Perform inter-file continuity check (before analyzing table data)
            result['inter_file_gap_ns'] = None
            result['inter_file_missing'] = 0
            
            if prev_end_time is not None and current_start_time is not None:
                file_gap_ns = current_start_time - prev_end_time
                result['inter_file_gap_ns'] = file_gap_ns
                
                self.log(f"\n  🔗 Inter-file continuity check:")
                self.log(f"    Previous file end time: {prev_end_time:,} ns")
                self.log(f"    Current file start time: {current_start_time:,} ns")
                self.log(f"    Inter-file gap: {file_gap_ns:,} ns ({file_gap_ns/1_000_000:.3f} ms)")
                
                # Calculate inter-file packet loss
                if file_gap_ns > self.max_file_gap_ns:
                    extra_gap_ns = file_gap_ns - self.max_file_gap_ns
                    missing_between_files = max(0, int(round(extra_gap_ns / self.expected_interval_ns)))
                    result['inter_file_missing'] = missing_between_files
                    self.total_inter_file_missing += missing_between_files
                    
                    self.log(f"    ⚠️  Inter-file packet loss: {missing_between_files:,} packets")
                    self.log(f"      Extra gap: {extra_gap_ns:,} ns")
                    self.log(f"      Theoretical packet interval: {self.expected_interval_ns:,} ns")
                else:
                    self.log(f"    ✅ Good inter-file continuity (within allowed {self.max_file_gap_ns:,} ns)")
            elif prev_end_time is None:
                self.log(f"\n  ℹ️  First file, skipping inter-file continuity check")
            elif current_start_time is None:
                self.log(f"\n  ⚠️  Cannot get current file start time, skipping inter-file check")
            else:
                self.log(f"\n  ⚠️  Cannot get inter-file continuity information")
            
            # 🔍 Step 3: Detailed analysis of accelerometer data table
            acc_table = None
            for table in table_names:
                if 'acc' in table.lower():
                    acc_table = table
                    break
            
            if acc_table:
                self.log(f"\n  📈 Analyzing accelerometer table: {acc_table}")
                acc_result = self._analyze_table(cursor, acc_table, "Accelerometer")
                
                # Update result dictionary
                for key, value in acc_result.items():
                    result[key] = value
                
                # Ensure accelerometer table end time is saved
                if 'acc_end_ns' in acc_result and acc_result['acc_end_ns']:
                    result['acc_end_ns'] = acc_result['acc_end_ns']
                    self.log(f"    Accelerometer table actual end time: {acc_result['acc_end_ns']:,} ns")
                    
                    # If previously obtained end time is inconsistent, use this one
                    if current_end_time is not None and current_end_time != acc_result['acc_end_ns']:
                        self.log(f"    ⚠️  Timestamp inconsistency, using detailed analysis result")
                        current_end_time = acc_result['acc_end_ns']
            
            # 🔍 Step 4: Detailed analysis of gyroscope data table
            gyro_table = None
            for table in table_names:
                if 'gyro' in table.lower():
                    gyro_table = table
                    break
            
            if gyro_table:
                self.log(f"\n  📉 Analyzing gyroscope table: {gyro_table}")
                gyro_result = self._analyze_table(cursor, gyro_table, "Gyroscope")
                
                # Update result dictionary
                for key, value in gyro_result.items():
                    result[key] = value
            
            conn.close()
            
            # Summary statistics - Fix here: directly use values from result dictionary
            acc_packets = result.get('acc_packets', 0)
            gyro_packets = result.get('gyro_packets', 0)
            total_packets = acc_packets + gyro_packets
            acc_missing = result.get('acc_missing', 0)
            gyro_missing = result.get('gyro_missing', 0)
            total_missing = acc_missing + gyro_missing + result['inter_file_missing']
            
            self.log(f"\n  📊 File summary:")
            self.log(f"    Accelerometer packets: {acc_packets:,}")
            self.log(f"    Gyroscope packets: {gyro_packets:,}")
            self.log(f"    Total packets: {total_packets:,}")
            self.log(f"    Accelerometer packet loss: {acc_missing:,}")
            self.log(f"    Gyroscope packet loss: {gyro_missing:,}")
            self.log(f"    Inter-file packet loss: {result['inter_file_missing']:,}")
            self.log(f"    Total packet loss: {total_missing:,}")
            
            if acc_packets > 0:
                acc_missing_rate = (acc_missing / acc_packets) * 100
                self.log(f"    Accelerometer packet loss rate: {acc_missing_rate:.6f}%")
            
            if gyro_packets > 0:
                gyro_missing_rate = (gyro_missing / gyro_packets) * 100
                self.log(f"    Gyroscope packet loss rate: {gyro_missing_rate:.6f}%")
            
            if total_packets > 0:
                total_missing_rate = (total_missing / total_packets) * 100
                self.log(f"    Total packet loss rate: {total_missing_rate:.6f}%")
            
            result['status'] = 'DONE'
            
            # 🔍 Step 5: Return file end time (last packet time of accelerometer table) for next file
            end_time_for_next = result.get('acc_end_ns', None)
            
            if end_time_for_next:
                self.log(f"\n  📍 Recording this file end time: {end_time_for_next:,} ns (for next file use)")
            else:
                self.log(f"\n  ⚠️  Cannot get this file end time")
                # If cannot get end time, use start time plus estimated duration
                if result.get('acc_start_ns'):
                    # Estimate duration based on packet count and expected frequency
                    acc_packets = result.get('acc_packets', 0)
                    if acc_packets > 0:
                        estimated_duration_ns = acc_packets * self.expected_interval_ns
                        end_time_for_next = result['acc_start_ns'] + estimated_duration_ns
                        self.log(f"    Using estimated end time: {end_time_for_next:,} ns (based on {acc_packets:,} packets)")
                    else:
                        estimated_duration = 600 * 1_000_000_000  # 600 second estimate
                        end_time_for_next = result['acc_start_ns'] + estimated_duration
                        self.log(f"    Using default estimated end time: {end_time_for_next:,} ns")
            
            return result, end_time_for_next
            
        except Exception as e:
            self.log(f"❌ Database analysis error: {e}")
            import traceback
            traceback.print_exc()
            result['status'] = f'ERROR: {str(e)}'
            return result, None
    
    def _analyze_table(self, cursor, table_name, sensor_name):
        """Analyze single data table"""
        # Build correct key names based on sensor name
        prefix = 'acc' if 'Accelerometer' in sensor_name else 'gyro'
        
        result = {}
        
        # Get column names
        cursor.execute(f"PRAGMA table_info({table_name});")
        columns = cursor.fetchall()
        column_names = [col[1] for col in columns]
        
        # Find timestamp column
        time_column = None
        for col_name in column_names:
            if any(keyword in col_name.lower() for keyword in ['time', 'timestamp', 'ts']):
                time_column = col_name
                break
        
        if not time_column:
            # Try other possible columns
            for col_name in ['timestamp', 'ts', 't']:
                if col_name in column_names:
                    time_column = col_name
                    break
        
        if not time_column:
            time_column = column_names[0]  # Default to first column
        
        self.log(f"    Using timestamp column: {time_column}")
        
        # Get data count
        cursor.execute(f"SELECT COUNT(*) FROM {table_name};")
        packet_count = cursor.fetchone()[0]
        result[f'{prefix}_packets'] = packet_count
        self.log(f"    Packet count: {packet_count:,}")
        
        if packet_count == 0:
            return result
        
        # Get min and max timestamps
        cursor.execute(f"SELECT MIN({time_column}), MAX({time_column}) FROM {table_name};")
        min_max_result = cursor.fetchone()
        start_time = min_max_result[0] if min_max_result and min_max_result[0] is not None else None
        end_time = min_max_result[1] if min_max_result and min_max_result[1] is not None else None
        
        # Get all timestamps for interval analysis
        cursor.execute(f"SELECT {time_column} FROM {table_name} ORDER BY {time_column};")
        rows = cursor.fetchall()
        timestamps = [row[0] for row in rows]
        
        result[f'{prefix}_timestamps'] = timestamps
        
        # Save start and end times
        if start_time:
            result[f'{prefix}_start_ns'] = start_time
        if end_time:
            result[f'{prefix}_end_ns'] = end_time
        
        if start_time and end_time:
            duration_ns = end_time - start_time
            result[f'{prefix}_duration_ns'] = duration_ns
            
            self.log(f"    Time range: {start_time:,} -> {end_time:,} ns")
            self.log(f"    Total duration: {duration_ns:,} ns ({duration_ns/1_000_000:.3f} ms)")
            self.log(f"          {duration_ns/1_000_000_000:.6f} seconds")
        
        # Calculate time intervals and packet loss
        intervals = []
        missing_packets = 0
        
        for i in range(1, len(timestamps)):
            interval_ns = timestamps[i] - timestamps[i-1]
            intervals.append(interval_ns)
            
            # Calculate packet loss
            if interval_ns > self.max_interval_ns:
                # Extra interval minus allowed error
                extra_interval = interval_ns - self.max_interval_ns
                missing_in_gap = max(0, int(round(extra_interval / self.expected_interval_ns)))
                missing_packets += missing_in_gap
        
        result[f'{prefix}_intervals_ns'] = intervals
        result[f'{prefix}_missing'] = missing_packets
        
        # Calculate actual frequency
        if start_time and end_time and packet_count > 1:
            duration_seconds = (end_time - start_time) / 1_000_000_000
            if duration_seconds > 0:
                actual_freq = (packet_count - 1) / duration_seconds
                result[f'{prefix}_actual_freq'] = actual_freq
                self.log(f"    Actual frequency: {actual_freq:.2f} Hz")
        
        # Analyze interval distribution
        if intervals:
            self._analyze_interval_distribution(intervals, sensor_name, result)
        
        # Collect all intervals for global statistics
        self.all_intervals.extend(intervals)
        
        return result
    
    def _analyze_interval_distribution(self, intervals, sensor_name, result):
        """Analyze time interval distribution"""
        if not intervals:
            return
        
        # Count interval distribution (grouped by 1ms)
        interval_counter = Counter()
        for interval_ns in intervals:
            interval_ms = interval_ns / 1_000_000
            bin_start = int(interval_ms // 1)  # 1ms grouping
            interval_counter[bin_start] += 1
        
        # Print distribution
        self.log(f"\n    📊 {sensor_name} time interval distribution:")
        self.log(f"      {'Interval range(ms)':<15} | {'Count':<12} | {'Percentage':<10} | {'Missing packets':<12}")
        self.log(f"      {'-'*15} | {'-'*12} | {'-'*10} | {'-'*12}")
        
        total_intervals = len(intervals)
        missing_per_bin = defaultdict(int)
        
        # Calculate packet loss for each interval
        for bin_start, count in interval_counter.items():
            if bin_start * 1_000_000 > self.max_interval_ns:
                # Use interval midpoint to calculate packet loss
                interval_mid_ns = (bin_start + 0.5) * 1_000_000
                extra_interval = interval_mid_ns - self.max_interval_ns
                missing_in_bin = max(0, int(round((extra_interval / self.expected_interval_ns) * count)))
                missing_per_bin[bin_start] = missing_in_bin
        
        # Only show intervals with data
        sorted_bins = sorted(interval_counter.keys())
        for bin_start in sorted_bins:
            count = interval_counter[bin_start]
            percentage = (count / total_intervals) * 100
            bin_range = f"{bin_start}-{bin_start+1}"
            missing = missing_per_bin.get(bin_start, 0)
            
            self.log(f"      {bin_range:<15} | {count:<12,} | {percentage:8.4f}% | {missing:<12,}")
        
        self.log(f"    ⚠️  {sensor_name} missing packets: {result.get('acc_missing' if 'Accelerometer' in sensor_name else 'gyro_missing', 0):,}")
    
    def analyze_all_files(self, directory):
        """Analyze all database files"""
        # Set up logging
        log_path = self.setup_logging(directory)
        
        self.log(f"{'='*70}")
        self.log(f"🎯 IMU Database Time Interval Analysis")
        self.log(f"Expected frequency: {self.expected_freq} Hz")
        self.log(f"Theoretical interval: {self.expected_interval_ns:,} ns ({self.expected_interval_ns/1_000_000:.3f} ms)")
        self.log(f"Qualified interval: ≤ {self.max_interval_ns:,} ns ({self.max_interval_ns/1_000_000:.3f} ms)")
        self.log(f"Allowed inter-file gap: ≤ {self.max_file_gap_ns:,} ns ({self.max_file_gap_ns/1_000_000:.3f} ms)")
        if log_path:
            self.log(f"Log file: {log_path}")
        self.log(f"{'='*70}")
        
        if not self.find_imu_files(directory):
            return False
        
        self.log(f"\nStarting analysis of {len(self.imu_files)} database files...")
        
        # Reset statistics
        self.results = []
        self.all_intervals = []
        self.file_gaps = []
        self.total_acc_packets = 0
        self.total_gyro_packets = 0
        self.total_acc_missing = 0
        self.total_gyro_missing = 0
        self.total_inter_file_missing = 0
        
        # Analyze files in order
        prev_end_time = None
        
        for i, (seg_num, filepath, filename) in enumerate(self.imu_files):
            self.log(f"\n🔍 Processing file {i+1}/{len(self.imu_files)}: [{seg_num}] {filename}")
            
            # If there's a previous file, show its end time
            if prev_end_time is not None and i > 0:
                prev_filename = self.imu_files[i-1][2]
                self.log(f"  Previous file [{self.imu_files[i-1][0]}] {prev_filename} end time: {prev_end_time:,} ns")
            
            result, end_time = self.analyze_database(filepath, filename, seg_num, prev_end_time)
            self.results.append(result)
            
            # Update statistics - separate statistics for accelerometer and gyroscope
            acc_packets = result.get('acc_packets', 0)
            gyro_packets = result.get('gyro_packets', 0)
            acc_missing = result.get('acc_missing', 0)
            gyro_missing = result.get('gyro_missing', 0)
            
            self.total_acc_packets += acc_packets
            self.total_gyro_packets += gyro_packets
            self.total_acc_missing += acc_missing
            self.total_gyro_missing += gyro_missing
            
            # Inter-file packet loss already accumulated in analyze_database
            
            # Record inter-file gaps
            if result['inter_file_gap_ns'] is not None:
                self.file_gaps.append(result['inter_file_gap_ns'])
            
            # Update previous file end time
            if end_time is not None:
                prev_end_time = end_time
            else:
                # If cannot get end time, try to use estimated value
                if result.get('acc_end_ns'):
                    prev_end_time = result['acc_end_ns']
                elif result.get('acc_start_ns') and acc_packets > 0:
                    # Use start time plus estimated duration based on packet count
                    estimated_duration_ns = acc_packets * self.expected_interval_ns
                    prev_end_time = result['acc_start_ns'] + estimated_duration_ns
                
                if prev_end_time:
                    self.log(f"  ⚠️  Using estimated end time for next file: {prev_end_time:,} ns")
        
        # Generate global report
        self.generate_global_report()
        
        # Generate final report file
        self.save_summary_report(directory)
        
        return True
    
    def save_summary_report(self, directory):
        """Save summary report to file"""
        if not self.results:
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"imu_summary_report_{timestamp}.txt"
        report_path = os.path.join(directory, report_filename)
        
        try:
            with open(report_path, 'w', encoding='utf-8') as report_file:
                report_file.write(f"IMU Database Analysis Summary Report\n")
                report_file.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                report_file.write(f"Analysis directory: {directory}\n")
                report_file.write(f"{'='*70}\n\n")
                
                # File information
                report_file.write(f"📁 Analysis File Information:\n")
                report_file.write(f"  Total files: {len(self.imu_files)}\n")
                report_file.write(f"  Expected frequency: {self.expected_freq} Hz\n")
                report_file.write(f"  Theoretical interval: {self.expected_interval_ns:,} ns\n")
                report_file.write(f"  Maximum allowed interval: {self.max_interval_ns:,} ns\n")
                report_file.write(f"\n")
                
                # File summary table
                report_file.write(f"📊 File Analysis Summary:\n")
                report_file.write(f"{'-'*120}\n")
                header = f"{'File':6} | {'Accel':8} | {'Gyro':8} | {'Accel Loss':12} | {'Gyro Loss':12} | {'Inter-file Loss':12} | {'Status':8}\n"
                report_file.write(header)
                report_file.write(f"{'-'*120}\n")
                
                for result in sorted(self.results, key=lambda x: x['segment']):
                    seg = result['segment']
                    acc_packets = result.get('acc_packets', 0)
                    gyro_packets = result.get('gyro_packets', 0)
                    acc_missing = result.get('acc_missing', 0)
                    gyro_missing = result.get('gyro_missing', 0)
                    inter_missing = result['inter_file_missing']
                    status = result['status']
                    
                    row = f"{seg:6} | {acc_packets:8,} | {gyro_packets:8,} | {acc_missing:12,} | {gyro_missing:12,} | {inter_missing:12,} | {status:8}\n"
                    report_file.write(row)
                
                report_file.write(f"\n")
                
                # Overall statistics
                total_packets = self.total_acc_packets + self.total_gyro_packets
                total_missing = self.total_acc_missing + self.total_gyro_missing + self.total_inter_file_missing
                
                report_file.write(f"📈 Overall Statistics:\n")
                report_file.write(f"  Total accelerometer packets: {self.total_acc_packets:,}\n")
                report_file.write(f"  Accelerometer packet loss: {self.total_acc_missing:,}\n")
                if self.total_acc_packets > 0:
                    acc_rate = (self.total_acc_missing / self.total_acc_packets) * 100
                    report_file.write(f"  Accelerometer packet loss rate: {acc_rate:.6f}%\n")
                
                report_file.write(f"\n")
                report_file.write(f"  Total gyroscope packets: {self.total_gyro_packets:,}\n")
                report_file.write(f"  Gyroscope packet loss: {self.total_gyro_missing:,}\n")
                if self.total_gyro_packets > 0:
                    gyro_rate = (self.total_gyro_missing / self.total_gyro_packets) * 100
                    report_file.write(f"  Gyroscope packet loss rate: {gyro_rate:.6f}%\n")
                
                report_file.write(f"\n")
                report_file.write(f"  Inter-file packet loss: {self.total_inter_file_missing:,}\n")
                report_file.write(f"  Total packets: {total_packets:,}\n")
                report_file.write(f"  Total packet loss: {total_missing:,}\n")
                if total_packets > 0:
                    total_rate = (total_missing / total_packets) * 100
                    report_file.write(f"  Total packet loss rate: {total_rate:.6f}%\n")
                
                # Quality rating
                if total_packets > 0:
                    total_missing_rate = (total_missing / total_packets) * 100
                    if total_missing_rate < 0.01:
                        rating = "★★★★★ Excellent"
                    elif total_missing_rate < 0.1:
                        rating = "★★★★ Good"
                    elif total_missing_rate < 1:
                        rating = "★★★ Average"
                    elif total_missing_rate < 5:
                        rating = "★★ Poor"
                    else:
                        rating = "★ Very Poor"
                    
                    report_file.write(f"\n🏆 Quality Rating: {rating}\n")
            
            self.log(f"\n📄 Summary report saved: {report_path}")
            
        except Exception as e:
            self.log(f"❌ Failed to save summary report: {e}")
    
    def generate_global_report(self):
        """Generate global analysis report"""
        self.log(f"\n{'='*70}")
        self.log(f"📋 Global Analysis Report")
        self.log(f"{'='*70}")
        
        if not self.results:
            self.log("No analysis results")
            return
        
        # File summary table
        self.log(f"\n📁 File Analysis Summary:")
        self.log(f"{'-'*150}")
        header = f"{'File':6} | {'Accel':8} | {'Gyro':8} | {'Accel Loss':12} | {'Gyro Loss':12} | {'Inter-file Loss':12} | {'Start Time':25} | {'End Time':25} | {'Gap(ms)':15} | {'Status':8}"
        self.log(header)
        self.log(f"{'-'*150}")
        
        for result in sorted(self.results, key=lambda x: x['segment']):
            seg = result['segment']
            acc_packets = result.get('acc_packets', 0)
            gyro_packets = result.get('gyro_packets', 0)
            acc_missing = result.get('acc_missing', 0)
            gyro_missing = result.get('gyro_missing', 0)
            inter_missing = result['inter_file_missing']
            
            # Format time display
            start_time_str = "N/A"
            end_time_str = "N/A"
            
            if result.get('acc_start_ns'):
                start_time_str = f"{result['acc_start_ns']:,}"
            if result.get('acc_end_ns'):
                end_time_str = f"{result['acc_end_ns']:,}"
            
            # Format inter-file gap display
            gap_display = "N/A"
            if result['inter_file_gap_ns'] is not None:
                gap_ms = result['inter_file_gap_ns'] / 1_000_000
                gap_display = f"{gap_ms:+.3f} ms"
            
            status = result['status']
            
            # Shorten displayed timestamps (show only last few digits)
            if len(start_time_str) > 20 and start_time_str != "N/A":
                start_time_str = "..." + start_time_str[-15:]
            if len(end_time_str) > 20 and end_time_str != "N/A":
                end_time_str = "..." + end_time_str[-15:]
            
            row = f"{seg:6} | {acc_packets:8,} | {gyro_packets:8,} | {acc_missing:12,} | {gyro_missing:12,} | {inter_missing:12,} | {start_time_str:25} | {end_time_str:25} | {gap_display:15} | {status:8}"
            self.log(row)
        
        # Global interval distribution statistics
        if self.all_intervals:
            self.log(f"\n📊 Global Time Interval Distribution Statistics:")
            self.log(f"  Total intervals: {len(self.all_intervals):,}")
            
            # Convert to ms for statistics
            intervals_ms = np.array(self.all_intervals) / 1_000_000
            self.log(f"  Average interval: {np.mean(intervals_ms):.3f} ms")
            self.log(f"  Minimum interval: {np.min(intervals_ms):.3f} ms")
            self.log(f"  Maximum interval: {np.max(intervals_ms):.3f} ms")
            self.log(f"  Standard deviation: {np.std(intervals_ms):.3f} ms")
            
            # Detailed inter-file gap statistics
            if self.file_gaps:
                self.log(f"\n🔗 Detailed Inter-file Gap Statistics:")
                self.log(f"  Total inter-file gaps: {len(self.file_gaps)}")
                
                gaps_ms = np.array(self.file_gaps) / 1_000_000
                
                # Classify statistics by gap size
                gap_categories = {
                    'Excellent (≤10ms)': 0,
                    'Good (10-50ms)': 0,
                    'Warning (50-100ms)': 0,
                    'Severe (>100ms)': 0
                }
                
                for gap_ms in gaps_ms:
                    if gap_ms <= 10:
                        gap_categories['Excellent (≤10ms)'] += 1
                    elif gap_ms <= 50:
                        gap_categories['Good (10-50ms)'] += 1
                    elif gap_ms <= 100:
                        gap_categories['Warning (50-100ms)'] += 1
                    else:
                        gap_categories['Severe (>100ms)'] += 1
                
                self.log(f"\n  Inter-file gap distribution:")
                for category, count in gap_categories.items():
                    if count > 0:
                        percentage = (count / len(self.file_gaps)) * 100
                        self.log(f"    {category}: {count} times ({percentage:.1f}%)")
                
                self.log(f"\n  Inter-file gap numerical statistics:")
                self.log(f"    Average gap: {np.mean(gaps_ms):.3f} ms")
                self.log(f"    Minimum gap: {np.min(gaps_ms):.3f} ms")
                self.log(f"    Maximum gap: {np.max(gaps_ms):.3f} ms")
                self.log(f"    Total inter-file packet loss: {self.total_inter_file_missing:,} packets")
            
            # Detailed interval distribution
            self.log(f"\n📈 Detailed Global Interval Distribution:")
            self._print_detailed_distribution()
        
        # Overall packet loss statistics - separate for accelerometer and gyroscope
        self.log(f"\n⚠️  Overall Packet Loss Statistics:")
        
        # Accelerometer statistics
        self.log(f"\n  📈 Accelerometer Statistics:")
        self.log(f"    Total packets: {self.total_acc_packets:,}")
        self.log(f"    Intra-file packet loss: {self.total_acc_missing:,} ({self._percentage_str(self.total_acc_missing, self.total_acc_packets)})")
        if self.total_acc_packets > 0:
            acc_missing_rate = (self.total_acc_missing / self.total_acc_packets) * 100
            self.log(f"    Packet loss rate: {acc_missing_rate:.6f}%")
        
        # Gyroscope statistics
        self.log(f"\n  📉 Gyroscope Statistics:")
        self.log(f"    Total packets: {self.total_gyro_packets:,}")
        self.log(f"    Intra-file packet loss: {self.total_gyro_missing:,} ({self._percentage_str(self.total_gyro_missing, self.total_gyro_packets)})")
        if self.total_gyro_packets > 0:
            gyro_missing_rate = (self.total_gyro_missing / self.total_gyro_packets) * 100
            self.log(f"    Packet loss rate: {gyro_missing_rate:.6f}%")
        
        # Inter-file packet loss statistics
        self.log(f"\n  🔗 Inter-file Packet Loss Statistics:")
        self.log(f"    Total inter-file packet loss: {self.total_inter_file_missing:,}")
        
        # Totals
        total_packets = self.total_acc_packets + self.total_gyro_packets
        total_missing = self.total_acc_missing + self.total_gyro_missing + self.total_inter_file_missing
        self.log(f"\n  📊 Totals:")
        self.log(f"    Total packets: {total_packets:,}")
        self.log(f"    Total packet loss: {total_missing:,}")
        if total_packets > 0:
            total_missing_rate = (total_missing / total_packets) * 100
            self.log(f"    Total packet loss rate: {total_missing_rate:.6f}%")
        
        # Data integrity analysis - separate calculation for accelerometer and gyroscope
        self.log(f"\n📈 Data Integrity Analysis:")
        
        # Accelerometer integrity analysis
        total_acc_duration = 0
        valid_acc_results = [r for r in self.results if r.get('acc_duration_ns', 0) > 0]
        if valid_acc_results:
            total_acc_duration = sum(r['acc_duration_ns'] for r in valid_acc_results) / 1_000_000_000  # Convert to seconds
            expected_acc_packets = total_acc_duration * self.expected_freq
            
            self.log(f"\n  📈 Accelerometer:")
            self.log(f"    Theoretical packets: {expected_acc_packets:,.0f}")
            self.log(f"    Actual packets: {self.total_acc_packets:,.0f}")
            if expected_acc_packets > 0:
                acc_completeness = (self.total_acc_packets / expected_acc_packets * 100)
                self.log(f"    Data completeness rate: {acc_completeness:.2f}%")
        
        # Gyroscope integrity analysis
        total_gyro_duration = 0
        valid_gyro_results = [r for r in self.results if r.get('gyro_duration_ns', 0) > 0]
        if valid_gyro_results:
            total_gyro_duration = sum(r['gyro_duration_ns'] for r in valid_gyro_results) / 1_000_000_000  # Convert to seconds
            expected_gyro_packets = total_gyro_duration * self.expected_freq
            
            self.log(f"\n  📉 Gyroscope:")
            self.log(f"    Theoretical packets: {expected_gyro_packets:,.0f}")
            self.log(f"    Actual packets: {self.total_gyro_packets:,.0f}")
            if expected_gyro_packets > 0:
                gyro_completeness = (self.total_gyro_packets / expected_gyro_packets * 100)
                self.log(f"    Data completeness rate: {gyro_completeness:.2f}%")
        
        # Quality rating - based on total packet loss rate
        if total_packets > 0:
            total_missing_rate = (total_missing / total_packets) * 100
            if total_missing_rate < 0.01:
                rating = "★★★★★ Excellent"
            elif total_missing_rate < 0.1:
                rating = "★★★★ Good"
            elif total_missing_rate < 1:
                rating = "★★★ Average"
            elif total_missing_rate < 5:
                rating = "★★ Poor"
            else:
                rating = "★ Very Poor"
            
            self.log(f"\n🏆 Quality Rating: {rating}")
            
            # Show accelerometer and gyroscope quality ratings separately
            if self.total_acc_packets > 0:
                acc_missing_rate = (self.total_acc_missing / self.total_acc_packets) * 100
                if acc_missing_rate < 0.01:
                    acc_rating = "Excellent"
                elif acc_missing_rate < 0.1:
                    acc_rating = "Good"
                elif acc_missing_rate < 1:
                    acc_rating = "Average"
                elif acc_missing_rate < 5:
                    acc_rating = "Poor"
                else:
                    acc_rating = "Very Poor"
                self.log(f"  Accelerometer quality: {acc_rating}")
            
            if self.total_gyro_packets > 0:
                gyro_missing_rate = (self.total_gyro_missing / self.total_gyro_packets) * 100
                if gyro_missing_rate < 0.01:
                    gyro_rating = "Excellent"
                elif gyro_missing_rate < 0.1:
                    gyro_rating = "Good"
                elif gyro_missing_rate < 1:
                    gyro_rating = "Average"
                elif gyro_missing_rate < 5:
                    gyro_rating = "Poor"
                else:
                    gyro_rating = "Very Poor"
                self.log(f"  Gyroscope quality: {gyro_rating}")
    
    def _percentage_str(self, part, total):
        """Calculate percentage string"""
        if total == 0:
            return "0.00%"
        percentage = (part / total) * 100
        return f"{percentage:.4f}%"
    
    def _print_detailed_distribution(self):
        """Print detailed interval distribution"""
        if not self.all_intervals:
            return
        
        # Count all intervals
        interval_counter = Counter()
        for interval_ns in self.all_intervals:
            interval_ms = interval_ns / 1_000_000
            bin_start = int(interval_ms // 1)
            interval_counter[bin_start] += 1
        
        # Find maximum interval
        max_bin = max(interval_counter.keys()) if interval_counter else 0
        
        self.log(f"    {'Interval range(ms)':<15} | {'Count':<12} | {'Percentage':<10} | {'Cumulative %':<12} | {'Missing packets':<12}")
        self.log(f"    {'-'*15} | {'-'*12} | {'-'*10} | {'-'*12} | {'-'*12}")
        
        total_intervals = len(self.all_intervals)
        total_missing_in_dist = 0
        cumulative_percentage = 0
        
        # Print statistics for each interval
        for bin_start in range(0, min(max_bin + 1, 20)):  # Only show first 20 intervals
            count = interval_counter.get(bin_start, 0)
            if count == 0 and bin_start > 10:  # Skip larger empty intervals
                continue
                
            percentage = (count / total_intervals) * 100
            cumulative_percentage += percentage
            bin_range = f"{bin_start}-{bin_start+1}"
            
            # Calculate packet loss for this interval
            missing_in_bin = 0
            if bin_start * 1_000_000 > self.max_interval_ns:
                interval_mid_ns = (bin_start + 0.5) * 1_000_000
                extra_interval = interval_mid_ns - self.max_interval_ns
                missing_in_bin = max(0, int(round((extra_interval / self.expected_interval_ns) * count)))
                total_missing_in_dist += missing_in_bin
            
            self.log(f"    {bin_range:<15} | {count:<12,} | {percentage:8.4f}% | {cumulative_percentage:10.4f}% | {missing_in_bin:<12,}")
        
        # If there are larger intervals, show summary
        if max_bin >= 20:
            remaining_count = sum(count for bin_start, count in interval_counter.items() if bin_start >= 20)
            if remaining_count > 0:
                remaining_percentage = (remaining_count / total_intervals) * 100
                cumulative_percentage += remaining_percentage
                self.log(f"    {'20+':<15} | {remaining_count:<12,} | {remaining_percentage:8.4f}% | {cumulative_percentage:10.4f}% | {'N/A':<12}")
        
        self.log(f"    {'Total':<15} | {total_intervals:<12,} | {'100.0000%':<10} | {'100.0000%':<12} | {total_missing_in_dist:<12,}")

def main():
    parser = argparse.ArgumentParser(description='IMU Database Time Interval Analysis Tool')
    parser.add_argument('directory', help='Directory containing IMU database files')
    parser.add_argument('--freq', type=float, default=500.0, 
                       help='Expected sampling frequency (Hz) (default: 500 Hz)')
    parser.add_argument('--max-interval', type=float, default=2.10,
                       help='Maximum allowed intra-file time interval (ms) (default: 2.10 ms)')
    parser.add_argument('--max-file-gap', type=float, default=50.0,
                       help='Maximum allowed inter-file gap (ms) (default: 50 ms)')
    parser.add_argument('--no-log', action='store_true',
                       help='Do not save log file')
    
    args = parser.parse_args()
    
    if not os.path.isdir(args.directory):
        print(f"Error: Directory does not exist - {args.directory}")
        return 1
    
    print(f"Python version: {sys.version}")
    
    # Check NumPy
    try:
        import numpy as np
        print(f"NumPy version: {np.__version__}")
    except ImportError:
        print("❌ NumPy is required")
        print("Please run: pip install numpy or sudo apt install python3-numpy")
        return 1
    
    # Create analyzer and run
    analyzer = IMUAnalyzer(
        expected_freq=args.freq,
        max_interval_ms=args.max_interval,
        max_file_gap_ms=args.max_file_gap,
        log_to_file=not args.no_log
    )
    
    try:
        success = analyzer.analyze_all_files(args.directory)
        
        if success:
            print(f"\n{'='*70}")
            print(f"✅ IMU Analysis Complete!")
            print(f"{'='*70}")
            return 0
        else:
            print(f"\n❌ Analysis Failed!")
            return 1
    finally:
        analyzer.close_logging()

if __name__ == "__main__":
    sys.exit(main())
