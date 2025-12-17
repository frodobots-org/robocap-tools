#!/usr/bin/env python3
"""
Merge gyro_data and acc_data from multiple SQLite database files in a directory.
Matches gyro and acc data by timestamp with optimal matching algorithm.
Supports processing single or multiple IMU devices.
"""

import sqlite3
import csv
import argparse
import sys
import os
import re
from typing import List, Tuple, Optional, Dict
from pathlib import Path


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Merge gyro_data and acc_data from multiple SQLite database files to CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Process specific IMU device
  python merge_multi_imu_data.py -i /path/to/data_dir -d 0 -o output.csv -r 500
  
  # Process all IMU devices (generates multiple CSV files)
  python merge_multi_imu_data.py -i /path/to/data_dir -o output_prefix -r 500
        """
    )
    parser.add_argument('-i', '--input', required=True,
                        help='Input directory containing SQLite database files')
    parser.add_argument('-o', '--output', required=True,
                        help='Output CSV file path (or prefix if processing multiple IMU devices)')
    parser.add_argument('-r', '--rate', type=int, required=True,
                        help='IMU sampling rate in Hz (e.g., 500)')
    parser.add_argument('-d', '--imu-dev', type=int, choices=[0, 1, 2],
                        help='IMU device number to process (0, 1, or 2). If not specified, process all available IMU devices')
    
    return parser.parse_args()


def find_imu_files(data_dir: str, imu_dev: Optional[int] = None) -> Dict[int, List[str]]:
    """
    Find all IMU database files in the directory, grouped by device number.
    
    Args:
        data_dir: Directory path containing database files
        imu_dev: Optional IMU device number (0, 1, or 2) to filter
    
    Returns:
        Dictionary mapping device numbers to sorted list of file paths (sorted by segment number)
    """
    data_path = Path(data_dir)
    if not data_path.is_dir():
        print(f"Error: {data_dir} is not a directory", file=sys.stderr)
        sys.exit(1)
    
    # Pattern to match: IMUWriter_dev[number]_session[number]_segment[number]
    pattern = re.compile(r'IMUWriter_dev(\d+)_session(\d+)_segment(\d+)')
    
    imu_files: Dict[int, List[Tuple[int, str]]] = {}  # dev -> [(segment, filepath), ...]
    
    # Find all matching files
    for file_path in data_path.iterdir():
        if not file_path.is_file() or not file_path.suffix.lower() in ['.db', '.db3', '.sqlite', '.sqlite3']:
            continue
        
        match = pattern.match(file_path.name)
        if match:
            dev_num = int(match.group(1))
            segment_num = int(match.group(3))
            
            # Filter by imu_dev if specified
            if imu_dev is not None and dev_num != imu_dev:
                continue
            
            if dev_num not in imu_files:
                imu_files[dev_num] = []
            imu_files[dev_num].append((segment_num, str(file_path)))
    
    # Sort by segment number for each device
    result: Dict[int, List[str]] = {}
    for dev_num, file_list in imu_files.items():
        file_list.sort(key=lambda x: x[0])  # Sort by segment number
        result[dev_num] = [filepath for _, filepath in file_list]
    
    return result


def load_gyro_data(db_path: str) -> List[Tuple]:
    """
    Load gyro data from database.
    Returns list of tuples: (id, imuid_, x, y, z, timestamp)
    All values are integers, timestamp is 64-bit integer (nanoseconds).
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM gyro_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        result = []
        for row in rows:
            result.append((
                int(row[0]),  # id
                int(row[1]),  # imuid_
                int(row[2]),  # x
                int(row[3]),  # y
                int(row[4]),  # z
                int(row[5])   # timestamp (64-bit)
            ))
        conn.close()
        return result
    except sqlite3.Error as e:
        print(f"Error reading gyro_data from {db_path}: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)
    except (ValueError, TypeError) as e:
        print(f"Error converting gyro_data values from {db_path}: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)


def load_acc_data(db_path: str) -> List[Tuple]:
    """
    Load acc data from database.
    Returns list of tuples: (id, imuid_, x, y, z, timestamp)
    All values are integers, timestamp is 64-bit integer (nanoseconds).
    """
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM acc_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        result = []
        for row in rows:
            result.append((
                int(row[0]),  # id
                int(row[1]),  # imuid_
                int(row[2]),  # x
                int(row[3]),  # y
                int(row[4]),  # z
                int(row[5])   # timestamp (64-bit)
            ))
        conn.close()
        return result
    except sqlite3.Error as e:
        print(f"Error reading acc_data from {db_path}: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)
    except (ValueError, TypeError) as e:
        print(f"Error converting acc_data values from {db_path}: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)


def find_best_match(gyro_timestamp: int, acc_data: List[Tuple], sample_interval_ns: int, start_index: int = 0) -> Tuple[Optional[Tuple], int]:
    """
    Find the best matching acc_data for a given gyro timestamp.
    Uses binary search for efficiency since data is sorted by timestamp.
    All timestamps are 64-bit integers (nanoseconds).
    
    Args:
        gyro_timestamp: Timestamp of gyro data in nanoseconds (64-bit int)
        acc_data: List of acc data tuples (id, imuid_, x, y, z, timestamp)
        sample_interval_ns: Sample interval in nanoseconds (64-bit int)
        start_index: Starting index in acc_data for search (for optimization)
    
    Returns:
        Tuple of (best matching acc data tuple or None, next start index)
    """
    if not acc_data or start_index >= len(acc_data):
        return None, start_index
    
    # Binary search for the closest timestamp
    left = start_index
    right = len(acc_data) - 1
    best_match = None
    min_diff = None
    best_index = start_index
    
    while left <= right:
        mid = (left + right) // 2
        acc_timestamp = acc_data[mid][5]  # timestamp is the 6th element (index 5), 64-bit int
        diff = abs(acc_timestamp - gyro_timestamp)
        
        if min_diff is None or diff < min_diff:
            min_diff = diff
            best_match = acc_data[mid]
            best_index = mid
        
        if acc_timestamp < gyro_timestamp:
            left = mid + 1
        else:
            right = mid - 1
    
    # Check neighbors around the best match for even better match
    search_range = min(10, len(acc_data))
    start_check = max(start_index, best_index - search_range // 2)
    end_check = min(len(acc_data), best_index + search_range // 2 + 1)
    
    for i in range(start_check, end_check):
        acc_timestamp = acc_data[i][5]  # 64-bit int
        diff = abs(acc_timestamp - gyro_timestamp)
        if min_diff is None or diff < min_diff:
            min_diff = diff
            best_match = acc_data[i]
            best_index = i
    
    # Only return match if it's within sample interval
    if best_match is not None and min_diff is not None and min_diff <= sample_interval_ns:
        next_index = max(0, best_index - 1)
        return best_match, next_index
    
    # No match within sample interval, discard
    return None, best_index


def merge_data(gyro_data: List[Tuple], acc_data: List[Tuple], sampling_rate: int) -> List[Tuple]:
    """
    Merge gyro and acc data by timestamp.
    Uses optimal matching: finds the best matching acc for each gyro sample.
    
    Args:
        gyro_data: List of gyro data tuples
        acc_data: List of acc data tuples
        sampling_rate: IMU sampling rate in Hz
    
    Returns:
        List of merged data tuples: (timestamp, omega_x, omega_y, omega_z, alpha_x, alpha_y, alpha_z)
    """
    if not gyro_data:
        return []
    
    # Calculate sample interval in nanoseconds
    sample_interval_ns = 1000000000 // sampling_rate
    
    merged_data = []
    acc_index = 0
    
    sample_interval_ms = sample_interval_ns / 1000000.0
    print(f"Sample interval: {sample_interval_ns} ns ({sample_interval_ms:.2f} ms)")
    print(f"Processing {len(gyro_data)} gyro samples and {len(acc_data)} acc samples...")
    
    matched_count = 0
    discarded_count = 0
    
    for gyro in gyro_data:
        gyro_timestamp = gyro[5]  # timestamp is the 6th element (index 5)
        gyro_x, gyro_y, gyro_z = gyro[2], gyro[3], gyro[4]  # x, y, z
        
        # Find best matching acc data using optimized search
        best_acc, next_index = find_best_match(gyro_timestamp, acc_data, sample_interval_ns, acc_index)
        
        if best_acc:
            acc_x, acc_y, acc_z = best_acc[2], best_acc[3], best_acc[4]  # x, y, z
            
            # Use gyro timestamp as the reference timestamp
            merged_data.append((
                gyro_timestamp,
                gyro_x, gyro_y, gyro_z,
                acc_x, acc_y, acc_z
            ))
            matched_count += 1
            acc_index = next_index
        else:
            discarded_count += 1
    
    print(f"Matched: {matched_count}, Discarded: {discarded_count}")
    return merged_data


def write_csv(output_path: str, merged_data: List[Tuple]):
    """
    Write merged data to CSV file.
    Ensures all integers (especially 64-bit timestamps) are written as plain integers,
    not scientific notation.
    """
    in_anglvel_scales = 0.000266316
    in_acc_scales = 0.001197101

    try:
        with open(output_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(['timestamp', 'omega_x', 'omega_y', 'omega_z', 
                           'alpha_x', 'alpha_y', 'alpha_z'])
            
            # Write data
            for row in merged_data:
                writer.writerow([
                    int(row[0]),  # timestamp (64-bit)
                    float(row[1] * in_anglvel_scales),  # omega_x
                    float(row[2] * in_anglvel_scales),  # omega_y
                    float(row[3] * in_anglvel_scales),  # omega_z
                    float(row[4] * in_acc_scales),  # alpha_x
                    float(row[5] * in_acc_scales),  # alpha_y
                    float(row[6] * in_acc_scales)   # alpha_z
                ])
        
        print(f"Successfully wrote {len(merged_data)} rows to {output_path}")
    except IOError as e:
        print(f"Error writing CSV file: {e}", file=sys.stderr)
        sys.exit(1)


def process_imu_device(file_list: List[str], sampling_rate: int, output_path: str):
    """
    Process a single IMU device by merging data from multiple segment files.
    
    Args:
        file_list: List of database file paths, sorted by segment number
        sampling_rate: IMU sampling rate in Hz
        output_path: Output CSV file path
    """
    print(f"\nProcessing {len(file_list)} segment file(s)...")
    for i, file_path in enumerate(file_list, 1):
        print(f"  Segment {i}: {os.path.basename(file_path)}")
    
    # Load and concatenate data from all segments
    all_gyro_data = []
    all_acc_data = []
    
    for file_path in file_list:
        print(f"\nLoading data from {os.path.basename(file_path)}...")
        gyro_data = load_gyro_data(file_path)
        acc_data = load_acc_data(file_path)
        
        if gyro_data:
            all_gyro_data.extend(gyro_data)
            print(f"  Loaded {len(gyro_data)} gyro samples")
        if acc_data:
            all_acc_data.extend(acc_data)
            print(f"  Loaded {len(acc_data)} acc samples")
    
    if not all_gyro_data:
        print(f"Error: No gyro_data found in any database files", file=sys.stderr)
        return
    
    if not all_acc_data:
        print(f"Error: No acc_data found in any database files", file=sys.stderr)
        return
    
    # Sort combined data by timestamp (should already be sorted, but ensure it)
    all_gyro_data.sort(key=lambda x: x[5])
    all_acc_data.sort(key=lambda x: x[5])
    
    print(f"\nTotal loaded: {len(all_gyro_data)} gyro samples and {len(all_acc_data)} acc samples")
    
    # Merge data
    merged_data = merge_data(all_gyro_data, all_acc_data, sampling_rate)
    
    if not merged_data:
        print(f"Error: No data matched. Check timestamps and sampling rate.", file=sys.stderr)
        return
    
    # Write to CSV
    print(f"\nWriting merged data to {output_path}...")
    write_csv(output_path, merged_data)


def main():
    """Main function."""
    args = parse_arguments()
    
    # Validate sampling rate
    if args.rate <= 0:
        print("Error: Sampling rate must be greater than 0", file=sys.stderr)
        sys.exit(1)
    
    # Find IMU files
    print(f"Scanning directory: {args.input}")
    imu_files_dict = find_imu_files(args.input, args.imu_dev)
    
    if not imu_files_dict:
        print(f"Error: No matching IMU database files found in {args.input}", file=sys.stderr)
        sys.exit(1)
    
    # Process IMU devices
    if args.imu_dev is not None:
        # Process single IMU device
        if args.imu_dev not in imu_files_dict:
            print(f"Error: IMU device {args.imu_dev} not found", file=sys.stderr)
            sys.exit(1)
        
        file_list = imu_files_dict[args.imu_dev]
        process_imu_device(file_list, args.rate, args.output)
        print("\nDone!")
    else:
        # Process all IMU devices
        print(f"Found {len(imu_files_dict)} IMU device(s): {sorted(imu_files_dict.keys())}")
        
        for dev_num in sorted(imu_files_dict.keys()):
            file_list = imu_files_dict[dev_num]
            # Generate output filename
            output_path = args.output
            if not output_path.endswith('.csv'):
                output_path = f"{output_path}_IMU{dev_num}.csv"
            else:
                # Replace or append IMU number before .csv extension
                base, ext = os.path.splitext(output_path)
                output_path = f"{base}_IMU{dev_num}{ext}"
            
            print(f"\n{'='*60}")
            print(f"Processing IMU device {dev_num}")
            print(f"{'='*60}")
            process_imu_device(file_list, args.rate, output_path)
        
        print("\n" + "="*60)
        print("All IMU devices processed!")
        print("="*60)


if __name__ == '__main__':
    main()

