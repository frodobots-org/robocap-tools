#!/usr/bin/env python3
"""
Merge gyro_data and acc_data from SQLite database to CSV file.
Matches gyro and acc data by timestamp with optimal matching algorithm.
"""

import sqlite3
import csv
import argparse
import sys
from typing import List, Tuple, Optional


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Merge gyro_data and acc_data from SQLite database to CSV',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python merge_imu_data.py -i data.db -o output.csv -r 500
        """
    )
    parser.add_argument('-i', '--input', required=True,
                        help='Input SQLite database file path')
    parser.add_argument('-o', '--output', required=True,
                        help='Output CSV file path')
    parser.add_argument('-r', '--rate', type=int, required=True,
                        help='IMU sampling rate in Hz (e.g., 500)')
    
    return parser.parse_args()


def load_gyro_data(db_path: str) -> List[Tuple]:
    """
    Load gyro data from database.
    Returns list of tuples: (id, imuid_, x, y, z, timestamp)
    All values are integers, timestamp is 64-bit integer (nanoseconds).
    """
    conn = sqlite3.connect(db_path)
    # Ensure we get integers, not strings
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM gyro_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        # Convert to tuples with explicit integer conversion to ensure 64-bit handling
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
        print(f"Error reading gyro_data: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)
    except (ValueError, TypeError) as e:
        print(f"Error converting gyro_data values: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)


def load_acc_data(db_path: str) -> List[Tuple]:
    """
    Load acc data from database.
    Returns list of tuples: (id, imuid_, x, y, z, timestamp)
    All values are integers, timestamp is 64-bit integer (nanoseconds).
    """
    conn = sqlite3.connect(db_path)
    # Ensure we get integers, not strings
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute("SELECT id, imuid_, x, y, z, timestamp FROM acc_data ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        # Convert to tuples with explicit integer conversion to ensure 64-bit handling
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
        print(f"Error reading acc_data: {e}", file=sys.stderr)
        conn.close()
        sys.exit(1)
    except (ValueError, TypeError) as e:
        print(f"Error converting acc_data values: {e}", file=sys.stderr)
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
    # Use integer arithmetic throughout to maintain 64-bit precision
    left = start_index
    right = len(acc_data) - 1
    best_match = None
    min_diff = None  # Use None instead of float('inf') for integer comparison
    best_index = start_index
    
    # Find the closest timestamp using binary search
    # All operations use integer arithmetic to maintain 64-bit precision
    while left <= right:
        mid = (left + right) // 2
        acc_timestamp = acc_data[mid][5]  # timestamp is the 6th element (index 5), 64-bit int
        # Use integer subtraction, abs() handles both cases correctly for 64-bit ints
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
    # (in case binary search didn't find the absolute best)
    search_range = min(10, len(acc_data))  # Check up to 10 neighbors
    start_check = max(start_index, best_index - search_range // 2)
    end_check = min(len(acc_data), best_index + search_range // 2 + 1)
    
    for i in range(start_check, end_check):
        acc_timestamp = acc_data[i][5]  # 64-bit int
        diff = abs(acc_timestamp - gyro_timestamp)
        if min_diff is None or diff < min_diff:
            min_diff = diff
            best_match = acc_data[i]
            best_index = i
    
    # Only return match if it's within sample interval (integer comparison)
    if best_match is not None and min_diff is not None and min_diff <= sample_interval_ns:
        # Return the match and suggest next start index (slightly before best_index for next search)
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
    
    # Calculate sample interval in nanoseconds (ensure integer precision)
    # Use integer arithmetic to avoid floating point precision issues
    # For 500Hz: 1,000,000,000 / 500 = 2,000,000 ns = 2ms
    sample_interval_ns = 1000000000 // sampling_rate
    
    merged_data = []
    acc_index = 0  # Start from beginning of acc_data for optimization
    
    # Calculate sample interval in milliseconds for display (using float for display only)
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
            acc_index = next_index  # Update start index for next search
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
    try:
        with open(output_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(['timestamp', 'omega_x', 'omega_y', 'omega_z', 
                           'alpha_x', 'alpha_y', 'alpha_z'])
            
            # Write data - ensure all values are written as integers (not floats)
            for row in merged_data:
                # Convert all values to integers explicitly to ensure 64-bit precision
                # and avoid any scientific notation
                writer.writerow([
                    int(row[0]),  # timestamp (64-bit)
                    int(row[1]),  # omega_x
                    int(row[2]),  # omega_y
                    int(row[3]),  # omega_z
                    int(row[4]),  # alpha_x
                    int(row[5]),  # alpha_y
                    int(row[6])   # alpha_z
                ])
        
        print(f"Successfully wrote {len(merged_data)} rows to {output_path}")
    except IOError as e:
        print(f"Error writing CSV file: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    """Main function."""
    args = parse_arguments()
    
    # Validate sampling rate
    if args.rate <= 0:
        print("Error: Sampling rate must be greater than 0", file=sys.stderr)
        sys.exit(1)
    
    # Load data from database
    print(f"Loading data from {args.input}...")
    gyro_data = load_gyro_data(args.input)
    acc_data = load_acc_data(args.input)
    
    if not gyro_data:
        print("Error: No gyro_data found in database", file=sys.stderr)
        sys.exit(1)
    
    if not acc_data:
        print("Error: No acc_data found in database", file=sys.stderr)
        sys.exit(1)
    
    print(f"Loaded {len(gyro_data)} gyro samples and {len(acc_data)} acc samples")
    
    # Merge data
    merged_data = merge_data(gyro_data, acc_data, args.rate)
    
    if not merged_data:
        print("Error: No data matched. Check timestamps and sampling rate.", file=sys.stderr)
        sys.exit(1)
    
    # Write to CSV
    print(f"Writing merged data to {args.output}...")
    write_csv(args.output, merged_data)
    
    print("Done!")


if __name__ == '__main__':
    main()

