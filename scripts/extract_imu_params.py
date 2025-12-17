#!/usr/bin/env python3
"""
Extract IMU parameters from imu{n}_imu_param.yaml file and convert to kalibr format.
"""

import argparse
import os
import re
import sys
import yaml
from pathlib import Path
from typing import Optional


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Extract IMU parameters from yaml file and convert to kalibr format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Process all IMU devices
  python extract_imu_params.py -i /path/to/search/dir -o /path/to/output/dir
  
  # Process specific IMU device
  python extract_imu_params.py -i /path/to/search/dir -o /path/to/output/dir -d 0
        """
    )
    parser.add_argument('-i', '--input', required=True,
                        help='Input directory to search for imu{n}_imu_param.yaml files')
    parser.add_argument('-o', '--output', required=True,
                        help='Output directory for converted parameter files')
    parser.add_argument('-d', '--imu-dev', type=int, choices=[0, 1, 2],
                        help='IMU device number to process (0, 1, or 2). If not specified, process all available IMU devices')
    
    return parser.parse_args()


def find_imu_param_files(search_dir: str, imu_dev: Optional[int] = None):
    """
    Find all imu{n}_imu_param.yaml files in the directory.
    
    Args:
        search_dir: Directory path to search
        imu_dev: Optional IMU device number (0, 1, or 2) to filter
    
    Returns:
        List of tuples: (imu_number, file_path)
    """
    search_path = Path(search_dir)
    if not search_path.is_dir():
        print(f"Error: {search_dir} is not a directory", file=sys.stderr)
        sys.exit(1)
    
    # Pattern to match: imu{n}_imu_param.yaml
    pattern = re.compile(r'imu(\d+)_imu_param\.yaml$')
    
    imu_files = []
    
    # Search recursively in the directory
    for file_path in search_path.rglob('imu*_imu_param.yaml'):
        match = pattern.match(file_path.name)
        if match:
            imu_num = int(match.group(1))
            
            # Filter by imu_dev if specified
            if imu_dev is not None and imu_num != imu_dev:
                continue
            
            imu_files.append((imu_num, str(file_path)))
    
    # Sort by IMU number
    imu_files.sort(key=lambda x: x[0])
    
    return imu_files


def load_yaml_file(file_path: str):
    """
    Load YAML file and return parsed data.
    
    Args:
        file_path: Path to YAML file
    
    Returns:
        Parsed YAML data as dictionary
    """
    try:
        with open(file_path, 'r') as f:
            content = f.read()
            # Remove YAML directive line if present (e.g., %YAML:1.0)
            # This directive can cause parsing issues with some YAML parsers
            lines = content.split('\n')
            filtered_lines = []
            for line in lines:
                # Skip YAML directive lines (starting with %)
                if line.strip().startswith('%'):
                    continue
                filtered_lines.append(line)
            content = '\n'.join(filtered_lines)
            data = yaml.safe_load(content)
        return data
    except FileNotFoundError:
        print(f"Error: File not found: {file_path}", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error: Failed to parse YAML file {file_path}: {e}", file=sys.stderr)
        sys.exit(1)


def extract_avg_axis_values(data: dict):
    """
    Extract avg-axis values from IMU parameter data.
    
    Args:
        data: Parsed YAML data
    
    Returns:
        Dictionary with extracted values: {'gyr_n', 'gyr_w', 'acc_n', 'acc_w', 'imu_name'}
    """
    try:
        # Extract IMU name (e.g., imu0)
        imu_name = data.get('name', 'imu0')
        
        # Extract gyroscope avg-axis values
        gyr_data = data.get('Gyr', {})
        avg_axis_gyr = gyr_data.get('avg-axis', {})
        gyr_n = avg_axis_gyr.get('gyr_n')
        gyr_w = avg_axis_gyr.get('gyr_w')
        
        # Extract accelerometer avg-axis values
        acc_data = data.get('Acc', {})
        avg_axis_acc = acc_data.get('avg-axis', {})
        acc_n = avg_axis_acc.get('acc_n')
        acc_w = avg_axis_acc.get('acc_w')
        
        if gyr_n is None or gyr_w is None or acc_n is None or acc_w is None:
            raise ValueError("Missing required avg-axis values in YAML file")
        
        return {
            'gyr_n': gyr_n,
            'gyr_w': gyr_w,
            'acc_n': acc_n,
            'acc_w': acc_w,
            'imu_name': imu_name
        }
    except (KeyError, AttributeError) as e:
        print(f"Error: Failed to extract avg-axis values: {e}", file=sys.stderr)
        sys.exit(1)


def write_kalibr_format(output_path: str, values: dict, update_rate: float = 500.0):
    """
    Write extracted values to kalibr format file.
    
    Args:
        output_path: Output file path
        values: Dictionary with extracted values
        update_rate: IMU update rate in Hz
    """
    try:
        with open(output_path, 'w') as f:
            f.write("#Accelerometers\n")
            f.write(f"accelerometer_noise_density: {values['acc_n']:.12e}   #Noise density (continuous-time)\n")
            f.write(f"accelerometer_random_walk:   {values['acc_w']:.12e}   #Bias random walk\n")
            f.write("\n")
            f.write("#Gyroscopes\n")
            f.write(f"gyroscope_noise_density:     {values['gyr_n']:.12e}   #Noise density (continuous-time)\n")
            f.write(f"gyroscope_random_walk:       {values['gyr_w']:.12e}   #Bias random walk\n")
            f.write("\n")
            f.write(f"rostopic:                    /{values['imu_name']}      #the IMU ROS topic\n")
            f.write(f"update_rate:                 {update_rate:.1f}      #Hz (for discretization of the values above)\n")
        
        print(f"Successfully wrote converted parameters to {output_path}")
    except IOError as e:
        print(f"Error: Failed to write output file {output_path}: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    """Main function."""
    args = parse_arguments()
    
    # Find IMU parameter files
    if args.imu_dev is not None:
        print(f"Searching for imu{args.imu_dev}_imu_param.yaml files in {args.input}...")
    else:
        print(f"Searching for imu*_imu_param.yaml files in {args.input}...")
    imu_files = find_imu_param_files(args.input, args.imu_dev)
    
    if not imu_files:
        if args.imu_dev is not None:
            print(f"Error: No imu{args.imu_dev}_imu_param.yaml files found in {args.input}", file=sys.stderr)
        else:
            print(f"Error: No imu*_imu_param.yaml files found in {args.input}", file=sys.stderr)
        sys.exit(1)
    
    print(f"Found {len(imu_files)} IMU parameter file(s)")
    
    # Create output directory if it doesn't exist
    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Process each file
    for imu_num, file_path in imu_files:
        print(f"\nProcessing: {os.path.basename(file_path)}")
        
        # Load YAML file
        data = load_yaml_file(file_path)
        
        # Extract avg-axis values
        values = extract_avg_axis_values(data)
        
        # Generate output filename (same as input filename)
        output_filename = os.path.basename(file_path)
        output_file_path = output_path / output_filename
        
        # Write converted format
        write_kalibr_format(str(output_file_path), values)
    
    print(f"\nDone! Processed {len(imu_files)} file(s)")


if __name__ == '__main__':
    main()

