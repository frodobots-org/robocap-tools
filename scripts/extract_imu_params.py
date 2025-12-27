#!/usr/bin/env python3
"""
Extract IMU parameters from imu{n}_imu_param.yaml file and convert to kalibr format.
"""

import argparse
import os
import sys
import yaml
from pathlib import Path

# Add script directory to path to ensure robocap_env can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Import environment variables
try:
    import robocap_env
except ImportError:
    print("Warning: Could not import robocap_env, using default topic names", file=sys.stderr)
    robocap_env = None


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Extract IMU parameters from yaml file and convert to kalibr format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  # Process single file
  python extract_imu_params.py -i /path/to/imu0_imu_param.yaml -o /path/to/output.yaml
        """
    )
    parser.add_argument('-i', '--input', required=True,
                        help='Input YAML file path (e.g., imu0_imu_param.yaml)')
    parser.add_argument('-o', '--output', required=True,
                        help='Output YAML file path for converted parameters')
    
    return parser.parse_args()




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


def write_kalibr_format(output_path: str, values: dict, update_rate: float = 200.0):
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
            # Get ROS topic from robocap_env based on imu_name
            if robocap_env is not None:
                ros_topic = robocap_env.get_imu_topic_from_name(values['imu_name'])
            else:
                # Fallback: use imu_name directly
                ros_topic = f"/{values['imu_name']}"
            f.write(f"rostopic:                    {ros_topic}      #the IMU ROS topic\n")
            f.write(f"update_rate:                 {update_rate:.1f}      #Hz (for discretization of the values above)\n")
        
        print(f"Successfully wrote converted parameters to {output_path}")
    except IOError as e:
        print(f"Error: Failed to write output file {output_path}: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    """Main function."""
    args = parse_arguments()
    
    # Check if input file exists
    input_path = Path(args.input)
    if not input_path.is_file():
        print(f"Error: Input file does not exist: {args.input}", file=sys.stderr)
        sys.exit(1)
    
    print(f"Processing: {args.input}")
    
    # Load YAML file
    data = load_yaml_file(str(input_path))
    
    # Extract avg-axis values
    values = extract_avg_axis_values(data)
    
    # Create output directory if it doesn't exist
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Write converted format
    write_kalibr_format(str(output_path), values)
    
    print(f"\nDone! Output written to {args.output}")


if __name__ == '__main__':
    main()

