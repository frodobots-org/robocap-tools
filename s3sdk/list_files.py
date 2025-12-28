#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to list files in S3 for a specific device ID.
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

# Add s3sdk to path
project_root = Path(__file__).parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from s3_sdk import S3Config, S3SDK
from config_reader import load_s3_config


def list_device_files(
    device_id: str,
    config_file: str = 's3_config.json',
    device_prefix: str = '',
    recursive: bool = True,
    show_details: bool = False
) -> None:
    """
    List all files in S3 for a specific device ID.

    Args:
        device_id: Device ID to list files for
        config_file: Path to S3 configuration file
        device_prefix: Prefix to add before device ID in S3 path (default: '')
        recursive: If True, list files recursively (default: True)
        show_details: If True, show file size and last modified time (default: False)
    """
    device_id = device_id.strip()
    if not device_id:
        print("Error: Device ID cannot be empty")
        return

    # Load configuration
    try:
        config = load_s3_config(config_file)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nPlease create a configuration file or provide the correct path.")
        return
    except ValueError as e:
        print(f"Error: {e}")
        return

    # Initialize S3 SDK
    try:
        sdk = S3SDK(config)
    except Exception as e:
        print(f"Error initializing S3 SDK: {e}")
        return

    # Construct S3 folder path
    s3_folder = f"{device_prefix}{device_id}/"
    if device_prefix and not device_prefix.endswith('/'):
        s3_folder = f"{device_prefix}/{device_id}/"

    print(f"{'='*80}")
    print(f"Listing files for device: {device_id}")
    print(f"S3 path: {s3_folder}")
    print(f"Recursive: {recursive}")
    print(f"{'='*80}\n")

    try:
        if show_details:
            # List all objects with details
            objects = sdk.list_all_objects(prefix=s3_folder, recursive=recursive, include_folders=False)
            
            if not objects:
                print(f"No files found with prefix '{s3_folder}'")
                return

            # Sort by key (path)
            objects.sort(key=lambda x: x.get('Key', ''))

            print(f"Found {len(objects)} file(s):\n")
            
            # Calculate total size
            total_size = 0
            
            for obj in objects:
                key = obj.get('Key', '')
                size = obj.get('Size', 0)
                last_modified = obj.get('LastModified', '')
                
                total_size += size
                
                # Get relative path (remove the folder prefix)
                relative_path = key[len(s3_folder):] if key.startswith(s3_folder) else key
                
                # Format size
                size_str = format_size(size)
                
                # Format last modified
                if last_modified:
                    modified_str = last_modified.strftime('%Y-%m-%d %H:%M:%S')
                else:
                    modified_str = 'N/A'
                
                print(f"  {relative_path}")
                print(f"    Size: {size_str}")
                print(f"    Last Modified: {modified_str}")
                print()
            
            # Print summary
            print(f"{'='*80}")
            print(f"Summary:")
            print(f"  Total files: {len(objects)}")
            print(f"  Total size: {format_size(total_size)}")
            print(f"{'='*80}")
        else:
            # List files only (simple mode)
            files = sdk.list_files(folder_path=s3_folder, recursive=recursive)
            
            if not files:
                print(f"No files found with prefix '{s3_folder}'")
                return

            # Sort files
            files.sort()
            
            print(f"Found {len(files)} file(s):\n")
            
            for file_path in files:
                # Get relative path (remove the folder prefix)
                relative_path = file_path[len(s3_folder):] if file_path.startswith(s3_folder) else file_path
                print(f"  {relative_path}")
            
            print(f"\n{'='*80}")
            print(f"Total files: {len(files)}")
            print(f"{'='*80}")

    except Exception as e:
        print(f"Error listing files: {e}")
        return


def format_size(size_bytes: int) -> str:
    """
    Format file size in human-readable format.

    Args:
        size_bytes: Size in bytes

    Returns:
        Formatted size string (e.g., "1.5 MB")
    """
    if size_bytes == 0:
        return "0 B"
    
    units = ['B', 'KB', 'MB', 'GB', 'TB']
    unit_index = 0
    size = float(size_bytes)
    
    while size >= 1024 and unit_index < len(units) - 1:
        size /= 1024
        unit_index += 1
    
    if unit_index == 0:
        return f"{int(size)} {units[unit_index]}"
    else:
        return f"{size:.2f} {units[unit_index]}"


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description='List files in S3 for a specific device ID',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # List files for a device (simple mode)
  python list_files.py faf2a598869ccfc8

  # List files with details (size and last modified time)
  python list_files.py faf2a598869ccfc8 --details

  # List files with custom config and prefix
  python list_files.py faf2a598869ccfc8 --config /path/to/s3_config.json --device-prefix "prefix/"
        """
    )
    
    parser.add_argument(
        'device_id',
        type=str,
        help='Device ID to list files for'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='s3_config.json',
        help='Path to S3 configuration file (default: s3_config.json)'
    )
    
    parser.add_argument(
        '--device-prefix',
        type=str,
        default='',
        help='Prefix to add before device ID in S3 path (default: empty)'
    )
    
    parser.add_argument(
        '--no-recursive',
        action='store_true',
        help='List files non-recursively (only direct children)'
    )
    
    parser.add_argument(
        '--details',
        action='store_true',
        help='Show file details (size and last modified time)'
    )
    
    args = parser.parse_args()
    
    list_device_files(
        device_id=args.device_id,
        config_file=args.config,
        device_prefix=args.device_prefix,
        recursive=not args.no_recursive,
        show_details=args.details
    )


if __name__ == '__main__':
    main()

