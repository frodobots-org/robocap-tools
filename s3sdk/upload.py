#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to upload calibration results to S3.
Uploads the results directory for a specific device ID.
"""

import argparse
import os
import sys
from pathlib import Path
from typing import List, Optional

# Add s3sdk to path
project_root = Path(__file__).parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from s3_sdk import S3Config, S3SDK
from config_reader import load_s3_config


def upload_results_folder(
    device_id: str,
    results_path: str,
    config_file: str = 's3_config.json',
    s3_base_path: Optional[str] = None,
    dry_run: bool = False
) -> None:
    """
    Upload calibration results folder to S3.

    Args:
        device_id: Device ID
        results_path: Local path to results directory (e.g., /data/{device_id}/v1/results)
        config_file: Path to S3 configuration file
        s3_base_path: Base S3 path (default: {device_id}/v1/results)
        dry_run: If True, only show what would be uploaded without actually uploading
    """
    device_id = device_id.strip()
    if not device_id:
        print("Error: Device ID cannot be empty")
        return

    # Check if results directory exists
    results_dir = Path(results_path)
    if not results_dir.exists():
        print(f"Error: Results directory does not exist: {results_path}")
        return

    if not results_dir.is_dir():
        print(f"Error: Path is not a directory: {results_path}")
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

    # Determine S3 base path
    if s3_base_path is None:
        s3_base_path = f"{device_id}/v1/results"
    
    # Ensure S3 path ends with /
    if not s3_base_path.endswith('/'):
        s3_base_path += '/'

    print(f"{'='*80}")
    print(f"Uploading calibration results")
    print(f"{'='*80}")
    print(f"Device ID: {device_id}")
    print(f"Local path: {results_dir.absolute()}")
    print(f"S3 path: s3://{config.bucket_name}/{s3_base_path}")
    if dry_run:
        print(f"Mode: DRY RUN (no files will be uploaded)")
    print(f"{'='*80}\n")

    # Collect all files to upload
    files_to_upload = []
    for root, dirs, files in os.walk(results_dir):
        for file in files:
            local_file_path = Path(root) / file
            # Calculate relative path from results directory
            relative_path = local_file_path.relative_to(results_dir)
            # Convert to S3 path format (use forward slashes)
            s3_file_path = f"{s3_base_path}{str(relative_path).replace(os.sep, '/')}"
            files_to_upload.append((str(local_file_path), s3_file_path))

    if not files_to_upload:
        print("No files found in results directory")
        return

    print(f"Found {len(files_to_upload)} file(s) to upload\n")

    if dry_run:
        print("Files that would be uploaded:")
        for local_path, s3_path in files_to_upload:
            file_size = os.path.getsize(local_path)
            size_str = format_size(file_size)
            print(f"  {local_path}")
            print(f"    -> s3://{config.bucket_name}/{s3_path} ({size_str})")
        print(f"\nTotal: {len(files_to_upload)} files")
        return

    # Upload files
    success_count = 0
    fail_count = 0
    total_size = 0

    for i, (local_path, s3_path) in enumerate(files_to_upload, 1):
        try:
            file_size = os.path.getsize(local_path)
            total_size += file_size
            size_str = format_size(file_size)
            
            print(f"[{i}/{len(files_to_upload)}] Uploading: {os.path.basename(local_path)} ({size_str})")
            print(f"  Local: {local_path}")
            print(f"  S3: s3://{config.bucket_name}/{s3_path}")
            
            success = sdk.upload_file(local_path, s3_path)
            
            if success:
                print(f"  ✓ Upload successful\n")
                success_count += 1
            else:
                print(f"  ✗ Upload failed\n")
                fail_count += 1
                
        except Exception as e:
            print(f"  ✗ Error uploading {local_path}: {e}\n")
            fail_count += 1
            continue

    # Summary
    print(f"{'='*80}")
    print(f"Upload Summary:")
    print(f"  Total files: {len(files_to_upload)}")
    print(f"  Successful: {success_count}")
    print(f"  Failed: {fail_count}")
    print(f"  Total size: {format_size(total_size)}")
    print(f"  S3 location: s3://{config.bucket_name}/{s3_base_path}")
    print(f"{'='*80}")


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
        description='Upload calibration results to S3',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Upload results for a device (default path: /data/{device_id}/v1/results)
  python upload.py --device-id faf2a598869ccfc8

  # Upload with custom results path
  python upload.py --device-id faf2a598869ccfc8 --results-path /data/faf2a598869ccfc8/v1/results

  # Dry run (show what would be uploaded without actually uploading)
  python upload.py --device-id faf2a598869ccfc8 --dry-run

  # Custom S3 base path
  python upload.py --device-id faf2a598869ccfc8 --s3-base-path "custom/path/results"
        """
    )
    
    parser.add_argument(
        '--device-id',
        type=str,
        required=True,
        help='Device ID'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='s3_config.json',
        help='Path to S3 configuration file (default: s3_config.json)'
    )
    
    parser.add_argument(
        '--results-path',
        type=str,
        default=None,
        help='Local path to results directory (default: /data/{device_id}/v1/results)'
    )
    
    parser.add_argument(
        '--s3-base-path',
        type=str,
        default=None,
        help='Base S3 path (default: {device_id}/v1/results)'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Dry run mode: show what would be uploaded without actually uploading'
    )
    
    args = parser.parse_args()
    
    # Determine results path
    if args.results_path is None:
        # Default path: /data/{device_id}/v1/results
        results_path = f"/data/{args.device_id}/v1/results"
    else:
        results_path = args.results_path
    
    upload_results_folder(
        device_id=args.device_id,
        results_path=results_path,
        config_file=args.config,
        s3_base_path=args.s3_base_path,
        dry_run=args.dry_run
    )


if __name__ == '__main__':
    main()

