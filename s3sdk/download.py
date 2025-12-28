"""
Script to download device data from S3 based on device list from Excel file.
"""

import argparse
import os
from pathlib import Path
from typing import List, Optional

from device_list_reader import read_device_list
from s3_sdk import S3Config, S3SDK
from config_reader import load_s3_config


def download_single_device(
    device_id: str,
    output_dir: str,
    access_key: str,
    secret_key: str,
    bucket_name: str,
    region: str = 'us-east-1',
    device_prefix: str = '',
    connect_timeout: Optional[int] = None,
    read_timeout: Optional[int] = None
) -> None:
    """
    Download data for a single device from S3.

    Args:
        device_id: Device ID to download
        output_dir: Output directory to save downloaded data
        access_key: AWS Access Key ID
        secret_key: AWS Secret Access Key
        bucket_name: S3 bucket name
        region: AWS region (default: 'us-east-1')
        device_prefix: Prefix to add before device ID in S3 path (default: '')
        connect_timeout: Connection timeout in seconds
        read_timeout: Read timeout in seconds
    """
    device_id = device_id.strip()
    if not device_id:
        print("Error: Device ID cannot be empty")
        return

    # Initialize S3 SDK
    config = S3Config(
        access_key=access_key,
        secret_key=secret_key,
        bucket_name=bucket_name,
        region=region,
        connect_timeout=connect_timeout if connect_timeout is not None else 60,
        read_timeout=read_timeout if read_timeout is not None else 60
    )
    sdk = S3SDK(config)

    # Create output directory if it doesn't exist
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_path.absolute()}")

    print(f"\nProcessing device: {device_id}")

    # Construct S3 folder path
    s3_folder = f"{device_prefix}{device_id}/"
    if device_prefix and not device_prefix.endswith('/'):
        s3_folder = f"{device_prefix}/{device_id}/"

    # Construct local output path
    local_device_dir = output_path / device_id

    try:
        # Download folder from S3 (will list all objects with prefix and download them)
        print(f"  Downloading from S3: {s3_folder}")
        print(f"  Saving to: {local_device_dir}")
        results = sdk.download_folder(s3_folder, str(local_device_dir))

        # Count successful downloads
        successful = sum(1 for _, _, success in results if success)
        total = len(results)

        if total > 0:
            print(f"  Downloaded {successful}/{total} files")
            if successful > 0:
                print(f"\n{'='*60}")
                print(f"Download Summary:")
                print(f"  Device: {device_id}")
                print(f"  Status: Success")
                print(f"  Files downloaded: {successful}/{total}")
                print(f"  Output directory: {output_path.absolute()}")
                print(f"{'='*60}")
            else:
                print(f"\n{'='*60}")
                print(f"Download Summary:")
                print(f"  Device: {device_id}")
                print(f"  Status: Failed (no files downloaded)")
                print(f"  Output directory: {output_path.absolute()}")
                print(f"{'='*60}")
        else:
            print(f"  No files found with prefix '{s3_folder}'")
            print(f"\n{'='*60}")
            print(f"Download Summary:")
            print(f"  Device: {device_id}")
            print(f"  Status: Failed (no files found)")
            print(f"  Output directory: {output_path.absolute()}")
            print(f"{'='*60}")

    except Exception as e:
        print(f"  Error downloading device {device_id}: {e}")
        print(f"\n{'='*60}")
        print(f"Download Summary:")
        print(f"  Device: {device_id}")
        print(f"  Status: Failed")
        print(f"  Error: {e}")
        print(f"  Output directory: {output_path.absolute()}")
        print(f"{'='*60}")


def download_device_data(
    xlsx_filename: str,
    output_dir: str,
    access_key: str,
    secret_key: str,
    bucket_name: str,
    region: str = 'us-east-1',
    sheet_name: Optional[str] = None,
    column: int = 0,
    header: Optional[int] = None,
    device_prefix: str = '',
    connect_timeout: Optional[int] = None,
    read_timeout: Optional[int] = None
) -> None:
    """
    Download device data from S3 based on device list in Excel file.

    Args:
        xlsx_filename: Path to Excel file containing device list
        output_dir: Output directory to save downloaded data
        access_key: AWS Access Key ID
        secret_key: AWS Secret Access Key
        bucket_name: S3 bucket name
        region: AWS region (default: 'us-east-1')
        sheet_name: Sheet name in Excel file (default: None for first sheet)
        column: Column index containing device IDs (default: 0)
        header: Header row index (default: None for no header)
        device_prefix: Prefix to add before device ID in S3 path (default: '')
        connect_timeout: Connection timeout in seconds
        read_timeout: Read timeout in seconds
    """
    print(f"Reading device list from: {xlsx_filename}")
    
    # Read device list from Excel file
    try:
        devices = read_device_list(
            xlsx_filename,
            sheet_name=sheet_name,
            column=column,
            header=header,
            skip_empty=True
        )
        print(f"Found {len(devices)} devices")
    except Exception as e:
        print(f"Error reading device list: {e}")
        return

    if not devices:
        print("No devices found in the Excel file")
        return

    # Initialize S3 SDK
    config = S3Config(
        access_key=access_key,
        secret_key=secret_key,
        bucket_name=bucket_name,
        region=region,
        connect_timeout=connect_timeout if connect_timeout is not None else 60,
        read_timeout=read_timeout if read_timeout is not None else 60
    )
    sdk = S3SDK(config)

    # Create output directory if it doesn't exist
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    print(f"Output directory: {output_path.absolute()}")

    # Download data for each device
    success_count = 0
    fail_count = 0

    for i, device_id in enumerate(devices, 1):
        device_id = device_id.strip()
        if not device_id:
            continue

        print(f"\n[{i}/{len(devices)}] Processing device: {device_id}")

        # Construct S3 folder path
        s3_folder = f"{device_prefix}{device_id}/"
        if device_prefix and not device_prefix.endswith('/'):
            s3_folder = f"{device_prefix}/{device_id}/"

        # Construct local output path
        local_device_dir = output_path / device_id

        try:
            # Download folder from S3 (will list all objects with prefix and download them)
            print(f"  Downloading from S3: {s3_folder}")
            print(f"  Saving to: {local_device_dir}")
            results = sdk.download_folder(s3_folder, str(local_device_dir))

            # Count successful downloads
            successful = sum(1 for _, _, success in results if success)
            total = len(results)

            if total > 0:
                print(f"  Downloaded {successful}/{total} files")
                if successful > 0:
                    success_count += 1
                else:
                    fail_count += 1
            else:
                print(f"  No files found with prefix '{s3_folder}'")
                fail_count += 1

        except Exception as e:
            print(f"  Error downloading device {device_id}: {e}")
            fail_count += 1
            continue

    # Summary
    print(f"\n{'='*60}")
    print(f"Download Summary:")
    print(f"  Total devices: {len(devices)}")
    print(f"  Successful: {success_count}")
    print(f"  Failed: {fail_count}")
    print(f"  Output directory: {output_path.absolute()}")
    print(f"{'='*60}")


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description='Download device data from S3. Can download from Excel file or single device ID.'
    )
    
    parser.add_argument(
        '--xlsx-file',
        type=str,
        default=None,
        help='Path to Excel file containing device list (optional if --device-id is provided)'
    )
    
    parser.add_argument(
        '--device-id',
        type=str,
        default=None,
        help='Single device ID to download (optional if --xlsx-file is provided)'
    )
    
    parser.add_argument(
        'output_dir',
        type=str,
        help='Output directory to save downloaded data'
    )
    
    parser.add_argument(
        '--config',
        type=str,
        default='s3_config.json',
        help='Path to S3 configuration file (default: s3_config.json)'
    )
    
    parser.add_argument(
        '--access-key',
        type=str,
        default=None,
        help='AWS Access Key ID (overrides config file)'
    )
    
    parser.add_argument(
        '--secret-key',
        type=str,
        default=None,
        help='AWS Secret Access Key (overrides config file)'
    )
    
    parser.add_argument(
        '--bucket',
        type=str,
        default=None,
        help='S3 bucket name (overrides config file)'
    )
    
    parser.add_argument(
        '--region',
        type=str,
        default=None,
        help='AWS region (overrides config file)'
    )
    
    parser.add_argument(
        '--sheet-name',
        type=str,
        default=None,
        help='Sheet name in Excel file (default: first sheet)'
    )
    
    parser.add_argument(
        '--column',
        type=int,
        default=0,
        help='Column index containing device IDs (0-based, default: 0)'
    )
    
    parser.add_argument(
        '--header',
        type=int,
        default=None,
        help='Header row index (default: None for no header)'
    )
    
    parser.add_argument(
        '--device-prefix',
        type=str,
        default='',
        help='Prefix to add before device ID in S3 path (default: empty)'
    )
    
    parser.add_argument(
        '--connect-timeout',
        type=int,
        default=None,
        help='Connection timeout in seconds (default: 60, or from config file)'
    )
    
    parser.add_argument(
        '--read-timeout',
        type=int,
        default=None,
        help='Read timeout in seconds (default: 60, or from config file)'
    )
    
    args = parser.parse_args()
    
    # Validate arguments: must provide either xlsx_file or device_id
    if not args.xlsx_file and not args.device_id:
        parser.error("Either --xlsx-file or --device-id must be provided")
    
    if args.xlsx_file and args.device_id:
        parser.error("Cannot specify both --xlsx-file and --device-id. Please choose one.")
    
    # Load configuration from file
    try:
        config = load_s3_config(args.config)
        access_key = args.access_key or config.access_key
        secret_key = args.secret_key or config.secret_key
        bucket_name = args.bucket or config.bucket_name
        region = args.region or config.region
        # Use command line timeout if provided, otherwise use config file value, otherwise default to 60
        connect_timeout = args.connect_timeout if args.connect_timeout is not None else config.connect_timeout
        read_timeout = args.read_timeout if args.read_timeout is not None else config.read_timeout
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nPlease create a configuration file or provide credentials via command line arguments.")
        return
    
    # Download based on provided input
    if args.device_id:
        # Download single device
        download_single_device(
            device_id=args.device_id,
            output_dir=args.output_dir,
            access_key=access_key,
            secret_key=secret_key,
            bucket_name=bucket_name,
            region=region,
            device_prefix=args.device_prefix,
            connect_timeout=connect_timeout,
            read_timeout=read_timeout
        )
    else:
        # Download from Excel file
        download_device_data(
            xlsx_filename=args.xlsx_file,
            output_dir=args.output_dir,
            access_key=access_key,
            secret_key=secret_key,
            bucket_name=bucket_name,
            region=region,
            sheet_name=args.sheet_name,
            column=args.column,
            header=args.header,
            device_prefix=args.device_prefix,
            connect_timeout=connect_timeout,
            read_timeout=read_timeout
        )


if __name__ == '__main__':
    main()

