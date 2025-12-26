"""
Configuration reader for S3 settings.
"""

import json
import os
from pathlib import Path
from typing import Dict, Optional

from s3_sdk import S3Config


def load_s3_config(config_file: str = 's3_config.json') -> S3Config:
    """
    Load S3 configuration from JSON file.

    Args:
        config_file: Path to configuration JSON file (default: 's3_config.json')

    Returns:
        S3Config instance

    Raises:
        FileNotFoundError: If configuration file doesn't exist
        ValueError: If configuration file is invalid or missing required fields

    Example:
        config = load_s3_config('s3_config.json')
        sdk = S3SDK(config)
    """
    config_path = Path(config_file)

    if not config_path.exists():
        raise FileNotFoundError(
            f"Configuration file not found: {config_file}\n"
            f"Please create a configuration file with the following structure:\n"
            f'{{\n'
            f'    "access_key": "YOUR_ACCESS_KEY",\n'
            f'    "secret_key": "YOUR_SECRET_KEY",\n'
            f'    "bucket_name": "YOUR_BUCKET_NAME",\n'
            f'    "region": "us-east-1"\n'
            f'}}'
        )

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = json.load(f)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in configuration file: {e}")

    # Validate required fields
    required_fields = ['access_key', 'secret_key', 'bucket_name', 'region']
    missing_fields = [field for field in required_fields if field not in config_data]

    if missing_fields:
        raise ValueError(
            f"Missing required fields in configuration file: {', '.join(missing_fields)}\n"
            f"Required fields: {', '.join(required_fields)}"
        )

    # Create S3Config instance
    config = S3Config(
        access_key=config_data['access_key'],
        secret_key=config_data['secret_key'],
        bucket_name=config_data['bucket_name'],
        region=config_data.get('region', 'us-east-1')
    )

    return config


def get_s3_config_dict(config_file: str = 's3_config.json') -> Dict[str, str]:
    """
    Load S3 configuration as dictionary from JSON file.

    Args:
        config_file: Path to configuration JSON file (default: 's3_config.json')

    Returns:
        Dictionary containing S3 configuration

    Raises:
        FileNotFoundError: If configuration file doesn't exist
        ValueError: If configuration file is invalid or missing required fields
    """
    config_path = Path(config_file)

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_file}")

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = json.load(f)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in configuration file: {e}")

    # Validate required fields
    required_fields = ['access_key', 'secret_key', 'bucket_name', 'region']
    missing_fields = [field for field in required_fields if field not in config_data]

    if missing_fields:
        raise ValueError(
            f"Missing required fields in configuration file: {', '.join(missing_fields)}"
        )

    return {
        'access_key': config_data['access_key'],
        'secret_key': config_data['secret_key'],
        'bucket_name': config_data['bucket_name'],
        'region': config_data.get('region', 'us-east-1')
    }

