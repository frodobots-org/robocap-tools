"""
S3 SDK - A Python SDK for AWS S3 operations using boto3.
"""

from .config import S3Config
from .s3_sdk import S3SDK
from .exceptions import (
    S3SDKError,
    S3ConfigError,
    S3OperationError,
    S3FileNotFoundError
)

__all__ = [
    'S3Config',
    'S3SDK',
    'S3SDKError',
    'S3ConfigError',
    'S3OperationError',
    'S3FileNotFoundError',
]

__version__ = '1.0.0'

