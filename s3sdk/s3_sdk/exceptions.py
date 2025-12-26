"""
Custom exceptions for S3 SDK.
"""


class S3SDKError(Exception):
    """Base exception for S3 SDK errors."""
    pass


class S3ConfigError(S3SDKError):
    """Raised when configuration is invalid."""
    pass


class S3OperationError(S3SDKError):
    """Raised when an S3 operation fails."""
    pass


class S3FileNotFoundError(S3SDKError):
    """Raised when a file or folder is not found in S3."""
    pass

