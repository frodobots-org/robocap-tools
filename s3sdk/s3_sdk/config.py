"""
S3 Configuration class for storing AWS credentials and S3 settings.
"""


class S3Config:
    """
    Configuration class for S3 SDK.
    Stores AWS credentials (Access Key, Secret Key), bucket name, and region.
    """

    def __init__(self, access_key: str, secret_key: str, bucket_name: str, region: str = 'us-east-1'):
        """
        Initialize S3 configuration.

        Args:
            access_key: AWS Access Key ID
            secret_key: AWS Secret Access Key
            bucket_name: S3 bucket name
            region: AWS region (default: 'us-east-1')
        """
        self.access_key = access_key
        self.secret_key = secret_key
        self.bucket_name = bucket_name
        self.region = region

    def validate(self) -> bool:
        """
        Validate configuration parameters.

        Returns:
            True if all required fields are set, False otherwise
        """
        if not all([self.access_key, self.secret_key, self.bucket_name, self.region]):
            return False
        return True

    def __repr__(self) -> str:
        """String representation of configuration (hides sensitive data)."""
        return f"S3Config(bucket='{self.bucket_name}', region='{self.region}')"

