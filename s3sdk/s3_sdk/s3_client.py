"""
Low-level S3 client wrapper using boto3.
"""

import boto3
from botocore.exceptions import ClientError, BotoCoreError
from typing import List, Optional, BinaryIO, Dict

from .config import S3Config
from .exceptions import S3ConfigError, S3OperationError


class S3Client:
    """
    Low-level S3 client wrapper.
    Handles direct interaction with boto3 S3 client.
    """

    def __init__(self, config: S3Config):
        """
        Initialize S3 client with configuration.

        Args:
            config: S3Config instance containing AWS credentials and settings

        Raises:
            S3ConfigError: If configuration is invalid
        """
        if not config.validate():
            raise S3ConfigError("Invalid S3 configuration. All fields are required.")

        self.config = config
        self._client = None
        self._resource = None

    @property
    def client(self):
        """Get or create boto3 S3 client."""
        if self._client is None:
            self._client = boto3.client(
                's3',
                aws_access_key_id=self.config.access_key,
                aws_secret_access_key=self.config.secret_key,
                region_name=self.config.region
            )
        return self._client

    @property
    def resource(self):
        """Get or create boto3 S3 resource."""
        if self._resource is None:
            self._resource = boto3.resource(
                's3',
                aws_access_key_id=self.config.access_key,
                aws_secret_access_key=self.config.secret_key,
                region_name=self.config.region
            )
        return self._resource

    def upload_fileobj(self, file_obj: BinaryIO, key: str, extra_args: Optional[Dict] = None) -> bool:
        """
        Upload a file-like object to S3.

        Args:
            file_obj: File-like object to upload
            key: S3 object key (path)
            extra_args: Additional arguments to pass to upload

        Returns:
            True if successful

        Raises:
            S3OperationError: If upload fails
        """
        try:
            self.client.upload_fileobj(
                file_obj,
                self.config.bucket_name,
                key,
                ExtraArgs=extra_args or {}
            )
            return True
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to upload file to S3: {str(e)}")

    def upload_file(self, local_path: str, key: str, extra_args: Optional[Dict] = None) -> bool:
        """
        Upload a local file to S3.

        Args:
            local_path: Local file path
            key: S3 object key (path)
            extra_args: Additional arguments to pass to upload

        Returns:
            True if successful

        Raises:
            S3OperationError: If upload fails
        """
        try:
            self.client.upload_file(
                local_path,
                self.config.bucket_name,
                key,
                ExtraArgs=extra_args or {}
            )
            return True
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to upload file to S3: {str(e)}")

    def download_file(self, key: str, local_path: str) -> bool:
        """
        Download a file from S3 to local path.

        Args:
            key: S3 object key (path)
            local_path: Local file path to save to

        Returns:
            True if successful

        Raises:
            S3OperationError: If download fails
            S3FileNotFoundError: If file doesn't exist
        """
        try:
            self.client.download_file(
                self.config.bucket_name,
                key,
                local_path
            )
            return True
        except ClientError as e:
            error_code = e.response.get('Error', {}).get('Code', '')
            if error_code == '404' or error_code == 'NoSuchKey':
                raise S3FileNotFoundError(f"File not found in S3: {key}")
            raise S3OperationError(f"Failed to download file from S3: {str(e)}")
        except BotoCoreError as e:
            raise S3OperationError(f"Failed to download file from S3: {str(e)}")

    def list_objects(self, prefix: str = '', delimiter: Optional[str] = None) -> List[Dict]:
        """
        List all objects in S3 bucket with given prefix.
        Handles pagination automatically to get all objects.

        Args:
            prefix: Object key prefix to filter by
            delimiter: Delimiter for grouping keys (default: None for recursive listing)

        Returns:
            List of object metadata dictionaries

        Raises:
            S3OperationError: If listing fails
        """
        try:
            all_objects = []
            continuation_token = None

            while True:
                params = {
                    'Bucket': self.config.bucket_name,
                    'Prefix': prefix
                }
                
                if delimiter is not None:
                    params['Delimiter'] = delimiter
                
                if continuation_token:
                    params['ContinuationToken'] = continuation_token

                response = self.client.list_objects_v2(**params)
                
                contents = response.get('Contents', [])
                all_objects.extend(contents)

                # Check if there are more objects to fetch
                if not response.get('IsTruncated', False):
                    break

                continuation_token = response.get('NextContinuationToken')

            return all_objects
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to list objects in S3: {str(e)}")

    def list_common_prefixes(self, prefix: str = '', delimiter: Optional[str] = '/') -> List[str]:
        """
        List common prefixes (folders) in S3 bucket.

        Args:
            prefix: Prefix to filter by
            delimiter: Delimiter for grouping keys (default: '/')

        Returns:
            List of common prefix strings

        Raises:
            S3OperationError: If listing fails
        """
        try:
            response = self.client.list_objects_v2(
                Bucket=self.config.bucket_name,
                Prefix=prefix,
                Delimiter=delimiter
            )
            return [prefix['Prefix'] for prefix in response.get('CommonPrefixes', [])]
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to list prefixes in S3: {str(e)}")

    def delete_object(self, key: str) -> bool:
        """
        Delete a single object from S3.

        Args:
            key: S3 object key (path)

        Returns:
            True if successful

        Raises:
            S3OperationError: If deletion fails
        """
        try:
            self.client.delete_object(
                Bucket=self.config.bucket_name,
                Key=key
            )
            return True
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to delete object from S3: {str(e)}")

    def delete_objects(self, keys: List[str]) -> bool:
        """
        Delete multiple objects from S3.

        Args:
            keys: List of S3 object keys (paths)

        Returns:
            True if successful

        Raises:
            S3OperationError: If deletion fails
        """
        if not keys:
            return True

        try:
            objects = [{'Key': key} for key in keys]
            self.client.delete_objects(
                Bucket=self.config.bucket_name,
                Delete={
                    'Objects': objects,
                    'Quiet': True
                }
            )
            return True
        except (ClientError, BotoCoreError) as e:
            raise S3OperationError(f"Failed to delete objects from S3: {str(e)}")

    def object_exists(self, key: str) -> bool:
        """
        Check if an object exists in S3.

        Args:
            key: S3 object key (path)

        Returns:
            True if object exists, False otherwise
        """
        try:
            self.client.head_object(
                Bucket=self.config.bucket_name,
                Key=key
            )
            return True
        except ClientError as e:
            if e.response['Error']['Code'] == '404':
                return False
            raise S3OperationError(f"Failed to check object existence: {str(e)}")

