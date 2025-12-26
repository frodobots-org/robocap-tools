"""
High-level S3 SDK for file and folder operations.
"""

import os
from pathlib import Path
from typing import List, Optional, Union, Dict
import mimetypes

from .config import S3Config
from .s3_client import S3Client
from .exceptions import S3FileNotFoundError, S3OperationError


class S3SDK:
    """
    High-level S3 SDK providing file and folder operations.
    """

    def __init__(self, config: S3Config):
        """
        Initialize S3 SDK with configuration.

        Args:
            config: S3Config instance containing AWS credentials and settings
        """
        self.config = config
        self._client = S3Client(config)

    def upload_file(
        self,
        local_path: str,
        s3_path: str,
        content_type: Optional[str] = None
    ) -> bool:
        """
        Upload a single file to S3.

        Args:
            local_path: Local file path to upload
            s3_path: Destination path in S3 (can include folder structure)
            content_type: MIME type of the file (auto-detected if not provided)

        Returns:
            True if successful

        Raises:
            S3OperationError: If upload fails
            FileNotFoundError: If local file doesn't exist

        Example:
            sdk.upload_file('local/file.txt', 'folder/file.txt')
        """
        if not os.path.exists(local_path):
            raise FileNotFoundError(f"Local file not found: {local_path}")

        if not os.path.isfile(local_path):
            raise ValueError(f"Path is not a file: {local_path}")

        # Normalize S3 path
        s3_key = self._normalize_key(s3_path)

        # Detect content type if not provided
        extra_args = {}
        if content_type:
            extra_args['ContentType'] = content_type
        else:
            detected_type, _ = mimetypes.guess_type(local_path)
            if detected_type:
                extra_args['ContentType'] = detected_type

        return self._client.upload_file(local_path, s3_key, extra_args=extra_args)

    def create_folder(self, folder_path: str) -> bool:
        """
        Create a folder (directory) in S3.

        Note: In S3, folders don't actually exist as objects.
        This method creates an empty object with a trailing slash to represent a folder.

        Args:
            folder_path: Folder path in S3 (e.g., 'myfolder/subfolder/')

        Returns:
            True if successful

        Raises:
            S3OperationError: If folder creation fails

        Example:
            sdk.create_folder('myfolder/subfolder/')
        """
        # Normalize folder path to ensure it ends with '/'
        folder_key = self._normalize_key(folder_path)
        if not folder_key.endswith('/'):
            folder_key += '/'

        # Create empty object to represent folder
        try:
            self._client.client.put_object(
                Bucket=self.config.bucket_name,
                Key=folder_key,
                Body=b''
            )
            return True
        except Exception as e:
            raise S3OperationError(f"Failed to create folder: {str(e)}")

    def delete_folder(self, folder_path: str, recursive: bool = True) -> bool:
        """
        Delete a folder and all its contents from S3.

        Args:
            folder_path: Folder path in S3 to delete
            recursive: If True, delete all contents recursively (default: True)

        Returns:
            True if successful

        Raises:
            S3OperationError: If folder deletion fails

        Example:
            sdk.delete_folder('myfolder/subfolder/')
        """
        # Normalize folder path
        folder_key = self._normalize_key(folder_path)
        if not folder_key.endswith('/'):
            folder_key += '/'

        # List all objects with this prefix
        try:
            objects = self._client.list_objects(prefix=folder_key)
            if not objects:
                # Folder doesn't exist or is empty
                return True

            # Extract keys
            keys_to_delete = [obj['Key'] for obj in objects]

            # Delete all objects
            if keys_to_delete:
                self._client.delete_objects(keys_to_delete)

            return True
        except Exception as e:
            raise S3OperationError(f"Failed to delete folder: {str(e)}")

    def upload_multiple_files(
        self,
        local_files: List[Union[str, tuple]],
        s3_folder: str
    ) -> List[tuple]:
        """
        Upload multiple files to a folder in S3.

        Args:
            local_files: List of file paths, or list of (local_path, s3_name) tuples
            s3_folder: Destination folder path in S3

        Returns:
            List of tuples (local_path, s3_path, success) for each file

        Raises:
            S3OperationError: If any upload fails

        Example:
            # Using simple file paths (filename preserved)
            sdk.upload_multiple_files(['file1.txt', 'file2.txt'], 'myfolder/')

            # Using tuples (custom S3 names)
            sdk.upload_multiple_files([
                ('local/file1.txt', 'custom1.txt'),
                ('local/file2.txt', 'custom2.txt')
            ], 'myfolder/')
        """
        # Normalize folder path
        folder_key = self._normalize_key(s3_folder)
        if not folder_key.endswith('/'):
            folder_key += '/'

        results = []

        for item in local_files:
            if isinstance(item, tuple):
                local_path, s3_name = item
            else:
                local_path = item
                s3_name = os.path.basename(local_path)

            # Construct full S3 path
            s3_path = folder_key + s3_name

            try:
                success = self.upload_file(local_path, s3_path)
                results.append((local_path, s3_path, success))
            except Exception as e:
                results.append((local_path, s3_path, False))
                # Continue with other files even if one fails
                continue

        return results

    def download_file(self, s3_path: str, local_path: str) -> bool:
        """
        Download a single file from S3.

        Args:
            s3_path: Source file path in S3
            local_path: Destination local file path

        Returns:
            True if successful

        Raises:
            S3OperationError: If download fails
            S3FileNotFoundError: If file doesn't exist in S3

        Example:
            sdk.download_file('folder/file.txt', 'local/file.txt')
        """
        # Normalize S3 path
        s3_key = self._normalize_key(s3_path)

        # Create local directory if it doesn't exist
        local_dir = os.path.dirname(local_path)
        if local_dir and not os.path.exists(local_dir):
            os.makedirs(local_dir, exist_ok=True)

        return self._client.download_file(s3_key, local_path)

    def download_folder(
        self,
        s3_folder: str,
        local_folder: str,
        recursive: bool = True
    ) -> List[tuple]:
        """
        Download a folder and all its contents from S3.
        
        Lists all objects with the folder prefix, then downloads each object
        individually, preserving the S3 path structure locally.

        Args:
            s3_folder: Source folder path in S3 (prefix)
            local_folder: Destination local folder path
            recursive: If True, download all contents recursively (default: True)

        Returns:
            List of tuples (s3_path, local_path, success) for each file downloaded

        Raises:
            S3OperationError: If download fails

        Example:
            sdk.download_folder('myfolder/', 'local/myfolder/')
        """
        # Normalize folder path (prefix)
        folder_key = self._normalize_key(s3_folder)
        if not folder_key.endswith('/'):
            folder_key += '/'

        # Create local folder if it doesn't exist
        if not os.path.exists(local_folder):
            os.makedirs(local_folder, exist_ok=True)

        # List all objects with this prefix (recursively, no delimiter)
        try:
            # Use delimiter=None to get all objects recursively
            delimiter = None if recursive else '/'
            objects = self._client.list_objects(prefix=folder_key, delimiter=delimiter)

            if not objects:
                return []

            results = []
            
            # Count total objects (excluding folder markers)
            total_objects = sum(1 for obj in objects if not obj['Key'].endswith('/'))
            current_index = 0

            for obj in objects:
                s3_key = obj['Key']

                # Skip folder markers (empty objects ending with '/')
                if s3_key.endswith('/'):
                    continue

                current_index += 1

                # Calculate relative path from the folder prefix
                # This preserves the full path structure
                if s3_key.startswith(folder_key):
                    relative_path = s3_key[len(folder_key):]
                else:
                    # Fallback: use the full key
                    relative_path = s3_key

                # Convert S3 path separators (/) to OS-specific separators
                # Split by '/' and join with os.sep to preserve directory structure
                path_parts = relative_path.split('/')
                path_parts = [part for part in path_parts if part]  # Remove empty parts
                
                # Construct local file path, preserving the S3 directory structure
                local_file_path = os.path.join(local_folder, *path_parts)

                # Create subdirectories if needed
                local_file_dir = os.path.dirname(local_file_path)
                if local_file_dir and not os.path.exists(local_file_dir):
                    os.makedirs(local_file_dir, exist_ok=True)

                # Download file with progress logging
                try:
                    print(f"    [{current_index}/{total_objects}] Downloading: {s3_key}")
                    success = self._client.download_file(s3_key, local_file_path)
                    results.append((s3_key, local_file_path, success))
                    if not success:
                        print(f"    [{current_index}/{total_objects}] Failed to download: {s3_key}")
                except Exception as e:
                    print(f"    [{current_index}/{total_objects}] Error downloading {s3_key}: {e}")
                    results.append((s3_key, local_file_path, False))
                    continue

            return results
        except Exception as e:
            raise S3OperationError(f"Failed to download folder: {str(e)}")

    def list_files(self, folder_path: str = '', recursive: bool = True) -> List[str]:
        """
        List all files in a folder.

        Args:
            folder_path: Folder path in S3 (empty string for root)
            recursive: If True, list files recursively (default: True)

        Returns:
            List of file paths in S3

        Example:
            sdk.list_files('myfolder/')
        """
        # Normalize folder path
        folder_key = self._normalize_key(folder_path)
        if folder_key and not folder_key.endswith('/'):
            folder_key += '/'

        delimiter = None if recursive else '/'
        objects = self._client.list_objects(prefix=folder_key, delimiter=delimiter)

        # Filter out folder markers (empty objects ending with '/')
        files = [obj['Key'] for obj in objects if not obj['Key'].endswith('/')]
        return files

    def list_folders(self, folder_path: str = '') -> List[str]:
        """
        List all folders (subdirectories) in a folder.

        Args:
            folder_path: Folder path in S3 (empty string for root)

        Returns:
            List of folder paths in S3

        Example:
            sdk.list_folders('myfolder/')
        """
        # Normalize folder path
        folder_key = self._normalize_key(folder_path)
        if folder_key and not folder_key.endswith('/'):
            folder_key += '/'

        folders = self._client.list_common_prefixes(prefix=folder_key)
        return folders

    def list_all_files(self, prefix: str = '', recursive: bool = True) -> List[str]:
        """
        List all files in the bucket, optionally filtered by prefix.

        Args:
            prefix: Optional prefix to filter files (empty string for all files)
            recursive: If True, list files recursively (default: True)

        Returns:
            List of all file paths in S3

        Example:
            sdk.list_all_files()  # List all files in bucket
            sdk.list_all_files('folder/')  # List all files with prefix 'folder/'
        """
        # Normalize prefix
        prefix_key = self._normalize_key(prefix) if prefix else ''

        delimiter = None if recursive else '/'
        objects = self._client.list_objects(prefix=prefix_key, delimiter=delimiter)

        # Filter out folder markers (empty objects ending with '/')
        files = [obj['Key'] for obj in objects if not obj['Key'].endswith('/')]
        return files

    def list_all_objects(
        self,
        prefix: str = '',
        recursive: bool = True,
        include_folders: bool = True
    ) -> List[Dict]:
        """
        List all objects (files and optionally folders) in the bucket.

        Args:
            prefix: Optional prefix to filter objects (empty string for all objects)
            recursive: If True, list objects recursively (default: True)
            include_folders: If True, include folder markers in results (default: True)

        Returns:
            List of object metadata dictionaries with keys like 'Key', 'Size', 'LastModified', etc.

        Example:
            sdk.list_all_objects()  # List all objects in bucket
            sdk.list_all_objects('folder/', include_folders=False)  # List only files
        """
        # Normalize prefix
        prefix_key = self._normalize_key(prefix) if prefix else ''

        delimiter = None if recursive else '/'
        objects = self._client.list_objects(prefix=prefix_key, delimiter=delimiter)

        # Filter out folder markers if not included
        if not include_folders:
            objects = [obj for obj in objects if not obj['Key'].endswith('/')]

        return objects

    def file_exists(self, s3_path: str) -> bool:
        """
        Check if a file exists in S3.

        Args:
            s3_path: File path in S3

        Returns:
            True if file exists, False otherwise

        Raises:
            S3OperationError: If check operation fails

        Example:
            if sdk.file_exists('folder/file.txt'):
                print('File exists')
        """
        s3_key = self._normalize_key(s3_path)
        return self._client.object_exists(s3_key)

    def folder_exists(self, folder_path: str) -> bool:
        """
        Check if a folder exists in S3.

        Note: In S3, folders don't actually exist as objects.
        A folder is considered to exist if:
        1. There's a folder marker object (key ending with '/'), OR
        2. There are any objects with the folder path as prefix

        Args:
            folder_path: Folder path in S3

        Returns:
            True if folder exists, False otherwise

        Raises:
            S3OperationError: If check operation fails

        Example:
            if sdk.folder_exists('myfolder/subfolder/'):
                print('Folder exists')
        """
        # Normalize folder path
        folder_key = self._normalize_key(folder_path)
        if not folder_key.endswith('/'):
            folder_key += '/'

        try:
            # Check if folder marker exists
            if self._client.object_exists(folder_key):
                return True

            # Check if any objects exist with this prefix
            objects = self._client.list_objects(prefix=folder_key, delimiter='/')
            if objects:
                return True

            return False
        except Exception as e:
            raise S3OperationError(f"Failed to check folder existence: {str(e)}")

    def _normalize_key(self, key: str) -> str:
        """
        Normalize S3 key by removing leading slashes and converting backslashes.

        Args:
            key: S3 key to normalize

        Returns:
            Normalized key
        """
        # Replace backslashes with forward slashes
        key = key.replace('\\', '/')
        # Remove leading slashes
        key = key.lstrip('/')
        return key

