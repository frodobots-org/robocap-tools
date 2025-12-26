# S3 SDK

A Python SDK for AWS S3 operations using boto3. Provides high-level APIs for file and folder operations.

## Features

- Upload single file
- Upload multiple files to a folder
- Create folders
- Delete folders (with all contents)
- Download single file
- Download folder (with all contents)
- List files and folders
- Configurable AWS credentials and settings

## Installation

### Prerequisites

- Python 3.7 or higher
- pip (Python package installer)

### Step 1: Clone or Download the Project

Navigate to the project directory:

```bash
cd robocap-s3
```

### Step 2: Create Virtual Environment (Recommended)

It's recommended to use a virtual environment to isolate project dependencies:

**On Windows:**
```bash
python -m venv venv
venv\Scripts\activate
```

**On Linux/Mac:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### Step 3: Install Dependencies

Install required packages:

```bash
pip install -r requirements.txt
```

Or install as a package (editable mode):

```bash
pip install -e .
```

This will install:
- boto3 (>=1.26.0)
- botocore (>=1.29.0)

## Running the Examples

### Basic Usage

Create a Python script and import the SDK:

```python
from s3_sdk import S3Config, S3SDK

config = S3Config(
    access_key='your-access-key',
    secret_key='your-secret-key',
    bucket_name='your-bucket-name',
    region='us-east-1'
)

sdk = S3SDK(config)
```

### Run Example Script

The project includes an example script (`example.py`) that demonstrates all features:

```bash
python example.py
```

Make sure to update the credentials in `example.py` before running:
- access_key: Your AWS Access Key ID
- secret_key: Your AWS Secret Access Key
- bucket_name: Your S3 bucket name
- region: AWS region (e.g., 'us-east-1')

## Quick Start

```python
from s3_sdk import S3Config, S3SDK

# Initialize configuration
config = S3Config(
    access_key='your-access-key',
    secret_key='your-secret-key',
    bucket_name='your-bucket-name',
    region='us-east-1'
)

# Create SDK instance
sdk = S3SDK(config)

# Upload a file
sdk.upload_file('local_file.txt', 'folder/file.txt')

# Create a folder
sdk.create_folder('myfolder/subfolder/')

# Upload multiple files
sdk.upload_multiple_files(['file1.txt', 'file2.txt'], 'myfolder/')

# Download a file
sdk.download_file('folder/file.txt', 'downloaded_file.txt')

# Download a folder
sdk.download_folder('myfolder/', 'local_downloads/')

    # Delete a folder
    sdk.delete_folder('myfolder/subfolder/')

    # List all files in bucket
    all_files = sdk.list_all_files()

    # List all objects with metadata
    all_objects = sdk.list_all_objects()

    # Check if file exists
    if sdk.file_exists('folder/file.txt'):
        print('File exists')

    # Check if folder exists
    if sdk.folder_exists('myfolder/subfolder/'):
        print('Folder exists')
```

## Architecture

### Project Structure

```
s3_sdk/
├── __init__.py          # Package initialization and exports
├── config.py            # S3Config class for credentials and settings
├── exceptions.py        # Custom exception classes
├── s3_client.py         # Low-level S3 client wrapper
└── s3_sdk.py           # High-level SDK API

setup.py                 # Package setup script
requirements.txt         # Python dependencies
example.py              # Usage examples
README.md               # Documentation
```

### Class Structure

#### S3Config (config.py)
Configuration class that stores:
- AWS Access Key ID
- AWS Secret Access Key
- S3 Bucket name
- AWS Region

#### S3Client (s3_client.py)
Low-level client wrapper that provides:
- Direct boto3 client/resource access
- Basic S3 operations (upload, download, list, delete)
- Error handling and exception conversion

#### S3SDK (s3_sdk.py)
High-level SDK that provides:
- File upload/download operations
- Folder create/delete operations
- Batch file operations
- Path normalization and helper methods

#### Exceptions (exceptions.py)
Custom exception hierarchy:
- S3SDKError: Base exception
- S3ConfigError: Configuration errors
- S3OperationError: Operation failures
- S3FileNotFoundError: File not found errors

## API Reference

### S3Config

```python
S3Config(access_key: str, secret_key: str, bucket_name: str, region: str = 'us-east-1')
```

### S3SDK

#### upload_file(local_path: str, s3_path: str, content_type: Optional[str] = None) -> bool
Upload a single file to S3.

#### create_folder(folder_path: str) -> bool
Create a folder in S3.

#### delete_folder(folder_path: str, recursive: bool = True) -> bool
Delete a folder and all its contents.

#### upload_multiple_files(local_files: List[Union[str, tuple]], s3_folder: str) -> List[tuple]
Upload multiple files to a folder. Returns list of (local_path, s3_path, success) tuples.

#### download_file(s3_path: str, local_path: str) -> bool
Download a single file from S3.

#### download_folder(s3_folder: str, local_folder: str, recursive: bool = True) -> List[tuple]
Download a folder and all its contents. Returns list of (s3_path, local_path, success) tuples.

#### list_files(folder_path: str = '', recursive: bool = True) -> List[str]
List all files in a folder.

#### list_folders(folder_path: str = '') -> List[str]
List all folders in a folder.

#### list_all_files(prefix: str = '', recursive: bool = True) -> List[str]
List all files in the bucket, optionally filtered by prefix.

#### list_all_objects(prefix: str = '', recursive: bool = True, include_folders: bool = True) -> List[Dict]
List all objects (files and optionally folders) in the bucket. Returns list of object metadata dictionaries.

#### file_exists(s3_path: str) -> bool
Check if a file exists in S3.

#### folder_exists(folder_path: str) -> bool
Check if a folder exists in S3.

## License

MIT

