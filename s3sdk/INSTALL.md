# Installation and Setup Guide

## Environment Setup

### Step 1: Check Python Version

Make sure you have Python 3.7 or higher installed:

```bash
python --version
# or
python3 --version
```

### Step 2: Create Virtual Environment (Recommended)

Creating a virtual environment is recommended to avoid conflicts with other Python projects.

**Windows:**
```bash
python -m venv venv
venv\Scripts\activate
```

**Linux/Mac:**
```bash
python3 -m venv venv
source venv/bin/activate
```

After activation, you should see `(venv)` prefix in your terminal prompt.

### Step 3: Install Dependencies

Install all required packages:

```bash
pip install -r requirements.txt
```

This will install:
- boto3 (AWS SDK for Python)
- botocore (Core functionality for boto3)

### Step 4: Install SDK Package (Optional)

If you want to install the SDK as a package in editable mode:

```bash
pip install -e .
```

This allows you to import the SDK from any Python script in your system (when virtual environment is activated).

## Configuration

Before using the SDK, you need to configure your AWS credentials and S3 bucket information.

### Option 1: Direct Configuration in Code

```python
from s3_sdk import S3Config, S3SDK

config = S3Config(
    access_key='YOUR_ACCESS_KEY',
    secret_key='YOUR_SECRET_KEY',
    bucket_name='YOUR_BUCKET_NAME',
    region='us-east-1'
)

sdk = S3SDK(config)
```

### Option 2: Environment Variables (Not Currently Supported)

Currently, the SDK requires direct configuration. Environment variable support can be added if needed.

## Running Examples

### Run the Example Script

1. Edit `example.py` and update the configuration with your AWS credentials:

```python
config = S3Config(
    access_key='YOUR_ACCESS_KEY',
    secret_key='YOUR_SECRET_KEY',
    bucket_name='YOUR_BUCKET_NAME',
    region='us-east-1'
)
```

2. Run the example:

```bash
python example.py
```

### Create Your Own Script

Create a new Python file (e.g., `my_script.py`):

```python
from s3_sdk import S3Config, S3SDK

# Configure
config = S3Config(
    access_key='YOUR_ACCESS_KEY',
    secret_key='YOUR_SECRET_KEY',
    bucket_name='YOUR_BUCKET_NAME',
    region='us-east-1'
)

# Create SDK instance
sdk = S3SDK(config)

# Use SDK methods
files = sdk.list_all_files()
print(f"Found {len(files)} files in bucket")
```

Run your script:

```bash
python my_script.py
```

## Troubleshooting

### Import Error

If you get `ModuleNotFoundError: No module named 's3_sdk'`:

1. Make sure you're in the project directory
2. Make sure virtual environment is activated
3. Try installing in editable mode: `pip install -e .`

### AWS Credentials Error

If you get authentication errors:

1. Verify your AWS Access Key and Secret Key are correct
2. Make sure your AWS credentials have permissions to access the S3 bucket
3. Check that the bucket name and region are correct

### Connection Errors

If you get connection errors:

1. Check your internet connection
2. Verify the region is correct
3. Check AWS service status

## Uninstalling

To uninstall the package:

```bash
pip uninstall robocap-s3-sdk
```

To deactivate virtual environment:

```bash
deactivate
```

