# Calibration Results Download Web Application

A simple web application for downloading calibration results from S3.

## Features

- Web-based interface for downloading calibration results
- Check if device results exist before downloading
- Automatic ZIP file creation and download
- S3 configuration loaded from server-side fixed location

## Installation

1. Install Python dependencies:

```bash
cd web_download
pip install -r requirements.txt
```

2. Ensure S3 SDK is available in the parent directory (s3sdk folder)

3. Configure S3 credentials:

Create or ensure the S3 configuration file exists at one of these locations:
- `../s3sdk/s3_config.json` (default)
- Or set `S3_CONFIG_PATH` environment variable to point to your config file

The S3 config file should have the following format:

```json
{
    "access_key": "YOUR_ACCESS_KEY",
    "secret_key": "YOUR_SECRET_KEY",
    "bucket_name": "YOUR_BUCKET_NAME",
    "region": "us-east-1"
}
```

## Usage

### Run the Application

```bash
python3 app.py
```

The application will start on `http://0.0.0.0:5000`

### Access the Web Interface

Open your web browser and navigate to:

```
http://localhost:5000
```

### Using the Interface

1. Enter the device ID in the input field
2. Click "Check" to verify if results exist for the device
3. Click "Download" to download the results as a ZIP file

## API Endpoints

### POST /api/check

Check if device results exist in S3.

**Request:**
```json
{
    "device_id": "12d4730bb6a4382c"
}
```

**Response:**
```json
{
    "exists": true,
    "file_count": 42,
    "s3_path": "12d4730bb6a4382c/v1/results/"
}
```

### POST /api/download

Download calibration results directory from S3 and return as ZIP file.

**Request:**
```json
{
    "device_id": "12d4730bb6a4382c"
}
```

**Response:**
- Success: ZIP file download (application/zip)
- Error: JSON error message

## Configuration

### S3 Config Location

The S3 configuration file is loaded from:

1. Environment variable `S3_CONFIG_PATH` (if set)
2. Default: `../s3sdk/s3_config.json` (relative to app.py)

You can override the config path by setting the environment variable:

```bash
export S3_CONFIG_PATH=/path/to/your/s3_config.json
python3 app.py
```

## S3 Path Structure

The application downloads from the following S3 path structure:

```
{device_id}/v1/results/
```

This matches the structure used by the calibration system when uploading results.

## Notes

- The application creates temporary files during download and cleans them up automatically
- Large result directories may take some time to download and package
- The ZIP file preserves the directory structure from S3

