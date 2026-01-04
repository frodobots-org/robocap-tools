#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web application for downloading calibration results from S3
"""

import os
import sys
import zipfile
import tempfile
import shutil
import json
import threading
from pathlib import Path
from flask import Flask, render_template, request, jsonify, send_file, Response, stream_with_context
from werkzeug.utils import secure_filename

# Add s3sdk to path
project_root = Path(__file__).parent.parent
s3sdk_path = project_root / "s3sdk"
if str(s3sdk_path) not in sys.path:
    sys.path.insert(0, str(s3sdk_path))

try:
    from s3_sdk import S3SDK, S3Config
    from config_reader import load_s3_config
except ImportError as e:
    print(f"Error: Could not import S3 SDK: {e}")
    sys.exit(1)

app = Flask(__name__)

# Default S3 config path (can be overridden via environment variable)
DEFAULT_S3_CONFIG_PATH = project_root / "s3sdk" / "s3_config.json"
S3_CONFIG_PATH = os.environ.get('S3_CONFIG_PATH', str(DEFAULT_S3_CONFIG_PATH))

# Store download progress (device_id -> progress_info)
download_progress = {}
progress_lock = threading.Lock()


def get_s3_sdk():
    """Initialize and return S3 SDK instance"""
    try:
        config = load_s3_config(S3_CONFIG_PATH)
        return S3SDK(config)
    except Exception as e:
        raise Exception(f"Failed to load S3 configuration: {e}")


@app.route('/')
def index():
    """Serve the main HTML page"""
    return render_template('index.html')


def download_with_progress(device_id, s3_sdk, s3_results_path, temp_dir, local_results_dir):
    """Download folder with progress updates"""
    # List objects first to get total count
    objects = s3_sdk._client.list_objects(prefix=s3_results_path, delimiter=None)
    files = [obj for obj in objects if 'Key' in obj and not obj['Key'].endswith('/')]
    total_files = len(files)
    
    if total_files == 0:
        return 0, 0
    
    # Update progress
    with progress_lock:
        download_progress[device_id] = {
            'stage': 'downloading',
            'current': 0,
            'total': total_files,
            'message': f'Downloading {total_files} files from S3...'
        }
    
    # Download files one by one with progress updates
    folder_key = s3_results_path
    if not folder_key.endswith('/'):
        folder_key += '/'
    
    successful = 0
    for idx, obj in enumerate(files, 1):
        s3_key = obj['Key']
        
        if s3_key.endswith('/'):
            continue
        
        # Calculate relative path
        if s3_key.startswith(folder_key):
            relative_path = s3_key[len(folder_key):]
        else:
            relative_path = s3_key
        
        path_parts = relative_path.split('/')
        path_parts = [part for part in path_parts if part]
        local_file_path = os.path.join(local_results_dir, *path_parts)
        
        # Create subdirectories
        local_file_dir = os.path.dirname(local_file_path)
        if local_file_dir and not os.path.exists(local_file_dir):
            os.makedirs(local_file_dir, exist_ok=True)
        
        # Download file
        try:
            s3_sdk._client.download_file(s3_key, local_file_path)
            successful += 1
        except Exception as e:
            print(f"Failed to download {s3_key}: {e}")
        
        # Update progress
        with progress_lock:
            download_progress[device_id] = {
                'stage': 'downloading',
                'current': idx,
                'total': total_files,
                'message': f'Downloaded {idx}/{total_files} files...'
            }
    
    return successful, total_files


@app.route('/api/download', methods=['POST'])
def download_results():
    """Download calibration results directory from S3 and return as ZIP"""
    try:
        data = request.get_json()
        device_id = data.get('device_id', '').strip()
        
        if not device_id:
            return jsonify({'error': 'Device ID is required'}), 400
        
        # Construct S3 path for results directory
        s3_results_path = f"{device_id}/v1/results/"
        
        # Create temporary directory for download
        temp_dir = tempfile.mkdtemp(prefix='calib_results_')
        
        try:
            # Initialize S3 SDK
            s3_sdk = get_s3_sdk()
            
            # Initialize progress
            with progress_lock:
                download_progress[device_id] = {
                    'stage': 'initializing',
                    'current': 0,
                    'total': 0,
                    'message': 'Initializing download...'
                }
            
            # Download results folder from S3 with progress
            local_results_dir = os.path.join(temp_dir, 'results')
            successful, total = download_with_progress(device_id, s3_sdk, s3_results_path, temp_dir, local_results_dir)
            
            if total == 0:
                shutil.rmtree(temp_dir)
                with progress_lock:
                    download_progress.pop(device_id, None)
                return jsonify({
                    'error': f'No results found for device ID: {device_id}',
                    's3_path': s3_results_path
                }), 404
            
            # Update progress: creating ZIP
            with progress_lock:
                download_progress[device_id] = {
                    'stage': 'zipping',
                    'current': 0,
                    'total': 0,
                    'message': 'Creating ZIP file...'
                }
            
            # Create ZIP file
            zip_filename = f"{device_id}_results.zip"
            zip_path = os.path.join(temp_dir, zip_filename)
            
            # Count files for ZIP progress
            file_list = []
            for root, dirs, files in os.walk(local_results_dir):
                for file in files:
                    file_list.append((root, file))
            
            total_zip_files = len(file_list)
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for idx, (root, file) in enumerate(file_list, 1):
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, local_results_dir)
                    zipf.write(file_path, arcname)
                    
                    # Update progress every 10 files
                    if idx % 10 == 0 or idx == total_zip_files:
                        with progress_lock:
                            download_progress[device_id] = {
                                'stage': 'zipping',
                                'current': idx,
                                'total': total_zip_files,
                                'message': f'Adding {idx}/{total_zip_files} files to ZIP...'
                            }
            
            # Update progress: ready
            with progress_lock:
                download_progress[device_id] = {
                    'stage': 'ready',
                    'current': total_zip_files,
                    'total': total_zip_files,
                    'message': 'Ready for download!'
                }
            
            # Clean up temp directory after response is sent
            def cleanup():
                try:
                    if os.path.exists(temp_dir):
                        shutil.rmtree(temp_dir)
                    with progress_lock:
                        download_progress.pop(device_id, None)
                except Exception:
                    pass
            
            # Use stream_with_context to ensure cleanup happens
            @stream_with_context
            def generate():
                try:
                    with open(zip_path, 'rb') as f:
                        while True:
                            chunk = f.read(8192)  # 8KB chunks
                            if not chunk:
                                break
                            yield chunk
                finally:
                    cleanup()
            
            # Send ZIP file
            return Response(
                generate(),
                mimetype='application/zip',
                headers={
                    'Content-Disposition': f'attachment; filename="{zip_filename}"',
                    'Content-Length': str(os.path.getsize(zip_path))
                }
            )
            
        except Exception as e:
            # Clean up temp directory on error
            try:
                if os.path.exists(temp_dir):
                    shutil.rmtree(temp_dir)
                with progress_lock:
                    download_progress.pop(device_id, None)
            except Exception:
                pass
            return jsonify({'error': f'Download failed: {str(e)}'}), 500
            
    except Exception as e:
        return jsonify({'error': f'Server error: {str(e)}'}), 500


@app.route('/api/progress/<device_id>')
def get_progress(device_id):
    """Get download progress for a device (SSE endpoint)"""
    def generate():
        while True:
            with progress_lock:
                progress = download_progress.get(device_id, None)
            
            if progress:
                data = json.dumps(progress)
                yield f"data: {data}\n\n"
                
                # If ready or error, close connection
                if progress['stage'] in ['ready', 'error']:
                    break
            else:
                yield f"data: {json.dumps({'stage': 'waiting', 'message': 'Waiting...'})}\n\n"
            
            import time
            time.sleep(0.5)  # Update every 500ms
    
    return Response(stream_with_context(generate()), mimetype='text/event-stream')


@app.route('/api/check', methods=['POST'])
def check_device():
    """Check if device results exist in S3"""
    try:
        data = request.get_json()
        device_id = data.get('device_id', '').strip()
        
        if not device_id:
            return jsonify({'error': 'Device ID is required'}), 400
        
        try:
            s3_sdk = get_s3_sdk()
            s3_results_path = f"{device_id}/v1/results/"
            
            # List objects with the results prefix
            objects = s3_sdk._client.list_objects(prefix=s3_results_path, delimiter=None)
            
            # Filter out folder markers (keys ending with /)
            # Objects is a list of dicts with 'Key' and 'Size' fields
            files = [obj for obj in objects if 'Key' in obj and not obj['Key'].endswith('/')]
            
            if files:
                return jsonify({
                    'exists': True,
                    'file_count': len(files),
                    's3_path': s3_results_path
                })
            else:
                return jsonify({
                    'exists': False,
                    'file_count': 0,
                    's3_path': s3_results_path
                })
                
        except Exception as e:
            return jsonify({'error': f'Check failed: {str(e)}'}), 500
            
    except Exception as e:
        return jsonify({'error': f'Server error: {str(e)}'}), 500


if __name__ == '__main__':
    # Check if S3 config exists
    if not os.path.exists(S3_CONFIG_PATH):
        print(f"Error: S3 configuration file not found: {S3_CONFIG_PATH}")
        print(f"Please create the configuration file or set S3_CONFIG_PATH environment variable")
        sys.exit(1)
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5000, debug=True)

