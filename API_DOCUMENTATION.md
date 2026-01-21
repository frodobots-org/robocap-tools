# API Documentation

Complete HTTP API documentation for Camera Calibration Database Server

## Server Information

- **Server Address**: `http://127.0.0.1:6001`
- **Content Type**: `application/json`

---

## 1. Intrinsic Calibration API

### 1.1 Submit Single Intrinsic Calibration Data

**Endpoint**: `POST /{device_id}/api/v1/intrinsic/{cam}`

**URL Parameters**:
- `device_id` (required): Device ID, string type
- `cam` (required): Camera position, valid values: `left`, `right`

**Headers**:
```
Content-Type: application/json
```

**Request Body** (JSON format):
```json
{
  "projection": [640.0, 640.0, 960.0, 540.0],
  "projection_range": [20.0, 20.0, 20.0, 20.0],
  "distortion": [0.007, -0.002, 0.02, 0.78],
  "distortion_range": [0.001, 0.001, 0.001, 0.001]
}
```

**Field Descriptions**:
- `projection` (required): Array containing 4 floats [fx, fy, cx, cy]
- `projection_range` (required): Array containing 4 floats representing the error range of projection
- `distortion` (required): Array containing 4 floats [k1, k2, p1, p2]
- `distortion_range` (required): Array containing 4 floats representing the error range of distortion

**Success Response** (HTTP 201):
```json
{
  "message": "Calibration data saved successfully",
  "device_id": "device001",
  "cam": "left",
  "is_passed": true,
  "id": 1
}
```

**Example** (using curl):
```bash
curl -X POST http://127.0.0.1:6001/device001/api/v1/intrinsic/left \
  -H "Content-Type: application/json" \
  -d '{
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  }'
```

### 1.2 Submit Eye Group Intrinsic Calibration Data (left-eye + right-eye)

**Endpoint**: `POST /{device_id}/api/v1/intrinsic/eye`

**URL Parameters**:
- `device_id` (required): Device ID, string type

**Headers**:
```
Content-Type: application/json
```

**Request Body** (JSON format):
```json
{
  "cam0": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  },
  "cam1": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  }
}
```

**Field Descriptions**:
- `cam0` (required): Intrinsic calibration data for left-eye camera
- `cam1` (required): Intrinsic calibration data for right-eye camera
- Each camera data includes: `projection`, `projection_range`, `distortion`, `distortion_range`

**Success Response** (HTTP 201):
```json
{
  "message": "Eye group calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "eye",
  "cams": ["left-eye", "right-eye"],
  "all_passed": true,
  "ids": [1, 2]
}
```

**Example** (using curl):
```bash
curl -X POST http://127.0.0.1:6001/device001/api/v1/intrinsic/eye \
  -H "Content-Type: application/json" \
  -d '{
    "cam0": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.007, -0.002, 0.02, 0.78],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "cam1": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.007, -0.002, 0.02, 0.78],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    }
  }'
```

### 1.3 Submit Front Group Intrinsic Calibration Data (left-front + right-front)

**Endpoint**: `POST /{device_id}/api/v1/intrinsic/front`

**URL Parameters**:
- `device_id` (required): Device ID, string type

**Headers**:
```
Content-Type: application/json
```

**Request Body** (JSON format):
```json
{
  "cam0": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  },
  "cam1": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  }
}
```

**Field Descriptions**:
- `cam0` (required): Intrinsic calibration data for left-front camera
- `cam1` (required): Intrinsic calibration data for right-front camera
- Each camera data includes: `projection`, `projection_range`, `distortion`, `distortion_range`

**Success Response** (HTTP 201):
```json
{
  "message": "Front group calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "front",
  "cams": ["left-front", "right-front"],
  "all_passed": true,
  "ids": [3, 4]
}
```

**Example** (using curl):
```bash
curl -X POST http://127.0.0.1:6001/device001/api/v1/intrinsic/front \
  -H "Content-Type: application/json" \
  -d '{
    "cam0": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.007, -0.002, 0.02, 0.78],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "cam1": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.007, -0.002, 0.02, 0.78],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    }
  }'
```

### 1.4 Get All Intrinsic Calibration Data

**Endpoint**: `GET /api/v1/intrinsic`

**Request**: No parameters

**Success Response** (HTTP 200):
```json
[
  {
    "id": 1,
    "device_id": "device001",
    "cam": "left-eye",
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.007, -0.002, 0.02, 0.78],
    "distortion_range": [0.001, 0.001, 0.001, 0.001],
    "is_passed": true,
    "created_at": "2024-01-01T12:00:00"
  }
]
```

---

## 2. Extrinsic Calibration API

### 2.1 Submit Extrinsic Calibration Data

**Endpoint**: `POST /{device_id}/api/v1/extrinsic/{cam_group}`

**URL Parameters**:
- `device_id` (required): Device ID, string type
- `cam_group` (required): Camera group, valid values: `left`, `right`, `eye`, `front`

**Headers**:
```
Content-Type: application/json
```

**Request Body** (JSON format):

#### 2.1.1 Single Camera Group (left/right)

```json
{
  "cam0_reprojection_error": 1.4900,
  "imu0_gyroscope_error_mean": 0.003,
  "imu0_gyroscope_error_median": 0.003,
  "imu0_accelerometer_error_mean": 0.003134,
  "imu0_accelerometer_error_median": 0.053134,
  "imu1_gyroscope_error_mean": 0.003,
  "imu1_gyroscope_error_median": 0.003,
  "imu1_accelerometer_error_mean": 0.003134,
  "imu1_accelerometer_error_median": 0.053134,
  "imu2_gyroscope_error_mean": 0.003,
  "imu2_gyroscope_error_median": 0.003,
  "imu2_accelerometer_error_mean": 0.003134,
  "imu2_accelerometer_error_median": 0.053134
}
```

#### 2.1.2 Dual Camera Group (eye/front)

```json
{
  "cam0_reprojection_error": 1.4900,
  "cam1_reprojection_error": 1.4900,
  "imu0_gyroscope_error_mean": 0.003,
  "imu0_gyroscope_error_median": 0.003,
  "imu0_accelerometer_error_mean": 0.003134,
  "imu0_accelerometer_error_median": 0.053134,
  "imu1_gyroscope_error_mean": 0.003,
  "imu1_gyroscope_error_median": 0.003,
  "imu1_accelerometer_error_mean": 0.003134,
  "imu1_accelerometer_error_median": 0.053134,
  "imu2_gyroscope_error_mean": 0.003,
  "imu2_gyroscope_error_median": 0.003,
  "imu2_accelerometer_error_mean": 0.003134,
  "imu2_accelerometer_error_median": 0.053134
}
```

**Field Descriptions**:
- `cam0_reprojection_error` (required): Float, reprojection error of camera 0 (px)
  - left/right: Corresponds to left or right camera
  - eye: Corresponds to left-eye camera
  - front: Corresponds to left-front camera
- `cam1_reprojection_error` (required for eye/front): Float, reprojection error of camera 1 (px)
  - eye: Corresponds to right-eye camera
  - front: Corresponds to right-front camera
- `imu0_gyroscope_error_mean` (required): Float, IMU0 gyroscope error mean (rad/s)
- `imu0_gyroscope_error_median` (required): Float, IMU0 gyroscope error median (rad/s)
- `imu0_accelerometer_error_mean` (required): Float, IMU0 accelerometer error mean (m/s²)
- `imu0_accelerometer_error_median` (required): Float, IMU0 accelerometer error median (m/s²)
- `imu1_*` (required): Same fields for IMU1
- `imu2_*` (required): Same fields for IMU2

**Success Response** (HTTP 201):
```json
{
  "message": "Extrinsic calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "left",
  "is_passed": true,
  "id": 1
}
```

**Error Response** (HTTP 400):
```json
{
  "error": "Missing required field: cam0_reprojection_error"
}
```

**Examples** (using curl):
```bash
# Single camera group (left)
curl -X POST http://127.0.0.1:6001/device001/api/v1/extrinsic/left \
  -H "Content-Type: application/json" \
  -d '{
    "cam0_reprojection_error": 1.4900,
    "imu0_gyroscope_error_mean": 0.003,
    "imu0_gyroscope_error_median": 0.003,
    "imu0_accelerometer_error_mean": 0.003134,
    "imu0_accelerometer_error_median": 0.053134,
    "imu1_gyroscope_error_mean": 0.003,
    "imu1_gyroscope_error_median": 0.003,
    "imu1_accelerometer_error_mean": 0.003134,
    "imu1_accelerometer_error_median": 0.053134,
    "imu2_gyroscope_error_mean": 0.003,
    "imu2_gyroscope_error_median": 0.003,
    "imu2_accelerometer_error_mean": 0.003134,
    "imu2_accelerometer_error_median": 0.053134
  }'

# Dual camera group (eye)
curl -X POST http://127.0.0.1:6001/device001/api/v1/extrinsic/eye \
  -H "Content-Type: application/json" \
  -d '{
    "cam0_reprojection_error": 1.4900,
    "cam1_reprojection_error": 1.4900,
    "imu0_gyroscope_error_mean": 0.003,
    "imu0_gyroscope_error_median": 0.003,
    "imu0_accelerometer_error_mean": 0.003134,
    "imu0_accelerometer_error_median": 0.053134,
    "imu1_gyroscope_error_mean": 0.003,
    "imu1_gyroscope_error_median": 0.003,
    "imu1_accelerometer_error_mean": 0.003134,
    "imu1_accelerometer_error_median": 0.053134,
    "imu2_gyroscope_error_mean": 0.003,
    "imu2_gyroscope_error_median": 0.003,
    "imu2_accelerometer_error_mean": 0.003134,
    "imu2_accelerometer_error_median": 0.053134
  }'
```

### 2.2 Get All Extrinsic Calibration Data

**Endpoint**: `GET /api/v1/extrinsic`

**Request**: No parameters

**Success Response** (HTTP 200):
```json
[
  {
    "id": 1,
    "device_id": "device001",
    "cam_group": "left",
    "cam0_reprojection_error": 1.49,
    "cam1_reprojection_error": null,
    "imu0": {
      "gyroscope_error_mean": 0.003,
      "gyroscope_error_median": 0.003,
      "accelerometer_error_mean": 0.003134,
      "accelerometer_error_median": 0.053134
    },
    "imu1": { ... },
    "imu2": { ... },
    "is_passed": true,
    "created_at": "2024-01-01T12:00:00"
  }
]
```

---

## 3. Device Management API

### 3.1 Query All Calibration Records by Device ID

**Endpoint**: `GET /api/v1/device/{device_id}`

**URL Parameters**:
- `device_id` (required): Device ID, string type

**Request**: No request body

**Success Response** (HTTP 200):
```json
{
  "device_id": "device001",
  "intrinsic": [
    {
      "id": 1,
      "device_id": "device001",
      "cam": "left-eye",
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.007, -0.002, 0.02, 0.78],
      "distortion_range": [0.001, 0.001, 0.001, 0.001],
      "is_passed": true,
      "created_at": "2024-01-01T12:00:00"
    }
  ],
  "extrinsic": [
    {
      "id": 1,
      "device_id": "device001",
      "cam_group": "left",
      "cam0_reprojection_error": 1.49,
      "cam1_reprojection_error": null,
      "imu0": { ... },
      "imu1": { ... },
      "imu2": { ... },
      "is_passed": true,
      "created_at": "2024-01-01T12:00:00"
    }
  ],
  "intrinsic_count": 6,
  "extrinsic_count": 4,
  "total_count": 10
}
```

**Example** (using curl):
```bash
curl -X GET http://127.0.0.1:6001/api/v1/device/device001
```

### 3.2 Delete All Calibration Records by Device ID

**Endpoint**: `DELETE /api/v1/device/{device_id}`

**URL Parameters**:
- `device_id` (required): Device ID, string type

**Request**: No request body

**Success Response** (HTTP 200):
```json
{
  "message": "Successfully deleted all calibration data for device_id: device001",
  "device_id": "device001",
  "deleted_intrinsic_count": 6,
  "deleted_extrinsic_count": 4,
  "deleted_total_count": 10
}
```

**Not Found Response** (HTTP 404):
```json
{
  "message": "No calibration data found for device_id: device001",
  "device_id": "device001",
  "deleted_count": 0
}
```

**Error Response** (HTTP 500):
```json
{
  "error": "Internal server error message"
}
```

**Example** (using curl):
```bash
curl -X DELETE http://127.0.0.1:6001/api/v1/device/device001
```

**Warning**: The delete operation will permanently delete all intrinsic and extrinsic calibration records for the device. Please use with caution!

---

## 4. Calibration Pass Standards

### 4.1 Intrinsic Calibration Standards

- **Projection**: 
  - fx: 640 ± 20
  - fy: 640 ± 20
  - cx: 960 ± 40
  - cy: 540 ± 40

- **Distortion**: 
  - k1: 0.04 ± 0.03
  - k2: 0.0 ± 0.06
  - p1: 0.0 ± 0.06
  - p2: 0.0 ± 0.04

### 4.2 Extrinsic Calibration Standards

- **Camera Reprojection Error**: < 1.5px

- **Gyroscope Error**:
  - Mean: 0.003 - 0.008 rad/s
  - Median: 0.002 - 0.006 rad/s

- **Accelerometer Error**:
  - Mean: 0.04 - 0.10 m/s²
  - Median: 0.03 - 0.08 m/s²

---

## 5. Error Codes

- **200 OK**: Request successful (GET requests)
- **201 Created**: Data saved successfully (POST requests)
- **400 Bad Request**: Request parameter error or missing required fields
- **404 Not Found**: Resource not found
- **500 Internal Server Error**: Internal server error

---

## 6. Web Interface

Visit `http://127.0.0.1:6001/` to view the web interface for all calibration results.

**Interface Features**:
- Display calibration results grouped by device ID
- Real-time display of intrinsic and extrinsic calibration data
- Auto-refresh functionality (optional)
- Color-coded pass/fail status indicators
- Search functionality by device ID
- Filter to show only failed records
- Delete device records functionality

---

## 7. Calibration Data Reporting Flow

### Intrinsic Calibration Reporting

The system supports three methods for reporting intrinsic calibration data:

1. **Single Camera**: Report one camera at a time (left or right)
   - Endpoint: `POST /{device_id}/api/v1/intrinsic/{cam}`
   - Where `cam` is either `left` or `right`

2. **Eye Group**: Report both left-eye and right-eye cameras together
   - Endpoint: `POST /{device_id}/api/v1/intrinsic/eye`
   - Request body contains `cam0` (left-eye) and `cam1` (right-eye) data

3. **Front Group**: Report both left-front and right-front cameras together
   - Endpoint: `POST /{device_id}/api/v1/intrinsic/front`
   - Request body contains `cam0` (left-front) and `cam1` (right-front) data

### Extrinsic Calibration Reporting

The system supports four camera groups for extrinsic calibration:

1. **Left Group**: `POST /{device_id}/api/v1/extrinsic/left`
   - Contains left camera reprojection error + 3 IMU calibrations

2. **Right Group**: `POST /{device_id}/api/v1/extrinsic/right`
   - Contains right camera reprojection error + 3 IMU calibrations

3. **Eye Group**: `POST /{device_id}/api/v1/extrinsic/eye`
   - Contains left-eye and right-eye camera reprojection errors + 3 IMU calibrations

4. **Front Group**: `POST /{device_id}/api/v1/extrinsic/front`
   - Contains left-front and right-front camera reprojection errors + 3 IMU calibrations

Each extrinsic calibration submission includes calibration data for 3 IMUs (IMU0, IMU1, IMU2), each with gyroscope and accelerometer error measurements (mean and median).

---

## 8. Response Validation

All calibration submissions are automatically validated against the calibration standards defined in section 4. The `is_passed` field in the response indicates whether the calibration data meets all requirements:

- `is_passed: true` - All calibration values are within the acceptable ranges
- `is_passed: false` - One or more calibration values are outside the acceptable ranges

---

## 9. Data Persistence

All calibration data is stored in a SQLite database (`calibration.db`). The database is automatically created on first run and includes:

- Intrinsic calibration records with timestamps
- Extrinsic calibration records with timestamps
- Automatic validation results (pass/fail status)

---

## 10. Best Practices

1. **Device ID**: Use consistent device ID format across all submissions
2. **Timing**: Submit intrinsic and extrinsic calibration data as soon as calibration is complete
3. **Error Handling**: Check the `is_passed` field in the response to verify calibration quality
4. **Querying**: Use `GET /api/v1/device/{device_id}` to retrieve all calibration data for a specific device
5. **Deletion**: Use `DELETE /api/v1/device/{device_id}` carefully, as deletion is permanent

---

## Support

For issues or questions, please check the web interface at `http://127.0.0.1:6001/` or review the server logs.
