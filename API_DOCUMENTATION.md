# POST API Documentation — Camera Calibration Server

This document describes **all POST-only APIs** for the camera calibration database server. Use it to implement a client or a compatible server. All request/response bodies are JSON; use `Content-Type: application/json`.

---

## 1. Overview

### 1.1 Base URL

```
http://127.0.0.1:6001
```

(Or `http://0.0.0.0:6001` when bound to all interfaces.)

### 1.2 General Conventions

- **Method**: `POST` for all endpoints below.
- **Content-Type**: `application/json` for request and response.
- **Success**: Typically HTTP `201` for create.
- **Errors**: `400` (validation/bad request), `404` (not used by POST endpoints below), `500` (server error).

### 1.3 Error Code Fields (`errcode` / `errmsg`)

- **`errcode`** (integer, optional in request): Application-level error code. `0` = calibration completed (success); non-zero = calibration failed or error.
- **`errmsg`** (string, optional in request): Human-readable error or failure reason.
- Responses echo these as **`error_code`** and **`error_message`** where applicable.

---

## 2. Calibration Standards (Validation Reference)

Implement validation using these thresholds. A calibration **passes** only if **all** applicable checks pass.

### 2.1 Intrinsic (Projection)

| Parameter | Center | Tolerance | Valid Range     |
|----------|--------|-----------|------------------|
| fx      | 640.0  | 20.0      | [620.0, 660.0]   |
| fy      | 640.0  | 20.0      | [620.0, 660.0]   |
| cx      | 960.0  | 40.0      | [920.0, 1000.0]  |
| cy      | 540.0  | 40.0      | [500.0, 580.0]   |

**Format**: `projection` = `[fx, fy, cx, cy]` (index 0–3).

### 2.2 Intrinsic (Distortion)

| Parameter | Center | Tolerance | Valid Range    |
|----------|--------|-----------|-----------------|
| k1      | 0.04   | 0.03      | [0.01, 0.07]    |
| k2      | 0.0    | 0.06      | [-0.06, 0.06]   |
| p1      | 0.0    | 0.06      | [-0.06, 0.06]   |
| p2      | 0.0    | 0.04      | [-0.04, 0.04]   |

**Format**: `distortion` = `[k1, k2, p1, p2]` (index 0–3).

### 2.3 Extrinsic

- **Reprojection error** (per camera, pixels): must be **< 1.5**.
- **Gyroscope error** (rad/s): **mean** and **median** both **< 0.015** for IMU0, IMU1, IMU2.
- **Accelerometer error** (m/s²): **mean** and **median** both **< 0.12** for IMU0, IMU1, IMU2.

---

## 3. POST APIs

### 3.1 Submit Single-Camera Intrinsic Calibration

Submit intrinsic calibration for **one** camera (left or right).

| Item | Value |
|------|--------|
| **Method** | `POST` |
| **URL** | `/{device_id}/api/v1/intrinsic/{cam}` |
| **Path parameters** | `device_id` (string), `cam` (string) |

#### Path Parameters

| Name        | Type   | Required | Description |
|-------------|--------|----------|-------------|
| `device_id` | string | Yes      | Device identifier. |
| `cam`       | string | Yes      | Camera position. Allowed: `left`, `right`. |

#### Request Body

```json
{
  "projection": [640.0, 640.0, 960.0, 540.0],
  "projection_range": [20.0, 20.0, 20.0, 20.0],
  "distortion": [0.04, 0.0, 0.0, 0.0],
  "distortion_range": [0.001, 0.001, 0.001, 0.001],
  "errcode": 0,
  "errmsg": null
}
```

| Field               | Type    | Required | Description |
|---------------------|---------|----------|-------------|
| `projection`        | [float] | If `errcode` = 0 | `[fx, fy, cx, cy]`. Length 4. |
| `projection_range`  | [float] | If `errcode` = 0 | Length 4. |
| `distortion`        | [float] | If `errcode` = 0 | `[k1, k2, p1, p2]`. Length 4. |
| `distortion_range`  | [float] | If `errcode` = 0 | Length 4. |
| `errcode`           | int     | No       | Default `0`. `0` = success; non-zero = calibration failed. |
| `errmsg`            | string  | No       | Error/failure message. |

**Validation rules (apply in order):**

1. Request body must be valid JSON. If missing or invalid → `400` with `{"error": "No JSON data provided"}`.
2. If `errcode` is omitted, treat as `0`.
3. **If `errcode` ≠ 0 (failure report):**
   - Do **not** require `projection`, `projection_range`, `distortion`, `distortion_range`.
   - If any of these are missing, use defaults: `[0.0, 0.0, 0.0, 0.0]` for each.
   - Persist with `is_passed` = `false`, and store `errcode` / `errmsg`.
4. **If `errcode` = 0:**
   - Require all four fields. If any missing → `400` with `{"error": "Missing required field: <field>"}`.
   - Each of `projection`, `projection_range`, `distortion`, `distortion_range` must have **exactly 4** elements. Otherwise → `400` with `{"error": "projection and projection_range must have 4 elements"}` or `"distortion and distortion_range must have 4 elements"`.
   - Run intrinsic validation (Section 2.1 and 2.2). Set `is_passed` = `true` only if all pass.

#### Success Response (HTTP 201)

```json
{
  "message": "Calibration data saved successfully",
  "device_id": "device001",
  "cam": "left",
  "is_passed": true,
  "error_code": 0,
  "error_message": null,
  "id": 1
}
```

- `error_code`: same as request `errcode` (or 0 if omitted).
- `error_message`: same as request `errmsg`.
- `is_passed`: result of validation (or `false` when `errcode` ≠ 0).
- `id`: database ID of the saved intrinsic record.

#### Error Responses

- **400** — Validation error, e.g. `{"error": "Missing required field: projection"}`.
- **500** — Server error, e.g. `{"error": "<message>", "error_code": -1}`.

#### Example: Success (errcode = 0)

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/intrinsic/left" \
  -H "Content-Type: application/json" \
  -d '{
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.04, 0.0, 0.0, 0.0],
    "distortion_range": [0.001, 0.001, 0.001, 0.001],
    "errcode": 0,
    "errmsg": null
  }'
```

#### Example: Calibration Failure (errcode ≠ 0, minimal body)

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/intrinsic/left" \
  -H "Content-Type: application/json" \
  -d '{"errcode": 1, "errmsg": "Calibration failed - insufficient image quality"}'
```

Response still `201`; `is_passed` = `false`, `error_code` = 1, `error_message` = `"Calibration failed - insufficient image quality"`.

---

### 3.2 Submit Eye-Group Intrinsic Calibration (Two Cameras)

Submit intrinsic calibration for **left-eye** and **right-eye** together. **`cam0`** = left-eye, **`cam1`** = right-eye.

| Item | Value |
|------|--------|
| **Method** | `POST` |
| **URL** | `/{device_id}/api/v1/intrinsic/eye` |
| **Path parameters** | `device_id` (string) |

#### Path Parameters

| Name        | Type   | Required | Description |
|-------------|--------|----------|-------------|
| `device_id` | string | Yes      | Device identifier. |

#### Request Body

```json
{
  "cam0": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.04, 0.0, 0.0, 0.0],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  },
  "cam1": {
    "projection": [640.0, 640.0, 960.0, 540.0],
    "projection_range": [20.0, 20.0, 20.0, 20.0],
    "distortion": [0.04, 0.0, 0.0, 0.0],
    "distortion_range": [0.001, 0.001, 0.001, 0.001]
  },
  "errcode": 0,
  "errmsg": null
}
```

**Top-level fields:**

| Field    | Type   | Required | Description |
|----------|--------|----------|-------------|
| `cam0`   | object | **Always** | Left-eye intrinsic. See per-cam schema below. |
| `cam1`   | object | **Always** | Right-eye intrinsic. See per-cam schema below. |
| `errcode`| int    | No       | Default `0`. Top-level only; **not** inside `cam0`/`cam1`. |
| `errmsg` | string | No       | Top-level only; **not** inside `cam0`/`cam1`. |

**Per-camera object (`cam0` / `cam1`):**

| Field               | Type    | Required | Description |
|---------------------|---------|----------|-------------|
| `projection`        | [float] | Yes      | `[fx, fy, cx, cy]`, length 4. |
| `projection_range`  | [float] | Yes      | Length 4. |
| `distortion`        | [float] | Yes      | `[k1, k2, p1, p2]`, length 4. |
| `distortion_range`  | [float] | Yes      | Length 4. |

**Important:** `cam0` and `cam1` are **always** required with full calibration data. There is no “failure-only” minimal body like single-camera intrinsic. `errcode` / `errmsg` exist **only at top level**.

**Validation rules:**

1. Request body must be valid JSON. Otherwise → `400` with `{"error": "No JSON data provided"}`.
2. Require `cam0` and `cam1`. If missing → `400` with `{"error": "Missing required field: cam0"}` or `cam1`.
3. For each of `cam0`, `cam1`: require `projection`, `projection_range`, `distortion`, `distortion_range`. If any missing → `400` with `{"error": "Missing required field: cam0.<field>"}` (or `cam1.<field>`).
4. For each camera: `projection` and `projection_range` length 4; `distortion` and `distortion_range` length 4. Otherwise → `400` with appropriate message.
5. Read `errcode` (default 0) and `errmsg` from **top level only**.
6. If `errcode` ≠ 0: set `is_passed` = `false` for both cameras; still use provided `cam0`/`cam1` data.
7. If `errcode` = 0: run intrinsic validation (Section 2.1, 2.2) per camera. Set each camera’s `is_passed` accordingly. `all_passed` = `true` only if **both** pass.

#### Success Response (HTTP 201)

```json
{
  "message": "Eye group calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "eye",
  "cams": ["left-eye", "right-eye"],
  "all_passed": true,
  "error_code": 0,
  "error_message": null,
  "ids": [1, 2]
}
```

- `all_passed`: `true` only if both cameras passed validation (and `errcode` = 0).
- `ids`: database IDs for the two saved intrinsic records (cam0, then cam1).

#### Error Responses

- **400** — Missing/invalid fields as above.
- **500** — `{"error": "<message>"}`.

#### Example

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/intrinsic/eye" \
  -H "Content-Type: application/json" \
  -d '{
    "cam0": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.04, 0.0, 0.0, 0.0],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "cam1": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.04, 0.0, 0.0, 0.0],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "errcode": 0,
    "errmsg": null
  }'
```

---

### 3.3 Submit Front-Group Intrinsic Calibration (Two Cameras)

Same as **Eye** (Section 3.2), but for **left-front** and **right-front**. **`cam0`** = left-front, **`cam1`** = right-front.

| Item | Value |
|------|--------|
| **Method** | `POST` |
| **URL** | `/{device_id}/api/v1/intrinsic/front` |
| **Path parameters** | `device_id` (string) |

Request body shape identical to Eye: top-level `cam0`, `cam1`, `errcode`, `errmsg`. Per-camera schema and validation rules are the same. **`errcode` and `errmsg` only at top level; not inside `cam0`/`cam1`.**

#### Success Response (HTTP 201)

```json
{
  "message": "Front group calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "front",
  "cams": ["left-front", "right-front"],
  "all_passed": true,
  "error_code": 0,
  "error_message": null,
  "ids": [3, 4]
}
```

#### Example

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/intrinsic/front" \
  -H "Content-Type: application/json" \
  -d '{
    "cam0": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.04, 0.0, 0.0, 0.0],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "cam1": {
      "projection": [640.0, 640.0, 960.0, 540.0],
      "projection_range": [20.0, 20.0, 20.0, 20.0],
      "distortion": [0.04, 0.0, 0.0, 0.0],
      "distortion_range": [0.001, 0.001, 0.001, 0.001]
    },
    "errcode": 0,
    "errmsg": null
  }'
```

---

### 3.4 Submit Extrinsic Calibration

Submit extrinsic calibration for a camera group: **left**, **right**, **eye**, or **front**.  
- **left** / **right**: single camera; only `cam0_reprojection_error` is used.  
- **eye** / **front**: two cameras; both `cam0_reprojection_error` and `cam1_reprojection_error` are used.

| Item | Value |
|------|--------|
| **Method** | `POST` |
| **URL** | `/{device_id}/api/v1/extrinsic/{cam_group}` |
| **Path parameters** | `device_id` (string), `cam_group` (string) |

#### Path Parameters

| Name         | Type   | Required | Description |
|--------------|--------|----------|-------------|
| `device_id`  | string | Yes      | Device identifier. |
| `cam_group`  | string | Yes      | One of: `left`, `right`, `eye`, `front`. |

#### Request Body (Single-Camera: left / right)

```json
{
  "cam0_reprojection_error": 1.2,
  "imu0_gyroscope_error_mean": 0.005,
  "imu0_gyroscope_error_median": 0.004,
  "imu0_accelerometer_error_mean": 0.06,
  "imu0_accelerometer_error_median": 0.05,
  "imu1_gyroscope_error_mean": 0.005,
  "imu1_gyroscope_error_median": 0.004,
  "imu1_accelerometer_error_mean": 0.06,
  "imu1_accelerometer_error_median": 0.05,
  "imu2_gyroscope_error_mean": 0.005,
  "imu2_gyroscope_error_median": 0.004,
  "imu2_accelerometer_error_mean": 0.06,
  "imu2_accelerometer_error_median": 0.05,
  "errcode": 0,
  "errmsg": null
}
```

#### Request Body (Dual-Camera: eye / front)

Same as above, plus:

```json
"cam1_reprojection_error": 1.3
```

**Field summary:**

| Field | Type  | Required (errcode=0) | Required (errcode≠0) | Description |
|-------|-------|----------------------|----------------------|-------------|
| `cam0_reprojection_error` | float | Yes | No (default 999.0) | Reprojection error, pixels. |
| `cam1_reprojection_error` | float | Yes for `eye`/`front` only | No (default 999.0 for eye/front) | Reprojection error, pixels. Not used for `left`/`right`. |
| `imu0_gyroscope_error_mean` | float | Yes | No (default 999.0) | rad/s. |
| `imu0_gyroscope_error_median` | float | Yes | No (default 999.0) | rad/s. |
| `imu0_accelerometer_error_mean` | float | Yes | No (default 999.0) | m/s². |
| `imu0_accelerometer_error_median` | float | Yes | No (default 999.0) | m/s². |
| `imu1_gyroscope_error_mean` | float | Yes | No (default 999.0) | rad/s. |
| `imu1_gyroscope_error_median` | float | Yes | No (default 999.0) | rad/s. |
| `imu1_accelerometer_error_mean` | float | Yes | No (default 999.0) | m/s². |
| `imu1_accelerometer_error_median` | float | Yes | No (default 999.0) | m/s². |
| `imu2_gyroscope_error_mean` | float | Yes | No (default 999.0) | rad/s. |
| `imu2_gyroscope_error_median` | float | Yes | No (default 999.0) | rad/s. |
| `imu2_accelerometer_error_mean` | float | Yes | No (default 999.0) | m/s². |
| `imu2_accelerometer_error_median` | float | Yes | No (default 999.0) | m/s². |
| `errcode` | int | No | No | Default 0. |
| `errmsg` | string | No | No | Error/failure message. |

**Validation rules:**

1. Request body must be valid JSON. Otherwise → `400` with `{"error": "No JSON data provided"}`.
2. `cam_group` must be one of `left`, `right`, `eye`, `front`. Otherwise → `400` with `{"error": "Invalid cam_group. Must be one of: ['left', 'right', 'eye', 'front']"}`.
3. **If `errcode` ≠ 0:**
   - All calibration fields optional. Use 999.0 for any missing numeric field.  
   - For `left`/`right`, `cam1_reprojection_error` is not used (can be absent).  
   - Set `is_passed` = `false`. Store `errcode` and `errmsg`.
4. **If `errcode` = 0:**
   - **left / right:** Require `cam0_reprojection_error` and all 12 IMU fields.  
     - If missing → `400` with `{"error": "Missing required field: <field>"}`.
   - **eye / front:** Require `cam0_reprojection_error`, `cam1_reprojection_error`, and all 12 IMU fields. Same missing-field error otherwise.
   - Run extrinsic validation (Section 2.3). Set `is_passed` = `true` only if all checks pass.

#### Success Response (HTTP 201)

```json
{
  "message": "Extrinsic calibration data saved successfully",
  "device_id": "device001",
  "cam_group": "left",
  "is_passed": true,
  "error_code": 0,
  "error_message": null,
  "id": 1
}
```

- `id`: database ID of the saved extrinsic record.

#### Error Responses

- **400** — Invalid `cam_group`, missing required field, or invalid JSON.
- **500** — `{"error": "<message>"}`.

#### Example: left (success)

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/extrinsic/left" \
  -H "Content-Type: application/json" \
  -d '{
    "cam0_reprojection_error": 1.2,
    "imu0_gyroscope_error_mean": 0.005,
    "imu0_gyroscope_error_median": 0.004,
    "imu0_accelerometer_error_mean": 0.06,
    "imu0_accelerometer_error_median": 0.05,
    "imu1_gyroscope_error_mean": 0.005,
    "imu1_gyroscope_error_median": 0.004,
    "imu1_accelerometer_error_mean": 0.06,
    "imu1_accelerometer_error_median": 0.05,
    "imu2_gyroscope_error_mean": 0.005,
    "imu2_gyroscope_error_median": 0.004,
    "imu2_accelerometer_error_mean": 0.06,
    "imu2_accelerometer_error_median": 0.05,
    "errcode": 0,
    "errmsg": null
  }'
```

#### Example: eye (success)

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/extrinsic/eye" \
  -H "Content-Type: application/json" \
  -d '{
    "cam0_reprojection_error": 1.2,
    "cam1_reprojection_error": 1.3,
    "imu0_gyroscope_error_mean": 0.005,
    "imu0_gyroscope_error_median": 0.004,
    "imu0_accelerometer_error_mean": 0.06,
    "imu0_accelerometer_error_median": 0.05,
    "imu1_gyroscope_error_mean": 0.005,
    "imu1_gyroscope_error_median": 0.004,
    "imu1_accelerometer_error_mean": 0.06,
    "imu1_accelerometer_error_median": 0.05,
    "imu2_gyroscope_error_mean": 0.005,
    "imu2_gyroscope_error_median": 0.004,
    "imu2_accelerometer_error_mean": 0.06,
    "imu2_accelerometer_error_median": 0.05,
    "errcode": 0,
    "errmsg": null
  }'
```

#### Example: Calibration Failure (errcode ≠ 0, minimal body)

```bash
curl -X POST "http://127.0.0.1:6001/device001/api/v1/extrinsic/left" \
  -H "Content-Type: application/json" \
  -d '{"errcode": 1, "errmsg": "Extrinsic calibration failed - IMU data collection error"}'
```

Response `201`; `is_passed` = `false`, `error_code` = 1, stored values use defaults (e.g. 999.0) for missing fields.

---

## 4. Summary of POST Endpoints

| Endpoint | Description |
|----------|-------------|
| `POST /{device_id}/api/v1/intrinsic/{cam}` | Single-camera intrinsic (`left` / `right`). |
| `POST /{device_id}/api/v1/intrinsic/eye` | Eye-group intrinsic (left-eye + right-eye). |
| `POST /{device_id}/api/v1/intrinsic/front` | Front-group intrinsic (left-front + right-front). |
| `POST /{device_id}/api/v1/extrinsic/{cam_group}` | Extrinsic (`left` / `right` / `eye` / `front`). |

All of the above use `Content-Type: application/json`. The calibration standards in Section 2 define how **pass** vs **fail** is determined for intrinsic and extrinsic data.
