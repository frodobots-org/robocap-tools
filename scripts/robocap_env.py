#!/usr/bin/env python3
import os
import re

LAUNCH_FILES_ROOT_DIR = "/catkin_ws_imu/src/imu_utils/launch"
DATASET_ROOT_DIR = "/data"
_DEFAULT_DEVICE_ID = "dc0c9f0cd3efa0c6"

ROBOCAP_DEVICE_ID = os.environ.get("ROBOCAP_DEVICE_ID", _DEFAULT_DEVICE_ID)

def _update_all_paths():
    global DATASET_IMUS_INTRINSIC_DIR
    global DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
    global DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
    global DATASET_IMUS_CAM_L_EXTRINSIC_DIR
    global DATASET_IMUS_CAM_R_EXTRINSIC_DIR
    global OUTPUT_IMUS_INTRINSIC_DIR
    global OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR
    global OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR
    global OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR
    global OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR
    global ROSBAG_FILE_IMUS_INTRINSIC
    global ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC
    global ROSBAG_FILE_CAM_LR_EYE_INTRINSIC
    global ROSBAG_FILE_CAM_L_INTRINSIC
    global ROSBAG_FILE_CAM_R_INTRINSIC
    global ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC
    global ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC
    global ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC
    global ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC
    global CAMCHAIN_FILE_LR_FRONT
    global CAMCHAIN_FILE_LR_EYE
    global CAMCHAIN_FILE_L
    global CAMCHAIN_FILE_R
    global YAML_FILE_IMU_MID_0
    global YAML_FILE_IMU_RIGHT_1
    global YAML_FILE_IMU_LEFT_2
    
    DATASET_IMUS_INTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data5"
    DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data6"
    DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data7"
    DATASET_IMUS_CAM_L_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data8"
    DATASET_IMUS_CAM_R_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/data9"
    
    OUTPUT_IMUS_INTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_intrinsic"
    OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_cam_lr_front_extrinsic"
    OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_cam_lr_eye_extrinsic"
    OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_cam_l_extrinsic"
    OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/{ROBOCAP_DEVICE_ID}/v1/results/imus_cam_r_extrinsic"
    
    ROSBAG_FILE_IMUS_INTRINSIC = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imus_intrinsic.bag"
    ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC = f"{OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR}/cam_lr_front_intrinsic.bag"
    ROSBAG_FILE_CAM_LR_EYE_INTRINSIC = f"{OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR}/cam_lr_eye_intrinsic.bag"
    ROSBAG_FILE_CAM_L_INTRINSIC = f"{OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR}/cam_l_intrinsic.bag"
    ROSBAG_FILE_CAM_R_INTRINSIC = f"{OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR}/cam_r_intrinsic.bag"
    ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC = f"{OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR}/imus_cam_lr_front_extrinsic.bag"
    ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC = f"{OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR}/imus_cam_lr_eye_extrinsic.bag"
    ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC = f"{OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR}/imus_cam_l_extrinsic.bag"
    ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC = f"{OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR}/imus_cam_r_extrinsic.bag"
    
    CAMCHAIN_FILE_LR_FRONT = f"{OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR}/cam_lr_front_intrinsic-camchain.yaml"
    CAMCHAIN_FILE_LR_EYE = f"{OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR}/cam_lr_eye_intrinsic-camchain.yaml"
    CAMCHAIN_FILE_L = f"{OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR}/cam_l_intrinsic-camchain.yaml"
    CAMCHAIN_FILE_R = f"{OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR}/cam_r_intrinsic-camchain.yaml"
    
    YAML_FILE_IMU_MID_0 = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imu_mid_0.yaml"
    YAML_FILE_IMU_RIGHT_1 = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imu_right_1.yaml"
    YAML_FILE_IMU_LEFT_2 = f"{OUTPUT_IMUS_INTRINSIC_DIR}/imu_left_2.yaml"

_update_all_paths()

LAUNCH_FILE_IMU_MID_0 = f"{LAUNCH_FILES_ROOT_DIR}/imu_mid_0.launch"
LAUNCH_FILE_IMU_RIGHT_1 = f"{LAUNCH_FILES_ROOT_DIR}/imu_right_1.launch"
LAUNCH_FILE_IMU_LEFT_2 = f"{LAUNCH_FILES_ROOT_DIR}/imu_left_2.launch"

IMU_UTILS_SETUP_FILE = "/catkin_ws_imu/devel/setup.bash"
KALIBR_SETUP_FILE = "/catkin_ws/devel/setup.bash"
ROSCORE_SETUP_FILE = "/opt/ros/noetic/setup.bash"
TARGET_FILE = "/robocap-tools/config/april_6x6.yaml"

IMU0_ROSTOPIC = "/imu_mid_0"
IMU1_ROSTOPIC = "/imu_right_1"
IMU2_ROSTOPIC = "/imu_left_2"

def get_imu_topic_name(dev_number: int) -> str:
    if dev_number == 0:
        return IMU0_ROSTOPIC
    elif dev_number == 1:
        return IMU1_ROSTOPIC
    elif dev_number == 2:
        return IMU2_ROSTOPIC
    else:
        return f"/imu_{dev_number}"

def get_imu_topic_from_name(imu_name: str) -> str:
    match = re.search(r'imu(\d+)', imu_name.lower())
    if match:
        return get_imu_topic_name(int(match.group(1)))
    if not imu_name.startswith('/'):
        return f"/{imu_name}"
    return imu_name

def get_video_topic_name(dev_number: int, position: str) -> str:
    return f"/cam{dev_number}_{position.replace('-', '_')}/image_raw"

def get_video_topic_name_from_file(video_file: str) -> str:
    basename = os.path.basename(video_file)
    name_without_ext = os.path.splitext(basename)[0]
    
    # Try new format first: left-front.mp4, right-front.mp4, left-eye.mp4, right-eye.mp4, left.mp4, right.mp4
    # These don't have dev number, use dev 0
    if not re.search(r'video_dev', basename, re.IGNORECASE):
        # New format: filename is the position
        return get_video_topic_name(0, name_without_ext)
    
    # Old format: video_dev{N}_session{S}_segment{G}_{position}
    dev_match = re.search(r'video_dev(\d+)', basename)
    if not dev_match:
        return None
    
    dev_number = int(dev_match.group(1))
    parts = name_without_ext.split('_')
    
    for i, part in enumerate(parts):
        if part.startswith('dev') and part[3:].isdigit():
            if len(parts) > i + 1:
                return get_video_topic_name(dev_number, parts[-1])
            break
    return None

def set_device_id(device_id: str) -> None:
    global ROBOCAP_DEVICE_ID
    ROBOCAP_DEVICE_ID = device_id
    _update_all_paths()

__all__ = [
    "DATASET_ROOT_DIR",
    "DATASET_IMUS_INTRINSIC_DIR",
    "DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_L_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_R_EXTRINSIC_DIR",
    "OUTPUT_IMUS_INTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR",
    "ROSBAG_FILE_IMUS_INTRINSIC",
    "ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC",
    "ROSBAG_FILE_CAM_LR_EYE_INTRINSIC",
    "ROSBAG_FILE_CAM_L_INTRINSIC",
    "ROSBAG_FILE_CAM_R_INTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC",
    "CAMCHAIN_FILE_LR_FRONT",
    "CAMCHAIN_FILE_LR_EYE",
    "CAMCHAIN_FILE_L",
    "CAMCHAIN_FILE_R",
    "YAML_FILE_IMU_MID_0",
    "YAML_FILE_IMU_RIGHT_1",
    "YAML_FILE_IMU_LEFT_2",
    "LAUNCH_FILE_IMU_MID_0",
    "LAUNCH_FILE_IMU_RIGHT_1",
    "LAUNCH_FILE_IMU_LEFT_2",
    "IMU_UTILS_SETUP_FILE",
    "KALIBR_SETUP_FILE",
    "ROSCORE_SETUP_FILE",
    "TARGET_FILE",
    "IMU0_ROSTOPIC",
    "IMU1_ROSTOPIC",
    "IMU2_ROSTOPIC",
    "get_imu_topic_name",
    "get_imu_topic_from_name",
    "get_video_topic_name",
    "get_video_topic_name_from_file",
    "set_device_id",
]