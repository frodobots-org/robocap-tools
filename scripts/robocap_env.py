#!/usr/bin/env python3
"""
robocap_env.py
Environment configuration module for robocap tools.
Exports dataset and output directory paths.
"""

LAUNCH_FILES_ROOT_DIR = "/catkin_ws_imu/src/imu_utils/launch"

# Dataset directories
DATASET_ROOT_DIR = "/data"
DATASET_IMUS_INTRINSIC_DIR = f"{DATASET_ROOT_DIR}/imus_intrinsic"
DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/imus_cam_lr_front_extrinsic"
DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/imus_cam_lr_eye_extrinsic"
DATASET_IMUS_CAM_L_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/imus_cam_l_extrinsic"
DATASET_IMUS_CAM_R_EXTRINSIC_DIR = f"{DATASET_ROOT_DIR}/imus_cam_r_extrinsic"

# Output directories
OUTPUT_ROOT_DIR = "/tmp/output"
OUTPUT_IMUS_INTRINSIC_DIR = f"{OUTPUT_ROOT_DIR}/imus_intrinsic"
OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR = f"{OUTPUT_ROOT_DIR}/imus_cam_lr_front_extrinsic"
OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR = f"{OUTPUT_ROOT_DIR}/imus_cam_lr_eye_extrinsic"
OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR = f"{OUTPUT_ROOT_DIR}/imus_cam_l_extrinsic"
OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR = f"{OUTPUT_ROOT_DIR}/imus_cam_r_extrinsic"

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

LAUNCH_FILE_IMU_MID_0 = f"{LAUNCH_FILES_ROOT_DIR}/imu_mid_0.launch"
LAUNCH_FILE_IMU_RIGHT_1 = f"{LAUNCH_FILES_ROOT_DIR}/imu_right_1.launch"
LAUNCH_FILE_IMU_LEFT_2 = f"{LAUNCH_FILES_ROOT_DIR}/imu_left_2.launch" 

IMU_UTILS_SETUP_FILE = "/catkin_ws_imu/devel/setup.bash"
KALIBR_SETUP_FILE = "/catkin_ws/devel/setup.bash"
ROSCORE_SETUP_FILE = "/opt/ros/noetic/setup.bash"
TARGET_FILE = "/robocap-tools/config/april_6x6.yaml"

# IMU ROS topics
IMU0_ROSTOPIC = "/imu_mid_0"
IMU1_ROSTOPIC = "/imu_right_1"
IMU2_ROSTOPIC = "/imu_left_2"

def get_imu_topic_name(dev_number: int) -> str:
    """
    Return IMU topic name based on device number.
    
    Args:
        dev_number: IMU device number (0, 1, or 2)
        
    Returns:
        ROS topic name string
    """
    if dev_number == 0:
        return IMU0_ROSTOPIC
    elif dev_number == 1:
        return IMU1_ROSTOPIC
    elif dev_number == 2:
        return IMU2_ROSTOPIC
    else:
        return f"/imu_{dev_number}"

def get_imu_topic_from_name(imu_name: str) -> str:
    """
    Return IMU topic name based on IMU name (e.g., "imu0", "imu1", "imu2").
    
    Args:
        imu_name: IMU name string (e.g., "imu0", "imu1", "imu2")
        
    Returns:
        ROS topic name string
    """
    # Extract number from imu_name (e.g., "imu0" -> 0)
    import re
    match = re.search(r'imu(\d+)', imu_name.lower())
    if match:
        dev_number = int(match.group(1))
        return get_imu_topic_name(dev_number)
    # Fallback: try to use imu_name directly
    if not imu_name.startswith('/'):
        return f"/{imu_name}"
    return imu_name

def get_video_topic_name(dev_number: int, position: str) -> str:
    """
    Generate video topic name based on device number and position.
    
    Args:
        dev_number: Camera device number (0, 1, 2, etc.)
        position: Position string (e.g., "right_eye", "left_front", "right")
                  Hyphens will be replaced with underscores automatically
        
    Returns:
        ROS topic name string (e.g., "/cam0_right_eye/image_raw")
    """
    # Replace hyphens with underscores in position
    position_clean = position.replace('-', '_')
    return f"/cam{dev_number}_{position_clean}/image_raw"

def get_video_topic_name_from_file(video_file: str) -> str:
    """
    Generate video topic name from video filename.
    Supported format: video_dev{N}_session{S}_segment{G}_{position}.mp4
    
    Args:
        video_file: Video file path or filename
        
    Returns:
        ROS topic name string or None if parsing fails
    """
    import os
    import re
    
    basename = os.path.basename(video_file)
    # Extract dev number
    dev_match = re.search(r'video_dev(\d+)', basename)
    if not dev_match:
        return None
    
    dev_number = int(dev_match.group(1))
    
    # Remove extension
    name_without_ext = os.path.splitext(basename)[0]
    
    # Filename format: video_dev{N}_session{S}_segment{G}_{position}
    # We need to extract the part after the last underscore as position info
    parts = name_without_ext.split('_')
    
    # Find dev number position
    dev_index = -1
    for i, part in enumerate(parts):
        if part.startswith('dev') and part[3:].isdigit():
            dev_index = i
            break
    
    if dev_index == -1:
        return None
    
    # Position info should be the part after the last underscore
    # Example: video_dev0_session11_segment1_right-eye -> right-eye
    # Or: video_dev3_session11_segment1_right -> right
    if len(parts) > dev_index + 1:
        # Take the last part as position (skip session and segment parts)
        position_str = parts[-1]
    else:
        return None
    
    return get_video_topic_name(dev_number, position_str)

# Export all environment variables
__all__ = [
    # Dataset directories
    "DATASET_ROOT_DIR",
    "DATASET_IMUS_INTRINSIC_DIR",
    "DATASET_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_LR_EYE_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_L_EXTRINSIC_DIR",
    "DATASET_IMUS_CAM_R_EXTRINSIC_DIR",
    # Output directories
    "OUTPUT_ROOT_DIR",
    "OUTPUT_IMUS_INTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_LR_FRONT_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_LR_EYE_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_L_EXTRINSIC_DIR",
    "OUTPUT_IMUS_CAM_R_EXTRINSIC_DIR",
    # ROS bag files
    "ROSBAG_FILE_IMUS_INTRINSIC",
    "ROSBAG_FILE_CAM_LR_FRONT_INTRINSIC",
    "ROSBAG_FILE_CAM_LR_EYE_INTRINSIC",
    "ROSBAG_FILE_CAM_L_INTRINSIC",
    "ROSBAG_FILE_CAM_R_INTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_LR_FRONT_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_LR_EYE_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_L_EXTRINSIC",
    "ROSBAG_FILE_IMUS_CAM_R_EXTRINSIC",
    # Camera chain files
    "CAMCHAIN_FILE_LR_FRONT",
    "CAMCHAIN_FILE_LR_EYE",
    "CAMCHAIN_FILE_L",
    "CAMCHAIN_FILE_R",
    # YAML files
    "YAML_FILE_IMU_MID_0",
    "YAML_FILE_IMU_RIGHT_1",
    "YAML_FILE_IMU_LEFT_2", 
    # Launch files
    "LAUNCH_FILE_IMU_MID_0",
    "LAUNCH_FILE_IMU_RIGHT_1",
    "LAUNCH_FILE_IMU_LEFT_2",
    # Setup files
    "IMU_UTILS_SETUP_FILE",
    "KALIBR_SETUP_FILE",
    "ROSCORE_SETUP_FILE",
    "TARGET_FILE",
    # IMU topics
    "IMU0_ROSTOPIC",
    "IMU1_ROSTOPIC",
    "IMU2_ROSTOPIC",
    # IMU topic functions
    "get_imu_topic_name",
    "get_imu_topic_from_name",
    # Camera topic functions
    "get_video_topic_name",
    "get_video_topic_name_from_file",
]