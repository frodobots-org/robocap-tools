#!/bin/bash

# Check number of arguments
if [ $# -ne 2 ]; then
    echo "Usage: $0 <top-directory> <camx (x=0-5)>"
    echo "Example: $0 dc0c9f0cd3efa0c6 cam0"
    exit 1
fi

# Assign arguments
TOP_DIR="$1"
CAM_PARAM="$2"

# Validate cam parameter format (must be cam0~cam5)
if ! [[ "$CAM_PARAM" =~ ^cam[0-5]$ ]]; then
    echo "Error: Second argument must be one of cam0/cam1/cam2/cam3/cam4/cam5"
    exit 1
fi

# Extract the number after "cam" (x in camx)
CAM_NUM="${CAM_PARAM:3}"


# Step 1: Find all *extrinsic directories (direct subdirs of top directory)
echo "Looking for extrinsic directories under ${TOP_DIR}..."
EXTRINSIC_DIRS=$(find "$TOP_DIR" -maxdepth 1 -type d -name "*extrinsic")

if [ -z "$EXTRINSIC_DIRS" ]; then
    echo "Error: No *extrinsic directories found under ${TOP_DIR}"
    exit 1
fi


# Step 2: Find all .mp4 files in extrinsic directories
echo "Looking for .mp4 files in extrinsic directories..."
MP4_FILES=$(find $EXTRINSIC_DIRS -type f -name "*.mp4")

if [ -z "$MP4_FILES" ]; then
    echo "No .mp4 files found in extrinsic directories"
    exit 1
fi


# Step 3: Filter MP4 files starting with "video_dev${CAM_NUM}_" (match camx to devx)
echo "Filtering MP4 files starting with video_dev${CAM_NUM}_..."
TARGET_MP4_FILES=$(echo "$MP4_FILES" | grep "video_dev${CAM_NUM}_")

if [ -z "$TARGET_MP4_FILES" ]; then
    echo "No MP4 files found starting with video_dev${CAM_NUM}_"
    exit 1
fi


# Step 4: For each MP4 file, find the IMU0 DB file in the same directory
echo "Looking for IMU0 DB files in the same directories as MP4 files..."
TARGET_DB_FILES=""

for mp4_file in $TARGET_MP4_FILES; do
    # Get the directory containing the MP4 file
    mp4_dir=$(dirname "$mp4_file")
    
    # Find IMU0 DB files (IMUWriter_dev0_*) in the same directory
    db_files=$(find "$mp4_dir" -maxdepth 1 -type f \( -name "IMUWriter_dev0_*.db" -o -name "IMUWriter_dev0_*.db3" -o -name "IMUWriter_dev0_*.sqlite" -o -name "IMUWriter_dev0_*.sqlite3" \))
    
    if [ -z "$db_files" ]; then
        echo "Warning: No IMU0 DB files found in the same directory as $mp4_file"
    else
        if [ -z "$TARGET_DB_FILES" ]; then
            TARGET_DB_FILES="$db_files"
        else
            TARGET_DB_FILES="$TARGET_DB_FILES"$'\n'"$db_files"
        fi
    fi
done

if [ -z "$TARGET_DB_FILES" ]; then
    echo "Error: No IMU0 DB files found in the same directories as MP4 files"
    exit 1
fi


# Output results
echo -e "\nTarget MP4 files found:"
echo "$TARGET_MP4_FILES"

echo -e "\nTarget DB files found:"
echo "$TARGET_DB_FILES"

CAM_EXTRINSIC_DATASET_DIR=/tmp/dataset/extrinsic/dataset-dir/$CAM_PARAM
rm -rf $CAM_EXTRINSIC_DATASET_DIR
mkdir -p $CAM_EXTRINSIC_DATASET_DIR

# Decode mp4 to PNG.
/robocap-tools/build/bin/MP4FrameExtractor -i $TARGET_MP4_FILES -o $CAM_EXTRINSIC_DATASET_DIR/$CAM_PARAM

# Create rosbag.
CAM_EXTRINSIC_ROSBAG_DIR=/tmp/dataset/extrinsic/rosbag/$CAM_PARAM
if [ ! -d $CAM_EXTRINSIC_ROSBAG_DIR ]; then
    mkdir -p $CAM_EXTRINSIC_ROSBAG_DIR
fi

/robocap-scripts/merge_imu_data.py -i $TARGET_DB_FILES -o $CAM_EXTRINSIC_DATASET_DIR/imu0.csv -r 500

cd /catkin_ws/ && source ./devel/setup.bash && rosrun kalibr kalibr_bagcreater --folder $CAM_EXTRINSIC_DATASET_DIR/. --output-bag $CAM_EXTRINSIC_ROSBAG_DIR/$CAM_PARAM-imu0.bag

IMU_YAML_FILE=/tmp/result/intrinsic/imu0/imu0_imu_param.yaml
CAMCHAIN_FILE=/tmp/dataset/intrinsic/rosbag/$CAM_PARAM/$CAM_PARAM-camchain.yaml

cd /catkin_ws/ && source ./devel/setup.bash && rosrun kalibr kalibr_calibrate_imu_camera --bag $CAM_EXTRINSIC_ROSBAG_DIR/$CAM_PARAM-imu0.bag --cam $CAMCHAIN_FILE --imu $IMU_YAML_FILE --target /robocap-tools/config/april_6x6.yaml
exit 0
