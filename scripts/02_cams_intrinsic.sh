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


# Step 1: Find all *static directories (direct subdirs of top directory)
echo "Looking for static directories under ${TOP_DIR}..."
STATIC_DIRS=$(find "$TOP_DIR" -maxdepth 1 -type d -name "*static")

if [ -z "$STATIC_DIRS" ]; then
    echo "Error: No *static directories found under ${TOP_DIR}"
    exit 1
fi


# Step 2: Find all .mp4 files in static directories (exclude .db files)
echo "Looking for .mp4 files in static directories..."
MP4_FILES=$(find $STATIC_DIRS -type f -name "*.mp4")

if [ -z "$MP4_FILES" ]; then
    echo "No .mp4 files found"
    exit 1
fi


# Step 3: Filter files starting with "video_dev${CAM_NUM}_" (match camx to devx)
echo "Filtering files starting with video_dev${CAM_NUM}_..."
TARGET_FILES=$(echo "$MP4_FILES" | grep "video_dev${CAM_NUM}_")

if [ -z "$TARGET_FILES" ]; then
    echo "No files found starting with video_dev${CAM_NUM}_"
    exit 1
fi

# Output results
echo -e "\nTarget files found:"
echo "$TARGET_FILES"

CAM_INTRINSIC_DATASET_DIR=/tmp/dataset/intrinsic/dataset-dir/$CAM_PARAM
rm -rf $CAM_INTRINSIC_DATASET_DIR
mkdir -p $CAM_INTRINSIC_DATASET_DIR

# Decode mp4 to PNG.
/robocap-tools/build/bin/MP4FrameExtractor -i $TARGET_FILES -o $CAM_INTRINSIC_DATASET_DIR/$CAM_PARAM -r 5

# Create rosbag.
CAM_INTRINSIC_ROSBAG_DIR=/tmp/dataset/intrinsic/rosbag/$CAM_PARAM
if [ ! -d $CAM_INTRINSIC_ROSBAG_DIR ]; then
    mkdir -p $CAM_INTRINSIC_ROSBAG_DIR
fi
cd /catkin_ws/ && source ./devel/setup.bash && rosrun kalibr kalibr_bagcreater --folder $CAM_INTRINSIC_DATASET_DIR/. --output-bag $CAM_INTRINSIC_ROSBAG_DIR/$CAM_PARAM.bag

# Run kalibr intrinsic calibration.
cd /catkin_ws/ && source ./devel/setup.bash && rosrun kalibr  kalibr_calibrate_cameras --bag $CAM_INTRINSIC_ROSBAG_DIR/$CAM_PARAM.bag --topics /$CAM_PARAM/image_raw --models pinhole-equi --target /robocap-tools/config/april_6x6.yaml  --no-outliers-removal
exit 0
