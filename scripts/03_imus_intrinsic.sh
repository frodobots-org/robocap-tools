#!/bin/bash

# Check number of arguments
if [ $# -ne 2 ]; then
    echo "Usage: $0 <top-directory> <imux (x=0-2)>"
    echo "Example: $0 dc0c9f0cd3efa0c6 imu0"
    exit 1
fi

# Assign arguments
TOP_DIR="$1"
IMU_PARAM="$2"

# Validate imu parameter format (must be imu0~imu2)
if ! [[ "$IMU_PARAM" =~ ^imu[0-2]$ ]]; then
    echo "Error: Second argument must be one of imu0/imu1/imu2"
    exit 1
fi

# Extract the number after "imu" (x in imux)
IMU_NUM="${IMU_PARAM:3}"


# Step 1: Find all *static directories (direct subdirs of top directory)
echo "Looking for static directories under ${TOP_DIR}..."
STATIC_DIRS=$(find "$TOP_DIR" -maxdepth 1 -type d -name "*static")

if [ -z "$STATIC_DIRS" ]; then
    echo "Error: No *static directories found under ${TOP_DIR}"
    exit 1
fi


# Step 2: Find imu-static subdirectories within static directories
echo "Looking for imu-static subdirectories in static directories..."
IMU_STATIC_DIRS=$(find $STATIC_DIRS -type d -name "imu-static")

if [ -z "$IMU_STATIC_DIRS" ]; then
    echo "Error: No imu-static subdirectories found in static directories"
    exit 1
fi


# Step 3: Find all .db files in imu-static directories
echo "Looking for .db files in imu-static directories..."
DB_FILES=$(find $IMU_STATIC_DIRS -type f \( -name "*.db" -o -name "*.db3" -o -name "*.sqlite" -o -name "*.sqlite3" \))

if [ -z "$DB_FILES" ]; then
    echo "No .db files found in imu-static directories"
    exit 1
fi


# Step 4: Filter files containing "IMUWriter_dev${IMU_NUM}_" (match imux to devx)
echo "Filtering files containing IMUWriter_dev${IMU_NUM}_..."
TARGET_FILES=$(echo "$DB_FILES" | grep "IMUWriter_dev${IMU_NUM}_")

if [ -z "$TARGET_FILES" ]; then
    echo "No files found containing IMUWriter_dev${IMU_NUM}_"
    exit 1
fi

# Output results
echo -e "\nTarget files found:"
echo "$TARGET_FILES"

IMU_INTRINSIC_DATASET_DIR=/tmp/dataset/intrinsic/dataset-dir/$IMU_PARAM
if [ ! -d $IMU_INTRINSIC_DATASET_DIR ]; then
    mkdir -p $IMU_INTRINSIC_DATASET_DIR
fi
/robocap-scripts/merge_multi_imu_data.py -i ${TOP_DIR}/imu-static -o $IMU_INTRINSIC_DATASET_DIR/$IMU_PARAM.csv -r 500 -d $IMU_NUM

# Create rosbag.
IMU_INTRINSIC_ROSBAG_DIR=/tmp/dataset/intrinsic/rosbag/$IMU_PARAM
if [ ! -d $IMU_INTRINSIC_ROSBAG_DIR ]; then
    mkdir -p $IMU_INTRINSIC_ROSBAG_DIR
fi
cd /catkin_ws/ && source ./devel/setup.bash && rosrun kalibr kalibr_bagcreater --folder $IMU_INTRINSIC_DATASET_DIR/. --output-bag $IMU_INTRINSIC_ROSBAG_DIR/$IMU_PARAM.bag

nohup rosbag play -r 20 $IMU_INTRINSIC_ROSBAG_DIR/$IMU_PARAM.bag  > /tmp/rosbag_launch.log 2>&1 &
cd /catkin_ws_imu && source ./devel/setup.bash && roslaunch imu_utils -t 10 imu$IMU_NUM.launch
/robocap-scripts/extract_imu_params.py -i /catkin_ws_imu/src/imu_utils/data -o /tmp/result/intrinsic/imu$IMU_NUM -d $IMU_NUM
exit 0
