# robocap-tools for in/extrinsic calibration.

* download repo

```shell
git clone https://github.com/frodobots-org/robocap-tools.git
```

* initialize submodule

```shell
git submodule update --init --recursive
```

* build docker image

```shell
make
```

* download dataset(google drive)
  
  https://drive.google.com/drive/folders/1SEXwZtHckQblQ_WbQn9JkqGuP2ZRZf-I?dmr=1&ec=wgc-drive-globalnav-goto
  
* organize your dataset dir like this

```shell
➜  dc0c9f0cd3efa0c6 tree .
.
├── imus_cam_l_extrinsic
│   ├── IMUWriter_dev0_session8_segment1.db
│   ├── IMUWriter_dev1_session8_segment1.db
│   ├── IMUWriter_dev2_session8_segment1.db
│   └── video_dev4_session8_segment1_left.mp4
├── imus_cam_lr_eye_extrinsic
│   ├── IMUWriter_dev0_session7_segment1.db
│   ├── IMUWriter_dev1_session7_segment1.db
│   ├── IMUWriter_dev2_session7_segment1.db
│   ├── video_dev0_session7_segment1_right-eye.mp4
│   └── video_dev2_session7_segment1_left-eye.mp4
├── imus_cam_lr_front_extrinsic
│   ├── IMUWriter_dev0_session6_segment1.db
│   ├── IMUWriter_dev1_session6_segment1.db
│   ├── IMUWriter_dev2_session6_segment1.db
│   ├── video_dev1_session6_segment1_left-front.mp4
│   └── video_dev5_session6_segment1_right-front.mp4
├── imus_cam_r_extrinsic
│   ├── IMUWriter_dev0_session9_segment1.db
│   ├── IMUWriter_dev1_session9_segment1.db
│   ├── IMUWriter_dev2_session9_segment1.db
│   └── video_dev3_session9_segment1_right.mp4
└── imus_intrinsic
    ├── IMUWriter_dev0_session4_segment1.db
    ├── IMUWriter_dev0_session4_segment2.db
    ├── IMUWriter_dev0_session4_segment3.db
    ├── IMUWriter_dev0_session4_segment4.db
    ├── IMUWriter_dev0_session4_segment5.db
    ├── IMUWriter_dev0_session4_segment6.db
    ├── IMUWriter_dev0_session4_segment7.db
    ├── IMUWriter_dev0_session4_segment8.db
    ├── IMUWriter_dev1_session4_segment1.db
    ├── IMUWriter_dev1_session4_segment2.db
    ├── IMUWriter_dev1_session4_segment3.db
    ├── IMUWriter_dev1_session4_segment4.db
    ├── IMUWriter_dev1_session4_segment5.db
    ├── IMUWriter_dev1_session4_segment6.db
    ├── IMUWriter_dev1_session4_segment7.db
    ├── IMUWriter_dev1_session4_segment8.db
    ├── IMUWriter_dev2_session4_segment1.db
    ├── IMUWriter_dev2_session4_segment2.db
    ├── IMUWriter_dev2_session4_segment3.db
    ├── IMUWriter_dev2_session4_segment4.db
    ├── IMUWriter_dev2_session4_segment5.db
    ├── IMUWriter_dev2_session4_segment6.db
    ├── IMUWriter_dev2_session4_segment7.db
    └── IMUWriter_dev2_session4_segment8.db
```

* mount dataset and scripts while running the docker container
  
  unzip the dataset you downloaded.

```
docker run -it -v /home/dell/ethan/linux/robocap-tools/scripts:/robocap-scripts -v /home/dell/ethan/linux/dc0c9f0cd3efa0c6:/data robocap-calib bash
```

* run roscore program
  
```shell
source /opt/ros/noetic/setup.bash
/robocap-scripts/01_ros_env.sh
```

* run imus intrinsic calibration program
  
  >imu0(middle) + imu1(right) + imu2(left)

```shell
/robocap-scripts/calib_imus_intrinsic.py
```

* run front(eye/left/right) cams intrinsic calibration program
  
```shell
/robocap-scripts/calib_cams_intrinsic.py front
```

* run front(eye/left/right) cams extrinsic calibration program
  
```shell
/robocap-scripts/calib_cams_extrinsic.py front
```
