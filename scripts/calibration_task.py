#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定任务定义模块
定义所有标定任务的类型和配置
"""

from enum import Enum
from dataclasses import dataclass
from typing import Optional, List


class CalibrationTaskType(Enum):
    """标定任务类型枚举"""
    IMU_INTRINSIC = "imu_intrinsic"  # data5: IMU内参标定（3个IMU）
    CAM_LR_FRONT_INTRINSIC = "cam_lr_front_intrinsic"  # data6: 左右前摄像头内参
    CAM_LR_FRONT_EXTRINSIC = "cam_lr_front_extrinsic"  # data6: 左右前摄像头外参
    CAM_LR_EYE_INTRINSIC = "cam_lr_eye_intrinsic"  # data7: 左右眼摄像头内参
    CAM_LR_EYE_EXTRINSIC = "cam_lr_eye_extrinsic"  # data7: 左右眼摄像头外参
    CAM_L_INTRINSIC = "cam_l_intrinsic"  # data8: 左摄像头内参
    CAM_L_EXTRINSIC = "cam_l_extrinsic"  # data8: 左摄像头外参
    CAM_R_INTRINSIC = "cam_r_intrinsic"  # data9: 右摄像头内参
    CAM_R_EXTRINSIC = "cam_r_extrinsic"  # data9: 右摄像头外参


@dataclass
class CalibrationTask:
    """标定任务定义"""
    task_type: CalibrationTaskType
    data_dir: str  # 数据目录（data5-data9）
    script_name: str  # 执行的脚本名称
    script_args: List[str]  # 脚本参数
    expected_output_files: List[str]  # 期望的输出文件列表
    s3_upload_path_template: str  # S3上传路径模板，使用{device_id}占位符
    
    def get_s3_path(self, device_id: str, filename: str = "{filename}") -> str:
        """获取S3上传路径"""
        return self.s3_upload_path_template.format(device_id=device_id, filename=filename)


# 定义所有标定任务配置
CALIBRATION_TASKS = {
    CalibrationTaskType.IMU_INTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.IMU_INTRINSIC,
        data_dir="data5",
        script_name="calib_imus_intrinsic.py",
        script_args=[],
        expected_output_files=[
            "imu_mid_0.yaml",
            "imu_right_1.yaml",
            "imu_left_2.yaml"
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_intrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_LR_FRONT_INTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_LR_FRONT_INTRINSIC,
        data_dir="data6",
        script_name="calib_cams_intrinsic.py",
        script_args=["front"],
        expected_output_files=[
            "cam_lr_front_intrinsic-camchain.yaml"
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_lr_front_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
        data_dir="data6",
        script_name="calib_cams_imu_extrinsic.py",
        script_args=["front"],
        expected_output_files=[
            "imus-cam_lr_front_extrinsic-camchain.yaml"  # 外参标定生成的camchain文件
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_lr_front_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_LR_EYE_INTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_LR_EYE_INTRINSIC,
        data_dir="data7",
        script_name="calib_cams_intrinsic.py",
        script_args=["eye"],
        expected_output_files=[
            "cam_lr_eye_intrinsic-camchain.yaml"
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_lr_eye_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_LR_EYE_EXTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
        data_dir="data7",
        script_name="calib_cams_imu_extrinsic.py",
        script_args=["eye"],
        expected_output_files=[
            "imus-cam_lr_eye_extrinsic-camchain.yaml"  # 外参标定生成的camchain文件
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_lr_eye_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_L_INTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_L_INTRINSIC,
        data_dir="data8",
        script_name="calib_cams_intrinsic.py",
        script_args=["left"],
        expected_output_files=[
            "cam_l_intrinsic-camchain.yaml"
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_l_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_L_EXTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_L_EXTRINSIC,
        data_dir="data8",
        script_name="calib_cams_imu_extrinsic.py",
        script_args=["left"],
        expected_output_files=[
            "imus-cam_l_extrinsic-camchain.yaml"  # 外参标定生成的camchain文件
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_l_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_R_INTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_R_INTRINSIC,
        data_dir="data9",
        script_name="calib_cams_intrinsic.py",
        script_args=["right"],
        expected_output_files=[
            "cam_r_intrinsic-camchain.yaml"
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_r_extrinsic/{filename}"
    ),
    CalibrationTaskType.CAM_R_EXTRINSIC: CalibrationTask(
        task_type=CalibrationTaskType.CAM_R_EXTRINSIC,
        data_dir="data9",
        script_name="calib_cams_imu_extrinsic.py",
        script_args=["right"],
        expected_output_files=[
            "imus-cam_r_extrinsic-camchain.yaml"  # 外参标定生成的camchain文件
        ],
        s3_upload_path_template="{device_id}/v1/results/imus_cam_r_extrinsic/{filename}"
    ),
}


def get_task_by_type(task_type: CalibrationTaskType) -> CalibrationTask:
    """根据任务类型获取任务配置"""
    return CALIBRATION_TASKS[task_type]


def get_all_task_types() -> List[CalibrationTaskType]:
    """获取所有任务类型列表（按执行顺序）"""
    return [
        CalibrationTaskType.IMU_INTRINSIC,
        CalibrationTaskType.CAM_LR_FRONT_INTRINSIC,
        CalibrationTaskType.CAM_LR_FRONT_EXTRINSIC,
        CalibrationTaskType.CAM_LR_EYE_INTRINSIC,
        CalibrationTaskType.CAM_LR_EYE_EXTRINSIC,
        CalibrationTaskType.CAM_L_INTRINSIC,
        CalibrationTaskType.CAM_L_EXTRINSIC,
        CalibrationTaskType.CAM_R_INTRINSIC,
        CalibrationTaskType.CAM_R_EXTRINSIC,
    ]

