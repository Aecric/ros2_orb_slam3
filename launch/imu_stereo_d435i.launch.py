"""Launch the ORB-SLAM3 IMU_STEREO node for an Intel RealSense D435i.

Defaults assume the realsense2_camera node was started with:
    enable_infra1:=true  enable_infra2:=true
    depth_module.infra_profile:=640x480x30
    depth_module.emitter_enabled:=0          # IR projector OFF for clean stereo
    enable_gyro:=true  enable_accel:=true  unite_imu_method:=2
    camera_name:=cam_head
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_SRC_ROOT = os.path.join(
    os.path.expanduser('~'), 'orb_slam3', 'src', 'ros2_orb_slam3'
)

DEFAULT_VOC_FILE = os.path.join(
    PACKAGE_SRC_ROOT, 'orb_slam3', 'Vocabulary', 'ORBvoc.txt.bin'
)
DEFAULT_SETTINGS_FILE = os.path.join(
    PACKAGE_SRC_ROOT, 'orb_slam3', 'config', 'Stereo-Inertial', 'RealSense_D435i.yaml'
)


def generate_launch_description():
    left_topic_arg = DeclareLaunchArgument(
        'left_topic',
        default_value='/camera/cam_head/infra1/image_rect_raw',
        description='Left IR (mono8) image topic.',
    )
    right_topic_arg = DeclareLaunchArgument(
        'right_topic',
        default_value='/camera/cam_head/infra2/image_rect_raw',
        description='Right IR (mono8) image topic.',
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/camera/cam_head/imu',
        description='Combined IMU topic produced by unite_imu_method:=2.',
    )
    voc_file_arg = DeclareLaunchArgument(
        'voc_file',
        default_value=DEFAULT_VOC_FILE,
    )
    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=DEFAULT_SETTINGS_FILE,
    )
    enable_pangolin_arg = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='true',
    )
    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='30',
    )

    imu_stereo_node = Node(
        package='ros2_orb_slam3',
        executable='imu_stereo_node_cpp',
        name='orb_slam3_imu_stereo',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voc_file_arg': LaunchConfiguration('voc_file'),
            'settings_file_path_arg': LaunchConfiguration('settings_file'),
            'left_image_topic': LaunchConfiguration('left_topic'),
            'right_image_topic': LaunchConfiguration('right_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'enable_pangolin': LaunchConfiguration('enable_pangolin'),
            'sync_queue_size': LaunchConfiguration('sync_queue_size'),
        }],
    )

    return LaunchDescription([
        left_topic_arg,
        right_topic_arg,
        imu_topic_arg,
        voc_file_arg,
        settings_file_arg,
        enable_pangolin_arg,
        sync_queue_size_arg,
        imu_stereo_node,
    ])
