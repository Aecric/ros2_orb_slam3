"""Launch the ORB-SLAM3 RGB-D node configured for an Intel RealSense D435 / D435i.

Defaults assume the realsense2_camera node was started with:
    camera_name:=cam_head  align_depth.enable:=true  rgb_camera.color_profile:=640x480x30

Override any of these from the command line, e.g.:
    ros2 launch ros2_orb_slam3 rgbd_d435i.launch.py \
        rgb_topic:=/my_cam/color/image_raw \
        depth_topic:=/my_cam/aligned_depth_to_color/image_raw \
        enable_pangolin:=false
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
    PACKAGE_SRC_ROOT, 'orb_slam3', 'config', 'RGB-D', 'RealSense_D435i.yaml'
)


def generate_launch_description():
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/cam_head/color/image_raw',
        description='Color image topic (encoded as rgb8/bgr8).',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/cam_head/aligned_depth_to_color/image_raw',
        description='Depth image topic. Must be aligned to the color frame (16UC1, mm).',
    )
    voc_file_arg = DeclareLaunchArgument(
        'voc_file',
        default_value=DEFAULT_VOC_FILE,
        description='Path to the ORB vocabulary binary.',
    )
    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value=DEFAULT_SETTINGS_FILE,
        description='Path to the ORB-SLAM3 RGB-D settings YAML.',
    )
    enable_pangolin_arg = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='true',
        description='Show the Pangolin viewer window.',
    )
    sync_queue_size_arg = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='30',
        description='message_filters ApproximateTime queue size for RGB+depth sync.',
    )

    rgbd_node = Node(
        package='ros2_orb_slam3',
        executable='rgbd_node_cpp',
        name='orb_slam3_rgbd',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'node_name_arg': 'rgbd_slam_cpp',
            'voc_file_arg': LaunchConfiguration('voc_file'),
            'settings_file_path_arg': LaunchConfiguration('settings_file'),
            'rgb_image_topic': LaunchConfiguration('rgb_topic'),
            'depth_image_topic': LaunchConfiguration('depth_topic'),
            'enable_pangolin': LaunchConfiguration('enable_pangolin'),
            'sync_queue_size': LaunchConfiguration('sync_queue_size'),
        }],
    )

    return LaunchDescription([
        rgb_topic_arg,
        depth_topic_arg,
        voc_file_arg,
        settings_file_arg,
        enable_pangolin_arg,
        sync_queue_size_arg,
        rgbd_node,
    ])
