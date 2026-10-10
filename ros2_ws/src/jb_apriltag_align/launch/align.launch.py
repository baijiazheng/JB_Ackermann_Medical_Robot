# -*- coding: utf-8 -*-
"""
Launch —— 最小视觉闭环：摄像头 + AprilTag 对准
================================================
    ros2 launch jb_apriltag_align align.launch.py
    ros2 launch jb_apriltag_align align.launch.py port:=/dev/ttyS0 tag_id:=0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port',        default_value='/dev/ttyS0'),
        DeclareLaunchArgument('tag_id',      default_value='0'),
        DeclareLaunchArgument('target_dist', default_value='0.5'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video8'),
    ]

    # ① 摄像头发布节点（复用 jb_camera_test）
    cam = Node(
        package='jb_camera_test', executable='camera_test',
        name='jb_camera_test', output='screen',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'width': 1280, 'height': 720,
        }],
        respawn=True, respawn_delay=2.0,
    )

    # ② 对准节点（核心闭环）
    align = Node(
        package='jb_apriltag_align', executable='align_node',
        name='jb_align_node', output='screen',
        parameters=[{
            'port':        LaunchConfiguration('port'),
            'tag_id':      LaunchConfiguration('tag_id'),
            'target_dist': LaunchConfiguration('target_dist'),
        }],
    )

    return LaunchDescription(args + [cam, align])
