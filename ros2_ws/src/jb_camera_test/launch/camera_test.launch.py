# -*- coding: utf-8 -*-
"""
Launch 文件 —— 一次启动发布端 + 订阅端
================================================

【launch 是什么】
    一个 Python 脚本，用来"一次启动多个节点 + 设置参数"。
    不然你要开好几个终端，每个手敲一遍 ros2 run，很烦。

【跑法】
    ros2 launch jb_camera_test camera_test.launch.py
    想改参数：
    ros2 launch jb_camera_test camera_test.launch.py device:=/dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 声明可覆盖的参数（命令行 device:=xxx 就能改）──
    device_arg = DeclareLaunchArgument(
        'device', default_value='/dev/video8',
        description='摄像头设备节点'
    )
    width_arg = DeclareLaunchArgument('width',  default_value='1280')
    height_arg = DeclareLaunchArgument('height', default_value='720')

    # ── 发布端 ──
    # ★ respawn=True：节点崩了自动重启。调试期很有用。
    pub = Node(
        package='jb_camera_test',
        executable='camera_test',
        name='jb_camera_test',
        output='screen',                    # 日志打到终端
        parameters=[{
            'device': LaunchConfiguration('device'),
            'width':  LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    # ── 订阅端 ──
    sub = Node(
        package='jb_camera_test',
        executable='camera_sub',
        name='jb_camera_sub',
        output='screen',
        parameters=[{
            'save_dir': '/home/sunrise/captures',
            'report_every': 30,
        }],
    )

    return LaunchDescription([device_arg, width_arg, height_arg, pub, sub])
