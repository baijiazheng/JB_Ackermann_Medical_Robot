# -*- coding: utf-8 -*-
"""
Launch —— 一次启动「雷达驱动 + 测试节点 + 安全节点」
================================================

【跑法】
    ros2 launch jb_lidar_test lidar_test.launch.py
    # 换端口：
    ros2 launch jb_lidar_test lidar_test.launch.py port:=/dev/ttyUSB0

【启动顺序说明】
    雷达驱动要先起来，/scan 才有数据。
    测试节点和安全节点只订阅，所以可以同时起（收不到数据就等）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 可覆盖参数 ──
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='雷达串口（默认 /dev/ydlidar 需要有 udev 软链）'
    )
    baud_arg = DeclareLaunchArgument('baudrate', default_value='230400')

    # ── ① 雷达驱动（来自 ydlidar_ros2_driver 包）──
    # ★ 关键参数（T-mini Plus 专用）：
    #   lidar_type  = 1   → TYPE_TRIANGLE（三角测距，Tmini Pro/Plus 用这个）
    #   sample_rate = 4
    #   port        = 实际串口
    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[{
            'port':           LaunchConfiguration('port'),
            'baudrate':       LaunchConfiguration('baudrate'),
            'lidar_type':     1,        # TYPE_TRIANGLE
            'device_type':    0,        # TYPE_SERIAL
            'sample_rate':    4,
            'isSingleChannel': False,
            'intensity':      False,
            'frequency':      10.0,
            'frame_id':       'laser_frame',
            'angle_min':      -180.0,
            'angle_max':      180.0,
            'range_min':      0.02,
            'range_max':      12.0,
            'invalid_range_is_inf': False,
        }],
        respawn=True,
        respawn_delay=2.0,
    )

    # ── ② 测试节点（打印统计 + 预警）──
    tester = Node(
        package='jb_lidar_test',
        executable='lidar_test',
        name='jb_lidar_test',
        output='screen',
        parameters=[{'front_deg': 90.0, 'warn_dist': 1.0, 'report_every': 20}],
    )

    # ── ③ 安全节点（发布 CLEAR/SLOW/STOP）──
    safety = Node(
        package='jb_lidar_test',
        executable='lidar_safety',
        name='jb_lidar_safety',
        output='screen',
        parameters=[{'stop_dist': 0.35, 'warn_dist': 0.9}],
    )

    return LaunchDescription([port_arg, baud_arg, lidar, tester, safety])
