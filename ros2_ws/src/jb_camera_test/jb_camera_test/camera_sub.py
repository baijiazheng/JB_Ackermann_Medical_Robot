#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JB 阿克曼底盘 · 摄像头订阅端（验证 + 存图）
================================================

【这个节点干什么】
    订阅 /camera/image_raw，然后：
      ① 统计收到的帧率（验证发布端真的在发）
      ② 计算"端到端延迟"（当前时间 - 图像时间戳）★ 这个数很重要
      ③ 按需保存一帧到文件（给 OCR 分析用）

【为什么要算延迟】
    你的车要"看到屏幕 → 识别中文 → 转向"。
    如果图像延迟 200ms，车在 0.5m/s 下已经走了 10cm —— 会撞。
    所以延迟必须实测，不能想当然。

【跑法】
    ros2 run jb_camera_test camera_sub --ros-args -p save_dir:=/home/sunrise/captures
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time


class CameraSubNode(Node):

    def __init__(self):
        super().__init__('jb_camera_sub')

        # ── 参数 ──
        self.declare_parameter('save_dir', os.path.expanduser('~/captures'))
        self.declare_parameter('save_every_n', 0)   # 0 = 不自动存；>0 = 每 N 帧存一张
        self.declare_parameter('report_every', 30)  # 每多少帧打印一次统计

        self.save_dir = self.get_parameter('save_dir').value
        self.save_every_n = self.get_parameter('save_every_n').value
        self.report_every = self.get_parameter('report_every').value
        os.makedirs(self.save_dir, exist_ok=True)

        # ── 订阅者 ──
        # create_subscription(消息类型, 话题名, 回调函数, 队列长度)
        #   ★ 队列长度这里用小值(1)：视觉要的是"最新帧"，
        #     积压的旧帧没意义，反而增加延迟
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 1
        )

        self.bridge = CvBridge()
        self.count = 0
        self.t_start = time.time()
        self.lat_sum = 0.0       # 延迟累计（算平均）
        self.lat_max = 0.0       # 最大延迟

        self.get_logger().info('=' * 58)
        self.get_logger().info('👀 订阅 /camera/image_raw')
        self.get_logger().info(f'   存图目录: {self.save_dir}')
        if self.save_every_n > 0:
            self.get_logger().info(f'   每 {self.save_every_n} 帧自动存一张')
        self.get_logger().info('=' * 58)

    def on_image(self, msg):
        """每收到一帧就调用一次"""
        self.count += 1

        # ── ① 算延迟 ──
        # msg.header.stamp 是"采集时刻"（发布端打的）
        # 现在的时间 - 采集时刻 = 这一帧在路上花了多久
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age_ms = (self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000.0
            if 0 <= age_ms < 10000:          # 过滤明显异常的（比如时钟没同步）
                self.lat_sum += age_ms
                self.lat_max = max(self.lat_max, age_ms)
        except Exception:
            age_ms = -1

        # ── ② 定期打印统计 ──
        if self.count % self.report_every == 0:
            dt = time.time() - self.t_start
            fps = self.report_every / dt if dt > 0 else 0
            avg_lat = self.lat_sum / self.count if self.count else 0
            self.get_logger().info(
                f'📥 收到 {self.count} 帧 | {fps:.1f} fps | '
                f'延迟 当前{age_ms:.0f}ms 平均{avg_lat:.0f}ms 最大{self.lat_max:.0f}ms | '
                f'{msg.width}x{msg.height} {msg.encoding}'
            )
            self.t_start = time.time()

        # ── ③ 按需存图 ──
        if self.save_every_n > 0 and self.count % self.save_every_n == 0:
            self.save_one(msg)

    def save_one(self, msg):
        """把一帧存成 PNG（文件名带时间戳）"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            fn = os.path.join(self.save_dir,
                              time.strftime('cap_%Y%m%d_%H%M%S_') + f'{self.count:05d}.png')
            cv2.imwrite(fn, frame)
            self.get_logger().info(f'💾 已存 {fn}')
        except Exception as e:
            self.get_logger().error(f'存图失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraSubNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n👋 退出')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
