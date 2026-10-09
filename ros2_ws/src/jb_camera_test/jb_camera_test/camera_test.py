#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JB 阿克曼底盘 · 摄像头测试节点
================================================

【这个节点干什么】
    把 RDK X3 上插的 USB 摄像头的画面，转成 ROS2 的 Image 消息发布出去，
    同时在终端打印实测的分辨率和帧率。

【为什么先做这个】
    ROS2 里所有视觉节点都是靠"话题(topic)"通信的。
    摄像头节点发，识别节点收。如果这个环节不通，后面 YOLO/OCR 全都无从谈起。

【数据流】
    摄像头(/dev/video8)
         ↓  OpenCV 读取
      BGR 的 numpy 数组
         ↓  cv_bridge 转换
      sensor_msgs/Image 消息
         ↓  发布
      话题 /camera/image_raw
         ↓
      （下游：识别 / OCR / 显示）

【跑法】
    ros2 run jb_camera_test camera_test --ros-args -p device:=/dev/video8

【查看画面】
    另开一个终端：
      ros2 run rqt_image_view rqt_image_view        # 图形界面
    或者用我们的订阅端（见 camera_sub.py）
"""

import rclpy                              # ROS2 的 Python 库（核心）
from rclpy.node import Node               # 所有节点的基类
from sensor_msgs.msg import Image         # ★ 相机图像的"标准信封"
from cv_bridge import CvBridge            # ★ OpenCV Mat <-> ROS Image 互转
import cv2                                # OpenCV：实际抓图靠它
import time                               # 计时（算帧率用）


class CameraTestNode(Node):
    """摄像头测试节点

    继承 Node 后，super().__init__() 会：
      ① 向 ROS2 注册这个节点（名字叫 'jb_camera_test'）
      ② 允许用 self.create_publisher / create_timer 等
    """

    def __init__(self):
        super().__init__('jb_camera_test')

        # ─────────────── 1. 声明参数 ───────────────
        # ROS2 的参数系统：可以在命令行覆盖，不用改代码
        #   -p device:=/dev/video8 -p width:=1280 -p height:=720
        self.declare_parameter('device', '/dev/video8')   # 摄像头设备节点
        self.declare_parameter('width',  1280)            # 请求的宽度
        self.declare_parameter('height', 720)             # 请求的高度
        self.declare_parameter('fps',    30)              # 请求的帧率
        self.declare_parameter('show_fps_every', 30)      # 每多少帧打印一次帧率

        self.device = self.get_parameter('device').value
        self.req_w  = self.get_parameter('width').value
        self.req_h  = self.get_parameter('height').value
        self.req_fps = self.get_parameter('fps').value
        self.report_every = self.get_parameter('show_fps_every').value

        # ─────────────── 2. 打开摄像头 ───────────────
        # VideoCapture 的参数：
        #   第一个：设备路径（也可以用数字 8 表示 /dev/video8）
        #   第二个：后端。V4L2 是 Linux 的标准摄像头接口
        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # 打不开就直接报错退出 —— 早失败比晚失败好
            self.get_logger().error(
                f'❌ 打不开摄像头 {self.device}\n'
                f'   排查：\n'
                f'     1) ls /dev/video*        看节点在不在\n'
                f'     2) ls -l {self.device}    看权限（要在 video 组）\n'
                f'     3) sudo fuser {self.device}  看有没有被别的程序占用'
            )
            raise RuntimeError(f'camera open failed: {self.device}')

        # 尝试设置 MJPG 格式
        #   ★ 为什么用 MJPG？
        #     USB 2.0 带宽只有 ~480Mbps。720p 的 YUYV(未压缩) 30fps 要 ~440Mbps，
        #     基本跑不动。MJPG 是压缩格式，同样画质只占 1/5 带宽。
        #     很多摄像头不显式设 MJPG 就自动降到 640x480。
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.req_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.req_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.req_fps)

        # ─────────────── 3. 读回"实际"参数 ───────────────
        # ★ 重要：摄像头不一定给你要的！
        #   你请求 1920x1080，它可能只给 1280x720。必须读回实际值。
        self.act_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.act_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.act_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        # fourcc 是个整数，转成 4 个字符的字符串
        self.fourcc_str = ''.join([chr((fourcc >> (8 * i)) & 0xFF) for i in range(4)])

        self.get_logger().info('=' * 58)
        self.get_logger().info(f'📷 摄像头已打开: {self.device}')
        self.get_logger().info(f'   请求: {self.req_w}x{self.req_h} @ {self.req_fps}fps')
        self.get_logger().info(f'   实际: {self.act_w}x{self.act_h} @ {self.act_fps:.0f}fps  格式={self.fourcc_str}')
        self.get_logger().info('=' * 58)
        if (self.act_w, self.act_h) != (self.req_w, self.req_h):
            self.get_logger().warn(
                f'⚠️ 分辨率没达到请求值！'
                f'请求 {self.req_w}x{self.req_h}，实得 {self.act_w}x{self.act_h}'
            )

        # ─────────────── 4. 创建发布者 ───────────────
        # create_publisher(消息类型, 话题名, 队列长度)
        #   消息类型：Image（标准图像格式）
        #   话题名  ：/camera/image_raw 是 ROS 社区约定俗成的名字
        #   队列长度：10 —— 发布太快时最多缓存 10 条，超了丢最旧的
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ─────────────── 5. 创建 cv_bridge ───────────────
        # 它负责 OpenCV 的 BGR 数组 <-> ROS 的 Image 消息
        self.bridge = CvBridge()

        # ─────────────── 6. 创建定时器（整个节点的"心跳"）───────────────
        # create_timer(周期秒, 回调函数)
        #   按 30fps 算，周期 = 1/30 ≈ 0.0333 秒
        #   ★ 用定时器而不是 while 循环，是因为 ROS2 需要定期"喘气"
        #     去处理订阅、参数更新等内部事务
        self.timer_period = 1.0 / max(1.0, self.act_fps if self.act_fps > 0 else 30.0)
        self.timer = self.create_timer(self.timer_period, self.on_timer)

        # ─────────────── 7. 帧率统计变量 ───────────────
        self.frame_count = 0        # 累计发了多少帧
        self.t_start = time.time()  # 上次统计的起点
        self.fail_count = 0         # 读帧失败次数

    # ================================================================
    #  定时器回调：每次触发就抓一帧、转换、发布
    # ================================================================
    def on_timer(self):
        # ── ① 抓一帧 ──
        # read() 返回 (成功?, 图像数组)
        # 图像数组的形状是 (高, 宽, 3)，通道顺序 BGR（不是 RGB！OpenCV 的传统）
        ok, frame = self.cap.read()
        if not ok:
            self.fail_count += 1
            # 偶尔失败正常（USB 丢包），连续失败才值得报警
            if self.fail_count % 30 == 1:
                self.get_logger().warn(f'⚠️ 读帧失败（累计 {self.fail_count} 次）')
            return

        # ── ② 转成 ROS 消息 ──
        # cv_bridge.cv2_to_imgmsg(图像, 编码)
        #   'bgr8' = 8 位每通道、BGR 顺序。要和 frame 的实际格式一致。
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 转换失败: {e}')
            return

        # ── ③ 填时间戳和坐标系 ──
        # header.stamp    ：这一帧的采集时刻。下游做时间同步全靠它
        # header.frame_id ：坐标系名。视觉里用来查"相机在车上的位姿"（TF 树）
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        # ── ④ 发布 ──
        self.pub.publish(msg)

        # ── ⑤ 统计帧率 ──
        self.frame_count += 1
        if self.frame_count % self.report_every == 0:
            dt = time.time() - self.t_start
            fps = self.report_every / dt if dt > 0 else 0.0
            self.get_logger().info(
                f'📊 已发 {self.frame_count} 帧 | 实测 {fps:.1f} fps | '
                f'分辨率 {frame.shape[1]}x{frame.shape[0]}'
            )
            self.t_start = time.time()

    # ================================================================
    #  节点销毁时释放摄像头（不然下次打不开）
    # ================================================================
    def destroy_node(self):
        self.get_logger().info('🔌 释放摄像头...')
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """程序入口

    rclpy.init()         : 初始化 ROS2
    spin(node)           : 让节点开始"转"—— 不断处理回调（定时器/订阅）
                           这行会一直阻塞，直到 Ctrl+C
    finally 里做清理     : 释放资源、关闭 ROS2
    """
    rclpy.init(args=args)
    node = None
    try:
        node = CameraTestNode()
        rclpy.spin(node)          # ★ 阻塞在这里，处理回调
    except KeyboardInterrupt:
        print('\n👋 收到 Ctrl+C，退出')
    except Exception as e:
        print(f'❌ 出错: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
