#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JB 阿克曼 · 最小视觉闭环（AprilTag 对准）
================================================

【这个节点实现什么闭环】

    摄像头 ─→ AprilTag 检测 ─→ 算误差 ─→ P 控制 ─→ 串口 ─→ 下位机 ─→ 车动
       ↑                                                              │
       └──────────────────── 车动导致图像变化 ←──────────────────────┘

    这就是最基础的【视觉伺服】（visual servoing）闭环。

【为什么选 AprilTag 而不是 SLAM 做最小闭环】

    目标：把车开到标签正前方并停住（"精准进框"）。

    SLAM（gmapping）：
      输出【全局位姿】+ 地图 → 适合"从 A 开到 B"
      需要：里程计（odom）+ 雷达 + 建图时间
      ⚠️ 现在 AS5600 还没到，里程计做不了 → SLAM 暂时跑不起来

    AprilTag 视觉伺服：
      输出【相对目标的横向/纵向误差】→ 直接用于对准
      需要：只有摄像头（已有）
      ✅ 今天就能跑通

    ★ 结论：最小闭环 = AprilTag 视觉伺服。SLAM 是后续加分项。

【控制律（P 控制器）】

    图像/位姿误差 → 速度指令：

      横向误差 ex (m) :  车偏左/偏右多少
        → 角速度 ω = Kp_yaw_ · ex        （偏了就打方向）
        → ⚠️ 阿克曼不能原地转，所以 ω 只在前进时有意义

      纵向误差 ez (m) :  离标签还有多远
        → 线速度 v = Kp_dist · ez        （远了就快，近了就慢）
        → 限幅到 [v_min, v_max]

    为什么要限幅？
      · v 太小：电机死区，走不动 → 设 v_min
      · v 太大：冲过头，来不及停 → 设 v_max
      · 距离小于 stop_dist 时 v = 0（到位停车）

【为什么用 P 而不是 PID】

    视觉伺服的误差本身就在快速变化（车在动），
    积分项 I 容易累积导致过冲；微分项 D 对图像噪声敏感。
    P 控制够用，而且调参简单（只有 2 个参数）。
    ★ 先跑通 P，不够再加 D。

【串口协议】（与下位机 UartProtocol.cpp 对应）

    发送:  "V,<linearX>,<angularZ>\n"
      linearX  : m/s    前进速度（负=后退）
      angularZ : rad/s  角速度（正=左转）
    下位机会自动把 ω 反算成前轮转角：δ = atan(ω·L/v)

【跑法】
    ros2 run jb_apriltag_align align_node \
      --ros-args -p port:=/dev/ttyS0 -p tag_id:=0 -p target_dist:=0.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist          # ★ 标准速度消息
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import math
import time
import numpy as np

try:
    import serial                              # pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


class AlignNode(Node):

    def __init__(self):
        super().__init__('jb_align_node')

        # ══════════ 参数 ══════════
        # 摄像头
        self.declare_parameter('camera_topic', '/camera/image_raw')
        # AprilTag
        self.declare_parameter('tag_id', 0)          # 只跟这个 ID 的标签
        self.declare_parameter('tag_size', 0.05)     # 标签实际边长（米）
        # ★ 相机内参：RDK X3 + 200万摄像头的近似值
        #   ⚠️ 正式用必须自己标定（见 README）
        self.declare_parameter('fx', 700.0)
        self.declare_parameter('fy', 700.0)
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        # 目标位置
        self.declare_parameter('target_dist', 0.5)   # 想停在离标签多远（米）
        self.declare_parameter('stop_dist', 0.45)    # 小于这个距离就停
        # 控制增益
        self.declare_parameter('kp_dist', 1.2)       # 距离 → 线速度
        self.declare_parameter('kp_yaw', 2.0)        # 横向误差 → 角速度
        self.declare_parameter('v_min', 0.12)        # 最小线速度（克服死区）
        self.declare_parameter('v_max', 0.5)         # 最大线速度
        self.declare_parameter('w_max', 1.2)         # 最大角速度
        # 串口
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_hz', 20.0)      # 串口发送频率

        self.tag_id      = self.get_parameter('tag_id').value
        self.tag_size    = self.get_parameter('tag_size').value
        self.fx          = self.get_parameter('fx').value
        self.fy          = self.get_parameter('fy').value
        self.cx          = self.get_parameter('cx').value
        self.cy          = self.get_parameter('cy').value
        self.target_dist = self.get_parameter('target_dist').value
        self.stop_dist   = self.get_parameter('stop_dist').value
        self.kp_dist     = self.get_parameter('kp_dist').value
        self.kp_yaw      = self.get_parameter('kp_yaw').value
        self.v_min       = self.get_parameter('v_min').value
        self.v_max       = self.get_parameter('v_max').value
        self.w_max       = self.get_parameter('w_max').value

        # ══════════ AprilTag 检测器 ══════════
        # ★ 优先用 pupil_apriltags（更准），没有就退回 cv2.aruco
        self.detector = None
        self.detector_type = None
        try:
            from pupil_apriltags import Detector
            self.detector = Detector(
                families='tag36h11',      # 最常用的族
                nthreads=2,
                quad_decimate=2.0,        # 降采样加速（X3 算力有限）
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            self.detector_type = 'pupil'
            self.get_logger().info('✅ 用 pupil_apriltags 检测')
        except ImportError:
            # ── 退回 OpenCV 自带（不需要额外装包）──
            # ⚠️ OpenCV 4.7 起 aruco API 大改：
            #    旧 (<=4.6): Dictionary_get() + DetectorParameters_create() + detectMarkers()
            #    新 (>=4.7): getPredefinedDictionary() + DetectorParameters() + ArucoDetector
            #    两套 API 不兼容，必须按版本分支！
            if not hasattr(cv2, 'aruco'):
                self.get_logger().error('❌ 没有 AprilTag 检测器！装 pupil-apriltags 或 opencv-contrib')
                raise RuntimeError('no apriltag detector')

            cv_ver = tuple(int(x) for x in cv2.__version__.split('.')[:2])
            if cv_ver >= (4, 7):
                # ── 新 API ──
                d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
                p = cv2.aruco.DetectorParameters()
                det = cv2.aruco.ArucoDetector(d, p)
                self.detector = ('new', det)
                self.get_logger().info(f'✅ 用 cv2.aruco 新 API 检测 (OpenCV {cv2.__version__})')
            else:
                # ── 旧 API ──
                d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
                p = cv2.aruco.DetectorParameters_create()
                self.detector = ('old', (d, p))
                self.get_logger().info(f'✅ 用 cv2.aruco 旧 API 检测 (OpenCV {cv2.__version__})')
            self.detector_type = 'aruco'

        # ══════════ 订阅摄像头 ══════════
        # ★ 传感器话题用 BEST_EFFORT（和雷达一样的坑）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cam_topic = self.get_parameter('camera_topic').value
        self.sub = self.create_subscription(Image, cam_topic, self.on_image, qos)
        self.bridge = CvBridge()

        # ══════════ 发布（调试用）══════════
        # 把算出来的速度也发成 Twist，方便用 rqt 看 / 录 bag
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/align_state', 10)

        # ══════════ 串口 ══════════
        self.ser = None
        self.serial_ok = False
        if HAS_SERIAL:
            port = self.get_parameter('port').value
            baud = self.get_parameter('baudrate').value
            try:
                self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
                self.serial_ok = True
                self.get_logger().info(f'✅ 串口已打开 {port} @ {baud}')
            except Exception as e:
                self.get_logger().error(f'❌ 串口打开失败 {port}: {e}')
                self.get_logger().error('   -> 只发布 /cmd_vel，不发串口')
        else:
            self.get_logger().warn('⚠️ 没装 pyserial，只发布 /cmd_vel')

        # ══════════ 状态 ══════════
        self.last_err = None
        self.last_seen = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.lost_count = 0

        # 定时器：独立于相机帧率，稳定发串口
        send_hz = self.get_parameter('send_hz').value
        self.timer = self.create_timer(1.0 / max(1.0, send_hz), self.on_timer)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 最小视觉闭环 · AprilTag 对准')
        self.get_logger().info(f'   标签 ID={self.tag_id}  边长={self.tag_size}m')
        self.get_logger().info(f'   目标距离={self.target_dist}m  停止距离={self.stop_dist}m')
        self.get_logger().info(f'   增益 Kp_dist={self.kp_dist}  Kp_yaw={self.kp_yaw}')
        self.get_logger().info(f'   速度限幅 v∈[{self.v_min},{self.v_max}]  ω≤{self.w_max}')
        self.get_logger().info('=' * 60)

    # ================================================================
    #  图像回调：检测标签 → 算误差 → P 控制
    # ================================================================
    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = self.detect(gray)

        if result is None:
            # ── 没看到标签 ──
            self.lost_count += 1
            # 连续丢失超过 10 帧 → 停车（安全）
            if self.lost_count > 10:
                self.cmd_v = 0.0
                self.cmd_w = 0.0
                if self.lost_count % 50 == 1:
                    self.get_logger().warn('⚠️ 看不到标签，停车等待')
            return

        self.lost_count = 0
        self.last_seen = time.time()

        # ── 算误差 ──
        cx_img, cy_img, tag_px = result      # 标签中心像素坐标 + 像素边长

        # ① 横向误差：标签中心离图像中心多远（归一化到 [-1,1]）
        #    正 = 标签在右边 → 车要右转 → ω 为负（ROS 约定：正 ω = 左转）
        ex_norm = (cx_img - self.cx) / self.cx

        # ② 距离估算：用"已知实际边长 / 像素边长 × 焦距"的单目测距公式
        #    dist = (真实边长 × 焦距) / 像素边长
        #    ★ 这是针孔模型的近似，只在标签正对相机时准
        dist = (self.tag_size * self.fx) / max(1.0, tag_px)

        # ③ 纵向误差
        ez = dist - self.target_dist

        # ── P 控制 ──
        # 线速度：距离误差 × 增益，然后限幅
        v = self.kp_dist * ez
        if v > 0:
            # 前进时保证有最小速度（克服电机死区）
            if v < self.v_min: v = self.v_min
            if v > self.v_max: v = self.v_max
        else:
            # 后退（离太近了）
            if v < -self.v_max: v = -self.v_max

        # 到位判定
        if dist < self.stop_dist:
            v = 0.0

        # 角速度：横向误差 × 增益（反向，因为标签偏右要右转 = 负 ω）
        w = -self.kp_yaw * ex_norm
        if w >  self.w_max: w =  self.w_max
        if w < -self.w_max: w = -self.w_max

        # ★ 阿克曼特性：ω 只在有速度时有效
        #   如果 v=0 但 w≠0，下位机会"原地打方向"（舵机转但车不动）
        #   对准阶段这样反而好（先打死方向再走）
        self.cmd_v = v
        self.cmd_w = w
        self.last_err = (ex_norm, ez, dist)

    # ================================================================
    def detect(self, gray):
        """检测 AprilTag，返回 (cx, cy, 像素边长) 或 None"""
        h, w = gray.shape[:2]

        if self.detector_type == 'pupil':
            # pupil_apriltags 需要灰度图 + 相机内参（用于返回位姿）
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=False,      # 我们只用图像坐标，不用 3D 位姿
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size,
            )
            for t in tags:
                if t.tag_id != self.tag_id:
                    continue
                # t.center 是标签中心 (x, y)
                # 用四个角点算像素边长（取平均）
                c = t.corners
                side = (abs(c[1][0]-c[0][0]) + abs(c[2][0]-c[1][0])) / 2.0
                return (float(t.center[0]), float(t.center[1]), float(side))
            return None

        else:
            # cv2.aruco 分支（新旧 API 分别调用）
            kind, obj = self.detector
            if kind == 'new':
                corners, ids, _ = obj.detectMarkers(gray)
            else:
                d, p = obj
                corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=p)
            if ids is None:
                return None
            for i, tid in enumerate(ids.flatten()):
                if tid != self.tag_id:
                    continue
                pts = corners[i][0]
                cx = float(pts[:, 0].mean())
                cy = float(pts[:, 1].mean())
                side = float(np.linalg.norm(pts[0] - pts[1]))
                return (cx, cy, side)
            return None

    # ================================================================
    #  定时器：稳定发速度（不受相机帧率影响）
    # ================================================================
    def on_timer(self):
        v, w = self.cmd_v, self.cmd_w

        # 安全：超过 1 秒没看到标签 → 归零
        if self.last_seen > 0 and (time.time() - self.last_seen) > 1.0:
            v = 0.0
            w = 0.0

        # ① 发串口（下位机）
        if self.serial_ok:
            try:
                # ★ 协议："V,<linearX>,<angularZ>\n"
                #   对应下位机 UartProtocol.cpp 的解析
                self.ser.write(f"V,{v:.3f},{w:.3f}\n".encode())
            except Exception as e:
                self.get_logger().error(f'串口发送失败: {e}')
                self.serial_ok = False

        # ② 发 ROS 话题（调试/记录）
        tw = Twist()
        tw.linear.x  = float(v)
        tw.angular.z = float(w)
        self.pub_cmd.publish(tw)

    # ================================================================
    def destroy_node(self):
        # ★ 退出前必须停车 + 关串口
        self.get_logger().info('🛑 退出：停车')
        if self.serial_ok:
            try:
                self.ser.write(b"V,0.000,0.000\n")
                time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AlignNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n👋 Ctrl+C')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
