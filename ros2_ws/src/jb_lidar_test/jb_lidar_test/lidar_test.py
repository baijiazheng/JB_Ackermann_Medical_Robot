#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JB 阿克曼底盘 · 雷达测试节点（YDLIDAR T-mini Plus）
================================================

【这个节点干什么】
    订阅 /scan（LaserScan 消息），把雷达的一圈扫描解析成人能看懂的数字：
      · 这一圈有多少个点
      · 角度范围
      · 最近障碍物有多远、在哪个方向
      · 车头正前方（分几个扇区）有没有障碍

【★ 先搞懂 LaserScan 消息结构（这是雷达数据的"标准信封"）】

    sensor_msgs/LaserScan 的字段：

      header.stamp      扫描【起始】时刻（注意：不是每个点的时刻）
      header.frame_id   坐标系名，通常是 "laser_frame"

      angle_min         起始角度（弧度）。T-mini Plus 是 -π
      angle_max         结束角度（弧度）。T-mini Plus 是 +π
      angle_increment   相邻两个点之间的角度步进 = (angle_max-angle_min)/点数

      time_increment    相邻两个点的时间差（秒）
      scan_time         扫一整圈的时间（秒）。10Hz → 0.1s

      range_min         有效最小距离（米）
      range_max         有效最大距离（米）

      ranges[]          ★ 核心数组：每个角度上的距离（米）
                        索引 i 对应的角度 = angle_min + i * angle_increment
                        ⚠️ 无效值可能是 inf 或 nan（超出量程/反射太弱）

      intensities[]     反射强度（可选，有些雷达没有）

【★ 怎么把索引 i 换算成角度/方向】
    angle = angle_min + i * angle_increment

    ROS 的约定（右手系）：
      angle = 0      → 正前方（x 轴）
      angle > 0      → 左侧（逆时针）
      angle < 0      → 右侧
      角度单位是【弧度】，转成角度：deg = rad * 180 / π

【跑法】
    ros2 run jb_lidar_test lidar_test
    # 只看前方 60° 扇区，只报 1.5m 内的
    ros2 run jb_lidar_test lidar_test --ros-args -p front_deg:=60.0 -p warn_dist:=1.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan       # ★ 雷达扫描消息
import math                                  # 角度换算
# ★ QoS：ROS2 的"服务质量"配置。雷达驱动用 BEST_EFFORT 发布，
#   订阅端如果还用默认的 RELIABLE，两边对不上就一条数据都收不到。
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class LidarTestNode(Node):

    def __init__(self):
        super().__init__('jb_lidar_test')

        # ── 参数 ──
        self.declare_parameter('front_deg', 90.0)    # "车头前方"的扇区总角度（度）
        self.declare_parameter('warn_dist', 1.0)     # 预警距离（米）
        self.declare_parameter('report_every', 20)   # 每多少圈打印一次
        self.declare_parameter('topic', '/scan')     # 订阅的话题名

        self.front_deg  = self.get_parameter('front_deg').value
        self.warn_dist  = self.get_parameter('warn_dist').value
        self.report_every = self.get_parameter('report_every').value
        topic = self.get_parameter('topic').value

        # ── 订阅 ──
        # ★★ QoS 必须匹配！这是 ROS2 最经典的坑之一。
        #
        #   QoS 关键字段：
        #     reliability : RELIABLE（可靠，丢包重传）vs BEST_EFFORT（尽力而为，丢了就丢）
        #     history     : KEEP_LAST（只留最近 N 条）vs KEEP_ALL（全留）
        #     depth       : 配合 KEEP_LAST，缓存多少条
        #     durability  : VOLATILE（不补发历史）vs TRANSIENT_LOCAL（新订阅者能收到历史）
        #
        #   ydlidar_ros2_driver 用的是 sensor-data QoS：
        #     BEST_EFFORT + KEEP_LAST(1) + VOLATILE
        #
        #   ⚠️ 如果订阅端用默认的 RELIABLE，两边不兼容，
        #      你会看到 "offering incompatible QoS... No messages will be received"
        #      而且【不报错，只是静默收不到数据】—— 很难查！
        #
        #   规则：订阅传感器数据（雷达/相机/IMU）时，
        #        一律用 rclpy.qos.qos_profile_sensor_data 或手写 BEST_EFFORT。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # ★ 关键
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                                     # 只要最新一圈
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(LaserScan, topic, self.on_scan, qos)

        self.count = 0
        self.warn_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📡 订阅 {topic}')
        self.get_logger().info(f'   前方扇区: ±{self.front_deg/2:.0f}°   预警距离: {self.warn_dist} m')
        self.get_logger().info('=' * 60)

    # ================================================================
    def on_scan(self, msg: LaserScan):
        """每收到一圈扫描调用一次"""
        self.count += 1

        # ── ① 基本信息（第一圈时打印一次，后面的信息都一样）──
        if self.count == 1:
            n = len(msg.ranges)
            self.get_logger().info(
                f'📐 扫描参数: {n} 个点 | '
                f'角度 {math.degrees(msg.angle_min):.1f}° ~ {math.degrees(msg.angle_max):.1f}° | '
                f'步进 {math.degrees(msg.angle_increment):.2f}° | '
                f'量程 {msg.range_min:.2f}~{msg.range_max:.1f} m | '
                f'一圈 {msg.scan_time:.3f}s'
            )

        # ── ② 遍历一圈，找出有效数据 ──
        # ★ 关键技巧：radar 的无效值是 inf/nan，必须过滤
        valid = []          # [(角度(度), 距离(米)), ...]
        inc_deg = math.degrees(msg.angle_increment)

        for i, r in enumerate(msg.ranges):
            # 过滤无效值：
            #   math.isfinite(r)  排除 inf 和 nan
            #   range_min/max     排除超量程的
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            valid.append((ang, r))

        if not valid:
            self.get_logger().warn('⚠️ 这一圈没有有效点')
            return

        # ── ③ 找最近障碍 ──
        # min(可迭代, key=...) 返回"按 key 排序后最小的那个元素"
        # 这里 key 用 lambda 取元组的第二个元素（距离）
        near_ang, near_dist = min(valid, key=lambda t: t[1])

        # ── ④ 车头前方扇区统计 ──
        # 前方 = 角度在 [-front_deg/2, +front_deg/2] 内的点
        half = self.front_deg / 2.0
        front = [r for ang, r in valid if -half <= ang <= half]
        front_min = min(front) if front else float('inf')

        # ── ⑤ 定期打印 ──
        if self.count % self.report_every == 0:
            self.get_logger().info(
                f'📊 第 {self.count} 圈 | 有效点 {len(valid)}/{len(msg.ranges)} | '
                f'最近 {near_dist:.2f}m @ {near_ang:+.0f}° | '
                f'前方({self.front_deg:.0f}°)最近 {front_min:.2f}m'
            )

        # ── ⑥ 安全预警 ──
        # ★ 这就是这个节点真正有用的地方：
        #   前方 warn_dist 米内有东西 → 报警
        if front_min < self.warn_dist:
            self.warn_count += 1
            # 每 10 次报一次，避免刷屏
            if self.warn_count % 10 == 1:
                self.get_logger().warn(
                    f'🚨 前方 {front_min:.2f}m 有障碍！'
                    f'（最近点在 {near_ang:+.0f}°, {near_dist:.2f}m）'
                )

    # ================================================================
    def destroy_node(self):
        self.get_logger().info('👋 退出')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarTestNode()
        rclpy.spin(node)              # 阻塞，处理回调
    except KeyboardInterrupt:
        print('\n👋 收到 Ctrl+C')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
