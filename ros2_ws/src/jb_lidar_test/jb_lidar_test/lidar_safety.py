#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JB 阿克曼底盘 · 雷达安全节点
================================================

【这个节点干什么】
    把雷达的一圈数据，压缩成"车能不能往前走"的判断。

【为什么需要它】
    阿克曼底盘（像汽车）和差速底盘（像坦克）不一样：
      · 差速车：原地转圈就能躲开
      · 阿克曼车：**只能往前开+打方向**，转弯需要空间
    所以阿克曼更需要"提前知道前面有没有路"。

【分区策略】
    把前方 180° 分成三个扇区（可调）：

            左前            正前            右前
        ┌──────────┬──────────────┬──────────┐
        │ +90°~+30°│ +30°~-30°    │ -30°~-90°│
        └──────────┴──────────────┴──────────┘
              ↑                          ↑
           向左打方向时要看            向右打方向时要看

    · 正前最近 < stop_dist   → 🔴 必须停
    · 左前最近 < warn_dist   → ⚠️ 不能向左打
    · 右前最近 < warn_dist   → ⚠️ 不能向右打

【输出】
    发布 /lidar_safety（std_msgs/String）：
      "CLEAR"  /  "SLOW"  /  "STOP"
    下游（决策节点）订阅它，决定要不要覆盖速度指令。

【跑法】
    ros2 run jb_lidar_test lidar_safety
    ros2 run jb_lidar_test lidar_safety --ros-args -p stop_dist:=0.4 -p warn_dist:=1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math


class LidarSafetyNode(Node):

    def __init__(self):
        super().__init__('jb_lidar_safety')

        # ── 参数 ──
        self.declare_parameter('stop_dist', 0.35)    # 正前 < 这个距离 → 停
        self.declare_parameter('warn_dist', 0.9)     # 侧前 < 这个距离 → 不能再往那边打
        self.declare_parameter('front_half_deg', 30.0)  # 正前扇区的半角
        self.declare_parameter('side_deg', 30.0)        # 侧前扇区的角度宽度
        self.declare_parameter('report_hz', 2.0)        # 打印频率

        self.stop_dist = self.get_parameter('stop_dist').value
        self.warn_dist = self.get_parameter('warn_dist').value
        self.front_half = self.get_parameter('front_half_deg').value
        self.side_w = self.get_parameter('side_deg').value
        report_hz = self.get_parameter('report_hz').value

        # ── 订阅雷达 + 发布安全状态 ──
        # ★ QoS 必须匹配雷达驱动（BEST_EFFORT），否则收不到数据
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.pub = self.create_publisher(String, '/lidar_safety', 10)

        # ── 定时打印（不跟扫描频率绑定，避免刷屏）──
        self.last_state = 'CLEAR'
        self.timer = self.create_timer(1.0 / max(0.1, report_hz), self.report)

        # 缓存最近一次的分区结果
        self.zone = {'left': float('inf'), 'front': float('inf'), 'right': float('inf')}

        self.get_logger().info('=' * 60)
        self.get_logger().info('🛡  雷达安全节点')
        self.get_logger().info(f'   正前扇区: ±{self.front_half:.0f}°   停止距离: {self.stop_dist} m')
        self.get_logger().info(f'   侧前扇区: 各 {self.side_w:.0f}° 宽   预警距离: {self.warn_dist} m')
        self.get_logger().info(f'   发布: /lidar_safety (CLEAR/SLOW/STOP)')
        self.get_logger().info('=' * 60)

    # ================================================================
    def on_scan(self, msg: LaserScan):
        """把一圈数据压缩成三个扇区的最小距离"""
        # 三个扇区初始为无穷远
        z = {'left': float('inf'), 'front': float('inf'), 'right': float('inf')}

        for i, r in enumerate(msg.ranges):
            # 过滤无效值
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)

            # 按角度归入扇区
            #   正前: [-front_half, +front_half]
            if -self.front_half <= ang <= self.front_half:
                if r < z['front']: z['front'] = r
            #   左前: [+front_half, +front_half+side_w]
            elif self.front_half < ang <= self.front_half + self.side_w:
                if r < z['left']: z['left'] = r
            #   右前: [-front_half-side_w, -front_half]
            elif -self.front_half - self.side_w <= ang < -self.front_half:
                if r < z['right']: z['right'] = r

        self.zone = z

        # ── 决策 ──
        # 优先级：正前停 > 侧前预警 > 通畅
        if z['front'] < self.stop_dist:
            state = 'STOP'
        elif z['left'] < self.warn_dist or z['right'] < self.warn_dist:
            state = 'SLOW'
        else:
            state = 'CLEAR'

        # ── 只在状态变化时发布（避免刷爆话题）──
        if state != self.last_state:
            out = String()
            out.data = state
            self.pub.publish(out)
            self.get_logger().info(f'🔄 状态变化: {self.last_state} → {state}')
            self.last_state = state

    # ================================================================
    def report(self):
        """定期打印三个扇区的距离"""
        z = self.zone
        def fmt(v): return f'{v:.2f}' if math.isfinite(v) else '  ∞ '
        icon = {'CLEAR': '🟢', 'SLOW': '🟡', 'STOP': '🔴'}.get(self.last_state, '?')
        self.get_logger().info(
            f'{icon} {self.last_state:<5} | 左前 {fmt(z["left"])}m  '
            f'正前 {fmt(z["front"])}m  右前 {fmt(z["right"])}m'
        )

    def destroy_node(self):
        self.get_logger().info('👋 退出')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarSafetyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n👋 收到 Ctrl+C')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
