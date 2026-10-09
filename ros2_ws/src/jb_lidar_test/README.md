# jb_lidar_test · 雷达测试包

> 硬件：**YDLIDAR T-mini Plus**（`/dev/ttyUSB0`，230400，型号码 151）

## 两个节点

| 可执行 | 作用 |
|---|---|
| `lidar_test` | 订阅 `/scan`，打印点数/角度范围/最近障碍/前方扇区预警 |
| `lidar_safety` | 分左前/正前/右前三个扇区，发布 `/lidar_safety`（CLEAR/SLOW/STOP）|

## 跑法

```bash
# 一键（驱动 + 测试 + 安全）
ros2 launch jb_lidar_test lidar_test.launch.py

# 或单独跑（驱动要先起来）
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  -p port:=/dev/ttyUSB0 -p lidar_type:=1 -p sample_rate:=4 \
  -p isSingleChannel:=false -p frequency:=10.0
ros2 run jb_lidar_test lidar_test
ros2 run jb_lidar_test lidar_safety
```

## 实测（2026-10-09）

```
扫描参数: 400 个点 | 角度 -180°~+180° | 步进 0.90° | 量程 0.10~64 m | 一圈 0.100s
有效点: 135~144 / 400（室内，很多角度超量程，正常）
安全节点: 正前 0.29m → 正确判定 STOP
```

## ⚠️ 踩过的坑

### 1. **QoS 不匹配（最隐蔽的坑）**

```
[WARN] New publisher discovered on topic '/scan', offering incompatible QoS.
       No messages will be received from it. Last incompatible policy: RELIABILITY
```

**根因**：`ydlidar_ros2_driver` 用 `BEST_EFFORT` 发布，订阅端默认是 `RELIABLE` → **不兼容，一条数据都收不到，而且不报错！**

**修法**：订阅传感器数据一律用 BEST_EFFORT
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,   # ★ 关键
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
```

> **★ 通用规则**：订阅 **雷达 / 相机 / IMU** 等传感器话题时，
> 一律用 `rclpy.qos.qos_profile_sensor_data` 或手写 `BEST_EFFORT`。

### 2. 驱动默认端口是 `/dev/ydlidar`

需要建 udev 规则软链，或者命令行 `-p port:=/dev/ttyUSB0`：
```bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"' \
  | sudo tee /etc/udev/rules.d/ydlidar.rules
sudo udevadm control --reload && sudo udevadm trigger
```

## T-mini Plus 权威参数

```yaml
port:            /dev/ttyUSB0     # 或 /dev/ydlidar（需 udev）
baudrate:        230400
lidar_type:      1                # TYPE_TRIANGLE（三角测距）
device_type:     0                # TYPE_SERIAL
sample_rate:     4
isSingleChannel: false
frequency:       10.0             # Hz
frame_id:        laser_frame
range_min:       0.02
range_max:       12.0
```

**型号码**：`140=Tmini  150=TminiPro  151=TminiPlus ★  152=TminiPlusSH(森合,TOF)`
