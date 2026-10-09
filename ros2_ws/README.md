# ros2_ws · JB 阿克曼底盘上位机工作区

> **在本机写代码 → 验证语法 → `scp` 到 RDK X3 编译运行**
> （X3 上直接用 SSH 编辑不方便，所以本地开发 + 远程部署）

## 环境

| 项 | 值 |
|---|---|
| **目标板** | RDK X3（`ssh rdk`，IP `192.168.172.236`）|
| **系统** | Ubuntu 22.04.5 LTS / 内核 4.14.87 aarch64 |
| **ROS** | Humble（`/opt/ros/humble` + `/opt/tros/humble` TogetheROS）|
| **工作区** | X3 上 `~/ros2_ws`（本机这个目录是源码镜像）|

## 部署流程

```bash
# ① 本机写代码
cd ~/JB_Ackermann_Medical_Robot/ros2_ws/src/<包名>

# ② 本机语法检查（不需要 ROS 环境）
python3 -c "import ast;ast.parse(open('xxx.py',encoding='utf-8').read())"

# ③ 传到 X3
scp -r <包名> rdk:~/ros2_ws/src/

# ④ X3 上编译运行
ssh rdk
source ~/ros2_ws/setup_env.sh
cd ~/ros2_ws && colcon build --packages-select <包名>
ros2 run <包名> <可执行名>
```

## 已有的包

### `jb_camera_test` —— 摄像头测试

| 可执行 | 作用 |
|---|---|
| `camera_test` | 打开 USB 摄像头，发 `/camera/image_raw`，打印实测分辨率/帧率 |
| `camera_sub` | 订阅图像，统计帧率 + **端到端延迟**，可按需存图 |

**跑法：**
```bash
ros2 launch jb_camera_test camera_test.launch.py
# 或单独跑
ros2 run jb_camera_test camera_test --ros-args -p device:=/dev/video8
ros2 run jb_camera_test camera_sub  --ros-args -p save_every_n:=30
```

**实测结果（2026-10-09）：**
```
摄像头 /dev/video8  →  1280x720 @ 30fps  MJPG
发布/订阅帧率: 20.1 fps（USB 2.0 带宽上限）
★ 端到端延迟: 平均 19ms，最大 55ms   → 1m/s 下车只走 1.9cm，完全够用
```

## ⚠️ 踩过的坑

| # | 坑 | 修法 |
|---|---|---|
| **1** | **`ros2 run` 报 `No executable found`，但 colcon 编译"成功"** | **ament_python 包必须有 `setup.cfg`**（指定 `script_dir=$base/lib/<包名>`）否则脚本装不到 `ros2 run` 能找到的地方 |
| **2** | 摄像头只有 640x480，达不到 720p | **必须显式设置 MJPG**：`cap.set(cv2.CAP_PROP_FOURCC, fourcc('MJPG'))`。USB 2.0 带宽不够跑 YUYV 720p |
| **3** | 请求 1920x1080 却拿到别的分辨率 | 摄像头不一定给要的值 → **必须读回实际值**（`cap.get(CAP_PROP_FRAME_WIDTH)`）|

## 硬件设备

| 设备 | 节点 | 说明 |
|---|---|---|
| **摄像头** | `/dev/video8` | USB 2.0 Camera (Microdia)，video0~7 是地平线 ISP 虚拟节点 |
| **雷达** | `/dev/ttyUSB0` | CP2102 USB-UART 桥（待接）|
