#pragma once
// ============================================================
// AckermannConfig.h —— 阿克曼底盘的全部可调参数
//
// ★ 所有"魔数"集中在这里，改参数只改这一个文件。
//   这是嵌入式项目的基本纪律：不要把数字散落在代码里。
// ============================================================

// ---------- 通信 ----------
#define UART_BAUDRATE   115200        // 上位机串口波特率
#define UART_SERIAL     Serial1       // 上位机接 Serial1 (D18=TX1, D19=RX1)
                                      // ★ Serial0 留给 USB 调试 —— 插着 USB 也能收指令
#define DEBUG_SERIAL    Serial        // 调试输出（USB）

// ---------- 舵机（转向）----------
// 舵机接 D5（OC3A，独立定时器，不被其他 PWM 干扰）
#define SERVO_PIN       5

// ★ 舵机脉宽标定表（来自 档案/23-当前基准-最终版.md）
//   左轮转角 → 脉宽（微秒）
//   注意：这是【实测标定值】，不是线性推算的
#define SERVO_US_LEFT_20  1709        // 左轮 -20°（=右转）
#define SERVO_US_LEFT_10  1604
#define SERVO_US_CENTER   1500        // 中位（直行）★ 关键基准
#define SERVO_US_LEFT_P10 1391        // 左轮 +10°
#define SERVO_US_LEFT_P20 1275
#define SERVO_US_LEFT_P35 1098        // 左轮 +35°（最大左转）

#define SERVO_ANGLE_MAX   35.0f       // 机械最大转角（度）
#define SERVO_US_MIN      1000        // 脉宽安全下限（防堵转烧舵机）
#define SERVO_US_MAX      2000        // 脉宽安全上限

// ---------- 底盘几何 ----------
#define WHEEL_DIAMETER_MM   65.0f     // 车轮直径（mm）★ 装好后实测再改
                                      // ⚠️ 用卷尺量轮子外径，或在地上滚一圈量距离/π
#define WHEEL_BASE_MM       200.0f    // 轴距 L（mm）—— 档案基准
#define TRACK_WIDTH_MM      237.0f    // 前轮距（mm）
#define MAX_SPEED_MS        1.5f      // 最大线速度（m/s）—— 保守值

// ---------- 电机 ----------
// hiwonder 四路模块的速度范围约 -50 ~ +50
#define MOTOR_SPEED_MAX     50
// 驱动方式：1 = 后驱（用 M3/M4，阿克曼标准）
//           2 = 四驱（M1~M4 同速，转向时会滑）
#define DRIVE_MODE          1

// ---------- 控制周期 ----------
#define CONTROL_HZ          50        // 控制频率（Hz）→ 周期 20ms
#define CMD_TIMEOUT_MS      500       // 上位机指令超时（ms）→ 超时自动停车
