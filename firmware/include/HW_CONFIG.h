#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include <Arduino.h>

// ==========================================================
// JB-board V1.2 · 阿克曼底盘背板 · 引脚映射
// 主控：RobotDyn Mega 2560 PRO MINI (ATmega2560-16AU)
// ⚠️ 本文件与 docs/02-引脚分配.md 必须保持一致
// 版本：V1.2-draft  日期：2026-10-05
// ==========================================================

// ---------- 0. 整车几何常量（阿克曼）----------
// 参见 档案/23-当前基准-最终版.md
#define CAR_WHEELBASE_MM      200.0f   // 轴距 L
#define CAR_KINGPIN_W_MM      161.0f   // 主销中心距 W
#define CAR_KINGPIN_OFFSET_MM  38.0f   // 主销偏移距 s
#define CAR_TRACK_FRONT_MM    237.0f   // 前轮距
#define CAR_TRACK_REAR_MM     225.0f   // 后轮距

// 舵机脉宽标定（μs）
#define SERVO_PULSE_CENTER   1500
#define SERVO_PULSE_MIN      1098      // 对应 +35°（右）
#define SERVO_PULSE_MAX      1709      // 对应 -20°（左）

// ---------- 1. 转向舵机 ----------
constexpr uint8_t PIN_SERVO_PWM   = 5;    // OC3A，独立定时器

// ---------- 2. I²C 总线（⚠️ 经电平转换到 3.3V）----------
// 挂载：AS5600(0x36) + MPU6050(0x68/0x69) + hiwonder 电机模块(?)
// 使用 Wire (D20=SDA, D21=SCL)，无需重定义，此处仅作记录
#define I2C_ADDR_AS5600       0x36
#define I2C_ADDR_MPU6050      0x68    // AD0 拉高则为 0x69

// ---------- 3. 上位机通信（RDK X3）----------
// 用 Serial1，把 Serial0(D0/D1) 留给 USB 调试
#define UART_UPSTREAM         Serial1
constexpr uint8_t PIN_UPC_TX      = 18;   // TX1 ⚠️ 分压到 3.3V
constexpr uint8_t PIN_UPC_RX      = 19;   // RX1

// ---------- 4. 语音模块 JQ8900-16P ----------
// 用 Serial2；RX 需分压 (1k+2k)，DC-5V 由 5V 支路供
#define UART_AUDIO            Serial2
constexpr uint8_t PIN_AUDIO_TX    = 16;   // TX2 ⚠️ 必须分压
constexpr uint8_t PIN_AUDIO_RX    = 17;   // RX2
constexpr uint8_t PIN_AUDIO_BUSY  = 22;   // 播放指示（高=在播）

// ---------- 5. 电池监测 ----------
constexpr uint8_t PIN_VBAT_ADC    = A0;   // 分压后接入
// 分压比依实际电阻填写（例如 10k:2.2k → 比值 5.545）
#define VBAT_DIVIDER_RATIO    5.545f
#define VBAT_CELL_MIN_MV      3300    // 单节 3.3V 告警
#define VBAT_CELL_STOP_MV     3000    // 单节 3.0V 强制停

// ---------- 6. 人机交互 ----------
constexpr uint8_t PIN_LED_PWR     = 26;
constexpr uint8_t PIN_LED_RUN     = 27;
constexpr uint8_t PIN_LED_FAULT   = 28;
constexpr uint8_t PIN_LED_COMM    = 29;
constexpr uint8_t PIN_BUZZER      = 4;
constexpr uint8_t PIN_ESTOP       = 2;    // INT4，低电平=急停

// ---------- 7. 硬件初始化 ----------
void Hardware_Init();

#endif // HW_CONFIG_H
