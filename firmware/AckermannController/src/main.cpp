// ============================================================
// main.cpp —— 阿克曼底盘下位机主程序
//
// 【整体架构】
//
//   RDK X3（上位机）                   Mega2560（下位机）
//   ┌──────────────┐   Serial1    ┌──────────────────────┐
//   │ 视觉 / 雷达   │ ──────────→  │ UartProtocol 解析指令 │
//   │ 决策         │  v, ω        │        ↓             │
//   └──────────────┘              │ AckermannDrive       │
//                                 │   ├→ ServoSteering → 舵机(D5)
//                                 │   └→ MotorDriverI2C → 电机模块(I²C 0x34)
//                                 └──────────────────────┘
//
// 【控制流】
//   50Hz 主循环：
//     ① 解析串口（收上位机指令）
//     ② 检查指令超时（断了就停车 —— 安全兜底）
//     ③ 下发到执行器
//
// 【为什么要有超时保护】
//   如果上位机崩溃 / 线松了，下位机不能继续执行最后的指令 ——
//   那车会一直往前冲。所以超过 CMD_TIMEOUT_MS 没收到新指令就停车。
//   ★ 这是机器人安全的基本要求，不能省。
// ============================================================

#include <Arduino.h>
#include "AckermannConfig.h"
#include "MotorDriverI2C.h"
#include "UartProtocol.h"
#include "ServoSteering.h"
#include "AckermannDrive.h"

// ---------- 全局对象 ----------
static MotorDriverI2C   g_motor;
static ServoSteering    g_steer;
static AckermannDrive   g_drive(g_motor, g_steer);

static unsigned long    g_lastCmdMs   = 0;      // 上次收到指令的时刻
static bool             g_wasTimeout  = false;  // 超时状态（用于只打印一次）

// ---------- 打印辅助 ----------
static void printState(float v, float steerDeg, int8_t cmd)
{
    DEBUG_SERIAL.print(F("[状态] v="));
    DEBUG_SERIAL.print(v, 2);
    DEBUG_SERIAL.print(F("m/s  steer="));
    DEBUG_SERIAL.print(steerDeg, 1);
    DEBUG_SERIAL.print(F("°  cmd="));
    DEBUG_SERIAL.print(cmd);
    DEBUG_SERIAL.print(F("  R="));
    DEBUG_SERIAL.print(g_drive.getTurnRadius(), 0);
    DEBUG_SERIAL.println(F("mm"));
}

// ============================================================
void setup()
{
    // ① 调试串口（USB）
    DEBUG_SERIAL.begin(115200);
    delay(200);
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("============================================"));
    DEBUG_SERIAL.println(F(" JB 阿克曼底盘 · 下位机 v1.0"));
    DEBUG_SERIAL.println(F("============================================"));

    // ② 上位机串口
    UartProtocol::begin();

    // ③ 电机驱动（I²C）
    g_motor.begin();
    g_motor.stop();

    // ④ 转向舵机
    g_steer.begin();

    // ⑤ 运动控制
    g_drive.begin();

    g_lastCmdMs = millis();

    DEBUG_SERIAL.println(F("[系统] 初始化完成，等待上位机指令..."));
    DEBUG_SERIAL.println(F("============================================"));
}

// ============================================================
void loop()
{
    // ── ① 解析串口（非阻塞，每圈都调）──
    UartProtocol::update();

    // ── ② 收到新指令？──
    if (UartProtocol::hasNewCommand()) {
        float v  = UartProtocol::getLinearX();
        float wz = UartProtocol::getAngularZ();

        g_drive.setVelocity(v, wz);
        g_lastCmdMs  = millis();
        g_wasTimeout = false;

        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 200) {          // 限流打印，别刷屏
            printState(v, g_drive.getSteerDeg(), g_drive.getMotorCmd());
            lastPrint = millis();
        }
    }

    // ── ③ 指令超时保护 ──
    //   ★ 关键安全机制：上位机断了必须停车
    if (millis() - g_lastCmdMs > CMD_TIMEOUT_MS) {
        if (!g_wasTimeout) {
            DEBUG_SERIAL.println(F("[安全] ⚠️ 指令超时，自动停车！"));
            g_drive.stop();
            g_wasTimeout = true;
        }
    }

    // ── ④ 固定周期 ──
    //   用 delay 实现（简单）；如果要更准可以用 millis 做时间片
    delay(1000 / CONTROL_HZ);
}
