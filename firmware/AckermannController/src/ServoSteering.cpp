#include "ServoSteering.h"
#include <Servo.h>
// ★ Servo.h 是 Arduino 自带的舵机库
//   它内部用定时器产生 50Hz 的 PWM，脉宽 1000~2000µs 对应 0~180°

// ============================================================
// 标定表：角度(度) → 脉宽(µs)
//
// ★ 顺序必须【从小到大】排列，插值算法依赖这个顺序。
//   数据来自档案/23-当前基准-最终版.md 的实测值。
// ============================================================
static const float CAL_ANGLE[] = { -20.0f, -10.0f,   0.0f,  10.0f,  20.0f,  35.0f };
static const int   CAL_PULSE[] = {  1709,   1604,   1500,   1391,   1275,   1098 };
static const int   CAL_N       = sizeof(CAL_ANGLE) / sizeof(CAL_ANGLE[0]);

static Servo s_servo;   // 舵机对象（全局，因为 Servo 库要求）

// ============================================================
void ServoSteering::begin()
{
    s_servo.attach(SERVO_PIN);
    center();                       // 上电先回中位 —— 安全
    DEBUG_SERIAL.println(F("[Servo] init OK, center=1500us"));
}

// ============================================================
// 核心：查表 + 线性插值
//
// 例如 angle = 5°，落在 [0°, 10°] 区间：
//   比例 t = (5-0)/(10-0) = 0.5
//   脉宽 = 1500 + 0.5*(1391-1500) = 1445.5 → 1445
//
// 超出表格范围 → 取端点值（外推很危险，一律钳位）
// ============================================================
int ServoSteering::angleToPulse(float angleDeg)
{
    // ── ① 限幅：绝不超出表格两端 ──
    if (angleDeg <= CAL_ANGLE[0])        return CAL_PULSE[0];
    if (angleDeg >= CAL_ANGLE[CAL_N-1])  return CAL_PULSE[CAL_N-1];

    // ── ② 找到 angleDeg 落在哪个区间 ──
    for (int i = 0; i < CAL_N - 1; i++) {
        float a0 = CAL_ANGLE[i], a1 = CAL_ANGLE[i+1];
        if (angleDeg >= a0 && angleDeg <= a1) {
            float t = (angleDeg - a0) / (a1 - a0);       // 0~1 的比例
            int   p = CAL_PULSE[i] + (int)(t * (CAL_PULSE[i+1] - CAL_PULSE[i]) + 0.5f);
            // ③ 二次安全钳位（防标定数据异常）
            if (p < SERVO_US_MIN) p = SERVO_US_MIN;
            if (p > SERVO_US_MAX) p = SERVO_US_MAX;
            return p;
        }
    }
    return SERVO_US_CENTER;   // 理论上到不了这里
}

// ============================================================
void ServoSteering::setAngle(float angleDeg)
{
    // ★ 机械限幅：软件再挡一层，防上位机发来离谱的角度
    if (angleDeg >  SERVO_ANGLE_MAX) angleDeg =  SERVO_ANGLE_MAX;
    if (angleDeg < -SERVO_ANGLE_MAX) angleDeg = -SERVO_ANGLE_MAX;

    _angle   = angleDeg;
    _pulseUs = angleToPulse(angleDeg);
    s_servo.writeMicroseconds(_pulseUs);
    // ★ 用 writeMicroseconds 而不是 write(angle)
    //   因为 write() 用的是库内部的角度映射，我们有自己的标定表
}

// ============================================================
void ServoSteering::center()
{
    setAngle(0.0f);
}
