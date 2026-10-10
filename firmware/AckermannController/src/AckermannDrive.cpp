#include "AckermannDrive.h"
#include <math.h>

// ============================================================
AckermannDrive::AckermannDrive(MotorDriverI2C& motorDriver, ServoSteering& steering)
    : motor(motorDriver), steer(steering)
{
}

// ============================================================
void AckermannDrive::begin()
{
    steer.begin();
    motor.stop();
    DEBUG_SERIAL.println(F("[Ackermann] init OK"));
    DEBUG_SERIAL.print(F("  L="));      DEBUG_SERIAL.print(WHEEL_BASE_MM);
    DEBUG_SERIAL.print(F("mm  dmax=")); DEBUG_SERIAL.print(SERVO_ANGLE_MAX);
    // 最小转弯半径 = L / tan(δmax)
    float rmin = WHEEL_BASE_MM / tanf(SERVO_ANGLE_MAX * PI / 180.0f);
    DEBUG_SERIAL.print(F("deg  Rmin=")); DEBUG_SERIAL.print(rmin);
    DEBUG_SERIAL.println(F("mm"));
}

// ============================================================
// 线速度 → 电机指令值
//
// 我们【不做】精确的 RPM 换算（那需要轮径、减速比、编码器线数），
// 而是用一个【标定比例】：把 MAX_SPEED_MS 映射到 MOTOR_SPEED_MAX。
//
// 为什么这样够用？
//   · 需要精确速度时，用编码器做闭环（那是下一步）
//   · 现在只要"能按比例走"，开环映射就够
//   · 而且轮径/减速比这些参数未必测得准，算出来反而是假精度
//
// ★ 这是嵌入式常见的取舍：先开环跑通，再上闭环。
// ============================================================
int8_t AckermannDrive::mpsToMotorCmd(float v)
{
    // 除以最大速度 → 归一化到 [-1, 1]
    float norm = v / MAX_SPEED_MS;

    // 限幅
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    // 映射到电机指令范围
    int cmd = (int)(norm * MOTOR_SPEED_MAX);

    // 死区处理：很小的值直接归零，避免电机嗡嗡响不转
    if (cmd > -2 && cmd < 2) cmd = 0;

    return (int8_t)cmd;
}

// ============================================================
// 把电机指令发给驱动板
//
// DRIVE_MODE 决定哪几个电机出力：
//   1 = 后驱（M3, M4）—— 阿克曼标准布局
//   2 = 四驱（M1~M4）
//
// ⚠️ 电机方向（正负）取决于接线，装好后要实测标定：
//    如果某个轮子反转，就在这里把它的符号取反
// ============================================================
void AckermannDrive::applyMotor(int8_t cmd)
{
#if DRIVE_MODE == 1
    // 后驱：M1/M2 空转（前轮从动），M3/M4 驱动
    // ★ M3/M4 方向可能相反（左右对称安装），实测后调这里
    motor.setSpeed(0, 0, -cmd, cmd);
#else
    // 四驱：四个轮同速
    motor.setSpeed(-cmd, cmd, -cmd, cmd);
#endif
}

// ============================================================
// cmd_vel 风格接口
// ============================================================
void AckermannDrive::setVelocity(float linearX, float angularZ)
{
    _v = linearX;

    // ── 角速度 → 前轮转角 ──
    //   δ = atan( ω · L / v )
    //
    // ⚠️ v 接近 0 时数学上会除零。
    //   物理上：车停着不可能有角速度。
    //   工程上：这种情况下"原地打方向"——舵机转到位，但车不走。
    //          这对阿克曼是合法的（前轮可以原地转，车不动）。
    const float V_EPS = 0.02f;      // 2cm/s 以下认为是静止
    float steerDeg;

    if (fabsf(linearX) < V_EPS) {
        // 静止：不计算角速度，直接把舵机打到最大角（如果要转的话）
        // 这样"原地打方向"也能用 —— 调试时很有用
        if (fabsf(angularZ) > 1e-3f) {
            steerDeg = (angularZ > 0) ? SERVO_ANGLE_MAX : -SERVO_ANGLE_MAX;
        } else {
            steerDeg = 0.0f;
        }
    } else {
        float L_m = WHEEL_BASE_MM / 1000.0f;            // mm → m
        float deltaRad = atanf(angularZ * L_m / linearX);
        steerDeg = deltaRad * 180.0f / PI;               // rad → 度
    }

    setVelocitySteer(linearX, steerDeg);
}

// ============================================================
// 直接指定转角（跳过 ω 反算）
// ============================================================
void AckermannDrive::setVelocitySteer(float linearX, float steerDeg)
{
    _v = linearX;

    // 限幅
    if (steerDeg >  SERVO_ANGLE_MAX) steerDeg =  SERVO_ANGLE_MAX;
    if (steerDeg < -SERVO_ANGLE_MAX) steerDeg = -SERVO_ANGLE_MAX;
    _steerDeg = steerDeg;

    // 转向
    steer.setAngle(steerDeg);

    // 驱动
    _motorCmd = mpsToMotorCmd(linearX);
    applyMotor(_motorCmd);
}

// ============================================================
void AckermannDrive::stop()
{
    _v = 0.0f;
    _motorCmd = 0;
    motor.stop();
    // ★ 舵机【不回中】：停车时保持转向角，方便"停在框里还带着角度"
}

void AckermannDrive::centerSteering()
{
    steer.center();
    _steerDeg = 0.0f;
}

// ============================================================
float AckermannDrive::getTurnRadius() const
{
    // R = L / tan(δ)
    float rad = _steerDeg * PI / 180.0f;
    if (fabsf(rad) < 1e-4f) return 1e9f;    // 直行 = 无穷大半径
    return fabsf(WHEEL_BASE_MM / tanf(rad));
}
