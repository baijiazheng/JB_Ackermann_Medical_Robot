#pragma once
// ============================================================
// AckermannDrive.h —— 阿克曼底盘运动控制
//
// 【和差速驱动的本质区别】
//
//   差速车（坦克）：
//     输入 v(前进), ω(自转)
//     输出 左轮 = v - ω·W/2,  右轮 = v + ω·W/2
//     → 可以原地转圈
//
//   阿克曼车（汽车）：
//     输入 v(前进), δ(前轮转角)
//     输出 后轮 = v,  舵机 = δ
//     → 【不能原地转】，最小转弯半径 R = L / tan(δ)
//
// 【为什么还保留 cmd_vel 的 (v, ω) 接口】
//   ROS 的 cmd_vel 是通用约定，上位机（导航/遥控）都发这个。
//   所以下位机内部把 ω 反算成 δ：
//
//        δ = atan( ω · L / v )
//
//   推导：阿克曼的角速度 ω = v / R，而 R = L / tan(δ)
//         → ω = v · tan(δ) / L
//         → tan(δ) = ω · L / v
//         → δ = atan( ω · L / v )
//
//   ⚠️ 注意 v→0 时这个式子会炸（除以零）。
//      物理意义：车不动时不可能有角速度。
//      代码里必须处理：v 接近 0 时只转向、不前进（原地打方向）
//
// 【最小转弯半径】
//   R_min = L / tan(δ_max)
//   我们的参数：L = 200mm, δ_max = 35°  →  R_min = 286mm
//   ★ 这个数决定了车能不能进那个框！提前算清楚。
// ============================================================

#include <Arduino.h>
#include "MotorDriverI2C.h"
#include "ServoSteering.h"

class AckermannDrive
{
public:
    AckermannDrive(MotorDriverI2C& motorDriver, ServoSteering& steering);

    void begin();

    // ── 主接口：cmd_vel 风格（上位机直接用这个）──
    //   linearX  : 前进速度 (m/s)，负值 = 后退
    //   angularZ : 角速度 (rad/s)，正 = 左转
    void setVelocity(float linearX, float angularZ);

    // ── 直接接口：指定转角（调试用，或上位机自己算好角度）──
    //   linearX : 前进速度 (m/s)
    //   steerDeg: 前轮转角 (度)，正 = 左转
    void setVelocitySteer(float linearX, float steerDeg);

    // stop()：停车（舵机保持当前角，电机停）
    void stop();

    // centerSteering()：回正
    void centerSteering();

    // ── 查询 ──
    float getSpeed()      const { return _v; }
    float getSteerDeg()   const { return _steerDeg; }
    int8_t getMotorCmd()  const { return _motorCmd; }

    // getTurnRadius()：根据当前转角算转弯半径（mm）—— 调试用
    float getTurnRadius() const;

private:
    MotorDriverI2C& motor;
    ServoSteering&  steer;

    float  _v        = 0.0f;   // 当前线速度 (m/s)
    float  _steerDeg = 0.0f;   // 当前转角 (度)
    int8_t _motorCmd = 0;      // 当前电机指令值 (-50~50)

    // mpsToMotorCmd()：线速度(m/s) → 电机指令值(-50~50)
    int8_t mpsToMotorCmd(float v);

    // applyMotor()：把电机指令发给驱动（按 DRIVE_MODE 分发到各路）
    void applyMotor(int8_t cmd);
};
