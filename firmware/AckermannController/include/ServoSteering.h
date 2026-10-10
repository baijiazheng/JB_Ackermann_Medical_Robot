#pragma once
// ============================================================
// ServoSteering.h —— 转向舵机控制
//
// 【职责】
//   把"前轮要转多少度"翻译成"舵机该给多少微秒的脉宽"。
//
// 【为什么需要标定表而不是线性公式】
//   理想情况下舵机是线性的：角度 ∝ 脉宽。
//   但实际有：
//     · 舵机内部的机械非线性
//     · 连杆机构的几何非线性（摆臂转角的 sin/cos 关系）
//     · 安装误差
//   所以必须【实测标定】—— 我们用的是档案里那张实测表。
//
//   标定表比线性公式准，代价是要多做几个点。
// ============================================================

#include <Arduino.h>
#include "AckermannConfig.h"

class ServoSteering
{
public:
    // begin()：初始化舵机引脚，回到中位
    void begin();

    // setAngle()：设置前轮转角
    //   angleDeg : 前轮转角（度）
    //              > 0 = 左转，< 0 = 右转，0 = 直行
    //              超过 SERVO_ANGLE_MAX 会被限幅（保护舵机）
    void setAngle(float angleDeg);

    // center()：回中位（直行）
    void center();

    // getAngle()：返回当前设定的角度（度）
    float getAngle() const { return _angle; }

    // getPulseUs()：返回当前脉宽（微秒）—— 调试用
    int getPulseUs() const { return _pulseUs; }

private:
    float _angle   = 0.0f;     // 当前角度（度）
    int   _pulseUs = SERVO_US_CENTER;  // 当前脉宽（µs）

    // angleToPulse()：查标定表 + 线性插值
    int angleToPulse(float angleDeg);
};
