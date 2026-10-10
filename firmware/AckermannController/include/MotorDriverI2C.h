#pragma once
// #pragma once：防止头文件被重复包含

#include <Arduino.h>
// Arduino.h：提供 uint8_t、int8_t、int32_t 等基础类型


class MotorDriverI2C
{
public:

    void begin();
    // begin()：初始化 I²C，并配置电机驱动板

    void setSpeed(
        int8_t motor1,
        int8_t motor2,
        int8_t motor3,
        int8_t motor4
    );
    // setSpeed()：设置四路电机目标速度
    // 每个电机范围：约 -50 ~ +50

    void stop();
    // stop()：停止所有电机

    bool readEncoder(
        int32_t encoder[4]
    );
    // readEncoder()：读取四路编码器累计脉冲
    // encoder[0]：M1
    // encoder[1]：M2
    // encoder[2]：M3
    // encoder[3]：M4

    void setChassisSpeed(int8_t left, int8_t right);
};