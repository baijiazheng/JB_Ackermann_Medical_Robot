#include "MotorDriverI2C.h"
// MotorDriverI2C.h：电机驱动模块接口

#include <Wire.h>
// Wire.h：Arduino I²C 通信库


namespace
{
    constexpr uint8_t I2C_ADDR = 0x34;
    // I2C_ADDR：驱动板 I²C 从机地址

    constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;
    // MOTOR_TYPE_ADDR：电机类型寄存器

    constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
    // MOTOR_ENCODER_POLARITY_ADDR：编码器方向极性寄存器

    constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;
    // MOTOR_FIXED_SPEED_ADDR：四路闭环固定速度寄存器

    constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 0x3C;
    // MOTOR_ENCODER_TOTAL_ADDR：四路编码器累计脉冲寄存器

    constexpr int8_t M1_DIR = 1;
    constexpr int8_t M2_DIR = -1;
    constexpr int8_t M3_DIR = -1;
    constexpr int8_t M4_DIR = 1;
}


// ============================================================
// 初始化
// ============================================================

void MotorDriverI2C::begin()
{
    Wire.begin();
    // Wire.begin()：初始化 Mega2560 I²C 主机

    uint8_t motorType = 3;
    // 3：JGB37_520-12V-110RPM，90:1 编码电机

    Wire.beginTransmission(I2C_ADDR);
    // beginTransmission()：开始向驱动板发送数据

    Wire.write(MOTOR_TYPE_ADDR);
    // write()：发送目标寄存器地址

    Wire.write(motorType);
    // write()：发送电机类型

    Wire.endTransmission();
    // endTransmission()：结束 I²C 写操作


    uint8_t encoderPolarity = 0;
    // 0：默认编码器方向极性

    Wire.beginTransmission(I2C_ADDR);

    Wire.write(MOTOR_ENCODER_POLARITY_ADDR);

    Wire.write(encoderPolarity);

    Wire.endTransmission();
}


// ============================================================
// 设置四路电机速度
// ============================================================

void MotorDriverI2C::setSpeed(
    int8_t motor1,
    int8_t motor2,
    int8_t motor3,
    int8_t motor4
)
{
    int8_t speed[4] =
    {
        motor1,
        motor2,
        motor3,
        motor4
    };
    // speed[4]：保存四路电机目标速度


    Wire.beginTransmission(I2C_ADDR);
    // 开始 I²C 通信

    Wire.write(MOTOR_FIXED_SPEED_ADDR);
    // 指定 0x33：固定速度寄存器

    Wire.write(
        reinterpret_cast<uint8_t *>(speed),
        4
    );
    // reinterpret_cast()：把 int8_t 数组转换成字节数组
    // 发送 4 个速度值


    Wire.endTransmission();
    // 结束 I²C 通信
}


// ============================================================
// 停止
// ============================================================

void MotorDriverI2C::stop()
{
    setSpeed(
        0,
        0,
        0,
        0
    );
    // setSpeed()：四个电机目标速度全部设为 0
}


// ============================================================
// 读取编码器
// ============================================================

bool MotorDriverI2C::readEncoder(
    int32_t encoder[4]
)
{
    Wire.beginTransmission(I2C_ADDR);
    // 开始 I²C 通信

    Wire.write(MOTOR_ENCODER_TOTAL_ADDR);
    // 指定 0x3C：编码器累计脉冲寄存器

    if (Wire.endTransmission(false) != 0)
    {
        // false：保持 I²C 总线连接，准备读取

        return false;
        // false：通信失败
    }


    uint8_t received =
        Wire.requestFrom(
            I2C_ADDR,
            (uint8_t)16
        );
    // requestFrom()：请求读取 16 字节
    // 4 个 int32_t = 16 字节


    if (received != 16)
    {
        return false;
        // 数据长度不正确
    }


    for (int motor = 0; motor < 4; motor++)
    {
        // motor：当前读取的电机编号

        encoder[motor] =
            (int32_t)Wire.read() |
            ((int32_t)Wire.read() << 8) |
            ((int32_t)Wire.read() << 16) |
            ((int32_t)Wire.read() << 24);
        // read()：读取一个字节
        // <<：把字节移动到对应的位置
        // |：组合成完整的 int32_t
    }


    return true;
    // true：编码器读取成功
}

void MotorDriverI2C::setChassisSpeed(int8_t left, int8_t right)
{
    MotorDriverI2C::setSpeed(
        left  * M1_DIR,
        left  * M2_DIR,
        right * M3_DIR,
        right * M4_DIR
    );
}