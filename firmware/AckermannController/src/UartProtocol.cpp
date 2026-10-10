#include "UartProtocol.h"
// UartProtocol.h：声明本模块对外提供的接口

#include "Config.h"

#include <Arduino.h>

namespace
{
    String rxBuffer = "";
    // rxBuffer：UART 接收缓冲区

    float linearX = 0.0f;
    // linearX：前后速度

    float angularZ = 0.0f;
    // angularZ：机器人绕 Z 轴旋转速度
    // 单位：rad/s

    bool newCommand = false;
    // newCommand：是否收到新的控制指令
}

void UartProtocol::begin()
{
    UART_SERIAL.begin(UART_BAUDRATE);;
    // Serial3：Mega2560 硬件 UART3
    // begin()：设置通信波特率
}

void UartProtocol::update()
{
    while (UART_SERIAL.available())
    {
        // available()：检查是否有新的 UART 数据

        char c = UART_SERIAL.read();
        // read()：读取一个字符

        if (c == '\n')
        {
            // '\n'：一帧数据结束

            if (rxBuffer.startsWith("V,"))
            {
                // startsWith()：判断字符串是否以指定内容开头

                int firstComma = rxBuffer.indexOf(',');
                // indexOf()：寻找指定字符的位置

                int secondComma = rxBuffer.indexOf(',', firstComma + 1);
                // 寻找第二个逗号

                if (firstComma > 0 && secondComma > firstComma)
                {
                    linearX = rxBuffer.substring(
                        firstComma + 1,
                        secondComma
                    ).toFloat();
                    // substring()：截取字符串
                    // toFloat()：把字符串转换成浮点数

                    angularZ = rxBuffer.substring(
                        secondComma + 1
                    ).toFloat();
                    // substring()：截取第二个逗号之后的数据
                    // toFloat()：把字符串转换成浮点数

                    newCommand = true;
                    // 标记：收到一条新的控制指令
                }
            }

            rxBuffer = "";
            // 清空缓冲区
        }
        else
        {
            rxBuffer += c;
            // +=：把字符追加到接收缓冲区
        }
    }
}

bool UartProtocol::hasNewCommand()
{
    return newCommand;
    // return：返回是否收到新指令
}

float UartProtocol::getLinearX()
{
    newCommand = false;
    // 读取控制量后清除新数据标志

    return linearX;
}

float UartProtocol::getAngularZ()
{
    return angularZ;
}
// getAngularZ()：获取旋转速度