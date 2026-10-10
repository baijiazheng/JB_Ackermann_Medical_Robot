#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

class UartProtocol
{
public:

    static void begin();
    // begin()：初始化 UART

    static void update();
    // update()：持续解析 UART 数据

    static bool hasNewCommand();
    // hasNewCommand()：判断是否收到新指令

    static float getLinearX();
    // getLinearX()：获取前后速度

    static float getAngularZ();
    // getAngularZ()：获取旋转速度
};

#endif