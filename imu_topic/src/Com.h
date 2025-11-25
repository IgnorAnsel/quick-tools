// 在Com.h文件中添加这个声明
#ifndef COM_H
#define COM_H

#ifdef __cplusplus
extern "C" {
#endif

    // 函数声明
    signed char SendUARTMessageLength(const char chrSendBuffer[], const unsigned short usLen);
    signed char OpenCOMDevice(const unsigned long ulPortNo, const unsigned long ulBaundrate);
    signed char SetBaundrate(const unsigned long ulBaundrate);
    void CloseCOMDevice();
    int GetCOMDeviceStatus();

#ifdef __cplusplus
}
#endif

#endif // COM_H