#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <errno.h>

#include "Com.h"

#define iBufferSize 1000
#define UARTBufferLength 1000

static int serial_fd = -1;
static pthread_t hCOMThread = 0;
static volatile int thread_running = 0;

static volatile char chrUARTBuffers[UARTBufferLength] = {0};
static volatile unsigned long ulUARTBufferStart = 0, ulUARTBufferEnd = 0;

// 设置串口参数
static int set_serial_attributes(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    // 忽略调制解调器控制线，启用接收器
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8位数据位
    tty.c_cflag &= ~PARENB;             // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;             // 1位停止位
    tty.c_cflag &= ~CRTSCTS;            // 无硬件流控

    // 本地模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 输入模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    // 输出模式
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // 超时设置
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1秒超时

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

// 将Windows波特率常量转换为Linux波特率
static speed_t get_baud_rate(unsigned long ulBaundrate)
{
    switch (ulBaundrate) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B9600;
    }
}

signed char SendUARTMessageLength(const char chrSendBuffer[], const unsigned short usLen)
{
    if (serial_fd == -1) {
        return -1;
    }

    ssize_t bytes_written = write(serial_fd, chrSendBuffer, usLen);
    if (bytes_written != usLen) {
        perror("write");
        return -1;
    }

    // Linux下串口通常不需要DTR控制，所以移除了相关代码
    usleep(10000); // 10ms延迟，替代原来的Sleep(10)

    return 0;
}

// 串口接收线程函数
void* ReceiveCOMData(void* pParam)
{
    char chrBuffer[iBufferSize] = {0};

    while (thread_running) {
        fd_set readfds;
        struct timeval timeout;

        FD_ZERO(&readfds);
        FD_SET(serial_fd, &readfds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int result = select(serial_fd + 1, &readfds, NULL, NULL, &timeout);

        if (result > 0 && FD_ISSET(serial_fd, &readfds)) {
            ssize_t uLen = read(serial_fd, chrBuffer, iBufferSize - 1);

            if (uLen > 0) {
                extern void ComRxCallBack(char *p_data, unsigned int uiSize);
                ComRxCallBack(chrBuffer, (unsigned int)uLen);
            } else if (uLen < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("read");
                    break;
                }
            }
        } else if (result < 0) {
            perror("select");
            break;
        }
    }

    return NULL;
}

signed char OpenCOMDevice(const unsigned long ulPortNo, const unsigned long ulBaundrate)
{
    char portName[20];

    // 尝试不同的串口设备命名约定
    snprintf(portName, sizeof(portName), "/dev/ttyCH341USB%lu", ulPortNo); // 标准串口
    serial_fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd == -1) {
        snprintf(portName, sizeof(portName), "/dev/ttyUSB%lu", ulPortNo); // USB转串口
        serial_fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    }

    if (serial_fd == -1) {
        snprintf(portName, sizeof(portName), "/dev/ttyACM%lu", ulPortNo); // ACM设备
        serial_fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    }

    if (serial_fd == -1) {
        fprintf(stderr, "Cannot open serial port %s: %s\n", portName, strerror(errno));
        return -1;
    }

    printf("Opened serial port: %s\n", portName);

    // 恢复阻塞模式
    int flags = fcntl(serial_fd, F_GETFL, 0);
    fcntl(serial_fd, F_SETFL, flags & ~O_NDELAY);

    // 设置串口参数
    if (set_serial_attributes(serial_fd, get_baud_rate(ulBaundrate)) != 0) {
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }

    // 清空缓冲区
    tcflush(serial_fd, TCIOFLUSH);

    // 创建接收线程
    thread_running = 1;
    if (pthread_create(&hCOMThread, NULL, ReceiveCOMData, NULL) != 0) {
        perror("pthread_create");
        thread_running = 0;
        close(serial_fd);
        serial_fd = -1;
        return -1;
    }

    usleep(200000); // 200ms延迟，替代原来的Sleep(200)

    return 0;
}

signed char SetBaundrate(const unsigned long ulBaundrate)
{
    if (serial_fd == -1) {
        return -1;
    }

    return set_serial_attributes(serial_fd, get_baud_rate(ulBaundrate));
}

void CloseCOMDevice()
{
    thread_running = 0;
    ulUARTBufferEnd = 0;
    ulUARTBufferStart = 0;

    if (hCOMThread) {
        pthread_join(hCOMThread, NULL);
        hCOMThread = 0;
    }

    if (serial_fd != -1) {
        close(serial_fd);
        serial_fd = -1;
    }
}

// 添加一个函数来获取串口状态，便于调试
int GetCOMDeviceStatus()
{
    if (serial_fd == -1) {
        return -1;
    }

    int status;
    if (ioctl(serial_fd, TIOCMGET, &status) == -1) {
        perror("ioctl(TIOCMGET)");
        return -1;
    }

    return status;
}