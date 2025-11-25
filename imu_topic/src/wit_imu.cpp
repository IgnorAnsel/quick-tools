#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "wit_c_sdk.h"
#include "Com.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>

static char s_cDataUpdate = 0;
int iComPort = 4;
int iBaud = 9600;
int iAddress = 0x50;

// 实现ComRxCallBack函数
void ComRxCallBack(char *p_data, unsigned int uiSize)
{
    for(unsigned int i = 0; i < uiSize; i++)
    {
        WitSerialDataIn(p_data[i]);
    }
}

static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void DelayMs(uint16_t ms);

// 欧拉角转四元数
void eulerToQuaternion(double roll, double pitch, double yaw,
                      double* qx, double* qy, double* qz, double* qw)
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    *qw = cy * cp * cr + sy * sp * sr;
    *qx = cy * cp * sr - sy * sp * cr;
    *qy = sy * cp * sr + cy * sp * cr;
    *qz = sy * cp * cr - cy * sp * sr;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    float a[3], w[3], Angle[3], h[3];

    // 创建节点和发布者
    auto node = std::make_shared<rclcpp::Node>("wit_imu");
    auto imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // 设置发布频率
    rclcpp::WallRate loop_rate(100); // 100Hz

    printf("Please enter the serial number: ");
    scanf("%d", &iComPort);
    printf("Trying to open COM%d with baudrate %d\n", iComPort, iBaud);

    if (OpenCOMDevice(iComPort, iBaud) != 0) {
        printf("Failed to open serial port!\n");
        return -1;
    }

    printf("Initializing WIT sensor...\n");
    WitInit(WIT_PROTOCOL_MODBUS, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(CopeSensorData);
    WitDelayMsRegister(DelayMs);

    printf("Starting auto scan...\n");
    AutoScanSensor();
    printf("Auto scan completed.\n");

    int loop_count = 0;
    auto last_publish_time = node->now();

    while (rclcpp::ok()) {
        loop_count++;

        WitReadReg(AX, 15);
        DelayMs(10);

        if (s_cDataUpdate) {
            for (int i = 0; i < 3; i++) {
                a[i] = (float)sReg[AX+i] / 32768.0f * 16.0f;
                w[i] = (float)sReg[GX+i] / 32768.0f * 2000.0f;
                Angle[i] = (float)sReg[Roll+i] / 32768.0f * 180.0f;
                h[i] = (float)sReg[HX+i];
            }

            // 创建IMU消息
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

            // 设置时间戳
            imu_msg->header.stamp = node->now();
            imu_msg->header.frame_id = "imu_link";

            // 设置角速度 (rad/s)
            // 注意：WIT传感器输出的角速度单位是度/秒，需要转换为弧度/秒
            imu_msg->angular_velocity.x = w[0] * M_PI / 180.0;
            imu_msg->angular_velocity.y = w[1] * M_PI / 180.0;
            imu_msg->angular_velocity.z = w[2] * M_PI / 180.0;

            // 设置线加速度 (m/s²)
            // 注意：WIT传感器输出的加速度单位是g，需要转换为m/s²
            imu_msg->linear_acceleration.x = a[0] * 9.80665;
            imu_msg->linear_acceleration.y = a[1] * 9.80665;
            imu_msg->linear_acceleration.z = a[2] * 9.80665;

            // 将欧拉角转换为四元数
            double qx, qy, qz, qw;
            // 注意：WIT传感器的角度单位是度，需要转换为弧度
            double roll_rad = Angle[0] * M_PI / 180.0;
            double pitch_rad = Angle[1] * M_PI / 180.0;
            double yaw_rad = Angle[2] * M_PI / 180.0;

            eulerToQuaternion(roll_rad, pitch_rad, yaw_rad, &qx, &qy, &qz, &qw);

            imu_msg->orientation.x = qx;
            imu_msg->orientation.y = qy;
            imu_msg->orientation.z = qz;
            imu_msg->orientation.w = qw;

            // 发布IMU消息
            imu_publisher->publish(std::move(imu_msg));

            // 可选：在终端显示数据（调试用）
            if (loop_count % 10 == 0) { // 每10次循环显示一次，避免输出过于频繁
                printf("\033[2J\033[H"); // 清屏并移动光标到左上角
                printf("WIT IMU Publisher (Loop: %d)\n", loop_count);
                printf("==============================\n");
                printf("Accel: %7.3f %7.3f %7.3f m/s²\n",
                       a[0]*9.80665, a[1]*9.80665, a[2]*9.80665);
                printf("Gyro:  %7.3f %7.3f %7.3f rad/s\n",
                       w[0]*M_PI/180.0, w[1]*M_PI/180.0, w[2]*M_PI/180.0);
                printf("Angle: %6.1f %6.1f %6.1f °\n",
                       Angle[0], Angle[1], Angle[2]);
                printf("Publishing to: /imu/data\n");
            }

            s_cDataUpdate = 0;
        }

        // 处理ROS回调
        rclcpp::spin_some(node);
        loop_rate.sleep();

        // 定期检查串口状态
        if (loop_count % 100 == 0) {
            int status = GetCOMDeviceStatus();
            if (status == -1) {
                printf("Serial port disconnected!\n");
                break;
            }
        }
    }

    CloseCOMDevice();
    rclcpp::shutdown();
    return 0;
}

static void DelayMs(uint16_t ms)
{
    usleep(ms * 1000);
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    SendUARTMessageLength((const char*)p_data, uiSize);
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
    s_cDataUpdate = 1;
}

static void AutoScanSensor(void)
{
    const uint32_t c_uiBaud[7] = {4800, 9600, 19200, 38400, 57600, 115200, 230400};
    int i, iRetry;

    printf("Starting baudrate auto scan...\n");

    for(i = 0; i < 7; i++) {
        printf("Trying baudrate: %d\n", c_uiBaud[i]);
        SetBaundrate(c_uiBaud[i]);
        iRetry = 2;

        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            DelayMs(100);

            if(s_cDataUpdate != 0) {
                printf("%d baud find sensor\n\n", c_uiBaud[i]);
                iBaud = c_uiBaud[i];
                return;
            }
            iRetry--;
        } while(iRetry);
    }

    printf("Can not find sensor\n");
    printf("Please check your connection\n");
}