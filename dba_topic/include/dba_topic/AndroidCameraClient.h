#ifndef ANDROID_CAMERA_CLIENT_H
#define ANDROID_CAMERA_CLIENT_H

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>

class AndroidCameraClient {
private:
    int sock = -1;
    std::string ip;
    int port;
    bool is_connected = false;

    // 辅助函数：接收指定长度的数据
    bool recv_all(char* buf, int total_len);

    // 辅助函数：网络字节序转本机 int
    int bytes_to_int(char* bytes);

public:
    AndroidCameraClient(const std::string& targetIp = "127.0.0.1", int targetPort = 5000)
        : ip(targetIp), port(targetPort) {}

    ~AndroidCameraClient() {
        disconnect();
    }

    // 建立连接
    bool connectServer();
    cv::Mat getFrame(bool rotate = true);
    // 断开连接
    void disconnect();

    bool isConnected() const { return is_connected; }
};

#endif