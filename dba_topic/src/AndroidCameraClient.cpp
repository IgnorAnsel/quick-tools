#include "dba_topic/AndroidCameraClient.h"

bool AndroidCameraClient::recv_all(char* buf, int total_len) {
    int total_received = 0;
    while (total_received < total_len) {
        int received = recv(sock, buf + total_received, total_len - total_received, 0);
        if (received <= 0) return false; // 连接断开或出错
        total_received += received;
    }
    return true;
}

int AndroidCameraClient::bytes_to_int(char *bytes)  {
    unsigned char b0 = (unsigned char)bytes[0];
    unsigned char b1 = (unsigned char)bytes[1];
    unsigned char b2 = (unsigned char)bytes[2];
    unsigned char b3 = (unsigned char)bytes[3];
    return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

bool AndroidCameraClient::connectServer()  {
    if (is_connected) disconnect();

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "[Error] Socket creation failed." << std::endl;
        return false;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr) <= 0) {
        std::cerr << "[Error] Invalid IP Address: " << ip << std::endl;
        return false;
    }

    std::cout << "Connecting to " << ip << ":" << port << "..." << std::endl;
    if (connect(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "[Error] Connection failed." << std::endl;
        if (ip == "127.0.0.1") {
            std::cerr << "Hint: Did you run 'adb forward tcp:5000 tcp:5000'?" << std::endl;
        }
        close(sock);
        sock = -1;
        return false;
    }

    std::cout << "Connected successfully!" << std::endl;
    is_connected = true;
    return true;
}

void AndroidCameraClient::disconnect()  {
    if (sock != -1) {
        close(sock);
        sock = -1;
    }
    is_connected = false;
}

// 获取一帧图像
// 如果失败或连接断开，返回空的 cv::Mat
cv::Mat AndroidCameraClient::getFrame(bool rotate) {
    if (!is_connected) return cv::Mat();

    // 读取长度
    char lenBuf[4];
    if (!recv_all(lenBuf, 4)) {
        std::cerr << "[Info] Server disconnected." << std::endl;
        disconnect();
        return cv::Mat();
    }

    int imgSize = bytes_to_int(lenBuf);

    // 校验长度 (防止脏数据导致内存爆炸)
    if (imgSize <= 0 || imgSize > 20 * 1024 * 1024) {
        std::cerr << "[Warning] Invalid image size received: " << imgSize << std::endl;
        return cv::Mat();
    }

    // 读取图片数据
    std::vector<char> imgData(imgSize);
    if (!recv_all(imgData.data(), imgSize)) {
        disconnect();
        return cv::Mat();
    }

    // 解码
    try {
        cv::Mat frame = cv::imdecode(imgData, cv::IMREAD_COLOR);
        if (frame.empty()) return frame;

        // 旋转处理 (通常 Android 相机传输过来的画面需要旋转 90 度)
        if (rotate) {
            cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);
        }
        return frame;
    } catch (...) {
        std::cerr << "[Error] Image decode failed." << std::endl;
        return cv::Mat();
    }
}
