#include "include/serialBase.h"

namespace hitcrt {

SerialBase::SerialBase(std::string str, int baud_rate) : serial_fd(-1) {
    // 打开串口设备
    serial_fd = open(str.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "Error opening serial port " << str << ": " << strerror(errno) << std::endl;
        return;
    }
    
    // 配置串口参数
    if (!setupSerial(baud_rate)) {
        std::cerr << "Error setting up serial port" << std::endl;
        close(serial_fd);
        serial_fd = -1;
    }
}

SerialBase::~SerialBase() {
    if (serial_fd >= 0) {
        close(serial_fd);
    }
}

bool SerialBase::setupSerial(int baud_rate) {
    if (serial_fd < 0) return false;
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    // 获取当前配置
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting termios attributes: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 设置波特率
    speed_t speed = getBaudRateConstant(baud_rate);
    if (speed == B0) {
        std::cerr << "Unsupported baud rate: " << baud_rate << std::endl;
        return false;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 设置数据位：8位
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    
    // 禁用奇偶校验
    tty.c_cflag &= ~PARENB;
    
    // 1位停止位
    tty.c_cflag &= ~CSTOPB;
    
    // 禁用硬件流控
    tty.c_cflag &= ~CRTSCTS;
    
    // 启用接收器
    tty.c_cflag |= (CLOCAL | CREAD);
    
    // 设置原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // 禁用软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 原始输出模式
    tty.c_oflag &= ~OPOST;
    
    // 读取超时和最小字符数设置
    tty.c_cc[VMIN] = 0;  // 最小读取字符数
    tty.c_cc[VTIME] = 5; // 超时时间（单位：0.1秒）
    
    // 应用配置
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 清空输入输出缓冲区
    tcflush(serial_fd, TCIOFLUSH);
    
    return true;
}

int SerialBase::getBaudRateConstant(int baud_rate) {
    switch (baud_rate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return B0;
    }
}

void SerialBase::send(unsigned char* ch, size_t length) {
    if (serial_fd < 0) {
        std::cerr << "Serial port not initialized" << std::endl;
        return;
    }
    
    ssize_t written = write(serial_fd, ch, length);
    if (written < 0) {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    } else if (static_cast<size_t>(written) != length) {
        std::cerr << "Incomplete write: " << written << " of " << length << " bytes" << std::endl;
    }
    
    // 确保数据被发送
    tcdrain(serial_fd);
}

void SerialBase::receive(unsigned char* buff, size_t& length) {
    if (serial_fd < 0) {
        std::cerr << "Serial port not initialized" << std::endl;
        length = 0;
        return;
    }
    // std::cout<<"开始读取"<<std::endl;
    ssize_t bytes_read = read(serial_fd, buff, MAX_BUFFER_LENGTH);
    // std::cout<<"开始读取到的字节长度"<<bytes_read<<std::endl;

    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
        }
        length = 0;
    } else {
        length = bytes_read;
    }
}

} // namespace hitcrt