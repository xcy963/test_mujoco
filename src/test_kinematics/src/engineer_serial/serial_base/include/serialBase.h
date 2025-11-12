#ifndef RM_HERO_SERIALBASE_H
#define RM_HERO_SERIALBASE_H

#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <iostream>

namespace hitcrt {
class SerialBase {
  public:
    // 输入：端口号，波特率
    SerialBase(std::string str, int baud_rate);
    ~SerialBase();
    void send(unsigned char* ch, size_t length);
    void receive(unsigned char* buff, size_t& length);

  private:
    int serial_fd; // 串口文件描述符
    static const int MAX_BUFFER_LENGTH = 40;
    
    bool setupSerial(int baud_rate);
    int getBaudRateConstant(int baud_rate);
};




} // namespace hitcrt
#endif // RM_HERO_SERIALBASE_H