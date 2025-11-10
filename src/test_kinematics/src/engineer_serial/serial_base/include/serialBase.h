//
// Created by bg2edg on 2020/11/10.
//

#ifndef RM_HERO_SERIALBASE_H
#define RM_HERO_SERIALBASE_H

#include <boost/asio.hpp>
namespace hitcrt {
class SerialBase {
  public:
    // 输入：端口号，波特率
    SerialBase(std::string str, int baud_rate);
    ~SerialBase();
    void send(unsigned char* ch, size_t length);
    void receive(unsigned char* buff, size_t& length);

  private:
    boost::asio::io_service* io;
    boost::asio::serial_port* port;
    static const int MAX_BUFFER_LENGTH = 40;//每次读取的最大字节数,最好是电控发送结构体的数据的两倍-1,这样一定能保证接收
    //但是给2倍会不会丢帧呢
};
} // namespace hitcrt
#endif // RM_HERO_SERIALBASE_H
