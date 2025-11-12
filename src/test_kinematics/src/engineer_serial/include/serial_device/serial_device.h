// 首尾帧封装
// 接入ros系统
// 串口掉线时触发处理程序 TODO

#include "receive_decoder/frame_decoder.h"
// #include "send_encoder/frame_encoder.h"
#include "serial_com.h"
#include <iomanip>
#include <thread>
#include <atomic> 

// using std::placeholders::_1;

namespace hitcrt {

class SerialDevice {
  public:
    SerialDevice(const std::string& name, int baudRate);
    void stop() {
      is_running.store(false)  ;
    } // 新增：显式停止方法
    ~SerialDevice(){
      is_running.store(false)  ;

    }
    std::atomic<bool> is_running{true};
  public:
    std::shared_ptr<SerialCom> serial_base_;

    static const size_t RECEIVE_FRAME_LENGTH = FrameDecoder::FRAME_LENGTH;//86B
    // static const size_t SEND_FRAME_LENGTH = FrameEncoder::FRAME_LENGTH;

  public:
    // 这两个函数不涉及ros通信
    // void send(const VecU8&);//老版本的接收,我们不用这个
    void send(std::vector<unsigned char> &data,size_t length);//给struct留下来的接口
    VecU8 receive_a_frame();
};

} // namespace hitcrt