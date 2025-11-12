// 首尾帧封装
// 接入ros系统
// 串口掉线时触发处理程序 TODO


#include "serial_device.h"

namespace hitcrt {
SerialDevice::SerialDevice(const std::string& name, int baudRate) {
    // std::cout<<""
    serial_base_ = std::make_shared<SerialCom>();
    if (serial_base_->init(name, baudRate) !=
        SerialCom::INIT_STATE::SUCCEEDED) {
        throw "serial base init fail";
        // std::cout << "serial base init fail" << std::endl;
        return;
    }

}

// 接受**一帧**，分发话题 循环
/*
    @author hitcrt 前人种树的结果,这个函数理论上是万金油
    @brief 只要满足电控的发送结构体大小小于80就可以接收
    输入的第三个元素代表的长度不包括他自己!!!
    理由frame_length = PREFIX_LENGTH + byte_length + SUFFIX_LENGTH;//这里证明byte_length是不包括他自己的
    PREFIX_LENGTH是3 SUFFIX_LENGTH是2
    struct SendToSerial {     // 视觉发给电控
        uint8_t head[2];      // 2 //55 11
        uint8_t data_lenth;   // 1 //不包括他自己在这里应该是29 //PREFIX_LENGTH 3

        uint8_t bocchi_FLAG;  // 要改的 //1 //byte_length 29
        float joints[7];      // 要改的 //28

        uint8_t tail[2];      // 2 //CRC的值 //SUFFIX_LENGTH 2
    };  // 共34

    @return 只有中间的数据段比如上面的那个就是        
        uint8_t bocchi_FLAG;  // 要改的 //1 //byte_length 29
        float joints[7];      // 要改的 //28
    对应的VecU8
*/
auto SerialDevice::receive_a_frame() -> VecU8 {
    // std::cout<<"我在串口函数里面开始接受一帧";
    static FrameDecoder decoder;
    static size_t rcv_length;
    static u8 buff[RECEIVE_FRAME_LENGTH] = {};//开这么多

    VecU8 raw_data;
    while (is_running.load()) {//需要有退出的机制,所以是isrunning
        serial_base_->receive(buff, rcv_length);//TODO最多接收两倍于接收结构体的值,这样会丢帧更厉害马?
        // std::cout<<"收到的长度"<<rcv_length<<std::endl;
        //具体需要收多少是不一定的,电控发得慢我们就可以少收一点,发得快就多受,基本收2倍以上就没意义了
        VecU8 raw_msg(buff, buff + rcv_length);//转化为u8数组,表示他收到的数据,一般能

        // std::stringstream hex_ss;
        // hex_ss << "Hex: ";
        // for (size_t i = 0; i < raw_msg.size(); ++i) { 
        //     hex_ss << std::hex << std::setw(2) << std::setfill('0') 
        //         << static_cast<int>(raw_msg[i]) << " ";
        // }
        // std::cout<<"收到的长度"<<hex_ss.str()<<std::endl;

        if (decoder.append_decode(raw_msg, raw_data)) {
            // std::cout << fmt::format("data: len:{}\n{:02x}", raw_data.size(),
            //                          fmt::join(raw_data, ", "))
            //           << std::endl;
            // std::cout<<"crc通过"<<std::endl;
            return raw_data;
        }else{
            // std::cout<<"crc不通过"<<std::endl;

        }
    }
    return {};
}

// void SerialDevice::send(const VecU8& data) {
//     auto encoded_data = FrameEncoder(data);
//     // std::cout << fmt::format("len0: {} len: {}", data.size(),
//     //                          encoded_data.getlen())
//     //           << std::endl;
//     serial_base_->send(encoded_data.getbuff(), encoded_data.getlen());
// }

void SerialDevice::send(std::vector<unsigned char> &data,size_t length) {//其实只是区分开而以，有点多余

    // std::cout << fmt::format("len0: {} len: {}", data.size(),
    //                          encoded_data.getlen())
    //           << std::endl;
    serial_base_->send(data.data(), length);
}

} // namespace hitcrt
