/*
Usage:
一个长期存在的 FrameDecoder 实例，将新的 buff 传入 append_decode ，若返回 true
，从第二个参数获取解帧结果.

55 11 len(1) | ... | CRC(2)
*/

#pragma once

#include "verify.h"

#include <algorithm>
#include <cstddef>
#include <iostream>

#include <fmt/core.h>
#include <fmt/ranges.h>

class FrameDecoder {
  public:
    static const std::size_t MAX_BYTE_LENGTH = 20 * 4;//这个会和电控发送的数据长度进行比较,电控需要小于这个

    static const std::size_t PREFIX_LENGTH = 3, SUFFIX_LENGTH = 2;//PREFIX_LENGTH是55 11 数据长度
    static const std::size_t FRAME_LENGTH =
        MAX_BYTE_LENGTH + PREFIX_LENGTH + SUFFIX_LENGTH;

    static const u8 FIRST_ONE = 0x55;
    static const u8 FIRST_TWO = 0x11;

  private:
  public:
    bool append_decode(const VecU8&, VecU8&);
};

// 只提取数据帧，不处理帧内数据
inline bool FrameDecoder::append_decode(const VecU8& buff, VecU8& ret) {
    static unsigned int state = 0;
    static std::size_t byte_length, frame_length, rest_length, buff_rest_len;
    static u8 temp_save_frame[FRAME_LENGTH], *temp_save_p;//初始化一段内存空间以及一个指针，后面会把这个指针指向这个头
    bool return_flag = false;

    for (auto it = buff.begin(); it != buff.end(); ++it) {
        switch (state) {
        case 0:
            // std::cout << 0 << std::endl;
            if (*it == FIRST_ONE) {
                state++;
                temp_save_p = temp_save_frame;//每一次都会从内存空间起始位置开始操作,表示我们寻找的字节数组第一个是FIRST_ONE
                *temp_save_p++ = FIRST_ONE;
            }
            break;
        case 1:
            // std::cout << 1 << std::endl;
            if (*it == FIRST_TWO) {
                state++;
                *temp_save_p++ = FIRST_TWO;
            } else {
                if (*it == FIRST_ONE)
                    state = 1;//如果是第一个针头，那么从头开始，类似于学的那个字符串匹配
                else
                    state = 0;
            }
            break;
        case 2:
            // std::cout << 2 << std::endl;
            byte_length = *it;//电控发的数据长度
            if (byte_length <= MAX_BYTE_LENGTH) {//小于就认为是对的,然后去验证CRC,可以适配很多的代码
                rest_length = byte_length;
                frame_length = PREFIX_LENGTH + byte_length + SUFFIX_LENGTH;//这里证明byte_length是不包括他自己的
                *temp_save_p++ = *it;
                state++;
            } else {
                state = 0;
            }
            break;
        case 3:
            // std::cout << 3 << std::endl;
            buff_rest_len = std::distance(it, buff.end());//当前buff数组里面剩下的数据够用,
            //distance返回的数包括it 和end 比如it指向下标为1的数(前向后数第2个) buff总长为10,那么返回9
            if (buff_rest_len >= rest_length) {//buff里面的数据是够用的
                // std::cout << "all data involved" << std::endl;
                // 本帧包含了整个数据段
                temp_save_p = std::copy_n(it, rest_length, temp_save_p);//从it开始的,读取完长度之后it++了,所以it复制的数组不包括byte_length(长度)
                std::advance(it, rest_length - 1); // it向后移动(rest_length - 1),相当于删除这么多字节
                rest_length = 0;

                // fmt::print("it now is: {}", *it);
                state++;
            } else {//不够要去借,所以需要再去读取
                // std::cout << "not all data involved" << std::endl;
                // 本帧未包含全部数据
                temp_save_p = std::copy_n(it, buff_rest_len, temp_save_p);
                std::advance(it, buff_rest_len - 1); // 移动到帧内最后一个字节,注意这个break了之后it还要再++才指向帧尾两个
                rest_length -= buff_rest_len;
                //注意这里没有更新state,所以下一次再读取之后会跳转到这里
                // fmt::print("it now is: {}", *it);
            }
            break;
        case 4://最后两个尾巴
            // std::cout << 4 << std::endl;
            *temp_save_p++ = *it;
            state++;
            break;
        case 5:
            // std::cout << 5 << std::endl;
            *temp_save_p++ = *it;

            // debug
            // fmt::print("temp buff: {:02x}", fmt::join(temp_save_frame, ","));
            
            // check crc
            // std::cout << fmt::format("要进crc了: {:02x}", fmt::join(temp_save_frame, ", "))
            //       << std::endl;
            if (Verify_CRC16_Check_Sum(temp_save_frame, frame_length)) {//数组和总长度传入,注意他传入的是有55 11 和两个尾巴的
                // std::cout << "CRCSucceeded" << std::endl;
                ret = VecU8(temp_save_frame + PREFIX_LENGTH,
                            temp_save_frame + PREFIX_LENGTH + byte_length);//只有中间有用的,他没有返回55 11
                return_flag = true;
            }else{
                // std::cout<<"crc失败\n";
            }
            state = 0;
        }
    }

    return return_flag;
}
