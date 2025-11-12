/*
  @brief 从寒假就开始预谋的结构体项目,区赛回来之后终于有时间全部搞定了
    所有的结构体都是3 + x + 2的结构,然后电控的发送其实是十分冗余的,但是这个都是陈年往事了
*/
#pragma once
#include "common.h"
#include <assert.h>
#include <cstring>
#include <cstddef>
#include <vector>
#include "verify.h"
#include <sstream>
#include <iomanip>
#include <iostream>
#define SEND_HEX_DEBUG 0 //控制是否输出发出的16进制

namespace hitcrt{
namespace ENGINEERstruct{

#pragma pack(push, 1)
struct SendToSerial {     // 视觉发给电控
    uint8_t head[2];    // 2
    uint8_t data_lenth; // 1

    float DATA1[3];     // 12 dm_1 dm_2 dm_3

    uint8_t tail[2];    // 2
};  // 共17

struct ReceiveFromSerial {  // 电控发给视觉的
    uint8_t head[2];    // 2
    uint8_t data_lenth; // 1 //在这里是32,要提醒电控改这个,他这个不对的话视觉这边要改

    float DATA1[3];     // 12 dm_1 dm_2 dm_3

    uint8_t tail[2];    // 2


};  // 17
#pragma pack(pop)  // 开这个保证不会自动对齐,他的字节数才对
/*
  @brief 下面是两个decode函数
    配合以前的学长写的通用的函数,实现从串口流的解密
  @note 一个例子
    struct SendToSerial {     // 视觉发给电控
        uint8_t head[2];      // 2 //55 11
        uint8_t data_lenth;   // 1 //不包括他自己在这里应该是29 //PREFIX_LENGTH3

        uint8_t bocchi_FLAG;  // 要改的 //1 //byte_length 29
        float joints[7];      // 要改的 //28

        uint8_t tail[2];      // 2 //CRC的值 //SUFFIX_LENGTH 2
    };  // 共34

    对于这个结构体,输入是第二部分也就是下面两个对应的数组
        uint8_t bocchi_FLAG;  // 要改的 //1 //byte_length 29
        float joints[7];      // 要改的 //28
    填充到结构体里面,然后就可以拿出数据了
*/
// inline std::pair<u8,std::vector<float>> DECODEsimulator(const VecU8 &data){//模拟的电控那边的解密函数,输出bocchi_FLAG和joints
//   SendToSerial DECODEstruct;
//   assert(data.size()<sizeof(DECODEstruct)-3);
//   u8 *bytes = reinterpret_cast<u8 *>(&DECODEstruct);  // 结构体的指针,其实memcopy好像是一样的
//   for(size_t i=0;i<data.size();i++){
//     bytes[i+3] = data.at(i);
//   }
//   return std::pair(DECODEstruct.bocchi_FLAG,std::vector<float>(DECODEstruct.joints,DECODEstruct.joints + 7));
// }

inline std::vector<float> DECODEvisual(
    const VecU8& data) {  // 视觉使用的解密函数,输出flag_task,joints
    ReceiveFromSerial DECODEstruct;
    // std::cout<<"这个结构体的大小是: "<<sizeof(DECODEstruct)<<std::endl;
    assert(data.size() < sizeof(DECODEstruct) - 3);
    u8* bytes = reinterpret_cast<u8*>(&DECODEstruct);  // 结构体的指针
    for (size_t i = 0; i < data.size(); i++) {
        bytes[i + 3] = data.at(i);
    }
    // HWT_ANG = DECODEstruct.HWT_ANG;
    // HWT_ANG_V = DECODEstruct.HWT_ANG_V;
    // left_dis = DECODEstruct.left_dis;
    // behind_dis = DECODEstruct.behind_dis;
    std::vector<float> res_decoded = std::vector<float>(DECODEstruct.DATA1, DECODEstruct.DATA1 + 3);
    // std::cout<<"收到的数据是:1"<<res_decoded[0]<<std::endl;
    return res_decoded;
}

};  // namespace ENGINEERstruct
using namespace ENGINEERstruct;

class DataStruct{
  public:

    DataStruct(){//不初始化对象sizeof也是那么多字节，他似乎是在编译的时候就确定了sizeof()的值

    }
    void init(){
      send.head[0] = 0x55; send.head[1] = 0x11;
      send.data_lenth = sizeof(send)-3-2;//不包括头尾，不包括自己
      // send.visual_FLAG = 0x11;send.space = 0x00;
      // send.bocchi_FLAG = 0x00;

      simulator.head[0] = 0x55;simulator.head[1] = 0x11;
      simulator.data_lenth = sizeof(simulator)-3-2;//应该是32
      // simulator.flag_lenth = 0x01;simulator.float_lenth = 0x07;
      // simulator.HWT_ANG = 0.0;
      // simulator.HWT_ANG_V = 0.0;

    }
    // int debug_level = 1;//现在默认是1
    private:
    const uint16_t CRC16_INIT = 0xffff;
    uint8_t last_bocchi = 0x00;
    SendToSerial send;
    ReceiveFromSerial simulator;
  public:
      void update_SendToSerial_v2(const std::vector<float> &joints,std::vector<u8> &buff){//他会给出CRC
        // send.visual_FLAG = visual_FLAG;
        // send.space = space;
        // send.bocchi_FLAG = bocchi_FLAG;
        // send.see_entry_flag = see_entry_flag;
        // send.ik_flag = ik_flag;
        for(size_t i=0;i<7;i++){
          send.DATA1[i] = joints[i];
        }
        buff.resize(sizeof(send));
        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&send);  // 我们结构体的头指针
        for (size_t i = 0; i < sizeof(send); i++) {
            buff[i] = bytes[i];
            // if()
        }
        uint16_t wExpected = Get_CRC16_Check_Sum(buff.data(), buff.size()-2, CRC16_INIT);//输出的CRC校验码
        buff[buff.size()-2] = wExpected & 0xff;//逻辑低8位，放内存低8位
        buff[buff.size()-1] = wExpected >> 8;//逻辑高8位，放内存高8位
        // get_SendCharVec(buff);

    }
    
/*计算机以字节(8位二进制)存储数据,如果要存储uint16_t a = 0x0102 访问内存 高地址位是栈*/
// -exec p&a
// $1 = (uint16_t *) 0x7fffffff92e6
// -exec x/100xb 0x7fffffff92e6
// 0x7fffffff92e6:	0x02	0x01	0x02	0x00	0x00	0x00	0x00	0x00
// 0x7fffffff92ee:	0x00	0x00	0xe6	0x92	0xff	0xff	0xff	0x7f

};
}

//下面是电控代码中的定义
//-------------------------------USART4-----------------------------------//
/*电控发给视觉的*/
// typedef struct
// {                            
//     uint8_t head[2];    // 2
//     uint8_t data_lenth; // 1

//     float DATA1[3];     // 12 dm_1 dm_2 dm_3

//     uint8_t tail[2];    // 2
// } USART2_SEND;          // 17
/*视觉发给电控的*/
// typedef struct
// {
//     uint8_t head[2];        // 2

//     uint8_t usart4_rxlen;   // 1

//     float DM1;              // 4
//     float DM2;              // 4
//     float DM3;              // 4

//     uint8_t tail[2];        // 2
// } USART2_RECIEVE;           // 17