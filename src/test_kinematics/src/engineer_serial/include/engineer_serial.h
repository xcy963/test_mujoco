#pragma once

#define PLOTALLSERIAL false 
#define JOINT6ERROE 3  //这个是为了应对齿轮带来的误差
#include "coding_struct/data_struct.h"
#include "serial_device/serial_device.h"
// #include "matplotlibcpp.h"
#include <iostream>
#include<fstream>
#include <filesystem> 

namespace hitcrt{
    namespace TASKS{
        enum class RECEIVE_TASKS : u8 { JointValues = 0x01, DONOTSTART = 0x00 };
        enum class IN_SENDING : u8 {ONLINE = 0x00,BOX_not_detected = 0x01 ,BOX1_detected = 0x02,BOX2_detected = 0x03,SENDING = 0x04,LASTONE = 0x05
                ,CAMERAOFFLINE = 0x06};

        enum class VISUAL : u8 {CAMERAOFFLINE = 0x00,  CAMERAONLINE = 0x01 };
        enum class space :  u8 {BOXNOTDETECTED = 0x00, BOX1DETECTED = 0x01,BOX2DETECTED = 0x02  };
        enum class bocchi_FLAG : u8 {SENDING = 0x00,  LASTONE = 0x01 };
        enum class SEE_ENTRY_FLAG : u8 {NO = 0x00,  YES = 0x01 };
        enum class IK_FLAG : u8 {FAIL = 0x00,  SUCCEED = 0x01 };


        
        //视觉发给电控的时候使用,说明:Nothing表示我是在线的(到5.1这个还没用起来) FirstValue表示这个值是需要插值过去的,
    };
    
    class engineer_serial{
    private:
        // SerialDevice serial_device_;
        std::shared_ptr<SerialDevice> serial_device_;
        std::shared_ptr<DataStruct> data_struct_;

        std::mutex receive_mutex_;     //控制读取角度值的
        std::vector<float> joints_now = std::vector<float>(7,0.0) ;//串口过来的角度值，存放在主类里面,为了保证读取的帧率,使用float
        u8 flag  = 0x00;    //串口发送过来的字节数组


        int flush_counter = 0;
        int flush_counter_DIS = 0;
        const int FLUSH_INTERVAL = 50;  // 每50次写入刷新一次
        std::chrono::time_point<std::chrono::steady_clock> start_timepoint = std::chrono::steady_clock::now();
    public:
        void read_once(u8&flag_temp,std::vector<float> &joints);

        engineer_serial(const std::string& name, int baudRate){
            // auto time_now = std::chrono::system_clock::now();
            // std::chrono::duration<double> elapsed = time_now.time_since_epoch();

            SERIALrunning_flag = true;

            serial_device_ = std::make_shared<SerialDevice>(name,baudRate);

            if(!serial_device_){
                throw "串口创建失败,抛出异常";//这个类一定要说自己创建失败了,其他的别的地方写
            }
            data_struct_ = std::make_shared<DataStruct>();
            data_struct_->init();
            std::cout<<"串口初始化成功顺便init了两个结构体"<<std::endl;
            joints_now.resize(7);
            reading_thread_ = std::thread(
                [this](){
                    // std::vector<bool> FLAGpackageloss(256,false);//之前测试丢包率的
                    // int count = 0;
                    while (SERIALrunning_flag){ 
                        
                        this->receive_fromserial();
                        // size_t spare_data = static_cast<size_t>(flag);//不上锁了,这个阶段不会有人写的
                        // if(!FLAGpackageloss[spare_data]){
                        //     count++;FLAGpackageloss[spare_data] = true;
                        //     std::cout<<"受到了第"<<spare_data<<"个,总共收到"<<count<<"/256"<<std::endl;
                        // }
                    }
            });
        }
        ~engineer_serial(){

            SERIALrunning_flag = false;
            if(reading_thread_.joinable()){
                reading_thread_.join();
            }
        }
        std::pair<u8,std::vector<float>> getdatas(){
            std::lock_guard<std::mutex> joints(receive_mutex_);
            return std::make_pair(flag, joints_now);                
        }
        void send_once(const int &flag,const std::vector<float> &now_joints,const int& LEFT_RIGHT = 0,const bool&IK_FLAG_success = false);
        std::thread reading_thread_;
        bool SERIALrunning_flag;
        bool wait_till_theydone(const std::vector<float>& goal);
        // bool send_tra(const std::vector<std::vector<double>>& trajectory);

    private:

        inline void receive_fromserial(){//正常比赛要使用的,接收电控发送给视觉的数据
            auto raw_data = serial_device_->receive_a_frame();//只要满足电控发送视觉的结构体小于80,那么就可以接收



            auto [flag_serial, floats] = hitcrt::ENGINEERstruct::DECODEvisual(raw_data);
            
            {
                std::lock_guard<std::mutex> joints(receive_mutex_);
                flag = flag_serial;
                joints_now = floats;
            }
        }
        inline void receive_simulator(){

            auto raw_data = serial_device_->receive_a_frame();//只要满足电控发送视觉的结构体小于80,那么就可以接收
            // auto [flag_serial, floats] = DataDecoder::decode(raw_data);
            auto [flag_serial, floats] = hitcrt::ENGINEERstruct::DECODEsimulator(raw_data);
            
            {
                std::lock_guard<std::mutex> joints(receive_mutex_);
                flag = flag_serial;
                joints_now = floats;
            }
        }

        };

    
};