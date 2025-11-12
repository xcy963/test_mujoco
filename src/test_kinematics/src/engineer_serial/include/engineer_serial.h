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
    
    class engineer_serial{
    private:
        // SerialDevice serial_device_;
        std::shared_ptr<SerialDevice> serial_device_;
        std::shared_ptr<DataStruct> data_struct_;

        std::mutex receive_mutex_;     //控制读取角度值的
        std::vector<float> joints_now;//串口过来的角度值，存放在主类里面,为了保证读取的帧率,使用float
        u8 flag  = 0x00;    //串口发送过来的字节数组


    public:
        void read_once(std::vector<float> &joints){
            // std::cout<<"准备读取一次"<<std::endl;

            std::lock_guard<std::mutex>readingloc(receive_mutex_);
            joints = joints_now;
        }
        bool send_once_inter(const std::vector<float> &joints_command ,const  float scale){
            std::vector<float> joints_copy;
            {
                std::lock_guard<std::mutex> readingloc(receive_mutex_);
                joints_copy = joints_now;
            }
            if(joints_copy.size() != joints_command.size()){
                return false;
            }
            std::vector<float> joints_send;
            // float sum = 0.0f;
            for(size_t i = 0;i < joints_copy.size();i++){
                float joint_temp = joints_command[i] - joints_copy[i];
                joints_send.push_back(joint_temp);
                // sum += std::abs(joint_temp);
            }
            // sum = std::sqrt(sum);

            for(size_t i = 0;i < joints_copy.size();i++){
                joints_send[i] = (joints_send[i] ) * scale + joints_copy[i];//做一个缩放,不让电控那边执行得太猛
            }
            // std::cout<<"发送给电控的控制量joint1:"<<joints_send[0]<<" joint2:"<<joints_send[1]<<" joint3:"<<joints_send[2]<<std::endl;

            std::vector<u8> buff;
            // u8 visual = 0;
            // u8 space = 0;
            // u8 bocchi_FLAG = 0;
            // u8 see_entry_flag = 0;
            // u8 ik_flag = 0;
            data_struct_->update_SendToSerial_v2(joints_send,buff);
            serial_device_->send(buff,buff.size());
            return true;
        }

        engineer_serial(const std::string& name, int baudRate,int joint_nums = 3){
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
            joints_now = std::vector<float>(joint_nums,0);//初始都是0
            reading_thread_ = std::thread(
                [this](){
                    std::cout<<"开始创建接收线程"<<std::endl;
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
            if (serial_device_) {
                serial_device_->stop();
            }
            if(reading_thread_.joinable()){
                reading_thread_.join();
            }
        }
        std::pair<u8,std::vector<float>> getdatas(){
            std::lock_guard<std::mutex> joints(receive_mutex_);
            return std::make_pair(flag, joints_now);                
        }
        void send_once(const std::vector<float> &now_joints){
            std::vector<u8> buff;

            data_struct_->update_SendToSerial_v2(now_joints,buff);
            // std::stringstream hex_ss;
            // // hex_ss << "进入crc之前发送的数据Hex: ";
            // for (size_t i = 0; i < std::min(buff.size(), size_t(40)); ++i) { // 限制输出前20字节
            //     hex_ss << std::hex << std::setw(2) << std::setfill('0') 
            //         << static_cast<int>(buff[i]) << " ";
            // }
            // std::cout<<hex_ss.str()<<std::endl;

            serial_device_->send(buff,buff.size());
        }
        std::thread reading_thread_;
        bool SERIALrunning_flag;
        // bool wait_till_theydone(const std::vector<float>& goal);
        // bool send_tra(const std::vector<std::vector<double>>& trajectory);

    private:

        inline void receive_fromserial(){//正常比赛要使用的,接收电控发送给视觉的数据
            std::vector<u8> raw_data = serial_device_->receive_a_frame();//只要满足电控发送视觉的结构体小于80,那么就可以接收
            // std::cout<<"接受到的数组的长度"<<raw_data.size()<<""<<std::endl;
            // std::stringstream hex_ss;
            // hex_ss << "Hex: ";
            // for (size_t i = 0; i < std::min(raw_data.size(), size_t(20)); ++i) { // 限制输出前20字节
            //     hex_ss << std::hex << std::setw(2) << std::setfill('0') 
            //         << static_cast<int>(raw_data[i]) << " ";
            // }
            // std::cout<<hex_ss.str()<<std::endl;

            auto  floats = hitcrt::ENGINEERstruct::DECODEvisual(raw_data);
            
            {
                std::lock_guard<std::mutex> joints(receive_mutex_);
                // flag = flag_serial;
                joints_now = floats;
            }
        }


        };

    
};