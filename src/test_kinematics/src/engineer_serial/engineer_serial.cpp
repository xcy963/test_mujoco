#include "engineer_serial.h"
using namespace hitcrt;
#define SHOWWHATWESEND 0
void  engineer_serial::send_once(const int &flag,const std::vector<float> &now_joints,const int& LEFT_RIGHT ,const bool&IK_FLAG_success){//2是左边
        assert(now_joints.size()==7);
        // if(SHOWWHATWESEND){
            
        // }
        using namespace TASKS;

        std::vector<u8> buff;
        u8 visual;
        u8 space;
        u8 bocchi_FLAG;
        u8 see_entry_flag = static_cast<u8>(SEE_ENTRY_FLAG::NO);
        u8 ik_flag = static_cast<u8>(SEE_ENTRY_FLAG::NO);
        // if(LEFT_RIGHT == 0){//这样是默认值,直接两个no

        // }

        std::vector<float> joints_error = now_joints;
        joints_error[5] = joints_error[5] + JOINT6ERROE;
        switch(flag){
            case static_cast<int>(TASKS::IN_SENDING::ONLINE): 
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);

                space = static_cast<u8>(space::BOXNOTDETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);
                break;
            case static_cast<int>(TASKS::IN_SENDING::BOX_not_detected)://和ONLINE没有区分
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);

                space = static_cast<u8>(space::BOXNOTDETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;
            case static_cast<int>(TASKS::IN_SENDING::BOX1_detected):
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);

                space = static_cast<u8>(space::BOX1DETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;
            case static_cast<int>(TASKS::IN_SENDING::BOX2_detected)://这个状态还需要给电控发送左右
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);
                if(LEFT_RIGHT>0){
                    visual = visual+ (static_cast<u8>(LEFT_RIGHT)<<4);
                    see_entry_flag = static_cast<u8>(SEE_ENTRY_FLAG::YES);
                }
                if(IK_FLAG_success){
                    ik_flag = static_cast<u8>(SEE_ENTRY_FLAG::YES);
                }

                space = static_cast<u8>(space::BOX2DETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;
            case static_cast<int>(TASKS::IN_SENDING::SENDING)://和BOX_detected 没有区分，这个在逻辑上面是正常发送轨迹的
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);
                space = static_cast<u8>(space::BOX2DETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;
            case static_cast<int>(TASKS::IN_SENDING::LASTONE):
                visual = static_cast<u8>(VISUAL::CAMERAONLINE);
                space = static_cast<u8>(space::BOX2DETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::LASTONE);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;

            case static_cast<int>(TASKS::IN_SENDING::CAMERAOFFLINE):
                visual = static_cast<u8>(VISUAL::CAMERAOFFLINE);
                space = static_cast<u8>(space::BOXNOTDETECTED);
                bocchi_FLAG = static_cast<u8>(bocchi_FLAG::SENDING);
                data_struct_->update_SendToSerial_v2(visual,space,bocchi_FLAG,see_entry_flag,ik_flag,joints_error,buff);

                break;

        }
        // if(flag == static_cast<u8>(TASKS::IN_SENDING::Torque)){
        //     data_struct_->update_SendToSerial(flag,now_joints,buff,0x01);
        // }else{
        //     data_struct_->update_SendToSerial(flag,now_joints,buff);
        // }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));//发送之前限制,保证最多500帧
        // std::cout<<
        serial_device_->send(buff,buff.size());
        
        // std::cout<<"已经发出来了"<<buff.size()<<"头两个"<<(int)buff[0]<<"another:"<<(int)buff[1]<<std::endl;
        //raw_data是一个uint8数组，第一个字节是一个flag
        return;
    }


void engineer_serial::read_once(u8&flag_temp,std::vector<float> &joints){
    std::lock_guard<std::mutex>readingloc(receive_mutex_);
    flag_temp = flag;
    joints = joints_now;
    joints[5] = joints[5]-JOINT6ERROE;
}

