#include <cstdio>
#include "engineer_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include "opencv_setinput.hpp"
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>



void print_params(const hitcrt::kinematics::params& p, 
                  const std::shared_ptr<rclcpp::Node>& node) {

    auto rad_to_deg = [](const double &rad) {
        return rad * 180.0 / M_PI;
    };
    RCLCPP_INFO(node->get_logger(), "=== 结构体参数 ===");
    RCLCPP_INFO(node->get_logger(), "delta:  %.2f° (%.4f rad)", rad_to_deg(p.delta), p.delta);
    RCLCPP_INFO(node->get_logger(), "yita1:  %.2f° (%.4f rad)", rad_to_deg(p.yita1), p.yita1);
    RCLCPP_INFO(node->get_logger(), "yita2:  %.2f° (%.4f rad)", rad_to_deg(p.yita2), p.yita2);
    RCLCPP_INFO(node->get_logger(), "yita3:  %.2f° (%.4f rad)", rad_to_deg(p.yita3), p.yita3);
    RCLCPP_INFO(node->get_logger(), "arfa1:  %.2f° (%.4f rad)", rad_to_deg(p.arfa1), p.arfa1);
    RCLCPP_INFO(node->get_logger(), "arfa2:  %.2f° (%.4f rad)", rad_to_deg(p.arfa2), p.arfa2);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_kinematicsNODE");
    hitcrt::kinematics::params params_temp;
    // params_temp
    print_params(params_temp, node);
    hitcrt::kinematics::engineer_kinematics instance_(params_temp);


    // hitcrt::DoubleTrackbarController controller("参数控制器");
    std::shared_ptr<hitcrt::DoubleTrackbarController>  controller_ins = 
                    std::make_shared<hitcrt::DoubleTrackbarController>("参数控制器");
    RCLCPP_INFO(node->get_logger(), "创建窗口成功");
    
    // 添加要控制的变量
    controller_ins->addVariable("joint1", 0.5);
    controller_ins->addVariable("joint2", 1.0);
    controller_ins->addVariable("joint3", 0.8);
    RCLCPP_INFO(node->get_logger(), "添加变量成功");

    std::atomic<bool> running{true};
    std::thread th_params([&](){
        // controller_ins->run(running);
        if (!controller_ins->initialize()) {
            return;
        }

        while (running.load()) {
            char key = cv::waitKey(30);
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == 'p' || key == 'P') {
                controller_ins->printCurrentValues();
            } 
            // else if (key == 's' || key == 'S') {
            //     controller_ins->saveValuesToFile("current_values.txt");
            // }
        }

        std::cout << "\nFinal values:" << std::endl;
        controller_ins->printCurrentValues();
    });
    
    RCLCPP_INFO(node->get_logger(), "不是opencv的问题");

    std::thread th_ser;
    
    try {
        auto temp_ser = std::make_shared<hitcrt::engineer_serial>("auto", 563200);
        
        if(temp_ser) {
            auto receive = [&](){
                u8 flag_temp;
                std::vector<float> joints;
                temp_ser->read_once(flag_temp, joints);
                
                std::stringstream ss;
                ss << "读取到的角度值:";
                for(size_t i = 0; i < joints.size(); ++i) {
                    ss << i << ":" << joints[i] << " , ";
                }
                RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
            };
            
            th_ser = std::thread([&](){
                while(running.load()) {
                    receive();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });
        }
        
        // rclcpp::spin(node);
        
    } catch (const char* e) {
        RCLCPP_ERROR(node->get_logger(), "发生异常: %s", e);
    }catch (...) {
        RCLCPP_ERROR(node->get_logger(), "发生未知异常");
    }

    rclcpp::spin(node);
    
    // 清理资源
    running = false;
    // if(th_ser.joinable()) {
    //     th_ser.join();
    // }
    // th_params
    if(th_params.joinable()) {
        th_params.join();
    }
    rclcpp::shutdown();
    return 0;
}