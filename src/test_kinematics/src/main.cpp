#include <cstdio>
#include "engineer_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include "opencv_setinput.hpp"
#include "matplotlibcpp.h"
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
const std::string exit_str = "exit";

const std::string jointZ_str = "joint1_Z";
const std::string jointY_str = "joint2_Y";
const std::string jointX_str = "joint3_X";




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

void draw_pic(const std::vector<std::vector<float>> &test_angle_res){
    namespace plt = matplotlibcpp;

    if (test_angle_res.empty()) {
        std::cout << "警告：test_angle_res为空，无法绘图！" << std::endl;
        return;
    }

    // 准备数据：将三维点分解为x、y、z坐标向量
    std::vector<float> x_vals, y_vals, z_vals;
    for (const auto &point : test_angle_res) {
        // 确保每个点至少有三个维度（x, y, z）
        if (point.size() >= 3) {
            z_vals.push_back(point[0]); // 第一个元素作为x坐标
            y_vals.push_back(point[1]); // 第二个元素作为y坐标
            x_vals.push_back(point[2]); // 第三个元素作为z坐标
        }
    }

    // 创建三维图形
    // plt::figure_size(1000, 800); // 设置图形大小

    std::map<std::string, std::string> keywords;
    keywords["marker"] = "o";           // 圆形标记
    // keywords["linestyle"] = "none";     // 无连线
    keywords["color"] = "blue";         // 蓝色
    keywords["label"] = "joint_pose";      // 图例标签
    // keywords["s"] = "100"; 
    // 绘制三维散点图
    plt::scatter(x_vals, y_vals,10.0, keywords);

    std::map<std::string, std::string> label_style = {
        {"fontsize", "16"},
        {"color", "red"},
        {"labelpad", "15"},
        {"fontweight", "bold"}
    };

    // 添加图形标签和标题
    plt::title("SPACE_distri");
    plt::xlabel("X",label_style);
    plt::ylabel("Y",label_style);
    // plt::zlabel("Z",label_style);

    // 添加网格使观察更直观
    plt::grid(true);

    // 显示图形
    plt::show();
}

void test_joint_limit(const std::shared_ptr<hitcrt::engineer_serial> &ser,
            const std::shared_ptr<hitcrt::kinematics::engineer_kinematics> &instance_ptr,
        const std::shared_ptr<hitcrt::DoubleTrackbarController>  &controller_ins){

    auto judge_hitpos = [](const std::vector<float> &joints_pos,const std::vector<float> &joints_command,float error){
        float sum = 0.0f;
        if(joints_pos.size() != joints_command.size()){
            return false;
        }
        for(size_t i = 0;i < joints_pos.size();i++){
            float joint_temp = joints_command[i] - joints_pos[i];
            sum += std::abs(joint_temp);
        }
        if(sum < error){
            return true;
        }else{
            return false;
        }
        // return sum;
    };

    auto push_once = [](std::vector<std::vector<float>> &test_angle_res,const std::vector<float> &joinst_push,float threash = 2){//默认用3度作为阈值

        float dis_his_min = std::numeric_limits<float>::max();;//目标是找出最近的,所以这个最好很大 
        for(size_t i = 0;i < test_angle_res.size();i++){//遍历目前已经有的点

            float sum = 0.0f;
            for(size_t j = 0;j < joinst_push.size();j++){
                float joint_temp = test_angle_res[i][j] - joinst_push[j];
                sum += std::abs(joint_temp);
            }
            if(sum < dis_his_min){
                dis_his_min = sum;
            }

        }

        if(threash < dis_his_min){//大于阈值才放进去
            test_angle_res.push_back(joinst_push);
        }

    };

    std::vector<float> test_angle_command = {0.0, 0.0, 0.0};//输出的控制,这个表示ZYX欧拉角
    std::vector<std::vector<float>> test_angle_res;//收集电机能到达的位置

    bool flag = true;
    // float step = 0.30f;
    float error  = 1.0f;

    while(flag && rclcpp::ok()){
        // std::cout<<"还在检测"<<std::endl;

        double exit_val = controller_ins->getVariable(exit_str);//这个指令控制退出循环
        if(exit_val < 0){
            flag = false;
        }

        test_angle_command.at(0) = controller_ins->getVariable(jointZ_str);
        test_angle_command.at(1) = controller_ins->getVariable(jointY_str);
        test_angle_command.at(2) = controller_ins->getVariable(jointX_str);

        //把控制的欧拉角变成矩阵
        std::cout<<"这个时候的欧拉角joint1:"<<test_angle_command[0]<<" joint2:"<<test_angle_command[1]<<" joint3:"<<test_angle_command[2]<<std::endl;

        Eigen::Vector3d euler_angles(test_angle_command.at(0)/180*M_PI, test_angle_command.at(1)/180*M_PI, test_angle_command.at(2)/180*M_PI);
    
        // Eigen 中 ZYX 欧拉角的默认顺序
        Eigen::Matrix3d R = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(R);
        std::cout<<"看一下矩阵\n"<<R<<std::endl;
        std::vector<double> joints_command;

        bool flag_inverse = instance_ptr->inverse_kinematics(T,joints_command);//这个现在还不能接受当前的角度值,以后需要修改这个,让他变得更加理想
        
        if(!flag_inverse){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            continue;
        }
        std::cout<<"发送给电控的控制量joint1:"<<joints_command[0]*180*M_1_PI<<" joint2:"<<joints_command[1]*180*M_1_PI<<" joint3:"<<joints_command[2]*180*M_1_PI<<std::endl;
        std::vector<float> joints_command_float;
        joints_command_float.push_back(static_cast<float>(joints_command[0] * 180 * M_1_PI));
        joints_command_float.push_back(static_cast<float>(joints_command[1] * 180 * M_1_PI));
        joints_command_float.push_back(static_cast<float>(joints_command[2] * 180 * M_1_PI));
        
        if(ser){
            // ser->send_once_inter(joints_command_float,step);
            ser->send_once(joints_command_float);
            
            std::vector<float> joints;
            ser->read_once(joints);//获取电控的反馈,是角度制度
            if(judge_hitpos(joints,joints_command_float,error)){    //过判断,表示电控的现在到给定位置了
                push_once(test_angle_res,test_angle_command);//注意这里放进去的是欧拉角,不是逆运动学的结果
            }   
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


    }
    draw_pic(test_angle_res);



}

std::vector<std::vector<float>> create_joint_trajectory() {
    std::vector<std::vector<float>> trajectory;
    
    // 模拟一个简单的关节运动轨迹
    int steps = 50;
    for (int i = 0; i < steps; ++i) {
        float t = static_cast<float>(i) / steps;
        
        std::vector<float> point(3);
        // 模拟三个关节的周期性运动
        point[0] = sin(2 * M_PI * t) * 1.5f;           // 关节Z
        point[1] = cos(2 * M_PI * t) * 1.0f;           // 关节Y
        point[2] = sin(4 * M_PI * t + M_PI/4) * 0.8f;  // 关节X
        
        trajectory.push_back(point);
    }
    
    return trajectory;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_kinematicsNODE");
    // auto data = create_joint_trajectory();
    // draw_pic(data);
    hitcrt::kinematics::params params_temp;
    // params_temp
    print_params(params_temp, node);
    // hitcrt::kinematics::engineer_kinematics instance_(params_temp);
       std::shared_ptr<hitcrt::kinematics::engineer_kinematics>  instance_ptr = 
                    std::make_shared<hitcrt::kinematics::engineer_kinematics>(params_temp); 
    std::atomic<bool> running{true};

    std::shared_ptr<hitcrt::DoubleTrackbarController>  controller_ins = 
                    std::make_shared<hitcrt::DoubleTrackbarController>("参数控制器");
    // RCLCPP_INFO(node->get_logger(), "创建窗口成功");
    
    // 添加要控制的变量
    controller_ins->addVariable(jointZ_str, 0.0);
    controller_ins->addVariable(jointY_str, 0.0);
    controller_ins->addVariable(jointX_str, 0.0);
    controller_ins->addVariable("exit", 90);

    // RCLCPP_INFO(node->get_logger(), "添加变量成功");

    std::thread th_params([&](){//控制可视化调参窗口的线程
        // controller_ins->run(running);
        if (!controller_ins->initialize()) {
            return;
        }

        while (running.load()) {
            char key = cv::waitKey(30);//顺便是给线程加等待他是控制30ms
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == 'p' || key == 'P') {
                controller_ins->printCurrentValues();
            } 
            // else if (key == 's' || key == 'S') {
            //     controller_ins->saveValuesToFile("current_values.txt");
            // }
            // double data = controller_ins->getVariable(jointZ_str);
            // RCLCPP_INFO(node->get_logger(),"现在的角度是%f",data);
            // if(controller_ins->getVariable(exit_str) < 0 ){
            //     running.store(false);
            //     break;
            // }
        }

        std::cout << "\nFinal values:" << std::endl;
        controller_ins->printCurrentValues();
        cv::destroyAllWindows();
    });

    
    // RCLCPP_INFO(node->get_logger(), "不是opencv的问题");

    std::thread th_ser;
    std::shared_ptr<hitcrt::engineer_serial> temp_ser;
    try {
        temp_ser = std::make_shared<hitcrt::engineer_serial>("auto", 921600);
        
        if(temp_ser) {
            th_ser = std::thread([&](){
                while(running.load() && rclcpp::ok()) {
                    // test_joint_limit(temp_ser,instance_ptr,controller_ins);
                    //这段代码是测试测试和电控的通信的
                    std::vector<float> joints_command_float = {1,2,3};
                    joints_command_float.at(0) = controller_ins->getVariable(jointZ_str);
                    joints_command_float.at(1) = controller_ins->getVariable(jointY_str);
                    joints_command_float.at(2) = controller_ins->getVariable(jointX_str);
                    temp_ser->send_once(joints_command_float);

                    std::stringstream ss_send;
                    ss_send << "要发送的角度值:";
                    for(size_t i = 0; i < joints_command_float.size(); ++i) {
                        ss_send << i << ":" << joints_command_float[i] << " , ";
                    }
                    RCLCPP_INFO(node->get_logger(), "%s", ss_send.str().c_str());

                    std::vector<float> joints;
                    temp_ser->read_once( joints);

                    std::stringstream ss;
                    ss << "读取到的角度值:";
                    for(size_t i = 0; i < joints.size(); ++i) {
                        ss << i << ":" << joints[i] << " , ";
                    }
                    RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
                    float sum_error = 0.0f;
                    std::stringstream ss_error;

                    for(size_t k = 0;k < joints_command_float.size();k++){
                        float temp_error = std::abs(joints_command_float[k] - joints[k]);
                        ss_error<<"第"<<k + 1<<"个的误差是: "<<temp_error<<" ";
                        sum_error += temp_error;
                    }
                    ss_error<<"总的误差是"<<sum_error;
                    RCLCPP_INFO(node->get_logger(), "%s", ss_error.str().c_str());


                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    if(controller_ins->getVariable(exit_str) < 0 ){
                        break;
                    }
                }

                // test_joint_limit(temp_ser,instance_ptr,controller_ins);
            });

        }
        
        // rclcpp::spin(node);
        
    } catch (const char* e) {
        RCLCPP_ERROR(node->get_logger(), "发生异常: %s", e);
    }catch (...) {
        RCLCPP_ERROR(node->get_logger(), "发生未知异常");
    }

    rclcpp::spin(node);
    // RCLCPP_INFO(node->get_logger(), "马上要退出");

    running = false;

    if(th_ser.joinable()) {
        th_ser.join();
    }
    if(th_params.joinable()) {
        th_params.join();
    } 
    // 清理资源
    running = false;
    // th_params


    rclcpp::shutdown();
    return 0;
}