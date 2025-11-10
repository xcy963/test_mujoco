#include <cstdio>
#include "engineer_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
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
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    std::vector<double> res;
    bool flag = instance_.inverse_kinematics(T,res);
    if(flag){
        RCLCPP_INFO(node->get_logger(),"使用单位阵的测试结果,第一个角度：%f 第二个： %f 第三个： %f",res[0],res[1],res[2]);

    }else{
        RCLCPP_INFO(node->get_logger(),"逆运动学失败");

    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}