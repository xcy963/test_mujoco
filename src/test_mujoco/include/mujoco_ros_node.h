// mujoco_ros_node.h
#ifndef MUJOCO_ROS_NODE_H
#define MUJOCO_ROS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <atomic>

#include "mujoco_test/mujoco_simulator.h"

class MuJoCoROSNode : public rclcpp::Node {
public:
    MuJoCoROSNode();
    ~MuJoCoROSNode();
    
    bool initialize();
    void run();
    void shutdown();

private:
    void publishJointStates(const mjModel* model, const mjData* data);
    void controlCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timerCallback();
    void stopROSComponents();
    
    std::unique_ptr<MuJoCoSimulator> simulator_;
    
    // ROS发布器和订阅器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_control_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_requested_{false};
};

#endif // MUJOCO_ROS_NODE_H