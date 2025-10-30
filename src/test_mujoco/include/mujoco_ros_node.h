// mujoco_ros_node.h
#ifndef MUJOCO_ROS_NODE_H
#define MUJOCO_ROS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <atomic>
#include <opencv2/opencv.hpp>


#include "mujoco_test/mujoco_simulator.h"

class MuJoCoROSNode : public rclcpp::Node {
public:
    MuJoCoROSNode();
    ~MuJoCoROSNode();
    
    bool initialize();
    void run();
    void shutdown();
    inline void start_plot_thread() {
        plot_thread_ = std::thread(&MuJoCoROSNode::plot_worker, this);
    }
    
    inline void stop_plot_thread() {
        plot_running_ = false;
        plot_cv_.notify_all();
        if (plot_thread_.joinable()) {
            plot_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(),"画图的节点退出");
    }

private:
    void publishJointStates(const mjModel* model, const mjData* data);
    void controlCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timerCallback();
    void stopROSComponents();

    void draw_pics(const std::vector<double>& time_history,
                    const std::vector<std::vector<double>> &torque_history);
    
    std::unique_ptr<MuJoCoSimulator> simulator_;
    
    // ROS发布器和订阅器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_control_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_requested_{false};

    std::vector<double> time_history_;
    std::vector<std::vector<double>> torque_history_;
    const size_t max_history_size_ = 2000;
    // size_t now_size
    std::mutex pic_loc_;

    std::thread plot_thread_;
    std::atomic<bool> plot_running_{true};
    std::mutex plot_queue_mutex_;
    std::condition_variable plot_cv_;
    std::queue<std::pair<std::vector<double>, std::vector<std::vector<double>>>> plot_queue_;//中间数组
    
    void plot_worker() ;

};

#endif // MUJOCO_ROS_NODE_H