// main.cpp
#include "mujoco_ros_node.h"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <iostream>
#include <atomic>
#include <csignal>

std::shared_ptr<MuJoCoROSNode> node;
std::atomic<bool> shutdown_signal_received{false};

void signalHandler(int signum) {
    if (shutdown_signal_received) {
        return; // 已经在关闭中，避免重复处理
    }
    
    shutdown_signal_received = true;
    std::cout << "Received signal " << signum << ", shutting down gracefully..." << std::endl;
    
    if (node) {
        node->shutdown();
    }
    
    // 给ROS一些时间进行清理
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    rclcpp::shutdown();
    std::exit(signum);
}

void setupSignalHandlers() {
    struct sigaction action;
    action.sa_handler = signalHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    
    // 忽略SIGPIPE，避免网络连接问题导致程序退出
    signal(SIGPIPE, SIG_IGN);
}

int main(int argc, char** argv) {
    // 设置信号处理
    setupSignalHandlers();
    
    try {
        // 初始化ROS
        rclcpp::init(argc, argv);
        
        // 创建节点
        node = std::make_shared<MuJoCoROSNode>();
        
        // 初始化节点
        if (!node->initialize()) {
            std::cerr << "Failed to initialize MuJoCo ROS node" << std::endl;
            rclcpp::shutdown();
            return -1;
        }
        
        // std::cout << "MuJoCo ROS node started. Press Ctrl+C to exit." << std::endl;
        
        // 运行模拟器（这会阻塞，直到模拟器窗口关闭）
        node->run();
        
        RCLCPP_INFO(node->get_logger(),"MuJoCo ROS node finished.");
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        if (node) {
            node->shutdown();
        }
        rclcpp::shutdown();
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}