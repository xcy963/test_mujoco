// mujoco_ros_node.cpp
#include "mujoco_ros_node.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

MuJoCoROSNode::MuJoCoROSNode() 
    : Node("mujoco_viewer") {
    
    // 声明参数
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("publish_rate", 100.0);
}

MuJoCoROSNode::~MuJoCoROSNode() {
    running_.store(false);
    shutdown_requested_.store(true);
    shutdown();
}

bool MuJoCoROSNode::initialize() {
    // 获取参数
    std::string model_path = this->get_parameter("model_path").as_string();
    
    if (model_path.empty()) {
        // 使用默认路径
        model_path = "/home/hitcrt/enginner_26/test_mujoco_ros/src/test_mujoco/modules/trs_so_arm100/scene.xml";
        RCLCPP_WARN(this->get_logger(), "No model path specified, using default: %s", model_path.c_str());
    }
    
    // 初始化MuJoCo模拟器
    simulator_ = std::make_unique<MuJoCoSimulator>();
    if (!simulator_->initialize(model_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MuJoCo simulator!");
        return false;
    }
    
    // 设置步进回调
    simulator_->setStepCallback(
        [this](const mjModel* model, const mjData* data) {
            if (running_ && !shutdown_requested_) {
                
                this->publishJointStates(model, data);
            }else{
                RCLCPP_INFO(this->get_logger(),"还在尝试发消息,但是应该不发");
            }
        }
    );
    
    // 创建ROS发布器和订阅器
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/mujoco/joint_states", 10);
    
    joint_control_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/mujoco/joint_controls", 10,
        std::bind(&MuJoCoROSNode::controlCallback, this, std::placeholders::_1));
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/mujoco/status", 10);
    
    // 创建定时器
    double publish_rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&MuJoCoROSNode::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "MuJoCo ROS node initialized successfully");
    return true;
}

void MuJoCoROSNode::run() {
    if (!simulator_) {
        RCLCPP_ERROR(this->get_logger(), "Simulator not initialized!");
        return;
    }
    
    running_ = true;
    shutdown_requested_ = false;
    
    // 发布状态信息
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "MuJoCo simulator running";
    status_pub_->publish(status_msg);
    
    RCLCPP_INFO(this->get_logger(), "MuJoCo simulator started");
    
    // 启动模拟器（这会阻塞，直到模拟器窗口关闭）
    simulator_->start();
    simulator_->renderLoop();
    
    // 模拟器窗口关闭后，执行清理
    shutdown();
}

void MuJoCoROSNode::shutdown() {
    if (shutdown_requested_) {
        return;
    }
    
    shutdown_requested_ = true;
    running_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Shutting down MuJoCo ROS node");
    
    // 先停止ROS组件
    stopROSComponents();
    
    // 然后停止模拟器
    if (simulator_) {
        simulator_->stop();
    }
    
    // 短暂延迟，确保所有消息都已处理
    std::this_thread::sleep_for(100ms);
    
    RCLCPP_INFO(this->get_logger(), "MuJoCo ROS node shutdown complete");
}

void MuJoCoROSNode::stopROSComponents() {
    // 停止定时器
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // 重置发布器和订阅器
    joint_state_pub_.reset();
    joint_control_sub_.reset();
    status_pub_.reset();
}

void MuJoCoROSNode::publishJointStates(const mjModel* model, const mjData* data) {
    if (!model || !data || !running_ || shutdown_requested_) return;
    
    try {
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        joint_msg.header.frame_id = "world";
        
        // 填充关节状态信息
        // 注意：这里需要根据你的模型结构来正确填充关节名称和状态
        for (int i = 0; i < model->nq; i++) {
            joint_msg.position.push_back(data->qpos[i]);
        }
        for (int i = 0; i < model->nv; i++) {
            joint_msg.velocity.push_back(data->qvel[i]);
        }
        
        // 发布关节状态
        if (joint_state_pub_ && running_ && !shutdown_requested_) {
            joint_state_pub_->publish(joint_msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing joint states: %s", e.what());
    }
}

void MuJoCoROSNode::controlCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!running_ || shutdown_requested_) return;
    
    // 处理关节控制命令
    RCLCPP_DEBUG(this->get_logger(), "Received joint control command");
    // 这里可以实现将ROS控制命令应用到MuJoCo模拟器中
}

void MuJoCoROSNode::timerCallback() {
    if (!running_ || shutdown_requested_) return;
    
    // 定期发布状态信息
    try {
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "MuJoCo simulator running";
        if (status_pub_ && running_ && !shutdown_requested_) {
            status_pub_->publish(status_msg);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
    }
}