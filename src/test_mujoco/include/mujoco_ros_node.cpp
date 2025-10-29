// mujoco_ros_node.cpp
#include "mujoco_ros_node.h"
#include <chrono>
#include <thread>
#include "matplotlib/matplotlibcpp.h"
using namespace std::chrono_literals;

MuJoCoROSNode::MuJoCoROSNode() 
    : Node("mujoco_viewer") {
    start_plot_thread();
    // 声明参数
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("publish_rate", 100.0);
}

MuJoCoROSNode::~MuJoCoROSNode() {
    stop_plot_thread();
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
    // 设置步进回调,可以看做这个函数把所有的东西都拿出来了
    simulator_->setStepCallback(
        [this](const mjModel* model, const mjData* data) {
            if (running_ && !shutdown_requested_) {
                this->publishJointStates(model, data);

                std::lock_guard<std::mutex> lock(pic_loc_);//防止两个线程发生数据争端
                time_history_.push_back(data->time);
                if(torque_history_.size() != model->nv){
                    torque_history_.resize(model->nv);
                }
                // 存储关节力矩历史
                for (int i = 0; i < model->nv; i++) {
                    double this_joint = data->qfrc_actuator[i];
                    torque_history_[i].push_back(this_joint);
                }
                if(time_history_.size() > max_history_size_) {
                    auto time_copy = time_history_;
                    auto torque_copy = torque_history_;
                    
                    time_history_.clear();
                    torque_history_.clear();
                    
                    {
                        std::lock_guard<std::mutex> lock(plot_queue_mutex_);
                        plot_queue_.emplace(std::move(time_copy), std::move(torque_copy));
                    }
                    plot_cv_.notify_one();//因为我们只有一个线程,所以这个使用一个的
                }

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

void MuJoCoROSNode::draw_pics(
    const std::vector<double>& time_history,
    const std::vector<std::vector<double>>& torque_history)
{
    namespace plt = matplotlibcpp;

    if (time_history.empty() || torque_history.empty()) {
        RCLCPP_WARN(this->get_logger(), "空列表，跳过绘图");
        return;
    }

    size_t rows = torque_history.size();

    static std::mutex plt_mutex;
    std::lock_guard<std::mutex> lock(plt_mutex);

    plt::backend("Agg");
    plt::close();
    plt::figure_size(1200, 800);

    for (size_t i = 0; i < rows; i++) {
        if (torque_history[i].empty()) {
            RCLCPP_WARN(this->get_logger(), "torque_history[%zu] 为空，跳过绘制", i);
            continue;
        }

        plt::subplot(rows, 1, i + 1);

        std::stringstream ss;
        ss << "Joint " << i + 1 << " Torque";
        std::vector<double> test_x = time_history;
        std::vector<double> test_y = torque_history[i];//直接给会有bug
        plt::plot(time_history,torque_history[i]);//注意这里不能给他加第三个参数,不然subplot会报错
        plt::xlabel("Time (s)");
        plt::ylabel("Torque (N·m)");
        plt::title(ss.str());
        // plt::grid(true);
    }
    RCLCPP_INFO(this->get_logger(), "开始一次绘图");

    plt::tight_layout();
    // plt::show(false);
    // plt::save("/home/hitcrt/enginner_26/test_mujoco_ros/debug/1.jpg");
    std::vector<unsigned char> rgba_buffer;
    int width = 0, height = 0;
    plt::buffer_rgba(rgba_buffer, width, height);

    if (!rgba_buffer.empty() && width > 0 && height > 0) {
        cv::Mat rgba_mat(height, width, CV_8UC4, rgba_buffer.data());
        cv::Mat bgr_mat;
        cv::cvtColor(rgba_mat, bgr_mat, cv::COLOR_RGBA2BGR);
        cv::imwrite("/home/hitcrt/enginner_26/test_mujoco_ros/debug/2.png", bgr_mat);
    } else {
        RCLCPP_INFO(this->get_logger(), "返回的是空的或尺寸为0");
    }
    // plt::pause(1); // 短暂暂停以更新图形
}

void MuJoCoROSNode::plot_worker() {
    while (plot_running_) {
        std::unique_lock<std::mutex> lock(plot_queue_mutex_);
        plot_cv_.wait_for(lock, std::chrono::milliseconds(100));

        if (!plot_running_) break;

        if (!plot_queue_.empty()) {
            auto data = std::move(plot_queue_.front());
            plot_queue_.pop();
            lock.unlock();

            this->draw_pics(data.first, data.second);
        }
    }
    return;
}