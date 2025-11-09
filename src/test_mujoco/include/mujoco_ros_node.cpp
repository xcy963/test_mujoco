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
    this->declare_parameter<double>("publish_rate", 30.0);
    CameraRenderer_ = std::make_unique<CameraRenderer>();
}

MuJoCoROSNode::~MuJoCoROSNode() {
    // RCLCPP_INFO(this->get_logger(),"准备退出画图线程");
    stop_plot_thread();
    running_.store(false);
    shutdown_requested_.store(true);
    shutdown();
    CameraRenderer_->shutdown();
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
    // 设置步进回调,这个函数是步进的一部分，在mujoco计算完毕他的仿真之后执行这个函数
    time_last_ = std::chrono::steady_clock::now();
    simulator_->setStepCallback(
        [this](const mjModel* model, const mjData* data) {
            if (running_ && !shutdown_requested_) {
                // this->publishJointStates(model, data);
                // auto time_now = std::chrono::steady_clock::now();
                // double fps = std::chrono::duration<double>(time_now - time_last_).count();
                // time_last_ = time_now;
                // fps = 1.0/fps;
                // RCLCPP_INFO(this->get_logger(),"现在的步进帧率%f",fps);
                
                // CameraRenderer_->list_cameras(model);
                std::lock_guard<std::mutex> lock(pic_loc_);//防止两个线程发生数据争端
                double time_now = data->time;
                time_history_.push_back(time_now);

                if(torque_history_.size() != model->nv){
                    torque_history_.clear();
                    torque_history_.resize(model->nv);
                }

                // 存储关节力矩历史
                for (int i = 0; i < model->nv; i++) {
                    double this_joint = data->qfrc_actuator[i];
                    if(std::abs(this_joint)<1e-5){//太小直接给0,这样就不会有轴的问题了
                        this_joint = 0;
                    }
                    torque_history_[i].push_back(this_joint);
                }

                if((time_history_.size() % max_history_size_) == 0 ) {
                    auto time_copy = time_history_;
                    auto torque_copy = torque_history_;
                    //第一次优化，这个线程不做画图的处理，只是负责处理数据
                    // time_history_.clear();
                    // torque_history_.clear();
                    
                    {
                        std::lock_guard<std::mutex> lock(plot_queue_mutex_);
                        plot_queue_.emplace(std::move(time_copy), std::move(torque_copy));
                    }
                    plot_cv_.notify_one();//因为我们只有一个线程,所以这个使用一个的
                }

                if((time_history_.size() / max_history_size_) >= 5 ){
                    time_history_.erase(time_history_.begin(), time_history_.begin() + max_history_size_);//删掉前面的这么多个,不然容易内存泄漏
                    for (int i = 0; i < model->nv; i++) {
                        double this_joint = data->qfrc_actuator[i];
                        
                        torque_history_[i].erase(torque_history_[i].begin(), torque_history_[i].begin() + max_history_size_);//删除第一个，这样他就平衡了
                    }
                }

            }else{
                // RCLCPP_INFO(this->get_logger(),"还在尝试发消息,但是应该不发");
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
    // 运行模拟器（这会阻塞，直到模拟器窗口关闭）
    simulator_->start();
    // std::cout<<"start 函数返回"<<std::endl;
    simulator_->renderLoop(CameraRenderer_);
    
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
    //关闭画图的线程
    stop_plot_thread();
    // 停止ROS组件
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

    CameraFrame frame;
    bool ok = CameraRenderer_->wait_for_frame(frame);
    if(ok){
        cv::Mat img_rgb(frame.height, frame.width, CV_8UC3, const_cast<unsigned char*>(frame.rgb_data.data()));
        cv::Mat img_bgr;
        cv::cvtColor(img_rgb, img_bgr, cv::COLOR_RGB2BGR);
        cv::flip(img_bgr, img_bgr, 0);

        cv::imshow("camera_rgb", img_bgr);
        cv::waitKey(1);//放在这里为了刷新opencv的窗口
    }
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
    plt::figure_size(1280, 1300);

    for (size_t i = 0; i < rows; i++) {
        if (torque_history[i].empty()) {
            RCLCPP_WARN(this->get_logger(), "torque_history[%zu] 为空，跳过绘制", i);
            continue;
        }
        // RCLCPP_INFO(this->get_logger(), "Joint %zu Torque 范围: [%f, %f]", 
        //    i, 
        //    ,
        //    );
        // RCLCPP_INFO(this->get_logger(), "现在的i是%ld",i);

        plt::subplot(rows, 1, i + 1);

        std::vector<double> test_x = time_history;
        std::vector<double> test_y = torque_history[i];//直接给会有bug
        std::map<std::string, std::string> keywords;
        double min_ele = *std::min_element(torque_history[i].begin(), torque_history[i].end());
        double max_ele = *std::max_element(torque_history[i].begin(), torque_history[i].end());
        keywords["label"] = std::format("max{:.2f},min{:.2f}",max_ele,min_ele);
        plt::plot(time_history,torque_history[i],keywords);//注意这里不能给他加第三个参数,不然subplot会报错
        plt::ylim(-100, 100);
        // std::vector<double> y_ticks = {-100,  -60,  -20, 0,  20, 60,  100};
        // plt::yticks(y_ticks);
        plt::xlabel("Time (s)");
        plt::ylabel("Torque (N·m)");
        // plt::ticklabel_format(axis="y", style="plain"); 
        plt::title(std::format("Joint {} Torque", i + 1));

        std::map<std::string, std::string> legend_kw;
        legend_kw["loc"] = "upper right";
        plt::legend(legend_kw);
        plt::grid(true);
    }
    // RCLCPP_INFO(this->get_logger(), "开始一次绘图");

    plt::tight_layout();
    // plt::show(false);
    // plt::save("/home/hitcrt/enginner_26/test_mujoco_ros/debug/1.jpg");
    std::vector<unsigned char> rgba_buffer;
    int width = 0, height = 0;
    plt::buffer_rgba(rgba_buffer, width, height);//a是透明度

    if (!rgba_buffer.empty() && width > 0 && height > 0) {
        cv::Mat rgba_mat(height, width, CV_8UC4, rgba_buffer.data());
        cv::Mat bgr_mat;
        cv::cvtColor(rgba_mat, bgr_mat, cv::COLOR_RGBA2BGR);
        // cv::imwrite("/home/hitcrt/enginner_26/test_mujoco_ros/debug/2.png", bgr_mat);
        cv::imshow("test",bgr_mat);
        // cv::waitKey(1);
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