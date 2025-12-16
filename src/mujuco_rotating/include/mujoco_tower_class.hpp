// mujoco_tower_class.cpp
// 把 MuJoCo 离屏渲染封装成一个类，支持：
// 1) 相机回调（OpenCV 显示/录制）；
// 2) 控制回调（外部控制机械臂电机 d->ctrl[]）；
// 3) 兑换站 entry_free 位姿接口（使用 Eigen）。
#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
GLFWwindow* g_window = nullptr;

#include <opencv2/opencv.hpp>

// Eigen 用来表示位姿
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "window_control.hpp"

//============================= 类定义 ======================================

namespace hitcrt{

struct TeleopState {//底盘的控制结构体
    double vx = 0.0;  // 线速度 x（m/s，车体前方为正）
    double vy = 0.0;  // 线速度 y（m/s，车体左方为正）
    double wz = 0.0;  // 角速度（rad/s，绕 z，左转为正）
};


class MujocoOffscreenRenderer {
   public:
    // std::mutexs

    // 相机回调：给 RGB 图像 + 宽高 + 仿真时间
    using CameraCallback =
        std::function<void(const unsigned char*,const float*, int, int, double)>;
    // 控制回调：每个仿真步调用一次，你在里面写 d->ctrl / 改 qpos 等
    using ControlCallback = std::function<void(mjModel*, mjData*, double)>;

    MujocoOffscreenRenderer(const std::string& model_path, double fps)
        : model_path_(model_path), fps_(fps) {
        running_.store(true);
        initOpenGL();
        initMuJoCo();
        // setupOffscreen();
    }

    ~MujocoOffscreenRenderer() { cleanup(); }

    void set_running(const bool& flag) { running_.store(flag); }
    //记得在run之前需要设置一遍这个
    void useFixedCamera(const std::string& cam_name) {
      int cam_id = mj_name2id(m_, mjOBJ_CAMERA, cam_name.c_str());
      if (cam_id < 0) {
        std::printf("Warn: camera '%s' not found in model.\n", cam_name.c_str());
        return;
      }
      cam_.type = mjCAMERA_FIXED;
      cam_.fixedcamid = cam_id;
    //   setupOffscreen();
        setupOffscreenWithCamera(cam_id);
    }
    // 设置用户的相机回调
    void setCameraCallback(CameraCallback cb) {
        camera_callback_ = std::move(cb);
    }

    // 设置用户的控制回调（用来控制机械臂 / 其他关节）
    void setControlCallback(ControlCallback cb) {
        control_callback_ = std::move(cb);
    }

    // 设置 world->exchanger 的位姿（一般你可以直接设为 Identity）
    void setWorldExchangerPose(const Eigen::Isometry3d& T_world_exchanger) {
        T_world_exchanger_ = T_world_exchanger;
    }

    // 设置 exchanger->entry 的相对位姿（带范围约束的话你在外面先 clamp）
    void setEntryPoseRelativeToExchanger(
        const Eigen::Isometry3d& T_exchanger_entry) {
        if (entry_free_qpos_adr_ < 0) {
            // 没有这个关节就什么都不做
            return;
        }

        Eigen::Isometry3d T_world_entry =
            T_world_exchanger_ * T_exchanger_entry;
        Eigen::Vector3d p = T_world_entry.translation();
        Eigen::Quaterniond q(T_world_entry.linear());
        q.normalize();

        d_->qpos[entry_free_qpos_adr_ + 0] = q.w();
        d_->qpos[entry_free_qpos_adr_ + 1] = q.x();
        d_->qpos[entry_free_qpos_adr_ + 2] = q.y();
        d_->qpos[entry_free_qpos_adr_ + 3] = q.z();
        d_->qpos[entry_free_qpos_adr_ + 4] = p.x();
        d_->qpos[entry_free_qpos_adr_ + 5] = p.y();
        d_->qpos[entry_free_qpos_adr_ + 6] = p.z();

        // 修改完位姿后，做一次 forward，保证一致
        mj_forward(m_, d_);
    }

    // 主循环：一直跑，直到 set_running(false)
    void run(const std::shared_ptr<GlfwRgbViewer> &window_ptr) {
        double frametime = 0.0;
        const double dt_frame = 1.0 / fps_;

        while (running_.load()) {
  
            if (window_ptr && window_ptr->isClosed()) {
                running_.store(false);
                break;
            }
            // 切换到离屏上下文，执行控制与渲染
            glfwMakeContextCurrent(g_window);
            // 1) 先调用控制回调（外部控制机械臂、电机等）
            base_control(m_, d_, d_->time);//先控制底盘
            if (control_callback_) {
                control_callback_(m_, d_, d_->time);
            }

            // 2) 按 fps 渲染
            if ((d_->time - frametime) > dt_frame || frametime == 0.0) {
                // 更新场景
                mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL,
                                &scn_);
                // 渲染到当前 framebuffer
                mjr_render(viewport_, &scn_, &con_);
                // 读出像素（只要 RGB,depth分辨率默认是一样的,额这个无所谓把,反正转化也不是我们写的）
                mjr_readPixels(rgb_buffer_, nullptr, viewport_, &con_);

                // 相机回调
                if (camera_callback_) {
                    camera_callback_(rgb_buffer_,nullptr, width_, height_, d_->time);
                }

                frametime = d_->time;
            }

            // 3) 推进仿真
            mj_step(m_, d_);
        }
        std::cout<<"马上要推出run函数了"<<std::endl;
    }

    // 给外部提供一些只读信息（可选）
    int width() const { return width_; }
    int height() const { return height_; }

    mjModel* model() { return m_; }
    mjData* data() { return d_; }

   private:
    //---------------- MuJoCo 相关 ----------------
    void initMuJoCo() {
        char error[1000] = "Could not load binary model";
        if (model_path_.size() > 4 &&
            model_path_.substr(model_path_.size() - 4) == ".mjb") {
            m_ = mj_loadModel(model_path_.c_str(), nullptr);
        } else {
            m_ = mj_loadXML(model_path_.c_str(), nullptr, error, 1000);
        }
        if (!m_) {
            mju_error("Load model error: %s", error);
        }

        d_ = mj_makeData(m_);
        mj_forward(m_, d_);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, 200);

        // 自由相机 + 一个合适的视角
        mjv_defaultFreeCamera(m_, &cam_);
        cam_.lookat[0] = 0.0;
        cam_.lookat[1] = 0.0;
        cam_.lookat[2] = 1.0;
        cam_.distance = 3.0;
        cam_.elevation = -20.0;
        cam_.azimuth = 90.0;

        // 可选：找原来的 spin_motor，如果模型里没有就直接忽略
        spin_motor_id_ = mj_name2id(m_, mjOBJ_ACTUATOR, "spin_motor");
        if (spin_motor_id_ < 0) {
            std::printf("Info: actuator 'spin_motor' not found, skip.\n");
        }

        // 找兑换站的 free joint：entry_free（如果有）
        int jid = mj_name2id(m_, mjOBJ_JOINT, "entry_free");
        if (jid >= 0) {
            if (m_->jnt_type[jid] != mjJNT_FREE) {
                mju_error("'entry_free' joint is not type free");
            }
            entry_free_qpos_adr_ = m_->jnt_qposadr[jid];
            std::printf("Found entry_free joint, qpos adr = %d\n",
                        entry_free_qpos_adr_);
        } else {
            std::printf(
                "Info: joint 'entry_free' not found, exchanger-entry pose API "
                "disabled.\n");
        }
        // act_wheel_fl_ = mj_name2id(m_, mjOBJ_ACTUATOR, "wheel_fl_vel");
        // act_wheel_fr_ = mj_name2id(m_, mjOBJ_ACTUATOR, "wheel_fr_vel");
        // act_wheel_rl_ = mj_name2id(m_, mjOBJ_ACTUATOR, "wheel_rl_vel");
        // act_wheel_rr_ = mj_name2id(m_, mjOBJ_ACTUATOR, "wheel_rr_vel");
        act_base_x_ = mj_name2id(m_, mjOBJ_ACTUATOR, "base_x_vel");
        act_base_y_ = mj_name2id(m_, mjOBJ_ACTUATOR, "base_y_vel");
        act_base_yaw_ = mj_name2id(m_, mjOBJ_ACTUATOR, "base_yaw_vel");



    }

    void closeMuJoCo() {
        if (d_) {
            mj_deleteData(d_);
            d_ = nullptr;
        }
        if (m_) {
            mj_deleteModel(m_);
            m_ = nullptr;
        }
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
    }

    //---------------- OpenGL 相关 ----------------
    void initOpenGL() {
        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
        }

        glfwWindowHint(GLFW_VISIBLE, 0);
        glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
        g_window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
        if (!g_window) {
            mju_error("Could not create GLFW window");
        }

        glfwMakeContextCurrent(g_window);
    }

    void closeOpenGL() {
        // Linux + NV 驱动有时 glfwTerminate 会崩，这里可以视情况调用

    }
    void setupOffscreenWithCamera(int cam_id) {
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);

        // 从模型里读这个相机的 resolution（2 ints: [w, h]）
        int cam_w = m_->cam_resolution[2 * cam_id + 0];
        int cam_h = m_->cam_resolution[2 * cam_id + 1];

        if (cam_w <= 0 || cam_h <= 0) {
            // 没设置就给个默认
            cam_w = 640;
            cam_h = 480;
        }

        // 调整离屏 framebuffer 大小（MuJoCo 3.x 里有 mjr_resizeOffscreen）
        mjr_resizeOffscreen(cam_w, cam_h, &con_);

        // // 配置 scene 的输出分辨率（重要：在 mjv_updateScene 前设置）
        // scn_.offwidth  = cam_w;
        // scn_.offheight = cam_h;

        // 设置 viewport,他只是一个结构体而以
        viewport_.left   = 0;
        viewport_.bottom = 0;
        viewport_.width  = cam_w;
        viewport_.height = cam_h;

        width_  = cam_w;
        height_ = cam_h;

        // 重新分配 buffer
        if (rgb_buffer_) std::free(rgb_buffer_);
        rgb_buffer_ = (unsigned char*)std::malloc(3 * width_ * height_);

        if (depth_buffer_) std::free(depth_buffer_);
        depth_buffer_ = (float*)std::malloc(width_ * height_ * sizeof(float));

        std::printf("Offscreen viewport: %d x %d\n", width_, height_);
    }
    //---------------- 离屏渲染相关 ----------------
    void setupOffscreen() {
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);
        if (con_.currentBuffer != mjFB_OFFSCREEN) {
            std::printf(
                "Warning: offscreen rendering not fully supported, "
                "using default/window framebuffer\n");
        }

        // viewport_ = mjr_maxViewport(&con_);//这个不好用
        mjr_resizeOffscreen(1920, 1080, &con_);

        viewport_.left   = 0;
        viewport_.bottom = 0;
        viewport_.width  = 1920;
        viewport_.height = 1080;
        width_ = viewport_.width;
        height_ = viewport_.height;

        rgb_buffer_ = (unsigned char*)std::malloc(3 * width_ * height_);
        if (!rgb_buffer_) {
            mju_error("Could not allocate RGB buffer");
        }

        depth_buffer_ = (float*)std::malloc(4*width_*height_);

        std::printf("Viewport: %d x %d\n", width_, height_);
    }

    void cleanup() {
        if (rgb_buffer_) {
            std::free(rgb_buffer_);
            rgb_buffer_ = nullptr;
        }

        if(depth_buffer_){
            std::free(depth_buffer_);
            depth_buffer_ = nullptr;

        }
        closeMuJoCo();
        closeOpenGL();
    }

    void base_control(const mjModel* m, mjData* d, double time) {
        // 麦轮底盘（前进/平移/旋转）
        const double wheel_radius = 0.05;  // 与 XML 里 wheel size 匹配
        const double lx = 0.20;            // 前后到几何中心的距离
        const double ly = 0.30;            // 左右到几何中心的距离
        const double k = lx + ly;          // 旋转半径项

        double vx, vy, wz;
        {
            std::lock_guard<std::mutex> lock(base_mutex_);
            vx = teleop_.vx;
            vy = teleop_.vy;
            wz = teleop_.wz;
        }

        // // 经典麦轮正解（车体坐标系: x前+，y左+，wz左转+）
        // const double w_fl = (vx - vy - k * wz) / wheel_radius;
        // const double w_fr = (vx + vy + k * wz) / wheel_radius;
        // const double w_rl = (vx + vy - k * wz) / wheel_radius;
        // const double w_rr = (vx - vy + k * wz) / wheel_radius;

        // auto clamp_ctrl = [](double v) {
        //     const double limit = 20.0;  // 与 XML ctrlrange 匹配
        //     return std::max(-limit, std::min(limit, v));
        // };

        // if (act_wheel_fl_ >= 0) d->ctrl[act_wheel_fl_] = clamp_ctrl(w_fl);
        // if (act_wheel_fr_ >= 0) d->ctrl[act_wheel_fr_] = clamp_ctrl(w_fr);
        // if (act_wheel_rl_ >= 0) d->ctrl[act_wheel_rl_] = clamp_ctrl(w_rl);
        // if (act_wheel_rr_ >= 0) d->ctrl[act_wheel_rr_] = clamp_ctrl(w_rr);
        auto clamp = [](double v, double lo, double hi) {
            return std::max(lo, std::min(hi, v));
        };

        // 这些限幅应与 XML ctrlrange 匹配
        vx = clamp(vx, -2.0, 2.0);
        vy = clamp(vy, -2.0, 2.0);
        wz = clamp(wz, -6.0, 6.0);

        // std::cout<<"控制id,x:"<<act_base_x_<<" y:"<<act_base_y_<<std::endl;

        if (act_base_x_   >= 0) {
            d->ctrl[act_base_x_]   = vx;
            // std::cout<<"控制x"<<d->ctrl[act_base_x_]<<std::endl;
        }
        if (act_base_y_   >= 0) d->ctrl[act_base_y_]   = vy;
        if (act_base_yaw_ >= 0) d->ctrl[act_base_yaw_] = wz;
    }

    public:
    void update_TeleopState(const double vx ,const double vy ,const double wz){//给变化量
        std::lock_guard<std::mutex> lock(base_mutex_);
        teleop_.vx = vx;
        teleop_.vy = vy;
        teleop_.wz = wz;
    }

   private:
    std::mutex base_mutex_;//底盘控制的锁teleop_是共享的变量
    TeleopState teleop_;//底盘的控制结构体,控制的是速度
    int act_base_x_;
    int act_base_y_;
    int act_base_yaw_;

    // int act_wheel_fl_  ;
    // int act_wheel_fr_ ;
    // int act_wheel_rl_  ;
    // int act_wheel_rr_ ;
    std::atomic<bool> running_;

    // 配置
    std::string model_path_;
    double fps_;

    // MuJoCo 对象
    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;

    mjvScene scn_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext con_;

    // 旧的转塔 motor（可选）
    int spin_motor_id_ = -1;

    // entry_free 关节在 qpos 里的地址（如果存在）
    int entry_free_qpos_adr_ = -1;
    Eigen::Isometry3d T_world_exchanger_ = Eigen::Isometry3d::Identity();

    // 渲染缓冲
    mjrRect viewport_;
    int width_ = 0;
    int height_ = 0;
    unsigned char* rgb_buffer_ = nullptr;
    float* depth_buffer_ = nullptr;


    // 用户回调
    CameraCallback camera_callback_;
    ControlCallback control_callback_;
};


};//hitcrt
