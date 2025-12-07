// mujoco_tower_class.cpp
// 把 MuJoCo 离屏渲染封装成一个类，支持：
// 1) 相机回调（OpenCV 显示/录制）；
// 2) 控制回调（外部控制机械臂电机 d->ctrl[]）；
// 3) 兑换站 entry_free 位姿接口（使用 Eigen）。
#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
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

//============================= 类定义 ======================================

class MujocoOffscreenRenderer {
   public:
    // 相机回调：给 RGB 图像 + 宽高 + 仿真时间
    using CameraCallback =
        std::function<void(const unsigned char*,const float*, int, int, double)>;
    // 控制回调：每个仿真步调用一次，你在里面写 d->ctrl / 改 qpos 等
    using ControlCallback = std::function<void(mjModel*, mjData*, double)>;

    MujocoOffscreenRenderer(const std::string& model_path,
                            double /*duration_sec*/, double fps)
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
    void run() {
        double frametime = 0.0;
        const double dt_frame = 1.0 / fps_;

        while (running_.load()) {
            // 1) 先调用控制回调（外部控制机械臂、电机等）
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
                mjr_readPixels(rgb_buffer_, depth_buffer_, viewport_, &con_);

                // 相机回调
                if (camera_callback_) {
                    camera_callback_(rgb_buffer_,depth_buffer_, width_, height_, d_->time);
                }

                frametime = d_->time;
            }

            // 3) 推进仿真
            mj_step(m_, d_);
        }
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

        viewport_ = mjr_maxViewport(&con_);//这个不好用
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

   private:
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

// //==================== 一个简单的 OpenCV demo 回调 ===================

// static void OpenCVCameraCallback(const unsigned char* rgb,
//                                  int width, int height, double /*sim_time*/)
// {
//   cv::Mat img(height, width, CV_8UC3);

//   for (int r = 0; r < height; ++r) {
//     const unsigned char* src_row = rgb + (height - 1 - r) * width * 3;
//     cv::Vec3b* dst_row = img.ptr<cv::Vec3b>(r);
//     for (int c = 0; c < width; ++c) {
//       unsigned char R = src_row[3*c + 0];
//       unsigned char G = src_row[3*c + 1];
//       unsigned char B = src_row[3*c + 2];
//       dst_row[c][0] = B;
//       dst_row[c][1] = G;
//       dst_row[c][2] = R;
//     }
//   }

//   cv::imshow("MuJoCo Camera", img);
//   cv::waitKey(1);
// }
