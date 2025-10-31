#pragma once
#include <mujoco/mujoco.h>
#include <vector>
#include <iostream>
#include <memory>

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

#include <GLFW/glfw3.h>
#include <condition_variable>  // 添加这行
#include <vector>
#include <iostream>
#include <memory>
#include <cmath>
#include <limits>
#include <cstring>
#include <EGL/egl.h>
#include <cstring>            // 添加这行
#include <limits>             // 添加这行

struct CameraFrame {
    int camera_id;
    uint64_t frame_number;
    std::vector<unsigned char> rgb_data;
    std::vector<float> depth_data;
    int width;
    int height;
    double timestamp;
    
    CameraFrame() : camera_id(-1), frame_number(0), width(0), height(0), timestamp(0.0) {}
};

class CameraRenderer {
private:
    // MuJoCo渲染资源
    mjvScene scn_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext con_;
    
    // OpenGL上下文管理
    GLFWwindow* glfw_window_;
    bool owns_window_;
    
    // 状态标志
    std::atomic<bool> initialized_;
    std::atomic<bool> shutdown_requested_;
    
    // 帧数据同步
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    CameraFrame latest_frame_;
    std::atomic<uint64_t> frame_counter_;
    std::atomic<bool> new_frame_available_;
    
    // 渲染参数
    int render_width_;
    int render_height_;
    bool render_depth_;

public:
    CameraRenderer() 
        : glfw_window_(nullptr)
        , owns_window_(false)
        , initialized_(false)
        , shutdown_requested_(false)
        , frame_counter_(0)
        , new_frame_available_(false)
        , render_width_(640)
        , render_height_(480)
        , render_depth_(true) {
        
        mjv_defaultScene(&scn_);
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjr_defaultContext(&con_);
    }

    ~CameraRenderer() {
        shutdown();
        cleanup();
    }

    // 禁止拷贝
    CameraRenderer(const CameraRenderer&) = delete;
    CameraRenderer& operator=(const CameraRenderer&) = delete;

    /**
     * @brief 初始化相机渲染器
     * @param model MuJoCo模型
     * @param shared_with_window 用于资源共享的主窗口（可选）
     * @param font_scale 字体缩放
     * @return 初始化是否成功
     */
bool initialize(const mjModel* model, GLFWwindow* shared_with_window = nullptr, int font_scale = mjFONTSCALE_150) {
        if (!model) {
            std::cerr << "[CameraRenderer] initialize: model is nullptr\n";
            return false;
        }

        if (initialized_.load()) {
            cleanup();
        }

        // 检查GLFW是否已初始化
        if (!glfwInit()) {
            std::cerr << "[CameraRenderer] GLFW not initialized\n";
            return false;
        }

        // 创建或使用现有的OpenGL上下文
        GLFWwindow* previous_context = glfwGetCurrentContext();
        
        try {
            if (!previous_context && !shared_with_window) {
                // 创建新的GLFW窗口和上下文
                glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
                glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
                
                glfw_window_ = glfwCreateWindow(1, 1, "CameraRenderer", nullptr, nullptr);
                if (!glfw_window_) {
                    std::cerr << "[CameraRenderer] Failed to create GLFW window\n";
                    return false;
                }
                owns_window_ = true;
            } else if (shared_with_window) {
                // 创建共享上下文
                glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
                glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
                glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
                
                glfw_window_ = glfwCreateWindow(1, 1, "CameraRenderer", nullptr, shared_with_window);
                if (!glfw_window_) {
                    std::cerr << "[CameraRenderer] Failed to create shared GLFW window\n";
                    return false;
                }
                owns_window_ = true;
            } else {
                // 使用当前上下文
                glfw_window_ = previous_context;
                owns_window_ = false;
            }

            // 设置为当前上下文
            glfwMakeContextCurrent(glfw_window_);

            // 初始化MuJoCo渲染资源 - 添加错误检查
            mjr_makeContext(model, &con_, font_scale);

            // 重置状态
            shutdown_requested_ = false;
            new_frame_available_ = false;
            frame_counter_ = 0;

            initialized_ = true;
            std::cout << "[CameraRenderer] Initialized successfully with model: " 
                      << (model->names ? model->names : "unnamed") << std::endl;
            
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "[CameraRenderer] Initialization exception: " << e.what() << std::endl;
            cleanup();
            return false;
        }
    }

    /**
     * @brief 清理资源
     */
    void cleanup() {
        if (initialized_) {
            shutdown();
            
            // 释放MuJoCo资源
            mjr_freeContext(&con_);
            mjv_freeScene(&scn_);
            
            // 释放GLFW窗口（如果是我们创建的）
            if (owns_window_ && glfw_window_) {
                glfwDestroyWindow(glfw_window_);
                glfw_window_ = nullptr;
            }
            
            initialized_ = false;
            std::cout << "[CameraRenderer] Cleaned up resources" << std::endl;
        }
    }

    /**
     * @brief 请求关闭渲染器
     */
    void shutdown() {
        shutdown_requested_ = true;
        frame_condition_.notify_all();  // 唤醒所有等待的线程
    }

    /**
     * @brief 渲染一帧并发布
     * @param model MuJoCo模型
     * @param data MuJoCo数据
     * @param camera_id 相机ID
     * @return 渲染是否成功
     */
    int camera_id = 0;

    bool render_frame(const mjModel* model, mjData* data) {
        if (!initialized_.load() || shutdown_requested_.load()) {
            std::cerr << "[CameraRenderer] render_frame: not initialized or shutdown requested\n";
            return false;
        }
        if (!model || !data) {
            std::cerr << "[CameraRenderer] render_frame: null model or data\n";
            return false;
        }
        if (camera_id < 0 || camera_id >= model->ncam) {
            std::cerr << "[CameraRenderer] render_frame: invalid camera id " << camera_id
                    << " (model has " << model->ncam << " cameras)\n";
            return false;
        }
        if (!glfw_window_) {
            std::cerr << "[CameraRenderer] render_frame: no valid GLFW window\n";
            return false;
        }

        // --- 保守检查和调整 scene scale（修复 "scale too small"） ---
        // 如果模型的 extent 极小，使用合理默认（避免 mjv_room2model 错误）
        float extent = model->stat.extent;
        if (extent <= 1e-6f) {
            extent = 1.0f; // fallback：模型没有正确统计 extent 时使用 1.0
        }
        // 保证 scn_.scale 有合理大小（避免 MuJoCo 内部的房间->模型缩放错误）
        if (scn_.scale <= 1e-6f) {
            scn_.scale = extent * 0.3f; // 设为 extent 的一个比例
        }

        // 设置相机
        cam_.type = mjCAMERA_FIXED;
        cam_.fixedcamid = camera_id;

        // 视口
        mjrRect viewport = {0, 0, render_width_, render_height_};

        // 更新场景
        mjv_updateScene(model, data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

        // 渲染
        mjr_render(viewport, &scn_, &con_);

        // 准备缓冲区
        const int px = render_width_ * render_height_;
        std::vector<unsigned char> rgb_buffer(3 * px);
        std::vector<float> tmp_depth;
        std::vector<float> depth_buffer;
        if (render_depth_) {
            tmp_depth.resize(px);
            depth_buffer.resize(px, std::numeric_limits<float>::infinity());
        }

        // 读像素（深度或不深度）
        if (render_depth_) {
            mjr_readPixels(rgb_buffer.data(), tmp_depth.data(), viewport, &con_);
        } else {
            mjr_readPixels(rgb_buffer.data(), nullptr, viewport, &con_);
        }

        // 深度转换（逆深度 -> 线性深度），加入稳健性检查
        if (render_depth_) {
            // 计算 near/far（使用 model->vis.map 和 extent；若不合理使用 fallback）
            float near = model->vis.map.znear * extent;
            float far  = model->vis.map.zfar  * extent;
            if (!(near > 0.0f && far > near + 1e-6f)) {
                // fallback 范围
                near = 0.01f;
                far  = std::max(near + 0.1f, 10.0f);
            }

            for (int i = 0; i < px; ++i) {
                float d = tmp_depth[i];
                // MuJoCo 的深度返回：d in [0,1] 表示深度，1 表示无穷远或不在视锥
                if (d > 0.0f && d < 1.0f) {
                    float denom = far - d * (far - near);
                    if (std::abs(denom) > 1e-8f) {
                        depth_buffer[i] = (near * far) / denom;
                    } else {
                        depth_buffer[i] = std::numeric_limits<float>::infinity();
                    }
                } else {
                    depth_buffer[i] = std::numeric_limits<float>::infinity();
                }
            }
        }
        // std::cout<<"发布一次"<<std::endl;
        // 发布新帧（线程安全）
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_.camera_id = camera_id;
            latest_frame_.frame_number = frame_counter_++;
            latest_frame_.rgb_data = std::move(rgb_buffer);
            if (render_depth_) latest_frame_.depth_data = std::move(depth_buffer);
            else latest_frame_.depth_data.clear();
            latest_frame_.width = render_width_;
            latest_frame_.height = render_height_;
            latest_frame_.timestamp = glfwGetTime();
            new_frame_available_ = true;
        }
        frame_condition_.notify_all();

        return true;
    }


    /**
     * @brief 等待新帧
     * @param frame 输出帧数据
     * @param timeout_ms 超时时间（毫秒）
     * @return 是否成功获取新帧
     */
    bool wait_for_frame(CameraFrame& frame, int timeout_ms = 1000) {
        if (shutdown_requested_) {
            return false;
        }

        std::unique_lock<std::mutex> lock(frame_mutex_);
        
        // 等待新帧或超时
        bool success = frame_condition_.wait_for(lock, 
            std::chrono::milliseconds(timeout_ms),
            [this]() { return new_frame_available_ || shutdown_requested_; });

        if (success && new_frame_available_ && !shutdown_requested_) {
            // 复制帧数据
            frame = latest_frame_;
            new_frame_available_ = false;
            return true;
        }

        return false;
    }

    /**
     * @brief 尝试获取最新帧（非阻塞）
     * @param frame 输出帧数据
     * @return 是否成功获取帧
     */
    bool try_get_frame(CameraFrame& frame) {
        if (!new_frame_available_ || shutdown_requested_) {
            return false;
        }

        std::lock_guard<std::mutex> lock(frame_mutex_);
        
        if (new_frame_available_) {
            frame = latest_frame_;
            new_frame_available_ = false;
            return true;
        }

        return false;
    }

    /**
     * @brief 设置渲染分辨率
     */
    void set_resolution(int width, int height) {
        render_width_ = width;
        render_height_ = height;
    }

    /**
     * @brief 设置是否渲染深度图
     */
    void set_render_depth(bool render_depth) {
        render_depth_ = render_depth;
    }

    /**
     * @brief 列出所有可用相机
     */
    void list_cameras(const mjModel* model) {
        if (!model) {
            std::cout << "[CameraRenderer] list_cameras: model == nullptr\n";
            return;
        }
        std::cout << "Available cameras (" << model->ncam << "):\n";
        for (int i = 0; i < model->ncam; ++i) {
            const char* name = mj_id2name(model, mjOBJ_CAMERA, i);
            std::cout << "  Camera " << i << ": " << (name ? name : "unnamed")
                      << " (fovy: " << model->cam_fovy[i] << "°)\n";
        }
    }

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { 
        return initialized_; 
    }

    /**
     * @brief 检查是否请求关闭
     */
    bool is_shutdown_requested() const { 
        return shutdown_requested_; 
    }

    /**
     * @brief 获取帧计数器
     */
    uint64_t get_frame_count() const { 
        return frame_counter_; 
    }

private:
    /**
     * @brief 转换深度缓冲区为线性深度
     */
    void convertDepthBuffer(const mjModel* model, int camera_id,
                           const std::vector<float>& depth_image,
                           std::vector<float>& depth_buffer,
                           int width, int height) {
        if (!model) return;

        // 获取近远平面距离
        float near = model->vis.map.znear * model->stat.extent;
        float far = model->vis.map.zfar * model->stat.extent;

        for (int i = 0; i < width * height; ++i) {
            float d = depth_image[i];
            if (d < 1.0f) {
                // 逆深度映射转换
                depth_buffer[i] = near * far / (far - d * (far - near));
            } else {
                depth_buffer[i] = std::numeric_limits<float>::infinity();
            }
        }
    }
};