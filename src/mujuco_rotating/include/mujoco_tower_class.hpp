// mujoco_tower_class.cpp
// 把 MuJoCo 离屏渲染封装成一个类，用户只需设置回调然后 run() 即可。

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <functional>
#include <string>

#include <mujoco/mujoco.h>
#include <atomic>

#include <GLFW/glfw3.h>
GLFWwindow* g_window = nullptr;

#include <opencv2/opencv.hpp>

//============================= 类定义 ======================================

class MujocoOffscreenRenderer {
public:
  // 回调类型：给你 RGB 图像 + 宽高 + 仿真时间
  using CameraCallback = std::function<void(const unsigned char*, int, int, double)>;

  MujocoOffscreenRenderer(const std::string& model_path,
                          double duration_sec,
                          double fps)
    : model_path_(model_path),
    //   duration_(duration_sec),
      fps_(fps)
  {
    runnning_.store(true);
    initOpenGL();
    initMuJoCo();
    setupOffscreen();
  }

  ~MujocoOffscreenRenderer() {
    cleanup();
  }
  void set_runnning(const bool &flag){
    runnning_.store(false);
  }

  // 设置用户的相机回调
  void setCameraCallback(CameraCallback cb) {
    camera_callback_ = std::move(cb);
  }

  // 主循环：跑到 duration_ 秒结束
  void run() {
    double frametime = 0.0;
    const double dt_frame = 1.0 / fps_;

    while (runnning_.load()) {
      // 控制逻辑：给 spin_motor 一个恒定角速度（rad/s）
      if (spin_motor_id_ >= 0) {
        double target_vel = 2.0;
        d_->ctrl[spin_motor_id_] = target_vel;
      }

      // 按 fps 渲染
      if ((d_->time - frametime) > dt_frame || frametime == 0.0) {
        // 更新场景
        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

        // 渲染到当前 framebuffer
        mjr_render(viewport_, &scn_, &con_);

        // 读出像素（只要 RGB，不要 depth）
        mjr_readPixels(rgb_buffer_, nullptr, viewport_, &con_);

        // 调用用户回调
        if (camera_callback_) {
          camera_callback_(rgb_buffer_, width_, height_, d_->time);
        }

        frametime = d_->time;
      }

      // 推进仿真
      mj_step(m_, d_);
    }
  }

  // 给外部提供一些只读信息（可选）
  int width()  const { return width_; }
  int height() const { return height_; }

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
    cam_.distance  = 3.0;
    cam_.elevation = -20.0;
    cam_.azimuth   = 90.0;

    // 找 actuator "spin_motor"
    spin_motor_id_ = mj_name2id(m_, mjOBJ_ACTUATOR, "spin_motor");
    if (spin_motor_id_ < 0) {
      mju_error("Cannot find actuator 'spin_motor' in model.");
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

  }

  //---------------- 离屏渲染相关 ----------------
  void setupOffscreen() {
    // 使用离屏 framebuffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    if (con_.currentBuffer != mjFB_OFFSCREEN) {
      std::printf("Warning: offscreen rendering not fully supported, "
                  "using default/window framebuffer\n");
    }

    viewport_ = mjr_maxViewport(&con_);
    width_  = viewport_.width;
    height_ = viewport_.height;

    rgb_buffer_ = (unsigned char*)std::malloc(3 * width_ * height_);
    if (!rgb_buffer_) {
      mju_error("Could not allocate RGB buffer");
    }

    std::printf("Viewport: %d x %d\n", width_, height_);
  }

  void cleanup() {
    if (rgb_buffer_) {
      std::free(rgb_buffer_);
      rgb_buffer_ = nullptr;
    }
    closeMuJoCo();
    closeOpenGL();
  }

private:
  std::atomic<bool> runnning_;
  // 配置
  std::string model_path_;
//   double duration_;
  double fps_;

  // MuJoCo 对象
  mjModel* m_ = nullptr;
  mjData*  d_ = nullptr;

  mjvScene   scn_;
  mjvCamera  cam_;
  mjvOption  opt_;
  mjrContext con_;

  int spin_motor_id_ = -1;

  // 渲染缓冲
  mjrRect       viewport_;
  int           width_  = 0;
  int           height_ = 0;
  unsigned char* rgb_buffer_ = nullptr;

  // 用户回调
  CameraCallback camera_callback_;
};


//==================== （可选）一个简单的 OpenCV demo 回调 ===================

// 这个回调就是你之前的 cameraCallback：显示 OpenCV 窗口
static void OpenCVCameraCallback(const unsigned char* rgb,
                                 int width, int height, double sim_time)
{
  (void)sim_time;  // 如果暂时不用时间，避免编译 warning

  cv::Mat img(height, width, CV_8UC3);

  for (int r = 0; r < height; ++r) {
    const unsigned char* src_row = rgb + (height - 1 - r) * width * 3;
    cv::Vec3b* dst_row = img.ptr<cv::Vec3b>(r);
    for (int c = 0; c < width; ++c) {
      unsigned char R = src_row[3*c + 0];
      unsigned char G = src_row[3*c + 1];
      unsigned char B = src_row[3*c + 2];
      dst_row[c][0] = B;
      dst_row[c][1] = G;
      dst_row[c][2] = R;
    }
  }

  cv::imshow("MuJoCo Camera", img);
  cv::waitKey(1);
}


// //==================== （可选）示例 main ====================
// // 你在 ROS 里用的话，可以不要下面这个 main，自己 new 类 & run()。

// int main() {
//   const char* model_file =
//     "/home/hitcrt/enginner_26/test_mujoco_ros/src/mujuco_rotating/engineer_module/demo.xml";

//   MujocoOffscreenRenderer renderer(model_file, /*duration*/10.0, /*fps*/30.0);

//   // 设置回调（你也可以传自己的 lambda / 函数）
//   renderer.setCameraCallback(OpenCVCameraCallback);

//   // 开始跑
//   renderer.run();

//   return 0;
// }
