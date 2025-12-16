#include "mujoco_tower_class.hpp"   // 或者做成 .h + .cpp 分离
#include <rclcpp/rclcpp.hpp>
#include "opencv_input.hpp"
#include <chrono>
#include <thread>
#include <atomic>

#include "kinematics.hpp"

// #include "window_control.hpp"
// void process(const FrameQueue &frame_queue){

// }


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto viewer = std::make_shared<GlfwRgbViewer>(); 
    if(!viewer->init(1920,1080,"LOOK IN MY EYE")){
        return -1;
    }
    // viewer->setInputCallback([](const InputState& in) {
    //     if (in.w || in.a || in.s || in.d) {
    //     std::cout << "WASD: " << in.w << in.a << in.s << in.d << "\n";
    //     }
    //     if (in.dx != 0.0 || in.dy != 0.0) {
    //     std::cout << "Mouse dx=" << in.dx << " dy=" << in.dy << "\n";
    //     }
    //     if (in.flickLeft)  std::cout << "Flick LEFT\n";
    //     if (in.flickRight) std::cout << "Flick RIGHT\n";
    //     if (in.flickUp)    std::cout << "Flick UP\n";
    //     if (in.flickDown)  std::cout << "Flick DOWN\n";
    // });

    std::atomic<bool> app_running{true};

    auto node = std::make_shared<rclcpp::Node>("mujuco_rotate");
    node->declare_parameter<std::string>("model_path", "");
    std::string model_path = node->get_parameter("model_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(),"使用的模型文件是"<<model_path);

    // hitcrt::MujocoOffscreenRenderer renderer();
    auto renderer = std::make_shared<hitcrt::MujocoOffscreenRenderer>(model_path, 60.0);
    renderer->useFixedCamera("arm_cam");
    // 键鼠 → 麦轮速度
    const double linear_speed = 0.8;    // m/s
    const double strafe_speed = 0.8;    // m/s
    const double yaw_per_px   = 0.02;   // rad per mouse dx 像素,调节灵敏度
    viewer->setInputCallback(
        [renderer, linear_speed, strafe_speed, yaw_per_px](const InputState& in) {
            double vx = 0.0, vy = 0.0, wz = 0.0;
            if (in.w) vx += linear_speed;
            if (in.s) vx -= linear_speed;
            if (in.a) vy += strafe_speed;   // 左平移
            if (in.d) vy -= strafe_speed;   // 右平移

            wz = -in.dx * yaw_per_px;       // 鼠标左/右转身
            renderer->update_TeleopState(vx, vy, wz);
        });
    // 把上下文交给渲染线程
    viewer->detachContext();
    std::thread viewer_thread([&viewer, &app_running,&renderer](){
        viewer->loop(&app_running);
        renderer->set_running(false);

    });
    //调参工具
    // hitcrt::DoubleTrackbarController controller("Params");//输入的是窗口名称
    // controller.addVariable("joint1_deg", 0.0);
    // controller.addVariable("joint2_deg", 30.0);
    // controller.addVariable("joint3_deg", 30.0);

    // controller.addVariable("joint4_deg", 30.0);
    // controller.addVariable("joint6_deg", 30.0);
    // controller.addVariable("joint7_deg", 30.0);

    // controller.addBoolVariable("use_gravity", true);
    // controller.addBoolVariable("enable_debug", false);
    // controller.initialize();
    // cv::namedWindow("MuJoCo Camera");//保证这个窗口有焦点
    // auto last_time = std::chrono::steady_clock::now();
    // auto now_time = std::chrono::steady_clock::now();

 
    // ===== 设置 camera 回调：只丢数据，不显示 =====
    renderer->setCameraCallback(
        [&](const unsigned char* rgb, const float* depth, int width,
            int height, double t) {
            viewer->pushFrame(rgb, width, height);

        }
    );
    // std::thread viewer_thread([&viewer](){//opengl的所有东西需要在同一个线程里面
    //     viewer->loop();
    // });

    int some_index = 0;
    int direction  = 1;   
    renderer->setControlCallback(
        [&](mjModel* m, mjData* d, double sim_time) {

            }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::cout << "MuJoCo ROS node started. Press Ctrl+C to exit." << std::endl;
    std::thread executor_thread([&executor]() {
        executor.spin();
    });


    renderer->run(viewer);

    app_running.store(false);
    // if (executor.get_node_base_interface()) executor.cancel();
    if (viewer_thread.joinable()) viewer_thread.join();
    viewer->shutdown();
    rclcpp::shutdown();
    if (executor_thread.joinable()) executor_thread.join();
    return 0;
}
