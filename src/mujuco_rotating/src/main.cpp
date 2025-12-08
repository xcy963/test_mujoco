#include "mujoco_tower_class.hpp"   // 或者做成 .h + .cpp 分离
#include <rclcpp/rclcpp.hpp>
#include "opencv_input.hpp"
#include <chrono>

#include "recall_process.hpp"

void process(const FrameQueue &frame_queue){

}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujuco_rotate");
    node->declare_parameter<std::string>("model_path", "");
    std::string model_path = node->get_parameter("model_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(),"使用的模型文件是"<<model_path);

    // hitcrt::MujocoOffscreenRenderer renderer();
    auto renderer = std::make_shared<hitcrt::MujocoOffscreenRenderer>(model_path, 60.0);
    renderer->useFixedCamera("arm_cam");
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
    cv::namedWindow("MuJoCo Camera");//保证这个窗口有焦点
    auto last_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();

    FrameQueue frame_queue;//相机回调处理
    std::atomic<bool> running_disp{true};
    std::thread display_thread([&]() {
        cv::namedWindow("MuJoCo Camera");

        auto last_time = std::chrono::steady_clock::now();
        Frame frame;

        while (running_disp.load()) {
            if (!frame_queue.pop(frame)) {
                break;  // 队列关闭
            }

            if (frame.rgb.empty()) continue;

            int width = frame.width;
            int height = frame.height;

            cv::Mat img(height, width, CV_8UC3);

            // 把 RGB（倒着的）拷到 OpenCV BGR
            for (int r = 0; r < height; ++r) {
                const unsigned char* src_row =
                    frame.rgb.data() + (height - 1 - r) * width * 3;
                cv::Vec3b* dst_row = img.ptr<cv::Vec3b>(r);
                for (int c = 0; c < width; ++c) {
                    unsigned char R = src_row[3 * c + 0];
                    unsigned char G = src_row[3 * c + 1];
                    unsigned char B = src_row[3 * c + 2];
                    dst_row[c][0] = B;
                    dst_row[c][1] = G;
                    dst_row[c][2] = R;
                }
            }

            // 统计 FPS（显示线程自己的 FPS）
            auto now_time = std::chrono::steady_clock::now();
            double dt =
                std::chrono::duration<double>(now_time - last_time).count();
            last_time = now_time;
            double fps = 1.0 / dt;

            char fpsText[64];
            std::snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", fps);

            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.7;
            int thickness = 2;
            int baseline = 0;

            cv::Size textSize = cv::getTextSize(fpsText, fontFace, fontScale,
                                                thickness, &baseline);
            cv::Point org(width - textSize.width - 10, textSize.height + 10);

            cv::putText(img, fpsText, org + cv::Point(1, 1), fontFace,
                        fontScale, cv::Scalar(0, 0, 0), thickness + 1,
                        cv::LINE_AA);
            cv::putText(img, fpsText, org, fontFace, fontScale,
                        cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);

            cv::imshow("MuJoCo Camera", img);
            char ch = cv::waitKey(1);

            if (ch == 'q') {
                renderer->set_running(false);  // 通知渲染线程停
                running_disp.store(false);     // 自己也准备退
            } else if (ch == 'w') {
                renderer->update_TeleopState(-1, 0);
            } else if (ch == 's') {
                renderer->update_TeleopState(1, 0);
            } else if (ch == 'a') {
                renderer->update_TeleopState(0, -1);
            } else if (ch == 'd') {
                renderer->update_TeleopState(0, 1);
            } else {
                renderer->update_TeleopState(0, 0);
            }
        }
    });

    // ===== 设置 camera 回调：只丢数据，不显示 =====
    renderer->setCameraCallback(
        [&](const unsigned char* rgb, const float* depth, int width,
            int height, double t) {

            // 拷到自己的 buffer 里（注意：renderer 的 rgb_buffer_ 会复用）
            Frame f;
            f.width = width;
            f.height = height;
            f.t = t;
            f.rgb.resize(width * height * 3);
            std::memcpy(f.rgb.data(), rgb, width * height * 3);

            frame_queue.push(std::move(f));
        }
    );


    renderer->setControlCallback(
        [&](mjModel* m, mjData* d, double sim_time) {
            // // 1) 从你的控制器里拿到当前关节目标（假设是“角度 / 度”）
            // double j1_deg = controller.getVariable("joint1_deg");
            // double j2_deg = controller.getVariable("joint2_deg");
            // double j3_deg = controller.getVariable("joint3_deg");
            // double j4_deg = controller.getVariable("joint4_deg");
            // double j6_deg = controller.getVariable("joint6_deg");
            // double j7_deg = controller.getVariable("joint7_deg");

            // // 2) 度 -> 弧度
            // auto deg2rad = [](double x) { return x * M_PI / 180.0; };

            // // 3) 一个小工具函数：按关节名字写 qpos
            // auto setJointPosByName = [&](const char* jname, double q_rad) {
            //     int jid = mj_name2id(m, mjOBJ_JOINT, jname);
            //     if (jid < 0) {
            //         // 找不到这个关节就直接跳过
            //         return;
            //     }
            //     int qadr = m->jnt_qposadr[jid];
            //     // hinge / slide 关节在 qpos 中只占 1 个元素，这里直接赋值就行 
            //     d->qpos[qadr] = q_rad;
            // };

            // // 4) 把当前目标角度写到各个关节里
            // setJointPosByName("Joint1", deg2rad(j1_deg));
            // setJointPosByName("Joint2", deg2rad(j2_deg));
            // setJointPosByName("Joint3", deg2rad(j3_deg));
            // setJointPosByName("Joint4", deg2rad(j4_deg));
            // setJointPosByName("Joint6", deg2rad(j6_deg));
            // setJointPosByName("Joint7", deg2rad(j7_deg));

        }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::cout << "MuJoCo ROS node started. Press Ctrl+C to exit." << std::endl;
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    renderer->run();
    running_disp.store(false);
    display_thread.join();
    // node.
}
