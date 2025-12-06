#include "mujoco_tower_class.hpp"   // 或者做成 .h + .cpp 分离
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujuco_rotate");
    node->declare_parameter<std::string>("model_path", "");
    std::string model_path = node->get_parameter("model_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(),"使用的模型文件是"<<model_path);


    MujocoOffscreenRenderer renderer(model_path, 10.0, 30.0);

    renderer.setCameraCallback([&](const unsigned char* rgb, int width,
                                  int height, double t) {
        // 这个回调就是你之前的 cameraCallback：显示 OpenCV 窗口

        cv::Mat img(height, width, CV_8UC3);

        for (int r = 0; r < height; ++r) {
            const unsigned char* src_row = rgb + (height - 1 - r) * width * 3;
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

        cv::imshow("MuJoCo Camera", img);
        char ch =  cv::waitKey(1);
        if(ch =='q'){
          renderer.set_runnning(false);
        }

    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::cout << "MuJoCo ROS node started. Press Ctrl+C to exit." << std::endl;
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    renderer.run();
    // node.
}
