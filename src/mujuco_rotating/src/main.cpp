#include "mujoco_tower_class.hpp"   // 或者做成 .h + .cpp 分离

int main(int argc, char** argv) {
    // ros::init(argc, argv, "mujoco_node");

    MujocoOffscreenRenderer renderer("/home/hitcrt/enginner_26/test_mujoco_ros/src/mujuco_rotating/engineer_module/demo.xml", 10.0, 30.0);

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

    renderer.run();
}
