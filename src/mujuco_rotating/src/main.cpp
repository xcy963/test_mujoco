#include "mujoco_tower_class.hpp"   // 或者做成 .h + .cpp 分离
#include <rclcpp/rclcpp.hpp>
#include "opencv_input.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujuco_rotate");
    node->declare_parameter<std::string>("model_path", "");
    std::string model_path = node->get_parameter("model_path").as_string();
    RCLCPP_INFO_STREAM(node->get_logger(),"使用的模型文件是"<<model_path);

    MujocoOffscreenRenderer renderer(model_path, 10.0, 30.0);
    renderer.useFixedCamera("arm_cam");
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

    renderer.setCameraCallback([&](const unsigned char* rgb,const float*depth, int width,
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
        // double j1 = controller.getVariable("joint1_deg");
        // bool use_g = controller.getBoolVariable("use_gravity");
        // print()
        // std::cout<<"获取到的角度值:"<<j1<<std::endl;

        cv::imshow("MuJoCo Camera", img);
        char ch =  cv::waitKey(1);
        if(ch =='q'){
          renderer.set_running(false);
        }

    });
    // renderer.setControlCallback(
    //     [&](mjModel* m, mjData* d, double sim_time) {
    //         // 1) 从你的控制器里拿到当前关节目标（假设是“角度 / 度”）
    //         double j1_deg = controller.getVariable("joint1_deg");
    //         double j2_deg = controller.getVariable("joint2_deg");
    //         double j3_deg = controller.getVariable("joint3_deg");
    //         double j4_deg = controller.getVariable("joint4_deg");
    //         double j6_deg = controller.getVariable("joint6_deg");
    //         double j7_deg = controller.getVariable("joint7_deg");

    //         // 2) 度 -> 弧度
    //         auto deg2rad = [](double x) { return x * M_PI / 180.0; };

    //         // 3) 一个小工具函数：按关节名字写 qpos
    //         auto setJointPosByName = [&](const char* jname, double q_rad) {
    //             int jid = mj_name2id(m, mjOBJ_JOINT, jname);
    //             if (jid < 0) {
    //                 // 找不到这个关节就直接跳过
    //                 return;
    //             }
    //             int qadr = m->jnt_qposadr[jid];
    //             // hinge / slide 关节在 qpos 中只占 1 个元素，这里直接赋值就行 
    //             d->qpos[qadr] = q_rad;
    //         };

    //         // 4) 把当前目标角度写到各个关节里
    //         setJointPosByName("Joint1", deg2rad(j1_deg));
    //         setJointPosByName("Joint2", deg2rad(j2_deg));
    //         setJointPosByName("Joint3", deg2rad(j3_deg));
    //         setJointPosByName("Joint4", deg2rad(j4_deg));
    //         setJointPosByName("Joint6", deg2rad(j6_deg));
    //         setJointPosByName("Joint7", deg2rad(j7_deg));

    //         // 如果你只是想做“随时间刷新姿态”的 kinematic 动画，这样就够了。
    //         // 不需要在这里 mj_step；外面的 run() 里已经会 mj_step(m, d)。
    //     }
    // );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // std::cout << "MuJoCo ROS node started. Press Ctrl+C to exit." << std::endl;
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    renderer.run();
    // node.
}
