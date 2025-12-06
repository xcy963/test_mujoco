#include "opencv_setinput.hpp"
#include "BIG_porject.hpp"

void test_identity_tf(const std::unique_ptr<hitcrt::NODEWrapper>& wrapper)
{
    // // 构造一个单位变换 Isometry3d（平移=0，旋转=单位）
    // Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // // 循环发一会，方便用 rviz2 或 tf2_tools 看到
    // rclcpp::Rate rate(2.0);  // 2 Hz

    // for (int i = 0; rclcpp::ok() && i < 50; ++i) {
    //     wrapper->sendEigenTransform(T, "openarm_link0", "test_frame");
    //     rate.sleep();
    // }
    while(rclcpp::ok()){
        std::array<double, 7> q;
    
        if (!wrapper->getCurrentJointPositions(q)) {
            // 可能是刚启动，还没收到 joint_states
            RCLCPP_WARN(rclcpp::get_logger("test_fk"),
                        "获取关节角失败，稍后再试");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        Eigen::Isometry3d T = wrapper->kinematics_->forwardKinematics(q,4);
        T.translation() = T.translation() - T.rotation().col(1)*0.0315;
        wrapper->sendEigenTransform(T, "openarm_link0", "P_4ORG");

        Eigen::Isometry3d T_2 = wrapper->kinematics_->forwardKinematics(q);
        T_2 = T.inverse() * T_2;
        T_2.translation() =  T_2.translation() + T_2.rotation().col(2)*0.1001;
        wrapper->sendEigenTransform(T_2, "P_4ORG", "P_8ORG");

    }
}

int main(int argc, char** argv){
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<rclcpp::Node>("ANODE");
    // node->declare_parameter<std::string>("URDF_PATH", "");//一定要先声明才ok

    // auto wrapper = std::make_unique<hitcrt::NODEWrapper>(node);
    // // wrapper->sendEigenTransform(Eigen::Isometry3d::Identity());
    // std::thread th([&](){
    //     test_identity_tf(wrapper);
    // });


    // throw "something";
    hitcrt::DoubleTrackbarController controller("Params");//输入的是窗口名称

    controller.addVariable("joint1_deg", 0.0);
    controller.addVariable("joint2_deg", 30.0);
    controller.addBoolVariable("use_gravity", true);
    controller.addBoolVariable("enable_debug", false);

    controller.initialize();

    while (true) {
        char key = cv::waitKey(30);
        if (key == 'q') break;
        if (key == 'p') controller.printCurrentValues();

        double j1 = controller.getVariable("joint1_deg");
        bool use_g = controller.getBoolVariable("use_gravity");
        // ... 用这些参数做控制/仿真
    }



        // rclcpp::spin(node);
    // rclcpp::shutdown();
}

