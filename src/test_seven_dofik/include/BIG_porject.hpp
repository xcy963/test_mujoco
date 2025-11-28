#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>  // eigenToTransform / transformToEigen
#include <vector>

#include <memory>

#include<fstream>

//订阅角度
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <algorithm>  // std::find


namespace hitcrt{
const std::vector<std::string> joint_names = {
    "openarm_joint1",
    "openarm_joint2",
    "openarm_joint3",
    "openarm_joint4",
    "openarm_joint5",
    "openarm_joint6",
    "openarm_joint7"};

double solve_triangle(const double &a, const double &b, const double &c) {
    // 余弦定理：c^2 = a^2 + b^2 - 2ab cos(theta)
    double cos_sita = (a * a + b * b - c * c) / (2.0 * a * b);

    // 数值误差可能导致 cos_sita 稍微超过 [-1, 1]，要压一下
    cos_sita = std::clamp(cos_sita, -1.0, 1.0);

    // acos 的返回范围是 [0, π]，刚好对应三角形内角
    double sita = std::acos(cos_sita);
    return sita;
}


// 这个暂时的运动学类：
// 1. 用给定的 URDF 字符串和关节链 joint_names 构造；
// 2. 把每个关节的 origin(父 link -> 关节坐标系) 转成 Eigen::Isometry3d 存下来；
// 3. forwardKinematics() 输入 7 个关节位置，输出末端位姿。
// TODO 完成逆运动学
class UrdfKinematics {
   public:
    struct JointKinematics {
        std::string name;
        std::string parent_link;
        std::string child_link;

        // 父 link -> joint frame（q=0 时的固定变换）
        Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();

        // 关节轴（在 joint frame 下）
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

        // 关节类型：直接用 urdf::Joint::type 的 int 值
        int type = urdf::Joint::UNKNOWN;
    };

    /// 使用 URDF XML 字符串和正运动学链上的关节名字构造
    UrdfKinematics(const std::string& urdf_xml,
                   const std::vector<std::string>& joint_names_chain) {
        if (!model_.initString(urdf_xml)) {
            throw std::runtime_error("Failed to parse URDF from string.");
        }

        joints_.reserve(joint_names_chain.size());
        for (const auto& jn : joint_names_chain) {
            // std::cout<<"尝试获取"<<jn<<std::endl;
            auto j = model_.getJoint(jn);
            if (!j) {
                throw std::runtime_error("Joint " + jn + " not found in URDF.");
            }

            JointKinematics jk;
            jk.name = j->name;
            jk.type = j->type;
            jk.parent_link = j->parent_link_name;
            jk.child_link = j->child_link_name;

            // origin: parent link -> joint frame
            jk.origin = Eigen::Isometry3d::Identity();
            const urdf::Pose& p = j->parent_to_joint_origin_transform;

            // translation
            jk.origin.translation() =
                Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
            // std::cout<<"他对应的矩阵是\n"<<jk.origin.matrix()<<std::endl;
            // rotation from RPY
            double roll, pitch, yaw;
            p.rotation.getRPY(roll, pitch, yaw);
            jk.origin.linear() = rpyToRotationMatrix(roll, pitch, yaw);
            // jk.origin.linear() = rpyToRotationMatrix(roll, pitch, yaw);

            // joint axis
            if (j->type != urdf::Joint::FIXED) {
                jk.axis = Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
                if (jk.axis.norm() == 0.0) {
                    jk.axis = Eigen::Vector3d::UnitZ();
                }
                jk.axis.normalize();
            } else {
                jk.axis = Eigen::Vector3d::Zero();
            }

            joints_.push_back(jk);
        }

        if (joints_.size() != 7) {
            // 如果不是 7 个，也可以用 vector 版本的接口，这里只是提醒一下
            // 你可以根据自己需求改掉这个检查
            // throw std::runtime_error("Expected 7 joints in chain.");
        }
        // length_l1 = joints_[3].origin.translation().z();
        // length_l1 = joints_[6].origin.translation().z() - joints_[3].origin.translation().z();;
    }

    /// 输入 7 个关节角，返回末端位姿
    Eigen::Isometry3d forwardKinematics(const std::array<double, 7>& q,int last = 7) const {
        if (joints_.size() != 7) {
            throw std::runtime_error(
                "Joint chain size != 7, use the vector overload instead.");
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        for (size_t i = 0; i < last; ++i) {
            T = T * jointTransform(joints_[i], q[i]);
        }

        return T;
    }

    /// 更通用：输入任意长度的关节向量，对应构造时的 joint_names_chain 顺序
    Eigen::Isometry3d forwardKinematics(const std::vector<double>& q) const {
        if (q.size() != joints_.size()) {
            throw std::runtime_error(
                "Size of q does not match number of joints in chain.");
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        for (size_t i = 0; i < joints_.size(); ++i) {
            T = T * jointTransform(joints_[i], q[i]);
        }

        return T;
    }

    /// 获取某个关节（在 q=0 时）的固定变换：父 link -> joint frame
    const JointKinematics& getJointInfo(size_t idx) const {
        if (idx >= joints_.size())
            throw std::out_of_range("Joint index out of range");
        return joints_[idx];
    }

    const std::vector<JointKinematics>& getJoints() const { return joints_; }


    bool IK_fromL7(const Eigen::Isometry3d &T_end,const double angle,std::vector<double> &res){
        Eigen::Vector3d P_7ORG = T_end.translation();
        Eigen::Vector3d P_4ORG;
        bool ok = get_P_4ORG(P_7ORG,angle, P_4ORG);
        if(!ok){
            return false;
        }
        double temp  = std::acos(P_4ORG.z()/P_4ORG.norm());
        if(std::abs(temp) < 1e-5 ){//平行的奇异
            std::cout<<"1号奇异现在不解"<<std::endl;
            return false;//TODO把这个改成能解的
        }

        //不奇异,解前两个
        double Joint1 = std::atan2(-P_4ORG.x(),P_4ORG.y());
        double Joint2;
        {
            Eigen::Vector3d temp;
            temp<< -std::sin(Joint1),std::cos(Joint1),0;//旋转之后的L1的Y轴
            double cos_J2 = P_4ORG.dot(Eigen::Vector3d::UnitZ());
            double sin_J2 = P_4ORG.dot(temp);
            Joint2 = std::atan2(sin_J2,cos_J2);
        }
        


    }

   private:
    urdf::Model model_;
    std::vector<JointKinematics> joints_;

    const double length_l1 = 0.3425;
    const double length_l2 = 0.2160;

    static Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch,
                                               double yaw) {
        // Z-Y-X (yaw-pitch-roll)：与 URDF 约定一致
        Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();
        return R;
    }

    /// 计算单个关节在给定关节位置 q_i 时的变换：父 link -> 子 link
    Eigen::Isometry3d jointTransform(const JointKinematics& jk,
                                     double q_i) const {
        Eigen::Isometry3d T = jk.origin;  // 先乘 origin

        switch (jk.type) {
            case urdf::Joint::REVOLUTE:
            case urdf::Joint::CONTINUOUS: {
                Eigen::AngleAxisd aa(q_i, jk.axis);
                Eigen::Isometry3d R = Eigen::Isometry3d::Identity();
                R.linear() = aa.toRotationMatrix();
                T = T * R;
                // std::cout<<"我发现一个矩阵\n"<<T.matrix()<<std::endl;
                break;
            }
            case urdf::Joint::PRISMATIC: {
                Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
                trans.translation() = jk.axis * q_i;
                T = T * trans;
                break;
            }
            case urdf::Joint::FIXED:
            default:
                // fixed joint: no extra transform beyond origin
                break;
        }

        return T;
    }

    bool get_P_4ORG(const Eigen::Vector3d & P_7ORG,const double &sita,Eigen::Vector3d &P_4ORG){
        return true; 
    }
};


class NODEWrapper {
   public:
    explicit NODEWrapper(const rclcpp::Node::SharedPtr& node)
        : node_(node),
        tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)) {
                    // 获取 URDF 字符串
            std::string urdf_PATH;
            if (!node_->get_parameter("URDF_PATH", urdf_PATH)){
                RCLCPP_ERROR(node_->get_logger(), "URDF_PATH parameter not set");
                throw std::runtime_error("URDF_PATH parameter not set");
            }
            RCLCPP_INFO_STREAM(node_->get_logger(),"使用的路径: "<<urdf_PATH);

            std::ifstream ifs(urdf_PATH);
            if (!ifs.is_open()) {
                RCLCPP_ERROR_STREAM(node_->get_logger(), "文件打不开,目录是: "<<urdf_PATH);
                throw std::runtime_error("Failed to open URDF file");
            }
            std::stringstream buffer;
            buffer << ifs.rdbuf();
            std::string urdf_xml = buffer.str();
            // 这里填你真正参与正运动学的 7 个关节名（按从 base 到末端顺序）
            // std::vector<std::string> joint_names = {
            //     "openarm_joint1",
            //     "openarm_joint2",
            //     "openarm_joint3",
            //     "openarm_joint4",
            //     "openarm_joint5",
            //     "openarm_joint6",
            //     "openarm_joint7"};

            kinematics_ = std::make_unique<UrdfKinematics>(urdf_xml, joint_names);

            // 举个例子：给一组关节角，计算末端位姿
            // std::array<double, 7> q = {0.0, 0.1, -0.2, 0.3, -0.1, 0.2, 0.0};
            std::array<double, 7> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


            Eigen::Isometry3d T = kinematics_->forwardKinematics(q);

            Eigen::Vector3d t = T.translation();
            Eigen::Quaterniond q_end(T.rotation());
            RCLCPP_INFO_STREAM(node_->get_logger(),"这个测试输出的是\n"<<T.matrix());

            RCLCPP_INFO(node_->get_logger(), "End-effector position: [%.3f, %.3f, %.3f]",
                        t.x(), t.y(), t.z());
            RCLCPP_INFO(node_->get_logger(), "End-effector orientation (quaternion): [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
                        q_end.w(), q_end.x(), q_end.y(), q_end.z());

            joint_state_sub_ =
            node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(&NODEWrapper::jointStateCallback, this, std::placeholders::_1));
        }

    // 你要的函数：输入 Eigen::Isometry3d，外加父子帧和时间戳
    void sendEigenTransform(const Eigen::Isometry3d& T,
                            const std::string& parent_frame,
                            const std::string& child_frame,
                            const rclcpp::Time& stamp = rclcpp::Time(0)) {
        // 1) Eigen -> TransformStamped
        geometry_msgs::msg::TransformStamped tf_msg =
            tf2::eigenToTransform(T);  // 来自 tf2_eigen

        // 2) 填 header 信息
        tf_msg.header.stamp =
            (stamp == rclcpp::Time(0)) ? node_->get_clock()->now() : stamp;
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id = child_frame;

        // 3) 发送
        tf_broadcaster_->sendTransform(tf_msg);
    }

    bool getCurrentJointPositions(std::array<double, 7>& q_out)
 {
     std::lock_guard<std::mutex> lock(joint_state_mutex_);

     if (!last_joint_state_) {
         RCLCPP_WARN(node_->get_logger(), "还没有收到任何 /joint_states 消息");
         return false;
     }

     const auto& msg = *last_joint_state_;

     // 对每一个我们关心的关节名，去 joint_state 里找索引
     for (size_t i = 0; i < joint_names.size(); ++i) {
         const std::string& name = joint_names[i];

         auto it = std::find(msg.name.begin(), msg.name.end(), name);
         if (it == msg.name.end()) {
             RCLCPP_WARN(node_->get_logger(),
                         "在 /joint_states 里找不到关节: %s", name.c_str());
             return false;
         }

         size_t idx = std::distance(msg.name.begin(), it);
         if (idx >= msg.position.size()) {
             RCLCPP_WARN(node_->get_logger(),
                         "关节 %s 在 /joint_states 中没有 position 数据", name.c_str());
             return false;
         }

         q_out[i] = msg.position[idx];
     }

     return true;
 }
   private:

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        last_joint_state_ = msg;
    }
    //获取角度
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    std::mutex joint_state_mutex_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
public:
    std::unique_ptr<UrdfKinematics> kinematics_;


};

};
