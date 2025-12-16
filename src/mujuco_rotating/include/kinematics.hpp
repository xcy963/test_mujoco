#pragma once
#include<vector>
#include <cmath>
#include <array>
// #include "tools.hpp"
#include <eigen3/Eigen/Dense>
#include<iostream>

namespace hitcrt{

namespace tools{

Eigen::Matrix3d create_rotation_x(const double& angle_x) {
    double cos_angle = std::cos(angle_x);
    double sin_angle = std::sin(angle_x);
    Eigen::Matrix3d res;
    res << 1, 0,         0,
            0, cos_angle, -sin_angle,
            0, sin_angle, cos_angle;
    return res;
}

Eigen::Matrix3d create_rotation_y(const double& angle_y) {
    double cos_angle = std::cos(angle_y);
    double sin_angle = std::sin(angle_y);
    Eigen::Matrix3d res;
    res << cos_angle, 0, sin_angle,
            0,         1, 0,
            -sin_angle, 0, cos_angle;
    return res;
}

Eigen::Matrix3d create_rotation_z(const double& angle_z) {
    double cos_angle = std::cos(angle_z);
    double sin_angle = std::sin(angle_z);
    Eigen::Matrix3d res;
    res << cos_angle, -sin_angle, 0,
            sin_angle, cos_angle , 0,
            0,           0,        1;
    return res;
}
};
class homework_kinematics
{
private:
    /* data */
    double l1_;
    double l2_;
    double sita_;

    Eigen::Isometry3d T_J1_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_J2_ = Eigen::Isometry3d::Identity();//link1到link2
    Eigen::Isometry3d T_J3_ = Eigen::Isometry3d::Identity();//link2到link3
    Eigen::Isometry3d T_J4_ = Eigen::Isometry3d::Identity();//link2到link3


public:
    homework_kinematics(const double & l1 = 0.275,const double & l2 = 0.483,const double &sita = 40);//默认使用工程的参数
    ~homework_kinematics() = default;

    Eigen::Isometry3d forward_kinematics(const std::vector<double>& joints);
    void test();
    //留给小登写的:)
    std::vector<double> inverse_kinematics(const Eigen::Isometry3d &T);
    double solve_triangle(const double &a,const double &b,const double &c);//注意处理共线的情况哦

};

homework_kinematics::homework_kinematics(const double & l1,const double & l2,const double & sita){
    T_J1_ =  Eigen::Isometry3d::Identity();
    double sin_sita = std::sin(sita/180*M_PI);
    double cos_sita = std::cos(sita/180*M_PI);

    T_J2_.linear()<<-cos_sita,sin_sita,0,
                    0,        0,       1,
                    sin_sita, cos_sita, 0;//没有平移

    T_J3_.linear()<<sin_sita,cos_sita, 0,
                    cos_sita,-sin_sita,0,
                    0,      0         ,-1;
    T_J3_.translation()<<l1,0,0;

    T_J4_.linear()<<1,0, 0,
                    0,0, 1,
                    0,-1,0;
    T_J4_.translation()<<0,-l2,0;

    l1_ = l1;
    l2_ = l2;
    sita_ = sita/180*M_PI;
    
}

Eigen::Isometry3d homework_kinematics::forward_kinematics(const std::vector<double>& joints){
    //默认是要求弧度制度,写警告太麻烦了,不如麻烦小登让他们注意点:)
    // std::cout
    using namespace hitcrt::tools;
    Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_z_1 = Eigen::Isometry3d(create_rotation_z(-joints[0]));
    Eigen::Isometry3d T_z_2 = Eigen::Isometry3d(create_rotation_z(joints[1]));
    Eigen::Isometry3d T_z_3 = Eigen::Isometry3d(create_rotation_z(joints[2]));//命名为R的T矩阵,大家见谅

    ret = ret * (T_J1_ * T_z_1) * (T_J2_ * T_z_2) * (T_J3_ * T_z_3) * T_J4_;
    ret.linear() = ret.rotation() * create_rotation_z(joints[3]) * create_rotation_y(-joints[4]) * create_rotation_z(-joints[5]);


    return ret;
}

std::vector<double> homework_kinematics::inverse_kinematics(const Eigen::Isometry3d &T){
    //实际上会有多解,但是机械没给那么大的限位
    Eigen::Vector3d P_4ORG = T.translation();
    // Eigen::Vector3d X_4 = T.rotation().col(0);
    // Eigen::Vector3d Y_4 = T.rotation().col(1);
    // Eigen::Vector3d Z_4 = T.rotation().col(2);

    double joint1 = std::atan2(P_4ORG.y(),P_4ORG.x());
    if(joint1 > M_PI_2){
        joint1 = joint1 - M_PI;
    }else if(joint1 < -M_PI_2){
        joint1 = M_PI + joint1;
    }//限制在 -M_PI_2 到 M_PI_2

    double l_temp = P_4ORG.norm();
    double joint3_temp = solve_triangle(l1_,l2_,l_temp);
    double joint3 = joint3_temp - sita_;

    
    Eigen::Vector3d direction_XOY;
    direction_XOY<<std::cos(joint1),std::sin(joint1),0;
    double angle_temp = std::atan2(P_4ORG.z(),P_4ORG.dot(direction_XOY));
    double angle_temp2 = solve_triangle(l1_,l_temp,l2_);

    double joint2 = M_PI - angle_temp - angle_temp2 - sita_;

    std::vector<double>res = {-joint1,joint2,joint3,0.0,0.0,0.0};
    Eigen::Isometry3d T_tmep = forward_kinematics(res);

    Eigen::Isometry3d T_next = T_tmep.inverse()*T;
    Eigen::Matrix3d R = T_next.rotation();   // 或 T_next.linear()
    // 2. 按 zyz 顺序求欧拉角（单位：弧度）
    Eigen::Vector3d euler_xyx = R.eulerAngles(2, 1, 2);

    if(std::abs(euler_xyx[1])<1e-5){
        double temp_sum =  euler_xyx[0] + euler_xyx[2];
        std::cout<<"加起来: "<<temp_sum<<std::endl;
        euler_xyx[0] = 0;
        euler_xyx[2] = temp_sum;
    }
    res[3] = euler_xyx[0] ;
    res[4] = -euler_xyx[1] ;
    res[5] = -euler_xyx[2] ;


    return res;
    
}

/* @brief c是对边,返回的是0 - 180度的
        注意处理浮点误差
        大家可能要使用的函数
*/
double homework_kinematics::solve_triangle(const double &a,const double &b,const double &c){//之前已经判断过三角形的构成条件了
    const double tolerance = 1e-5;//处理三角形退化成线段相加的情况
    double cos_sita = (a*a + b*b - c*c) / (2 * a * b);
    double sin_sita ;
    if (cos_sita > 1.0 - tolerance){
        cos_sita = 1.0;
        sin_sita = 0;
    } else if(cos_sita < -1.0 + tolerance){
        cos_sita = -1.0;
        sin_sita = 0;
    }else{
        sin_sita = std::sqrt(1-cos_sita*cos_sita);
    }

    double sita = std::atan2(sin_sita,cos_sita);//因为我们默认三角形内角是0 - 180所以他的坐标一定是位于上半平面

    return sita;
}




};