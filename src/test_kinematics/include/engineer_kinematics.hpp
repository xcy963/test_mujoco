#include<vector>
#include <cmath>
#include <array>
#include<iostream>
#include <eigen3/Eigen/Dense>
#include "engineer_serial.h"

namespace hitcrt{
namespace kinematics{

struct params{
    double delta = M_PI_2;//初始状态下在上平台投影点的夹角,用于使用我们定义的坐标系去计算V

    double yita1 = -M_PI_2;//初始W轴角度
    double yita2 = 30.0/180.0*M_PI; 
    double yita3 = 150.0/180.0*M_PI;

    double arfa1 = 54.0/180.0*M_PI;//连杆参量
    double arfa2 = M_PI_2;
};

class engineer_kinematics
{
private:
    /* data */
    double delta_ = M_PI_2;//初始状态下在上平台投影点的夹角

    double yita_[3];//初始状态下上平台三个投影点对应的角度
    
    double arfa1_;
    double arfa2_;

    inline bool get_3v_fromT(const Eigen::Isometry3d &T,
                        Eigen::Vector3d &vector_V1,Eigen::Vector3d &vector_V2,Eigen::Vector3d &vector_V3);
    inline bool solve_oneangle(const Eigen::Vector3d &vector_V,const size_t& index,double& angle);//从一个固定的上平台向量中反解出电机角度

public:
    engineer_kinematics(const params& params_temp);//默认使用工程的参数
    ~engineer_kinematics() = default;

    Eigen::Isometry3d forward_kinematics(const std::vector<double>& joints);
    bool inverse_kinematics(const Eigen::Isometry3d &T,std::vector<double>& res);

    // double solve_triangle(const double &a,const double &b,const double &c);//注意处理共线的情况哦

};

engineer_kinematics::engineer_kinematics(const params& params_temp){
    delta_ = params_temp.delta;

    yita_[0] = params_temp.yita1;
    yita_[1] = params_temp.yita2;
    yita_[2] = params_temp.yita3;

    arfa1_ = params_temp.arfa1;
    arfa2_ = params_temp.arfa2;
}

// Eigen::Isometry3d engineer_kinematics::forward_kinematics(const std::vector<double>& joints){

// }

bool engineer_kinematics::inverse_kinematics(const Eigen::Isometry3d &T,std::vector<double>& res){
    Eigen::Vector3d vector_V1;
    Eigen::Vector3d vector_V2;
    Eigen::Vector3d vector_V3;
    // std::cout<<"输入的矩阵\n"<<T.matrix()<<std::endl;

    bool flag1 = get_3v_fromT(T,vector_V1,vector_V2,vector_V3);
    // std::cout<<"算出来的三个向量\n"<<vector_V1<<"\n第二个\n"<<vector_V2<<"\n第三个\n"<<vector_V3<<std::endl;

    if(!flag1) return false;
    double angle1;
    double angle2;
    double angle3;//电机的三个角度值

    bool flag2 = solve_oneangle(vector_V1,0,angle1);

    if(!flag2){
        std::cout<<"是sita1没有解"<<std::endl;
        return false;
    } 

    flag2 = solve_oneangle(vector_V2,1,angle2);
    
    if(!flag2){
        std::cout<<"是sita2没有解"<<std::endl;
        return false;
    } 
    
    flag2 = solve_oneangle(vector_V3,2,angle3);
    
    if(!flag2){
        std::cout<<"是sita3没有解"<<std::endl;
        return false;
    } 
    res.clear();
    res = {angle1,angle2,angle3};
    return true;
}

/* @brief c是对边,返回的是0 - 180度的
        注意处理浮点误差
        大家可能要使用的函数
*/
// double engineer_kinematics::solve_triangle(const double &a,const double &b,const double &c){//之前已经判断过三角形的构成条件了
//     const double tolerance = 1e-5;//处理三角形退化成线段相加的情况
//     double cos_sita = (a*a + b*b - c*c) / (2 * a * b);
//     double sin_sita ;
//     if (cos_sita > 1.0 - tolerance){
//         cos_sita = 1.0;
//         sin_sita = 0;
//     } else if(cos_sita < -1.0 + tolerance){
//         cos_sita = -1.0;
//         sin_sita = 0;
//     }else{
//         sin_sita = std::sqrt(1-cos_sita*cos_sita);
//     }

//     double sita = std::atan2(sin_sita,cos_sita);//因为我们默认三角形内角是0 - 180所以他的坐标一定是位于上半平面

//     return sita;
// }

inline bool engineer_kinematics::get_3v_fromT(const Eigen::Isometry3d &T,
                    Eigen::Vector3d &vector_V1,Eigen::Vector3d &vector_V2,Eigen::Vector3d &vector_V3){
    double angle_V1 = yita_[0] + delta_;
    double angle_V2 = yita_[1] + delta_;
    double angle_V3 = yita_[2] + delta_;
    // std::cout<<"三个角度的值1:"<<angle_V1*180*M_1_PI<<"第二个角度:"<<angle_V2*180*M_1_PI<<"第三个角度:"<<angle_V3*180*M_1_PI<<std::endl;
    auto X_axis = T.rotation().col(0);
    auto Y_axis = T.rotation().col(1);
    // auto Z_axis = T.rotation().col(2);

    vector_V1 = X_axis * std::cos(angle_V1) + Y_axis * std::sin(angle_V1);
    vector_V1.normalize();
    vector_V2 = X_axis * std::cos(angle_V2) + Y_axis * std::sin(angle_V2);
    vector_V2.normalize();
    vector_V3 = X_axis * std::cos(angle_V3) + Y_axis * std::sin(angle_V3);
    vector_V3.normalize();//减少浮点误差带来的影响

    return true;

}

inline bool engineer_kinematics::solve_oneangle(const Eigen::Vector3d &vector_V,const size_t& index,double& angle){//更加符合逻辑的做法是加入一个参考角度
    if(index >= 3 ){
        return false;
    }
    //从一个固定的上平台向量中反解出电机角度
    double tan_angle_2;//tan(angle + yita) / 2
    double a = vector_V.z() * std::cos(arfa1_) + std::cos(arfa2_) + vector_V.x() * std::sin(arfa1_);
    double b = -2 * vector_V.y() * std::sin(arfa1_)   ;
    double c = vector_V.z() * std::cos(arfa1_) + std::cos(arfa2_) - vector_V.x() * std::sin(arfa1_);
    // std::endl
    // std::cout<<"检查一下sin(arfa1):"<<std::sin(arfa1_)<<" cos(arfa2_):"<<std::cos(arfa2_)<<std::endl;
    // std::cout<<"这次的向量是\n"<<vector_V<<std::endl;
    // std::cout<<"二次方程的三个系数1:"<<a<<" 2:"<<b<<" 3:"<<c<<std::endl;
    if(std::abs(a) < 1e-5){//放一个误差,这个情况代表是0,退化
        tan_angle_2 = c/(-b);
    }else {
        double delta_fun = b * b -4 * a * c;
        if(std::abs(delta_fun) < 1e-5 ){//说明是相等的解
            tan_angle_2 = ( -b )/(2 * a);
        }else if(delta_fun<0) {
            std::cout<<"这次逆运动学没有解"<<std::endl;
            return false;
        }else{
            std::cout<<"这次逆运动学产生多解,选择小的"<<std::endl;
            tan_angle_2 = (-b - std::sqrt(delta_fun))/(2 * a);
            
            // double angle_other = std::atan2((-b + std::sqrt(delta_fun))/(2 * a),1);//范围是-pi/2到pi/2,选取1 4像限
            // angle_other = std::atan2(angle_other,1);
            // angle_other = (angle_other * 2 -yita_[index])
            // std::cout<<"对应的另外一个角度是: "<<angle_other*180*M_1_PI<<std::endl;
            
        }
    }
    
    angle = std::atan2(tan_angle_2,1);//范围是-pi/2到pi/2,选取1 4像限
    // Eigen::Vector3d W_temp;
    // W_temp<<std::cos(angle * 2) * std::sin(arfa1_),std::sin(angle * 2) * std::sin(arfa1_),-std::cos(arfa1_);
    // W_temp.normalize();
    // double dot_res = W_temp.dot(vector_V);
    // std::cout<<"计算出来的夹角"<<std::acos(dot_res)*180*M_1_PI<<std::endl;
    angle = angle * 2 - yita_[index]; //减去W的初始角度
    return true;
}

};//kinematics
};//hitcrt