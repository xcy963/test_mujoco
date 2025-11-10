#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>


namespace hitcrt{

class DoubleTrackbarController {
private:
    std::string window_name_;
    cv::Mat display_image_;
    int max_slider_value_;
    std::vector<std::string> variable_names_;
    std::map<std::string, double> variables_;//cpp不知道有没有哈希表
    std::map<std::string, int> slider_values_;
    cv::Size display_size_;

    // 静态回调函数包装器
    static void onTrackbarWrapper(int value, void* userdata) {
        TrackbarData* data = static_cast<TrackbarData*>(userdata);
        data->controller->onTrackbar(value, data->variable_name);
    }

    // 实际回调处理
    void onTrackbar(int value, const std::string& var_name) {
        if (slider_values_.find(var_name) != slider_values_.end()) {
            slider_values_[var_name] = value;
            variables_[var_name] = value / 100.0; // 转换为double，精度0.01
            updateDisplay();
        }
    }

    // 内部数据结构用于回调
    struct TrackbarData {
        DoubleTrackbarController* controller;
        std::string variable_name;
    };
    std::vector<TrackbarData*> trackbar_data_;

public:
    // 构造函数
    DoubleTrackbarController(const std::string& window_name = "Double Controller", 
                           cv::Size display_size = cv::Size(600, 300),
                           int max_slider_value = 1000)
        : window_name_(window_name), display_size_(display_size), 
          max_slider_value_(max_slider_value) {
        display_image_ = cv::Mat::zeros(display_size_, CV_8UC3);
    }

    // 析构函数
    ~DoubleTrackbarController() {
        // 清理动态分配的内存
        for (auto data : trackbar_data_) {
            delete data;
        }
        cv::destroyWindow(window_name_);
    }

    // 添加双精度变量
    void addVariable(const std::string& name, double initial_value = 0.0) {
        variable_names_.push_back(name);
        variables_[name] = initial_value;
        int initial_slider_value = static_cast<int>(initial_value * 100);
        slider_values_[name] = initial_slider_value;
    }

    // 初始化窗口和滑动条
    bool initialize() {
        if (variable_names_.empty()) {
            std::cerr << "Error: No variables added!" << std::endl;
            return false;
        }

        // 创建窗口
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

        // 为每个变量创建滑动条
        for (size_t i = 0; i < variable_names_.size(); ++i) {
            const std::string& var_name = variable_names_[i];
            
            // 创建回调数据
            TrackbarData* data = new TrackbarData{this, var_name};
            trackbar_data_.push_back(data);

            // 创建滑动条
            cv::createTrackbar(var_name, window_name_, 
                             &slider_values_[var_name], max_slider_value_,
                             onTrackbarWrapper, data);
        }

        // 初始显示
        updateDisplay();
        
        std::cout << "DoubleTrackbarController initialized successfully!" << std::endl;
        std::cout << "Window: " << window_name_ << std::endl;
        std::cout << "Controls " << variable_names_.size() << " variables" << std::endl;
        std::cout << "Press 'q' to quit, 'p' to print current values" << std::endl;
        
        return true;
    }

    // 更新显示
    void updateDisplay() {
        // 清空显示图像
        display_image_ = cv::Mat::zeros(display_size_, CV_8UC3);
        
        // 绘制标题
        cv::putText(display_image_, "Double Variables Controller", 
                   cv::Point(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                   cv::Scalar(255, 255, 255), 2);

        // 显示每个变量的当前值
        for (size_t i = 0; i < variable_names_.size(); ++i) {
            const std::string& var_name = variable_names_[i];
            double value = variables_[var_name];
            
            std::string text = var_name + ": " + std::to_string(value);
            cv::Scalar color(0, 255 - i * 80, 255); // 不同变量不同颜色
            
            cv::putText(display_image_, text, 
                       cv::Point(50, 80 + i * 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       color, 2);
        }

        // 显示操作说明
        cv::putText(display_image_, "Adjust sliders to change values", 
                   cv::Point(50, display_size_.height - 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        cv::putText(display_image_, "Press 'q':quit, 'p':print values, 's':save", 
                   cv::Point(50, display_size_.height - 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        cv::imshow(window_name_, display_image_);
    }

    // 获取变量值
    double getVariable(const std::string& name) const {
        auto it = variables_.find(name);
        if (it != variables_.end()) {
            return it->second;
        }
        std::cerr << "Error: Variable '" << name << "' not found!" << std::endl;
        return 0.0;
    }

    // 获取所有变量值
    std::map<std::string, double> getAllVariables() const {
        return variables_;
    }

    // 设置变量值
    void setVariable(const std::string& name, double value) {
        if (variables_.find(name) != variables_.end()) {
            variables_[name] = value;
            slider_values_[name] = static_cast<int>(value * 100);
            cv::setTrackbarPos(name, window_name_, slider_values_[name]);
            updateDisplay();
        }
    }

    // 打印当前所有变量值
    void printCurrentValues() const {
        std::cout << "\n=== Current Variable Values ===" << std::endl;
        for (const auto& pair : variables_) {
            std::cout << pair.first << ": " << pair.second << std::endl;
        }
        std::cout << "===============================" << std::endl;
    }

    // // 保存当前值到文件
    // void saveValuesToFile(const std::string& filename) const {
    //     // 实际实现中可以添加文件保存逻辑
    //     std::cout << "Values saved to " << filename << " (simulated)" << std::endl;
    //     printCurrentValues();
    // }

    // 主运行循环
    // void run() {
    //     if (!initialize()) {
    //         return;
    //     }

    //     while (true) {
    //         char key = cv::waitKey(30);
    //         if (key == 'q' || key == 'Q') {
    //             break;
    //         } else if (key == 'p' || key == 'P') {
    //             printCurrentValues();
    //         } 
    //         // else if (key == 's' || key == 'S') {
    //         //     saveValuesToFile("current_values.txt");
    //         // }
    //     }

    //     std::cout << "\nFinal values:" << std::endl;
    //     printCurrentValues();
    // }
};

};
