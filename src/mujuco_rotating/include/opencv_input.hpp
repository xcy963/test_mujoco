#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

//opencv 只有框,但是完全足够了

namespace hitcrt{

class DoubleTrackbarController {
private:
    std::string window_name_;
    cv::Size display_size_;
    int max_slider_value_;
    cv::Mat display_image_;
    cv::Mat background_image_;


    std::vector<std::string> variable_names_;

    // 原来的 double 变量
    std::map<std::string, double> variables_;
    std::map<std::string, int> slider_values_;

    // ✅ 新增：bool 变量支持
    std::map<std::string, bool> bool_variables_;
    std::map<std::string, int> bool_slider_values_;   // 0 或 1
    std::map<std::string, bool> is_bool_;             // true 表示这个名字是 bool

    // 内部数据结构用于回调
    struct TrackbarData {
        DoubleTrackbarController* controller;//自己的指针
        std::string variable_name;
    };
    std::vector<TrackbarData*> trackbar_data_;

    // 静态回调函数包装器
    static void onTrackbarWrapper(int value, void* userdata) {
        TrackbarData* data = static_cast<TrackbarData*>(userdata);
        data->controller->onTrackbar(value, data->variable_name);
    }

    // 实际回调处理
    void onTrackbar(int value, const std::string& var_name) {
        // 根据类型区分
        auto type_it = is_bool_.find(var_name);
        if (type_it == is_bool_.end()) {
            std::cerr << "Error: variable type for '" << var_name << "' not found!" << std::endl;
            return;
        }

        if (type_it->second) { 
            // ✅ bool 类型
            bool_slider_values_[var_name] = value;
            bool_variables_[var_name] = (value != 0);
        } else {
            // ✅ double 类型（你的原始逻辑）
            auto it = slider_values_.find(var_name);
            if (it != slider_values_.end()) {
                slider_values_[var_name] = value;
                variables_[var_name] =
                    (value - max_slider_value_/2.0) / (max_slider_value_/360.0);
            }
        }

        updateDisplay();
    }

public:
    // 构造函数
    DoubleTrackbarController(const std::string& window_name = "Double Controller", 
                           cv::Size display_size = cv::Size(600, 300),
                           int max_slider_value = 36000)
        : window_name_(window_name), display_size_(display_size), 
          max_slider_value_(max_slider_value) {

        display_image_ = cv::Mat::zeros(display_size_, CV_8UC3);
        cv::Mat raw_bg = cv::imread("/home/hitcrt/enginner_26/test_mujoco_ros/imgs/bg.jpg");
        if (raw_bg.empty()) {
            std::cerr << "Failed to load background image, use black background instead\n";
            background_image_ = cv::Mat::zeros(display_size_, CV_8UC3);
        } else {
            // 2. resize 到 display_size_
                double alpha = 0.4; // 0~1，越小越黑
            raw_bg.convertTo(raw_bg, -1, alpha, 0);
            cv::resize(raw_bg, background_image_, display_size_);
            display_image_ = background_image_.clone();
        }
    }

    // 析构函数
    ~DoubleTrackbarController() {
        // 清理动态分配的内存
        for (auto data : trackbar_data_) {
            delete data;
        }
        cv::destroyWindow(window_name_);
    }

    // 添加 double 变量
    void addVariable(const std::string& name, double initial_value = 0.0) {
        variable_names_.push_back(name);
        is_bool_[name] = false;         // 标记为 double

        variables_[name] = initial_value;
        int initial_slider_value = static_cast<int>(
            initial_value * (max_slider_value_/360.0) + max_slider_value_/2.0
        );
        slider_values_[name] = initial_slider_value;
    }

    // ✅ 添加 bool 变量
    void addBoolVariable(const std::string& name, bool initial_value = false) {
        variable_names_.push_back(name);
        is_bool_[name] = true;          // 标记为 bool

        bool_variables_[name] = initial_value;
        int initial_slider_value = initial_value ? 1 : 0;
        bool_slider_values_[name] = initial_slider_value;
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

            bool is_bool_var = is_bool_[var_name];

            if (is_bool_var) {
                // ✅ bool：0/1 滑动条
                cv::createTrackbar(
                    var_name, window_name_,
                    &bool_slider_values_[var_name], 1,  // max=1
                    onTrackbarWrapper, data
                );
            } else {
                // double：原始逻辑
                cv::createTrackbar(
                    var_name, window_name_,
                    &slider_values_[var_name], max_slider_value_,
                    onTrackbarWrapper, data
                );
            }
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
        display_image_ = background_image_.clone();;
        
        // 绘制标题
        cv::putText(display_image_, "Double & Bool Variables Controller", 
                   cv::Point(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                   cv::Scalar(255, 255, 255), 2);

        // 显示每个变量的当前值
        for (size_t i = 0; i < variable_names_.size(); ++i) {
            const std::string& var_name = variable_names_[i];

            bool is_bool_var = is_bool_[var_name];
            std::string text;

            if (is_bool_var) {
                bool value = bool_variables_[var_name];
                text = var_name + " (bool): " + std::string(value ? "true" : "false");
            } else {
                double value = variables_[var_name];
                text = var_name + " (double): " + std::to_string(value);
            }

            cv::Scalar color(0, 255 - int(i) * 60, 255);
            cv::putText(display_image_, text, 
                       cv::Point(50, 80 + int(i) * 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                       color, 2);
        }

        // 显示操作说明
        cv::putText(display_image_, "Adjust sliders to change values", 
                   cv::Point(50, display_size_.height - 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        cv::putText(display_image_, "Press 'q':quit, 'p':print values", 
                   cv::Point(50, display_size_.height - 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

        cv::imshow(window_name_, display_image_);

        // cv::imwrite("/home/hitcrt/enginner_26/test_mujoco_ros/1.png",display_image_);nnd画不了上面的
        // throw "画完毕直接退出";

    }

    // 获取 double 变量值
    double getVariable(const std::string& name) const {
        auto t_it = is_bool_.find(name);
        if (t_it != is_bool_.end() && t_it->second) {
            std::cerr << "Error: Variable '" << name 
                      << "' is a bool, use getBoolVariable()" << std::endl;
            return 0.0;
        }

        auto it = variables_.find(name);
        if (it != variables_.end()) {
            return it->second;
        }
        std::cerr << "Error: Variable '" << name << "' not found!" << std::endl;
        return 0.0;
    }

    // ✅ 获取 bool 变量值
    bool getBoolVariable(const std::string& name) const {
        auto t_it = is_bool_.find(name);
        if (t_it == is_bool_.end() || !t_it->second) {
            std::cerr << "Error: Variable '" << name 
                      << "' is not a bool, or not found!" << std::endl;
            return false;
        }
        auto it = bool_variables_.find(name);
        if (it != bool_variables_.end()) {
            return it->second;
        }
        std::cerr << "Error: Bool variable '" << name << "' not found!" << std::endl;
        return false;
    }

    // 获取所有 double 变量（保持你原来的接口）
    std::map<std::string, double> getAllVariables() const {
        return variables_;
    }

    // 设置 double 变量值
    void setVariable(const std::string& name, double value) {
        auto t_it = is_bool_.find(name);
        if (t_it != is_bool_.end() && t_it->second) {
            std::cerr << "Error: Variable '" << name 
                      << "' is a bool, use setBoolVariable()" << std::endl;
            return;
        }

        if (variables_.find(name) != variables_.end()) {
            variables_[name] = value;
            slider_values_[name] = static_cast<int>(
                value * (max_slider_value_/360.0) + max_slider_value_/2.0
            );
            cv::setTrackbarPos(name, window_name_, slider_values_[name]);
            updateDisplay();
        }
    }

    // ✅ 设置 bool 变量值
    void setBoolVariable(const std::string& name, bool value) {
        auto t_it = is_bool_.find(name);
        if (t_it == is_bool_.end() || !t_it->second) {
            std::cerr << "Error: Variable '" << name 
                      << "' is not a bool, or not found!" << std::endl;
            return;
        }

        bool_variables_[name] = value;
        bool_slider_values_[name] = value ? 1 : 0;
        cv::setTrackbarPos(name, window_name_, bool_slider_values_[name]);
        updateDisplay();
    }

    // 打印当前所有变量值（同时打印 double + bool）
    void printCurrentValues() const {
        std::cout << "\n=== Current Variable Values ===" << std::endl;
        for (const auto& name : variable_names_) {
            auto t_it = is_bool_.find(name);
            if (t_it != is_bool_.end() && t_it->second) {
                auto itb = bool_variables_.find(name);
                if (itb != bool_variables_.end()) {
                    std::cout << name << " (bool): " 
                              << (itb->second ? "true" : "false") << std::endl;
                }
            } else {
                auto itd = variables_.find(name);
                if (itd != variables_.end()) {
                    std::cout << name << " (double): " 
                              << itd->second << std::endl;
                }
            }
        }
        std::cout << "===============================" << std::endl;
    }
};

} // namespace hitcrt