// mujoco_simulator.cpp
#include "mujoco_simulator.h"
#include <iostream>
#include <chrono>

namespace {
    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    const int kErrorLength = 1024;
}

MuJoCoSimulator::MuJoCoSimulator() 
    : model_(nullptr), data_(nullptr) {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultPerturb(&perturb_);
}

MuJoCoSimulator::~MuJoCoSimulator() {
    stop();
    cleanup();
}

bool MuJoCoSimulator::initialize(const std::string& model_path) {
    // 打印版本信息
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version()) {
        mju_error("Headers and library have different versions");
        return false;
    }

    // 加载插件
    loadPluginLibraries();

    // 创建模拟对象
    simulate_ = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &camera_, &option_, &perturb_, false
    );

    // 如果有模型路径，加载模型
    if (!model_path.empty()) {
        simulate_->LoadMessage(model_path.c_str());
        model_ = loadModel(model_path);
        if (model_) {
            data_ = mj_makeData(model_);
            if (data_) {
                simulate_->Load(model_, data_, model_path.c_str());
                mj_forward(model_, data_);
                return true;
            }
        }
        simulate_->LoadMessageClear();
        return false;
    }

    return true;
}

void MuJoCoSimulator::start() {
    if (running_) return;
    
    running_ = true;
    exit_request_ = false;
    
    // 启动物理线程
    physics_thread_ = std::thread(&MuJoCoSimulator::physicsThread, this);
}

void MuJoCoSimulator::stop() {
    if (!running_) return;
    
    exit_request_ = true;
    if (physics_thread_.joinable()) {
        physics_thread_.join();
    }
    running_ = false;
}

bool MuJoCoSimulator::isRunning() const {
    return running_;
}

void MuJoCoSimulator::setStepCallback(StepCallback callback) {
    step_callback_ = callback;
}

std::pair<mjModel*, mjData*> MuJoCoSimulator::getModelAndData() {
    return {model_, data_};
}

void MuJoCoSimulator::physicsThread() {
    // CPU-模拟同步点
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    double syncSim = 0;

    while (!exit_request_.load()) {
        // 处理加载请求
        if (simulate_->droploadrequest.load()) {
            simulate_->LoadMessage(simulate_->dropfilename);
            mjModel* new_model = loadModel(simulate_->dropfilename);
            simulate_->droploadrequest.store(false);

            mjData* new_data = nullptr;
            if (new_model) new_data = mj_makeData(new_model);
            if (new_data) {
                simulate_->Load(new_model, new_data, simulate_->dropfilename);

                std::lock_guard<std::recursive_mutex> lock(simulate_->mtx);
                if (data_) mj_deleteData(data_);
                if (model_) mj_deleteModel(model_);

                model_ = new_model;
                data_ = new_data;
                mj_forward(model_, data_);
            } else {
                simulate_->LoadMessageClear();
            }
        }

        // UI加载请求
        if (simulate_->uiloadrequest.load()) {
            simulate_->uiloadrequest.fetch_sub(1);
            simulate_->LoadMessage(simulate_->filename);
            mjModel* new_model = loadModel(simulate_->filename);
            mjData* new_data = nullptr;
            if (new_model) new_data = mj_makeData(new_model);
            if (new_data) {
                simulate_->Load(new_model, new_data, simulate_->filename);

                std::lock_guard<std::recursive_mutex> lock(simulate_->mtx);
                if (data_) mj_deleteData(data_);
                if (model_) mj_deleteModel(model_);

                model_ = new_model;
                data_ = new_data;
                mj_forward(model_, data_);
            } else {
                simulate_->LoadMessageClear();
            }
        }

        // 休眠或yield
        if (simulate_->run && simulate_->busywait) {
            std::this_thread::yield();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        {
            std::lock_guard<std::recursive_mutex> lock(simulate_->mtx);
            
            if (model_ && data_) {
                if (simulate_->run) {
                    // 运行模拟逻辑（简化版，完整逻辑参考原代码）
                    mj_step(model_, data_);
                    
                    // 调用步进回调
                    if (step_callback_) {
                        step_callback_(model_, data_);
                    }
                    
                    simulate_->AddToHistory();
                } else {
                    // 暂停状态
                    mj_forward(model_, data_);
                    if (simulate_->pause_update) {
                        mju_copy(data_->qacc_warmstart, data_->qacc, model_->nv);
                    }
                    simulate_->speed_changed = true;
                }
            }
        }
    }
}

// 其他辅助方法的实现...
void MuJoCoSimulator::loadPluginLibraries() {
    // 实现插件加载逻辑（参考原代码）
}

std::string MuJoCoSimulator::getExecutableDir() {
    // 实现获取可执行文件目录逻辑（参考原代码）
    return "";
}

mjModel* MuJoCoSimulator::loadModel(const std::string& file_path) {
    // 实现模型加载逻辑（参考原代码）
    char error[kErrorLength] = "";
    mjModel* model = mj_loadXML(file_path.c_str(), nullptr, error, kErrorLength);
    if (!model) {
        std::printf("Failed to load model: %s\n", error);
    }
    return model;
}

void MuJoCoSimulator::cleanup() {
    if (data_) {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_) {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}