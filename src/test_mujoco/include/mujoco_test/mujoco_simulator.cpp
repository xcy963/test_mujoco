// mujoco_simulator.cpp
#include "mujoco_simulator.h"
#include <iostream>
#include <chrono>

// 包含必要的头文件
#include "glfw_adapter.h"
#include "simulate.h"

namespace {
    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    const int kErrorLength = 1024;
}

// 在实现文件中定义别名
namespace mj = mujoco;

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
    initial_model_path_ = model_path;
    
    // 打印版本信息
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version()) {
        std::cerr << "Headers and library have different versions" << std::endl;
        return false;
    }

    // 加载插件
    loadPluginLibraries();

    // 创建模拟对象 - 现在使用正确的命名空间
    simulate_ = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &camera_, &option_, &perturb_, false
    );
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

void MuJoCoSimulator::renderLoop(const std::unique_ptr<CameraRenderer> &CameraRenderer) {

    if (simulate_) {
        simulate_->RenderLoop(CameraRenderer);
    }
}

void MuJoCoSimulator::setStepCallback(StepCallback callback) {
    step_callback_ = callback;
}

std::pair<mjModel*, mjData*> MuJoCoSimulator::getModelAndData() {
    return {model_, data_};
}

void MuJoCoSimulator::physicsThread() {
    // 如果有初始模型路径，加载模型
    if (!initial_model_path_.empty()) {
        if (simulate_) {
            simulate_->LoadMessage(initial_model_path_.c_str());
        }
        model_ = loadModel(initial_model_path_);
        if (model_) {
            data_ = mj_makeData(model_);
            if (data_ && simulate_) {
                simulate_->Load(model_, data_, initial_model_path_.c_str());
                mj_forward(model_, data_);
            }
        }
        if (simulate_ && (!model_ || !data_)) {
            simulate_->LoadMessageClear();//这个意味着加载失败了
            std::cerr<<"fail to init simulate_"<<std::endl;
        }
    }
    // if (egl_context_) {
    //     egl_context_->makeCurrent();
    // }


    // CPU-模拟同步点,不知道这个在这是干吗的
    // std::chrono::time_point<std::chrono::steady_clock> syncCPU;
    // double syncSim = 0;

    while (!exit_request_.load()) {
        if (!simulate_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

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

        if (simulate_ && model_ && data_) {
            std::lock_guard<std::recursive_mutex> lock(simulate_->mtx);
            
            if (simulate_->run) {
                // 运行模拟逻辑
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

void MuJoCoSimulator::loadPluginLibraries() {
    // 这里实现插件加载逻辑
    // 暂时留空，根据实际需要实现
}

std::string MuJoCoSimulator::getExecutableDir() {
    // 实现获取可执行文件目录逻辑
    return "";
}

mjModel* MuJoCoSimulator::loadModel(const std::string& file_path) {
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