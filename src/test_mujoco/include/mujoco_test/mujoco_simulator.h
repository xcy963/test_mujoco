// mujoco_simulator.h
#ifndef MUJOCO_SIMULATOR_H
#define MUJOCO_SIMULATOR_H

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"

class MuJoCoSimulator {
public:
    using StepCallback = std::function<void(const mjModel*, const mjData*)>;
    
    MuJoCoSimulator();
    ~MuJoCoSimulator();
    
    bool initialize(const std::string& model_path = "");
    void start();
    void stop();
    bool isRunning() const;
    
    // 设置回调函数
    void setStepCallback(StepCallback callback);
    
    // 获取模型和数据（线程安全）
    std::pair<mjModel*, mjData*> getModelAndData();
    
private:
    void physicsThread();
    void loadPluginLibraries();
    std::string getExecutableDir();
    mjModel* loadModel(const std::string& file_path);
    void cleanup();

    std::unique_ptr<mj::Simulate> simulate_;
    mjModel* model_;
    mjData* data_;
    
    std::thread physics_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> exit_request_{false};
    
    StepCallback step_callback_;
    
    // 相机和选项
    mjvCamera camera_;
    mjvOption option_;
    mjvPerturb perturb_;
};

#endif // MUJOCO_SIMULATOR_H