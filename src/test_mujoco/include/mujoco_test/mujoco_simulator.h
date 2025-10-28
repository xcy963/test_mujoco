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

// 前向声明
namespace mujoco {
class Simulate;
}

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
    
    // 渲染循环
    void renderLoop();

private:
    void physicsThread();
    void loadPluginLibraries();
    std::string getExecutableDir();
    mjModel* loadModel(const std::string& file_path);
    void cleanup();

    // 使用 void* 来避免包含具体头文件，或者使用前向声明的智能指针
    std::unique_ptr<mujoco::Simulate> simulate_;
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

    // 模拟参数
    std::string initial_model_path_;
};

#endif // MUJOCO_SIMULATOR_H