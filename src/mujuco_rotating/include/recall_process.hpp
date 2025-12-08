#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

struct Frame {
    std::vector<unsigned char> rgb;
    int width = 0;
    int height = 0;
    double t = 0.0;
};

class FrameQueue {
public:
    void push(Frame&& f) {
        std::lock_guard<std::mutex> lock(mtx_);
        // 如果队列积压太多，可以选择丢掉旧帧，只保留最新一帧
        if (!q_.empty()) {
            q_.pop();
        }
        q_.push(std::move(f));
        cv_.notify_one();
    }

    // 阻塞式 pop，返回 false 表示队列关闭
    bool pop(Frame& out) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [&] { return !q_.empty() || stopped_; });
        if (stopped_ && q_.empty())
            return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

private:
    std::queue<Frame> q_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stopped_ = false;
};
