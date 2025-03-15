#pragma once
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <queue>
#include "robot_control/actions_lcmt.hpp"
#include "robot_control/obs_lcmt.hpp"
#include <thread>
#include <atomic>

class TorqueBuffer {
public:
    void push(const robot_control::actions_lcmt& data);
    bool try_pop(robot_control::actions_lcmt& data);
    size_t size() const;

private:
    mutable std::mutex mtx_;
    std::queue<robot_control::actions_lcmt> buffer_;
};

class LCMHandler {
public:
    explicit LCMHandler(TorqueBuffer& buffer);
    ~LCMHandler(); // 添加析构函数
    void handleMessage(const lcm::ReceiveBuffer* rb, const std::string& chan, const robot_control::actions_lcmt* msg);
    void publishObs(const robot_control::obs_lcmt& obs);

private:
    TorqueBuffer& buffer_;
    lcm::LCM lcm_;
    std::thread lcm_thread_;
    std::atomic<bool> running_{false};
};