#include "lcm_handler.h"
#include "controller_listener.h"

LCMHandler::LCMHandler(TorqueBuffer& buffer) : buffer_(buffer), running_(true) {
    if (lcm_.good()) {
        lcm_.subscribe("ACTIONS", &LCMHandler::handleMessage, this);
//      lcm_.subscribe("GAMEPAD_MODE", &LCMHandler::handleMessage, this);
        // 启动线程处理 LCM 消息
        lcm_thread_ = std::thread([this]() {
            while (running_) {
                lcm_.handleTimeout(1); // 每 1ms 处理一次消息
            }
        });
    }
}
controller_LCMHandler::controller_LCMHandler(controller_ToqueBuffer& buffer): buffer_(buffer),lcm("udpm://239.255.76.67:7667?ttl=255"),running_(true){
    if(lcm.good()){

        lcm.subscribe("GAMEPAD_MODE", &controller_LCMHandler::handleMessage, this);
        lcm_thread_ = std::thread([this]() {
            while(running_){lcm.handleTimeout(1);}
    });
}}

// 析构函数
LCMHandler::~LCMHandler() {
    running_ = false;
    if (lcm_thread_.joinable()) {
        lcm_thread_.join();
    }
}
void LCMHandler::handleMessage(const lcm::ReceiveBuffer*,
                              const std::string&,
                              const robot_control::actions_lcmt* msg) {
    buffer_.push(*msg);
}

controller_LCMHandler::~controller_LCMHandler() {
    running_ = false;
    if(lcm_thread_.joinable()){
        lcm_thread_.join();
    }
}
void controller_LCMHandler::handleMessage(const lcm::ReceiveBuffer*,
                                          const std::string&,
                                          const exlcm::example_t* msg){
    buffer_.push(*msg);
}
void LCMHandler::publishObs(const robot_control::obs_lcmt& obs) {
    lcm_.publish("OBS", &obs);
}

void TorqueBuffer::push(const robot_control::actions_lcmt& data) {
    std::lock_guard<std::mutex> lock(mtx_);
    buffer_.push(data);
}

void controller_ToqueBuffer::push(const exlcm::example_t& data){
    std::lock_guard<std::mutex> lock(mtx_);
    buffer_.push(data);
}
bool TorqueBuffer::try_pop(robot_control::actions_lcmt& data) {
    std::lock_guard<std::mutex> lock(mtx_);
    if(buffer_.empty()) return false;
    
    data = buffer_.front();
    buffer_.pop();
    return true;
}

bool controller_ToqueBuffer::try_pop(exlcm::example_t& data){
    std::lock_guard<std::mutex> lock(mtx_);
    if(buffer_.empty()) return false;
    
    data = buffer_.front();
    buffer_.pop();
    return true;
}
size_t TorqueBuffer::size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return buffer_.size();
}

size_t controller_ToqueBuffer::size() const{
    std::lock_guard<std::mutex> lock(mtx_);
    return buffer_.size();
}