#pragma once

#include "MotorControl.h"
#include <thread>
#include <atomic>
#include <mutex> // 引入互斥锁
#include <fstream>

struct LegState
{
    double rf_q[3], lf_q[3], rh_q[3], lh_q[3];
    double rf_v[3], lf_v[3], rh_v[3], lh_v[3];
    double rf_tau[3], lf_tau[3], rh_tau[3], lh_tau[3];
};

class LegControl {
public:
    LegControl();
    ~LegControl();

    void LegInit();
    void StopLeg();

    void send(double tauff[12]);
    void send2(double pos_des[12], double vel_des[12], double Kp[12] , double Kd[12], double tauff[12]);
    
    void RecvDevice1Data();
    void RecvDevice2Data();
    MotorControl RF, LF, RH, LH;
    LegState getState() const;

private:
    int32_t device1;  // 第一个设备的句柄
    int32_t device2;  // 第二个设备的句柄

    uint8_t readDevice1Channel;
    uint8_t readDevice2Channel;
    FrameInfo readDevice1Info;
    FrameInfo readDevice2Info;
    uint8_t data1[6], data2[6];

    std::thread recvDevice1Thread; // 前腿接收线程
    std::thread recvDevice2Thread; // 后腿接收线程
    std::atomic<bool> running;

    mutable std::mutex stateMutex; // 互斥锁，mutable 允许在 const 函数中锁定

    
    LegState _state;
    int32_t timeout;

    // std::ofstream data2File;
};
