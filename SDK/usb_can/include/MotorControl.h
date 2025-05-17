#pragma once

#include "usb_can.h"

class MotorControl {
public: 
    MotorControl(int32_t device=0, uint8_t channel=1);
    ~MotorControl();

    bool MotorInit();
    
    void ComputeCommandFromTau(uint32_t canID, double tau);
    void ComputeHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau);

    void SendTauCommand(uint32_t canID, double tau);
    void SendHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau);

    void enableMotor();
    void disableMotor();

private:

    int32_t device;  // 设备句柄
    uint8_t _channel;
    FrameInfo sendInfo,enableInfo,disableInfo; 
    uint8_t data[8];   
};
