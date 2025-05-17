#include <iostream>
#include <algorithm>
#include <iomanip> 
#include <math.h>
#include <unistd.h>
#include "MotorControl.h"

MotorControl::MotorControl(int32_t device, uint8_t channel)
    : device(device),
      _channel(channel),
      sendInfo{0, STANDARD, 8},
      data{0}
{}
MotorControl::~MotorControl() {
    // 设备关闭由 LegControl 管理，不需要在这里关闭
}

bool MotorControl::MotorInit() {
    enableMotor();
    return true;
}

void MotorControl::ComputeCommandFromTau(uint32_t canID, double tau) {
    int torque = static_cast<int>((tau + 48.0) * 4095 / 96.0);
    torque = std::max(0, std::min(4095, torque));

    uint8_t torque_high4 = static_cast<uint8_t>((torque >> 8) & 0x0F);
    uint8_t torque_low8 = static_cast<uint8_t>(torque & 0xFF);

    data[0] = 0x7F;
    data[1] = 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = static_cast<uint8_t>((0x00 << 4) | torque_high4);
    data[7] = torque_low8;

    sendInfo.canID = canID;
    sendInfo.frameType = STANDARD;
    sendInfo.dataLength = 8;
}

void MotorControl::ComputeHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau) {
    // 计算位置映射
    int position = static_cast<int>((pos + 4.0 * M_PI) * 65535 / (8.0 * M_PI));
    position = std::max(0, std::min(65535, position)); // 确保位置值在0到65535之间

    // 提取位置值的高8位和低8位
    uint8_t position_high8 = static_cast<uint8_t>((position >> 8) & 0xFF);
    uint8_t position_low8 = static_cast<uint8_t>(position & 0xFF);

    // 计算速度映射
    int velocity = static_cast<int>((vel + 30.0) * 4095 / 60.0);
    velocity = std::max(0, std::min(4095, velocity)); // 确保速度值在0到4095之间

    // 提取速度值高8位和低4位
    uint8_t velocity_high8 = static_cast<uint8_t>((velocity >> 4) & 0xFF);
    uint8_t velocity_low4 = static_cast<uint8_t>(velocity & 0x0F);

    // 计算kp值映射
    int kp_mapped = static_cast<int>(kp * 4095 / 500.0);
    kp_mapped = std::max(0, std::min(4095, kp_mapped)); // 确保kp值在0到4095之间

    // 提取kp值的高4位和低8位
    uint8_t kp_high4 = static_cast<uint8_t>((kp_mapped >> 8) & 0x0F);
    uint8_t kp_low8 = static_cast<uint8_t>(kp_mapped & 0xFF);

    // 计算kd值映射
    int kd_mapped = static_cast<int>(kd * 4095 / 100.0);
    kd_mapped = std::max(0, std::min(4095, kd_mapped)); // 确保kd值在0到4095之间

    // 提取kd值的高8位和低4位
    uint8_t kd_high8 = static_cast<uint8_t>((kd_mapped >> 4) & 0xFF);
    uint8_t kd_low4 = static_cast<uint8_t>(kd_mapped & 0x0F);

    // 计算力矩映射
    int torque = static_cast<int>((tau + 48.0) * 4095 / 96.0);
    torque = std::max(0, std::min(4095, torque));

    uint8_t torque_high4 = static_cast<uint8_t>((torque >> 8) & 0x0F);
    uint8_t torque_low8 = static_cast<uint8_t>(torque & 0xFF);

    data[0] = position_high8;
    data[1] = position_low8;
    data[2] = velocity_high8;
    data[3] = static_cast<uint8_t>((velocity_low4 << 4) | (kp_high4 & 0x0F));
    data[4] = kp_low8;
    data[5] = kd_high8;
    data[6] = static_cast<uint8_t>((kd_low4 << 4) | torque_high4);
    data[7] = torque_low8;

    sendInfo.canID = canID;
    sendInfo.frameType = STANDARD;
    sendInfo.dataLength = 8;
}

void MotorControl::SendTauCommand(uint32_t canID, double tau) {
    ComputeCommandFromTau(canID, tau);
    int32_t result = sendUSBCAN(device, _channel, &sendInfo, data);
    if (result == -1) {
        std::cerr << "Failed to send CAN frame. Return value: " << result << std::endl;
        tauff[12]={0,0,0,0,0,0,0,0,0,0,0,0};

    }
}

void MotorControl::SendHybridCommand(uint32_t canID, double pos, double vel, double kp, double kd, double tau) {
    ComputeHybridCommand(canID, pos, vel, kp, kd, tau);
    int32_t result = sendUSBCAN(device, _channel, &sendInfo, data);
    if (result == -1) {
        std::cerr << "Failed to send CAN frame. Return value: " << result << std::endl;
    }
}

void MotorControl::enableMotor() {
    uint8_t enabledata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    for (int i = 1; i < 4; i++) {
        enableInfo.canID = static_cast<uint32_t>(i);
        enableInfo.frameType = STANDARD;
        enableInfo.dataLength = 8;

        int32_t result = sendUSBCAN(device, _channel, &enableInfo, enabledata);
        if (result == -1) {
            std::cerr << "Failed to enable the Motor!. Return value: " << result << std::endl;
        }
        usleep(100);
    }
}

void MotorControl::disableMotor() {
    uint8_t disabledata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    for (int i = 1; i < 4; i++) {
        disableInfo.canID = static_cast<uint32_t>(i);
        disableInfo.frameType = STANDARD;
        disableInfo.dataLength = 8;

        int32_t result = sendUSBCAN(device, _channel, &disableInfo, disabledata);
        if (result == -1) {
            std::cerr << "Failed to disable the Motor!. Return value: " << result << std::endl;
        }
        usleep(100);
    }
}
