#include "LegControl.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <mutex> // 引入互斥锁

const uint8_t channel1 = 1;
const uint8_t channel2 = 2;
const uint32_t canID1 = 1;
const uint32_t canID2 = 2;
const uint32_t canID3 = 3;

LegControl::LegControl()
    : device1(-1), device2(-1),
      RF(-1, channel1), LF(-1, channel2),
      RH(-1, channel1), LH(-1, channel2),
      timeout(1000000)
{}

LegControl::~LegControl() {
    running = false;
    if (recvDevice1Thread.joinable()) {
        recvDevice1Thread.join();
    }
    if (recvDevice2Thread.joinable()) {
        recvDevice2Thread.join();
    }
    if (device1 >= 0) {
        closeUSBCAN(device1);
    }
    if (device2 >= 0) {
        closeUSBCAN(device2);
    }
}

void LegControl::LegInit() {

    // data2File.open("data2.csv", std::ios::out);

    if (device1 < 0) {
        device1 = openUSBCAN("/dev/USB2CAN0");
        if (device1 < 0) {
            std::cerr << "Failed to open USB2CAN device 1!" << std::endl;
            return;
        }
        std::cout << "Device 1 opened successfully." << std::endl;
    }

    if (device2 < 0) {
        device2 = openUSBCAN("/dev/USB2CAN1");
        if (device2 < 0) {
            std::cerr << "Failed to open USB2CAN device 2!" << std::endl;
            return;
        }
        std::cout << "Device 2 opened successfully." << std::endl;
    }

    // 将设备句柄传递给 MotorControl 对象
    LF = MotorControl(device1, channel1);
    RF = MotorControl(device1, channel2);
    LH = MotorControl(device2, channel1);
    RH = MotorControl(device2, channel2);

   
    RF.MotorInit();
    
    RH.MotorInit();
    LF.MotorInit();
    LH.MotorInit();
    running = true;

    // 启动前腿和后腿的接收线程
    recvDevice1Thread = std::thread(&LegControl::RecvDevice1Data, this);
    recvDevice2Thread = std::thread(&LegControl::RecvDevice2Data, this);
}

void LegControl::StopLeg() {
    RF.disableMotor();
    LF.disableMotor();
    RH.disableMotor();
    LH.disableMotor();
}

void LegControl::send(double tauff[12]) {
    RF.SendTauCommand(canID1, tauff[0]);
    RH.SendTauCommand(canID1, tauff[6]);
    LF.SendTauCommand(canID1, tauff[3]);
    LH.SendTauCommand(canID1, tauff[9]);
    usleep(100);
    RF.SendTauCommand(canID2, tauff[1]);
    RH.SendTauCommand(canID2, tauff[7]);
    LF.SendTauCommand(canID2, tauff[4]);
    LH.SendTauCommand(canID2, tauff[10]);
    usleep(100);
    RF.SendTauCommand(canID3, tauff[2]);
    LF.SendTauCommand(canID3, tauff[5]);
    RH.SendTauCommand(canID3, tauff[8]);
    LH.SendTauCommand(canID3, tauff[11]);
}

void LegControl::send2(double pos_des[12], double vel_des[12], double Kp[12] , double Kd[12], double tauff[12]) {
    RF.SendHybridCommand(canID1, pos_des[0], vel_des[0], Kp[0] , Kd[0], tauff[0]); // 每条can通路相邻两次发的时间必须大于333us
    usleep(100);
    RH.SendHybridCommand(canID1, pos_des[6], vel_des[6], Kp[6] , Kd[6], tauff[6]);
    usleep(100);
    LF.SendHybridCommand(canID1, pos_des[3], vel_des[3], Kp[3] , Kd[3], tauff[3]);
    usleep(100);
    LH.SendHybridCommand(canID1, pos_des[9], vel_des[9], Kp[9] , Kd[9], tauff[9]);
    usleep(100);
    RF.SendHybridCommand(canID2, pos_des[1], vel_des[1], Kp[1] , Kd[1], tauff[1]);
    usleep(100);
    RH.SendHybridCommand(canID2, pos_des[7], vel_des[7], Kp[7] , Kd[7], tauff[7]);
    usleep(100);
    LF.SendHybridCommand(canID2, pos_des[4], vel_des[4], Kp[4] , Kd[4], tauff[4]);
    usleep(100);
    LH.SendHybridCommand(canID2, pos_des[10], vel_des[10], Kp[10] , Kd[10], tauff[10]);
    usleep(100);
    RF.SendHybridCommand(canID3, pos_des[2], vel_des[2], Kp[2] , Kd[2], tauff[2]);
    usleep(100);
    RH.SendHybridCommand(canID3, pos_des[8], vel_des[8], Kp[8] , Kd[8], tauff[8]);
    usleep(100);
    LF.SendHybridCommand(canID3, pos_des[5], vel_des[5], Kp[5] , Kd[5], tauff[5]);
    usleep(100);
    LH.SendHybridCommand(canID3, pos_des[11], vel_des[11], Kp[11] , Kd[11], tauff[11]);
    // usleep(100);
}

// 接收第一个模块的反馈数据（前腿）
void LegControl::RecvDevice1Data() {
    std::cout << "RecvDevice1Data thread started." << std::endl;
    while (running) {

        int32_t result = readUSBCAN(device1, &readDevice1Channel, &readDevice1Info, data1, timeout);
        // std::cout << "M1result"<< result << std::endl;
        // std::cout << "channel1: " << readDevice1Channel << std::endl;
        if (result == 0) {  // 成功接收一帧数据
            
            // 解析数据
            if (readDevice1Channel == 2) { // rf腿的三个电机
                
                uint16_t positionRaw = (static_cast<uint16_t>(data1[1]) << 8) | static_cast<uint16_t>(data1[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);

                uint16_t velocityRaw = (static_cast<uint16_t>(data1[3]) << 4) | (static_cast<uint16_t>(data1[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;

                uint16_t torqueRaw = ((static_cast<uint16_t>(data1[4] & 0x0F)) << 8) | static_cast<uint16_t>(data1[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;

                unsigned char id = data1[0];
                // std::cout << "canID:" << id << "position:" << position << std::endl;
                int motorIndex = static_cast<int>(id) - 1;
                std::cout << "canID:" << motorIndex << "position:" << position << std::endl;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.rf_q[motorIndex] = position;
                    _state.rf_v[motorIndex] = velocity;
                    _state.rf_tau[motorIndex] = torqueNm;
                }
                // std::cout << "canID:" << data1[0] << "position:" << position << std::endl;
            }
            else if (readDevice1Channel == 1) { // lf腿的三个电机
                uint16_t positionRaw = (static_cast<uint16_t>(data1[1]) << 8) | static_cast<uint16_t>(data1[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);

                uint16_t velocityRaw = (static_cast<uint16_t>(data1[3]) << 4) | (static_cast<uint16_t>(data1[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;

                uint16_t torqueRaw = ((static_cast<uint16_t>(data1[4] & 0x0F)) << 8) | static_cast<uint16_t>(data1[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;

                unsigned char id = data1[0];
                std::cout << "canID:" << id << "position:" << position << std::endl;
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.lf_q[motorIndex] = position;
                    _state.lf_v[motorIndex] = velocity;
                    _state.lf_tau[motorIndex] = torqueNm;
                }

                // std::cout << "canID:" << data1[0] << "position:" << position << std::endl;
            }
        }
        else if (result == -1) {
            std::cerr << "模块1 recv timeout!!!" << std::endl;
        }
    }
}

// 接收第二个模块的反馈数据（后腿）
void LegControl::RecvDevice2Data() {
    while (running) {
        int32_t result = readUSBCAN(device2, &readDevice2Channel, &readDevice2Info, data2, timeout);
        // std::cout << "M2result"<< result << std::endl;
        // std::cout << "channel2: " << readDevice2Channel << std::endl;
        if (result == 0) {  // 成功接收一帧数据
            std::cout << "2" << std::endl;
            // 解析数据
            if (readDevice2Channel == 2 ) { // rh腿的三个电机
            
                uint16_t positionRaw = (static_cast<uint16_t>(data2[1]) << 8) | static_cast<uint16_t>(data2[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);

                uint16_t velocityRaw = (static_cast<uint16_t>(data2[3]) << 4) | (static_cast<uint16_t>(data2[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;

                uint16_t torqueRaw = ((static_cast<uint16_t>(data2[4] & 0x0F)) << 8) | static_cast<uint16_t>(data2[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;

                unsigned char id = data2[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.rh_q[motorIndex] = position;
                    _state.rh_v[motorIndex] = velocity;
                    _state.rh_tau[motorIndex] = torqueNm;
                } 

                // std::cout << "canID:" << id << "position:" << position << std::endl;
            }
            else if (readDevice2Channel == 1) { // lh腿的三个电机
                uint16_t positionRaw = (static_cast<uint16_t>(data2[1]) << 8) | static_cast<uint16_t>(data2[2]);
                double position = (static_cast<double>(positionRaw) / 65535.0) * (8.0 * M_PI) - (4.0 * M_PI);

                uint16_t velocityRaw = (static_cast<uint16_t>(data2[3]) << 4) | (static_cast<uint16_t>(data2[4]) >> 4);
                double velocity = (static_cast<double>(velocityRaw) - 2047.0) / 2047.0 * 30.0 * M_PI;

                uint16_t torqueRaw = ((static_cast<uint16_t>(data2[4] & 0x0F)) << 8) | static_cast<uint16_t>(data2[5]);
                double torqueNm = (static_cast<double>(torqueRaw) - 2047.0) / 2047.0 * 48.0;

                unsigned char id = data2[0];
                int motorIndex = static_cast<int>(id) - 1;

                if (motorIndex >= 0 && motorIndex < 3) {
                    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁
                    _state.lh_q[motorIndex] = position;
                    _state.lh_v[motorIndex] = velocity;
                    _state.lh_tau[motorIndex] = torqueNm;
                } 

                // std::cout << "canID:" << id << "position:" << position << std::endl;
            }
        }
        else if (result == -1) {
            std::cerr << "模块2 recv timeout!!!" << std::endl;
        }
    }
}

LegState LegControl::getState() const {
    std::lock_guard<std::mutex> lock(stateMutex); // 锁定互斥锁以安全地读取_state
    return _state;
}

