// src/RobotRunner.cpp

#include "../include/RobotRunner.h"
#include <iostream>
#include <thread>
#include <cmath>
#include <iomanip> // For setting precision

// 构造函数
RobotRunner::RobotRunner()
    : tauff{0.0}, tauff0{0.0}, 
      pos_des0{0.0}, vel_des0{0.0}, Kp0{0.0}, Kd0{0.0},
      running_time(0.0),
      start_pos{0.0}, current_pos{0.0},
      pos_desired{0.0}, pos_err{0.0},
      vel_err{0.0}, previous_pos{0.0},
      filtered_vel{0.0},
      _lcm("udpm://239.255.76.67:7667?ttl=1") // lcm初始化
{
    _logThread = std::thread(&RobotRunner::logWorker, this);
}

// 析构函数
RobotRunner::~RobotRunner() {
    stop();

    _recvLcmQuit = true;
    if (_recvLcmThread.joinable()) {
        _recvLcmThread.join();
    }

    // 关闭日志线程
    _logQuit = true;
    _logCV.notify_one(); // 唤醒日志线程以便退出
    if (_logThread.joinable()) {
        _logThread.join();
    }

    if (dataFile.is_open()) {
        dataFile.close();
    }
}

// 日志线程工作函数
void RobotRunner::logWorker() {
    // 打开文件
    dataFile.open("data1.csv", std::ios::out | std::ios::out);
    if (!dataFile.is_open()) {
        std::cerr << "无法打开日志文件！" << std::endl;
        return;
    }

    // 设置输出精度（可选）
    dataFile << std::fixed << std::setprecision(6);

    while (!_logQuit) {
        std::unique_lock<std::mutex> lock(_logMutex);
        _logCV.wait(lock, [this]() { return _logReady || _logQuit; });

        if (_logQuit && !_logReady) {
            break;
        }

        if (_logReady) {
            // 记录当前的运行时间和相关变量
            double time = running_time;
            double qDesCopy[12];
            double qCopy[12];
            double linearAccCopy[3];
            double angularVelCopy[3];
            double rpyCopy[3];
            double quatCopy[4];

            // 复制数据以减少锁持有时间
            for (int i = 0; i < 12; ++i) {
                qDesCopy[i] = _jointcommand.qDes[i];
                qCopy[i] = _motordata.q[i];
            }

            for(int i = 0; i < 3; ++i) {
                linearAccCopy[i] = _imudata.linearAcc[i];
                angularVelCopy[i] = _imudata.angularVel[i];
                rpyCopy[i] = _imudata.rpy[i];
            }

            for(int i = 0; i < 4; ++i) {
                quatCopy[i] = _imudata.quat[i];
            }

            _logReady = false; // 重置标志
            lock.unlock();

            // 写入文件           
            dataFile << time << ",";
            for (int i = 0; i < 12; ++i) {
                dataFile << qDesCopy[i] << ","; 
                dataFile << qCopy[i] << ",";
            }
            for(int i = 0; i < 3; ++i) {
                dataFile << linearAccCopy[i] << ",";
            }
            for(int i = 0; i < 3; ++i) {
                dataFile << angularVelCopy[i] << ",";
            }
            for(int i = 0; i < 3; ++i) {
                dataFile << rpyCopy[i] << ",";
            }
            for(int i = 0; i < 4; ++i) {
                dataFile << quatCopy[i] << ",";
            }
            dataFile << "\n";
        }
    }

    // 写入剩余的数据
    if (_logReady) {
        double time = running_time;
        double qDesCopy[12];
        double qCopy[12];
        double linearAccCopy[3];
        double angularVelCopy[3];
        double rpyCopy[3];
        double quatCopy[4];

        for (int i = 0; i < 12; ++i) {
            qDesCopy[i] = _jointcommand.qDes[i];
            qCopy[i] = _motordata.q[i];
        }

        for(int i = 0; i < 3; ++i) {
            linearAccCopy[i] = _imudata.linearAcc[i];
            angularVelCopy[i] = _imudata.angularVel[i];
            rpyCopy[i] = _imudata.rpy[i];
        }

        for(int i = 0; i < 4; ++i) {
            quatCopy[i] = _imudata.quat[i];
        }

        dataFile << time << ",";
        for (int i = 0; i < 12; ++i) {
            dataFile << qDesCopy[i] << ","; 
            dataFile << qCopy[i] << ",";
        }
        for(int i = 0; i < 3; ++i) {
            dataFile << linearAccCopy[i] << ",";
        }
        for(int i = 0; i < 3; ++i) {
            dataFile << angularVelCopy[i] << ",";
        }
        for(int i = 0; i < 3; ++i) {
            dataFile << rpyCopy[i] << ",";
        }
        for(int i = 0; i < 4; ++i) {
            dataFile << quatCopy[i] << ",";
        }
        dataFile << "\n";
    }

    dataFile.close();
}

// 初始化机器人
void RobotRunner::initialize() {
    lc.LegInit(); // 打开设备,使能电机,开启两块硬件板接收的线程
    imu.initImu(); // 打开并初始化imu,开启imu接收的线程

    lcm_recv(); // 开启lcm接收的线程
}

// 停止运行
void RobotRunner::stop() {
    lc.StopLeg();
}

void RobotRunner::lcm_pub(){
    _lcm.publish("MOTOR_DATA", &_motordata);
    _lcm.publish("IMU_DATA", &_imudata);
}

void RobotRunner::lcm_recv(){
    _lcm.subscribe("MOTOR_COMMAND", &RobotRunner::handleMotorLCM, this);
    _recvLcmThread = std::thread(&RobotRunner::handleRecvLCM, this);
}

void RobotRunner::handleMotorLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const motor_command* msg) {
    (void)rbuf;
    (void)chan;
    std::lock_guard<std::mutex> lock(_jointcommandMutex);
    _jointcommand.set(msg);
}

void RobotRunner::handleRecvLCM() {
    while (!_recvLcmQuit) {
        _lcm.handle();
    }
}

void RobotRunner::lcm_run() {
    if (running_time <= 0.2) {
        // lc.send(tauff0);
        lc.send2(pos_des0, vel_des0, Kp0, Kd0, tauff0);  // 先发0

        s = lc.getState(); // 获取最新的电机状态
        d = imu.getImuData(); // 获取最新的imu数据

        // 给lcm的结构体赋值
        _motordata.q[0] = s.rf_q[0];
        _motordata.q[1] = s.rf_q[1];
        _motordata.q[2] = s.rf_q[2];
        _motordata.q[3] = s.lf_q[0];
        _motordata.q[4] = s.lf_q[1];
        _motordata.q[5] = s.lf_q[2];
        _motordata.q[6] = s.rh_q[0];
        _motordata.q[7] = s.rh_q[1];
        _motordata.q[8] = s.rh_q[2];
        _motordata.q[9] = s.lh_q[0];
        _motordata.q[10] = s.lh_q[1];
        _motordata.q[11] = s.lh_q[2];

        for (int i = 0; i < 3; i++) {
            _imudata.linearAcc[i] = d.linearAcc[i];
            _imudata.angularVel[i] = d.angularVel[i];
            _imudata.rpy[i] = d.rpy[i];
        }     

        for (int i = 0; i < 4; i++) {
            _imudata.quat[i] = d.quat[i];
        }

        // 用于第一次微分求速度
        for (int i = 0; i < 12; i++) {
            previous_pos[i] = _motordata.q[i];
        }

        lcm_pub(); // 发布状态
    }
    else {
        s = lc.getState(); // 获取最新的电机状态
        d = imu.getImuData(); // 获取最新的imu数据

        // 给lcm的结构体赋值
        _motordata.q[0] = s.rf_q[0];
        _motordata.q[1] = s.rf_q[1];
        _motordata.q[2] = s.rf_q[2];
        _motordata.q[3] = s.lf_q[0];
        _motordata.q[4] = s.lf_q[1];
        _motordata.q[5] = s.lf_q[2];
        _motordata.q[6] = s.rh_q[0];
        _motordata.q[7] = s.rh_q[1];
        _motordata.q[8] = s.rh_q[2];
        _motordata.q[9] = s.lh_q[0];
        _motordata.q[10] = s.lh_q[1];
        _motordata.q[11] = s.lh_q[2];

        for (int i = 0; i < 3; i++) {
            _imudata.linearAcc[i] = d.linearAcc[i];
            _imudata.angularVel[i] = d.angularVel[i];
            _imudata.rpy[i] = d.rpy[i];
        }     

        for (int i = 0; i < 4; i++) {
            _imudata.quat[i] = d.quat[i];
        }

        // 计算速度并应用低通滤波器
        for (int i = 0; i < 12; i++) {
            double velocity = (_motordata.q[i] - previous_pos[i]) / dt;
            // 应用低通滤波器
            _motordata.v[i] = alpha * velocity + (1.0 - alpha) * _motordata.v[i];
            previous_pos[i] = _motordata.q[i];

            // 不用滤波器
            // _motordata.v[i] = velocity;
           
        }

        lcm_pub(); // 发布状态

        // 力矩发给底层
        std::lock_guard<std::mutex> lock(_jointcommandMutex);
        // lc.send(_jointcommand.tauff);

        // 发送混合命令
        lc.send2(_jointcommand.qDes, _jointcommand.vDes, _jointcommand.Kp, _jointcommand.Kd, _jointcommand.tauff);
    }

    // 通知日志线程
    {
        std::lock_guard<std::mutex> lock(_logMutex);
        _logReady = true;
    }
    _logCV.notify_one();

    running_time += dt;
}
