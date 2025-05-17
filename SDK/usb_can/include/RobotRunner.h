// include/RobotRunner.h

#ifndef ROBOTRUNNER_H
#define ROBOTRUNNER_H

#include "LegControl.h"
#include "lcm_related/motor_data.hpp"
#include "lcm_related/motor_command.hpp"
#include "lcm_related/imu_data.hpp"
#include "lcm/lcm-cpp.hpp"
#include "JointCommand.h"
#include "LordImu.h"
#include <chrono>
#include <fstream>

class RobotRunner {
public:
    RobotRunner();
    ~RobotRunner();

    // 初始化机器人
    void initialize();

    // 站起来
    void stand_up();

    // lcm接口
    void lcm_run();
    void lcm_recv();
    void lcm_pub();

    void handleMotorLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, // suscribe的回调函数，接收电机指令
                        const motor_command* msg);

    void handleRecvLCM(); // 接收线程的回调函数

    // 停止运行
    void stop();

private:
    LegControl lc;
    LordImu imu;

    LegState s;
    ImuData d;

    // 控制参数
    static constexpr double dt = 0.002; // 2ms 循环周期
    static constexpr double alpha = 0.2; // 滤波系数
    static constexpr double kp = 75.0;  // 比例增益
    static constexpr double kd = 3.0;    // 微分增益
    // const double goal_positions[12] =  {
    //         0.0, -0.72, 1.44,  
    //         0.0, 0.72, -1.44,  
    //         0.0, 0.72, -1.44,  
    //         0.0, -0.72, 1.44
    // };

    const double goal_positions[12] =  {
            0.0, -0.6, 1.2,  
            0.0, 0.6, -1.2,  
            0.0, 0.6, -1.2,  
            0.0, -0.6, 1.2
    };

    const double T = 3.0;

    // 机器人状态
    double tauff[12];

    // 0命令
    double tauff0[12];
    double pos_des0[12];
    double vel_des0[12];
    double Kp0[12];
    double Kd0[12];

    double running_time;
    double start_pos[12];
    double current_pos[12];
    double pos_desired[12];
    double pos_err[12];
    double vel_err[12];

    // 位置和速度
    double previous_pos[12];
    double filtered_vel[12];

    // 日志相关
    std::ofstream dataFile;

    // 互斥锁和条件变量
    std::mutex _logMutex; // 保护共享变量的互斥锁
    std::condition_variable _logCV; // 条件变量
    std::atomic<bool> _logReady{false}; // 日志数据准备好标志
    std::atomic<bool> _logQuit{false}; // 日志线程退出标志

    std::thread _logThread; // 日志线程

    void logWorker(); // 日志线程工作函数

    // lcm相关
    lcm::LCM _lcm;
    motor_data _motordata;
    imu_data _imudata;
    motor_command _cmd;
    std::thread _recvLcmThread; // lcm的subscribe线程
    std::atomic<bool> _recvLcmQuit{false};

    JointCommand _jointcommand;
    std::mutex _jointcommandMutex;
};

#endif // ROBOTRUNNER_H
