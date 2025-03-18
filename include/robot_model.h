#pragma once
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
#include <vector>
#include "state_estimator.h"  // 包含状态估计器头文件
#include "utils.h"
#include <memory>


class RobotModel {
public:
    RobotModel(int time_step);
    ~RobotModel();
    
    void initializeDevices();
    void updateSensorData();
    void applyTorques(const double* torques);
    void slowToStandingPosition(); // 站立姿态渐进方法
    void applyDamping(double damping_scale = 1.0);

    double last_torque_time = 0; // 记录最后收到力矩的时间

    double base_ang_vel[3];
    double gravity[3];
    double motor_data[36];
    double ACTIONS[12];
    double motor_data_error[12];
    bool standfinish = false; // 是否完成站立姿态渐进
    int time_step_;
    int contact_count = 0; // 联接计数
    
    // 新增方法
    const std::vector<float>& getJointPositions() const { return joint_positions_; }
    const std::vector<float>& getJointVelocities() const { return joint_velocities_; }
    const std::vector<float>& getJointTorques() const { return joint_torques_; }

    const std::vector<bool>& getContactStatus() const { return contact_status_; }
    const Eigen::Vector3f& getTorsoVelocity() const { return torso_velocity_; }
    const Eigen::Vector3f& getAngularVelocity() const { return angular_velocity_; }
    const Eigen::Vector3f& getKinematic_vel() const { return kinematic_vel; }
    const std::vector<Eigen::Vector3f>& getFootForces() const { return foot_forces_; }
    const Eigen::Quaternionf& getOrientation() const { return orientation_; }


private:
    
    WbDeviceTag motors_[12];
    WbDeviceTag sensors_[12];
    WbDeviceTag accelerometer_;
    WbDeviceTag imu_;
    WbDeviceTag gyro_;

    double standing_kp[12];       // 各关节独立P增益
    double standing_kd[12];       // 各关节独立D增益
    double init_joint_pos[12];    // 初始关节位置
    double target_joint_pos[12];  // 目标关节位置
    double transition_time = 2.0; // 过渡时间(秒)
    double elapsed_time = 0.0;    // 已用时间
    bool initialized = false;     // 是否初始化
    constexpr static double BASE_DAMPING = 2.0; // 基础阻尼系数

    // 坐标系转换参数（根据URDF关节轴方向调整）
    const float hip_axis_sign = 1.0f;       // 髋关节旋转轴方向（URDF中axis xyz="1 0 0"）
    const float thigh_axis_sign = 1.0f;     // 大腿关节旋转轴方向（URDF中axis xyz="0 1 0"）
    const float calf_axis_sign = 1.0f;      // 小腿关节旋转轴方向（URDF中axis xyz="0 1 0"）
    // 从URDF提取的精确参数
    const float thigh_length = 0.088241f;   // 大腿长度 = thigh_joint origin y轴位移
    const float calf_length = 0.22023f;     // 小腿长度 = calf_joint origin z轴绝对值
    const float hip_offset = 0.05f;         // 髋关节侧向偏移 = hip_joint origin y轴值

    std::vector<float> joint_positions_;
    std::vector<float> joint_velocities_; 
    std::vector<float> joint_torques_;

    double motor_data_last[36];
    double default_dof_pos[12] = {
        // FL
        0.1,    // hip
        0.8,    // thigh
        -1.5,   // calf
        
        // FR
        -0.1,   // hip  
        0.8,    // thigh
        -1.5,   // calf
        
        // RL
        0.1,    // hip
        0.8,    // thigh
        -1.5,   // calf
        
        // RR
        -0.1,   // hip
        0.8,    // thigh
        -1.5    // calf
    };
    // double acceleration[3];
    // double quaternion[4];
    // double rpy[3];
    // double angular_velocity[3];
    double ang_vel[3];
    double v[3] = {0, 0, -1};
    double alpha = 0.1; // 滤波系数
    double Kp_stiffness = 20;
    double D = 0.5;
    const double ACTION_SCALE = 0.25; // 动作缩放系数
    const double velocity_decay_factor = 0.99; // 速度衰减系数
    double torque;
    // 新增速度滤波参数
    const double VEL_FILTER_CUTOFF = 20.0; // Hz
    double vel_filter_coeff = 1.0 / (1.0 + 1.0/(2*M_PI*VEL_FILTER_CUTOFF*time_step_/1000.0));

    std::vector<Eigen::Vector3f> foot_forces_;  // 足底力（世界坐标系）
    std::vector<bool> contact_status_;          // 接触状态
    Eigen::Vector3f torso_velocity_;            // 融合后的躯干速度
    Eigen::Vector3f angular_velocity_;
    Eigen::Vector3f kinematic_vel;

    Eigen::Quaternionf orientation_; // 新增成员变量

    Eigen::Vector3f kalman_state;  // 状态向量 [vx, vy, vz]
    Eigen::Matrix3f kalman_P;      // 协方差矩阵 (3x3)
    Eigen::Matrix3f kalman_Q;      // 过程噪声 (3x3)
    Eigen::Matrix3f kalman_R;      // 观测噪声 (3x3)
    Eigen::Vector3f acc_bias = Eigen::Vector3f::Zero();
    std::unique_ptr<StateEstimator> state_estimator_;

    // 新增方法
    // Eigen::Matrix3f computeLegJacobian(int leg) const;
    // Eigen::Vector3f forwardKinematics(int leg) const;
    // void kalmanUpdate(const Eigen::Vector3f& acc, const Eigen::Vector3f& obs_vel, float dt);
    void printDebugInfo() const;
};