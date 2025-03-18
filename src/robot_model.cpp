#include "robot_model.h"
#include <iostream>

RobotModel::RobotModel(int time_step) : time_step_(time_step) {
    // 设备初始化
    // 获取电机和位置传感器设备
    const char* motor_names[] = {
        // 前左腿
        "FL_hip_joint", 
        "FL_thigh_joint", 
        "FL_calf_joint",
        
        // 前右腿
        "FR_hip_joint", 
        "FR_thigh_joint", 
        "FR_calf_joint",
        
        // 后左腿 
        "RL_hip_joint", 
        "RL_thigh_joint", 
        "RL_calf_joint",
        
        // 后右腿
        "RR_hip_joint", 
        "RR_thigh_joint", 
        "RR_calf_joint"
    };
    
    const char* sensor_names[] = {
        // 前左腿
        "FL_hip_joint_sensor",
        "FL_thigh_joint_sensor",
        "FL_calf_joint_sensor",
        
        // 前右腿
        "FR_hip_joint_sensor",
        "FR_thigh_joint_sensor",
        "FR_calf_joint_sensor",
        
        // 后左腿
        "RL_hip_joint_sensor",
        "RL_thigh_joint_sensor",
        "RL_calf_joint_sensor",
        
        // 后右腿
        "RR_hip_joint_sensor",
        "RR_thigh_joint_sensor",
        "RR_calf_joint_sensor"
    };

    for (int i = 0; i < 12; ++i) {
        motors_[i] = wb_robot_get_device(motor_names[i]);
        sensors_[i] = wb_robot_get_device(sensor_names[i]);
    }

    accelerometer_ = wb_robot_get_device("accelerometer");
    imu_ = wb_robot_get_device("inertial unit");
    gyro_ = wb_robot_get_device("gyro");

    // 初始化容器大小
    joint_positions_.resize(12);      // 12 个关节的位置
    joint_velocities_.resize(12);     // 12 个关节的速度
    joint_torques_.resize(12);        // 12 个关节的力矩

    // 设置站立目标角度 (按关节顺序)
    constexpr double DEFAULT_JOINT_ANGLES[12] = {
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
    
    // 拷贝目标位置
    std::copy(DEFAULT_JOINT_ANGLES, DEFAULT_JOINT_ANGLES+12, target_joint_pos);
    
    // 设置站立控制增益
    constexpr double KP[12] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    constexpr double KD[12] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::copy(KP, KP+12, standing_kp);
    std::copy(KD, KD+12, standing_kd);

    // 初始化卡尔曼滤波器
    kalman_state.setZero();
    kalman_P = Eigen::Matrix3f::Identity() * 0.1f; // 初始化协方差
    kalman_Q = Eigen::Matrix3f::Identity() * 0.01f;
    kalman_R = Eigen::Matrix3f::Identity() * 0.1f;

    // 初始化状态估计器
    RobotParams params;
    params.hip_offset = hip_offset;
    params.thigh_length = thigh_length;
    params.calf_length = calf_length;
    params.hip_axis_sign = hip_axis_sign;
    params.thigh_axis_sign = thigh_axis_sign;
    params.calf_axis_sign = calf_axis_sign;
    
    state_estimator_ = std::make_unique<StateEstimator>(
        params, 
        time_step_ / 1000.0  // 转换为秒
    );

    initializeDevices();
}

RobotModel::~RobotModel() {
    // 释放资源
    // for(int i = 0; i < 12; ++i) {
        // wb_motor_disable_torque_feedback(motors_[i]);
        // wb_position_sensor_disable(sensors_[i]);
    // }
}

void RobotModel::slowToStandingPosition() {
    const double dt = time_step_ / 1000.0; // 转换为秒
    
    if (!initialized) {
        // 第一次调用时记录初始位置
        for(int i=0; i<12; ++i){
            init_joint_pos[i] = joint_positions_[i];
        }
        initialized = true;
        elapsed_time = 0.0;
    }
    
    elapsed_time += dt;
    
    if(elapsed_time < transition_time) {
        // 计算插值比例 (0~1)
        double ratio = std::min(elapsed_time / transition_time, 1.0);
        
        // 计算期望位置
        double desired_pos[12];
        for(int i=0; i<12; ++i){
            desired_pos[i] = init_joint_pos[i] + 
                           (target_joint_pos[i] - init_joint_pos[i]) * ratio;
        }
        
        // 应用PD控制
        for(int i=0; i<12; ++i){
            double error = desired_pos[i] - joint_positions_[i];
            double torque = standing_kp[i] * error - 
                          standing_kd[i] * joint_velocities_[i];
            if(i==0||i==3||i==1||i==2||i==7||i==8) torque*=-1;
            wb_motor_set_torque(motors_[i], torque);
            if(i==0||i==3||i==1||i==2||i==7||i==8) motor_data_last[i] = wb_position_sensor_get_value(sensors_[i])*(-1);
            else motor_data_last[i] = wb_position_sensor_get_value(sensors_[i]);
            motor_data_last[i + 24] = desired_pos[i]/ACTION_SCALE; // 这里是将站立后最后一刻的目标位置作为action回传给强化学习策略
            joint_torques_[i] = torque;
        }
    } else {
        // 过渡完成后保持目标位置
        for(int i=0; i<12; ++i){
            double error = target_joint_pos[i] - joint_positions_[i];
            double torque = standing_kp[i] * error - 
                          standing_kd[i] * joint_velocities_[i];
            if(i==0||i==3||i==1||i==2||i==7||i==8) torque*=-1;
            wb_motor_set_torque(motors_[i], torque);
            if(i==0||i==3||i==1||i==2||i==7||i==8) motor_data_last[i] =wb_position_sensor_get_value(sensors_[i])*(-1);
            else motor_data_last[i] = wb_position_sensor_get_value(sensors_[i]);
            motor_data_last[i + 24] = target_joint_pos[i]/ACTION_SCALE;
            joint_torques_[i] = torque;
            standfinish = true;
        }
    }
}

void RobotModel::initializeDevices() {
    // 初始化电机和传感器
    for(int i = 0; i < 12; ++i) {
        wb_motor_enable_torque_feedback(motors_[i], time_step_);
        wb_position_sensor_enable(sensors_[i], time_step_);
        wb_motor_set_velocity(motors_[i], 0);   // 设置零速度
        // motor_data_last[i] = wb_position_sensor_get_value(sensors_[i]);
        // std::cout << "Default position for motor " << i << ": " << default_dof_pos[i] << std::endl;
    }
    // std::cout << "Default positions set." << std::endl;

    // 初始化加速度计、IMU、陀螺仪
    wb_accelerometer_enable(accelerometer_, time_step_);
    std::cout << "Accelerometer enabled." << std::endl;
    wb_inertial_unit_enable(imu_, time_step_);
    std::cout << "IMU enabled." << std::endl;
    wb_gyro_enable(gyro_, time_step_);
    std::cout << "Gyro enabled." << std::endl;
    
    wb_robot_step(time_step_);
}

// Eigen::Vector3f RobotModel::forwardKinematics(int leg) const {

//     float q_hip = hip_axis_sign * joint_positions_[leg*3];
//     float q_thigh = thigh_axis_sign * joint_positions_[leg*3+1];
//     float q_calf = calf_axis_sign * joint_positions_[leg*3+2];

//     // 正运动学计算（与TerrainEstimator一致）
//     Eigen::Vector3f pos;
//     pos.x() = hip_offset * ((leg % 2 == 0) ? 1 : -1);
//     pos.y() = thigh_length * sin(q_thigh) + calf_length * sin(q_thigh + q_calf);
//     pos.z() = thigh_length * cos(q_thigh) + calf_length * cos(q_thigh + q_calf);

//     // 应用髋关节旋转
//     Eigen::AngleAxisf hip_rot(q_hip, Eigen::Vector3f::UnitX());
//     return hip_rot * pos;
// }

// Eigen::Matrix3f RobotModel::computeLegJacobian(int leg) const {
//     // 改用解析法计算雅可比矩阵（提高精度）
//     const float l1 = thigh_length;
//     const float l2 = calf_length;
    
//     float q_hip = hip_axis_sign * joint_positions_[leg*3];
//     float q_thigh = thigh_axis_sign * joint_positions_[leg*3+1];
//     float q_calf = calf_axis_sign * joint_positions_[leg*3+2];

//     Eigen::Matrix3f J;
    
//     // 髋关节影响 (绕x轴旋转)
//     J(0,0) = 0;
//     J(1,0) = -l1*sin(q_thigh) - l2*sin(q_thigh+q_calf);
//     J(2,0) = l1*cos(q_thigh) + l2*cos(q_thigh+q_calf);
    
//     // 大腿关节影响 (绕y轴旋转)
//     J(0,1) = 0;
//     J(1,1) = l1*cos(q_thigh) + l2*cos(q_thigh+q_calf);
//     J(2,1) = l1*sin(q_thigh) + l2*sin(q_thigh+q_calf);
    
//     // 小腿关节影响 (绕y轴旋转)
//     J(0,2) = 0;
//     J(1,2) = l2*cos(q_thigh+q_calf);
//     J(2,2) = l2*sin(q_thigh+q_calf);

//     // 应用髋关节旋转矩阵
//     Eigen::Matrix3f R_hip;
//     R_hip << 1, 0, 0,
//              0, cos(q_hip), -sin(q_hip),
//              0, sin(q_hip), cos(q_hip);
    
//     return R_hip * J;
// }

void RobotModel::updateSensorData() {
    contact_status_.assign(4, false); // 确保每次更新都重置状态

    // 积分计算线速度（带高通滤波） ======== 改进积分逻辑 ========
    double dt = time_step_ / 1000.0;

    // 更新关节数据 ======== 优化数据流 ========
    for (int i = 0; i < 12; ++i) {
        // 获取原始传感器数据
        double new_pos = wb_position_sensor_get_value(sensors_[i]);
        if(i==0||i==3||i==1||i==2||i==7||i==8) new_pos*=-1;
        double new_vel = (new_pos - motor_data_last[i]) / dt;

        // 关节速度滤波
        static double filtered_vel[12] = {0};
        filtered_vel[i] = vel_filter_coeff * filtered_vel[i] + (1 - vel_filter_coeff) * new_vel;
        
       
        // 更新数据存储
        motor_data[i] = new_pos;
        motor_data_error[i] = new_pos - default_dof_pos[i];
        motor_data[i + 12] = filtered_vel[i];
        motor_data[i + 24] = motor_data_last[i + 24];
        
        // 转换到float类型
        joint_positions_[i] = static_cast<float>(motor_data[i]);
        joint_velocities_[i] = static_cast<float>(motor_data[i + 12]);
        
        // 更新历史数据
        motor_data_last[i] = motor_data[i];
        motor_data_last[i + 12] = motor_data[i + 12];
    }
    // std::cout << "motor data updated." << std::endl;

    std::array<bool, 4> contact_states_array = {false, false, false, false};

    // 获取原始传感器数据
    const double *accel = wb_accelerometer_get_values(accelerometer_);
    const double *quat = wb_inertial_unit_get_quaternion(imu_);
    const double *gyro = wb_gyro_get_values(gyro_);

    quat_rotate_inverse(quat, v, gravity);

    // 转换关节数据到Eigen类型
    Vector12d joint_pos, joint_torque;
    for(int i=0; i<12; ++i){
        joint_pos[i] = static_cast<double>(joint_positions_[i]);
        joint_torque[i] = static_cast<double>(joint_torques_[i]);
    }

    // 调用状态估计器更新
    state_estimator_->update(
        Eigen::Map<const Vector4d>(quat),       // 四元数
        Eigen::Map<const Vector3d>(gyro),       // 角速度
        Eigen::Map<const Vector3d>(accel),      // 加速度
        joint_pos,                              // 关节位置
        joint_torque,                           // 关节力矩
        contact_states_array                    // 接触状态（将被更新）
    );

    // 将array转换回vector
    contact_status_.assign(
        contact_states_array.begin(), 
        contact_states_array.end()
    );

    // 获取估计结果
    Eigen::Vector3d linear_vel = state_estimator_->get_linear_velocity();
    Eigen::Vector3d angular_vel = state_estimator_->get_angular_velocity();
    const auto& contact_forces = state_estimator_->get_contact_forces();

    // 更新RobotModel成员变量
    torso_velocity_ = linear_vel.cast<float>();
    angular_velocity_ = angular_vel.cast<float>();
    
    // 转换接触力到世界坐标系
    foot_forces_.resize(4);
    for(int leg=0; leg<4; ++leg){
        foot_forces_[leg] = contact_forces[leg].cast<float>();
    }

    // 其他需要保持的调试信息
    printDebugInfo();
}

// // 卡尔曼滤波
// void RobotModel::kalmanUpdate(const Eigen::Vector3f& acc_body, 
//     const Eigen::Vector3f& obs_vel_world,
//     float dt) {
    
//     // 1. 偏差补偿
//     Eigen::Vector3f corrected_acc = acc_body - acc_bias;
    
//     // 2. 过程模型预测
//     Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
//     Eigen::Matrix3f B = Eigen::Matrix3f::Identity() * dt;
    
//     kalman_state = F * kalman_state + B * corrected_acc;
//     kalman_P = F * kalman_P * F.transpose() + kalman_Q;

//     // 3. 动态噪声调整
//     if(corrected_acc.norm() < 0.1f) {
//         kalman_Q.diagonal().setConstant(0.1f);
//     }

//     // 4. 观测有效性判断
//     bool has_valid_obs = false;
//     Eigen::Vector3f effective_obs = obs_vel_world;
    
//     // 条件1：有效运动观测
//     if(obs_vel_world.norm() > 0.1f) {
//         has_valid_obs = true;
//     }
//     // 条件2：强制零速度观测
//     else if(corrected_acc.norm() < 0.05f && kalman_state.norm() < 0.1f) {
//         effective_obs.setZero();
//         has_valid_obs = true;
//     }

//     // 5. 观测更新
//     if(has_valid_obs) {
//         Eigen::Matrix3f H = Eigen::Matrix3f::Identity();
//         Eigen::Vector3f y = effective_obs - H * kalman_state;
//         Eigen::Matrix3f S = H * kalman_P * H.transpose() + kalman_R;
//         Eigen::Matrix3f K = kalman_P * H.transpose() * S.inverse();

//         kalman_state += K * y;
//         kalman_P = (Eigen::Matrix3f::Identity() - K * H) * kalman_P;
//     }

//     // 6. 协方差正定保护
//     kalman_P = 0.5f * (kalman_P + kalman_P.transpose());
//     kalman_P.diagonal() = kalman_P.diagonal().cwiseMax(0.001f);

//     // 7. 零速度锁定
//     static float static_duration = 0.0f;
//     if(corrected_acc.norm() < 0.05f && kalman_state.norm() < 0.1f) {
//         static_duration += dt;
//         if(static_duration > 1.0f) {
//             kalman_state.setZero();
//             kalman_P = Eigen::Matrix3f::Identity() * 0.1f;
//         }
//     } else {
//         static_duration = 0.0f;
//     }
// }

void RobotModel::applyTorques(const double* torques) {
    for(int i=0; i<12; ++i){
        torque = 0;
        torque = Kp_stiffness * (torques[i] * ACTION_SCALE + default_dof_pos[i] - motor_data[i]) - D * motor_data[i+12];
        // std::cout << "torque: " << torque << std::endl;
        torque = torque > 48.0 ? 48.0 : torque;
        torque = torque < -48.0 ? -48.0 : torque;
        if(i==0||i==3||i==1||i==2||i==7||i==8) torque*=-1;
        wb_motor_set_torque(motors_[i], torque);
        motor_data_last[i + 24] = torques[i];
        ACTIONS[i] = torques[i]; // 保存动作
        joint_torques_[i] = static_cast<float>(torque);
    }
}

// 实现阻尼控制方法
void RobotModel::applyDamping(double damping_scale) {
    constexpr double MAX_TORQUE = 15.0; // 安全力矩限制
    
    for(int i=0; i<12; ++i){
        // 计算阻尼力矩：-阻尼系数 * 速度
        torque = -BASE_DAMPING * damping_scale * joint_velocities_[i];
        
        // 添加位置保持项（可选）
        double pos_error = target_joint_pos[i] - joint_positions_[i];
        torque += 50.0 * pos_error; // 弱刚度项
        
        // 力矩限幅
        torque = std::clamp(torque, -MAX_TORQUE, MAX_TORQUE);
        
        if(i==0||i==3||i==1||i==2||i==7||i==8) torque*=-1;
        wb_motor_set_torque(motors_[i], torque);
        joint_torques_[i] = static_cast<float>(torque);
    }
}

// 在RobotModel中添加调试输出
void RobotModel::printDebugInfo() const {
    std::cout << "=== Contact Status ===" << std::endl;
    for(int leg=0; leg<4; ++leg){
        std::cout << "Leg " << leg << ": " 
                  << (contact_status_[leg] ? "Contact" : "Swing") 
                  << " Force: " << foot_forces_[leg].transpose() << " N\n";
    }
    
    std::cout << "\n=== Velocity Estimate ===" << std::endl;
    std::cout << "Linear: " << torso_velocity_.transpose() << " m/s\n";
    std::cout << "Angular: " << angular_velocity_.transpose() << " rad/s\n";
    
}