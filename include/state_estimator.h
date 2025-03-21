// state_estimator.h
#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <array>

using namespace Eigen;

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<float, 12, 1> Vector12f;

struct RobotParams {
    // 运动学参数
    double hip_offset = 0.05;       // 髋关节侧向偏移(m)
    double thigh_length = 0.088241; // 大腿长度(m)
    double calf_length = 0.22023;   // 小腿长度(m)
    
    // 关节轴方向符号
    double hip_axis_sign = 1.0;     // X轴
    double thigh_axis_sign = 1.0;  // Y轴
    double calf_axis_sign = 1.0;   // Y轴
}; 

class StateEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 构造/初始化
    StateEstimator(const RobotParams& params, double dt);
    
    // 主更新接口
    void update(const Vector4d& quat,          // IMU四元数 [w,x,y,z]
                const Vector3d& gyro,          // 角速度 (rad/s)
                const Vector3d& accel,         // 加速度 (m/s²)
                const Vector12d& joint_pos,    // 12维关节位置
                const Vector12d& joint_torque, // 12维关节力矩
                std::array<bool,4>& contact_states); // 接触状态

    // 状态获取接口
    Vector3d get_linear_velocity() const;    // 线速度 (m/s)
    Vector3d get_angular_velocity() const;   // 角速度 (rad/s)
    double get_standing_height() const;      // 站高 (m)
    Vector2d get_ground_slope() const;       // 地面坡度 (rad)
    const std::vector<Eigen::Vector3d>& get_contact_forces() const;

private:
    // 核心估计算法
    Vector3d compute_kinematic_velocity(const Vector12d& joint_pos,
                                       const std::array<bool,4>& contact_states);
    void update_kalman_filter(const Vector3d& accel);
    Matrix3d compute_jacobian(int leg_id, const Vector3d& joint_angles) const;
    Vector3d forward_kinematics(int leg_id, const Vector3d& angles) const;
    
    // 辅助估计函数
    void estimate_contact_forces(const Vector12d& joint_pos,
                                const Vector12d& joint_torque,
                                std::array<bool,4>& contact_states);
    void estimate_height_and_slope(const Vector12d& joint_pos,
                                  const std::array<bool,4>& contact_states);

    // 参数配置
    RobotParams params_;
    double dt_;
    
    // 滤波器状态
    Vector12d prev_joint_pos_;
    Vector12d joint_vel_;
    bool first_update_;
    
    // 估计状态
    Matrix3d body_rotation_;
    Vector3d kf_velocity_;
    Matrix3d kf_P_;
    const Matrix3d kf_Q_;
    const Matrix3d kf_R_kinematic_;
    const Matrix3d kf_R_accel_;
    
    // 输出状态
    Vector3d filtered_velocity_;
    Vector3d angular_velocity_;
    std::vector<Eigen::Vector3d> contact_forces_; 
    double standing_height_;
    Vector2d ground_slope_;
};

#endif // STATE_ESTIMATOR_H