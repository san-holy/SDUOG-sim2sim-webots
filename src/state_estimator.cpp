// state_estimator.cpp
#include "state_estimator.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

// 构造函数
StateEstimator::StateEstimator(const RobotParams& params, double dt)
    : params_(params), dt_(dt),
      kf_Q_(Matrix3d::Identity() * 0.05),
      kf_R_kinematic_(Matrix3d::Identity() * 0.1),
      kf_R_accel_(Matrix3d::Identity() * 0.1),
      kf_velocity_(Vector3d::Zero()),
      kf_P_(Matrix3d::Identity()),
      first_update_(true) {
    prev_joint_pos_.setZero();
    contact_forces_.resize(4, Vector3d::Zero());
}

// 主更新函数
void StateEstimator::update(const Vector4d& quat,
                          const Vector3d& gyro,
                          const Vector3d& accel,
                          const Vector12d& joint_pos,
                          const Vector12d& joint_torque,
                          std::array<bool,4>& contact_states) {

    // 关节速度计算
    if(!first_update_) {
        joint_vel_ = (joint_pos - prev_joint_pos_) / dt_;
    } else {
        joint_vel_.setZero();
        first_update_ = false;
    }
    prev_joint_pos_ = joint_pos;

    // 姿态更新
    Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    q.normalize();
    body_rotation_ = q.toRotationMatrix();
    angular_velocity_ = gyro;

    // 加速度处理
    Vector3d world_accel = body_rotation_ * accel - Vector3d(0, 0, 9.81);

    // 接触力估计
    estimate_contact_forces(joint_pos, joint_torque, contact_states);

    // 运动学速度估计
    Vector3d kinematic_vel = compute_kinematic_velocity(joint_pos, contact_states);

    // 卡尔曼滤波更新
    update_kalman_filter(world_accel);

    // 地形估计
    estimate_height_and_slope(joint_pos, contact_states);
}

// 运动学速度计算
Vector3d StateEstimator::compute_kinematic_velocity(const Vector12d& joint_pos,
                                                   const std::array<bool,4>& contact_states) {
    Vector3d avg_vel = Vector3d::Zero();
    int contact_count = 0;

    for(int leg = 0; leg < 4; ++leg) {
        if(contact_states[leg]) {
            Vector3d angles = joint_pos.segment<3>(leg * 3);
            Vector3d vel = joint_vel_.segment<3>(leg * 3);

            // 计算雅可比矩阵
            Matrix3d J = compute_jacobian(leg, angles);

            // 计算足端位置
            Vector3d foot_pos = forward_kinematics(leg, angles);

            // 计算足端速度
            Vector3d v_foot = J * vel;

            // 计算机体速度
            Vector3d v_body = -(v_foot + angular_velocity_.cross(foot_pos));

            // 转换到世界坐标系
            avg_vel += body_rotation_ * v_body;
            contact_count++;
        }
    }

    if (contact_count > 0) {
        return avg_vel / static_cast<double>(contact_count);
    } else {
        return Vector3d::Zero();
    }
}

// 卡尔曼滤波更新
void StateEstimator::update_kalman_filter(const Vector3d& accel) {
    // 预测步骤：加速度积分
    kf_velocity_ += accel * dt_;
    kf_P_ += kf_Q_;

    // 加速度计更新（仅保留加速度观测）
    Matrix3d K_accel = kf_P_ * (kf_P_ + kf_R_accel_).inverse();
    kf_velocity_ += K_accel * (accel * dt_ - kf_velocity_);
    kf_P_ = (Matrix3d::Identity() - K_accel) * kf_P_;

    filtered_velocity_ = kf_velocity_;
}

// 雅可比矩阵计算
Matrix3d StateEstimator::compute_jacobian(int leg_id, const Vector3d& joint_angles) const {
    const double q1 = joint_angles[0] * params_.hip_axis_sign;
    const double q2 = joint_angles[1] * params_.thigh_axis_sign;
    const double q3 = joint_angles[2] * params_.calf_axis_sign;

    const double l1 = params_.thigh_length;
    const double l2 = params_.calf_length;

    Matrix3d J;

    // 髋关节导数项 (X轴旋转)
    J(0,0) = 0.0;
    J(1,0) = -l1*sin(q1) - l2*sin(q1)*cos(q2);
    J(2,0) = l1*cos(q1) + l2*cos(q1)*cos(q2);

    // 大腿关节导数项 (Y轴旋转)
    J(0,1) = l2*sin(q2 + q3);
    J(1,1) = -l2*sin(q1)*sin(q2);
    J(2,1) = l2*cos(q1)*sin(q2);

    // 小腿关节导数项 (Y轴旋转)
    J(0,2) = l2*sin(q2 + q3);
    J(1,2) = -l2*sin(q1)*sin(q2 + q3);
    J(2,2) = l2*cos(q1)*sin(q2 + q3);

    // 应用关节方向符号
    J.col(0) *= params_.hip_axis_sign;
    J.col(1) *= params_.thigh_axis_sign;
    J.col(2) *= params_.calf_axis_sign;

    return J;
}

// 接触力估计
void StateEstimator::estimate_contact_forces(const Vector12d& joint_pos,
                                           const Vector12d& joint_torque,
                                           std::array<bool,4>& contact_states) {
    constexpr double FORCE_THRESHOLD = -5.0;  // 接触力阈值(N)
    
    for(int leg = 0; leg < 4; ++leg) {
        Vector3d angles = joint_pos.segment<3>(leg * 3);
        Matrix3d J = compute_jacobian(leg, angles);
        Vector3d tau = joint_torque.segment<3>(leg * 3);
                                        
        // 计算接触力
        Eigen::BDCSVD<Eigen::MatrixXd> svd;
        svd.compute(J.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);
        contact_forces_[leg] = svd.solve(tau);
        // 根据接触力判断支撑相
        contact_states[leg] = contact_forces_[leg].z() < FORCE_THRESHOLD;
    }
}

// 地形估计
void StateEstimator::estimate_height_and_slope(const Vector12d& joint_pos,
                                            const std::array<bool,4>& contact_states) {
    std::vector<Vector3d> contact_points;
    double total_z = 0.0;
    int contact_count = 0;

    // 收集接触点
    for(int leg = 0; leg < 4; ++leg) {
        if(contact_states[leg]) {
            Vector3d foot_pos = forward_kinematics(leg, joint_pos.segment<3>(leg * 3));
            contact_points.push_back(foot_pos);
            total_z += foot_pos.z();
            contact_count++;
        }
    }

    if(contact_count > 0) {
        // 高度估计
        standing_height_ = total_z / contact_count;

        // 平面拟合
        Vector3d centroid = Vector3d::Zero();
        for(const auto& p : contact_points) {
            centroid += p;
        }
        centroid /= contact_count;

        Matrix3d covariance = Matrix3d::Zero();
        for(const auto& p : contact_points) {
            Vector3d d = p - centroid;
            covariance += d * d.transpose();
        }

        // SVD分解求法向量
        JacobiSVD<Matrix3d> svd(covariance, ComputeFullU);
        Vector3d normal = svd.matrixU().col(2);
        if(normal.z() < 0) normal = -normal;

        // 转换到世界坐标系
        Vector3d world_normal = body_rotation_ * normal;

        // 计算坡度角
        ground_slope_.x() = atan2(world_normal.x(), world_normal.z());  // 横滚方向
        ground_slope_.y() = atan2(world_normal.y(), world_normal.z());  // 俯仰方向
    }
}

// 正运动学计算
Vector3d StateEstimator::forward_kinematics(int leg_id, const Vector3d& angles) const {
    const double side_sign = (leg_id % 2 == 0) ? 1.0 : -1.0;  // 腿号奇偶决定方向
    
    const double q1 = angles[0] * params_.hip_axis_sign;   // 髋关节
    const double q2 = angles[1] * params_.thigh_axis_sign; // 大腿关节
    const double q3 = angles[2] * params_.calf_axis_sign;  // 小腿关节

    // 髋关节偏移
    const Vector3d hip_offset(0.0, side_sign * params_.hip_offset, 0.0);

    // 旋转矩阵链
    Matrix3d R_hip   = AngleAxisd(q1, Vector3d::UnitX()).toRotationMatrix();
    Matrix3d R_thigh = AngleAxisd(q2, Vector3d::UnitY()).toRotationMatrix();
    Matrix3d R_calf  = AngleAxisd(q3, Vector3d::UnitY()).toRotationMatrix();

    // 运动学链式计算
    Vector3d position = hip_offset;
    position += R_hip * Vector3d(0.0, 0.0, params_.thigh_length);
    position += R_hip * R_thigh * R_calf * Vector3d(0.0, 0.0, params_.calf_length);

    return position;
}

// 状态获取接口
Vector3d StateEstimator::get_linear_velocity() const { return filtered_velocity_; }
Vector3d StateEstimator::get_angular_velocity() const { return angular_velocity_; }
double StateEstimator::get_standing_height() const { return standing_height_; }
Vector2d StateEstimator::get_ground_slope() const { return ground_slope_; }
const std::vector<Eigen::Vector3d>& StateEstimator::get_contact_forces() const {
    return contact_forces_;
}