#include "terrain_estimator.h"
#include <algorithm>
#include <iostream>

TerrainEstimator::TerrainEstimator(const RobotModel& robot) 
    : robot_(robot), height_map_(grid_x * grid_y, 0.0f),
    x_step_((max_x - min_x) / (grid_x - 1)),
    y_step_((max_y - min_y) / (grid_y - 1)),
    center_x_((max_x + min_x) / 2.0f),
    center_y_((max_y + min_y) / 2.0f),
    robot_stand_height_(0.0f) {}

void TerrainEstimator::estimateTerrain() {
    std::vector<float> foot_positions(12);
    calculateFootPositions(foot_positions);
        
    // 新增地面方向估计
    estimateGroundOrientation();
        
    // 修正高度估计（考虑躯干速度）
    const Eigen::Vector3f& torso_vel = robot_.getTorsoVelocity();
    float dt = robot_.time_step_ / 1000.0f;
        
    // 对每个足端位置进行速度补偿
    for (int leg = 0; leg < 4; ++leg) {
        if (robot_.getContactStatus()[leg]) {
            foot_positions[leg*3+2] += torso_vel.z() * dt;
        }
    }

    // 计算基准高度：接触点高度平均值
    float sum_z = 0.0f;
    int contact_count = 0;
    for (int leg = 0; leg < 4; ++leg) {
        if (robot_.getContactStatus()[leg]) {
            sum_z += foot_positions[leg*3 + 2];
            contact_count++;
        }
    }
    robot_stand_height_ = contact_count > 0 ? sum_z / contact_count : 0.0f;
    std::cout << "Robot stand height: " << robot_stand_height_ << std::endl;
    
    generateHeightMap(foot_positions);
}

void TerrainEstimator::calculateFootPositions(std::vector<float>& foot_positions) const {
    const auto& joint_angles = robot_.getJointPositions();
    
    for(int leg = 0; leg < 4; ++leg) {
        int idx = leg * 3;
        float q_hip   = hip_axis_sign * joint_angles[idx];
        float q_thigh = thigh_axis_sign * joint_angles[idx+1];
        float q_calf  = calf_axis_sign * joint_angles[idx+2];

        // 根据URDF坐标系调整计算
        float x = hip_offset * ((leg % 2 == 0) ? 1 : -1); // 左右腿对称
        float y = thigh_length * sin(q_thigh) + calf_length * sin(q_thigh + q_calf);
        float z = thigh_length * cos(q_thigh) + calf_length * cos(q_thigh + q_calf);

        // 应用髋关节旋转（绕x轴）
        float final_x = x;
        float final_y = y * cos(q_hip) - z * sin(q_hip);
        float final_z = y * sin(q_hip) + z * cos(q_hip);

        foot_positions[leg*3]   = final_x;
        foot_positions[leg*3+1] = final_y;
        foot_positions[leg*3+2] = final_z;
    }
}

void TerrainEstimator::generateHeightMap(const std::vector<float>& foot_positions) {
    // 转换坡度角度为弧度
    const float slope_rad = terrain_slope_ * M_PI / 180.0f;
    const float roll_rad = terrain_roll_ * M_PI / 180.0f;
    
    // 1. 初始化基础高度图（基于姿态的斜坡）
    for (int y_idx = 0; y_idx < grid_y; ++y_idx) {
        for (int x_idx = 0; x_idx < grid_x; ++x_idx) {
            // 计算网格点世界坐标
            const float x = min_x + x_idx * x_step_;
            const float y = min_y + y_idx * y_step_;
            
            // 计算相对中心点的偏移量
            const float dx = x - center_x_;
            const float dy = y - center_y_;
            
            // 根据坡度计算高度变化
            height_map_[y_idx * grid_x + x_idx] = robot_stand_height_ 
                + dx * std::tan(slope_rad) 
                + dy * std::tan(roll_rad);
        }
    }
    
    // 2. 用足端测量值修正高度图
    for (int leg = 0; leg < 4; ++leg) {
        const float x = foot_positions[leg*3];
        const float y = foot_positions[leg*3 + 1];
        const float z = foot_positions[leg*3 + 2];
        
        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
            // 计算该位置的理论基础高度
            const float dx_leg = x - center_x_;
            const float dy_leg = y - center_y_;
            const float base_z = robot_stand_height_ 
                + dx_leg * std::tan(slope_rad) 
                + dy_leg * std::tan(roll_rad);
            
            // 计算高度差异并进行插值补偿
            bilinearInterpolation(x, y, z - base_z);
        }
    }
}

void TerrainEstimator::bilinearInterpolation(float x, float y, float delta_z) {
    // 计算网格索引
    const int xi = static_cast<int>((x - min_x) / x_step_);
    const int yi = static_cast<int>((y - min_y) / y_step_);
    
    if (xi < 0 || xi >= grid_x-1 || yi < 0 || yi >= grid_y-1) return;
    
    // 计算插值权重
    const float x_ratio = (x - (min_x + xi * x_step_)) / x_step_;
    const float y_ratio = (y - (min_y + yi * y_step_)) / y_step_;
    
    // 叠加高度差异到四个相邻点
    height_map_[yi * grid_x + xi]         += (1 - x_ratio) * (1 - y_ratio) * delta_z;
    height_map_[yi * grid_x + xi + 1]     += x_ratio * (1 - y_ratio) * delta_z;
    height_map_[(yi + 1) * grid_x + xi]   += (1 - x_ratio) * y_ratio * delta_z;
    height_map_[(yi + 1) * grid_x + xi + 1] += x_ratio * y_ratio * delta_z;
}

// 地形估计实现（terrain_estimator.cpp）
void TerrainEstimator::estimateGroundOrientation() {
    // 使用加权平均法线估计
    Eigen::Vector3f normal_sum = Eigen::Vector3f::Zero();
    float total_force = 0.0f;

    for (int leg = 0; leg < 4; ++leg) {
        if (robot_.getContactStatus()[leg]) {
            float force_weight = robot_.getFootForces()[leg].norm();
            Eigen::Vector3f f_normal = robot_.getFootForces()[leg].normalized();
            normal_sum += force_weight * f_normal;
            total_force += force_weight;
        }
    }

    if (total_force > 1e-3f) {
        Eigen::Vector3f avg_normal = normal_sum / total_force;
        
        // 转换到机体坐标系
        Eigen::Vector3f body_normal = robot_.getOrientation().inverse() * avg_normal;
        
        // 计算坡度角（考虑机体当前姿态）
        terrain_slope_ = atan2(body_normal.y(), body_normal.z());
        terrain_roll_ = atan2(body_normal.x(), body_normal.z());
        
        // 应用低通滤波
        terrain_slope_ = 0.9f * terrain_slope_prev_ + 0.1f * terrain_slope_;
        terrain_roll_ = 0.9f * terrain_roll_prev_ + 0.1f * terrain_roll_;
        
        // 更新历史值
        terrain_slope_prev_ = terrain_slope_;
        terrain_roll_prev_ = terrain_roll_;
    }
}