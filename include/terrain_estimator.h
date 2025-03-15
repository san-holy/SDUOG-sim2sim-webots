#pragma once
#include <cmath>
#include "robot_model.h"


class TerrainEstimator {
public:
    explicit TerrainEstimator(const RobotModel& robot);
    
    // 地面高度估计接口
    void estimateTerrain();
    const std::vector<float>& getHeightMeasurements() const { return height_map_; }
    // 新增方法
    float getTerrainSlope() const { return terrain_slope_; }
    float getTerrainRoll() const { return terrain_roll_; }

private:
    const RobotModel& robot_;
    std::vector<float> height_map_;
    
    // 从URDF提取的精确参数
    const float thigh_length = 0.088241f;   // 大腿长度 = thigh_joint origin y轴位移
    const float calf_length = 0.22023f;     // 小腿长度 = calf_joint origin z轴绝对值
    const float hip_offset = 0.05f;         // 髋关节侧向偏移 = hip_joint origin y轴值
    
    // 坐标系转换参数（根据URDF关节轴方向调整）
    const float hip_axis_sign = 1.0f;       // 髋关节旋转轴方向（URDF中axis xyz="1 0 0"）
    const float thigh_axis_sign = 1.0f;     // 大腿关节旋转轴方向（URDF中axis xyz="0 1 0"）
    const float calf_axis_sign = 1.0f;      // 小腿关节旋转轴方向（URDF中axis xyz="0 1 0"）
    
    // 测量网格参数（与训练时相同）
    static constexpr int grid_x = 17;  // X方向采样点数
    static constexpr int grid_y = 11;  // Y方向采样点数
    static constexpr float min_x = -0.8f;
    static constexpr float max_x = 0.8f;
    static constexpr float min_y = -0.5f;
    static constexpr float max_y = 0.5f;

    const float x_step_;         // X方向网格步长
    const float y_step_;         // Y方向网格步长
    const float center_x_;       // 机器人中心X坐标
    const float center_y_;       // 机器人中心Y坐标
    float robot_stand_height_;   // 机器人基准站立高度

    float terrain_slope_ = 0.0f;  // 地面坡度（deg）
    float terrain_roll_ = 0.0f;   // 地面侧倾角（deg）
    float terrain_slope_prev_ = 0.0f;
    float terrain_roll_prev_ = 0.0f;


    // 核心计算函数
    void calculateFootPositions(std::vector<float>& foot_positions) const;
    void generateHeightMap(const std::vector<float>& foot_positions);
    void bilinearInterpolation(float x, float y, float delta_z);
    void estimateGroundOrientation();
};