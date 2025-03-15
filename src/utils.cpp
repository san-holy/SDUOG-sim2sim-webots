#include "utils.h"
#include <cmath>

void quat_rotate_inverse(const double* q, const double* v, double* result) {
    // 解包四元数分量 (x,y,z,w 格式)
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    
    // 解包向量分量
    const double vx = v[0], vy = v[1], vz = v[2];
    
    // 计算 2*w² - 1 的公共项
    const double w_sq_scale = 2.0 * qw * qw - 1.0;
    
    // 计算a项：v * (2w² - 1)
    const double ax = vx * w_sq_scale;
    const double ay = vy * w_sq_scale;
    const double az = vz * w_sq_scale;
    
    // 计算叉乘项 q × v
    const double cross_x = qy * vz - qz * vy;
    const double cross_y = qz * vx - qx * vz;
    const double cross_z = qx * vy - qy * vx;
    
    // 计算b项：-2w*(q × v)
    const double bx = -2.0 * qw * cross_x;
    const double by = -2.0 * qw * cross_y;
    const double bz = -2.0 * qw * cross_z;
    
    // 计算点乘 q·v
    const double dot = qx * vx + qy * vy + qz * vz;
    
    // 计算c项：2*(q·v)*q
    const double cx = 2.0 * dot * qx;
    const double cy = 2.0 * dot * qy;
    const double cz = 2.0 * dot * qz;
    
    // 合并结果：a + b + c
    result[0] = ax + bx + cx;
    result[1] = ay + by + cy;
    result[2] = az + bz + cz;

    // for (int i = 0; i < 3; i++) {
    //   std::cout << result[i] << " ";
    // }
    // std::cout << std::endl;
}

void quaternion_to_rotation_matrix(const double *quat, double rot[3][3]) {
    double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    rot[0][0] = 1 - 2 * y * y - 2 * z * z;
    rot[0][1] = 2 * x * y - 2 * z * w;
    rot[0][2] = 2 * x * z + 2 * y * w;
    rot[1][0] = 2 * x * y + 2 * z * w;
    rot[1][1] = 1 - 2 * x * x - 2 * z * z;
    rot[1][2] = 2 * y * z - 2 * x * w;
    rot[2][0] = 2 * x * z - 2 * y * w;
    rot[2][1] = 2 * y * z + 2 * x * w;
    rot[2][2] = 1 - 2 * x * x - 2 * y * y;
}

void rotate_vector(const double rot[3][3], const double *vec, double *result) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++)
          result[i] += rot[i][j] * vec[j];
      }
}

