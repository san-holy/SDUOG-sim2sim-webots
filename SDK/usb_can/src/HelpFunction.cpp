#include "../include/HelpFunction.h"
#include "iostream"

// 判断关节角度是否在范围内
bool isqWithinLimits(const double q[12]) {
    for (int i = 0; i < 12; i++) {
        if (q[i] < qMin[i] || q[i] > qMax[i]) {
            std::cerr << "Joint " << i << " out of range: " << q[i]
                      << " (Allowed range: [" << qMin[i] << ", " << qMax[i] << "])" << std::endl;
            return false; 
        }
    }
    return true;
}

// 限制力矩
void clampTorques(double tau[12]) {
    for (int i = 0; i < 12; i++) {
        tau[i] = std::max(-48.0, std::min(tau[i], 48.0));
    }
}

// 规范角度
float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}