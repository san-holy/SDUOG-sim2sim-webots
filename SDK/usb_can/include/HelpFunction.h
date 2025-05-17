#pragma once

#include <math.h>

// 关节限位               RF                    LF                    RH                    LH
const double qMin[12] = {-1.0, -1.57, 0.2,     -1.0, -1.0, -2.2,     -1.0, -1.0, -2.2,     -1.0, -1.57, 0.2};
const double qMax[12] = { 1.0,  1.0,  2.2,      1.0,  1.57,-0.2,      1.0,  1.57,-0.2,      1.0,  1.0,  2.2};

bool isqWithinLimits(const double q[12]);
void clampTorques(double tau[12]);
float normalizeAngle(float angle);