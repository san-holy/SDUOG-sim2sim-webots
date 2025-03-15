#pragma once
#include <Eigen/Dense>
#include <Eigen/SVD>

void quat_rotate_inverse(const double* q, const double* v, double* result);
void quaternion_to_rotation_matrix(const double *quat, double rot[3][3]);
void rotate_vector(const double rot[3][3], const double *vec, double *result);
inline Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
    Eigen::Matrix3f m = q.toRotationMatrix();
    float roll = atan2(m(2,1), m(2,2));
    float pitch = atan2(-m(2,0), sqrt(m(2,1)*m(2,1) + m(2,2)*m(2,2)));
    float yaw = atan2(m(1,0), m(0,0));
    return Eigen::Vector3f(roll, pitch, yaw);
}