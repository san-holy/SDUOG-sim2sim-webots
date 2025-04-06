#pragma once
#ifndef MOTOR_SDK_H
#define MOTOR_SDK_H
#include <iostream>


    int robot_get_device(const char* name);
    void motor_set_torque(int32_t tag,double torque);
    double position_sensor_get_value(int32_t tag);
    void motor_enable_torque_feedback(int32_t tag,int time_step_);
    void position_sensor_enable(int32_t tag,int time_step_);
    void motor_set_velocity(int32_t tag,int time_step_);
    void accelerometer_enable(int32_t tag,int time_step_);
    void inertial_unit_enable(int imu_,int time_step_);
    void gyro_enable(int gyro_,int time_step_);
    int robot_step(int time_step_);
    const double* accelerometer_get_values(int accelerometer_);
    const double* inertial_unit_get_quaternion(int imu_);
    const double* gyro_get_values(int gyro_);
    void robot_init();
    void imuinit();
    void motorinit();
    void leginit();
    void robot_cleanup();
    void imu_init(int time_step_);
    void motor_and_sensor_init(int num,int time_step_,const char* motorname[],const char* sensorname[]);
#endif // MOTOR_SDK_H