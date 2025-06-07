#pragma once
#ifndef MOTOR_SDK_H
#define MOTOR_SDK_H
#include <iostream>
#include <vector>


    extern int motors_[12];
    extern int sensors_[12];
    extern int accelerometer_;
    extern int imu_;
    extern int gyro_;  
    // void imuHandler(const std::string& channel, float value);   
    double position_get_value(int i);
    double torque_get_value(int i);
    // void motor_enable_torque_feedback(int32_t tag,int time_step_);
    // void position_sensor_enable(int32_t tag,int time_step_);
    // void motor_set_velocity(int32_t tag,int time_step_);
    // void accelerometer_enable(int32_t tag,int time_step_);
    // void inertial_unit_enable(int imu_,int time_step_);
    // void gyro_enable(int gyro_);
    // void robot_step(int step);
    // int robot_get_device(const char* name);
    // void robot_init();
    void motor(int i,double torque);//motor_set_torque
    double*unit_get_quaternion(int imu_);
    double* gy_get_values(int gyro_);
    double* accel_get_values(int accelerometer_);
    // void motorinit();   
    void LegInit();
    // void robot_cleanup();
    void imu_init();
    void motors_init(const int motorname,const int sensorname);
#endif // MOTOR_SDK_H

