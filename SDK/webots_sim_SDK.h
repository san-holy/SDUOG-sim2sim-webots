#ifndef SDUOG_SIM2SIM_WEBOTS_SDK_H
#define SDUOG_SIM2SIM_WEBOTS_SDK_H
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
    WbDeviceTag robot_get_device(const char* name);
    void motor_set_torque(WbDeviceTag tag,double torque);
    double position_sensor_get_value(WbDeviceTag tag);
    void motor_enable_torque_feedback(WbDeviceTag tag,int time_step_);
    void position_sensor_enable(WbDeviceTag tag,int time_step_);
    void motor_set_velocity(WbDeviceTag tag,int time_step_);
    void accelerometer_enable(WbDeviceTag tag,int time_step_);
    void inertial_unit_enable(WbDeviceTag imu_,int time_step_);
    void gyro_enable(WbDeviceTag gyro_,int time_step_);
    int robot_step(int time_step_);
    const double* accelerometer_get_values(WbDeviceTag accelerometer_);
    const double* inertial_unit_get_quaternion(WbDeviceTag imu_);
    const double* gyro_get_values(WbDeviceTag gyro_);

#endif