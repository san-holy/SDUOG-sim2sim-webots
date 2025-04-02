#include "webots_sim_SDK.h"

WbDeviceTag motors_[12];
WbDeviceTag sensors_[12];
WbDeviceTag accelerometer_;
WbDeviceTag imu_;
WbDeviceTag gyro_;
WbDeviceTag robot_get_device(const char* name){
    return wb_robot_get_device(name);
}
void motor_set_torque(WbDeviceTag tag,double torque){
    wb_motor_set_torque(tag,torque);
}
double position_sensor_get_value(WbDeviceTag tag){
    return wb_position_sensor_get_value(tag);
}
void motor_enable_torque_feedback(WbDeviceTag tag,int time_step_){
    wb_motor_enable_torque_feedback(tag,time_step_);
}
void position_sensor_enable(WbDeviceTag tag,int time_step_){
    wb_position_sensor_enable(tag,time_step_);
}
void motor_set_velocity(WbDeviceTag tag,int time_step_){
    wb_motor_set_velocity(tag,time_step_);
}
void accelerometer_enable(WbDeviceTag tag,int time_step_){
    wb_accelerometer_enable(tag,time_step_);
}
void inertial_unit_enable(WbDeviceTag imu_,int time_step_){
    wb_inertial_unit_enable(imu_,time_step_);
}
void gyro_enable(WbDeviceTag gyro_,int time_step_){
    wb_gyro_enable(gyro_,time_step_);
}
int robot_step(int time_step_){
    return wb_robot_step(time_step_);
}
const double* accelerometer_get_values(WbDeviceTag accelerometer_){
    return wb_accelerometer_get_values(accelerometer_);
}
const double* inertial_unit_get_quaternion(WbDeviceTag imu_){
    return wb_inertial_unit_get_quaternion(imu_);
}
const double* gyro_get_values(WbDeviceTag gyro_){
    return wb_gyro_get_values(gyro_);
}
void robot_init(){
    wb_robot_init();
}
void robot_cleanup(){
    wb_robot_cleanup();
}
void imu_init1(){
    imu_ = robot_get_device("inertial unit");
    accelerometer_ = robot_get_device("accelerometer");
    gyro_ = robot_get_device("gyro");
}
void imu_init2(int time_step_){
    accelerometer_enable(accelerometer_, time_step_);
    std::cout << "Accelerometer enabled." << std::endl;
    inertial_unit_enable(imu_, time_step_);
    std::cout << "IMU enabled." << std::endl;
    gyro_enable(gyro_, time_step_);
    std::cout << "Gyro enabled." << std::endl;
    
    robot_step(time_step_);
}
void motor_and_sensor_init1(int num,const char* motorname[],const char* sensorname[]){
    motors_[num]=robot_get_device(motorname[num]);
    sensors_[num]=robot_get_device(sensorname[num]);
}
void motor_and_sensor_init2(int num,int time_step_){
    motor_enable_torque_feedback(motors_[num], time_step_);
    position_sensor_enable(sensors_[num], time_step_);
    motor_set_velocity(motors_[num], 0);   // 设置零速度
}