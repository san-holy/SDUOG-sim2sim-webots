#include "motor_SDK.h"
#include <iostream>
#include "usb_can/include/LordImu.h"
#include <vector>
#include"LegControl.h"
#include "MotorControl.h"

int motors_[12];
int sensors_[12];
int accelerometer_;
int imu_;
int gyro_; 
LordImu imu;
void imu_init(){
  imu.initImu();
}
ImuData data;
double acc_arr[3]; 
double qua_arr[4];
double gy_arr[3];
double* accel_get_values(int accelerometer_) {
    data = imu.getImuData();
    for (int i = 0; i < 3; ++i) {
        acc_arr[i] = static_cast<double>(data.linearAcc[i]); 
    return acc_arr;  
}
}
double* unit_get_quaternion(int imu_){
    data = imu.getImuData();  
    for (int i = 0; i < 4; ++i) {
        qua_arr[i] = static_cast<double>(data.quat[i]); 
    }
    return qua_arr;  
}
double* gy_get_values(int gyro_) {
    data = imu.getImuData(); 
     for (int i = 0; i < 3; ++i) {
        gy_arr[i] = static_cast<double>(data.angularVel[i]); 
    }
    return gy_arr;  
}

LegControl legControl;
void motor(int i,double torque)
{   
   switch(motors_[i]/3)
   {
   case 0:legControl.LF.SendTauCommand(motors_[i]%4+1, torque); 
   case 1:legControl.LH.SendTauCommand(motors_[i]%4+1, torque);
   case 2:legControl.RF.SendTauCommand(motors_[i]%4+1, torque);
   case 3:legControl.RH.SendTauCommand(motors_[i]%4+1, torque);
   }    
}

LegState state = legControl.getState();
double position_get_value(int i)
{   
   switch (sensors_[i]/3)
   {
   case 0: return state.lf_q[sensors_[i]%4];
   case 1: return state.lh_q[sensors_[i]%4];
   case 2: return state.rf_q[sensors_[i]%4];
   case 3: return state.rh_q[sensors_[i]%4];
   }
}
void motors_init(const int motorname,const int sensorname)
{
  legControl.LegInit();      
}