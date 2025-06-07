#include "motor_SDK.h"
#include <iostream>
#include "usb_can/include/LordImu.h"
#include <vector>
#include"LegControl.h"
#include "MotorControl.h"

int motors_[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
int sensors_[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
double position;
double torque;
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
    }
    return acc_arr;  
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
void motor(int i,double torques)
{   
   switch(motors_[i]/3)
   {
   case 0:legControl.LF.SendTauCommand(motors_[i]%3+1, 0); break;
   case 1:legControl.RF.SendTauCommand(motors_[i]%3+1, 0); break;
   case 2:legControl.LH.SendTauCommand(motors_[i]%3+1, 0); break;
   case 3:legControl.RH.SendTauCommand(motors_[i]%3+1, 0); break;
   }    
}


double position_get_value(int i)
{   
    LegState state = legControl.getState();
   
    switch (sensors_[i]/3)
    {
    case 0: position = state.lf_q[sensors_[i]%3]+1; break;
    case 1: position = state.rf_q[sensors_[i]%3]+1; break;
    case 2: position = state.lh_q[sensors_[i]%3]+1; break;
    case 3: position = state.rh_q[sensors_[i]%3]+1; break;
    }
    return position;
}


double torque_get_value(int i)
{   
    LegState state = legControl.getState();
    switch (sensors_[i]/3)
    {
    case 0: torque = state.lf_tau[sensors_[i]%3]+1; break;
    case 1: torque = state.rf_tau[sensors_[i]%3]+1; break;
    case 2: torque = state.lh_tau[sensors_[i]%3]+1; break;
    case 3: torque = state.rh_tau[sensors_[i]%3]+1; break;
    }
    return torque;
}







void motors_init(const int motorname,const int sensorname)
{
  legControl.LegInit();      
}