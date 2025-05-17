#ifndef LORDIMU_H
#define LORDIMU_H

#include <mscl/mscl.h> 
#include <string>
#include <thread>
#include <atomic>

struct Quaternion {
    float w, x, y, z;
};

struct ImuData {
    float linearAcc[3];
    float angularVel[3];
    float rpy[3];
    float quat[4];
};

class LordImu
{
public:
    LordImu();
    ~LordImu();

    // 初始化 IMU
    bool initImu();

    // 读取并将数据赋值到 imudata 中
    void readImuData();
    Quaternion rpyToQuaternion(float roll, float pitch, float yaw);

    ImuData getImuData() const;

private:
    mscl::Connection connection;
    mscl::InertialNode* node;

    bool firstRead = true; // 标记是否是第一次读取IMU数据   

    float y0; // 记录第一次的yaw角
    Quaternion q;

    ImuData imudata;

    std::thread _imuThread;
    std::atomic<bool> running;

    mutable std::mutex imuMutex; // 互斥锁，mutable 允许在 const 函数中锁定
};

#endif // LORDIMU_H
