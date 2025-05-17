
#include "../include/HelpFunction.h"
#include <math.h>
#include <iostream>
#include <string>
#include <functional>
#include <usb_can/include/LordImu.h>
#define Pi 3.14156265358979323846264338

Quaternion LordImu::rpyToQuaternion(float roll, float pitch, float yaw) {
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    Quaternion q;

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

LordImu::LordImu() : node(nullptr) {
}

LordImu::~LordImu()
{
    running = false;
    if (_imuThread.joinable()) {
        _imuThread.join();
    }

    if (node) {
        node->setToIdle();
        delete node;
    }
}

bool LordImu::initImu()
{
    try {
        connection = mscl::Connection::Serial("/dev/ttyACM0",921600); // 注意端口号
        node = new mscl::InertialNode(connection);

        mscl::MipChannels estFilterChs;
        estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl::SampleRate::Hertz(500)));
        estFilterChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl::SampleRate::Hertz(500)));

        node->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, estFilterChs);

        mscl::MipChannels ahrsImuChs;
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_EULER_ANGLES, mscl::SampleRate::Hertz(500)));
        ahrsImuChs.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_MATRIX, mscl::SampleRate::Hertz(500)));

        node->setActiveChannelFields(mscl::MipTypes::CLASS_AHRS_IMU, ahrsImuChs);

        node->resume();

        running  = true;

        _imuThread = std::thread(&LordImu::readImuData, this); // 开启读取imu的线程

        return true;
    } catch (const mscl::Error& e) {
        std::cerr << "Failed to initialize IMU: " << e.what() << std::endl;
        return false;
    }
}

void LordImu::readImuData()
{
    while (running)
    {
        mscl::MipDataPackets packets = node->getDataPackets(1000);

        for (mscl::MipDataPacket packet : packets)
        {
            mscl::MipDataPoints dataPoints = packet.data();

            for (mscl::MipDataPoint dataPoint : dataPoints)
            {
                std::string channelName = dataPoint.channelName();

                // 处理IMU数据
                if (channelName == "estLinearAccelX") {
                    imudata.linearAcc[0] = dataPoint.as_float();
                } 
                else if (channelName == "estLinearAccelY") {
                    imudata.linearAcc[1] = -dataPoint.as_float();
                } 
                else if (channelName == "estLinearAccelZ") {
                    imudata.linearAcc[2] = -dataPoint.as_float();
                } 
                else if (channelName == "estAngularRateX") {
                    imudata.angularVel[0] = dataPoint.as_float();
                } 
                else if (channelName == "estAngularRateY") {
                    imudata.angularVel[1] = -dataPoint.as_float();
                } 
                else if (channelName == "estAngularRateZ") {
                    imudata.angularVel[2] = -dataPoint.as_float();
                }
                else if (channelName == "roll") {
                    imudata.rpy[0] = dataPoint.as_float();
                }
                else if (channelName == "pitch") {
                    imudata.rpy[1] = -dataPoint.as_float();
                }
                else if (channelName == "yaw") {
                    if (firstRead) {
                        y0 = dataPoint.as_float(); // 记录初始角度
                        firstRead = false;
                    } else {
                        // 计算相对yaw并归一化
                        float rawYaw = dataPoint.as_float();
                        float relativeYaw = normalizeAngle(rawYaw - y0); // 计算差值并归一化

                        // 取反并存储到imudata.rpy[2]
                        imudata.rpy[2] = -relativeYaw;

                        // 将欧拉角转换为四元数
                        q = rpyToQuaternion(imudata.rpy[0], imudata.rpy[1], imudata.rpy[2]);
                        imudata.quat[0] = q.w;
                        imudata.quat[1] = q.x;
                        imudata.quat[2] = q.y;
                        imudata.quat[3] = q.z;
                    }
                }
            }
        }
    }   
}

ImuData LordImu::getImuData() const {
    std::lock_guard<std::mutex> lock(imuMutex); // 锁定互斥锁以安全地读取imudata
    return imudata;
}
