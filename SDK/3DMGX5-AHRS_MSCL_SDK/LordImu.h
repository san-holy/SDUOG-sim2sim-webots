#include <mscl/mscl.h> // 报错不用管
#include <iostream>
#include <vector>
#include <functional>

class LordImu
{
public:
    // 回调函数 每次接收到数据时都会调用这个函数
    using ImuCallback = std::function<void(const std::string&, float)>;

    LordImu(const std::string& port, uint32_t baudRate)
    {
        // 创建连接对象
        connection = mscl::Connection::Serial(port, baudRate);
        node = new mscl::InertialNode(connection); // IMU节点

        // 配置采样率 500 Hz
        mscl::MipChannels channels; // 消息通道
        // 线性加速度通道，即没有进行补偿的加速度
        channels.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl::SampleRate::Hertz(500)));
        // 旋转矩阵
        channels.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_MATRIX, mscl::SampleRate::Hertz(500)));
        // 四元数
        channels.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION, mscl::SampleRate::Hertz(500)));
        // rpy
        channels.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl::SampleRate::Hertz(500)));
        // 角速度
        channels.push_back(mscl::MipChannel(mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl::SampleRate::Hertz(500)));

        node->setActiveChannelFields(mscl::MipTypes::CLASS_ESTFILTER, channels);

        // 开始采集数据
        node->resume();
    }

    ~LordImu()
    {
        // 停止采集数据
        node->setToIdle();
        delete node;
    }

    void setImuCallback(ImuCallback callback)
    {
        imuCallback = callback;
    }

    void readData()
    {
        mscl::MipDataPackets packets = node->getDataPackets(5000); // 等待数据包，超时5000毫秒

        for (mscl::MipDataPacket packet : packets)
        {
            mscl::MipDataPoints dataPoints = packet.data();

            // 遍历所有数据点
            for (mscl::MipDataPoint dataPoint : dataPoints)
            {
                std::string channelName = dataPoint.channelName();

                if (imuCallback) // 如果回调函数已经被设置
                {
                    if (channelName == "estLinearAccelX" || channelName == "estLinearAccelY" || channelName == "estLinearAccelZ" ||
                        channelName == "estAngularRateX" || channelName == "estAngularRateY" || channelName == "estAngularRateZ" ||
                        channelName == "estRoll" || channelName == "estPitch" || channelName == "estYaw")
                    {
                        imuCallback(channelName, dataPoint.as_float()); // 传递通道名和数据
                    }
                    else if (channelName == "estOrientMatrix")
                    {
                        mscl::Matrix matrix = dataPoint.as_Matrix();
                        for (int i = 0; i < matrix.rows(); ++i)
                        {
                            for (int j = 0; j < matrix.columns(); ++j)
                            {
                                std::string channelName1;
                                channelName1 = channelName + std::to_string(i) + std::to_string(j);
                                imuCallback(channelName1, matrix.as_floatAt(i, j));
                            }
                        }
                    }
                    else if (channelName == "estOrientQuaternion")
                    {
                        mscl::Vector quaternion = dataPoint.as_Vector();
                        for (int i = 0; i < quaternion.size(); ++i)
                        {
                            std::string channelName1;
                            //将channelName,i合并为一个字符串，赋给channelName
                            channelName1 = channelName + std::to_string(i);
                            imuCallback(channelName1, quaternion.as_floatAt(i));
                        }
                    }
                }
            }
        }
    }

private:
    mscl::Connection connection;
    mscl::InertialNode* node;
    ImuCallback imuCallback;
};
