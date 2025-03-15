#include "data_logger.h"

DataLogger::DataLogger(const std::string& filename) {
    csv_file_.open(filename, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        std::cerr << "错误：无法打开日志文件！" << std::endl;
        return;
    }
    logHeader();
}

DataLogger::~DataLogger() {
    csv_file_.close();
}

void DataLogger::logHeader() {
    // 原CSV表头代码
    csv_file_ << "时间,线速度X,线速度Y,线速度Z,角速度X,角速度Y,角速度Z,重力方向x,重力方向y,重力方向z,"

    // 关节角度（每腿3关节 x 4腿）
    << "左前髋关节角度,左前大腿关节角度,左前小腿关节角度,"
    << "右前髋关节角度,右前大腿关节角度,右前小腿关节角度,"
    << "左后髋关节角度,左后大腿关节角度,左后小腿关节角度,"
    << "右后髋关节角度,右后大腿关节角度,右后小腿关节角度,"

    // 关节角速度（同角度顺序）
    << "左前髋关节角速度,左前大腿关节角速度,左前小腿关节角速度,"
    << "右前髋关节角速度,右前大腿关节角速度,右前小腿关节角速度,"
    << "左后髋关节角速度,左后大腿关节角速度,左后小腿关节角速度,"
    << "右后髋关节角速度,右后大腿关节角速度,右后小腿关节角速度,"

    // 关节力矩（同角度顺序）
    << "左前髋关节力矩,左前大腿关节力矩,左前小腿关节力矩,"
    << "右前髋关节力矩,右前大腿关节力矩,右前小腿关节力矩,"
    << "左后髋关节力矩,左后大腿关节力矩,左后小腿关节力矩,"
    << "右后髋关节力矩,右后大腿关节力矩,右后小腿关节力矩"
    << std::endl;
}

void DataLogger::logData(double time, const Eigen::Vector3f& lin_vel, const double* ang_vel,
                        const double* rpy, const double* motor_data) {
    // 写入数据到 CSV 文件
    csv_file_ << time << "," << lin_vel[0] << "," << lin_vel[1] << "," << lin_vel[2] << ","
             << ang_vel[0] << "," << ang_vel[1] << "," << ang_vel[2] << ","
             << rpy[0] << "," << rpy[1] << "," << rpy[2];
    for (int i = 0; i < 24; i++) {
      csv_file_ << "," << motor_data[i];
    }
    for (int i = 0; i < 12; i++) {
      csv_file_ << "," << motor_data[i + 24] * 0.25;
    }
    csv_file_ << std::endl;
}