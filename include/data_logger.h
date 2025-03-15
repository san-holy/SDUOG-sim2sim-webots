#pragma once
#include <fstream>
#include <string>
#include <iostream>
#include "utils.h"

class DataLogger {
public:
    DataLogger(const std::string& filename);
    ~DataLogger();
    
    void logHeader();
    void logData(double time, const Eigen::Vector3f& lin_vel, const double* ang_vel, 
                const double* rpy, const double* motor_data);

private:
    std::ofstream csv_file_;
};