#include "robot_model.h"
#include "lcm_handler.h"
#include "data_logger.h"
#include "utils.h"
#include "terrain_estimator.h"
#include <iostream>
#include "controller_listener.h"

#define TIME_STEP 5

robot_control::obs_lcmt computeObs(const Eigen::Vector3f& lin_vel, const Eigen::Vector3f& ang_vel , double* gravity, double* commands, double* motor_pos, double* motor_datas, const std::vector<float>& heights, double time){
    robot_control::obs_lcmt obs;
    for(int i = 0; i < 3; i++){
        obs.base_lin_vel[i] = static_cast<float>(lin_vel[i]);
        obs.base_ang_vel[i] = static_cast<float>(ang_vel[i]);
        obs.gravity[i] = gravity[i];
    }
    for(int i = 0; i < 4; i++){
        obs.commands[i] = commands[i];
    }
    for(int i = 0; i < 12; i++){
        obs.pos[i] = motor_pos[i];
        obs.vel[i] = motor_datas[i + 12];
        obs.torque[i] = motor_datas[i + 24];
    }
    for(int i = 0; i < heights.size(); i++){
        obs.terrain[i] = heights[i] - 0.5;
        // std::cout << obs.terrain[i] << " ";
    }
    // std::cout << std::endl;
    obs.timestamp_us = time;
    return obs;
}

int main(int argc, char **argv) {
    robot_init();

    double timer = 0;
    
    RobotModel robot(TIME_STEP);
    TerrainEstimator terrain(robot);
    DataLogger logger("log.csv");
    TorqueBuffer buffer;
    controller_ToqueBuffer controller_buffer;
    LCMHandler lcm_handler(buffer);
    controller_LCMHandler controller_lcm_handler(controller_buffer);
    GamepadHandler gamepad_handler;

    double commands[4] = {0.5, 0, 0, 0.25};
    
    // robot.initializeDevices();
    std::cout << "robot initialized" << std::endl;

    while (robot_step(TIME_STEP) != -1) {
        timer += TIME_STEP/1000.0;
        // std::cout << "time: " << timer << std::endl;
        robot.updateSensorData();
        terrain.estimateTerrain();

        const auto& heights = terrain.getHeightMeasurements();

        
        //if(收到心跳包){
            exlcm::example_t example_msg;
            gamepad_handler.handleMessage(&example_msg);
            //gamepad_handler.handleMessage(&example_msg);
            bool has_commands = controller_buffer.try_pop(example_msg);
            if (has_commands) {
                commands[0] = static_cast<double>(gamepad_handler.speed_x);
                commands[1] = static_cast<double>(gamepad_handler.speed_y);
                commands[2] = static_cast<double>(gamepad_handler.yaw);
                commands[3] = static_cast<double>(gamepad_handler.height);
            }
            //这里将原来GamepadHandler中handleMessage函数内置的四个元素拿了出来，直接归属于GamepadHandler类，
            //这样方便改变commands的值，下面的lcm_handler.publishObs函数就不需要再改了
        //}
        // 处理LCM消息
        for (int i = 0 ; i < 4 ; i++){
            if (i ==0) lcm_handler.publishObs(computeObs(robot.getTorsoVelocity(), robot.getAngularVelocity(), robot.gravity, commands, robot.motor_data_error, robot.motor_data, heights, timer));
        }

        // lcm_handler.publishObs(computeObs(robot.getTorsoVelocity(), robot.base_ang_vel, robot.gravity, commands, robot.motor_data_error, robot.motor_data, heights, timer));
        // 应用控制
        if (!robot.standfinish) 
            robot.slowToStandingPosition();
        else {
            robot_control::actions_lcmt torque_msg;
            bool has_torque = buffer.try_pop(torque_msg);
            
            if (has_torque) {
                robot.zerodriftcontrol(torque_msg.torque);
                robot.applyTorques(torque_msg.torque);
                robot.last_torque_time = timer; // 记录最后收到力矩的时间
            } 
            else {
                // 超过10ms没有收到力矩则进入软急停
                if(timer - robot.last_torque_time > 0.02) {
                    robot.applyDamping(3.0); // 增强阻尼系数
                }
                else {
                    robot.zerodriftcontrol(robot.ACTIONS);
                    robot.applyTorques(robot.ACTIONS);
                    // robot.applyDamping(1.0); // 普通阻尼模式
                }
            }
        }
        // 记录数据
        logger.logData(timer, robot.getTorsoVelocity(), robot.base_ang_vel, robot.gravity, robot.motor_data);
    }

    robot_cleanup();
    return 0;
}//test
