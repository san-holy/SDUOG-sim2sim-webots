#pragma once
#ifndef controller_listener_h
#define controller_listener_h

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "exlcm/example_t.hpp"  // 引入头文件


class GamepadHandler {
    public:
        float speed_x;
        float speed_y;  
        float yaw;
        float height;
        void handleMessage(const exlcm::example_t* msg);
    };

#endif