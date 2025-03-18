#include "include/controller_listener.h"

class GamepadHandler {
    public:
        float speed_x;
        float speed_y;  
        float yaw;
        float height;
        void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const exlcm::example_t* msg) {
        

        yaw = (float)msg->buffer[5] / 32768.0f; 
        height = 0.15f + (float)msg->buffer[6] * (0.25f / 65535.0f);
        std::cout << " yaw:" << yaw << " rad/s" << std::endl;
        std::cout << "height: " << height << " m" << std::endl;  
        switch (msg->buffer[0]) {
            case 0:
                std::cout << "速度模式：低速行走" << std::endl;
                speed_y = (float)msg->buffer[1] / 32768.0f; 
                speed_x = -(float) msg->buffer[2]/ 32768.0f;    
                std::cout << "Current x speed: " << speed_x << " m/s" << std::endl;
                std::cout << "Current y speed: " << speed_y << " m/s" << std::endl;   
                break;
            case 1:
                std::cout << "速度模式：快速行走" << std::endl;
                speed_y =(float)msg->buffer[1];
                speed_x =-(float) msg->buffer[2];
                // speed_y = (float)msg->buffer[1] / 32768.0f; 
                // speed_x = (float) msg->buffer[2]/ 32768.0f; 
                speed_x =  speed_x * (3.0f - (-2.0f))/2+0.5f;
                speed_y =  speed_y * (3.0f - (-2.0f))/2+0.5f;   
                std::cout << "Current x speed: " << speed_x << " m/s" << std::endl;
                std::cout << "Current y speed: " << speed_y << " m/s" << std::endl;
                break;
            case 2:
                std::cout << "速度模式：奔跑" << std::endl;
                speed_y= ((float)msg->buffer[1]/ 32768.0) * (5.0 - 3.5) + 3.5;
                speed_x = ((-(float) msg->buffer[2]/ 32768.0f))* (5.0 - 3.5) + 3.5;
                std::cout << "Current x speed: " << speed_x << " m/s" << std::endl;
                std::cout << "Current y speed: " << speed_y << " m/s" << std::endl;
                break;
            case 3:
                std::cout << "速度模式：疾速" << std::endl;
                speed_y= ((float)msg->buffer[1]/ 32768.0) *  (7.0 - 4.0) + 4.0;
                speed_x = ((-(float) msg->buffer[2])/ 32768.0f)*  (7.0 - 4.0) + 4.0;
                std::cout << "Current x speed: " << speed_x << " m/s" << std::endl;
                std::cout << "Current y speed: " << speed_y << " m/s" << std::endl;
                break;
    
            } 
    
        if(msg->buffer[3]==0) 
        {
    
            printf("导航模式\n");
        }
        else printf("手动模式\n");  
        if(msg->buffer[4]==0) 
        {
    
            printf("中继连接agx\n");
        }
        else printf("直连nuc\n");          
        }
    };
    