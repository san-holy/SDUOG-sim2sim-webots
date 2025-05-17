#pragma once



struct JointCommand {

    JointCommand() {zero();}

    double qDes[12];
    double vDes[12];
    double tauff[12];
    double Kp[12];
    double Kd[12];

    void set(const motor_command *msg) {
        for(int i = 0; i < 12; i++) {
            qDes[i] = msg->qDes[i];
            vDes[i] = msg->vDes[i];
            tauff[i] = msg->tauff[i];
            Kp[i] = msg->Kp[i];
            Kd[i] = msg->Kd[i];
            // std::cout << " msg->qDes[i]: " << std::endl;
        }
    }

    void zero() {
        for(int i = 0; i < 12; i++) {
            qDes[i] = 0;
            vDes[i] = 0;
            tauff[i] = 0;
            Kp[i] = 0;
            Kd[i] = 0;
        }
    }
};