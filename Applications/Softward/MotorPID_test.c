
#include "MotorPID_test.h"
#include "CAN_receive&send.h"
#include "pid.h"
#include "math.h"

#define MAX_RPM 1000

PID_GENERAL Speed_Motor0 = {
        3.0f,
        0.01f,
        0.0f,
        3000,
        -8000,
        8000,
        -4700,
        4700,
        1500,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0,
        0,
        0
};

PID_GENERAL Speed_Motor1 = {
        3.0f,
        0.01f,
        0.0f,
        3000,
        -8000,
        8000,
        -4700,
        4700,
        1500,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0,
        0,
        0
};

PID_GENERAL Speed_Motor2= {
        3.0f,
        0.01f,
        0.0f,
        3000,
        -8000,
        8000,
        -4700,
        4700,
        1500,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0,
        0,
        0
};

PID_GENERAL  Speed_Motor3 = {
        3.0f,
        0.01f,
        0.0f,
        3000,
        -8000,
        8000,
        -4700,
        4700,
        1500,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0,
        0,
        0
};

#define FR 0
#define FL 1
#define BL 2
#define BR 3
#define MAX_VX_SPEED 2000.0f
#define MAX_VY_SPEED 2000.0f
#define MAX_VW_SPEED 2000.0f
#define K_SPEED 100

void MotorON(float vx, float vy, float vw) {
    //限制输出值大小
    vx = fminf(MAX_VX_SPEED, fmaxf(-MAX_VX_SPEED, vx));
    vy = fminf(MAX_VY_SPEED, fmaxf(-MAX_VY_SPEED, vy));
    vw = fminf(MAX_VW_SPEED, fmaxf(-MAX_VW_SPEED, vw));

    //PID控制器开始运行
    PID_General(K_SPEED*(-vx - vy + vw), (float)Can1_motor_data[FR].speed_rpm, &Speed_Motor0);
    PID_General(K_SPEED*(-vx + vy + vw), (float)Can1_motor_data[FL].speed_rpm, &Speed_Motor1);
    PID_General(K_SPEED*(+vx + vy + vw), (float)Can1_motor_data[BL].speed_rpm, &Speed_Motor2);
    PID_General(K_SPEED*(+vx - vy + vw), (float)Can1_motor_data[BR].speed_rpm, &Speed_Motor3);
//    PID_General(5000, (float)Can1_motor_data[FR].speed_rpm, &Speed_Motor0);
//    PID_General(5000, (float)Can1_motor_data[FL].speed_rpm, &Speed_Motor1);
//    PID_General(5000, (float)Can1_motor_data[BL].speed_rpm, &Speed_Motor2);
//    PID_General(5000, (float)Can1_motor_data[BR].speed_rpm, &Speed_Motor3);

    set_motor_current(Speed_Motor0.output, &hcan1, FR);
    set_motor_current(Speed_Motor1.output, &hcan1, FL);
    set_motor_current(Speed_Motor2.output, &hcan1, BL);
    set_motor_current(Speed_Motor3.output, &hcan1, BR);

    CAN1_send_current();

}