
#include "DegCyclePID.h"
#include "pid.h"
#include "math.h"
#include "CAN_receive&send.h"

#define DegRatio_6020 (360.0f / 8191.0f) //  360/8191
#define DegRatio_3508 22       //   360 /



PID_GENERAL Degree_Motor = {
        3.0f,
        0.02f,
        0.0f,
        8200.0f,
        -10.0f,
        8200.0f, //由电机ecd最值决定8191
        -8200.0f,
        8200.0f,
        30000.0f,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0,
        0,
        0
};

void MotorSetDeg(float Deg) {
    Deg = fminf(360.0f, fmaxf(0.0f, Deg));

    PID_DegreeCycle(Deg*DegRatio_6020, (float)Can1_motor_data[0].ecd, &Degree_Motor);

    set_motor_current(Degree_Motor.output, &hcan1, 0);

    CAN1_send_current();
}