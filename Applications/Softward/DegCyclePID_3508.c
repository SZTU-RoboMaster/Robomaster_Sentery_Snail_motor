#include "DegCyclePID_3508.h"
#include "pid.h"
#include "math.h"
#include "CAN_receive&send.h"

#define DegRatio_6020 (8191.0f / 360.0f) //  8191/360
#define Ratio_3508 (187.0f/3591.0f)
#define DegRatio_3508 (8191.0f / 360.0f)   //   360 /


PID_GENERAL Deg_Motor = {
        3.0f,
        0.01f,
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
        1.0f,
        0,
        0,
        0
};

PID_GENERAL Rpm_Motor = {
        3.0f,
        0.01f,
        0.0f,
        8200.0f,
        -8200.0f,
        8200.0f, //由电机ecd最值决定8191
        -8200.0f,
        8200.0f,
        30000.0f,
        {0.0f, 0.0f},
        0.0f,
        0.0f,
        0.0f,
        1.0f,
        0,
        0,
        0
};

float watch_Deg_now;
float watch_Deg_target;

void MotorSetDeg_3508(float Deg) {
//    Deg = fminf(360.0f, fmaxf(0.0f, Deg));
//
//    watch_Deg_now = Can1_motor_data[0].ecd2.real_deg * Ratio_3508;
//    watch_Deg_target = Deg * DegRatio_3508;
//
//    PID_DegreeCycle_3508(Deg * DegRatio_3508, (float) Can1_motor_data[0].ecd2.real_deg, (float) Can1_motor_data[0].speed_rpm,
//                         &Deg_Motor, &Rpm_Motor);

//    PID_General(500, (float)Can1_motor_data->speed_rpm, &Rpm_Motor);
//
//    set_motor_current(Rpm_Motor.output, &hcan1, 0);
//
//    CAN1_send_current();

}
