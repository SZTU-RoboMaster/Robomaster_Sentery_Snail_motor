
#include "chassis_move1.h"
#include "math.h"
#include "CAN_receive&send.h"
#include "PID_controller.h"
#include "stdlib.h"
/**
 * @map   0 %++++++% 1
              ++++
              ++++
          2 %++++++% 3
*/
#define PI 3.1415926
#define MAX_VX_SPEED 2000.00f
#define MAX_VY_SPEED 2000.00f
#define MAX_VW_SPEED 2000.00f
//轮子编号
#define FL 0 //前
#define FR 1
#define BL 2 //后
#define BR 3
//全局变量
struct pid_controller speedrpmCycle[4];//速度控制
//PID速度环: 输入 输出 设定值(4个轮子不同的电流环参数)
float* input[4] = {NULL};
float* output[4] = {NULL};
float* setpoint[4] = {NULL};

void ChassisMove_Init() {
    //PID控制器初始化
    //指针指定对象
    input[FL] = (float*)&Can1_motor_data[FL].speed_rpm;
    input[FR] = (float*)&Can1_motor_data[FR].speed_rpm;
    input[BL] = (float*)&Can1_motor_data[BL].speed_rpm;
    input[BR] = (float*)&Can1_motor_data[BR].speed_rpm;

    //实例化输出值和目标值
    output[FL] = (float*)malloc(sizeof(float));
    output[FR] = (float*)malloc(sizeof(float));
    output[BL] = (float*)malloc(sizeof(float));
    output[BR] = (float*)malloc(sizeof(float));

    setpoint[FL] = (float*)malloc(sizeof(float));
    setpoint[FR] = (float*)malloc(sizeof(float));
    setpoint[BL] = (float*)malloc(sizeof(float));
    setpoint[BR] = (float*)malloc(sizeof(float));

    //PID参数初始化
    //PID系数
    pid_create(&speedrpmCycle[FL], input[FL], output[FL], setpoint[FL], 1.0f, 0.0f, 0.0f);
    pid_create(&speedrpmCycle[FR], input[FR], output[FR], setpoint[FR], 1.0f, 0.0f, 0.0f);
    pid_create(&speedrpmCycle[BL], input[BL], output[BL], setpoint[BL], 1.0f, 0.0f, 0.0f);
    pid_create(&speedrpmCycle[BR], input[BR], output[BR], setpoint[BR], 1.0f, 0.0f, 0.0f);
    //输出值与积分项上下限
    pid_limits(&speedrpmCycle[FL], -30000, 30000);
    pid_limits(&speedrpmCycle[FR], -30000, 30000);
    pid_limits(&speedrpmCycle[BL], -30000, 30000);
    pid_limits(&speedrpmCycle[BR], -30000, 30000);

}

void ChassisMotorSpeedrpm_Control(float vx, float vy, float vw) {
    //限制输出值大小
    vx = fminf(MAX_VX_SPEED, fmaxf(-MAX_VX_SPEED, vx));
    vy = fminf(MAX_VY_SPEED, fmaxf(-MAX_VY_SPEED, vy));
    vw = fminf(MAX_VW_SPEED, fmaxf(-MAX_VW_SPEED, vw));

    //设置目标值
    float k = 1.0f; //放大倍数
    *setpoint[FL] = k*(-vx - vy + vw);
    *setpoint[FR] = k*(-vx + vy + vw);
    *setpoint[BL] = k*(+vx + vy + vw);
    *setpoint[BR] = k*(+vx - vy + vw);

    //速度环控制
    pid_compute(&speedrpmCycle[FL]);
    pid_compute(&speedrpmCycle[FR]);
    pid_compute(&speedrpmCycle[BL]);
    pid_compute(&speedrpmCycle[BR]);

    //设置电机电流
    set_motor_current(*output[FL], &hcan1, FL);
    set_motor_current(*output[FR], &hcan1, FR);
    set_motor_current(*output[BL], &hcan1, BL);
    set_motor_current(*output[BR], &hcan1, BR);

    CAN1_send_current();
}