
#ifndef RMC_ADAM_GENERALROBOTSYSTEMCODE_CHASSIS_MOVE1_H
#define RMC_ADAM_GENERALROBOTSYSTEMCODE_CHASSIS_MOVE1_H


#include "struct_typedef.h"

void ChassisMove_Init(); //底盘电机速度环控制初始化
void ChassisMotorSpeedrpm_Control(float vx, float vy, float vw); //底盘电机速度环控制

#endif //RMC_ADAM_GENERALROBOTSYSTEMCODE_CHASSIS_MOVE1_H
