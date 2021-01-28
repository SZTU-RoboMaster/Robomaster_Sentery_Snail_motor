#ifndef _PID_H_
#define _PID_H_

#include "struct_typedef.h"


typedef struct
{
    //PID参数
    float k_p;
    float k_i;
    float k_d;
    //最大输入误差
    float input_maxerror;
    //输入最小值和最大值
    int32_t input_min;
    int32_t input_max;
    //输出最小值和最大值
    const int32_t output_min;
    const int32_t output_max;
    //求和最大值
    float i_sum_max;
    //误差值存储
    float Data_Save[2];
    //积分项
    float inte;
    //微分项
    float dire;
    //输出
    float output;
    //微小误差消除
    const int32_t tiny;
    //16位整数PID项
    int16_t t_error;
    int16_t t_dire;
    int16_t t_inte;
} __packed PID_GENERAL;


float PID_ChassisFollow_Variable_kp(float error);
float PID_General(float target,float current,PID_GENERAL *pid);
float PID_ChassisFollow(float target,float current,PID_GENERAL *pid);
float PID_Robust(float target,float current,float differential,PID_GENERAL *pid);
float PID_DegreeCycle(float target,float current,PID_GENERAL *pid);
float PID_DegreeCycle_3508(float target,float now_Deg, float now_Rpm, PID_GENERAL *pid_Deg, PID_GENERAL *pid_Rpm);

#endif
