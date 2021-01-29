//
// Created by Azure Dylan on 2021/1/27.
//

#ifndef RMC_CODE_FIRE_H
#define RMC_CODE_FIRE_H
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include "DBUS_remote_control.h"
#include "cmsis_os2.h"
#define  DEBUG_SNAIL 0
#define  FIRE_INIT 1001         //初始化进程
#define  FIRE_MAX  2201         //最大油门进程
#define  FIRE_DOWN 1201         //低油门进程
extern int move_speed;          //引入该变量进行油门逐步加速
extern TIM_HandleTypeDef htim1;             //时钟初始化
extern int count_i;
typedef struct PWM_value_snail_motor{
    int f;
    int psc;
    int ccr;
}all_command;
void Fire_Init();
void Fire_Command();           //Fire_command 给出转速转速范围--1201-2201   -通过遥控器的拨杆实现
void Fire_test(int Set_speed);              //直接给出油门进程 范围同上  不通过遥控器控制
void Fire_off();               //电机关闭
void Fire_special_Command(all_command *snail_command);        //直接进行频率调节控制(调节分频值)     由于当前重载值设为19999，ABP2总线频率168Mhz 故计算公式为 psc=168*10^6/f/19999
#endif //RMC_CODE_FIRE_H
