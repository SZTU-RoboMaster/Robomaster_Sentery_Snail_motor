//
// Created by Azure Dylan on 2021/1/27.
//

#include <LED_control.h>
#include "FIRE.h"
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

void Fire_Init(){
    HAL_TIM_Base_Start(&htim1);

    led_show(0xffffa500);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    led_show(0xff000000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,1000);
    HAL_Delay(1000);
}
void Fire_on(int speed){
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,speed);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,speed);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,speed);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,speed);
}
void Fire_off(){
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,1000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,1000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,1000);
}
int move_speed=0;
int count_i;
void Fire_test(int Set_speed){
    if(count_i==0) {
        move_speed = 1000;
    }
    move_speed++;
    if(move_speed>=Set_speed){
        move_speed=Set_speed;
    }
    Fire_on(move_speed);
}
void Fire_Command(){
    int i=0;
    if(DEBUG_SNAIL==1){
        for(int i;i<=10;i++){
        led_show(RED);
       osDelay(10);
        led_show(RED);
        osDelay(10);
        }
    }
    else if(DEBUG_SNAIL==0){
        for(i=0;i<=2;i++){
   //         led_show(GREEN);
            osDelay(1);
            led_show(0);
        }
        if(RC_data.rc.s[0]==RC_SW_UP)          //左边的switch  低速模式
            Fire_test(1400);
        else if(RC_data.rc.s[0]==RC_SW_DOWN)      //右边的switch 高速模式
            Fire_test(1400);
        else
            Fire_off();
        for(i=0;i<=2;i++){
   //         led_show(RED);
            osDelay(1);
            led_show(0);
        }
    }
    else if(DEBUG_SNAIL==2){
        for(int i;i<=10;i++){
            led_show(YELLOW);
            osDelay(10);
            led_show(YELLOW);
            osDelay(10);
        }
        Fire_test(RC_data.rc.ch[1]);
    }
}
void Fire_special_Command(all_command *snail_command){
    int p= snail_command->ccr-1/19999;
    int psc=168*10^6/snail_command->f/19999;
    __HAL_TIM_SET_PRESCALER(&htim1,psc);
    Fire_test(p);

}