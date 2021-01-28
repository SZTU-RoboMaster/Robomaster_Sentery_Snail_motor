#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "struct_typedef.h"

void PWM_servo_control_init(void);
void set_servo_angle(uint8_t channel,float angle);//���ö���Ƕ�
#define servo_1 1
#define servo_2 2
#define servo_3 3
#define servo_4 4
#define servo_5 5
#define servo_6 6
#define servo_7 7
#define servo_8 8
#define DEBUG 0;
#endif
