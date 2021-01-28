#include "PWM_control.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
//for servo

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

//1�ű�������ռ��
//������ͷ�ļ��޸ı�ţ�define������Ҫ������


void PWM_servo_control_init()
{
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim8);
	
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	
	//��ʼ���Ƕȣ����б�Ҫ���޸�
	//set_servo_angle(servo_1,90.0f);
	set_servo_angle(servo_2,90.0f);
	set_servo_angle(servo_3,90.0f);
	set_servo_angle(servo_4,90.0f);
	set_servo_angle(servo_5,90.0f);
	set_servo_angle(servo_6,90.0f);
	set_servo_angle(servo_7,90.0f);
	set_servo_angle(servo_8,90.0f);

}
void set_servo_angle(uint8_t channel,float angle)//ͳһ��180���
{
	uint16_t CCR = (1500.0 / 180.0) * angle + 500;
	
	switch(channel)
	{
		case servo_1:
			//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, CCR);
			break;
		case servo_2:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, CCR);
			break;
		case servo_3:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CCR);
			break;
		case servo_4:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, CCR);
			break;
		case servo_5:
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, CCR);
			break;
		case servo_6:
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, CCR);
			break;
		case servo_7:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CCR);
			break;
		case servo_8:
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, CCR);
			break;
	}
}

//end of file
