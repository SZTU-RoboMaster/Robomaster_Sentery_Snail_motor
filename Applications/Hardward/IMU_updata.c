#include "IMU_updata.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"

#include "PID_controller.h"
#include "tim.h"
#include "stdlib.h"
#include "ist8310driver.h"
//maed by sethome


//extern TIM_HandleTypeDef htim10;

struct IMU_t
{
	fp32 gyro[3];//��/s
	fp32 accel[3];//m/s^2
	fp32 mag[3];//ut
	fp32 temp;
};
struct IMU_t IMU_data;//IMU���ݽṹ��

struct pid_controller IMU_tempure_pid;
float * out;

float target_temp = 50.0f;

void IMU_init()
{
	ist8310_init();
	BMI088_init();

	//ʵ����out
	out = (float*)malloc(sizeof(float));
	pid_create(&IMU_tempure_pid, &IMU_data.temp, out, &target_temp, 1100.0f, 0.0f, 0.0f);
    pid_limits(&IMU_tempure_pid, -3000.0f, 3000.0f);
	
  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);//���ȵ���PWM
	
  HAL_TIM_Base_Start_IT(&htim14);//ʹ�ܸ����жϣ�200HZ
}

void IMU_updata()//200HZ
{
	//��ȡ�����Ǻ͵شż���Ϣ
	BMI088_read(IMU_data.gyro,IMU_data.accel,&IMU_data.temp);
	ist8310_read_mag(IMU_data.mag);
	
	//������PID����
    pid_compute(&IMU_tempure_pid);
	IMU_heat_set((int)IMU_tempure_pid.output);
}

void IMU_heat_set(uint16_t pwm)
{
  TIM10->CCR1 = (pwm);
}

//end of file
