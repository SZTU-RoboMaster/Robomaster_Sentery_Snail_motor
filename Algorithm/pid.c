#include "pid.h"

//TODO:��ʼ��
float ABS(float x)
{
    x=x<0?-x:x;
    return x;
}
/****************************************
�������ƣ�PID_General
�������ܣ�ʹ���ض�����ȶ��ĴﵽĿ��ֵ
����������Ŀ��ֵ��target
          ��ǰֵ��current
					PID�ṹ�������PID_GENERAL
��������ֵ�����:output
������ͨ��PID,��ͳPID
*****************************************/
float PID_General(float target,float current,PID_GENERAL *pid)
{
    //Ŀ�����ݴ�С����
    if (target > pid->input_max) { target = pid->input_max; }
    if (target < pid->input_min) { target = pid->input_min; }

    pid->Data_Save[0] = pid->Data_Save[1]; //��ʱDS[0]Ϊ��һ��DS[1]��ֵ
    pid->Data_Save[1] = target - current;   //��ʱDS[1]Ϊ���ֵ

    //��������С
    if (pid->Data_Save[1] > pid->input_maxerror)pid->Data_Save[1] = pid->input_maxerror;
    else if (pid->Data_Save[1] < -pid->input_maxerror)pid->Data_Save[1] = -pid->input_maxerror;

    //�����΢Сʱ���Ժ��Բ���
    if (ABS(target - current) <= pid->tiny)pid->Data_Save[1] = 0;

    //inteΪ�����direΪ΢����
    pid->inte = 0.999f * pid->inte + pid->Data_Save[1];
    pid->dire = pid->k_d * (pid->Data_Save[1] - pid->Data_Save[0]);

    //���ƻ������С
    if (pid->inte > pid->i_sum_max) { pid->inte = pid->i_sum_max; }
    if (pid->inte < -pid->i_sum_max) { pid->inte = -pid->i_sum_max; }

    //�������PIDֵ����������
    pid->t_error = (int16_t) (pid->Data_Save[1] * pid->k_p);
    pid->t_dire = (int16_t) pid->dire;
    pid->t_inte = (int16_t) (pid->inte * pid->k_i);
    pid->output = pid->Data_Save[1] * pid->k_p + pid->inte * pid->k_i + pid->dire;

    //������ݴ�С����
    if (pid->output > pid->output_max) { pid->output = pid->output_max; }
    if (pid->output < pid->output_min) { pid->output = pid->output_min; }
    return pid->output;
}


float PID_ChassisFollow_Variable_kp(float error)	//���̵��8192Ϊ360�ȣ�15Ϊ341��
{
    float kp=1;
    if(ABS(error)<400)
    {
        kp=ABS(error)/400+0.4f;
        kp=kp>1?1:kp;
        kp=kp<0?0:kp;
    }
    return kp;
}

/****************************************
�������ƣ�PID_ChassisFollow
�������ܣ�ʹ���ض�����ȶ��ĴﵽĿ��ֵ
����������Ŀ��ֵ��target
          ��ǰֵ��current
					PID�ṹ�������PID_ChassisFollow
��������ֵ�����:output
���������̸����õ�PID
*****************************************/
float PID_ChassisFollow(float target,float current,PID_GENERAL *pid)
{
//float kp_vari=1;

    //Ŀ�����ݴ�С����
    if(target>pid->input_max){target=pid->input_max;}
    if(target<pid->input_min){target=pid->input_min;}

    pid->Data_Save[0] = pid->Data_Save[1];
    pid->Data_Save[1] = target - current;

    if(target - current>pid->input_maxerror)pid->Data_Save[1]=pid->input_maxerror;
    else if(target - current<-pid->input_maxerror)pid->Data_Save[1]=-pid->input_maxerror;

    if(ABS(target - current)<=pid->tiny)pid->Data_Save[1]=0;

//		if(pid->Data_Save[1]<250)
//		{
//			kp_vari=1-0.5f*(250-pid->Data_Save[1])/250;
//		}



    pid->inte = 0.999f*pid->inte+pid->Data_Save[1];
    pid->dire = pid->k_d * (pid->Data_Save[1] - pid->Data_Save[0]);

    if(pid->inte>pid->i_sum_max){pid->inte=pid->i_sum_max;}
    if(pid->inte<-pid->i_sum_max){pid->inte=-pid->i_sum_max;}
//pid->t_error=(s16)(pid->Data_Save[1]*pid->k_p);
//pid->t_dire=(s16)pid->dire;
//pid->t_inte=(s16)(pid->inte * pid->k_i);
    pid->output = pid->Data_Save[1] * pid->k_p *PID_ChassisFollow_Variable_kp(pid->Data_Save[1]) + pid->inte * pid->k_i + pid->dire;

    //������ݴ�С����
    if(pid->output>pid->output_max){pid->output=pid->output_max;}
    if(pid->output<pid->output_min){pid->output=pid->output_min;}
    return pid->output;
}



float PID_Robust(float target,float current,float differential,PID_GENERAL *pid)
{
    //Ŀ�����ݴ�С����
    if(target>pid->input_max){target=pid->input_max;}
    if(target<pid->input_min){target=pid->input_min;}

    pid->Data_Save[0] = pid->Data_Save[1];
    pid->Data_Save[1] = target - current;

    if(target - current>pid->input_maxerror)pid->Data_Save[1]=pid->input_maxerror;
    else if(target - current<-pid->input_maxerror)pid->Data_Save[1]=-pid->input_maxerror;

    if(ABS(target - current)<=pid->tiny)pid->Data_Save[1]=0;

    pid->inte = 0.999f*pid->inte+pid->Data_Save[1];
    pid->dire = pid->k_d * differential;

    if(pid->inte>pid->i_sum_max){pid->inte=pid->i_sum_max;}
    if(pid->inte<-pid->i_sum_max){pid->inte=-pid->i_sum_max;}
//pid->t_error=(s16)(pid->Data_Save[1]*pid->k_p);
//pid->t_dire=(s16)pid->dire;
//pid->t_inte=(s16)(pid->inte * pid->k_i);
    pid->output = pid->Data_Save[1] * pid->k_p + pid->inte * pid->k_i + pid->dire;

    //������ݴ�С����
    if(pid->output>pid->output_max){pid->output=pid->output_max;}
    if(pid->output<pid->output_min){pid->output=pid->output_min;}
    return pid->output;
}


float PID_DegreeCycle(float target,float current,PID_GENERAL *pid)
{
    //Ŀ�����ݴ�С����
    if (target > pid->input_max) { target = pid->input_max; }
    if (target < pid->input_min) { target = pid->input_min; }

    pid->Data_Save[0] = pid->Data_Save[1]; //��ʱDS[0]Ϊ��һ��DS[1]��ֵ
    //��ʱDS[1]Ϊ���ֵ
    {
        float a = target,b = current, k = 1;
        if (a < b) {
            b = target;
            a = current;
            k = -k;
        }
        if (ABS(a - b) > ABS(a - b - 8191))
            pid->Data_Save[1] = k*(a - b - 8191);
        else
            pid->Data_Save[1] = k*(a - b);
    }


    //��������С
    if (pid->Data_Save[1] > pid->input_maxerror)pid->Data_Save[1] = pid->input_maxerror;
    else if (pid->Data_Save[1] < -pid->input_maxerror)pid->Data_Save[1] = -pid->input_maxerror;

    //�����΢Сʱ���Ժ��Բ���
    if (ABS(pid->Data_Save[1]) <= pid->tiny)pid->Data_Save[1] = 0;

    //inteΪ�����direΪ΢����
    pid->inte = 0.999f * pid->inte + pid->Data_Save[1];
    pid->dire = pid->k_d * (pid->Data_Save[1] - pid->Data_Save[0]);

    //���ƻ������С
    if (pid->inte > pid->i_sum_max) { pid->inte = pid->i_sum_max; }
    if (pid->inte < -pid->i_sum_max) { pid->inte = -pid->i_sum_max; }

    //�������PIDֵ����������
    pid->t_error = (int16_t) (pid->Data_Save[1] * pid->k_p);
    pid->t_dire = (int16_t) pid->dire;
    pid->t_inte = (int16_t) (pid->inte * pid->k_i);
    pid->output = pid->Data_Save[1] * pid->k_p + pid->inte * pid->k_i + pid->dire;

    //������ݴ�С����
    if (pid->output > pid->output_max) { pid->output = pid->output_max; }
    if (pid->output < pid->output_min) { pid->output = pid->output_min; }
    return pid->output;
}

float PID_DegreeCycle_3508(float target,float now_Deg, float now_Rpm, PID_GENERAL *pid_Deg, PID_GENERAL *pid_Rpm) {
    //�ǶȻ�����
    PID_DegreeCycle(target, now_Deg, pid_Deg);

    //�ٶȻ�Ŀ����
    float target_Rpm = pid_Deg->output;

    //�ٶȻ�����
    PID_General(target_Rpm, now_Rpm, pid_Rpm);

    return pid_Rpm->output;
}