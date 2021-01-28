
#include "PID_controller.h"

pid_t pid_create(pid_t pid, int16_t* in, int16_t* out, int16_t* set, int16_t kp, int16_t ki, int16_t kd)
{
    pid->input = in;
    pid->output = out;
    pid->setpoint = set;
    pid->automode = true;

    //Ĭ������255������0
    pid_limits(pid, 0, 255);
    pid->deadBand = 1;

    // ȡ��ʱ��Ĭ��Ϊ100ms
    pid->sampletime = 100 * (TICK_SECOND / 1000);

    // PID����Ĭ��Ϊ����
    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    pid->lasttime = HAL_GetTick() - pid->sampletime;
    return pid;
}

bool pid_need_compute(pid_t pid)
{
    // ���PID�����Ƿ��Ѿ�����
    return(HAL_GetTick() - pid->lasttime >= pid->sampletime) ? true : false;
}

void pid_compute(pid_t pid)
{
    // ���������Ƿ��Ѿ�����
    // δ��������false
    if (!pid->automode)
        return;

    // �ѿ���ִ���������
    // ����ֵ
    int16_t in = *(pid->input);
    // �������
    int16_t error = (*(pid->setpoint)) - in;

//    //TODO:������ֹ
//    if (in > -pid->deadBand || in < pid->deadBand) {
//        *pid->output = 0.0f;
//        pid->lastin = 0.0f;
//        pid->lasttime = HAL_GetTick();
//        return;
//    }


    // ���������
    pid->iterm += (pid->Ki * error);
    if (pid->iterm > pid->omax)
        pid->iterm = pid->omax;
    else if (pid->iterm < pid->omin)
        pid->iterm = pid->omin;

    // ���������΢����
    int16_t dinput = in - pid->lastin;
    // ����PID���
    int16_t out = pid->Kp * error + pid->iterm - pid->Kd * dinput;

    // �����ֵ��������
    if (out > pid->omax)
        out = pid->omax;
    else if (out < pid->omin)
        out = pid->omin;
    // ���ָ������
    (*pid->output) = out;
    // ���±���
    pid->lastin = in;
    pid->lasttime = HAL_GetTick();
}

void pid_tune(pid_t pid, int16_t kp, int16_t ki, int16_t kd)
{
    // �����Ч��
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    // ����ȡ��ʱ��
    int16_t ssec;
    ssec = ((int16_t) pid->sampletime) / (TICK_SECOND);

    pid->Kp = kp;
    pid->Ki = ki * ssec;
    pid->Kd = kd / ssec;

    //���PIDΪ����
    if (pid->direction == E_PID_REVERSE) {
        pid->Kp = 0 - pid->Kp;
        pid->Ki = 0 - pid->Ki;
        pid->Kd = 0 - pid->Kd;
    }
}

void pid_sample(pid_t pid, uint32_t time)
{
    if (time > 0) {
        // ����
        int16_t ratio =  (time * (TICK_SECOND / 1000)) / pid->sampletime;
        pid->Ki *= ratio;
        pid->Kd /= ratio;
        pid->sampletime = time * (TICK_SECOND / 1000);
    }
}

void pid_limits(pid_t pid, int16_t min, int16_t max)
{
    // �������ֵ�Ƿ���ϴ�С��ϵ�����޸������Сֵ
    if (min >= max) return;
    pid->omin = min;
    pid->omax = max;

    //����Զ�ģʽ�Ƿ���
    if (pid->automode) {
        //���ֵ�������ֵ����Ϊ���ֵ��С����Сֵ����Ϊ��Сֵ
        if (*(pid->output) > pid->omax)
            *(pid->output) = pid->omax;
        else if (*(pid->output) < pid->omin)
            *(pid->output) = pid->omin;

        //������ͬ��
        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;
    }
}

void pid_auto(pid_t pid)
{
    // ����Ǹմ��ֶ�ģʽ���Զ�ģʽ
    if (!pid->automode) {
        //�������΢���ֵ
        pid->iterm = *(pid->output);
        pid->lastin = *(pid->input);

        //���ƻ������С
        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;

        pid->automode = true;
    }
}

void pid_manual(pid_t pid)
{
    pid->automode = false;
}

void pid_direction(pid_t pid, enum pid_control_directions dir)
{
    //���PIDΪ�Զ�ģʽ���ҵ�ǰ��������Ҫ�޸ĵķ���ͬʱ
    if (pid->automode && pid->direction != dir) {
        pid->Kp = (0 - pid->Kp);
        pid->Ki = (0 - pid->Ki);
        pid->Kd = (0 - pid->Kd);
    }
    pid->direction = dir;
}
