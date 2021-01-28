
#include "PID_controller.h"

pid_t pid_create(pid_t pid, int16_t* in, int16_t* out, int16_t* set, int16_t kp, int16_t ki, int16_t kd)
{
    pid->input = in;
    pid->output = out;
    pid->setpoint = set;
    pid->automode = true;

    //默认上限255。下限0
    pid_limits(pid, 0, 255);
    pid->deadBand = 1;

    // 取样时间默认为100ms
    pid->sampletime = 100 * (TICK_SECOND / 1000);

    // PID方向默认为正向
    pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);

    pid->lasttime = HAL_GetTick() - pid->sampletime;
    return pid;
}

bool pid_need_compute(pid_t pid)
{
    // 检查PID周期是否已经结束
    return(HAL_GetTick() - pid->lasttime >= pid->sampletime) ? true : false;
}

void pid_compute(pid_t pid)
{
    // 检查控制器是否已经开启
    // 未开启返回false
    if (!pid->automode)
        return;

    // 已开启执行以下语句
    // 输入值
    int16_t in = *(pid->input);
    // 计算误差
    int16_t error = (*(pid->setpoint)) - in;

//    //TODO:死区截止
//    if (in > -pid->deadBand || in < pid->deadBand) {
//        *pid->output = 0.0f;
//        pid->lastin = 0.0f;
//        pid->lasttime = HAL_GetTick();
//        return;
//    }


    // 计算积分项
    pid->iterm += (pid->Ki * error);
    if (pid->iterm > pid->omax)
        pid->iterm = pid->omax;
    else if (pid->iterm < pid->omin)
        pid->iterm = pid->omin;

    // 计算输入的微分项
    int16_t dinput = in - pid->lastin;
    // 计算PID输出
    int16_t out = pid->Kp * error + pid->iterm - pid->Kd * dinput;

    // 对输出值进行限制
    if (out > pid->omax)
        out = pid->omax;
    else if (out < pid->omin)
        out = pid->omin;
    // 输出指定变量
    (*pid->output) = out;
    // 更新变量
    pid->lastin = in;
    pid->lasttime = HAL_GetTick();
}

void pid_tune(pid_t pid, int16_t kp, int16_t ki, int16_t kd)
{
    // 检查有效性
    if (kp < 0 || ki < 0 || kd < 0)
        return;

    // 计算取样时间
    int16_t ssec;
    ssec = ((int16_t) pid->sampletime) / (TICK_SECOND);

    pid->Kp = kp;
    pid->Ki = ki * ssec;
    pid->Kd = kd / ssec;

    //如果PID为反向
    if (pid->direction == E_PID_REVERSE) {
        pid->Kp = 0 - pid->Kp;
        pid->Ki = 0 - pid->Ki;
        pid->Kd = 0 - pid->Kd;
    }
}

void pid_sample(pid_t pid, uint32_t time)
{
    if (time > 0) {
        // 比例
        int16_t ratio =  (time * (TICK_SECOND / 1000)) / pid->sampletime;
        pid->Ki *= ratio;
        pid->Kd /= ratio;
        pid->sampletime = time * (TICK_SECOND / 1000);
    }
}

void pid_limits(pid_t pid, int16_t min, int16_t max)
{
    // 检查输入值是否符合大小关系。并修改最大最小值
    if (min >= max) return;
    pid->omin = min;
    pid->omax = max;

    //检查自动模式是否开启
    if (pid->automode) {
        //输出值超过最大值则设为最大值，小于最小值则设为最小值
        if (*(pid->output) > pid->omax)
            *(pid->output) = pid->omax;
        else if (*(pid->output) < pid->omin)
            *(pid->output) = pid->omin;

        //积分项同理
        if (pid->iterm > pid->omax)
            pid->iterm = pid->omax;
        else if (pid->iterm < pid->omin)
            pid->iterm = pid->omin;
    }
}

void pid_auto(pid_t pid)
{
    // 如果是刚从手动模式到自动模式
    if (!pid->automode) {
        //积分项和微分项赋值
        pid->iterm = *(pid->output);
        pid->lastin = *(pid->input);

        //限制积分项大小
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
    //如果PID为自动模式。且当前方向与需要修改的方向不同时
    if (pid->automode && pid->direction != dir) {
        pid->Kp = (0 - pid->Kp);
        pid->Ki = (0 - pid->Ki);
        pid->Kd = (0 - pid->Kd);
    }
    pid->direction = dir;
}
