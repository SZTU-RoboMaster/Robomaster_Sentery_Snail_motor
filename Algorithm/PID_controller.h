

#ifndef RMC_ADAM_GENERALROBOTSYSTEMCODE_PID_CONTROLLER_H
#define RMC_ADAM_GENERALROBOTSYSTEMCODE_PID_CONTROLLER_H

/*-------------------------------------------------------------*/
/*		Includes and dependencies			*/
/*-------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>

/*-------------------------------------------------------------*/
/*		Macros and definitions				*/
/*-------------------------------------------------------------*/

#define TICK_SECOND 1000

/*-------------------------------------------------------------*/
/*		Typedefs enums & structs			*/
/*-------------------------------------------------------------*/

/**
 * 定义控制器是正向还是反向
 */
enum pid_control_directions {
    E_PID_DIRECT,
    E_PID_REVERSE,
};

/**
 * PID数据结构体, 多个实体会使用不同的控制结构
 */
typedef struct pid_controller {
    // 输入输出与目标值
    int16_t * input; //!< 当前过程值
    int16_t * output; //!< 输出
    int16_t * setpoint; //!< 目标值
    // PID系数
    int16_t Kp; //!< 比例系数
    int16_t Ki; //!< 积分系数
    int16_t Kd; //!< 微分系数
    // 输出的最大值与最小值
    int16_t omin; //!< 输出最大值
    int16_t omax; //!< 输出最小值
    // PID算法变量
    int16_t iterm; //!< 积分项累加
    int16_t lastin; //!< 微分项的上一个输入值
    //死区
    int16_t deadBand;
    // 时间量
    uint32_t lasttime; //!< 存储上次控制器运行时间
    uint32_t sampletime; //!< 取样时间
    // 控制模式
    uint8_t automode; //!< 打开true或关闭false
    //PID方向
    enum pid_control_directions direction;
}pid_controller;

typedef struct pid_controller * pid_t;

/*-------------------------------------------------------------*/
/*		function prototype			*/
/*-------------------------------------------------------------*/
#ifdef	__cplusplus
extern "C" {
#endif
/**
 * @brief 创建一个新的PID控制器
 *
 * 创建一个新的PID控制器并初始化输入、输出和积分项变量。设置PID系数
 *
 * @param pid PID控制器指针
 * @param in 过程输入指针（浮点数据）
 * @param out 接受控制器输出值的指针
 * @param set 过程目标值指针（浮点数据）
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 *
 * @return 返回PID控制器句柄
 */
pid_t pid_create(pid_t pid, int16_t* in, int16_t* out, int16_t* set, int16_t kp, int16_t ki, int16_t kd);

/**
 * @brief 检查PID循环是否需要运行
 *
 * 决定PID算法是否应该计算一个新的输入值
 * 如果返回为真，用户应该读取过程反馈（传感器）并修改输入值为读取值，然后调用pid_compute()函数
 *
 * @return 如果PID循环需要运行则返回真，否则为假
 */
bool pid_need_compute(pid_t pid);

/**
 * @brief 计算PID控制器输出值
 *
 * 本函数计算基于参数、目标值和当前系统输入的PID输出值，
 *
 * @param pid PID控制器的指针
 */
void pid_compute(pid_t pid);

/**
 * @brief 设置新PID参数
 *
 * 设置控制器内PID参数
 *
 * @param pid PID控制器
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void pid_tune(pid_t pid, int16_t kp, int16_t ki, int16_t kd);

/**
 * @brief 设置PID算法周期
 *
 * Changes 改变PID控制计算周期
 *
 * @param pid PID控制器指针
 * @param time 计算周期时间ms
 */
void pid_sample(pid_t pid, uint32_t time);

/**
 * @brief 设置PID控制器输出限制
 *
 * @param pid PID控制器指针
 * @param min PID控制器最小输出值
 * @param max PID控制器最大输出值
 */
void pid_limits(pid_t pid, int16_t min, int16_t max);

/**
 * @brief 开启PID自动控制
 *
 * 开启PID控制循环。
 * 如果需要手动输出调整可以使用pid_manual()关闭循环。
 *
 * @param pid PID控制器指针
 */
void pid_auto(pid_t pid);

/**
 * @brief 关闭自动控制
 *
 * 关闭PID控制循环。用户可以手动修改输出值
 * 变量和控制器不会更改输出值
 *
 * @param PID控制器指针
 */
void pid_manual(pid_t pid);

/**
 * @brief 配置PID控制器方向
 *
 * 给PID控制器设置方向
 * 当方向为"DIRECT"时，控制器输出的增量会使得测量值也增加
 * 当方向为"REVERSE"时，控制器输出的增量会使测量值减小
 *
 * @param pid PID控制器指针
 * @param direction 方向("DIRECT","REVERSE")
 */
void pid_direction(pid_t pid, enum pid_control_directions dir);

#ifdef	__cplusplus
}
#endif

#endif //RMC_ADAM_GENERALROBOTSYSTEMCODE_PID_CONTROLLER_H
