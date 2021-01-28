

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
 * ����������������Ƿ���
 */
enum pid_control_directions {
    E_PID_DIRECT,
    E_PID_REVERSE,
};

/**
 * PID���ݽṹ��, ���ʵ���ʹ�ò�ͬ�Ŀ��ƽṹ
 */
typedef struct pid_controller {
    // ���������Ŀ��ֵ
    int16_t * input; //!< ��ǰ����ֵ
    int16_t * output; //!< ���
    int16_t * setpoint; //!< Ŀ��ֵ
    // PIDϵ��
    int16_t Kp; //!< ����ϵ��
    int16_t Ki; //!< ����ϵ��
    int16_t Kd; //!< ΢��ϵ��
    // ��������ֵ����Сֵ
    int16_t omin; //!< ������ֵ
    int16_t omax; //!< �����Сֵ
    // PID�㷨����
    int16_t iterm; //!< �������ۼ�
    int16_t lastin; //!< ΢�������һ������ֵ
    //����
    int16_t deadBand;
    // ʱ����
    uint32_t lasttime; //!< �洢�ϴο���������ʱ��
    uint32_t sampletime; //!< ȡ��ʱ��
    // ����ģʽ
    uint8_t automode; //!< ��true��ر�false
    //PID����
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
 * @brief ����һ���µ�PID������
 *
 * ����һ���µ�PID����������ʼ�����롢����ͻ��������������PIDϵ��
 *
 * @param pid PID������ָ��
 * @param in ��������ָ�루�������ݣ�
 * @param out ���ܿ��������ֵ��ָ��
 * @param set ����Ŀ��ֵָ�루�������ݣ�
 * @param kp ����ϵ��
 * @param ki ����ϵ��
 * @param kd ΢��ϵ��
 *
 * @return ����PID���������
 */
pid_t pid_create(pid_t pid, int16_t* in, int16_t* out, int16_t* set, int16_t kp, int16_t ki, int16_t kd);

/**
 * @brief ���PIDѭ���Ƿ���Ҫ����
 *
 * ����PID�㷨�Ƿ�Ӧ�ü���һ���µ�����ֵ
 * �������Ϊ�棬�û�Ӧ�ö�ȡ���̷����������������޸�����ֵΪ��ȡֵ��Ȼ�����pid_compute()����
 *
 * @return ���PIDѭ����Ҫ�����򷵻��棬����Ϊ��
 */
bool pid_need_compute(pid_t pid);

/**
 * @brief ����PID���������ֵ
 *
 * ������������ڲ�����Ŀ��ֵ�͵�ǰϵͳ�����PID���ֵ��
 *
 * @param pid PID��������ָ��
 */
void pid_compute(pid_t pid);

/**
 * @brief ������PID����
 *
 * ���ÿ�������PID����
 *
 * @param pid PID������
 * @param kp ����ϵ��
 * @param ki ����ϵ��
 * @param kd ΢��ϵ��
 */
void pid_tune(pid_t pid, int16_t kp, int16_t ki, int16_t kd);

/**
 * @brief ����PID�㷨����
 *
 * Changes �ı�PID���Ƽ�������
 *
 * @param pid PID������ָ��
 * @param time ��������ʱ��ms
 */
void pid_sample(pid_t pid, uint32_t time);

/**
 * @brief ����PID�������������
 *
 * @param pid PID������ָ��
 * @param min PID��������С���ֵ
 * @param max PID������������ֵ
 */
void pid_limits(pid_t pid, int16_t min, int16_t max);

/**
 * @brief ����PID�Զ�����
 *
 * ����PID����ѭ����
 * �����Ҫ�ֶ������������ʹ��pid_manual()�ر�ѭ����
 *
 * @param pid PID������ָ��
 */
void pid_auto(pid_t pid);

/**
 * @brief �ر��Զ�����
 *
 * �ر�PID����ѭ�����û������ֶ��޸����ֵ
 * �����Ϳ���������������ֵ
 *
 * @param PID������ָ��
 */
void pid_manual(pid_t pid);

/**
 * @brief ����PID����������
 *
 * ��PID���������÷���
 * ������Ϊ"DIRECT"ʱ�������������������ʹ�ò���ֵҲ����
 * ������Ϊ"REVERSE"ʱ�������������������ʹ����ֵ��С
 *
 * @param pid PID������ָ��
 * @param direction ����("DIRECT","REVERSE")
 */
void pid_direction(pid_t pid, enum pid_control_directions dir);

#ifdef	__cplusplus
}
#endif

#endif //RMC_ADAM_GENERALROBOTSYSTEMCODE_PID_CONTROLLER_H
