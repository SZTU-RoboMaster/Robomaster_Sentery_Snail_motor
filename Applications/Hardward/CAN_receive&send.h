/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
	*	 V99.99.00	Dec-10-2020			sethome					2,finalDone
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"

//������궨�����ĵ��ID 
//ע�����Ǹ�ͨ�ÿ�ܣ�������ȫ��ϰ�
//���޸�can_msg_id_e
//serialID
#define chassis_FR 0
#define chassis_FL 1
#define chassis_BR 2
#define chassis_BL 3
#define motor_5 4
#define motor_6 5
#define motor_7 6
#define motor_8 7

/* CAN send and receive ID */
typedef enum {
    CAN_1_4_SIGN_ID = 0x200,
    CAN_ID1 = 0x201,
    CAN_ID2 = 0x202,
    CAN_ID3 = 0x203,
    CAN_ID4 = 0x204,

    CAN_5_8_SIGN_ID = 0x1FF,
    CAN_ID5 = 0x205,
    CAN_ID6 = 0x206,
    CAN_ID7 = 0x207,
    CAN_ID8 = 0x208,
} can_msg_id_e;

typedef struct {
    int16_t deg;
    int16_t last_deg;
    int32_t real_deg;
    int16_t last_rpm;
} MotorDeg;

//rm motor data
typedef struct {
    int16_t set_current;//�趨�ĵ���

    int16_t speed_rpm;//ת��
    uint16_t ecd;//������
    int16_t given_current;//������ĵ���
    uint8_t temperate;//�¶ȣ���ȡ������
    int16_t last_ecd;//��һ�α���������ֵ
    MotorDeg ecd2;
} motor_measure_t;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;//����ԭ����can.c�ļ�

extern motor_measure_t Can1_motor_data[8];
extern motor_measure_t Can2_motor_data[8];

void CAN1_send_current(void);//���͵�����Ƶ���
void CAN2_send_current(void);//���͵�����Ƶ���
void set_motor_current(int16_t current, CAN_HandleTypeDef *hcan, char serialID);//�趨������
motor_measure_t get_motor_data(CAN_HandleTypeDef *hcan, char serialID);//��ȡ�����Ϣ
#endif
