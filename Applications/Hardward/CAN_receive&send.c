/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive&send.h"

//电机数据读取
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

motor_measure_t Can1_motor_data[8];
motor_measure_t Can2_motor_data[8];

void MotorDeg_update(motor_measure_t *motor) {

    motor->ecd2.deg = motor->ecd;

    //增量计算
    if (motor->ecd2.last_rpm > 0 && motor->ecd2.deg < motor->ecd2.last_deg)
        motor->ecd2.real_deg += (motor->ecd2.deg + 8191 - motor->ecd2.last_deg);
    else if (motor->ecd2.last_rpm < 0 && motor->ecd2.deg > motor->ecd2.last_deg)
        motor->ecd2.real_deg += (motor->ecd2.last_deg - 8191 - motor->ecd2.deg);
    else motor->ecd2.real_deg += (motor->ecd2.deg - motor->ecd2.last_deg);

    //循环
    if (157293 < motor->ecd2.real_deg) motor->ecd2.real_deg = motor->ecd2.real_deg - 157293;
    if (0 > motor->ecd2.real_deg) motor->ecd2.real_deg = motor->ecd2.real_deg + 157293;

    //更新变量
    motor->ecd2.last_rpm = motor->speed_rpm;
    motor->ecd2.last_deg = motor->ecd2.deg;

}

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    motor_measure_t *motor_data;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//取得信息
    if (hcan == &hcan1)//sethome 修改后支持CAN2
        motor_data = Can1_motor_data;
    else
        motor_data = Can2_motor_data;

    switch (rx_header.StdId) {
        case CAN_ID1: {
            get_motor_measure(motor_data, rx_data);
            MotorDeg_update(motor_data);
            break;
        }
        case CAN_ID2: {
            get_motor_measure(motor_data + 1, rx_data);
            break;
        }
        case CAN_ID3: {
            get_motor_measure(motor_data + 2, rx_data);
            break;
        }
        case CAN_ID4: {
            get_motor_measure(motor_data + 3, rx_data);
            break;
        }
        case CAN_ID5: {
            get_motor_measure(motor_data + 4, rx_data);
            break;
        }
        case CAN_ID6: {
            get_motor_measure(motor_data + 5, rx_data);
            break;
        }
        case CAN_ID7: {
            get_motor_measure(motor_data + 6, rx_data);
            break;
        }
        case CAN_ID8: {
            get_motor_measure(motor_data + 7, rx_data);
            break;
        }
        default: {
            break;
        }
    }
}

motor_measure_t get_motor_data(CAN_HandleTypeDef *hcan, char serialID)//获取马达数据
{
    motor_measure_t *target_data;
    if (hcan == &hcan1)
        target_data = &Can1_motor_data[serialID];
    else if (hcan == &hcan2)
        target_data = &Can2_motor_data[serialID];
    return *target_data;
}

void set_motor_current(int16_t current, CAN_HandleTypeDef *hcan, char serialID)//设定马达电流
{
    if (hcan == &hcan1)
        Can1_motor_data[serialID].set_current = current;
    else if (hcan == &hcan2)
        Can2_motor_data[serialID].set_current = current;
}

void CAN2_send_current()//发送电机控制电流
{
    uint8_t can_send_data[8];
    static CAN_TxHeaderTypeDef tx_message;
    uint32_t send_mail_box;

    //发送前4个
    tx_message.StdId = CAN_1_4_SIGN_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    can_send_data[0] = (Can2_motor_data[0].set_current >> 8);
    can_send_data[1] = Can2_motor_data[0].set_current;

    can_send_data[2] = (Can2_motor_data[1].set_current >> 8);
    can_send_data[3] = Can2_motor_data[1].set_current;

    can_send_data[4] = (Can2_motor_data[2].set_current >> 8);
    can_send_data[5] = Can2_motor_data[2].set_current;

    can_send_data[6] = (Can2_motor_data[3].set_current >> 8);
    can_send_data[7] = Can2_motor_data[3].set_current;
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);

    //发送后4个
    tx_message.StdId = CAN_5_8_SIGN_ID;

    can_send_data[0] = (Can2_motor_data[4].set_current >> 8);
    can_send_data[1] = Can2_motor_data[4].set_current + 4;

    can_send_data[2] = (Can2_motor_data[5].set_current >> 8);
    can_send_data[3] = Can2_motor_data[5].set_current;

    can_send_data[4] = (Can2_motor_data[6].set_current >> 8);
    can_send_data[5] = Can2_motor_data[6].set_current;

    can_send_data[6] = (Can2_motor_data[7].set_current >> 8);
    can_send_data[7] = Can2_motor_data[7].set_current;
    HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
}

void CAN1_send_current()//发送电机控制电流
{
    uint8_t can_send_data[8];
    static CAN_TxHeaderTypeDef tx_message;
    uint32_t send_mail_box;

    //发送前4个
    tx_message.StdId = CAN_1_4_SIGN_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;

    can_send_data[0] = (Can1_motor_data[0].set_current >> 8);
    can_send_data[1] = Can1_motor_data[0].set_current & 0xff;

    can_send_data[2] = (Can1_motor_data[1].set_current >> 8);
    can_send_data[3] = Can1_motor_data[1].set_current & 0xff;

    can_send_data[4] = (Can1_motor_data[2].set_current >> 8);
    can_send_data[5] = Can1_motor_data[2].set_current & 0xff;

    can_send_data[6] = (Can1_motor_data[3].set_current >> 8);
    can_send_data[7] = Can1_motor_data[3].set_current & 0xff;
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);

    //发送后4个
    tx_message.StdId = CAN_5_8_SIGN_ID;

    can_send_data[0] = (Can1_motor_data[4].set_current >> 8);
    can_send_data[1] = Can1_motor_data[4].set_current & 0xff;

    can_send_data[2] = (Can1_motor_data[5].set_current >> 8);
    can_send_data[3] = Can1_motor_data[5].set_current & 0xff;

    can_send_data[4] = (Can1_motor_data[6].set_current >> 8);
    can_send_data[5] = Can1_motor_data[6].set_current & 0xff;

    can_send_data[6] = (Can1_motor_data[7].set_current >> 8);
    can_send_data[7] = Can1_motor_data[7].set_current & 0xff;
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
}
