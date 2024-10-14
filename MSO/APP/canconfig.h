#ifndef __CANCONFIG_H__
#define __CANCONFIG_H__

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

uint8_t bsp_can1_filter_config(void);

uint8_t bsp_can1_filter_config_list(void);

uint8_t bsp_can1_filter_config_16(void);

uint8_t bsp_can1_filter_config_16_list(void);

uint8_t bsp_can2_filter_config(void);

void Can1Receive(void);

void Can2Receive(void);

void can1_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len);

void can2_transmit(uint32_t id_type,uint32_t basic_id,uint32_t ex_id,uint8_t *data,uint32_t data_len);

void Can1AndCan2CommunicationTest(void);

void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1);

typedef struct 
{
	uint16_t can_id;
	uint16_t  set_voltage ; //��ѹֵ
	uint16_t rotor_angle;  //��е�Ƕ�
	uint16_t  rotor_speed; //ת��
	uint16_t  torque_current;  //Ť�ص���
	uint8_t   temp;   //�¶�
	    
}motor_info_t;


#endif


