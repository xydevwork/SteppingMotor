#ifndef _flash_H
#define _flash_H
#include "stm32f0xx_hal.h"



void FLASHINIT_FLASH(uint32_t input_flash_data);

extern uint32_t		count_number;
extern uint16_t  exti_flag;

//����ȫ�ֱ���
//extern u32 flash_data;

#define flash_adr 0x08003000	//�������洢���ı������ݵ���ʼ��ַ


#endif
