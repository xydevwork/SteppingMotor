#include "bsp/bsp_l298n.h"

void L298N_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* ʹ��(����)�˿�ʱ�� */  
  L298N_RCC_CLK_ENABLE();
//	L298N_RCC_CLK_ENABLEB();
	
	HAL_GPIO_WritePin(L298N_GPIO_PORT,L298N_GPIO_PIN,GPIO_PIN_RESET);
	
	/* �趨����IO��� */
  GPIO_InitStruct.Pin = L298N_GPIO_PIN;
  /* �趨����IOΪ���ģʽ */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* �趨����IO�����ٶ� */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* ��ʼ������IO */
  HAL_GPIO_Init(L298N_GPIO_PORT, &GPIO_InitStruct);
	
}
