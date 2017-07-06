#include "bsp/bsp_l298n.h"

void L298N_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* 使能(开启)端口时钟 */  
  L298N_RCC_CLK_ENABLE();
//	L298N_RCC_CLK_ENABLEB();
	
	HAL_GPIO_WritePin(L298N_GPIO_PORT,L298N_GPIO_PIN,GPIO_PIN_RESET);
	
	/* 设定引脚IO编号 */
  GPIO_InitStruct.Pin = L298N_GPIO_PIN;
  /* 设定引脚IO为输出模式 */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  /* 设定引脚IO操作速度 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  /* 初始化引脚IO */
  HAL_GPIO_Init(L298N_GPIO_PORT, &GPIO_InitStruct);
	
}
