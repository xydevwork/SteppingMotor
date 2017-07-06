#ifndef   _BSP_KEY_H_
#define   _BSP_KEY_H_

#include "stm32f0xx_hal.h"

/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_4
#define KEY1_GPIO                     GPIOA
#define KEY1_DOWN_LEVEL               0  /* 根据原理图设计，KEY1按下时引脚为高电平，所以这里设置为1 */

#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_5
#define KEY2_GPIO                     GPIOA
#define KEY2_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_6
#define KEY3_GPIO                     GPIOA
#define KEY3_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY4_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY4_GPIO_PIN                 GPIO_PIN_7
#define KEY4_GPIO                     GPIOA
#define KEY4_DOWN_LEVEL               0  /* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY_ENABLE 										__HAL_RCC_GPIOA_CLK_DISABLE()

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);
KEYState_TypeDef KEY4_StateRead(void);
#endif
