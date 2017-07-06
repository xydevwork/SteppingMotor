#ifndef   _BSP_KEY_H_
#define   _BSP_KEY_H_

#include "stm32f0xx_hal.h"

/* ���Ͷ��� --------------------------------------------------------------*/
typedef enum
{
  KEY_UP   = 0,
  KEY_DOWN = 1,
}KEYState_TypeDef;

/* �궨�� --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOB_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_1
#define KEY1_GPIO                     GPIOB
#define KEY1_DOWN_LEVEL               0  /* ����ԭ��ͼ��ƣ�KEY1����ʱ����Ϊ�ߵ�ƽ��������������Ϊ1 */

#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_5
#define KEY2_GPIO                     GPIOA
#define KEY2_DOWN_LEVEL               0  /* ����ԭ��ͼ��ƣ�KEY2����ʱ����Ϊ�͵�ƽ��������������Ϊ0 */

#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_6
#define KEY3_GPIO                     GPIOA
#define KEY3_DOWN_LEVEL               0  /* ����ԭ��ͼ��ƣ�KEY2����ʱ����Ϊ�͵�ƽ��������������Ϊ0 */

#define KEY4_RCC_CLK_ENABLE           __HAL_RCC_GPIOA_CLK_ENABLE
#define KEY4_GPIO_PIN                 GPIO_PIN_7
#define KEY4_GPIO                     GPIOA
#define KEY4_DOWN_LEVEL               0  /* ����ԭ��ͼ��ƣ�KEY2����ʱ����Ϊ�͵�ƽ��������������Ϊ0 */

//#define KEY_ENABLE 										__HAL_RCC_GPIOA_CLK_DISABLE()

/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/
void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);
KEYState_TypeDef KEY4_StateRead(void);
#endif
