#include  "bsp/bsp_key.h"

void KEY_GPIO_Init(void)
{
   /* 定义IO硬件初始化结构体变量 */
  GPIO_InitTypeDef GPIO_InitStruct;
	
	/* 使能(开启)KEY引脚对应IO端口时钟 */  
  KEY1_RCC_CLK_ENABLE();
//  KEY2_RCC_CLK_ENABLE();
  
  /* 配置KEY1 GPIO:输入下拉模式 */
  GPIO_InitStruct.Pin = KEY1_GPIO_PIN|KEY2_GPIO_PIN|KEY3_GPIO_PIN|KEY4_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO, &GPIO_InitStruct);  
  
//  /* 配置KEY2 GPIO:输入上拉模式 */
//  GPIO_InitStruct.Pin = KEY2_GPIO_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(KEY2_GPIO, &GPIO_InitStruct);

//	/* 配置KEY3 GPIO:输入上拉模式 */
//  GPIO_InitStruct.Pin = KEY3_GPIO_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(KEY3_GPIO, &GPIO_InitStruct);
//	
//		/* 配置KEY3 GPIO:输入上拉模式 */
//  GPIO_InitStruct.Pin = KEY4_GPIO_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(KEY4_GPIO, &GPIO_InitStruct);
}

/**
  * 函数功能: 读取按键KEY1的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY1_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY2_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY3_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
//      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY4_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
//      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}
