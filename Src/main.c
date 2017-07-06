/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL-
 THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bsp/bsp_key.h"
#include "bsp/bsp_l298n.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define EIGHTSTEP   									1
#define STEPMOTOR_SPEED               1  	 // 定义步进电机速度，值越小，速度越快,最小不能小于1                                        
#define STEPMOTOR_CIRCLE_NUMBER       0    //  转动圈数
#define STEPMOTOR_DIRECTION           1    // 1：顺时针  0：逆时针

/* 私有变量 ------------------------------------------------------------------*/
// 速度，该值越小，速度越快，最小不能小于8
uint8_t speed=STEPMOTOR_SPEED;
// 转动圈数：步进电机的步距角度为18°，即每个脉冲转18°
// 要转一圈需要360/18=20个脉冲。
uint32_t Circle_number=STEPMOTOR_CIRCLE_NUMBER;
// 选择方向控制
uint8_t direction = STEPMOTOR_DIRECTION;

uint8_t tmr_flag;  // 1ms定时器标志位 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Motor_Direction(void);
void Motor_Stop(void);
void Motor_ToEnd(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
  * 函数功能: 输出一个数据给ULN2003从而实现向步进电机发送一个脉冲
  * 输入参数: step：指定步进序号，可选值0~7
  *           direction：方向选择
  *               可选值：1：顺时针
  *                       0：逆时针
  * 返 回 值: 无
  * 说    明: 无
  */

static void step_motor_pulse(uint8_t step,uint8_t direction)
{
  uint8_t temp=step;
	
  if(direction==1)    // 如果为顺时针旋转
  {
		switch(temp)
		{
			/*8个节拍控制：A->AB->B->BA_->A_->A_B_->B_->B_A
				 A_P_ON-->GPIO_PIN_0-->IN1 
				 A_N_ON-->GPIO_PIN_1-->IN2 
				 B_P_ON-->GPIO_PIN_2-->IN3
				 B_N_ON-->GPIO_PIN_3-->IN3 */
			case 0:
				A_P_ON;  A_N_OFF; B_P_OFF; B_N_OFF;//A   1000
			break;
			case 1:
				A_P_ON;  A_N_OFF; B_P_ON;  B_N_OFF;//AB  1010
			break;
			case 2:
				A_P_OFF; A_N_OFF; B_P_ON;  B_N_OFF;//B   0010
			break;
			case 3:
				A_P_OFF; A_N_ON;  B_P_ON;  B_N_OFF;//BA_ 0110
			break;
			case 4:
				A_P_OFF; A_N_ON;  B_P_OFF; B_N_OFF;//A_  0100
			break;
			case 5:
				A_P_OFF; A_N_ON;  B_P_OFF; B_N_ON; //A_B_0101
			break;
			case 6:
				A_P_OFF; A_N_OFF; B_P_OFF; B_N_ON; //B_  0001
			break;
			case 7:
				A_P_ON;  A_N_OFF; B_P_OFF; B_N_ON; //B_A 1001
			break;
		default:
				break;
		}
	}
	else //逆时针转
	{
		switch(temp)
		{
			/*8个节拍控制：A->AB->B->BA_->A_->A_B_->B_->B_A
				 A_P_ON-->GPIO_PIN_0-->IN1 
				 A_N_ON-->GPIO_PIN_1-->IN2 
				 B_P_ON-->GPIO_PIN_2-->IN3
				 B_N_ON-->GPIO_PIN_3-->IN3 */
			case 7:
				A_P_ON;  A_N_OFF; B_P_OFF; B_N_OFF;//A   1000
			break;
			case 6:
				A_P_ON;  A_N_OFF; B_P_ON;  B_N_OFF;//AB  1010
			break;
			case 5:
				A_P_OFF; A_N_OFF; B_P_ON;  B_N_OFF;//B   0010
			break;
			case 4:
				A_P_OFF; A_N_ON;  B_P_ON;  B_N_OFF;//BA_ 0110
			break;
			case 3:
				A_P_OFF; A_N_ON;  B_P_OFF; B_N_OFF;//A_  0100
			break;
			case 2:
				A_P_OFF; A_N_ON;  B_P_OFF; B_N_ON; //A_B_0101
			break;
			case 1:
				A_P_OFF; A_N_OFF; B_P_OFF; B_N_ON; //B_  0001
			break;
			case 0:
				A_P_ON;  A_N_OFF; B_P_OFF; B_N_ON; //B_A 1001
			break;
			default:
				break;
		}
	}
}
/* USER CODE END 0 */
uint8_t direction_flag; //微动开关标志位 1 ：逆时针转到底 2：顺时针转到底
//GPIO_PinState PinState;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured perip  herals */
 
  /* USER CODE BEGIN 2 */
	KEY_GPIO_Init(); 		//键盘初始化
	L298N_GPIO_Init();	//步进电机驱动初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	
  /* USER CODE BEGIN 3 */
	Motor_Direction(); 												//检测方向控制按钮是否按下	
  Motor_ToEnd();		 												//检测微动开关是否按下	
		if (tmr_flag )	 												//1ms执行一次
		{
			tmr_flag =0;	 												//清除中断标志位
			static uint8_t count=0;               // 用于旋转速度控制
			static uint8_t step=0;                // 当前步进节拍
			static uint16_t pulse_count=0;        // 脉冲计数，20个脉冲电机旋转一圈

			if(Circle_number)                     // 如果等待旋转圈数不为0
			{
				count++;                            // 增加时间计数
				if(count==speed)                    // 时间计数与目标速度相等时执行下一节拍输出 20ms
				{
					step_motor_pulse(step,direction); // 输出新节拍信号
					pulse_count++;                    // 脉冲输出数增加      
					step++;                           // 节拍数增加
					if(step==8) step=0;               // 循环开始输出节拍
					count=0;                          // 清零时间计数
				}
				if(pulse_count==20)               	// 如果已经输出了20个脉冲信号，已经转动了一圈
				{
					pulse_count=0;                    // 脉冲计数清零
					Circle_number--;                  // 等待旋转圈数减1
				}
			}
			else
			{
				Motor_Stop();												// 停止转动
			}
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	tmr_flag = 1;
}
//控制电机转动方向
void Motor_Direction(void)
{
	
	if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
	{
	/* 延时一小段时间，消除抖动 */
	//HAL_Delay(10);
	/* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
		if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL&&STEPMOTOR_CIRCLE_NUMBER==0)
		{
			if(direction_flag ==1)  //逆时针转到底 
			{
				Motor_Stop();					//立即停止电机转动
				direction_flag = 0;		//清除状态标识
				return;
			}	
				direction = 0;				//逆时针转动电机
				Circle_number =1000;	//设置转动圈数
		}
	}else if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
	{
	/* 延时一小段时间，消除抖动 */
	//	HAL_Delay(10);
	/* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
		if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL&&STEPMOTOR_CIRCLE_NUMBER==0)
		{
			if(direction_flag ==2)   //顺时针转到底 
			{
				Motor_Stop();					 //立即停止电机转动
				direction_flag = 0;		 //清除状态标识		
				return;
			}
				direction = 1;				 //顺时针转动电机
				Circle_number =1000;   //设置转动圈数
		}
	}else{
		Motor_Stop();
	}
}
// 电机停止转动
void Motor_Stop(void)
{
	A_P_OFF; A_N_OFF; B_P_OFF; B_N_OFF; // 停止转动
//	Circle_number = 0;
}

//电机转到终点
void Motor_ToEnd(void)
{
	if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
	{
	/* 延时一小段时间，消除抖动 */
//	HAL_Delay(10);
	/* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下
	PinState = HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN); */
		if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
		{
			switch(direction)
			{
				case 0:
					direction_flag =1; //逆时针转到终点
				break;
				default:
					direction_flag =0; //默认方向按钮控制电机
					break;
			}
		}
	}
	
	if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
	{
	/* 延时一小段时间，消除抖动 */
//	HAL_Delay(10);
	/* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下
	PinState = HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN); */
		if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
		{
			switch(direction)
			{
				case 1:
					direction_flag =2; //顺时针转到终点
				break;
				default:
					direction_flag =0; //默认方向按钮控制电机
					break;
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
