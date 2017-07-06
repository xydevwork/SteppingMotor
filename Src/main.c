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
#define STEPMOTOR_SPEED               1  	 // ���岽������ٶȣ�ֵԽС���ٶ�Խ��,��С����С��1                                        
#define STEPMOTOR_CIRCLE_NUMBER       0    //  ת��Ȧ��
#define STEPMOTOR_DIRECTION           1    // 1��˳ʱ��  0����ʱ��

/* ˽�б��� ------------------------------------------------------------------*/
// �ٶȣ���ֵԽС���ٶ�Խ�죬��С����С��8
uint8_t speed=STEPMOTOR_SPEED;
// ת��Ȧ������������Ĳ���Ƕ�Ϊ18�㣬��ÿ������ת18��
// ҪתһȦ��Ҫ360/18=20�����塣
uint32_t Circle_number=STEPMOTOR_CIRCLE_NUMBER;
// ѡ�������
uint8_t direction = STEPMOTOR_DIRECTION;

uint8_t tmr_flag;  // 1ms��ʱ����־λ 

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
  * ��������: ���һ�����ݸ�ULN2003�Ӷ�ʵ���򲽽��������һ������
  * �������: step��ָ��������ţ���ѡֵ0~7
  *           direction������ѡ��
  *               ��ѡֵ��1��˳ʱ��
  *                       0����ʱ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */

static void step_motor_pulse(uint8_t step,uint8_t direction)
{
  uint8_t temp=step;
	
  if(direction==1)    // ���Ϊ˳ʱ����ת
  {
		switch(temp)
		{
			/*8�����Ŀ��ƣ�A->AB->B->BA_->A_->A_B_->B_->B_A
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
	else //��ʱ��ת
	{
		switch(temp)
		{
			/*8�����Ŀ��ƣ�A->AB->B->BA_->A_->A_B_->B_->B_A
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
uint8_t direction_flag; //΢�����ر�־λ 1 ����ʱ��ת���� 2��˳ʱ��ת����
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
	KEY_GPIO_Init(); 		//���̳�ʼ��
	L298N_GPIO_Init();	//�������������ʼ��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	
  /* USER CODE BEGIN 3 */
	Motor_Direction(); 												//��ⷽ����ư�ť�Ƿ���	
  Motor_ToEnd();		 												//���΢�������Ƿ���	
		if (tmr_flag )	 												//1msִ��һ��
		{
			tmr_flag =0;	 												//����жϱ�־λ
			static uint8_t count=0;               // ������ת�ٶȿ���
			static uint8_t step=0;                // ��ǰ��������
			static uint16_t pulse_count=0;        // ���������20����������תһȦ

			if(Circle_number)                     // ����ȴ���תȦ����Ϊ0
			{
				count++;                            // ����ʱ�����
				if(count==speed)                    // ʱ�������Ŀ���ٶ����ʱִ����һ������� 20ms
				{
					step_motor_pulse(step,direction); // ����½����ź�
					pulse_count++;                    // �������������      
					step++;                           // ����������
					if(step==8) step=0;               // ѭ����ʼ�������
					count=0;                          // ����ʱ�����
				}
				if(pulse_count==20)               	// ����Ѿ������20�������źţ��Ѿ�ת����һȦ
				{
					pulse_count=0;                    // �����������
					Circle_number--;                  // �ȴ���תȦ����1
				}
			}
			else
			{
				Motor_Stop();												// ֹͣת��
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
//���Ƶ��ת������
void Motor_Direction(void)
{
	
	if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
	{
	/* ��ʱһС��ʱ�䣬�������� */
	//HAL_Delay(10);
	/* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
		if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL&&STEPMOTOR_CIRCLE_NUMBER==0)
		{
			if(direction_flag ==1)  //��ʱ��ת���� 
			{
				Motor_Stop();					//����ֹͣ���ת��
				direction_flag = 0;		//���״̬��ʶ
				return;
			}	
				direction = 0;				//��ʱ��ת�����
				Circle_number =1000;	//����ת��Ȧ��
		}
	}else if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
	{
	/* ��ʱһС��ʱ�䣬�������� */
	//	HAL_Delay(10);
	/* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������ */
		if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL&&STEPMOTOR_CIRCLE_NUMBER==0)
		{
			if(direction_flag ==2)   //˳ʱ��ת���� 
			{
				Motor_Stop();					 //����ֹͣ���ת��
				direction_flag = 0;		 //���״̬��ʶ		
				return;
			}
				direction = 1;				 //˳ʱ��ת�����
				Circle_number =1000;   //����ת��Ȧ��
		}
	}else{
		Motor_Stop();
	}
}
// ���ֹͣת��
void Motor_Stop(void)
{
	A_P_OFF; A_N_OFF; B_P_OFF; B_N_OFF; // ֹͣת��
//	Circle_number = 0;
}

//���ת���յ�
void Motor_ToEnd(void)
{
	if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
	{
	/* ��ʱһС��ʱ�䣬�������� */
//	HAL_Delay(10);
	/* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������
	PinState = HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN); */
		if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
		{
			switch(direction)
			{
				case 0:
					direction_flag =1; //��ʱ��ת���յ�
				break;
				default:
					direction_flag =0; //Ĭ�Ϸ���ť���Ƶ��
					break;
			}
		}
	}
	
	if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
	{
	/* ��ʱһС��ʱ�䣬�������� */
//	HAL_Delay(10);
	/* ��ʱʱ��������жϰ���״̬��������ǰ���״̬˵������ȷʵ������
	PinState = HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN); */
		if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
		{
			switch(direction)
			{
				case 1:
					direction_flag =2; //˳ʱ��ת���յ�
				break;
				default:
					direction_flag =0; //Ĭ�Ϸ���ť���Ƶ��
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
