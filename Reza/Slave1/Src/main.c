/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
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
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Slave1.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//	Registery();
	Slave_Init(10,0);				//7 for wc & 0 for up door
	
  while (1)
  {
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET){
			set_slave1(Send_pck.DIST_PCK.State,1);
			
			if(user_type==SLAVE1_DOOR){
				led_red.state=LED_OFF;
				led_green.state=LED_OFF;
				
				missed_list.index=0;
				called_flag=FLAG_DISABLE;
				calling_flag=FLAG_DISABLE;
				wait_flag=FLAG_DISABLE;
			}
			else if(user_type==SLAVE1_WC){
				led_red.state=LED_TOGGLE_FAST;
			}
		}
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//100ms timer
	if(htim->Instance==TIM1){
		if(led_green.state==LED_TOGGLE_NORM){
			led_green.counter++;
			if(led_green.counter>LED_NORMAL_PERIOD){
				led_green.counter=0;
				HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
			}
		}
		else if(led_green.state==LED_TOGGLE_FAST){
			led_green.counter++;
			if(led_green.counter>LED_FAST_PERIOD){
				led_green.counter=0;
				HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
			}
		}
		else if(led_green.state==LED_ON)HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
		else if(led_green.state==LED_OFF)HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET);

		
		if(led_red.state==LED_TOGGLE_NORM){
			led_red.counter++;
			if(led_red.counter>LED_NORMAL_PERIOD){
				led_red.counter=0;
				HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
			}
		}
		else if(led_red.state==LED_TOGGLE_FAST){
			led_red.counter++;
			if(led_red.counter>LED_FAST_PERIOD){
				led_red.counter=0;
				HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
			}
		}
		else if(led_red.state==LED_ON)HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);
		else if(led_red.state==LED_OFF)HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET);
		
		
		
		if(user_type==SLAVE1_DOOR && registered_flag==FLAG_ENABLE){
			counter_1s++;
			if(counter_1s>10){
				counter_1s=0;
				
				//check state of leds
				if(wait_flag)led_red.state=LED_TOGGLE_FAST;
				else if(missed_list.index!=0)led_red.state=LED_TOGGLE_NORM;
				else led_red.state=LED_OFF;
				
				if(calling_flag)led_green.state=LED_TOGGLE_NORM;
				else if(called_flag)led_green.state=LED_ON;
				else led_green.state=LED_OFF;
				
				calling_flag=FLAG_DISABLE;
				wait_flag=FLAG_DISABLE;
			}
		}
	}		
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	Disable_RS485_Line;
	HAL_UART_Abort(&Slave_Uart);
	HAL_UART_Receive_IT(&Slave_Uart,&buff,1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	pck_state=GetNewData(buff);
	
	if(registered_flag==FLAG_ENABLE){
		if(pck_state==PCK_With_Me){
			HAL_UART_Abort(&huart1);
			Send_PCK(SLAVE1_HELLO,0,0,0);
			if(user_type==SLAVE1_DOOR)set_slave1(Send_pck.DIST_PCK.State,0);
		}
		else{
			HAL_UART_Abort(&Slave_Uart);
			HAL_UART_Receive_IT(&Slave_Uart,&buff,1);	
		}
	}
	else {
		if(pck_state==PCK_REGISTERY){
			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
			if(Received_pck.DIST_PCK.Sensor2==0xFF){
				int aa=GET_USR_ROOM(Received_pck.DIST_PCK.Sensor3);
				if((aa==0) || (aa==7)){
					Slave_Init(GET_ROOM(Received_pck.DIST_PCK.Sensor3),aa);
				}
				else {
					HAL_UART_Abort(&Slave_Uart);
					HAL_UART_Receive_IT(&Slave_Uart,&buff,1);
				}
			}
			else Send_PCK(SLAVE1_REGISTER,0,0,0);
		}
		else {
			HAL_UART_Abort(&Slave_Uart);
			HAL_UART_Receive_IT(&Slave_Uart,&buff,1);
		}
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
