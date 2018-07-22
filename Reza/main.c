
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
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define DATA_SIZE	2000

#define Packet_Length 10
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define START_BYTE2 0x00
#define START_BYTE3 0xFF
#define STOP_BYTE 0x80
#define Buffer_Size Packet_Length

// state decode
#define read_call(A)		(A&0x80)==0x80
#define read_req(A)			(A&0x40)==0x40
#define read_lamp(A)		(A&0x20)==0x20
#define read_slamp(A)		(A&0x10)==0x10
#define read_slave1(A)	(A&0x08)==0x08
#define read_VIP(A)			(A&0x04)==0x04
#define read_user(A)		(A&0x03)

#define set_call(A,B)			A=((A&0x7F)|((uint8_t)(B<<7)))
#define set_req(A,B)			A=((A&0xBF)|((uint8_t)(B<<6)))
#define set_lamp(A,B)			A=((A&0xDF)|((uint8_t)(B<<5)))
#define set_slamp(A,B)		A=((A&0xEF)|((uint8_t)(B<<4)))
#define set_slave1(A,B)		A=((A&0xF7)|((uint8_t)(B<<3)))
#define set_VIP(A,B)			A=((A&0xFB)|((uint8_t)(B<<2)))
#define set_user(A,B)			A=((A&0xFC)|((uint8_t)(B)))

#define call_flag		 			read_call(send_pck.DIST_PCK.State)
#define req_flag	 				read_req(send_pck.DIST_PCK.State)

typedef union
{
	struct DATA_CONV
  {
    uint8_t ST0;
    uint8_t ST1;
    uint8_t addr;
    uint8_t func;
		uint8_t State;
		uint8_t Sensor1;
		uint8_t Sensor2;
		uint8_t Sensor3;
		uint8_t cksum;
		uint8_t stp;
  }DIST_PCK;															// Distributed packet
	
  uint8_t ASS_PCK[Packet_Length];					// Associated  packet
}PCK_CONV;

typedef enum{
	MASTER_HELLO,
	MASTER_REQ_AUDIO,
	MASTER_AUDIO_ALL,
	MASTER_REGISTER,
	
	SLAVE1_HELLO,
	SLAVE1_REGISTER,
	
	SLAVE2_HELLO,
	SLAVE2_ACK_AUDIO,
	SLAVE2_REGISTER,
	
	VATAN,
}FUNCTION;

typedef enum {
	VIP=0x3,
	SLAVE2=0x0,
	SLAVE1_WC=0x1,
	SLAVE1_DOOR=0x2
}USERS;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t buff[DATA_SIZE];
uint8_t flag=0;

PCK_CONV Send_pck;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_RESET);
//	HAL_UART_Receive_DMA(&huart2,buff,DATA_SIZE);
//	for(uint16_t ii=0;ii<DATA_SIZE;ii++)buff[ii]=0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_SET);
//	buff[1]=100;
	HAL_UART_Transmit_DMA(&huart2,buff,DATA_SIZE);
}

void Init_PCK(PCK_CONV * pck,uint8_t address,FUNCTION function,uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4){
	pck->DIST_PCK.ST0=START_BYTE0;
	pck->DIST_PCK.ST1=START_BYTE1;
	pck->DIST_PCK.func=function;
	pck->DIST_PCK.addr=address;
	pck->DIST_PCK.State=data1;
	pck->DIST_PCK.Sensor1=data2;
	pck->DIST_PCK.Sensor2=data3;
	pck->DIST_PCK.Sensor3=data4;
	
	int a=function+address+data1+data2+data3+data4;
	
	pck->DIST_PCK.cksum=a%256;
	pck->DIST_PCK.stp=STOP_BYTE;
}

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	uint8_t hour=14;
	uint8_t min=23;
	uint8_t second=15;
	
	uint8_t bimar=(uint8_t)(((0x10)<<4)+(0x2));
	
	uint8_t addr=0;
	
	HAL_UART_Receive_IT(&huart2,buff,DATA_SIZE);

	while (1)
  {
//		if(flag==0){
//			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
//			HAL_UART_Abort(&huart2);
//			HAL_UART_Receive_IT(&huart2,buff,DATA_SIZE);
//			HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_RESET);
//		}
//		else HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);
//		
		
		HAL_Delay(200);
		Init_PCK(&Send_pck,0xff,VATAN,0,hour,min,second);
		HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart2,Send_pck.ASS_PCK,10);
		
		addr=(uint8_t)(((5)<<4)+(6));
		set_call(Send_pck.DIST_PCK.State,0);
		set_slamp(Send_pck.DIST_PCK.State,0);
		set_lamp(Send_pck.DIST_PCK.State,0);
		set_req(Send_pck.DIST_PCK.State,1);
		set_call(Send_pck.DIST_PCK.State,0);
		set_user(Send_pck.DIST_PCK.State,SLAVE2);
		set_VIP(Send_pck.DIST_PCK.State,1);
		HAL_Delay(200);
		Init_PCK(&Send_pck,addr,SLAVE2_HELLO,Send_pck.DIST_PCK.State,0,0,0);
		HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart2,Send_pck.ASS_PCK,10);

		addr=(uint8_t)(((1)<<4)+(2));
		set_call(Send_pck.DIST_PCK.State,0);
		set_slamp(Send_pck.DIST_PCK.State,0);
		set_lamp(Send_pck.DIST_PCK.State,0);
		set_req(Send_pck.DIST_PCK.State,0);
		set_call(Send_pck.DIST_PCK.State,0);
		set_user(Send_pck.DIST_PCK.State,SLAVE2);
		set_VIP(Send_pck.DIST_PCK.State,0);
		HAL_Delay(200);
		Init_PCK(&Send_pck,addr,SLAVE2_HELLO,Send_pck.DIST_PCK.State,0,0,0);
		HAL_GPIO_WritePin(RS485_COPY_ENABLE_GPIO_Port,RS485_COPY_ENABLE_Pin,GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart2,Send_pck.ASS_PCK,10);

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
