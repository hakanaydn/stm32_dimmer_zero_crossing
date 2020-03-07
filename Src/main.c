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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#define _RX_LEN 3
#define NUM_CHANNELS 2
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint16_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max);
void Zero_Crossing_Int();
void dimTimerISR();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
bool time_Flag = false;
bool ZCD_Flag	=false;

typedef struct UartReceiveData_t
{
	int Rx_indx;
	char Rx_Buffer[_RX_LEN];
	char Rx_data;
	uint8_t flag;
}UartRx_t;

UartRx_t U_RX;

uint16_t DimmerLightDataFromPhone;
uint16_t DimmerLightDataFromPhone2;

volatile bool isHandled[NUM_CHANNELS] = {0,0};
volatile int Lvl_Counter[NUM_CHANNELS] = {120,0};

int NumActiveChannels = 2;
volatile bool zero_cross = 0;
volatile int NumHandled = 0;
char kelime[20];

int Dimming_Lvl[NUM_CHANNELS] = {0,0}; //(0-499) 
int State		   [NUM_CHANNELS] = {1,1};


/* USER CODE END 0 */

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
  MX_USART1_UART_Init();
  MX_TIM13_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_UART_Receive_IT(&huart1,(uint8_t*)&U_RX.Rx_data, 1);	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
//		if(time_Flag){
//					time_Flag = false; 
//					HAL_GPIO_TogglePin(GPIOG, LED2_Pin|LED1_Pin);
//		}
		
	/*	if(ZCD_Flag){
		ZCD_Flag=false;
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		// 10ms=10000us
		// (10000us - 10us) =9990 /10 =990
		DWT_Delay(DimmerLightDataFromPhone);    																				// Off cycle
		HAL_GPIO_WritePin(Trc_out_GPIO_Port, Trc_out_Pin, GPIO_PIN_SET);    // triac firing
		DWT_Delay(10);         																							// triac On propogation delay
		HAL_GPIO_WritePin(Trc_out_GPIO_Port, Trc_out_Pin, GPIO_PIN_RESET);  // triac Off
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	} */
		
		if(time_Flag == true){
		
			sprintf(kelime,"LUKS :%d \r\n",DimmerLightDataFromPhone);
			HAL_UART_Transmit(&huart1,(uint8_t*)kelime,strlen(kelime),10);
			Dimming_Lvl[0] = DimmerLightDataFromPhone;
			Dimming_Lvl[1] = DimmerLightDataFromPhone;
			time_Flag = false;
		}
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 44;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 9;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LAMP_1_Pin|LAMP_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LAMP_1_Pin LAMP_2_Pin */
  GPIO_InitStruct.Pin = LAMP_1_Pin|LAMP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ZCD_Pin */
  GPIO_InitStruct.Pin = ZCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZCD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ZCD_Pin) {
			//ZCD_Flag = true ;
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			Zero_Crossing_Int();
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if (huart->Instance == USART1) {
     		if (U_RX.Rx_data != '}')
				{
					 if(U_RX.Rx_data != '{' ) 
					 {  		
						 if(U_RX.Rx_data >= 0x30 && U_RX.Rx_data <= 0x39 ){
								U_RX.Rx_Buffer[U_RX.Rx_indx] = U_RX.Rx_data;	
								U_RX.Rx_indx++;
								if(U_RX.Rx_indx <= 3) {
									U_RX.flag=1;
								}else {
									U_RX.flag=0;
								}	
							}		
					 }
				}else {
					if(U_RX.flag==1){
							uint8_t PercentOfLight = atoi(U_RX.Rx_Buffer);
							if(PercentOfLight>=0 && PercentOfLight <= 100){
									DimmerLightDataFromPhone = map(100-PercentOfLight, 0, 100, 500, 950);
									DimmerLightDataFromPhone2 = map(PercentOfLight, 0, 100, 500, 950);
									//DimmerLightDataFromPhone = PercentOfLight * 5; 
									time_Flag = true; 
							}
							for (uint8_t i=0;i<_RX_LEN;i++) U_RX.Rx_Buffer[i]=NULL;
							U_RX.Rx_indx = 0;
							U_RX.flag=0;

					 }else {
					 	for (uint8_t i=0;i<_RX_LEN;i++) U_RX.Rx_Buffer[i]=NULL;
						U_RX.flag=0;
						U_RX.Rx_indx = 0;
						U_RX.flag=0;
						 
					 }
				}
				HAL_UART_Receive_IT(&huart1,(uint8_t*)&U_RX.Rx_data, 1);					
	}
}

static uint16_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM13) {
			//time_Flag = true; 	
			dimTimerISR();	
  }
/* USER CODE END Callback 1 */
}

void Zero_Crossing_Int()
{
  if(NumActiveChannels > 0)
  {
    NumHandled = 0;
    
    for(int i=0; i<NUM_CHANNELS; i++)
    {
      isHandled[i] = 0; 
      if(State[i] == 1)
      {
				if(i == 0){
				  HAL_GPIO_WritePin(GPIOC, LAMP_1_Pin, GPIO_PIN_RESET);
				}else if(i  == 1){
				  HAL_GPIO_WritePin(GPIOC, LAMP_2_Pin, GPIO_PIN_RESET);
				}
      }
    }  
    zero_cross = 1; 
  }
}

bool pLampState[2]={false,false};

void dimTimerISR()
{
	
	for(int i = 0; i < NUM_CHANNELS; i++) 
  {
		  if(pLampState[i] == 1)
      {
				if(i == 0){
					HAL_GPIO_WritePin(GPIOC, LAMP_1_Pin, GPIO_PIN_RESET);
					pLampState[i] = false; 
				}else if(i  == 1){
					HAL_GPIO_WritePin(GPIOC, LAMP_2_Pin, GPIO_PIN_RESET);						
					pLampState[i] = false; 
				} 
			}
	}
	
  if(zero_cross == 1)                     
  {
    for(int i = 0; i < NUM_CHANNELS; i++) 
    {
      if(State[i] == 1)
      {
        if(Lvl_Counter[i] > Dimming_Lvl[i] )       
        { 
					if(i == 0){
						HAL_GPIO_WritePin(GPIOC, LAMP_1_Pin, GPIO_PIN_SET);
						pLampState[i] = true; 
					}else if(i  == 1){
						HAL_GPIO_WritePin(GPIOC, LAMP_2_Pin, GPIO_PIN_SET);
						pLampState[i] = true; 
					} 
          Lvl_Counter[i] = 0;  
          isHandled[i] = 1; 
          
          NumHandled++;
          if(NumHandled == NumActiveChannels)
          {    
            zero_cross = 0;     
          }
        } 
        else if(isHandled[i] == 0)
        {
          Lvl_Counter[i]++;                     
        }          
     }
   }
  }
}

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
