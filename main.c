/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max30102.h"
#include "max30102_fir.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t max30102_int_flag=0;  		//中断标志
float max30102_data[2],fir_output[2];
#define CACHE_NUMS 150//缓存数
float ppg_data_cache_RED[CACHE_NUMS]={0};  //缓存区
float ppg_data_cache_IR[CACHE_NUMS]={0};  //缓存区
#define PPG_DATA_THRESHOLD 100000 	//检测阈值
uint16_t cache_counter=0;  //缓存计数器
int heartbeat = 0;	//心跳				
float blood_oxygen =0 ;  //血氧


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if(GPIO_Pin==MAX30102_INT_Pin)
		{				
			max30102_int_flag = 1 ;
    }
}

void MAX30102_WORK(){
	if(max30102_int_flag == 1)
		{
		    max30102_fifo_read(max30102_data);		//读取数据
				
				max30102_int_flag = 0;
					ir_max30102_fir(&max30102_data[0],&fir_output[0]);
					red_max30102_fir(&max30102_data[1],&fir_output[1]);  //滤波
					if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //大于阈值，说明传感器有接触
					{		
							ppg_data_cache_IR[cache_counter]=fir_output[0];
							ppg_data_cache_RED[cache_counter]=fir_output[1];
							cache_counter++;
					}
					else				//小于阈值
					{
							cache_counter=0;
					}


					if(cache_counter>=CACHE_NUMS)  //收集满了数据
					{
						heartbeat = max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS);
						blood_oxygen = max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS);
						cache_counter=0;
					}
       }
		 }
//我是将这段直接放中段了，因为我while里面东西很多，我直接放中断跳出程序来做，如果数据不准确
//看看是不是线太松了我被线坑了两天
//F1RCT6的板子我OLED和MAX30102都用硬件I2，MAX30102会卡死，不输出数据，如果要用oled我推荐用软件的I2
//以下是我程序里面的
		 
/*void MAX30102_WORK(){
		    max30102_fifo_read(max30102_data);		//读取数据
				
				max30102_int_flag = 0;
					ir_max30102_fir(&max30102_data[0],&fir_output[0]);
					red_max30102_fir(&max30102_data[1],&fir_output[1]);  //滤波
					if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //大于阈值，说明传感器有接触
					{		
							ppg_data_cache_IR[cache_counter]=fir_output[0];
							ppg_data_cache_RED[cache_counter]=fir_output[1];
							cache_counter++;
					}
					else				//小于阈值
					{
							cache_counter=0;
					}


					if(cache_counter>=CACHE_NUMS)  //收集满了数据
					{
						heartbeat = max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS);
						blood_oxygen = max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS);
						cache_counter=0;
					}
		 }

		 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if(GPIO_Pin==MAX30102_INT_Pin)
		{		
			MAX30102_WORK();
}
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	max30102_init();
	max30102_fir_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
 {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
