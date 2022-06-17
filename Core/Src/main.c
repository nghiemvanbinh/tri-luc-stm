/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SimpleKalmanFilter.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define mang_tiem_can1 HAL_GPIO_ReadPin(tiem_can1_GPIO_Port,tiem_can1_Pin);// tiem can trai nhin doi dien
#define mang_tiem_can2 HAL_GPIO_ReadPin(tiem_can2_GPIO_Port,tiem_can2_Pin);// tiem can giua
#define mang_tiem_can3 HAL_GPIO_ReadPin(tiem_can3_GPIO_Port,tiem_can3_Pin);// tiem can phai
//uint8_t mang_tiem_can[3]={mang_tiem_can,mang_tiem_can2,mang_tiem_can3};
volatile uint32_t ADC_value1,adc_kalman;
SimpleKalmanFilter adc = SimpleKalmanFilter(2,2,0.01);
uint8_t gia_tri_tiem_cam, gia_tri2,gia_tri3;
char pData[]= "{\"luc\":";

char dau_ngoac[]="}";
//uint8_t data_adc[10];

float max;
int i=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t tiem_can()
{
	uint8_t dich_trai, gia_tri;
    dich_trai = 0;
	  gia_tri=0;
		gia_tri   = mang_tiem_can1;
	  dich_trai = dich_trai |(gia_tri<< 2);
	  gia_tri   = mang_tiem_can2;
	  dich_trai = dich_trai |(gia_tri<<1);
	  gia_tri   = mang_tiem_can3;
	  dich_trai = dich_trai | gia_tri;
	  return dich_trai;
}
uint32_t read_average(uint32_t value) {
   static uint32_t sum = 0;
   static uint32_t store[16];
   static char pos=0;
   static int start_calc=0;
   if(start_calc)
      {sum=sum-store[pos];}
   sum=sum+value;
   store[pos]=value;
   pos++;
   if(pos>7)
      {
      pos=0;
      start_calc=1;
      }
   return sum>>3;

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
 if(hadc->Instance == hadc1.Instance) 
 {
   ADC_value1 = HAL_ADC_GetValue(&hadc1);
	// adc_kalman = read_average(adc.updateEstimate(ADC_value1));
 }
}
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

int vi_tri()
{
	int vi_tri_dam;
	if(tiem_can()==2)
	{
		vi_tri_dam=2;
	}
	else if(tiem_can()==4)
	{
		vi_tri_dam=4;
	}
	else if(tiem_can()==1)
	{
		vi_tri_dam=1;
	}
	else
	{
		vi_tri_dam=0;
	}
	return vi_tri_dam;
}

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  // HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		gia_tri2 = tiem_can();
		if(gia_tri2 !=0)
		{
			gia_tri_tiem_cam=1;
			adc_kalman = read_average(adc.updateEstimate(ADC_value1));
			if(adc_kalman > max)max=adc_kalman;
			gia_tri3 =  gia_tri2;
		}
		else
		{
			if(gia_tri_tiem_cam == 1)
			{
				char data_adc[5],data_position[5];
        char chuoi_trung_gian[30];
				sprintf(data_adc,"%f",max);
		    sprintf(data_position,"%d",gia_tri3);
		    strcpy(chuoi_trung_gian,pData);
				strcat(chuoi_trung_gian,data_adc);
		    strcat(chuoi_trung_gian,",\"position\":");
		    strcat(chuoi_trung_gian,data_position);
				strcat(chuoi_trung_gian,dau_ngoac);
				HAL_UART_Transmit(&huart1, (uint8_t *)chuoi_trung_gian,strlen((char *)chuoi_trung_gian),100);
				max=0;
				
			}
			gia_tri_tiem_cam =0;
		}
		adc_kalman = read_average(adc.updateEstimate(ADC_value1));
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
