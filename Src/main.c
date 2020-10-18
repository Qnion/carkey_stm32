#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
//#include "delay.h"
#include "key.h"
#include "myiic.h"
#include "max30102.h"
#include "algorithm.h"
#include "string.h"

/* USER CODE BEGIN Includes */
/**
************************************************************
* @file         main.c
* @brief        MCU�? 入口文件
* @author       Gizwits
* @date         2017-05-12
* @version      V01
* @copyright    Gizwits
* 
* @note         机智�?.只为智能硬件而生
*               Gizwits Smart Cloud  for Smart Products
*               链接|增�?�ֵ|�?放|中立|安全|自有|自由|生�??
*               www.gizwits.com
*
***********************************************************/
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define VREF                  3300
/* Variable used to get converted value */
uint32_t ConvertedValue = 0;
int16_t alcohol = 0;
/* USER CODE END PV */
#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[150]; //infrared LED sensor data
uint32_t aun_red_buffer[150];  //red LED sensor data
int32_t n_ir_buffer_length; //data length
float  n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

int32_t hr_buf[16];
int32_t hrSum;
int32_t hrAvg;
float spo2_buf[16];
float spo2Sum;
float spo2Avg;
int32_t spo2BuffFilled;
int32_t hrBuffFilled;
int32_t hrValidCnt = 0;
int32_t spo2ValidCnt = 0;
int32_t hrThrowOutSamp = 0;
int32_t spo2ThrowOutSamp = 0;
int32_t spo2Timeout = 0;
int32_t hrTimeout = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void loop(void);
uint8_t aTxEndMessage[] = "\r\n Example Finished\r\n";
uint8_t aRxBuffer[18];
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
	KEY_Init();
	bsp_InitI2C();
	maxim_max30102_reset();
	maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy);
	maxim_max30102_init();
	
  /* USER CODE BEGIN 2 */
	printf("Hello My Deer Teachers!\r\n");
	
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	
  if(HAL_ADC_Start_DMA(&hadc1,  &ConvertedValue, 1)!=HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		//HAL_Delay(500);
		//alcohol = ((float)((float)ConvertedValue * VREF)/MAX_CONVERTED_VALUE); 
    //printf("alcohol=%d\r\n",alcohol);
		loop();
  }
  /* USER CODE END 3 */

}


void loop(void)
{
    uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int32_t i;
    float f_temp;

    un_brightness = 0;
    un_min = 0x3FFFF;
    un_max = 0;

    n_ir_buffer_length = 150; //buffer length of 150 stores 3 seconds of samples running at 50sps

    //read the first 150 samples, and determine the signal range
    for(i = 0; i < n_ir_buffer_length; i++)
    {
        while(KEY0 == 1); //wait until the interrupt pin asserts
        maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i)); //read from MAX30102 FIFO

        if(un_min > aun_red_buffer[i])
            un_min = aun_red_buffer[i]; //update signal min
        if(un_max < aun_red_buffer[i])
            un_max = aun_red_buffer[i]; //update signal max
    }
    un_prev_data = aun_red_buffer[i];
    //calculate heart rate and SpO2 after first 150 samples (first 3 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
			
			if(HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 7) != HAL_OK)
			{
				
			}
					
        i = 0;
        un_min = 0x3FFFF;
        un_max = 0;

        //dumping the first 50 sets of samples in the memory and shift the last 100 sets of samples to the top
        for(i = 50; i < 150; i++)
        {
            aun_red_buffer[i - 50] = aun_red_buffer[i];
            aun_ir_buffer[i - 50] = aun_ir_buffer[i];

            //update the signal min and max
            if(un_min > aun_red_buffer[i])
                un_min = aun_red_buffer[i];
            if(un_max < aun_red_buffer[i])
                un_max = aun_red_buffer[i];
        }

        //take 50 sets of samples before calculating the heart rate.
        for(i = 100; i < 150; i++)
        {
            un_prev_data = aun_red_buffer[i - 1];
            while(KEY0 == 1);
            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));

            //calculate the brightness of the LED
            if(aun_red_buffer[i] > un_prev_data)
            {
                f_temp = aun_red_buffer[i] - un_prev_data;
                f_temp /= (un_max - un_min);
                f_temp *= MAX_BRIGHTNESS;
                f_temp = un_brightness - f_temp;
                if(f_temp < 0)
                    un_brightness = 0;
                else
                    un_brightness = (int)f_temp;
            }
            else
            {
                f_temp = un_prev_data - aun_red_buffer[i];
                f_temp /= (un_max - un_min);
                f_temp *= MAX_BRIGHTNESS;
                un_brightness += (int)f_temp;
                if(un_brightness > MAX_BRIGHTNESS)
                    un_brightness = MAX_BRIGHTNESS;
            }
			//Send_To_PC2( aun_red_buffer[i], aun_ir_buffer[i] );
            //send samples and calculation result to terminal program through UART
            /*SerialUSB.print(F("red="));
            SerialUSB.print(aun_red_buffer[i], DEC);
            SerialUSB.print(F(", ir="));
            SerialUSB.print(aun_ir_buffer[i], DEC);

            SerialUSB.print(F(", HR="));
            SerialUSB.print(n_heart_rate, DEC);

            SerialUSB.print(F(", HRvalid="));
            SerialUSB.print(ch_hr_valid, DEC);

            SerialUSB.print(F(", SPO2="));
            SerialUSB.print(n_spo2, DEC);

            SerialUSB.print(F(", SPO2Valid="));
            SerialUSB.println(ch_spo2_valid, DEC);*/

            //      SerialUSB.println(aun_ir_buffer[i], DEC);
        }
		//USART1_Receive_Check();
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

        if ((ch_hr_valid == 1) && (n_heart_rate < 190) && (n_heart_rate > 40))
        {
            hrTimeout = 0;

            // Throw out up to 1 out of every 5 valid samples if wacky
            if (hrValidCnt == 4)
            {
                hrThrowOutSamp = 1;
                hrValidCnt = 0;
                for (i = 12; i < 16; i++)
                {
                    if (n_heart_rate < hr_buf[i] + 10)
                    {
                        hrThrowOutSamp = 0;
                        hrValidCnt   = 4;
                    }
                }
            }
            else
            {
                hrValidCnt = hrValidCnt + 1;
            }

            if (hrThrowOutSamp == 0)
            {

                // Shift New Sample into buffer
                for(i = 0; i < 15; i++)
                {
                    hr_buf[i] = hr_buf[i + 1];
                }
                hr_buf[15] = n_heart_rate;

                // Update buffer fill value
                if (hrBuffFilled < 16)
                {
                    hrBuffFilled = hrBuffFilled + 1;
                }

                // Take moving average
                hrSum = 0;
                if (hrBuffFilled < 2)
                {
                    hrAvg = 0;
                }
                else if (hrBuffFilled < 4)
                {
                    for(i = 14; i < 16; i++)
                    {
                        hrSum = hrSum + hr_buf[i];
                    }
                    hrAvg = hrSum >> 1;
                }
                else if (hrBuffFilled < 8)
                {
                    for(i = 12; i < 16; i++)
                    {
                        hrSum = hrSum + hr_buf[i];
                    }
                    hrAvg = hrSum >> 2;
                }
                else if (hrBuffFilled < 16)
                {
                    for(i = 8; i < 16; i++)
                    {
                        hrSum = hrSum + hr_buf[i];
                    }
                    hrAvg = hrSum >> 3;
                }
                else
                {
                    for(i = 0; i < 16; i++)
                    {
                        hrSum = hrSum + hr_buf[i];
                    }
                    hrAvg = hrSum >> 4;
                }
            }
            hrThrowOutSamp = 0;
        }
        else
        {
            hrValidCnt = 0;
            if (hrTimeout == 4)
            {
                hrAvg = 0;
                hrBuffFilled = 0;
            }
            else
            {
                hrTimeout++;
            }
        }

        if ((ch_spo2_valid == 1) && (n_spo2 > 59))
        {
            spo2Timeout = 0;

            // Throw out up to 1 out of every 5 valid samples if wacky
            if (spo2ValidCnt == 4)
            {
                spo2ThrowOutSamp = 1;
                spo2ValidCnt = 0;
                for (i = 12; i < 16; i++)
                {
                    if (n_spo2 > spo2_buf[i] - 10)
                    {
                        spo2ThrowOutSamp = 0;
                        spo2ValidCnt   = 4;
                    }
                }
            }
            else
            {
                spo2ValidCnt = spo2ValidCnt + 1;
            }

            if (spo2ThrowOutSamp == 0)
            {

                // Shift New Sample into buffer
                for(i = 0; i < 15; i++)
                {
                    spo2_buf[i] = spo2_buf[i + 1];
                }
                spo2_buf[15] = n_spo2;

                // Update buffer fill value
                if (spo2BuffFilled < 16)
                {
                    spo2BuffFilled = spo2BuffFilled + 1;
                }

                // Take moving average
                spo2Sum = 0;
                if (spo2BuffFilled < 2)
                {
                    spo2Avg = 0;
                }
                else if (spo2BuffFilled < 4)
                {
                    for(i = 14; i < 16; i++)
                    {
                        spo2Sum = spo2Sum + spo2_buf[i];
                    }
                    spo2Avg = spo2Sum/2.f;
                }
                else if (spo2BuffFilled < 8)
                {
                    for(i = 12; i < 16; i++)
                    {
                        spo2Sum = spo2Sum + spo2_buf[i];
                    }
                    spo2Avg = spo2Sum/4.f;
                }
                else if (spo2BuffFilled < 16)
                {
                    for(i = 8; i < 16; i++)
                    {
                        spo2Sum = spo2Sum + spo2_buf[i];
                    }
                    spo2Avg = spo2Sum /8.f;
                }
                else
                {
                    for(i = 0; i < 16; i++)
                    {
                        spo2Sum = spo2Sum + spo2_buf[i];
                    }
                    spo2Avg = spo2Sum/16.f;
                }
            }
            spo2ThrowOutSamp = 0;
        }
        else
        {
            spo2ValidCnt = 0;
            if (spo2Timeout == 4)
            {
                spo2Avg = 0;
                spo2BuffFilled = 0;
            }
            else
            {
                spo2Timeout++;
            }
        }

        //Send_To_PC(hrAvg, spo2Avg);
		
	//	Send_To_Robot(hrAvg, spo2Avg);
					printf("HR:%d\r\n",hrAvg);
					printf("SP:%f\r\n",spo2Avg);
				//}
				alcohol = ((float)((float)ConvertedValue * VREF)/MAX_CONVERTED_VALUE); 
				printf("AL:%d\r\n",alcohol);
				

				if(strcmp(aRxBuffer,"ENABLEI")==0){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
					
				}
				if(strcmp(aRxBuffer,"DISABLE")==0){
					 
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
				}
    }
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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
    Error_Handler();
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
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
