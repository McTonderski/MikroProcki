/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2_i2c.h"
#include "ADXL345.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define GPIORealyPIN GPIO_PIN_5
#define GPIO_Relay_side GPIOB
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADXL345_SPI_I2C adxl;
uint8_t data_rec[6];
int16_t x,y,z;
float xg,yg,zg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write(uint8_t address, uint8_t value)
{
  uint8_t data[2];
  data[0] = address|0x40;
  data[1] = value;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, data, 2, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void adxl_read(uint8_t address)
{
  // address |= 0x80;
  // address |= 0x40;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &address, 1, 10);
  HAL_SPI_Receive(&hspi1, data_rec, 6, 10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
void adxl_init(void)
{
  adxl_write(0x2d, 0x00); 
  adxl_write(0x2d, 0x08);
  adxl_write(0x31, 0x01);
}
void quarterNote(void)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    HAL_Delay(300);
  }

  void eighthNote(void)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
    HAL_Delay(150);
  }

  void refrain(void)
  {
    quarterNote();
    eighthNote();
    quarterNote();
    eighthNote();
    quarterNote();
  }

  void firstTact(void)
  {
    quarterNote();
    quarterNote();	
  }

  void secondTact(void)
  {
    eighthNote();
    eighthNote();
    quarterNote();	
  }

  void playRythm(void)
  {
    refrain();
    firstTact();
    refrain();
    secondTact();
  }

  void playDelay(uint16_t *delayed){
    while(1){
      int x,y,z;
      ADXL345readAccel_x_y_z(&adxl, &x, &y, &z); 
      if(z < -5)
      {
        break;
      }
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
      char measured_delay[6];
      sprintf(measured_delay, "%d \r\n", *delayed);
      HAL_UART_Transmit(&huart2, (uint16_t *)&measured_delay,sizeof(measured_delay), 0xFFFF);
      HAL_Delay(*delayed);
    }
  }

  uint8_t listen_for_rythm()
  {
    uint16_t timerTap = 0;
    uint8_t delayStep = 45;
    
    while(1)
    {
      
      int x,y,z;
      char gravity2Data[18];
      ADXL345readAccel_x_y_z(&adxl, &x, &y, &z); 
      sprintf(gravity2Data, "x=%d y=%d z=%d \r\n", x,y,z);
      HAL_UART_Transmit(&huart2, (uint8_t *)&gravity2Data,sizeof(gravity2Data), 0xFFFF);

      if(z < 0)
      {
        break;
      }
      if(timerTap > 0)
      {
        timerTap += delayStep;
      }
      if(y > 10 && timerTap == 0)
      {
        timerTap += 1;
        char TapDetection[] = "Tap Detected";
        HAL_UART_Transmit(&huart2, (uint8_t *)&TapDetection,sizeof(TapDetection), 0xFFFF);
      }
      else if(y > 10 && timerTap > 200)
      {
        timerTap += delayStep;
        char SecondTapDetection[] = "Second Tap Detected";
        char measured_delay[6];
        sprintf(measured_delay, "%d \r\n", timerTap);
        HAL_UART_Transmit(&huart2, (uint8_t *)&SecondTapDetection,sizeof(SecondTapDetection), 0xFFFF);
        HAL_UART_Transmit(&huart2, (uint8_t *)&measured_delay,sizeof(measured_delay), 0xFFFF);
        break;
      }
      
      HAL_Delay(delayStep);
    }

    if(timerTap > 300)
    {
      char measured_delay[7];
      sprintf(measured_delay, "%d \r\n ", timerTap);
      char Playing[] = "Playing";
      HAL_UART_Transmit(&huart2, (uint8_t *)&Playing,sizeof(Playing), 0xFFFF);
      HAL_UART_Transmit(&huart2, (uint8_t *)&measured_delay,sizeof(measured_delay), 0xFFFF);
      lcd16x2_i2c_printf(measured_delay);
      HAL_Delay(2000);
      playDelay(&timerTap);
    }
  }

  void scanI2C(void){
    printf("Scanning I2C bus:\r\n");
    char scanningMsg2Data[21] = "Scanning I2C bus: \r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)&scanningMsg2Data,sizeof(scanningMsg2Data), 0xFFFF);
    HAL_StatusTypeDef result;
    uint8_t i;
    for (i=1; i<128; i++)
    {
      result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
      if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
      {
        printf("."); // No ACK received at that address
      }
      if (result == HAL_OK)
      {
        printf("0x%X", i); 
        char port2Data[50];
        sprintf(port2Data, "0x%X\r\n", i);
        HAL_UART_Transmit(&huart2, (uint8_t *)&port2Data,sizeof(port2Data), 0xFFFF);// Received an ACK at that address

      }
    }
    printf("\r\n");
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();


  /* USER CODE BEGIN 2 */

  ADXL345_SPI(&adxl, &hspi1, GPIOB, GPIO_PIN_6); 
  ADXL345powerOn(&adxl);                              // Power on the ADXL345
	
  ADXL345setRangeSetting(&adxl, 16);                  // Give the range settings
	                                              // Accepted values are 2g, 4g, 8g or 16g
	                                              // Higher Values = Wider Measurement Range
	                                              // Lower Values = Greater Sensitivity

  ADXL345setSpiBit(&adxl, 0);                         // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
	                                              // Default: Set to 1
	                                              // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library

  ADXL345setActivityXYZ(&adxl, 1, 0, 0);              // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  ADXL345setActivityThreshold(&adxl, 75);             // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  ADXL345setInactivityXYZ(&adxl, 1, 0, 0);            // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  ADXL345setInactivityThreshold(&adxl, 75);           // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  ADXL345setTimeInactivity(&adxl, 10);                // How many seconds of no activity is inactive?

  ADXL345setTapDetectionOnXYZ(&adxl, 0, 1, 0);        // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  ADXL345setTapThreshold(&adxl, 100);                 // 62.5 mg per increment
  ADXL345setTapDuration(&adxl, 20);                  // 625 μs per increment
  ADXL345setDoubleTapLatency(&adxl, 200);             // 1.25 ms per increment
  ADXL345setDoubleTapWindow(&adxl, 400);             // 1.25 ms per increment

  // Set values for what is considered FREE FALL (0-255)
  ADXL345setFreeFallThreshold(&adxl, 7);             // (5 - 9) recommended - 62.5mg per increment
  ADXL345setFreeFallDuration(&adxl, 30);             // (20 - 70) recommended - 5ms per increment

  ADXL345InactivityINT(&adxl, 1);
  ADXL345ActivityINT(&adxl, 1);
  ADXL345FreeFallINT(&adxl, 1);
  ADXL345doubleTapINT(&adxl, 1);
  ADXL345singleTapINT(&adxl, 1);

  if(lcd16x2_i2c_init(&hi2c1))
  {
    char init2Data[] = "I2C initialized\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)&init2Data,sizeof(init2Data), 0xFFFF);
  }
  
  // lcd16x2_i2c_printf("hello world");
  char uart2Data[24] = "Connected to UART Two\r\n";
   /*
    * Output to uart2
    * use screen or putty or whatever terminal software
    * 8N1 115200
    */
  HAL_UART_Transmit(&huart2, (uint8_t *)&uart2Data,sizeof(uart2Data), 0xFFFF);

 	printf("\r\n");

 	printf("Scanning I2C bus:\r\n");
  char scanningMsg2Data[21] = "Scanning I2C bus: \r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)&scanningMsg2Data,sizeof(scanningMsg2Data), 0xFFFF);
	HAL_StatusTypeDef result;
 	// uint8_t i;
 	// for (i=1; i<128; i++)
 	// {
 	//   /*
 	//    * the HAL wants a left aligned i2c address
 	//    * &hi2c1 is the handle
 	//    * (uint16_t)(i<<1) is the i2c address left aligned
 	//    * retries 2
 	//    * timeout 2
 	//    */
 	//   result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
 	//   if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
 	//   {
 	// 	  printf("."); // No ACK received at that address
 	//   }
 	//   if (result == HAL_OK)
 	//   {
 	// 	  printf("0x%X", i); 
  //     char port2Data[50];
  //     sprintf(port2Data, "0x%X\r\n", i);
  //     HAL_UART_Transmit(&huart2, (uint8_t *)&port2Data,sizeof(port2Data), 0xFFFF);// Received an ACK at that address
 	//   }
 	// }
 	// printf("\r\n");
  

  bool devReady = false;
  bool tap = false;

  /* USER CODE END 2 */
  
    /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    int x,y,z;
    ADXL345readAccel_x_y_z(&adxl, &x, &y, &z); 
    if(z < 0)
    {
      playRythm();
    }
    else{
      listen_for_rythm();
    }

    char gravity2Data[18];
    sprintf(gravity2Data, "x=%d y=%d z=%d \r\n", x,y,z);
    HAL_UART_Transmit(&huart2, (uint8_t *)&gravity2Data,sizeof(gravity2Data), 0xFFFF);// Received an ACK at that address
    // HAL_Delay(350);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
