/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t XData[2];
uint8_t YData[2];
uint8_t ZData[2];

int16_t x,y,z;
uint8_t chipID = 0;
float xg, yg, zg;

uint8_t int_source = 0;

int count = 0 ;

// Device Registers for ADXL345 Accelerometer
uint8_t DEVICE			= 0x00 ;	// Device ID
uint8_t THRESH_TAP		= 0x1D ;	// Tap threshold
uint8_t OFSX			= 0x1D ;	// X-axis offset
uint8_t OFSY			= 0x1E ;	// Y-axis offset
uint8_t OFSZ			= 0x20 ;	// Z-axis offset
uint8_t DUR				= 0x21 ;	// Tap duration
uint8_t Latent			= 0x22 ;	// Tap latency
uint8_t Window			= 0x23 ;	// Tap window
uint8_t THRESH_ACT		= 0x24 ;	// Activity threshold
uint8_t THRESH_INACT	= 0x25 ; 	// Inactivity threshold
uint8_t TIME_INACT		= 0x26 ;	// Inactivity time
uint8_t ACT_INACT_CTL	= 0x27 ; 	// Axis enable control for activity inactivity detection
uint8_t THRESH_FF		= 0x28 ; 	// Free-fall threshold
uint8_t TIME_FF			= 0x29 ; 	// Free-fall time
uint8_t TAP_AXES		= 0x2A ; 	// Axis control for single tap/double tap
uint8_t ACT_TAP_STATUS	= 0x2B ; 	// Source of single tap/double tap
uint8_t BW_RATE			= 0x2C ; 	// Data rate and power mode control
uint8_t POWER_CTL		= 0x2D ; 	// Power-saving features control
uint8_t INT_ENABLE     	= 0x2E ; 	// Interrupt enable control
uint8_t INT_MAP         = 0x2F ; 	// Interrupt mapping control
uint8_t INT_SOURCE      = 0x30 ; 	// Source of interrupts
uint8_t DATA_FORMAT     = 0x31 ; 	// Data format control
uint8_t DATAX0          = 0x32 ; 	// X-axis data 0
uint8_t DATAX1          = 0x33 ; 	// X-axis data 1
uint8_t DATAY0          = 0x34 ; 	// Y-axis data 0
uint8_t DATAY1          = 0x35 ; 	// Y-axis data 1
uint8_t DATAZ0          = 0x36 ; 	// Z-axis data 0
uint8_t DATAZ1          = 0x37 ; 	// Z-axis data 1
uint8_t FIFO_CTL        = 0x38 ; 	// FIFO control
uint8_t FIFO_STATUS     = 0x39 ; 	// FIFO status

/**
 * To read Polling.
 * - Write to
 */

void adxl_write (uint8_t Reg, uint8_t data)
{
	uint8_t writeBuf[2];
	writeBuf[0] = Reg|0x40;  // multi-byte write enabled
	writeBuf[1] = data;
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // pull the cs pin low to enable the slave
	HAL_SPI_Transmit (&hspi1, writeBuf, 2, 100);  // transmit the address and data
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // pull the cs pin high to disable the slave
}

void adxl_read (uint8_t Reg, uint8_t *Buffer, size_t len)
{
	Reg |= 0x80;  // read operation
	Reg |= 0x40;  // multi-byte read
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // pull the CS pin low to enable the slave
	HAL_SPI_Transmit (&hspi1, &Reg, 1, 100);  // send the address from where you want to read data
	HAL_SPI_Receive (&hspi1, Buffer, len, 100);  // read 6 BYTES of data
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // pull the CS pin high to disable the slave
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_7)
    {
//        printf("Interrupt detected\n");

        uint8_t int_source;
        adxl_read(INT_SOURCE, &int_source, 1);

        if (int_source & (1 << 5))
        {
            printf("Double Tap detected.\n");
        }
        else if (int_source & (1 << 6))
        {
            printf("Single Tap detected.\n");
        }
        else if (int_source & (1 << 4))
        {
            printf("Activity detected.\n");
        }
        else if (int_source & (1 << 3))
        {
            printf("Inactivity detected.\n");
        }
    }
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	__disable_irq();
//
//    if(GPIO_Pin == GPIO_PIN_7)
//    {
//    	printf("INT1 : ");
//    	adxl_read (INT_SOURCE , &int_source, 1 );
//    	if(int_source & (1 << 5))
//    	{
//    		// IF 0[1]000000	Double Tap
//    		printf("Double Tap.\r\n");
//    	}
//    	else if(int_source & (1 << 6))
//    	{
//    		// IF 00[1]00000	Single Tap
//    		printf("Single Tap.\r\n");
//    	}
//    }
//    else if(GPIO_Pin == GPIO_PIN_9)
////	if(GPIO_Pin == GPIO_PIN_9)
//    {
//    	printf("INT2 : ");
//    	adxl_read (INT_SOURCE , &int_source, 1 );
//    	if(int_source & (1 << 4))
//    	{
//    		// IF 000[1]0000	Activity
//    		printf("Activity Detection.\r\n");
//    	}
//    	else if(int_source & (1 << 3))
//    	{
//    		// IF 0000[1]000	Inactivity
//    		printf("Inactivity Detection.\r\n");
//    	}
//    }
//    __enable_irq();
//}

void adxl_init (void)
{
	adxl_read(DEVICE, &chipID, 1);
	if (chipID == 0xE5)
	{
		adxl_write (POWER_CTL, 0x00);		// Standby mode for initialize. (Reset all Bits.)
		adxl_write (BW_RATE, 0x0D);			// Disable sleep mode and Output Data Rate 800Hz

		// Low Power from 12.5 Hz to 400 Hz.
		// 000[0][1100] = 0x0C = 400  Hz
		// 000[0][1011] = 0x0B = 200  Hz
		// 000[0][1010] = 0x0A = 100  Hz
		// 000[0][1001] = 0x09 = 50   Hz
		// 000[0][1000] = 0x08 = 25   Hz
		// 000[0][0111] = 0x07 = 12.5 Hz

	////////// DATA FORMAT //////////
		// 00[0]01011		Set the interrupt to active high
		// 0000[1]011		Set in full resolution mode
		// 00001[0]11		Set in the right-justified mode
		// 000010[11]		Set the g range in // ±16 g
		adxl_write (DATA_FORMAT, 0x0B);

	////////// OFFSET CALIBRATION //////////
		// The scale factor of offset is 15.6mg/LSB = 0.0156g/LSB
		// x_avg (100 samples) = -0.04989g
		// y_avg (100 samples) = -0.03662g
		// z_avg (100 samples) = 0.946931g
		adxl_write (OFSX, 0x03);			// Set offset x-axis 3 x 0.0156g = +0.0468g
		adxl_write (OFSY, 0x03);			// Set offset y-axis 3 x 0.0156g = +0.0468g
		adxl_write (OFSZ, 0x03);			// Set offset z-axis 3 x 0.0156g = +0.0468g

	////////// TAP DETECTION //////////
		// Threshold tap, the scale factor is 62.5mg/LSB = 0.0625g/LSB
		adxl_write (THRESH_TAP, 0x18);		// Set threshold 24 x 0.0625 = 1.5g
//		adxl_write (THRESH_TAP, 0xFF);		// Set threshold 256 x 0.0625 = 16g (Maximum)
		// Tap duration, the scale factor is 625us/LSB = 0.625ms/LSB
		adxl_write (DUR, 0x50);				// Set duration 80 x 0.625ms = 50ms
		// Tap latency, the scale factor is 1.25ms/LSB
		adxl_write (Latent, 0x20);			// Set latency 32 x 1.25ms = 40ms
		// Tap Window, the scale factor is 1.25ms/LSB
		adxl_write (Window, 0xF0);			// Set window 240 x 1.25ms = 300ms
		// Tap Axes control
		adxl_write (TAP_AXES, 0x01),		// Enable z-axis for detect tap function

	////////// ACTIVITY ANS INACTIVITY DETECTION //////////
		// Threshold activity, the scale factor is 62.5mg/LSB = 0.0625g/LSB
		adxl_write (THRESH_ACT, 0x03);		// set threshold activity 3 x 0.0625g = 0.1875g
//		adxl_write (THRESH_ACT, 0x05);		// set threshold activity 4 x 0.0625g = 0.25g
		// Threshold inactivity, The scale factor of is 62.5mg/LSB = 0.0625g/LSB
		adxl_write (THRESH_INACT, 0x02);	// set threshold inactivity 2 x 0.0625g = 0.125g
//		adxl_write (THRESH_INACT, 0x05);	// set threshold inactivity 4 x 0.0625g = 0.25g
		// Time inactivity, the scale factor is 1sec/LSB
		adxl_write (TIME_INACT, 0x05);		// set time inactivity 5 x 1sec = 5sec
		// Control activity detection axis
		// ACT_ACT_CTL 0x60: 0110 0000 DC-coupled and detected X and Y axis
		// ACT_INACT_CTL 0x06: 0000 0110 DC-coupled and detected X and Y axis
		adxl_write (ACT_INACT_CTL, 0x66);

	////////// INTERRUPTS //////////
		adxl_write (INT_ENABLE, 0x00);		// Clear interrupt functions
		adxl_write (INT_MAP, 0x78);			// Set Single-Double Tap INI1 and Activity&Inactivity INIT2
		adxl_write (INT_ENABLE, 0x78);		// Enable interrupt tap, activity and inactivity functions

		adxl_write (POWER_CTL, 0x28);		// Charge power mode to measure mode and enable link bit
		HAL_Delay(500);
	}
}
void adxl_read_data (void)
{
	adxl_read (DATAX0, XData, 2);
	adxl_read (DATAY0, YData, 2);
	adxl_read (DATAZ0, ZData, 2);

	x = ((XData[1] << 8) | XData[0]);
	y = ((YData[1] << 8) | YData[0]);
	z = ((ZData[1] << 8) | ZData[0]);

	// Convert into 'g'
	xg = (float)x*0.0039 ;
	yg = (float)y*0.0039 ;
	zg = (float)z*0.0039 ;

	HAL_Delay(100);
}

/***
 * Function to read the Bad Detection by changing the detect the angle of delta.
 *
 * The concept read the angle delta is by convert the coordinate from Cartesian to Spherical.
 */
void Detect_Bad_Tilt(float x_g, float y_g, float z_g)
{
	float r, delta_angle, phi_angle;
	float delta_deg, phi_deg;


	r = sqrt(pow(x_g,2) + pow(y_g,2) + pow(z_g,2));

	delta_angle = acos(z_g/(r));

	phi_angle = atan(y_g/x_g);

	delta_deg = delta_angle * (180/3.14);
	phi_deg = phi_angle * (180/3.14);

	printf("Delta = %.2f	|	Phi = %.2f\r\n", delta_deg, phi_deg);

	if (delta_deg >= 30.0 && delta_deg <= 90.0)
	{
		printf("Bad Rider Detected!\r\n");
	}
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  adxl_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  adxl_read_data();

//	  Detect_Bad_Tilt(x,y,z);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
