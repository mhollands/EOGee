/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

#define base_clock 30000
#define sine_oversample 100
#define signal_mask 0x8000
#define ref_mask 0x4000
#define demod_mask 0x2000
#define gain1 66.87
#define gain2 22
#define gain3 1.5
#define dac_to_adc_counts (gain2*gain3)
#define double_buffered 1

// Store data for inphase and out of phase sine wave
int16_t inphase[sine_oversample] = {0,  128,  256,  383,  509,  632,  753,  871,  986, 1097, 1203, 1305,  1401, 1492, 1577, 1656, 1728, 1794, 1852, 1903, 1947, 1983, 2011, 2031,  2043, 2047, 2043, 2031, 2011, 1983, 1947, 1903, 1852, 1794, 1728, 1656,  1577, 1492, 1401, 1305, 1203, 1097,  986,  871,  753,  632,  509,  383,   256,  128,    0, -129, -257, -384, -510, -633, -754, -872, -987,-1098, -1204,-1306,-1402,-1493,-1578,-1657,-1729,-1795,-1853,-1904,-1948,-1984, -2012,-2032,-2044,-2048,-2044,-2032,-2012,-1984,-1948,-1904,-1853,-1795, -1729,-1657,-1578,-1493,-1402,-1306,-1204,-1098, -987, -872, -754, -633,  -510, -384, -257, -129};
int16_t quadphase[sine_oversample] = {2047, 2043, 2031, 2011, 1983, 1947, 1903, 1852, 1794, 1728, 1656, 1577,  1492, 1401, 1305, 1203, 1097,  986,  871,  753,  632,  509,  383,  256,   128,    0, -129, -257, -384, -510, -633, -754, -872, -987,-1098,-1204, -1306,-1402,-1493,-1578,-1657,-1729,-1795,-1853,-1904,-1948,-1984,-2012, -2032,-2044,-2048,-2044,-2032,-2012,-1984,-1948,-1904,-1853,-1795,-1729, -1657,-1578,-1493,-1402,-1306,-1204,-1098, -987, -872, -754, -633, -510,  -384, -257, -129,   -1,  128,  256,  383,  509,  632,  753,  871,  986,  1097, 1203, 1305, 1401, 1492, 1577, 1656, 1728, 1794, 1852, 1903, 1947,  1983, 2011, 2031, 2043};
// Pointer to sample of in-phase sine wave for Reference DAC
volatile uint16_t drive_ptr = 0;
// Flag to enable or disable drive signal
volatile uint8_t drive_enable = 1;

// Buffer to hold sense data
# define periods_per_demod 5
int16_t sense_buffer[sine_oversample*periods_per_demod];
// Pointer to current position in sense buffer
volatile uint16_t sense_ptr = 0;

// Buffer to hold signal data
# define signal_packet_size 64
# define signal_downsample_lenpower 5
uint32_t signal_downsample_accumulator = 0; // Accumulator used to downsample from base clock sample rate to reasonable sample rate
uint16_t signal_downsample_counter = 0; // Counter used to downsample from base clock sample rate to reasonable sample rate
uint32_t signal_averaging_accumulator = 0; // Accumulator used to average each packet so that we can target average value to our target
uint16_t signal_buffer[signal_packet_size*2]; // Double length for double buffering
volatile uint16_t signal_ptr = 0;
volatile uint8_t double_buffer_flag = 0;

// Buffers for SPI write
uint16_t dac_buffer;

// flag to indicate there is sense data available to demodulate
volatile uint8_t demod_ready_flag = 0;
// Each stage represents a spi transaction
volatile uint8_t comm_sequence_stage;
// Current reference voltage
volatile int16_t ref_voltage = 0;
#define ref_lower_threshold 500
#define ref_upper_threshold 3500
#define ref_target 2048
uint16_t ref_buffer[signal_packet_size*2]; // Double length for double buffering
// flag to indicate there is signal data available to demodulate
volatile uint8_t signal_packet_ready_flag = 0;

// Error flag for whenever something unexpected happens
volatile uint8_t error = 0;

// Flag whether we are in reference calibration mode
volatile uint8_t ref_calibration_mode = 0;
// Number of samples we should spend in each calibration step
#define ref_calibration_samples 100
volatile uint16_t ref_calibration_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void dispatch_drive_data(void);
void dispatch_ref_data(void);
void begin_sense_read(void);
void begin_signal_read(void);
uint16_t Package_DAC_Data(int16_t);
HAL_StatusTypeDef set_spi_cpol0_cpha1(void);
HAL_StatusTypeDef set_spi_cpol0_cpha0(void);
void fast_spi_transmit(uint16_t, uint8_t);
void fast_spi_initialise(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//  uint16_t data;
//  uint16_t level = 0;
//  HAL_StatusTypeDef r;

  fast_spi_initialise();
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  HAL_GPIO_WritePin(GPIOA, CS_REF_Pin, GPIO_PIN_RESET);
//  dac_buffer = Package_DAC_Data(level);
//  r = HAL_SPI_Transmit(&hspi1, (uint8_t *)&dac_buffer, 1, HAL_MAX_DELAY);
//  HAL_GPIO_WritePin(GPIOA, CS_REF_Pin, GPIO_PIN_SET);
//
////  HAL_GPIO_WritePin(GPIOA, CS_SIGNAL_Pin, GPIO_PIN_RESET);
////  r = HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
////  data = data >> 2;
////  HAL_GPIO_WritePin(GPIOA, CS_SIGNAL_Pin, GPIO_PIN_SET);
//  level+=1;
////  HAL_Delay(10);
//  continue;
//	 set_spi_cpol0_cpha0();
//	 r = HAL_SPI_Transmit(&hspi1, (uint8_t *)&dac_buffer, 1, HAL_MAX_DELAY);
//	 set_spi_cpol0_cpha1();
//	 r = HAL_SPI_Transmit(&hspi1, (uint8_t *)&dac_buffer, 1, HAL_MAX_DELAY);

	  if(demod_ready_flag)
	  {
//		  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
		  int32_t i_acculumator = 0; // Data should JUST fit inside a 32 bit integer for 500 samples at 12 bit
		  int32_t q_acculumator = 0; // Data should JUST fit inside a 32 bit integer for 500 samples at 12 bit
		  for(uint16_t i=0; i<sine_oversample*periods_per_demod; i++)
		  {
			  uint16_t idx = i % sine_oversample;
			  i_acculumator += sense_buffer[i]*inphase[idx];
			  q_acculumator += sense_buffer[i]*quadphase[idx];
		  }
		  int32_t i_accumulatorsquared32 = (i_acculumator >> 15)*(i_acculumator >> 15);
		  int32_t q_accumulatorsquared32 = (q_acculumator >> 15)*(q_acculumator >> 15);
		  int32_t accumulatorsquared32 = i_accumulatorsquared32 + q_accumulatorsquared32;
		  uint16_t accumulator12 = (accumulatorsquared32 >> 19) | demod_mask;
		  while(CDC_Transmit_FS((uint8_t *) &accumulator12, 2) == USBD_BUSY){};
//		  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);
		  demod_ready_flag = 0;
	  }

	  if(signal_packet_ready_flag != 0)
	  {
//		  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_TogglePin(DEBUG_GPIO_Port, DEBUG_Pin);
		  while(CDC_Transmit_FS((uint8_t *) &signal_buffer[(signal_packet_ready_flag-1)*signal_packet_size], signal_packet_size*2) == USBD_BUSY){};
		  while(CDC_Transmit_FS((uint8_t *) &ref_buffer[(signal_packet_ready_flag-1)*signal_packet_size], signal_packet_size*2) == USBD_BUSY){};
		  signal_packet_ready_flag = 0;
//		  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);
	  }

//	  CDC_Transmit_FS(m, 64);
//	  HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 400;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_DRIVE_Pin|CS_SENSE_Pin|CS_REF_Pin|CS_SIGNAL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_DRIVE_Pin CS_SENSE_Pin CS_REF_Pin CS_SIGNAL_Pin */
  GPIO_InitStruct.Pin = CS_DRIVE_Pin|CS_SENSE_Pin|CS_REF_Pin|CS_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Take nsigned 16-bit data and translate to correct format for DAC7311
// should be unisgned with 2-MSBs should be zero to indicate normal mode
// middle 12 bits should be data
// 2-LSBs are don't care, we send zeros
uint16_t Package_DAC_Data(int16_t data)
{
	uint16_t data_formatted = (uint16_t) (data + (1 << 11));
	data_formatted = ((data_formatted << 2) & 0x3FFC);
	return data_formatted;
}

// Send the next sample to the drive dac
void dispatch_drive_data()
{
	HAL_GPIO_WritePin(GPIOA, CS_DRIVE_Pin, GPIO_PIN_RESET);
	if(drive_enable)
	{
		dac_buffer = Package_DAC_Data(inphase[drive_ptr]);
	}
	else
	{
		dac_buffer = Package_DAC_Data(0);
	}
	fast_spi_transmit(dac_buffer, 0);
	if(++drive_ptr >= sine_oversample)
	{
		drive_ptr = 0;
	}
}

// Send the next sample to the ref dac
void dispatch_ref_data()
{
	HAL_GPIO_WritePin(GPIOA, CS_REF_Pin, GPIO_PIN_RESET);
	dac_buffer = Package_DAC_Data(ref_voltage);
	fast_spi_transmit(dac_buffer, 0);
}

// Begin spi transaction to load next sense ADC sample
void begin_sense_read()
{
	HAL_GPIO_WritePin(GPIOA, CS_SENSE_Pin, GPIO_PIN_RESET);
	fast_spi_transmit(0x0000, 0); // send dummy data
}

// Begin spi transaction to load next signal ADC sample
void begin_signal_read()
{
	HAL_GPIO_WritePin(GPIOA, CS_SIGNAL_Pin, GPIO_PIN_RESET);
	fast_spi_transmit(0x0000, 0); // send dummy data
}

// Write a single 16-bit value to the TXFIFO so it will get clocked out
// cpha not yet supported
void fast_spi_transmit(uint16_t data, uint8_t cpha)
{
//	HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
	// coming into this function we assume the SPI is not busy and also that the spi is disabled
	hspi1.Instance->DR = data;
}

// Callback is called when there is data received in RXFIFO.
// This occurs after both a TX and an RX because we are full duplex.
void fast_spi_rxcallback(uint16_t data)
{
	// Reset CS lines after transmission complete
	HAL_GPIO_WritePin(GPIOA, CS_REF_Pin|CS_SIGNAL_Pin|CS_SENSE_Pin|CS_DRIVE_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_RESET);

	switch(comm_sequence_stage)
	{
	case(0):	begin_sense_read(); break;
	case(1):
				sense_buffer[sense_ptr] = (int16_t)((data >> 2) - (1<<11)); // ADC 2 LSBs are always zeros
				if(++sense_ptr >= sine_oversample * periods_per_demod)
				{
					demod_ready_flag = 1;
					sense_ptr = 0;
				}
				dispatch_ref_data();
				break;
	case(2):	begin_signal_read(); break;
	case(3):	signal_downsample_accumulator += data >> 2; //add data to accumulator (ADC 2 LSBs are always zeros)
				if(++signal_downsample_counter == (1 << signal_downsample_lenpower))
				{
					uint16_t sample = (signal_downsample_accumulator >> signal_downsample_lenpower);
					ref_buffer[signal_ptr + double_buffer_flag*signal_packet_size] = ((uint16_t)(ref_voltage+2048)) | ref_mask;
					signal_buffer[signal_ptr + double_buffer_flag*signal_packet_size] = sample | signal_mask;
					signal_downsample_accumulator = 0;
					signal_downsample_counter = 0;
					signal_averaging_accumulator += sample;
					// If we are adding new data but we haven't finished transmitting the old data, throw error flag
					if(signal_packet_ready_flag == (double_buffer_flag + 1))
					{
						error = 1;
					}
					if(++signal_ptr >= signal_packet_size) // When we have enough data for a full packet
					{
						// Reset pointer to start of buffer
						signal_ptr = 0;
						// Mark that this buffer is ready for sending over USB
						signal_packet_ready_flag = double_buffer_flag + 1;
						if(double_buffered)
						{
							// Switch which buffer we are using
							double_buffer_flag = (double_buffer_flag == 0 ? 1 : 0);
						}

						// Adjust reference so packet average is the target
						uint32_t signal_packet_average = signal_averaging_accumulator / signal_packet_size;
						if(!ref_calibration_mode)
						{
							if(signal_packet_average > ref_upper_threshold)
							{
								if((ref_voltage -= (uint16_t) ((signal_packet_average - ref_target)/dac_to_adc_counts)) < -2048)
								{
									ref_voltage = -2048;
								}
							}
							if(signal_packet_average < ref_lower_threshold)
							{
								if((ref_voltage += (uint16_t) ((ref_target - signal_packet_average)/dac_to_adc_counts)) > 2047)
								{
									ref_voltage = 2047;
								}
							}
						}
						signal_averaging_accumulator = 0;
					}
					if(ref_calibration_mode)
					{
						if(++ref_calibration_count >= ref_calibration_samples)
						{
							if(++ref_voltage > 70)
							{
								ref_voltage = -70;
							}
							ref_calibration_count = 0;
						}
					}
				}
				break; //Handle signal read result
	default: break;
	}
	comm_sequence_stage++;
}

void fast_spi_initialise()
{
	// Enable only the RX interrupt as this will be thrown after 16 bits have been transmitted
	__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_RXNE));
	// Enable the SPI. Nothing should happend until we send the fast_spi_transmit function
	__HAL_SPI_ENABLE(&hspi1);
}

// Callback for when data is recieved via CDC, called from usbd_cdc_if.c
void receive_cdc_data(uint8_t* buf, uint16_t len)
{
	if(len == 1)
	{
		if(buf[0] == 'c')
		{
			ref_calibration_mode = !ref_calibration_mode;
		}
		if(buf[0] == 'm')
		{
			drive_enable = !drive_enable;
		}
		if(buf[0] == 'r')
		{
			NVIC_SystemReset();
		}
	}
}

// Timer output compare interrupt (main base clock for sampling)
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	dispatch_drive_data();
	comm_sequence_stage = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
