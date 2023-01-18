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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h" // why? - printf??
#include "BME_defines.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#warning dont forget to copypast defines from BME_defines.h
#define BME_latch()   HAL_GPIO_WritePin(BME_LATCH_GPIO_Port, BME_LATCH_Pin, GPIO_PIN_SET);\
  	HAL_Delay(1);\
  	HAL_GPIO_WritePin(BME_LATCH_GPIO_Port, BME_LATCH_Pin, GPIO_PIN_RESET)
#warning why do we need that function above?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char stringBuffer[128];
int32_t temper_int;// intercalculation vars

BME280_CalibData CalibData;
float tf = 0.0f;/*Переменная, куда записываем значение температуры*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){

	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
uint8_t buf;
uint8_t BME_regD;
void BME_ShiftRegs(){
			HAL_GPIO_WritePin(BME_nOE_GPIO_Port, BME_nOE_Pin, 1); //shiftreg 2 HiZ
			uint8_t tbuf;
			HAL_GPIO_WritePin(BME_LATCH_GPIO_Port, BME_LATCH_Pin, 0);  //отключаем защёлку

			HAL_SPI_TransmitReceive(&hspi2, &BME_regD, &tbuf, 1, 5000);  //отправляем данные для сдвигового регистра со светодиодами

			HAL_GPIO_WritePin(BME_LATCH_GPIO_Port, BME_LATCH_Pin, 1);  //включаем защёлку
			HAL_GPIO_WritePin(BME_nOE_GPIO_Port, BME_nOE_Pin, 0);  //shitfreg вывести из HiZ
}

void BME_cs(uint8_t flag){
			/*
			 * Данная функция нужна для управления ножкой CS BME280
			 * Если на вход подаётся 0, то на ножке появляется ноль, если другое число, то единица
			 */
		if(flag==0)
			{
			BME_regD &=~(1<<2); // BME CS is OUT2 on shiftreg
			}
		else
			{
			BME_regD |= 1<<2;
			}
		BME_ShiftRegs();
}

/* About spi write in BME:
 * In SPI mode, only 7 bits of the register addresses are used; the MSB of register address is not used and replaced by a read/write bit (RW = '0' for write and RW = '1' for read). Example: address OxF7 is accessed by using SPI register address 0x77. For write access, the byte 0x77 is transferred, for read access, the byte OxF7 is transferred.
 * Writing is done by lowering CSB and sending pairs control bytes and register data. The control bytes consist of the SPI register address (= full register address without bit 7) and the write command (bit7 = RW = '0'). Several pairs can be written without raising CSB. The transaction is ended by a raising CSB. The SPI write protocol is depicted in Figure 12.
 */

void BME_write_reg(uint8_t reg, uint8_t val){
#warning this is nrf copypasta, not sure if it will work
#warning check if uint16_t is needed
	BME_cs(0);

	reg &= BME_WRITE_MASK; // our write mask is ~0x80, e.g. we send 0x77 to access reg 0xF7
	HAL_SPI_TransmitReceive(&hspi2, &reg, &buf, sizeof(reg), 10000);
	HAL_SPI_TransmitReceive(&hspi2, &val, &buf, sizeof(val), 10000);

	BME_cs(1);
}
/* About spi read in BME:
 * Reading is done by lowering CSB and first sending one control byte. The control bytes consist of the SPI register address (= full register address without bit 7) and the read command (bit 7 = RW = '1'). After writing the control byte, data is sent out of the SDO pin (SDI in 3-wire mode); the register address is automatically incremented. The SPI read protocol is depicted in Figure 13.
 * No mask is required for read, we send the reg through SPI
 */
uint8_t BME_read_Data8(uint8_t reg){
#warning this is nrf copypasta, not sure if it will work
	uint8_t rx_data;
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 1, 10000);

	BME_cs(1);

	return rx_data;
}

void BME_read_DataU16(uint8_t reg, uint16_t *rx_data){ // unsigned
#warning this is nrf copypasta, not sure if it will work
	//uint16_t rx_data;
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 2, 10000);

	BME_cs(1);

	return rx_data;
}

void BME_read_DataS16(uint8_t reg, int16_t *rx_data){ // signed
#warning this is nrf copypasta, not sure if it will work
	//uint16_t rx_data;
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 2, 10000);

	BME_cs(1);
}


void BME_read_DataU24(uint8_t reg, uint32_t *rx_data){
#warning this is nrf copypasta, not sure if it will work
	//uint16_t rx_data; - thrash
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 3, 10000);
#warning i am sure that it won't work that way when we are trying to put 3 bytes into the int32, because hal spi recieve makes a uin8t from that
	// maybe i should try HAL_SPI_TransmitReceive(&hspi2, &reg, rx_data, 3, 10000);??

	*(uint32_t *) rx_data &= 0x00FFFFFF; // wtf??? - we clear the first byte.

	BME_cs(1);
}

void BME_read_DataU24_BE(uint8_t reg, uint32_t *rx_data){ // basicly irreversed, cause bme280 should send us reversed data
#warning this is nrf copypasta, not sure if it will work
	//uint16_t rx_data; - thrash
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 3, 10000);

	*(uint32_t *) rx_data = be24toword(*(uint32_t *) rx_data) & 0x00FFFFFF; // wtf??? - we clear the first byte.

	BME_cs(1);
}

void BME_read_DataS24(uint8_t reg, int32_t *rx_data){
#warning this is nrf copypasta, not sure if it will work
	//uint16_t rx_data; - thrash
	BME_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 3, 10000);

	BME_cs(1);
}

void BMEReadCalibData(BME280_CalibData *data){
	BME_read_DataU16(BME280_REGISTER_DIG_T1, &(data->dig_T1));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_T2, &(data->dig_T2));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_T3, &(data->dig_T3));HAL_Delay(1);
	BME_read_DataU16(BME280_REGISTER_DIG_P1, &(data->dig_P1));HAL_Delay(1); // let's not be hasty
	BME_read_DataS16(BME280_REGISTER_DIG_P2, &(data->dig_P2));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P3, &(data->dig_P3));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P4, &(data->dig_P4));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P5, &(data->dig_P5));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P6, &(data->dig_P6));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P7, &(data->dig_P7));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P8, &(data->dig_P8));HAL_Delay(1);
	BME_read_DataS16(BME280_REGISTER_DIG_P9, &(data->dig_P9));HAL_Delay(1);
	// > &(data->dig_P9) - certified c99 moment, not sure if correct, but gotta be?
}
void BME280_SetMode(uint8_t mode) { // so basicly mode is 2 bits in the MEAS reg, we set them here

  uint8_t reg = BME_read_Data8(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;

  HAL_Delay(10); // debug

  reg |= mode & BME280_MODE_MSK;

  BME_write_reg(BME280_REG_CTRL_MEAS, reg);

}

float BME280_ReadTemperature() // we actually need to do that because we need it to calc the pressure

{

  float temper_float = 0.0f;
  uint32_t temper_raw;
  int32_t val1, val2;// intercalculation vars
#warning code trips on the next line BME_read_DataU24_BE
  BME_read_DataU24_BE(BME280_REGISTER_TEMPDATA,&temper_raw);
  temper_raw >>= 4;

  val1 = ((((temper_raw>>3) - ((int32_t)CalibData.dig_T1 <<1))) * // no idea what is going on
  ((int32_t)CalibData.dig_T2)) >> 11;
  val2 = (((((temper_raw>>4) - ((int32_t)CalibData.dig_T1)) *
  ((temper_raw>>4) - ((int32_t)CalibData.dig_T1))) >> 12) *
  ((int32_t)CalibData.dig_T3)) >> 14;
  temper_int = val1 + val2;

  temper_float = ((temper_int * 5 + 128) >> 8);
  temper_float /= 100.0f;

  return temper_float;

}

float BME280_ReadPressure()

{
  float press_float = 0.0f;
  float temper_float = 0.0f;
  uint32_t press_raw, pres_int;
  int64_t val1, val2, p;

#warning code trips on read temp

  temper_float = BME280_ReadTemperature();
  BME_read_DataU24_BE(BME280_REGISTER_PRESSUREDATA,&press_raw);
  press_raw >>= 4;


  // i have no idea how does this work, if it does?
  val1 = ((int64_t) temper_int) - 128000;
  val2 = val1 * val1 * (int64_t)CalibData.dig_P6;
  val2 = val2 + ((val1 * (int64_t)CalibData.dig_P5) << 17);
  val2 = val2 + ((int64_t)CalibData.dig_P4 << 35);
  val1 = ((val1 * val1 * (int64_t)CalibData.dig_P3) >> 8) + ((val1 * (int64_t)CalibData.dig_P2) << 12);
  val1 = (((((int64_t)1) << 47) + val1)) * ((int64_t)CalibData.dig_P1) >> 33;
  if (val1 == 0) return 0; // avoid exception caused by division by zero
  p = 1048576 - press_raw;
  p = (((p << 31) - val2) * 3125) / val1;
  val1 = (((int64_t)CalibData.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  val2 = (((int64_t)CalibData.dig_P8) * p) >> 19;
  p = ((p + val1 + val2) >> 8) + ((int64_t)CalibData.dig_P7 << 4);
  pres_int = ((p >> 8) * 1000) + (((p & 0xff) * 390625) / 100000);
  press_float = pres_int / 100.0f;

  return press_float;

}

void BME280_Init(){
	/*
	 * first, we soft-reset.
	 * The self test uses a total wait time of 9 milliseconds. Of this, 2 milliseconds are used as wait time for soft reset and 7 milliseconds are used as wait time for conversion. The soft reset is performed in order to erase any possible old settings and could be omitted if the sensor is known to be in an untouched state after power on.
	 */
	BME_write_reg(BME280_REG_SOFTRESET,BME280_SOFTRESET_VALUE);
	HAL_Delay(10); // let's wait a little
	BMEReadCalibData(&CalibData);
#warning maybe i should comment the BMEReadCalibData???
#warning im not sure, but by default standby timing of the BME should be 0.5 ms.
#warning i should set filtration and oversampling, but **do i need to**?
	BME280_SetMode(BME280_MODE_NORMAL);

}

float workWithBME280(){ // - have no idea what to put in there, so ok.

	/*
	 * I love you, yes I do, I love you
	 * If you break my heart I'll die
	 */

	return 0;
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  //printf("innit\r\n");
  BME280_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//printf("hello\n"); // if I have set the frequency on 8 MHZ the uart doesn' work
	//float result = BME280_ReadPressure();
	float result = BME280_ReadPressure();
	HAL_Delay(2000);
	printf("%f\r\n", result);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nOE_Pin|LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nOE_Pin LATCH_Pin */
  GPIO_InitStruct.Pin = nOE_Pin|LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
