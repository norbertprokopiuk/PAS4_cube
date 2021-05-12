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
#include "bitmap.h"
#include <stdint.h>
#include "stm32f1xx.h"
#include "lcd.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_ADDRESS 0x3a // binarnie 00111010 odczytane z dokumentacji
#define ACC_CTRL1 0x20
#define ACC_X				0x28
#define ACC_Y				0x2a
#define ACC_Z				0x2c
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t pulse_count_1;
volatile uint16_t pulse_count_2;
char napis1[] = "pos:+000,+000"; //pozycja
char napis2[] = "rot:+000.0 deg"; //rotacja
char napis3[] = "vel:+00.0 mm/s"; //predkosc
char napis4[] = "w: +0.000 1/s"; //katowa
char napis5[] = "a+000.0A+000.0";
char napis6[] = "time: 000.0 s"; //czas
uint8_t i; // zmienna do zmiany trybu
uint32_t time = 0;
uint16_t sec = 0;
int16_t rot_e = 0; //zmienne do wyświetlanaia na ekran
int16_t ang_e = 0;
int16_t acc_e;
int16_t acc_a_e;
int16_t x_e;
uint8_t is_acc_ready = 0;
int16_t y_e;
int16_t dp1, dp2;
int16_t v_e;
uint16_t milsec = 0;
double velocity = 0;
double angular_velocity = 0;
double acceleration;
int32_t acc_acceleration; //przyspieszenie wypadkowe z akcelerometru
double x_n;
double y_n;
double rot;
double rot_d;
double delta_D;
double delta_rot;
double delta_v;
double last_rot;
double last_v;
double D, Dl, Dp, D_last;
float a_x_c, a_y_c;
float delta_a_x_c, delta_a_y_c;
float last_a_x_c, last_a_y_c;
int16_t last_state_1 = 0;
int16_t last_state_2 = 0;
uint32_t napiecie1;
int16_t a_x;
int16_t a_y;
double d_kola = 32; // w mm
double rozstaw = 149;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
uint8_t I2C_odczyt_rejestru(uint8_t rejestr);
int16_t I2C_odczyt_wartosci(uint8_t rejestr);
void I2C_zapis_rejestr(uint8_t rejestr, uint8_t value);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { //przerwanie timer 3
	if (htim->Instance == TIM3) { // Jeżeli przerwanie pochodzi od timera nr. 3
		time++; //zliczam czas z dokładnośćia do 0.1 s
		angular_velocity = delta_rot / 0.1;
		velocity = delta_D / 0.1;
		delta_v = velocity - last_v;
		acceleration = delta_v / 0.1;
		delta_rot = 0;
		delta_D = 0;
		last_v = velocity;
		//odczyt danych z acc
		a_x = I2C_odczyt_wartosci(ACC_X);
		a_y = I2C_odczyt_wartosci(ACC_Y);
		a_x_c = a_x * 2.0 / 32678.0;
		a_y_c = a_y * 2.0 / 32678.0;
		//z budowy akcelerometru wynika ze os y akcelerometru pokrywa się z osią x odometrii podobnie jest z osią x,y. Oś Z jest nieistotna
		delta_a_x_c = a_x_c - last_a_x_c;
		delta_a_y_c = a_y_c - last_a_y_c;
		acc_acceleration = (int) (sqrt(
				delta_a_x_c * delta_a_x_c + delta_a_y_c * delta_a_y_c) * 9810);
		if (delta_a_y_c < 0)
			acc_acceleration = acc_acceleration * -1;
		last_a_x_c = a_x_c;
		last_a_y_c = a_y_c;
	}

}
uint8_t I2C_odczyt_rejestru(uint8_t rejestr) //dla wygody aby podawać tylko rejestr który chce odczytac
{
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, ACC_ADDRESS, rejestr, 1, &value, sizeof(value),
	HAL_MAX_DELAY);
	return value;
}
int16_t I2C_odczyt_wartosci(uint8_t rejestr) {
	int16_t value = 0;

	HAL_I2C_Mem_Read(&hi2c1, ACC_ADDRESS, rejestr | 0x80, 1, (uint8_t*) &value,
			sizeof(value), HAL_MAX_DELAY);

	return value;
}

void I2C_zapis_rejestr(uint8_t rejestr, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, ACC_ADDRESS, rejestr, 1, &value, sizeof(value),
	HAL_MAX_DELAY);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SPI2_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
	__HAL_SPI_ENABLE(&hspi2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_ADC_Start(&hadc1);

	if (I2C_odczyt_rejestru(0xf) == 0x49)
		is_acc_ready = 1;
	else
		is_acc_ready = -1;
	if (is_acc_ready == 1)
		I2C_zapis_rejestr(ACC_CTRL1, 0x40 | 0x07); // predkosc 25Hz i aktywacja odczytów z osi x, y i z
	//domyslny punkt odniesienia akcelerometru to 2g oznacza to zakres pomiarowy (-2^15, 2^15)->(-2g,2g)

	lcd_setup();
	lcd_draw_text(0, 0, "PAS 4");
	lcd_draw_text(1, 0, "Projekt nr 1");
	lcd_draw_text(2, 0, "Obs. enkoderow");
	lcd_draw_text(3, 0, "Norbert");
	lcd_draw_text(4, 0, "Prokopiuk");
	lcd_draw_text(5, 0, "299024");
	lcd_copy();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
			i++;
			HAL_TIM_Base_Start_IT(&htim3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			if (i > 1)
				i = 0;
		}
		if (i >= 1) {
			pulse_count_1 = TIM1->CNT;
			pulse_count_2 = TIM2->CNT;
			napiecie1 = HAL_ADC_GetValue(&hadc1);
			if (napiecie1 <= 310)
				TIM3->CCR2 = 999;
			else
				TIM3->CCR2 = 0;

			if ((pulse_count_1 != last_state_1 || pulse_count_2 != last_state_2)) {

				dp1 = (pulse_count_1 - last_state_1) % 100;
				dp2 = (pulse_count_2 - last_state_2) % 100;

				Dl = Dl + (3.1415 * dp1) * d_kola / 1200;
				Dp = Dp + (3.1415 * dp2) * d_kola / 1200;

				D = (Dl + Dp) / 2;
				rot = (Dp - Dl) / rozstaw;
				delta_rot = rot - last_rot;
				if (rot >= 2 * 3.1415) {
					rot = rot - 2 * 3.1415;
				}
				rot_d = rot * 180 / 3.1415;
				delta_D = D - D_last;
				x_n = x_n + (D - D_last) * cos(rot);
				y_n = y_n + (D - D_last) * sin(rot);
				last_rot = rot;
			}

			//obliczanie czasu i innych zmiennych do aktualizacji napisów
			//pozycja
			x_e = (int) x_n;
			y_e = (int) y_n;

			//predkosc

			v_e = (int) velocity * 10;

			//czas
			milsec = time % 10;
			sec = (time - milsec) / 10;

			//rotacja
			rot_e = (int) 10 * rot_d;

			if (rot_e >= 1800)
				rot_e = rot_e - 3600;
			else if (rot_e <= -1800)
				rot_e = rot_e + 3600;

			if (rot_e >= 0)
				napis2[4] = '+';
			else
				napis2[4] = '-';

			rot_e = abs(rot_e);
			//predkosc katowa

			ang_e = (int) 1000 * angular_velocity;

			//aktualizacja opisow
			//pozycja
			if (x_e >= 0)
				napis1[4] = '+';
			else
				napis1[4] = '-';
			if (y_e >= 0)
				napis1[9] = '+';
			else
				napis1[9] = '-';
			x_e = abs(x_e);
			y_e = abs(y_e);
			//x
			napis1[5] = '0' + (x_e - x_e % 100) / 100;
			napis1[6] = '0' + (x_e % 100 - x_e % 10) / 10;
			napis1[7] = '0' + x_e % 10;
			//y
			napis1[10] = '0' + (y_e - y_e % 100) / 100;
			napis1[11] = '0' + (y_e % 100 - y_e % 10) / 10;
			napis1[12] = '0' + y_e % 10;

			//predkosc
			if (v_e >= 0)
				napis3[4] = '+';
			else
				napis3[4] = '-';
			v_e = abs(v_e);
			napis3[5] = '0' + (v_e - v_e % 100) / 100;
			napis3[6] = '0' + (v_e % 100 - v_e % 10) / 10;
			napis3[8] = '0' + v_e % 10;

			//rotacja
			napis2[5] = '0' + (rot_e - rot_e % 1000) / 1000;
			napis2[6] = '0' + (rot_e % 1000 - rot_e % 100) / 100;
			napis2[7] = '0' + (rot_e % 100 - rot_e % 10) / 10;
			napis2[9] = '0' + rot_e % 10;

			//predkosc katowa
			if (ang_e == 0) {
				napis4[3] = '+';
				napis4[4] = '0';
				napis4[6] = '0';
				napis4[7] = '0';
				napis4[8] = '0';
			} else {
				if (ang_e >= 0)
					napis4[3] = '+';
				else
					napis4[3] = '-';
				ang_e = abs(ang_e);
				napis4[6] = '0' + (ang_e - ang_e % 100) / 100;
				napis4[7] = '0' + (ang_e % 100 - ang_e % 10) / 10;
				napis4[8] = '0' + ang_e % 10;
			}
			//przyspieszenia
			if (acceleration >= 0)
				napis5[1] = '+';
			else
				napis5[1] = '-';
			if (acc_acceleration >= 0)
				napis5[8] = '+';
			else
				napis5[8] = '-';
			acc_e = abs((int) acceleration);

			//obliczone
			napis5[2] = '0' + (acc_e - acc_e % 1000) / 1000;
			napis5[3] = '0' + (acc_e % 1000 - acc_e % 100) / 100;
			napis5[4] = '0' + (acc_e % 100 - acc_e % 10) / 10;
			napis5[6] = '0' + acc_e % 10;
			acc_a_e = abs(acc_acceleration);
			napis5[9] = '0' + (acc_a_e - acc_a_e % 1000) / 1000;
			napis5[10] = '0' + (acc_a_e % 1000 - acc_a_e % 100) / 100;
			napis5[11] = '0' + (acc_a_e % 100 - acc_a_e % 10) / 10;
			napis5[13] = '0' + acc_a_e % 10;
			//czas
			napis6[8] = '0' + sec % 10;
			napis6[7] = '0' + (sec % 100 - sec % 10) / 10;
			napis6[6] = '0' + (sec - sec % 100) / 100;
			napis6[10] = '0' + milsec;

			lcd_clear();
			lcd_draw_text(0, 0, napis1);
			lcd_draw_text(1, 0, napis2);
			lcd_draw_text(2, 0, napis3);
			lcd_draw_text(3, 0, napis4);
			lcd_draw_text(4, 0, napis5);
			lcd_draw_text(5, 0, napis6);
			lcd_copy();
			last_state_1 = pulse_count_1;
			last_state_2 = pulse_count_2;
			D_last = D;

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

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
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1201;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1201;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 799;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC2 PC3 PC6 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
