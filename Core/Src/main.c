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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdio.h"
#include "pid.h"


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
static uint16_t global_gpio_pin = 0;
static uint16_t clear = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay (uint32_t us)   //Mikrosekunde
{
    __HAL_TIM_SET_COUNTER(&htim6,0);
    while ((__HAL_TIM_GET_COUNTER(&htim6))<us);
}



uint8_t Temp_byte1, Temp_byte2;
uint16_t SUM, TEMP;
float Temperature = 0;
uint8_t Presence = 0;

int set = 0;
PID_TypeDef TPID;
double Temp, PIDOut, TempSetpoint = 1;
uint8_t P, I, D;
uint8_t Menu = 2;
uint8_t value = 0;


/*************************** LCD ***************************/
#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd(char cmd)
{
    char upper_bits, lower_bits;
    uint8_t data[4];

    upper_bits = cmd & 0xF0;         // Get the upper 4 bits
    lower_bits = (cmd << 4) & 0xF0; // Shift the lower 4 bits to the upper position

    data[0] = upper_bits | 0x0C;    // Set EN=1 and RS=0
    data[1] = upper_bits | 0x08;    // Set EN=0 and RS=0
    data[2] = lower_bits | 0x0C;    // Set EN=1 and RS=0
    data[3] = lower_bits | 0x08;    // Set EN=0 and RS=0

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, data, 4, 100); // Send data via I2C
}


void lcd_send_data(char data)
{
    char upper_bits, lower_bits;
    uint8_t values[4];

    upper_bits = data & 0xF0;         // Get the upper 4 bits
    lower_bits = (data << 4) & 0xF0; // Shift the lower 4 bits to the upper position

    values[0] = upper_bits | 0x0D;   // Set EN=1 and RS=1
    values[1] = upper_bits | 0x09;   // Set EN=0 and RS=1
    values[2] = lower_bits | 0x0D;   // Set EN=1 and RS=1
    values[3] = lower_bits | 0x09;   // Set EN=0 and RS=1

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, values, 4, 100); // Send data via I2C
}


void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void Display_Temp(float Temp){
	char str[20] = {0};
	lcd_put_cur(0,0);

	sprintf(str, "TEMP: %.2f", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}
/*************************** DS18B20 ***************************/

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
    delay (80);    // delay according to datasheet
    if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
    else Response = -1;

    delay (400); // 480 us delay totally.

    return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
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
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_1);

  lcd_init();
  lcd_send_string("INITIALISING");
  HAL_Delay(2000);
  lcd_clear();

  P = 20;
  I = 20;
  D = 0;

  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);



  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&TPID, 500);
  PID_SetOutputLimits(&TPID, 1, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(Menu%2 != 0){
		  clear = 1;

		  if(set < 0){set = 0;}
		  if(set > 4){set = 4;}

		  char str[20] = {0};
		  lcd_put_cur(0,0);

		  switch(set){

		  case 0:
			  TempSetpoint = 1;
			  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);
			  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
			  PID_SetSampleTime(&TPID, 200);
			  PID_SetOutputLimits(&TPID, 1, 1000);

			  sprintf(str, "Temp not set    ");
			  lcd_send_string(str);
			  lcd_put_cur(1,0);
			  sprintf(str, "                ");
			  lcd_send_string(str);
			  break;
		  case 1:
			  TempSetpoint = 40;
			  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);
			  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
			  PID_SetSampleTime(&TPID, 200);
			  PID_SetOutputLimits(&TPID, 1, 1000);

			  sprintf(str, "Set temp PLA    ");
			  lcd_send_string(str);
			  lcd_put_cur(1,0);
			  sprintf(str, "Heating to 40C  ");
			  lcd_send_string(str);
			  break;
		  case 2:
			  TempSetpoint = 48;
			  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);
			  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
			  PID_SetSampleTime(&TPID, 200);
			  PID_SetOutputLimits(&TPID, 1, 1000);

			  sprintf(str, "Set temp PETG   ");
			  lcd_send_string(str);
			  lcd_put_cur(1,0);
			  sprintf(str, "Heating to 48C  ");
			  lcd_send_string(str);
			  break;
		  case 3:
			  TempSetpoint = 55;
			  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);
			  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
			  PID_SetSampleTime(&TPID, 200);
			  PID_SetOutputLimits(&TPID, 1, 1000);

			  sprintf(str, "Set temp ASA    ");
			  lcd_send_string(str);
			  lcd_put_cur(1,0);
			  sprintf(str, "Heating to 55C  ");
			  lcd_send_string(str);
			  break;
		  case 4:
			  TempSetpoint = 50;
			  PID(&TPID, &Temp, &PIDOut, &TempSetpoint, P, I, D, _PID_P_ON_E, _PID_CD_DIRECT);
			  PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
			  PID_SetSampleTime(&TPID, 200);
			  PID_SetOutputLimits(&TPID, 1, 1000);

			  sprintf(str, "Set temp TPU    ");
			  lcd_send_string(str);
			  lcd_put_cur(1,0);
			  sprintf(str, "Heating to 50C  ");
			  lcd_send_string(str);
			  break;

		  }


	  }else{

		  if(clear == 1){
			  char str[20] = {0};
			  lcd_put_cur(0,0);
			  sprintf(str, "                ");
			  lcd_send_string(str);
			  clear = 0;
		  }

	     Presence = DS18B20_Start ();
	     DS18B20_Write (0xCC);  // skip ROM
	     DS18B20_Write (0x44);  // convert t

	     Presence = DS18B20_Start ();
	     DS18B20_Write (0xCC);  // skip ROM
	     DS18B20_Write (0xBE);  // Read Scratch-pad

	     Temp_byte1 = DS18B20_Read();
	     Temp_byte2 = DS18B20_Read();
	     TEMP = ((Temp_byte2<<8))|Temp_byte1;
	     Temperature = (float)TEMP/16.0;  // resolution is 0.0625




	     Temp = Temperature;
	     PID_Compute(&TPID);
	     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PIDOut);
	     Display_Temp(Temperature);
	     HAL_Delay(300);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Callback funkcija koja se poziva kada se dogodi EXTI (vanjski prekid) na nekom GPIO pinu
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    global_gpio_pin = GPIO_Pin;  // Pohranjujemo broj pina koji je izazvao prekid

    if(global_gpio_pin == GPIO_PIN_4) {  // Ako je prekid generiran na pinu 4
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
    else if(global_gpio_pin == GPIO_PIN_10) {  // Ako je prekid generiran na pinu 10
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
    else if(global_gpio_pin == GPIO_PIN_9) {  // Ako je prekid generiran na pinu 9
        __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);  // Resetiramo zastavicu prekida tajmera 10
        HAL_TIM_Base_Start_IT(&htim10);  // Pokrećemo tajmer 10 u režimu prekida
    }
}

// Callback funkcija koja se poziva kada tajmer generira prekid (kada istekne zadano vrijeme)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {  // Provjeravamo je li prekid generirao tajmer 10
        if (HAL_GPIO_ReadPin(GPIOB, global_gpio_pin) == GPIO_PIN_RESET) {  // Provjera je li pritisnut gumb (logička nula)
            if (global_gpio_pin == GPIO_PIN_4) {  // Ako je gumb na pinu 4
                Menu++;  // Povećavamo varijablu Menu
            }

            if (Menu % 2 != 0) {  // Ako je Menu neparan broj (uključivanje neke opcije)
                if (global_gpio_pin == GPIO_PIN_10) {
                    set++;  // Povećavamo vrijednost varijable set
                }
                else if (global_gpio_pin == GPIO_PIN_9) {
                    set--;  // Smanjujemo vrijednost varijable set
                }
            }
        }
        HAL_TIM_Base_Stop_IT(&htim10);  // Zaustavljamo tajmer nakon što je obrada prekida završena
    }
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
