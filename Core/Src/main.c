/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "String.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int x1=0,x4=0;

int sevenseg_counter = 0;
int uart_counter = 0;
unsigned char counter[1];
char counter1[50];
char counter2[3];
int counter_flag = 0 , sw1 = 0 , isEmpty = 1 ;
int last_interrupt1=0;
char hello[7] = "hello \n";
int delay = 0;
int buzzer_flag = 0;

int max_floor = 9;
int current_floor = 0;
int selected_floor = 0;
int goto_floor = 0;
int last_floor = 0;

int items[SIZE], front = -1, rear = -1;

unsigned char data[20];

char admin_cmd[] = "ADMIN#";
char pass[] = "1234";
int admin_mode = 0;

char max_level_cmd[] = "SET MAX LEVEL";
char level_cmd[] = "SET LEVEL";
char wait_cmd[] = "SET WAIT";
char led_cmd[] = "SET LED";
char start_cmd[] = "START";
char test_cmd[]= "TEST#";

int led_condition = 0;

char message[50] = "";

int waiting_time = 500;

TIM_HandleTypeDef *pwm_timer = &htim8; // Point to PWM timer configured in CubeMX
uint32_t pwm_channel = TIM_CHANNEL_2;  // Specify configured PWM channel


void write_num_toBCD(int a) {
	if (a & 1) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	if (a & 2) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	if (a & 4) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	if (a & 8) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	}
}

void turn_off_all() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
			0);

}

void turn_on_led(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14 , 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15 , 1);
}

void turn_off_led(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14 , 0);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15 , 0);
}

/////////////////////////// queue ///////////////////////

void enQueue(int value) {
  if (rear == SIZE - 1)
    printf("\nQueue is Full!!");
  else {
    if (front == -1)
      front = 0;
    rear++;
    items[rear] = value;
    isEmpty = 0;
  }
}

void deQueue() {
  if (front == -1){
	isEmpty = 1;
  }
  else {
    goto_floor = items[front];
    front++;
    if (front > rear){
      front = rear = -1;
    }
  }
}

void peek() {
  if (front == -1){
	isEmpty = 1;
  }
  else {
    last_floor = items[rear];
  }
}

/////////////////////////////// buzzer /////////////////////////////////////////////////


void PWM_Start()
{
  HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
}

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
  if (pwm_freq == 0 || pwm_freq > 20000)
  {
    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
  }
  else
  {
    const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
	const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
    const uint32_t timer_clock = internal_clock_freq / prescaler;
    const uint32_t period_cycles = timer_clock / pwm_freq;
    const uint32_t pulse_width = volume * period_cycles / 1000 / 2;

    pwm_timer->Instance->PSC = prescaler - 1;
    pwm_timer->Instance->ARR = period_cycles - 1;
    pwm_timer->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
  }
}


////////////////////////////////////////////////
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

		if (GPIO_Pin == GPIO_PIN_4 && last_interrupt1 + 200 < HAL_GetTick()){
			last_interrupt1 = HAL_GetTick();

			if (selected_floor  < max_floor) {
				selected_floor++;
			}

		}
		else if (GPIO_Pin == GPIO_PIN_6 && last_interrupt1 + 200 < HAL_GetTick()){
			last_interrupt1 = HAL_GetTick();

			if (selected_floor  > 0) {
				selected_floor --;
			}

		}
		else if (GPIO_Pin == GPIO_PIN_12 && last_interrupt1 + 200 < HAL_GetTick()){
			last_interrupt1 = HAL_GetTick();
			peek();
			if(last_floor != selected_floor && selected_floor != current_floor){
			enQueue(selected_floor);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
			}

		}
		else if (GPIO_Pin == GPIO_PIN_1 && last_interrupt1 + 200 < HAL_GetTick()){
			last_interrupt1 = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			PWM_Start();
			sw1 = 1;
			if(led_condition){
				turn_on_led();
			}
			else{
				turn_off_led();
			}
		}

	}


////////////////////////////////////////////////////////////////


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
    	switch (delay%4) {
			case 0:
				turn_off_all();
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
				write_num_toBCD(selected_floor);
				break;
			case 1:
				turn_off_all();
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
				write_num_toBCD(0);
				break;
			case 2:
				turn_off_all();
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
				write_num_toBCD(0);
				break;
			case 3:
				turn_off_all();
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
				write_num_toBCD(current_floor);
				break;

		}
				delay++;



    }

    if (htim->Instance == TIM4)
    {

    	if (goto_floor == current_floor && !isEmpty && !admin_mode ){
    		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
    		HAL_Delay(waiting_time);
    		deQueue();
    		counter_flag = 1;
    	}

    	if(isEmpty){
    		counter_flag = 0;
    	}

    	if (counter_flag){
//    		deQueue();

    		if(goto_floor > current_floor ){
    			current_floor++;
    		}
    		else if(goto_floor < current_floor){
    			current_floor--;
    		}


    	}

    }

    if (htim->Instance == TIM2)
        {
    	if(sw1){
    		if(buzzer_flag % 2 == 0){
    			PWM_Change_Tone(300, 50);
    		}else{
    			PWM_Change_Tone(300, 0);
    		}
    		buzzer_flag++;
    	}
       }
}

/////////////////////////////////////////////////////////////////////////////////////////
void set_max_level(int max){
	if(max > 9){
		char incorrect_pass_msg[41] = "\nNumber of levels should be less than 10\n" ;
		HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
	}else{
		max_floor = max;
	}
	current_floor = 0;
}

void set_level(int level){
	if(level >= 0 && level <= max_floor){
		current_floor = level;
		goto_floor = level;
	} else{
		if(level < 0){
			char incorrect_pass_msg[44] = "\nNumber of current level should be positive\n" ;
			HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
		} else{
			char incorrect_pass_msg[68] = "\nNumber of current level should be less than total number of levels\n" ;
			HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
		}
	}
}

void set_led(char condition[]){
	char on[] = "ON";
	char off[] = "OFF";
	if(strstr(condition , on)){
		led_condition = 1;
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	}
	else if (strstr(condition , off)){
		led_condition = 0;
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
	}
	else{
		char incorrect_pass_msg[16] = "\nwrong command!\n" ;
		HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
	}
}

void set_wait(int wait){
	if (wait >= 500 && wait <= 5000 && (wait % 100 == 0)){
		waiting_time = wait;
	} else{
		if(wait % 100 != 0){
			char incorrect_pass_msg[41] = "\nWaiting time should be dividable by 100\n" ;
			HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
		} else if (wait < 500){
			char incorrect_pass_msg[38] = "\nWaiting time should be more than 500\n" ;
			HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
		} else if(wait > 5000){
			char incorrect_pass_msg[39] = "\nWaiting time should be less than 5000\n" ;
			HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
		}
	}
}

char y[5];
int r = 5 , i = 8 , j = 0;
int num = 0;
int numbers[4];
int count = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

		if(huart->Instance == USART2){
			if(strstr(data , admin_cmd) && isEmpty){
				  HAL_UART_Transmit(&huart2, hello, sizeof(hello) , 1000);
				if(data[6] == pass[0] && data[7] == pass[1] && data[8] == pass[2] && data[9] == pass[3]){
					  HAL_UART_Transmit(&huart2, hello, sizeof(hello) , 1000);
					  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
					  admin_mode = 1;
				}
				else{
					char incorrect_pass_msg[24] = "\nPassword is Incorrect!\n" ;
					HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
				}
			}

			if(admin_mode){
				if(strstr(data , max_level_cmd)){
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
					set_max_level(data[13]-48);
					char x = max_floor + 48;
					HAL_UART_Transmit(&huart2, &x, sizeof(x) , 1000);
				}
				else if(strstr(data , level_cmd)){
					set_level(data[9]-48);
					char x = current_floor + 48;
					HAL_UART_Transmit(&huart2, &x, sizeof(x) , 1000);
				}
				else if(strstr(data , wait_cmd)){
					while((data[i] != ' ') && i < 12 && j< 4){
						numbers[j] = (data[i]-48);
						HAL_UART_Transmit(&huart2, &data[i], 1 , 1000);
						i++;
						j++;
					}
					if(j == 3){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
						count = (numbers[0]*100) + (numbers[1]*10) + (numbers[2]);
					}else if(j==4){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
						count = (numbers[0]*1000) + (numbers[1]*100) + (numbers[2]*10) + numbers[3];
					}
					HAL_UART_Transmit(&huart2, &j, sizeof(j) , 1000);
					set_wait(count);
				}
				else if(strstr(data , led_cmd)){
					strncpy(y , data+7 , 3);
					HAL_UART_Transmit(&huart2, y, 3 , 1000);
					set_led(y);
				}
				else if(strstr(data , test_cmd)){
					while((data[r] != ' ') && r < 10){
						num = data[r]-48;
						enQueue(num);
						HAL_UART_Transmit(&huart2, &data[r], 1 , 1000);
						r++;
					}
				}
				else if(strstr(data , start_cmd)){
					admin_mode = 0;
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
					if(!isEmpty && goto_floor+1 != current_floor){
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
						deQueue();
						counter_flag = 1;
					}
				}
//				else{
//					char incorrect_pass_msg[16] = "\nwrong command!\n" ;
//					HAL_UART_Transmit(&huart2, &incorrect_pass_msg, sizeof(incorrect_pass_msg),1000);
//				}
			}



		}


			HAL_UART_Receive_IT(&huart2,data,20);

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
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2,data,20);
  HAL_UART_Transmit(&huart2, hello, sizeof(hello) , 1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT4_Pin MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC6 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
