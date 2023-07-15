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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
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

TIM_HandleTypeDef htim1;

/* Definitions for defaultTask */
osThreadId_t buttonTaskHandle;
osThreadId_t readDistanceTaskHandle;
osThreadId_t displayDistanceTaskHandle;
osMessageQueueId_t queue;
osSemaphoreId_t semaphore;

const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t readDistanceTask_attributes = {
  .name = "readDistance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t displayDistanceTask_attributes = {
  .name = "displayDistance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t lcd_status = LCD_OK;

#define TRIG_PIN GPIO_PIN_15
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_11
#define ECHO_PORT GPIOA

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_PORT GPIOA

#define GREEN_PIN GPIO_PIN_5
#define RED_PIN GPIO_PIN_13
#define LED_PORT GPIOJ
uint8_t THRESHOLD = 3;


//uint8_t THRESHOLD = 20;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void StartButtonTask(void *argument);
void StartReadDistanceTask(void *argument);
void StartDisplayDistanceTask(void *argument);

//void openGame(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < time);
}

typedef struct {
	uint16_t distance;
	uint16_t saved_distance;
} Distance;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t width;
	uint16_t height;
	uint16_t speed;
} Rectangle;

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);

  lcd_status = BSP_LCD_Init();
  while(lcd_status != LCD_OK);

  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

  uint8_t str[] = "Press the button to start !!";
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, str, CENTER_MODE);

  queue = osMessageQueueNew(1, sizeof(Distance), NULL);
  semaphore = osSemaphoreNew(1U, 0U, NULL);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);
  readDistanceTaskHandle = osThreadNew(StartReadDistanceTask, NULL, &readDistanceTask_attributes);
  displayDistanceTaskHandle = osThreadNew(StartDisplayDistanceTask, NULL, &displayDistanceTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
	 clocked below the maximum system frequency, to update the voltage scaling value
	 regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
	while(1) { ; }
  }
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 216-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Button
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Green/Red light
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_13, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void StartReadDistanceTask(void *argument)
{
    uint32_t start_time;
    uint32_t end_time;
    uint32_t tick;

    Distance d;
    d.saved_distance = 0;

    for (;;)
    {
        if (osSemaphoreAcquire(semaphore, osWaitForever) == osOK) {
            HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
            delay(2); // wait 2 us
            HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
            delay(10); // wait 10 us
            HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

            tick = HAL_GetTick();
            while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && tick + 10 > 0); // wait for echo to go to high state
            start_time = __HAL_TIM_GET_COUNTER(&htim1);

            tick = HAL_GetTick();
            while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && tick + 50 > 0); // wait for echo to go to low state
            end_time = __HAL_TIM_GET_COUNTER(&htim1);

            // Update the distance structure because of the saved distance
            osMessageQueueGet(queue, &d, NULL, 10);

            d.distance = (end_time - start_time) * 0.034 / 2;

            // Save the current distance if it's below the threshold
            if (d.distance < THRESHOLD) {
                d.saved_distance = d.distance;
            }

            osMessageQueuePut(queue, &d, 0, 10);
            osSemaphoreRelease(semaphore);
        }
        HAL_Delay(150);
    }
}


void StartDisplayDistanceTask(void *argument)
{
    Distance d;
    d.saved_distance = 0;
    unsigned char str[20];

    for (;;) {
        osMessageQueueGet(queue, &d, NULL, osWaitForever);
        if (osSemaphoreAcquire(semaphore, osWaitForever) == osOK) {
            osMessageQueuePut(queue, &d, NULL, 10);
            osSemaphoreRelease(semaphore);

            sprintf((char*)str, "    %d    ", d.distance);
            BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 20, str, CENTER_MODE);

            if (d.saved_distance && d.distance < d.saved_distance) {
                HAL_GPIO_WritePin(LED_PORT, RED_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_PORT, GREEN_PIN, GPIO_PIN_RESET);
            } else if (d.saved_distance) {
                HAL_GPIO_WritePin(LED_PORT, RED_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_PORT, GREEN_PIN, GPIO_PIN_SET);
            }

            // Check if distance exceeds the threshold
            if (d.distance > THRESHOLD) {
                // Simulate an alarm (e.g., display a message or sound a buzzer)
                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 60, (uint8_t*)"ALARM!", CENTER_MODE);
            } else {
                // Clear the alarm
                BSP_LCD_ClearStringLine(BSP_LCD_GetYSize() / 2 + 60);
            }
        }
        osDelay(150);
    }
}



Rectangle getRndRect() {
	uint32_t x = rand()%BSP_LCD_GetXSize();
	uint32_t y = rand()%BSP_LCD_GetYSize()/3;
	uint8_t width = 3+rand()%5;
	uint8_t height = 7*width+rand()%30;
	uint8_t speed = height/3+rand()%20;

	Rectangle rect = {x, y, width, height, speed};
	return rect;

}

void displayRain(void)
{
	uint8_t N_RECTS = 20;
	Rectangle rects[N_RECTS];
	Rectangle rect;

	for (int k=0; k<N_RECTS; k++) {
		rect = getRndRect();
		rects[k] = rect;
	}

	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN));

	for (;;)
	{
		if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
			return;
		}
		BSP_LCD_Clear(LCD_COLOR_BLACK);

		for (int i=0; i<N_RECTS; i++){

			BSP_LCD_FillRect(rects[i].x, rects[i].y, rects[i].width, rects[i].height);
			rects[i].y+=rects[i].speed;
			if (rects[i].y+rects[i].height >= BSP_LCD_GetYSize()) {
				rect = getRndRect();
				rects[i] = rect;
			}
			osDelay(1);
		}
		osDelay(10);
	}
}

/* USER CODE END 4 */


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	unsigned char str[20];
	Distance d;
	uint8_t started = 0;
//	uint32_t start_time;
//	uint32_t end_time;
	uint32_t time;
	for(;;)
    {
		if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
			time=0;
			while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)){
				time++;
				osDelay(1);
				if (time > 2000) {
					//while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN));
					if (osSemaphoreAcquire(semaphore, osWaitForever) == osOK) {
						displayRain();
						osSemaphoreRelease(semaphore);
						BSP_LCD_Clear(LCD_COLOR_WHITE);
						BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2-20, (uint8_t *) "Distance (cm):", CENTER_MODE);
//						started = 0;
					}
					break;
				}

			} // Wait to release the button

			if (started) {
				osMessageQueueGet(queue, &d, NULL, osWaitForever);
				if (osSemaphoreAcquire(semaphore, osWaitForever) == osOK) {
					d.saved_distance = d.distance;
					osMessageQueuePut(queue, &d, NULL, 10U);
					osSemaphoreRelease(semaphore);
				}
				sprintf(&str,"Saved distance: %d    ", d.saved_distance);
				BSP_LCD_DisplayStringAtLine(19, str);
	    	} else {
	    		started = 1;
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2-20, (uint8_t *) "Distance (cm):", CENTER_MODE);
			    osSemaphoreRelease(semaphore);
	    	}
	    }
	    osDelay(150);
    }
  /* USER CODE END 5 */
}

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

