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
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "SEGGER_SYSVIEW.h"

#include <printf.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	char buffer[128];
	size_t sz;
} ShellQueueTX_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHELL_QUEUE_RX_DEPTH	(32)
#define SHELL_QUEUE_TX_DEPTH	(16)
#define SHELL_QUEUE_TX_BUFF_SZ  (128)	/**!< Maximum size of each item in the
											  shell TX queue */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DWT_CTRL	((volatile uint32_t*)0xE0001000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
static TaskHandle_t task0_handle, task1_handle,
				          	task2_handle, task3_handle,
					          task4_handle, task_shell_rx_handle, task_shell_tx_handle;
static char shell_char_rcv;

QueueHandle_t shell_queue_rx, shell_queue_tx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
extern void SEGGER_UART_init(U32 baud);

extern size_t uxTaskGetTCBSize(void);
extern uint32_t uxTaskGetStackSize(TaskHandle_t xTask);

static void vTask0(void * pvParameters);
static void vTask1(void * pvParameters);
static void vTask2(void * pvParameters);
static void vTask3(void * pvParameters);
static void vTask4(void * pvParameters);
static void vTask_Shell_RX(void * pvParameters);
static void vTask_Shell_TX(void * pvParameters);

static int Shell_Printf(const char* format, ...);

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
  BaseType_t task0_rv, task1_rv, task2_rv,
	  	  	 task3_rv, task4_rv,
			 task_shell_rx_rv, task_shell_tx_rv;

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Enable cycle counting by setting the bit 0 of DWT_CTRL */
  *DWT_CTRL |= 0x1;

  NVIC_SetPriorityGrouping(0); // Set when using segger on debugger mode (not realtime)

  //SEGGER_UART_init(500000);

  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();	// no need to do this since SEGGER_SYSVIEW_Start() is already called inside SEGGER_UART_init();

  /* Determine how big a TCB is */
  char msg[32] = {0};
  snprintf(msg, 32, "Size of TCB: %d bytes", uxTaskGetTCBSize());
  SEGGER_SYSVIEW_PrintfTarget(msg);

  /* Create the task, storing the handle.
   * With hard assertion check for passing tasks
   */
  task0_rv = xTaskCreate(vTask0, "Task 0", 200, NULL, 1, &task0_handle);
  configASSERT(task0_rv == pdPASS);
  task1_rv = xTaskCreate(vTask1, "Task 1", 200, NULL, 1, &task1_handle);
  configASSERT(task1_rv == pdPASS);
  task2_rv = xTaskCreate(vTask2, "Task 2", 200, NULL, 1, &task2_handle);
  configASSERT(task2_rv == pdPASS);
  task3_rv = xTaskCreate(vTask3, "Task 3", 200, NULL, 1, &task3_handle);
  configASSERT(task3_rv == pdPASS);
  task4_rv = xTaskCreate(vTask4, "Task 4", 200, NULL, 2, &task4_handle);
  configASSERT(task4_rv == pdPASS);
  task_shell_rx_rv = xTaskCreate(vTask_Shell_RX, "Shell RX Task", 200, NULL, 5, &task_shell_rx_handle);
  configASSERT(task_shell_rx_rv == pdPASS);
  task_shell_tx_rv = xTaskCreate(vTask_Shell_TX, "Shell TX Task", 200, NULL, 4, &task_shell_tx_handle);
  configASSERT(task_shell_tx_rv == pdPASS);

  /* Create the queues for the debugger */
  shell_queue_rx = xQueueCreate(SHELL_QUEUE_RX_DEPTH, sizeof(char));
  shell_queue_tx = xQueueCreate(SHELL_QUEUE_TX_DEPTH, sizeof(ShellQueueTX_t));

  if( (shell_queue_rx == NULL) || (shell_queue_tx == NULL) )
  {
	  /* Queue was not created and must not be used. */
  }

  /* Start DBG_USR (USART3) Reception */
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&shell_char_rcv, 1);


  /* Start the scheduler */
  vTaskStartScheduler();

  /* If the control comes here then the launch of the scheduler has failed
   due to the insufficient memory */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void vTask0(void * pvParameters)
{
	while(1)
	{
		if (xTaskNotifyWait(0, 0, NULL, 0))
		{
			// suspend self
						vTaskSuspend(NULL);
		}
		HAL_Delay(4000);
		vTaskDelay(pdMS_TO_TICKS(4000));
	}
}

static void vTask1(void * pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		if (xTaskNotifyWait(0, 0, NULL, 0))
		{
			// suspend self
			vTaskSuspend(NULL);
		}
		SEGGER_SYSVIEW_PrintfTarget("GREEN_TOGGLE stack size: %d", uxTaskGetStackSize(task1_handle));
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

		Shell_Printf("fuck you\r\n");
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
	}
}

static void vTask2(void * pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		if (xTaskNotifyWait(0, 0, NULL, 0))
		{
			// suspend self
			vTaskSuspend(NULL);
		}
		SEGGER_SYSVIEW_PrintfTarget("ORANGE_TOGGLE stack size: %d", uxTaskGetStackSize(task2_handle));
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(800));
	}
}

static void vTask3(void * pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		if (xTaskNotifyWait(0, 0, NULL, 0))
		{
			// suspend self
			vTaskSuspend(NULL);
		}
		SEGGER_SYSVIEW_PrintfTarget("RED_TOGGLE stack size: %d", uxTaskGetStackSize(task3_handle));
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(400));
	}
}

static void vTask4(void * pvParameters)
{
#define USER_PIN_STATE HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)
	uint16_t debounceCtr = 0;
	uint8_t isPressedCtr = 0;
	bool isPressed = false;

	while(1)
	{
		if (USER_PIN_STATE == GPIO_PIN_SET)
		{
			if (!isPressed)
			{
				debounceCtr++;
				if (debounceCtr > 2)
				{
					isPressed = true;
					SEGGER_SYSVIEW_PrintfTarget("User Button Pressed %d times", ++isPressedCtr);

					// Distribute notifications
					switch(isPressedCtr)
					{
						case 1:
							(void)xTaskNotify(task1_handle, 0xFFFFFFFF, eSetBits);
							break;
						case 2:
							(void)xTaskNotify(task2_handle, 0xFFFFFFFF, eSetBits);
							break;
						case 3:
							(void)xTaskNotify(task3_handle, 0xFFFFFFFF, eSetBits);
							break;
						case 4:
							(void)xTaskNotify(task0_handle, 0xFFFFFFFF, eSetBits);
							break;
						case 5:
							// resume all tasks then reset isPressedCtr
							vTaskResume(task1_handle);
							vTaskResume(task2_handle);
							vTaskResume(task3_handle);
							vTaskResume(task0_handle);
							isPressedCtr = 0;
						default:
							// Nothing to do here
							break;
					};

				}
			}
			else
			{
				// do nothing. button is pressed and detected already
			}
		}
		else
		{
			// button is released
			debounceCtr = 0;
			if (isPressed)
			{
				SEGGER_SYSVIEW_PrintfTarget("User Button Released");
				isPressed = false;
			}
		}

		// Poll for 10ms
		vTaskDelay(10);
	}
}

static void vTask_Shell_RX(void * pvParameters)
{
	uint32_t ulNotifiedValue;

	while(1)
	{
		/* Block indefinitely while we wait for user input */
		xTaskNotifyWait(0x0, 0x0, &ulNotifiedValue, portMAX_DELAY);

        if( ( ulNotifiedValue & 0x01 ) != 0 )
        {
            /* Bit 0 was set: CR/LF was received */

        	/* Nothing to do here in theory */
        }

        if( ( ulNotifiedValue & 0x02 ) != 0 )
        {
            /* Bit 1 was set: Input queue is full */

        	/* Raise Error */
        }

        /* Process buffer */


	}
}

static void vTask_Shell_TX(void * pvParameters)
{
	while(1)
	{
		ShellQueueTX_t pTxItem;

		xQueuePeek( shell_queue_tx, (void* const)&pTxItem, portMAX_DELAY );

		if ( HAL_UART_Transmit_DMA( &huart3, (uint8_t*)&(pTxItem.buffer), pTxItem.sz) == HAL_OK )
		{
			/* Receive to remove used item from queue
			 * It means the item has been processed already*/
			(void)xQueueReceive( shell_queue_tx, (void* const)&pTxItem, (TickType_t) 0);
		}
		else
		{
			/* Delay 100ms before attempting again */
			vTaskDelay(pdMS_TO_TICKS(50));
		}
	}
}

/**
 * @brief Idle hook for kernel
 * 
 */
void vApplicationIdleHook( void )
{
  /* Set to sleep. Wake up when there is an interrupt */
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/**
 * UART Receive Callback IT
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart3)
	{
		BaseType_t queue_rv;

		/* Push to queue if received character is not CR*/
		if (shell_char_rcv != '\r')
		{
			queue_rv = xQueueSendFromISR( shell_queue_rx, &shell_char_rcv, NULL );

			if (queue_rv != pdTRUE)
			{
				/* Send notification to task that buffer is full */
				(void)xTaskNotifyFromISR( task_shell_rx_handle, 0x02, eSetBits, NULL );
			}
		}
		else
		{
			/* Notify the task that a carriage return was received */
			(void)xTaskNotifyFromISR( task_shell_rx_handle, 0x01, eSetBits, NULL );
		}

		HAL_UART_Receive_IT(&huart3, (uint8_t*)&shell_char_rcv, 1);
	}
}

/**
 * Shell function
 * Do not use inside any ISR
 */
static int Shell_Printf(const char* format, ...)
{
  ShellQueueTX_t buffer_tx;
  va_list va;

  /* Format input then write it in buffer for a
   * maximum length of SHELL_QUEUE_TX_BUFF_SZ
   */
  va_start(va, format);
  const int ret = vsnprintf(buffer_tx.buffer, SHELL_QUEUE_TX_BUFF_SZ, format, va);
  va_end(va);

  if (ret > 0){
	  /* Push to queue*/
	  BaseType_t queue_rv;

	  buffer_tx.sz = ret;
	  queue_rv = xQueueSend( shell_queue_tx, (void*)&buffer_tx, 0);

	  if (queue_rv != pdTRUE)
	  {
		  /* Todo: Handle case when buffer is full */
	  }
  }

  return ret;
}

/**
 * Implementation of _putchar for normal printf
 */
void _putchar(char character)
{
	(void)character;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
