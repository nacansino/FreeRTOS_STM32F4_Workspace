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
#include <shell/shell.h>
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
#define SEGGER_DEBUGGER_MODE	(1)
#define SEGGER_REALTIME_MODE	(2)

#define SEGGER_MODE	SEGGER_REALTIME_MODE

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

/* USER CODE BEGIN PV */
static TaskHandle_t task1_handle,
				    task2_handle, task3_handle,
					task4_handle, task_shell_rx_handle,
					task_shell_tx_handle;
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

static void vTask1(void * pvParameters);
static void vTask2(void * pvParameters);
static void vTask3(void * pvParameters);
static void vTask4(void * pvParameters);
static void vTask_Shell_RX(void * pvParameters);
static void vTask_Shell_TX(void * pvParameters);

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
  BaseType_t task1_rv, task2_rv,
	  	  	 task3_rv, task4_rv,
			 task_shell_rx_rv, task_shell_tx_rv;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Bug fix for the initialization order.
   * DMA must be initialized BEFORE the peripherals that uses them.
   */
  MX_DMA_Init();

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

#if SEGGER_MODE==SEGGER_REALTIME_MODE
  SEGGER_UART_init(500000);
  SEGGER_SYSVIEW_Conf();
#elif SEGGER_MODE==SEGGER_DEBUGGER_MODE
  NVIC_SetPriorityGrouping(0); // Set when using segger on debugger mode (not realtime)
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();	// no need to do this since SEGGER_SYSVIEW_Start() is already called inside SEGGER_UART_init();
#endif

  /* Determine how big a TCB is */
  char msg[32] = {0};
  snprintf(msg, 32, "Size of TCB: %d bytes", uxTaskGetTCBSize());
  SEGGER_SYSVIEW_PrintfTarget(msg);

  /* Create the task, storing the handle.
   * With hard assertion check for passing tasks
   */
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

  /* Initialize the shell */
  sShellImpl shell_impl = {
    .send_printf = Shell_Printf,
  };
  shell_boot(&shell_impl);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PB11   ------> USART3_RX
  PD8   ------> USART3_TX
  */
  GPIO_InitStruct.Pin = DBG_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(DBG_RX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DBG_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(DBG_TX_GPIO_Port, &GPIO_InitStruct);

  /* USART3 DMA Init */

  /* USART3_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin);

  /**/
  LL_GPIO_SetOutputPin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin);

  /**/
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE1);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(MEMS_INT2_GPIO_Port, MEMS_INT2_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(MEMS_INT2_GPIO_Port, MEMS_INT2_Pin, LL_GPIO_MODE_INPUT);

}

/* USER CODE BEGIN 4 */

static void vTask1(void * pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static uint8_t toggler;

	while(1)
	{
		if (xTaskNotifyWait(0, 0, NULL, 0))
		{
			// suspend self
			vTaskSuspend(NULL);
		}
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

		Shell_Printf("Task1 Toggler\r\n");

		toggler ^= 1;
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
						case 5:
							// resume all tasks then reset isPressedCtr
							vTaskResume(task1_handle);
							vTaskResume(task2_handle);
							vTaskResume(task3_handle);
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
	char c;

	while(1)
	{
		/* Block indefinitely while we wait for user input */
     	(void)xQueueReceive(shell_queue_rx, &c, (TickType_t)portMAX_DELAY);
     	shell_receive_char(c);
	}
}

static void vTask_Shell_TX(void * pvParameters)
{
	while(1)
	{
		ShellQueueTX_t pTxItem;

		if ( xQueueReceive( shell_queue_tx, (void* const)&pTxItem, (TickType_t) portMAX_DELAY) == pdTRUE )
		{
			/* Here we have the whole buffer to be sent, pTxItem.buffer with size pTxItem.sz */

			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3);
			LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_3, (uint32_t)pTxItem.buffer,
					LL_USART_DMA_GetRegAddr(USART3), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, pTxItem.sz);
			LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
			LL_USART_EnableDMAReq_TX(USART3);

			/* Block until DMA completion */
			xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

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

#if 0
/**
 * UART Receive Callback IT
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	if (huart == &huart3)
	{
		BaseType_t queue_rv;

		/* Push to queue if received character is not CR*/
		queue_rv = xQueueSendFromISR( shell_queue_rx, &shell_char_rcv, &xHigherPriorityTaskWoken );

		if (queue_rv != pdTRUE)
		{
			/* Send notification to task that buffer is full */
			(void)xTaskNotifyFromISR( task_shell_rx_handle, 0x02, eSetBits, &xHigherPriorityTaskWoken );
		}

		HAL_UART_Receive_IT(&huart3, (uint8_t*)&shell_char_rcv, 1);
	}

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
    }
}

#endif

/**
 * UART Transmit Callback IT
 */
void USART3_TX_DMA1_Callback(void)
{
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	if (LL_DMA_IsActiveFlag_TC3(DMA1) == 1)
	{
		/* Clear DMA flag */
		LL_DMA_ClearFlag_TC3(DMA1);

		/* Notify the TX to wake up*/
		(void)xTaskNotifyFromISR( task_shell_tx_handle, 0, eSetBits, &xHigherPriorityTaskWoken );
	}

	if( xHigherPriorityTaskWoken )
	{
		portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
	}
}

/**
 * Shell function
 * Do not use inside any ISR
 */
int Shell_Printf(const char* format, ...)
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
 * This is unused.
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
