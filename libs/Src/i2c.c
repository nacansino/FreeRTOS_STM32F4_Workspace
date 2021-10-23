#include <i2c.h>
#include <FreeRTOS.h>
#include <task.h>

static uint32_t timeout_ctr = 0;

#define TIMEOUT_DEADLINE 	(5000)
#define IS_NOT_TIMEOUT 		(timeout_ctr++ < TIMEOUT_DEADLINE)
#define HAS_TIMED_OUT 		(timeout_ctr >= TIMEOUT_DEADLINE)
#define RESET_TIMEOUT_CTR	(timeout_ctr = 0)

/**
 * @brief Blocks until the bus is free, or timeout
 * 
 * @param I2Cx 
 * @return int8_t Error Code
 */
int8_t I2C_WaitBusBusy(I2C_TypeDef *I2Cx)
{
	RESET_TIMEOUT_CTR;
	while(LL_I2C_IsActiveFlag_BUSY(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
	return HAS_TIMED_OUT ? -1 : 0;
}

/**
 * @brief 	Runs an I2C BUS reset. Use when a slave is not releasing the SDA line.
 * 			This converts the pins into GPIOs, clocks the SCK line 9 times,
 * 			then checks if the SDA has been released
 * 
 * @param GPIOx_SCK 	GPIO group of the SCK pin
 * @param PinMask_SCK 	Pin mask of the SCK pin
 * @param GPIOx_SDA 	GPIO group of the SDA pin
 * @param PinMask_SDA 	Pin mask of the SDA pin
 * @return int8_t Error Code
 * 			0: SDA has been released successfully
 */
int8_t I2C_BusReset(GPIO_TypeDef *GPIOx_SCK, uint32_t PinMask_SCK, 
					GPIO_TypeDef *GPIOx_SDA, uint32_t PinMask_SDA)
{
	LL_GPIO_InitTypeDef GPIO_Init_I2C = {
		.Mode = LL_GPIO_MODE_ALTERNATE,
		.Speed = LL_GPIO_SPEED_FREQ_HIGH,
		.OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
		.Pull = LL_GPIO_PULL_UP,
		.Alternate = LL_GPIO_AF_4
	};

	LL_GPIO_InitTypeDef GPIO_Init_GPIO = {
		.Speed = LL_GPIO_SPEED_FREQ_LOW,
		.Pull = LL_GPIO_PULL_NO
	};

	/* Set SCK as output, set SDA as input (for monitoring bus activity) */
	GPIO_Init_GPIO.Pin = PinMask_SDA;
	GPIO_Init_GPIO.Mode = LL_GPIO_MODE_INPUT;
	LL_GPIO_Init(GPIOx_SDA, &GPIO_Init_GPIO);
	GPIO_Init_GPIO.Pin = PinMask_SCK;
	GPIO_Init_GPIO.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_Init_GPIO.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOx_SCK, &GPIO_Init_GPIO);

	/* Issue 9 clock pulses */
	for (uint8_t i = 0; i < 9; ++i)
	{
		/* Interval of 1ms is OK */
		LL_GPIO_SetOutputPin(GPIOx_SCK, PinMask_SCK);
		vTaskDelay(1);
		LL_GPIO_ResetOutputPin(GPIOx_SCK, PinMask_SCK);
		vTaskDelay(1);
	}

	/* Monitor SDA. It should he HIGH by now. If not, fail */
	GPIO_PinState sda_state = LL_GPIO_IsInputPinSet(GPIOx_SDA, PinMask_SDA);
	if (!sda_state)
	{
		/* Error!! Can't take ownership of bus */
		return -1;
	}

	/* Revert the pins to I2C */
	GPIO_Init_I2C.Pin = PinMask_SCK;
	LL_GPIO_Init(GPIOx_SCK, &GPIO_Init_I2C);
	GPIO_Init_I2C.Pin = PinMask_SDA;
	LL_GPIO_Init(GPIOx_SDA, &GPIO_Init_I2C);

	return 0;
}

int8_t I2C_Write(I2C_TypeDef *I2Cx, const uint8_t* data, const size_t len, const I2C_GenStop_t gen_stop)
{
	size_t i = 0;

	while(i < len)
	{
		/* Wait for the transmit reg to be empty */
		RESET_TIMEOUT_CTR;
		while (!LL_I2C_IsActiveFlag_TXE(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
		if (HAS_TIMED_OUT)
		{
			/* Handle timeout */
			return -1;
		}

		LL_I2C_TransmitData8(I2Cx, data[i]);

		/* Wait for byte transfer completion */
		RESET_TIMEOUT_CTR;
		while (!LL_I2C_IsActiveFlag_BTF(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
		if (HAS_TIMED_OUT)
		{
			/* Handle timeout */
			return -2;
		}

		/* Check if NACK is detected (ack failure) */
		if (LL_I2C_IsActiveFlag_AF(I2Cx))
		{
			LL_I2C_ClearFlag_AF(I2Cx);
			return -3;
		}

		/* Generate stop during the last bit */
		if (gen_stop && (i == len - 1))
		{
			LL_I2C_GenerateStopCondition(I2Cx);
		}

		i++;
	}

	return 0;
}

int8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t* data, const size_t len, const I2C_Nack_t nack_last, const I2C_GenStop_t gen_stop)
{
	size_t i = 0;

	/* Enable Acknowledge */
	LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

	while(i < len)
	{
		if ( (i == len - 1) && nack_last )
		{
			/* No acknowledgment on the next received data */
			LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);

			/* Generate stop condition after this reception */
			if (gen_stop)
			{
				LL_I2C_GenerateStopCondition(I2Cx);
			}
		}

		/* Wait for the receive reg to be not empty */
		RESET_TIMEOUT_CTR;
		while (!LL_I2C_IsActiveFlag_RXNE(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
		if (HAS_TIMED_OUT)
		{
			/* Handle timeout */
			return -1;
		}

		data[i] = LL_I2C_ReceiveData8(I2Cx);

		i++;
	}

	return 0;
}

/**
 * @brief Issue an I2C START condition.
 * 		  
 * @warning This does NOT check for BUS busy condition.		  
 * 
 * @param I2Cx	I2C Handler
 * @param addr 	Device Address (with read/write bit)
 * @return int8_t	Error Code
 */
int8_t I2C_Start(I2C_TypeDef *I2Cx, const uint8_t addr)
{
	/* Generate Start */
	RESET_TIMEOUT_CTR;
	LL_I2C_GenerateStartCondition(I2Cx);
	while(!LL_I2C_IsActiveFlag_SB(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
	if (HAS_TIMED_OUT)
	{
		/* Handle timeout */
		return -1;
	}

	/* Send Address */
	RESET_TIMEOUT_CTR;
	LL_I2C_TransmitData8(I2Cx, addr);
	while (!LL_I2C_IsActiveFlag_ADDR(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
	if (HAS_TIMED_OUT)
	{
		/* Handle timeout */
		return -2;
	}

	/* Check if NACK is detected (ack failure) */
	if (LL_I2C_IsActiveFlag_AF(I2Cx))
	{
		LL_I2C_ClearFlag_AF(I2Cx);
		return -3;
	}

	LL_I2C_ClearFlag_ADDR(I2C1);

	return 0;
}

/**
 * @brief Issues an I2C STOP Condition
 * 
 * @param I2Cx	I2C Handler
 * @return int8_t Error Code
 */
int8_t I2C_Stop(I2C_TypeDef *I2Cx)
{
	LL_I2C_GenerateStopCondition(I2Cx);

	return 0;
}

