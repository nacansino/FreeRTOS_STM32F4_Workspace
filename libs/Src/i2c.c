#include <i2c.h>
#include <FreeRTOS.h>
#include <task.h>

static uint32_t timeout_ctr = 0;

#define TIMEOUT_DEADLINE 	(10000)
#define IS_NOT_TIMEOUT 		(timeout_ctr++ < TIMEOUT_DEADLINE)
#define HAS_TIMED_OUT 		(timeout_ctr >= TIMEOUT_DEADLINE)
#define RESET_TIMEOUT_CTR	(timeout_ctr = 0)

int8_t I2C_Write(I2C_TypeDef *I2Cx, const uint8_t* data, const size_t len)
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

		i++;
	}

	return 0;
}

int8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t* data, size_t len)
{
	size_t i = 0;

	/* Enable Acknowledge */
	LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

	while(i < len)
	{
		/* Wait for the receive reg to be not empty */
		RESET_TIMEOUT_CTR;
		while (!LL_I2C_IsActiveFlag_RXNE(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
		if (HAS_TIMED_OUT)
		{
			/* Handle timeout */
			return -1;
		}

		data[i] = LL_I2C_ReceiveData8(I2Cx);

		/* Wait for byte transfer completion */
		RESET_TIMEOUT_CTR;
		while (!LL_I2C_IsActiveFlag_BTF(I2Cx) && IS_NOT_TIMEOUT) taskYIELD();
		if (HAS_TIMED_OUT)
		{
			/* Handle timeout */
			return -2;
		}

		i++;
	}

	/* No acknowledgment here */
	LL_I2C_AcknowledgeNextData(I2Cx,LL_I2C_NACK);

	return 0;
}

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

int8_t I2C_Stop(I2C_TypeDef *I2Cx)
{
	LL_I2C_GenerateStopCondition(I2Cx);

	return 0;
}

