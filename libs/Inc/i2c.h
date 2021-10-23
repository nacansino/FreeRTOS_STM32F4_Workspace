#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

#include <stm32f4xx_ll_i2c.h>
#include <stm32f4xx_ll_gpio.h>

typedef enum
{
	I2C_Nack_Disable 	= 0,
	I2C_Nack_Enable 	= 1
} I2C_Nack_t;

typedef enum
{
	I2C_GenStop_Disable = 0,
	I2C_GenStop_Enable 	= 1
} I2C_GenStop_t;

/**
 * Functions:
 * 1. send one or more bytes
 * 2. receive one or more bytes
 *
 * Limitations:
 * 1. Works only for 7-bit addresses
 */

int8_t I2C_BusReset(GPIO_TypeDef *GPIOx_SCK, uint32_t PinMask_SCK, GPIO_TypeDef *GPIOx_SDA, uint32_t PinMask_SDA);
extern int8_t I2C_Write(I2C_TypeDef *I2Cx, const uint8_t* data, const size_t len, const I2C_GenStop_t gen_stop);
extern int8_t I2C_Read(I2C_TypeDef *I2Cx, uint8_t* data, const size_t len, const I2C_Nack_t nack_last, const I2C_GenStop_t gen_stop);
extern int8_t I2C_Start(I2C_TypeDef *I2Cx, const uint8_t addr);
extern int8_t I2C_Stop(I2C_TypeDef *I2Cx);

#endif
