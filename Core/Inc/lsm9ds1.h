#ifndef LSM9DS1_H_
#define LSM9DS1_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <i2c.h>

typedef enum {
    LSM9DS1_Err_OK = 0,
    LSM9DS1_Err_I2C,

    LSM9DS1_Err_Init_NullPtr = -1,
} LSM9DS1_Err_t;

extern LSM9DS1_Err_t LSM9DS1_Init(I2C_TypeDef *I2Cx);

#endif
