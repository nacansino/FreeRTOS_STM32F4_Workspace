#ifndef LSM9DS1_H_
#define LSM9DS1_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <i2c.h>

typedef enum {
    LSM9DS1_Err_OK = 0,
    LSM9DS1_Err_I2C = -1,
    LSM9DS1_Err_DataRead = -2,

    LSM9DS1_Err_Init_NullPtr = -3,
} LSM9DS1_Err_t;

extern LSM9DS1_Err_t LSM9DS1_Init(I2C_TypeDef *I2Cx);
extern LSM9DS1_Err_t LSM9DS1_ReadRawData(int16_t* accel_xyz, int16_t* gyro_xyz);
extern void LSM9DS1_RawXLToMS2(int16_t* i_accel_xyz, float* o_accel_xyz, uint8_t len);
extern void LSM9DS1_RawGToDPS(int16_t* i_gyro_xyz, float* o_gyro_xyz, uint8_t len);

#endif
