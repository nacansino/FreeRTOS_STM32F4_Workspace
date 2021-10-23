#include <lsm9ds1.h>
#include <lsm9ds1_regaddr.h>
#include <stdbool.h>

#define DEV_ADDR_MAG 0x1C
#define DEV_ADDR_AG  0x6A
#define DEV_ADDR_WRITE(addr) (addr << 1)
#define DEV_ADDR_READ(addr)  ((addr << 1) | 1)
#define XL_ACC_GRAV (9.80665)

typedef enum
{
	XL_FSR_2G  = 2,
	XL_FSR_4G  = 4,
	XL_FSR_8G  = 8,
	XL_FSR_16G = 24, /*!< This is the conversion factor for 16g as per datasheet 2.1 to produce 0.732 mg/LSB */
} XL_FSR_t;

typedef enum
{
	G_FSR_245dps	= 245,
	G_FSR_500dps	= 500,
	G_FSR_2000dps	= 2000
} G_FSR_t;

static int8_t prv_readReg(uint8_t regaddr, uint8_t* o_data, const size_t num_bytes);
static int8_t prv_writeReg(uint8_t regaddr, const uint8_t* const i_data, const uint8_t num_bytes);

static I2C_TypeDef *I2C_DEV = NULL;

LSM9DS1_Err_t LSM9DS1_Init(I2C_TypeDef *I2Cx)
{
	int8_t rv;
	uint8_t data;

	if (I2Cx == NULL) return LSM9DS1_Err_Init_NullPtr;
	
	I2C_DEV = I2Cx;

	/* Read device address */
	uint8_t wmidata = 0;
	rv = prv_readReg(0x0F, &wmidata, 1);
	if (rv) return LSM9DS1_Err_I2C;

	/* Config Accel */
	data = 	LSM9DS1_ACCELEROMETER_DATA_RATE_238HZ |		// accl output rate 238Hz
			LSM9DS1_ACCELEROMETER_SCALE_4G;				// Set Accelerometer full scale
	prv_writeReg(LSM9DS1_REGISTER_CTRL_REG6_XL, &data, 1);
	
	/* Config Gyro */
	data = 	LSM9DS1_GYROSCOPE_DATA_RATE_238_HZ |	// gyro output rate 238Hz
			LSM9DS1_GYROSCOPE_SCALE_245_DPS;		// Set gyro full scale 245dps
	prv_writeReg(LSM9DS1_REGISTER_CTRL_REG1_G, &data, 1);

	return LSM9DS1_Err_OK;
}

void LSM9DS1_RawXLToMS2(int16_t* i_accel_xyz, float* o_accel_xyz, uint8_t len)
{
	for(uint8_t i = 0; i < len; ++i)
	{
		o_accel_xyz[i] = (float)(i_accel_xyz[i]) * XL_FSR_4G * XL_ACC_GRAV /  (1 << 15);
	}
}

void LSM9DS1_RawGToDPS(int16_t* i_gyro_xyz, float* o_gyro_xyz, uint8_t len)
{
	for(uint8_t i = 0; i < len; ++i)
	{
		o_gyro_xyz[i] = (float)(i_gyro_xyz[i]) * G_FSR_245dps /  (1 << 15);
	}
}

LSM9DS1_Err_t LSM9DS1_ReadRawData(int16_t* accel_xyz, int16_t* gyro_xyz)
{
	int8_t rv = 0;

	/* Read Accel data */
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_X_XL_L, (uint8_t*)&accel_xyz[0], 2);
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_Y_XL_L, (uint8_t*)&accel_xyz[1], 2);
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_Z_XL_L, (uint8_t*)&accel_xyz[2], 2);

	/* Read Gyro data */
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_X_G_L, (uint8_t*)&gyro_xyz[0], 2);
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_Y_G_L, (uint8_t*)&gyro_xyz[1], 2);
	rv = rv | prv_readReg(LSM9DS1_REGISTER_OUT_Z_G_L, (uint8_t*)&gyro_xyz[2], 2);

	if (rv) return LSM9DS1_Err_DataRead;

	return LSM9DS1_Err_OK;
}

static int8_t prv_readReg(uint8_t regaddr, uint8_t* o_data, const size_t num_bytes)
{
	int8_t rv;

	if (I2C_DEV == NULL) return -1;

	/* Start (start condition + Address) */
	rv = I2C_Start(I2C1, DEV_ADDR_WRITE(DEV_ADDR_AG));
	if (rv)
	{
		return rv;
	}

	/* Send register sub-address (WHO-AM-I) */
	rv = I2C_Write(I2C1, &regaddr, 1, false);
	if (rv)
	{
		return rv;
	}

	/* Send repeated start with read-address
	 * The stop is
	 */
	rv = I2C_Start(I2C1, DEV_ADDR_READ(DEV_ADDR_AG));
	if (rv)
	{
		return rv;
	}
	rv = I2C_Read(I2C1, o_data, num_bytes, I2C_Nack_Enable, I2C_GenStop_Enable);
	if (rv)
	{
		return rv;
	}

	return rv;
}

static int8_t prv_writeReg(uint8_t regaddr, const uint8_t* const i_data, const uint8_t num_bytes)
{
	int8_t rv;

	/* Start (start condition + Address) */
	rv = I2C_Start(I2C1, DEV_ADDR_WRITE(DEV_ADDR_AG));
	if (rv)
	{
		return rv;
	}

	/* Send register sub-address (WHO-AM-I) */
	rv = I2C_Write(I2C1, &regaddr, 1, I2C_GenStop_Disable);
	if (rv)
	{
		return rv;
	}

	/* Send data */
	rv = I2C_Write(I2C1, i_data, num_bytes, I2C_GenStop_Enable);
	if (rv)
	{
		return rv;
	}
	
	return rv;
}
