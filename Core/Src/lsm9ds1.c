#include <lsm9ds1.h>
#include <lsm9ds1_regaddr.h>
#include <stdbool.h>

#define DEV_ADDR_MAG 0x1C
#define DEV_ADDR_AG  0x6A
#define DEV_ADDR_WRITE(addr) (addr << 1)
#define DEV_ADDR_READ(addr)  ((addr << 1) | 1)

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
	data = 	LSM6DS3_ACC_GYRO_ODR_XL_104Hz |			// accl output rate 104Hz
			LSM6DS3_ACC_GYRO_FS_XL_4g |				// Set Accelerometer full scale
			LSM6DS3_ACC_GYRO_BW_XL_DEFAULT;			// Anti-aliasing filter bandwidth selection.
													// Set XL_BW_SCAL_ODR to 1 for the selection to take effect
	prv_writeReg(LSM6DS3_ACC_GYRO_CTRL1_XL, &data, 1);
	
	data = 	LSM6DS3_ACC_GYRO_ODR_G_104Hz |			// gyro output rate 104Hz
			LSM6DS3_ACC_GYRO_FS_G_250dps ;			// Set gyro full scale
	prv_writeReg(LSM6DS3_ACC_GYRO_CTRL2_G, &data, 1);

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
	rv = I2C_Write(I2C1, &regaddr, 1);
	if (rv)
	{
		return rv;
	}

	/* Send repeated start with read-address */
	rv = I2C_Start(I2C1, DEV_ADDR_READ(DEV_ADDR_AG));
	if (rv)
	{
		return rv;
	}
	rv = I2C_Read(I2C1, o_data, num_bytes, true);
	if (rv)
	{
		return rv;
	}

	/* Send STOP condition */
	rv = I2C_Stop(I2C1);

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
	rv = I2C_Write(I2C1, &regaddr, 1);
	if (rv)
	{
		return rv;
	}

	/* Send data */
	rv = I2C_Write(I2C1, i_data, num_bytes);
	if (rv)
	{
		return rv;
	}

	/* Send STOP condition */
	rv = I2C_Stop(I2C1);
	
	return rv;
}
