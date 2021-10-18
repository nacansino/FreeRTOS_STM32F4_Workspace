#ifndef LSM9DS1_REGADDR_H_
#define LSM9DS1_REGADDR_H_

/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_TEST_PAGE  		0X00
#define LSM6DS3_ACC_GYRO_RAM_ACCESS  		0X01
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME  	0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN  	0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1  		0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2  		0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3  		0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4  		0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5  		0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G  		0X0B
#define LSM6DS3_ACC_GYRO_REFERENCE_G  		0X0C
#define LSM6DS3_ACC_GYRO_INT1_CTRL  		0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL  		0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG  		0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL  			0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G  			0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C  			0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C  			0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C  			0X14
#define LSM6DS3_ACC_GYRO_CTRL6_G  			0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  			0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  			0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  			0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  			0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  	0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  		0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  			0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  			0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  		0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L  		0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H  		0X21
#define LSM6DS3_ACC_GYRO_OUTX_L_G  			0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G  			0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G  			0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G  			0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G  			0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G  			0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL  		0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL  		0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL  		0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL  		0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL  		0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL  		0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  	0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  	0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  	0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  	0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  	0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  	0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  	0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  	0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  	0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  	0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  	0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  	0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  		0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  		0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  		0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  		0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  	0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  	0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  	0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  	0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  	0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  	0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  	0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC  			0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1  			0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  		0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  			0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  		0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  		0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  		0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  			0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  			0X5F


/***************** Fixed value **********************/
#define LSM6DS3_WHO_AM_I_VALUE 0x69
#define LSM6DSM_WHO_AM_I_VALUE 0x6A

/********************* MASK *************************/

/*******************************************************************************
* Register      : FIFO_STATUS1
* Address       : 0X3A
* Bit Group Name: DIFF_FIFO
* Permission    : RO
*******************************************************************************/
#define   LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_MASK   0xFF
#define   LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION   0
#define   LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_MASK  0xF
#define   LSM6DS3_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION   0

/*******************************************************************************
* Register      : FIFO_STATUS3
* Address       : 0X3C
* Bit Group Name: FIFO_PATTERN
* Permission    : RO
*******************************************************************************/
#define   LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK    0xFF
#define   LSM6DS3_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION    0
#define   LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK    0x03
#define   LSM6DS3_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION    0

/*******************************************************************************
* Register      : TIMESTAMP2_REG
* Address       : 0X42
* Bit Group Name: TIMESTAMP
* Permission    : RW
*******************************************************************************/
#define   LSM6DS3_ACC_GYRO_TIMESTAMP2_REG_RESET_COUNTER    0xAA

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: TIM_PEDO_FIFO_DRDY
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_ENABLED      = 0x40,
} LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_DRDY_t;

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: TIM_PEDO_FIFO_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_ENABLED      = 0x80,
} LSM6DS3_ACC_GYRO_TIM_PEDO_FIFO_EN_t;

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO      = 0x00,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION     = 0x01,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2     = 0x02,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3     = 0x03,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4     = 0x04,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8     = 0x05,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16      = 0x06,
  LSM6DS3_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32      = 0x07,
} LSM6DS3_ACC_GYRO_DEC_FIFO_XL_t;

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO     = 0x00,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION      = 0x08,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2      = 0x10,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3      = 0x18,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4      = 0x20,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8      = 0x28,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16     = 0x30,
  LSM6DS3_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32     = 0x38,
} LSM6DS3_ACC_GYRO_DEC_FIFO_G_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_DS3_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DATA_NOT_IN_FIFO      = 0x00,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_NO_DECIMATION     = 0x01,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_2     = 0x02,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_3     = 0x03,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_4     = 0x04,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_8     = 0x05,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_16      = 0x06,
  LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_DECIMATION_BY_32      = 0x07,
} LSM6DS3_ACC_GYRO_DEC_DS3_FIFO_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_DS4_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DATA_NOT_IN_FIFO      = 0x00,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_NO_DECIMATION     = 0x08,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_2     = 0x10,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_3     = 0x18,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_4     = 0x20,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_8     = 0x28,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_16      = 0x30,
  LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_DECIMATION_BY_32      = 0x38,
} LSM6DS3_ACC_GYRO_DEC_DS4_FIFO_t;

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: HI_DATA_ONLY
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_HI_DATA_ONLY_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_HI_DATA_ONLY_ENABLED      = 0x40,
} LSM6DS3_ACC_GYRO_HI_DATA_ONLY_t;

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: FIFO_MODE
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS			= 0x00,
  LSM6DS3_ACC_GYRO_FIFO_MODE_FIFO			= 0x01,
  LSM6DS3_ACC_GYRO_FIFO_MODE_CONT			= 0x06,
  LSM6DS3_ACC_GYRO_FIFO_MODE_CONT_FIFO		= 0x03,
  LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS_CONT	= 0x04,
} LSM6DS3_ACC_GYRO_FIFO_MODE_t;

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: ODR_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_ODR_FIFO_DISABLE  = 0x00,
  LSM6DS3_ACC_GYRO_ODR_FIFO_13Hz     = 0x08,
  LSM6DS3_ACC_GYRO_ODR_FIFO_26Hz     = 0x10,
  LSM6DS3_ACC_GYRO_ODR_FIFO_52Hz     = 0x18,
  LSM6DS3_ACC_GYRO_ODR_FIFO_104Hz      = 0x20,
  LSM6DS3_ACC_GYRO_ODR_FIFO_208Hz      = 0x28,
  LSM6DS3_ACC_GYRO_ODR_FIFO_416Hz      = 0x30,
  LSM6DS3_ACC_GYRO_ODR_FIFO_833Hz      = 0x38,
  LSM6DS3_ACC_GYRO_ODR_FIFO_1660Hz     = 0x40,
  LSM6DS3_ACC_GYRO_ODR_FIFO_3330Hz     = 0x48,
  LSM6DS3_ACC_GYRO_ODR_FIFO_6660Hz     = 0x50,
} LSM6DS3_ACC_GYRO_ODR_FIFO_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_DRDY_XL_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_INT1_DRDY_XL_ENABLED      = 0x01,
} LSM6DS3_ACC_GYRO_INT1_DRDY_XL_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_DRDY_G_DISABLED      = 0x00,
  LSM6DS3_ACC_GYRO_INT1_DRDY_G_ENABLED     = 0x02,
} LSM6DS3_ACC_GYRO_INT1_DRDY_G_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_BOOT_DISABLED      = 0x00,
  LSM6DS3_ACC_GYRO_INT1_BOOT_ENABLED     = 0x04,
} LSM6DS3_ACC_GYRO_INT1_BOOT_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FTH
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_FTH_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_INT1_FTH_ENABLED      = 0x08,
} LSM6DS3_ACC_GYRO_INT1_FTH_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_OVR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_OVR_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_INT1_OVR_ENABLED      = 0x10,
} LSM6DS3_ACC_GYRO_INT1_OVR_t;

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FSS5
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_INT1_FULL_DISABLED      = 0x00,
  LSM6DS3_ACC_GYRO_INT1_FULL_ENABLED     = 0x20,
} LSM6DS3_ACC_GYRO_INT1_FULL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN     = 0x00,
  LSM6DS3_ACC_GYRO_ODR_XL_13Hz     = 0x10,
  LSM6DS3_ACC_GYRO_ODR_XL_26Hz     = 0x20,
  LSM6DS3_ACC_GYRO_ODR_XL_52Hz     = 0x30,
  LSM6DS3_ACC_GYRO_ODR_XL_104Hz      = 0x40,
  LSM6DS3_ACC_GYRO_ODR_XL_208Hz      = 0x50,
  LSM6DS3_ACC_GYRO_ODR_XL_416Hz      = 0x60,
  LSM6DS3_ACC_GYRO_ODR_XL_833Hz      = 0x70,
  LSM6DS3_ACC_GYRO_ODR_XL_1660Hz     = 0x80,
  LSM6DS3_ACC_GYRO_ODR_XL_3330Hz     = 0x90,
  LSM6DS3_ACC_GYRO_ODR_XL_6660Hz     = 0xA0
} LSM6DS3_ACC_GYRO_ODR_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_FS_XL_2g      = 0x00,
  LSM6DS3_ACC_GYRO_FS_XL_16g     = 0x04,
  LSM6DS3_ACC_GYRO_FS_XL_4g      = 0x08,
  LSM6DS3_ACC_GYRO_FS_XL_8g      = 0x0C,
} LSM6DS3_ACC_GYRO_FS_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_BW_XL_DEFAULT   = 0x00,
  LSM6DS3_ACC_GYRO_BW_XL_400Hz     = 0x00,
  LSM6DS3_ACC_GYRO_BW_XL_200Hz     = 0x01,
  LSM6DS3_ACC_GYRO_BW_XL_100Hz     = 0x02,
  LSM6DS3_ACC_GYRO_BW_XL_50Hz      = 0x03,
} LSM6DS3_ACC_GYRO_BW_XL_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN      = 0x00,
  LSM6DS3_ACC_GYRO_ODR_G_13Hz      = 0x10,
  LSM6DS3_ACC_GYRO_ODR_G_26Hz      = 0x20,
  LSM6DS3_ACC_GYRO_ODR_G_52Hz      = 0x30,
  LSM6DS3_ACC_GYRO_ODR_G_104Hz     = 0x40,
  LSM6DS3_ACC_GYRO_ODR_G_208Hz     = 0x50,
  LSM6DS3_ACC_GYRO_ODR_G_416Hz     = 0x60,
  LSM6DS3_ACC_GYRO_ODR_G_833Hz     = 0x70,
  LSM6DS3_ACC_GYRO_ODR_G_1660Hz      = 0x80,
} LSM6DS3_ACC_GYRO_ODR_G_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_FS_G_125dps     = 0x02,
  LSM6DS3_ACC_GYRO_FS_G_250dps     = 0x00,
  LSM6DS3_ACC_GYRO_FS_G_500dps     = 0x04,
  LSM6DS3_ACC_GYRO_FS_G_1000dps      = 0x08,
  LSM6DS3_ACC_GYRO_FS_G_2000dps      = 0x0C,
} LSM6DS3_ACC_GYRO_FS_G_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_SW_RESET_NORMAL_MODE      = 0x00,
  LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE     = 0x01,
} LSM6DS3_ACC_GYRO_SW_RESET_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: IF_INC
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_IF_INC_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_IF_INC_ENABLED      = 0x04,
} LSM6DS3_ACC_GYRO_IF_INC_t;

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_BDU_CONTINUOS 	   = 0x00,
  LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE    = 0x40,
} LSM6DS3_ACC_GYRO_BDU_t;

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: XEN_XL | Yen_XL | Xen_XL
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_XYZ_XL_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_XYZ_XL_ENABLED      = 0x38,
} LSM6DS3_ACC_GYRO_XEN_XL_t;

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: XEN_G | YEN_G | ZEN_G
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_XYZ_G_DISABLED    = 0x00,
  LSM6DS3_ACC_GYRO_XYZ_G_ENABLED     = 0x38,
} LSM6DS3_ACC_GYRO_XEN_G_t;

/*******************************************************************************
* Register      : TAP_CFG1
* Address       : 0X58
* Bit Group Name: TIMER_EN
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_TIMER_EN_DISABLED     = 0x00,
  LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED      = 0x80,
} LSM6DS3_ACC_GYRO_TIMER_EN_t;

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: TIMER_HR
* Permission    : RW
*******************************************************************************/
typedef enum
{
  LSM6DS3_ACC_GYRO_TIMER_HR_6_4ms      = 0x00,
  LSM6DS3_ACC_GYRO_TIMER_HR_25us     = 0x10,
} LSM6DS3_ACC_GYRO_TIMER_HR_t;

#endif