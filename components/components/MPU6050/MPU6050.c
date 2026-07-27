#include "MPU6050.h"
#include <string.h>

#define SCL 6    /*!< I2C SCL pin number  */
#define SDA 5    /*!< I2C SDA pin number  */
#define I2C_TIMEOUT_MS  10    /*!< per-transfer timeout, matches legacy value */

/* New I2C master driver keeps the bus and the (single) MPU6050 device as
 * module-level handles so the read/write helpers don't have to plumb them
 * through every call site. They are initialised once in I2C_Init(). */
static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_mpu_dev  = NULL;

/**
 * @brief I2C initial
 *
 * Idempotent: safe to call multiple times. Repeated calls (e.g. via
 * MPU_Init -> I2C_Init from both app_main and mpu_dmp_init) skip the
 * bus/device setup if the handles are already live, instead of tripping
 * `i2c_new_master_bus`'s "bus already acquired" guard.
 */
void I2C_Init()
{
    if (s_mpu_dev != NULL) {
        return;     /*!< already initialised */
    }

    if (s_i2c_bus == NULL) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port          = I2C_NUM_0,
            .sda_io_num        = SDA,
            .scl_io_num        = SCL,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = {
                .enable_internal_pullup = 1,   /*!< keep the legacy internal-pullup behaviour */
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR,      /*!< 7-bit address; the driver appends R/W itself */
        .scl_speed_hz    = 40000,         /*!< 40 kHz, same as legacy config */
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_mpu_dev));
}

/**
 * @brief MPU-6050 initial
 */
uint8_t MPU_Init()
{
	uint8_t res = 0;
	I2C_Init();
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);
  vTaskDelay(pdMS_TO_TICKS(100));
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);
	MPU_Set_Gyro_FSR(3);
	MPU_Set_Accel_FSR(0);
	MPU_Set_Rate(50);
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	MPU_Read_Byte(MPU_DEVICE_ID_REG, &res);
	if(res == MPU_ADDR)
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
	}else
		return 1;
	return 0;
}

/**
 * @brief Set the Gyroscope full-scale range of ±250, ±500, ±1000, and ±2000°/sec (dps)
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Gyro_FSR(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief Set the Accelerometer full-scale range of ±2g, ±4g, ±8g, and ±16g
 *
 * @param fsr the number of register, it could be 0, 1, 2, 3
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Accel_FSR(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @brief Set the band of low pass filter
 *
 * @param lps parameter is the band of low pass filter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**
 * @brief Set the Sample rate of Gyroscope, Accelerometer, DMP, etc.
 *
 * @param rate parameter is the sample rate of Gyroscope, Accelerometer, DMP, etc.
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate > 1000)
		rate = 1000;
	if(rate < 4)
		rate = 4;
	data = 1000/rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
	return MPU_Set_LPF(rate / 2); /*!< set low pass filter the half of the rate */
}

/**
 * @brief Get the temperature of the MPU-6050
 *
 * @return
 *     - temp is the temperature of the MPU-6050
 *     - 1 is Error
 */
int16_t MPU_Get_Temperature()
{
	uint8_t buf[2];
	int16_t raw;
	float temp;
	if(MPU_Read_Len(MPU_TEMP_OUTH_REG, buf, 2) == 0)
		return 1;
	raw = ((uint16_t)(buf[1] << 8)) | buf[0];
	temp = 36.53 + ((double)raw/340);
	return temp*100;
}

/**
 * @brief Get the Gyroscope data of the MPU-6050
 *
 * @param gx parameter is the x axis data of Gyroscope
 * @param gy parameter is the y axis data of Gyroscope
 * @param gz parameter is the z axis data of Gyroscope
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_GYRO_XOUTH_REG, buf, 6);
	if(res == 0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];
		*gy=((uint16_t)buf[2]<<8)|buf[3];
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	}
	return res;
}

/**
 * @brief Get the Accelerometer data of the MPU-6050
 *
 * @param ax parameter is the x axis data of Accelerometer
 * @param ay parameter is the y axis data of Accelerometer
 * @param az parameter is the z axis data of Accelerometer
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Get_Accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buf[6], res;
	res = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, buf, 6);
	if(res == 0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];
		*ay=((uint16_t)buf[2]<<8)|buf[3];
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
	return res;
}

/**
 * @brief Write a byte to MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param data parameter will be written to the register of MPU-6050
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = { reg, data };
	return (i2c_master_transmit(s_mpu_dev, buf, sizeof(buf), I2C_TIMEOUT_MS) == ESP_OK) ? 0 : 1;
}

/**
 * @brief Write a buffer to MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param data parameter is a buffer which will be written to a register of MPU-6050
 * @param len parameter is the length of data
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Write_Len(uint8_t reg, uint8_t *data, uint8_t len)
{
	/* The new driver has no "write reg then N bytes" helper, so concatenate
	 * the register address and payload into one scratch buffer. 32 B is
	 * enough for every MPU6050 register write in this project (DMP firmware
	 * loads are funneled through inv_mpu.c, which uses this same helper). */
	uint8_t buf[32];
	if (len == 0 || (size_t)len + 1 > sizeof(buf))
		return 1;
	buf[0] = reg;
	memcpy(&buf[1], data, len);
	return (i2c_master_transmit(s_mpu_dev, buf, (size_t)len + 1, I2C_TIMEOUT_MS) == ESP_OK) ? 0 : 1;
}

/**
 * @brief Read a byte from MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param res the data read will be stored in this parameter
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Byte(uint8_t reg, uint8_t *res)
{
	return (i2c_master_transmit_receive(s_mpu_dev, &reg, 1, res, 1, I2C_TIMEOUT_MS) == ESP_OK) ? 0 : 1;
}

/**
 * @brief Read a buffer from MPU-6050 through I2C
 *
 * @param reg parameter is a register of MPU-6050
 * @param buf parameter is a buf witch will store the data
 * @param len parameter is the length of buf
 *
 * @return
 *     - 0 is Success
 *     - 1 is Error
 */
uint8_t MPU_Read_Len(uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (len == 0)
		return 1;
	return (i2c_master_transmit_receive(s_mpu_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS) == ESP_OK) ? 0 : 1;
}
