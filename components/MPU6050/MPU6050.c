#include "MPU6050.h"
#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us — used by the bus-recovery bit-bang */

#define TAG "MPU6050"

#define SCL 6    /*!< I2C SCL pin number  */
#define SDA 5    /*!< I2C SDA pin number  */
/* Per-transfer timeout. The new i2c_master driver queues a transaction and
 * waits on a semaphore, so this budget covers scheduling delay on top of wire
 * time (a 2-byte write at 40 kHz is ~0.5 ms). Raised from the legacy 10 ms so
 * a busy moment can't be mistaken for a dead sensor; generous here costs
 * nothing on the success path. Note this does NOT fix a hung bus — see
 * i2c_bus_recover() for that. */
#define I2C_TIMEOUT_MS  1000

/* New I2C master driver keeps the bus and the (single) MPU6050 device as
 * module-level handles so the read/write helpers don't have to plumb them
 * through every call site. They are initialised once in I2C_Init(). */
static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_mpu_dev  = NULL;

/**
 * @brief Free a hung I2C bus by bit-banging SCL until the slave lets SDA go.
 *
 * A soft reset (or a reflash) while the MPU is mid-transfer leaves the slave
 * still driving SDA low, waiting for the clocks that finish the byte it was
 * sending. The ESP32 is then stuck: SDA never returns high, so the master sees
 * a permanently busy bus and every transfer reports "I2C hardware timeout".
 * The sensor is fine — it just needs the rest of its clock pulses.
 *
 * The fix is the standard recovery sequence, done with plain GPIO before the
 * I2C driver claims the pins: up to 9 SCL pulses (one byte + ACK) to flush the
 * stuck byte out, then a STOP condition to resynchronise both sides.
 */
static void i2c_bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SDA) | (1ULL << SCL),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,   /*!< open-drain: never fight the slave */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        return;
    }
    gpio_set_level(SDA, 1);
    gpio_set_level(SCL, 1);
    esp_rom_delay_us(10);

    if (gpio_get_level(SDA) == 1) {
        return;         /*!< bus already idle, nothing to recover */
    }

    ESP_LOGW(TAG, "SDA held low — clocking the bus free");
    for (int i = 0; i < 9 && gpio_get_level(SDA) == 0; i++) {
        gpio_set_level(SCL, 0);
        esp_rom_delay_us(10);   /*!< ~50 kHz, comfortably inside spec */
        gpio_set_level(SCL, 1);
        esp_rom_delay_us(10);
    }

    /* STOP condition: SDA must go low while SCL is low, then rise while SCL is
     * high. The loop above leaves SCL high, so drop it first — driving SDA low
     * on a high clock would emit a START instead. */
    gpio_set_level(SCL, 0);
    esp_rom_delay_us(10);
    gpio_set_level(SDA, 0);
    esp_rom_delay_us(10);
    gpio_set_level(SCL, 1);
    esp_rom_delay_us(10);
    gpio_set_level(SDA, 1);
    esp_rom_delay_us(10);

    ESP_LOGW(TAG, "bus recovery done, SDA=%d SCL=%d",
             gpio_get_level(SDA), gpio_get_level(SCL));

    /* Hand the pins back so i2c_new_master_bus can reconfigure them. */
    gpio_reset_pin(SDA);
    gpio_reset_pin(SCL);
}

void I2C_Init()
{
    if (s_mpu_dev != NULL) {
        return;     /*!< already initialised */
    }

    if (s_i2c_bus == NULL) {
        i2c_bus_recover();
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
 * @brief Probe the I2C bus and log every address that ACKs.
 *
 * Diagnostic aid for the "MPU_Init returns 1" case: it separates "nothing on
 * the bus at all" (wiring / power / wrong pins) from "something answers, but
 * not at 0x68" (AD0 pulled high -> 0x69) from "right address, wrong WHO_AM_I"
 * (an MPU6500/9250 clone module).
 */
void MPU_Bus_Scan(void)
{
    I2C_Init();
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "i2c device found at 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "i2c scan: no devices on SDA=%d SCL=%d — check wiring, "
                      "3V3/GND and pull-ups", SDA, SCL);
    }
}

/**
 * @brief MPU-6050 initial
 *
 * @return 0 on success, 1 when the WHO_AM_I check fails (see the logs for the
 *         value actually read).
 */
uint8_t MPU_Init()
{
	uint8_t res = 0;
	I2C_Init();

	/* A dead bus makes every write below fail silently, and the WHO_AM_I read
	 * then returns a stale 0 — indistinguishable from a wrong-chip answer.
	 * Probe first so the log says which of the two it is. */
	if (i2c_master_probe(s_i2c_bus, MPU_ADDR, 100) != ESP_OK) {
		ESP_LOGE(TAG, "no ACK from 0x%02X on SDA=%d SCL=%d", MPU_ADDR, SDA, SCL);
		MPU_Bus_Scan();
		return 1;
	}

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

	if (MPU_Read_Byte(MPU_DEVICE_ID_REG, &res) != 0) {
		ESP_LOGE(TAG, "WHO_AM_I read failed (device ACKed the probe but not the read)");
		return 1;
	}
	ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x%02X)", res, MPU_ADDR);

	/* MPU6050 reports 0x68. Common drop-in clones report something else:
	 * MPU6500 -> 0x70, MPU9250 -> 0x71, MPU6555 -> 0x7C. The DMP driver in
	 * this project is the 6050 motion driver, so accept only 0x68 but name
	 * the alternative in the log to save a round of guessing. */
	if (res != MPU_ADDR) {
		const char *guess = (res == 0x70) ? " (looks like an MPU6500)"
		                  : (res == 0x71) ? " (looks like an MPU9250)"
		                  : (res == 0x7C) ? " (looks like an MPU6555)"
		                  : (res == 0x00 || res == 0xFF) ? " (bus stuck / no real data)"
		                  : "";
		ESP_LOGE(TAG, "unexpected WHO_AM_I 0x%02X%s", res, guess);
		MPU_Bus_Scan();
		return 1;
	}

	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
	MPU_Set_Rate(50);						//设置采样率为50Hz
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
