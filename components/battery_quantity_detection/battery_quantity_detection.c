/**
 * @file battery_quantity_detection.c
 * @brief 电池电量检测模块实现
 *
 * 基于ADC的电池电量检测，支持充电状态检测和3级LED显示。
 * 使用滑动平均滤波和去抖动算法。
 *
 * @author George Guo
 * @date 2026-02-28
 * @version 1.0.0
 */

#include "battery_quantity_detection.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BAT_QUANTITY";

/*============================================================================
 * 常量定义
 *============================================================================*/

#define BAT_QUANTITY_MAX_FILTER_LEN  5     // 最大滤波器长度
#define BAT_QUANTITY_DEBOUNCE_TIME   2000  // 去抖动时间 (ms)

// 充电电压阈值
#define BAT_CHARGING_THRESHOLD_MV    4300  // 充电检测阈值

/*============================================================================
 * 内部状态
 *============================================================================*/

/** @brief ADC oneshot句柄 */
static adc_oneshot_unit_handle_t adc_handle = NULL;

/** @brief ADC校准句柄 */
static adc_cali_handle_t adc_cali_handle = NULL;

/** @brief 电池电量等级缓存 */
static int bat_level_cache = 0;

/** @brief 充电状态 */
static bat_charge_state_t bat_charge_state = BAT_CHARGE_STATE_UNKNOWN;

/** @brief 去抖动计数器 */
static int debounce_counter = 0;

/** @brief 去抖动时间戳 */
static int64_t debounce_timestamp = 0;

/** @brief 滤波器数据 */
static float filter_buffer[BAT_QUANTITY_MAX_FILTER_LEN];
static int filter_index = 0;
static int filter_count = 0;

/** @brief GPIO引脚数组 */
static const gpio_num_t bat_pins[BAT_QUANTITY_MAX_LEVEL] = {
    LED_LOW_PIN,
    LED_MEDIUM_PIN,
    LED_HIGH_PIN
};

/*============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 读取电池电压（mV）
 */
static int read_battery_voltage_mv(void)
{
    int adc_reading = 0;

    // 多次采样取平均
    for (int i = 0; i < BAT_QUANTITY_SAMPLETimes; i++) {
        int raw = 0;
        adc_oneshot_read(adc_handle, BAT_ADC_CHANNEL, &raw);
        adc_reading += raw;
    }
    adc_reading /= BAT_QUANTITY_SAMPLETimes;

    // 校准转换为电压 (mV)，乘以2还原真实电池电压
    int voltage = 0;
    if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage);
    }
    voltage *= 2;

    ESP_LOGI(TAG, "Raw: %d, Voltage: %dmV", adc_reading, voltage);

    return voltage;
}

/**
 * @brief 滑动平均滤波
 */
static float apply_moving_average_filter(float new_value)
{
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % BAT_QUANTITY_MAX_FILTER_LEN;

    if (filter_count < BAT_QUANTITY_MAX_FILTER_LEN) {
        filter_count++;
    }

    float sum = 0;
    for (int i = 0; i < filter_count; i++) {
        sum += filter_buffer[i];
    }

    return sum / filter_count;
}

/**
 * @brief 更新电量等级显示
 */
static void update_battery_level(int level)
{
    // 清除所有指示灯
    for (int i = 0; i < BAT_QUANTITY_MAX_LEVEL; i++) {
        gpio_set_level(bat_pins[i], 0);
    }

    // 只点亮对应等级的指示灯
    if (level > 0 && level <= BAT_QUANTITY_MAX_LEVEL) {
        gpio_set_level(bat_pins[level - 1], 1);
    }

    bat_level_cache = level;
    ESP_LOGI(TAG, "Battery level updated: %d", level);
}

/*============================================================================
 * 公共API实现
 *============================================================================*/

int bat_quantity_detection_init(void)
{
    ESP_LOGI(TAG, "Initializing battery quantity detection...");

    // 配置GPIO输出 (先配置，不点亮)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = (1ULL << LED_HIGH_PIN) |
                       (1ULL << LED_MEDIUM_PIN) |
                       (1ULL << LED_LOW_PIN)
    };
    gpio_config(&io_conf);

    // 初始化所有LED为关闭状态
    for (int i = 0; i < BAT_QUANTITY_MAX_LEVEL; i++) {
        gpio_set_level(bat_pins[i], 0);
    }

    // 配置ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // 配置ADC通道
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = BAT_ADC_BITWIDTH,
        .atten = BAT_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, BAT_ADC_CHANNEL, &channel_config));

    // ADC校准
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Calibration scheme: curve fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BAT_ADC_UNIT,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Calibration scheme: line fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = BAT_ADC_UNIT,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle));
#else
    ESP_LOGW(TAG, "No calibration scheme supported, voltage will be inaccurate");
#endif

    // 初始化滤波器
    memset(filter_buffer, 0, sizeof(filter_buffer));
    filter_index = 0;
    filter_count = 0;

    ESP_LOGI(TAG, "Battery quantity detection initialized successfully");

    // 立即读取一次电量并点亮指示灯
    int voltage_mv = read_battery_voltage_mv();
    float filtered_voltage = apply_moving_average_filter((float)voltage_mv);
    int level = 0;
    if (filtered_voltage >= 3850) {
        level = 3;
    } else if (filtered_voltage >= 3650) {
        level = 2;
    } else {
        level = 1;
    }
    update_battery_level(level);

    return 0;
}

int bat_quantity_detection_get_level(void)
{
    // 读取当前电压
    int voltage_mv = read_battery_voltage_mv();

    // 应用滑动平均滤波
    float filtered_voltage = apply_moving_average_filter((float)voltage_mv);

    ESP_LOGI(TAG, "Filtered voltage: %.1f mV", filtered_voltage);

    // 根据电压确定电量等级 (3级: 低/中/高)
    int level = 0;
    if (filtered_voltage >= 3600) {
        level = 3;  // 高电量
    } else if (filtered_voltage >= 3400) {
        level = 2;  // 中电量
    } else {
        level = 1;  // 低电量
    }

    // 去抖动处理
    int64_t current_time = esp_timer_get_time() / 1000;  // 转换为ms

    if (level != bat_level_cache) {
        if (debounce_counter == 0) {
            debounce_counter++;
            debounce_timestamp = current_time;
        } else if (current_time - debounce_timestamp >= BAT_QUANTITY_DEBOUNCE_TIME) {
            update_battery_level(level);
            debounce_counter = 0;
        }
    } else {
        debounce_counter = 0;
    }

    return bat_level_cache;
}

bat_charge_state_t bat_quantity_detection_get_charge_state(void)
{
    int voltage_mv = read_battery_voltage_mv();

    if (voltage_mv >= BAT_CHARGING_THRESHOLD_MV) {
        bat_charge_state = BAT_CHARGE_STATE_CHARGING;
    } else {
        bat_charge_state = BAT_CHARGE_STATE_NOT_CHARGE;
    }

    return bat_charge_state;
}

int bat_quantity_detection_get_voltage_mv(void)
{
    return read_battery_voltage_mv();
}
