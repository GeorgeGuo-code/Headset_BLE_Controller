/**
 * @file battery_quantity_detection.h
 * @brief 电池电量检测模块
 *
 * 基于ADC的电池电量检测，支持充电状态检测和3级LED显示。
 * 使用滑动平均滤波和去抖动算法。
 *
 * @author George Guo
 * @date 2026-02-28
 * @version 1.0.0
 */

#ifndef BATTERY_QUANTITY_DETECTION_H
#define BATTERY_QUANTITY_DETECTION_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 硬件引脚定义
 *============================================================================*/

/** @brief ADC采集引脚（电池电压/2） */
#define BAT_ADC_UNIT        ADC_UNIT_1
#define BAT_ADC_CHANNEL     ADC_CHANNEL_0    // GPIO1
#define BAT_ADC_BITWIDTH    ADC_BITWIDTH_12
#define BAT_ADC_ATTEN       ADC_ATTEN_DB_12  // 衰减 12dB

/** @brief 指示灯引脚 */
#define LED_HIGH_PIN        GPIO_NUM_9       // 高电量指示灯
#define LED_MEDIUM_PIN      GPIO_NUM_8       // 中电量指示灯
#define LED_LOW_PIN         GPIO_NUM_7       // 低电量指示灯

/** @brief 电量显示等级数 */
#define BAT_QUANTITY_MAX_LEVEL 3

/** @brief ADC采集平均次数 */
#define BAT_QUANTITY_SAMPLETimes 15

/*============================================================================
 * 充电状态枚举
 *============================================================================*/

/**
 * @brief 充电状态枚举
 */
typedef enum {
    BAT_CHARGE_STATE_NOT_CHARGE = 0,  // 未充电
    BAT_CHARGE_STATE_CHARGING = 1,    // 充电中
    BAT_CHARGE_STATE_UNKNOWN = 2,     // 未知状态
} bat_charge_state_t;

/*============================================================================
 * API 函数
 *============================================================================*/

/**
 * @brief 初始化电池电量检测模块
 *
 * 配置ADC通道、校准参数和GPIO输出。
 * LED在所有初始化完成后才点亮。
 *
 * @return 0 成功，-1 失败
 */
int bat_quantity_detection_init(void);

/**
 * @brief 获取当前电量等级
 *
 * @return 电量等级 (1-3)，0表示未检测到
 */
int bat_quantity_detection_get_level(void);

/**
 * @brief 获取当前充电状态
 *
 * @return 充电状态枚举值
 */
bat_charge_state_t bat_quantity_detection_get_charge_state(void);

/**
 * @brief 获取电池电压（mV）
 *
 * @return 电池电压值，单位mV
 */
int bat_quantity_detection_get_voltage_mv(void);

#ifdef __cplusplus
}
#endif

#endif // BATTERY_QUANTITY_DETECTION_H
