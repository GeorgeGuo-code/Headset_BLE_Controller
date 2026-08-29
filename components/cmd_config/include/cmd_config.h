#ifndef CMD_CONFIG_H_
#define CMD_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "hid_output.h"  /* for hid_seq_step_t, HID_SEQ_MAX_STEPS */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cmd_config — persistent named command configurations.
 *
 * Each config maps a trigger (gesture or serial-command number) to a
 * multi-step HID sequence. Configs are stored in NVS so they survive
 * reboots, and can be synced from the PC configuration tool over BLE.
 *
 * STORAGE
 *   NVS namespace "cmds":
 *     "count"  — uint8_t, number of active configs (0..CMD_CFG_MAX)
 *     "cfg_N"  — blob of cmd_config_t for each id N
 *
 * USAGE
 *   1. Call cmd_config_init() once after nvs_flash_init().
 *   2. Use cmd_config_set() to store configs (from BLE `cmd set` command).
 *   3. Use cmd_config_execute() or cmd_config_execute_by_trigger() to run.
 *   4. Use cmd_config_list() / cmd_config_get() for diagnostics.
 */

#define CMD_CFG_NAME_MAX  32
#define CMD_CFG_MAX       16

/** Trigger types — what causes a config to fire. */
typedef enum {
    TRIGGER_NONE    = 0,    /*!< manual only (via `command N` or `cmd run N`) */
    TRIGGER_GESTURE = 1,    /*!< fires when a matching gesture bitmask is detected */
#ifdef ENABLE_SERIAL_TRIGGER
    TRIGGER_COMMAND = 2,    /*!< fires on `command N` (debug build only) */
#endif
} cmd_trigger_type_t;

/** One stored command configuration. */
typedef struct {
    uint8_t             id;                              /*!< 1–CMD_CFG_MAX; 0 = unused slot */
    char                name[CMD_CFG_NAME_MAX];          /*!< display name (UTF-8, NUL-term) */
    cmd_trigger_type_t  trigger_type;
    uint16_t            trigger_value;                   /*!< primary trigger bitmask (gesture bits) */
    uint16_t            fallback_value;                  /*!< fallback trigger bitmask — fires when gesture
                                                             confidence < 0.7 (detector unsure, likely
                                                             misclassification).  Set to same bitmask as
                                                             trigger_value for "always fire on these" or
                                                             add neighbor gestures for fuzzy matching. */
    uint16_t            cooldown_ms;                     /*!< per-config cooldown (0 = use global default).
                                                             Prevents cascade re-triggering from a single
                                                             gesture event.  Recommend 800–2000 ms. */
    uint8_t             min_confidence;                  /*!< minimum confidence 0–100 to trigger (0 = any).
                                                             Values above 0 filter out low-confidence
                                                             misclassifications.  30 = lenient, 70 = strict. */
    uint8_t             n_steps;
    hid_seq_step_t      steps[HID_SEQ_MAX_STEPS];
} cmd_config_t;

/**
 * @brief  Load all configs from NVS into memory. Call once at boot.
 */
esp_err_t cmd_config_init(void);

/**
 * @brief  Get a pointer to an in-memory config slot.
 * @param  id  1..CMD_CFG_MAX
 * @return Pointer to the config, or NULL if the slot is empty or id invalid.
 */
const cmd_config_t *cmd_config_get(uint8_t id);

/**
 * @brief  Store a config (writes to NVS + updates in-memory slot).
 * @param  cfg  config to store; cfg->id must be 1..CMD_CFG_MAX
 */
esp_err_t cmd_config_set(const cmd_config_t *cfg);

/**
 * @brief  Update only the fuzzy-matching parameters of an existing config.
 *         Does not touch trigger_type, trigger_value, steps, or name.
 *         Writes to NVS.
 *
 * @param  id              1..CMD_CFG_MAX
 * @param  fallback_value  fallback trigger bitmask (0 = no fallback)
 * @param  cooldown_ms     per-config cooldown in ms (0 = use global default)
 * @param  min_confidence  minimum confidence 0–100 (0 = any)
 */
esp_err_t cmd_config_set_fuzzy(uint8_t id, uint16_t fallback_value,
                               uint16_t cooldown_ms, uint8_t min_confidence);

/**
 * @brief  Delete a config from NVS and clear the in-memory slot.
 * @param  id  1..CMD_CFG_MAX
 */
esp_err_t cmd_config_delete(uint8_t id);

/**
 * @brief  Execute a config's seq immediately (calls hid_output_send_seq).
 * @param  id  1..CMD_CFG_MAX
 * @return ESP_OK on enqueue, ESP_ERR_NOT_FOUND if slot empty, etc.
 */
esp_err_t cmd_config_execute(uint8_t id);

/**
 * @brief  Find and execute all configs matching a trigger type+value.
 *         Called from gesture_bridge_task when a gesture fires.
 *
 *         Multi-confidence matching:
 *         For each config, check if ANY gesture in trigger_value has
 *         conf[i] >= min_confidence (per-gesture threshold).  The config
 *         fires if at least one gesture passes.  This replaces the old
 *         single-confidence fuzzy fallback with a cleaner model:
 *         "any gesture above the threshold is a valid trigger."
 *
 * @param  type       TRIGGER_GESTURE or TRIGGER_COMMAND
 * @param  value      gesture_type_t value or command number (primary)
 * @param  conf       array of 4 confidence values [NOD, LOOK_UP, TILT_L, TILT_R]
 *                    Pass NULL for command triggers (treated as conf=1.0).
 */
void cmd_config_execute_by_trigger(cmd_trigger_type_t type, uint16_t value,
                                   const float conf[4]);

/**
 * @brief  Format all stored configs as text lines for BLE console output.
 *
 *         Writes lines like:
 *           cfg: id=1 name="Notepad" trigger=command:1 n_steps=4
 *           cfg: count=2
 *
 * @param  out       output buffer
 * @param  out_size  buffer size in bytes
 * @return ESP_OK or ESP_ERR_NO_MEM if buffer too small.
 */
esp_err_t cmd_config_list(char *out, size_t out_size);

/**
 * @brief  Format a single config's seq steps as semicolon-delimited text.
 *
 *         Example: "key 0 4; sleep 350; type notepad; key 0 40"
 *
 * @param  cfg       config to format
 * @param  out       output buffer
 * @param  out_size  buffer size in bytes
 * @return ESP_OK or ESP_ERR_NO_MEM.
 */
esp_err_t cmd_config_format_seq(const cmd_config_t *cfg, char *out, size_t out_size);

/**
 * @brief  Parse a semicolon-delimited seq text into a step array.
 *         Reuses the same syntax as the `seq` console command.
 *
 * @param  text     seq text (e.g. "key 0 4; sleep 350; type hello")
 * @param  steps    output array (must hold HID_SEQ_MAX_STEPS)
 * @param  n_out    receives the number of parsed steps
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on parse error.
 */
esp_err_t cmd_config_parse_seq(const char *text, hid_seq_step_t *steps, size_t *n_out);

#ifdef __cplusplus
}
#endif

#endif /* CMD_CONFIG_H_ */
