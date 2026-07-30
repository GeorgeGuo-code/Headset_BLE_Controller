#ifndef BLE_STACK_H_
#define BLE_STACK_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ble_stack — sole owner of the BLE controller + Bluedroid bring-up, of the
 * single global GATTS/GAP callback pair, and of the merged advertising payload.
 *
 * WHY THIS EXISTS
 * ---------------
 * Bluedroid keeps exactly ONE global GATTS callback and ONE global GAP
 * callback. `esp_ble_gatts_register_callback()` overwrites whatever was
 * registered before it. Before this component existed, both
 * `ble_console_init()` and `hidd_register_cb()` (inside components/ble_hid)
 * called it — so bringing up HID silently killed the NUS console, and vice
 * versa.
 *
 * The fix is the standard ESP-IDF "profile table" pattern (see
 * reference/ble_hidd_demo_main.c): one global callback owned here, a table of
 * per-profile callbacks, dispatch by `gatts_if`.
 *
 * DIFFERENCE FROM THE IDF DEMO
 * ----------------------------
 * The demo captures `gatts_if` on ESP_GATTS_REG_EVT by *table index*, which
 * only works when exactly one app_id is ever registered. We register three
 * (console 0x0055, HID 0x1812, battery 0x180F), so the REG_EVT is claimed by
 * matching `param->reg.app_id` against the slot's app_id instead. Copying the
 * demo verbatim would make the first REG_EVT overwrite every slot's gatts_if.
 *
 * USAGE
 * -----
 *     ble_console_init(on_cmd);            // registers app_id 0x0055
 *     hid_output_init();                   // registers app_id 0x1812 + 0x180F
 *     ble_stack_start("HMBC-Console");     // brings the radio up, starts adv
 *
 * All ble_stack_register_profile() calls MUST happen before ble_stack_start().
 */

#define BLE_STACK_MAX_PROFILES 4

/** Per-profile GATTS callback. Only receives events for its own gatts_if
 *  (plus broadcast events where gatts_if == ESP_GATT_IF_NONE). */
typedef void (*ble_profile_gatts_cb_t)(esp_gatts_cb_event_t event,
                                       esp_gatt_if_t gatts_if,
                                       esp_ble_gatts_cb_param_t *param);

/** Per-profile GAP callback. Receives *every* GAP event — GAP events are not
 *  tagged with a gatts_if, so there is nothing to filter on. May be NULL. */
typedef void (*ble_profile_gap_cb_t)(esp_gap_ble_cb_event_t event,
                                     esp_ble_gap_cb_param_t *param);

/**
 * @brief  Claim a slot in the profile table. Call before ble_stack_start().
 *
 * @param app_id    GATTS application id; also the key the REG_EVT is matched
 *                  against, so it must be unique across profiles.
 * @param gatts_cb  Per-profile GATTS handler (may be NULL).
 * @param gap_cb    Per-profile GAP handler (may be NULL).
 *
 * @return ESP_OK, ESP_ERR_NO_MEM if the table is full,
 *         ESP_ERR_INVALID_STATE if the stack is already started,
 *         ESP_ERR_INVALID_ARG on a duplicate app_id.
 */
esp_err_t ble_stack_register_profile(uint16_t app_id,
                                     ble_profile_gatts_cb_t gatts_cb,
                                     ble_profile_gap_cb_t gap_cb);

/**
 * @brief  Bring up controller -> Bluedroid -> global callbacks -> security
 *         params -> per-profile app_register -> advertising.
 *
 *         Advertising payload is merged for all profiles:
 *           ADV      : flags + appearance(HID Generic) + UUID16 0x1812 + name
 *           SCAN RSP : the 128-bit NUS service UUID
 *
 * @param dev_name  GAP device name. Keep the "HMBC" prefix — the Electron
 *                  config tool filters on it. Watch the 31-byte ADV budget if
 *                  you lengthen it.
 */
esp_err_t ble_stack_start(const char *dev_name);

/** @brief True once ESP_GAP_BLE_AUTH_CMPL_EVT reported a successful pairing.
 *         HID reports are only accepted by the host on an encrypted link, so
 *         hid_output gates on this. */
bool ble_stack_is_bonded(void);

/** @brief True between ESP_GATTS_CONNECT_EVT and ESP_GATTS_DISCONNECT_EVT. */
bool ble_stack_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_STACK_H_ */
