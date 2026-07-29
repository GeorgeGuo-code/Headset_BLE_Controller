#ifndef HID_OUTPUT_H_
#define HID_OUTPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * hid_output — the "execution layer" of the gesture→action pipeline.
 *
 * Owns the BLE HID profile lifecycle (components/ble_hid) and turns abstract
 * requests ("press Win+A", "send play/pause") into timed HID report pairs.
 *
 * Every send is press → wait hold_ms → release, which has to happen off the
 * caller's task: the gesture engine must never block for tens of milliseconds
 * per keystroke. So all sends are queued to a private worker task.
 *
 * BOOT ORDER
 *     hid_output_init();               // registers app_id 0x1812
 *     ble_console_init(on_cmd);        // registers app_id 0x0055
 *     ble_stack_start("HMBC-Console"); // powers the radio
 *
 * Reports are silently dropped (with a log line) until the link is both
 * connected AND encrypted — the host rejects HID reports on a plaintext link,
 * so queueing them would just build a backlog of writes that go nowhere.
 */

/** Default press duration when a caller passes hold_ms == 0. Long enough for
 *  Windows/macOS to register a discrete keypress, short enough not to repeat. */
#define HID_OUTPUT_DEFAULT_HOLD_MS 30

/**
 * @brief  Register the HID profile with ble_stack and start the report worker
 *         task. Call BEFORE ble_stack_start().
 */
esp_err_t hid_output_init(void);

/** @brief True when a central is connected and the link is encrypted, i.e.
 *         when reports will actually be delivered. */
bool hid_output_is_ready(void);

/** @brief conn_id of the HID link, or 0xFFFF when not connected. Diagnostics. */
uint16_t hid_output_conn_id(void);

/**
 * @brief  Queue a keyboard chord: hold `modifiers` + up to 4 keycodes for
 *         `hold_ms`, then release everything.
 *
 * @param modifiers  key_mask_t bitmask (LEFT_CONTROL_KEY_MASK, LEFT_GUI_KEY_MASK…)
 * @param keys       up to 4 HID_KEY_* codes; trailing zeros are ignored. May be
 *                   NULL for a modifiers-only chord.
 * @param hold_ms    press duration; 0 → HID_OUTPUT_DEFAULT_HOLD_MS.
 *
 * @return ESP_OK if queued, ESP_ERR_INVALID_STATE if the link is not ready,
 *         ESP_ERR_NO_MEM if the worker queue is full.
 */
esp_err_t hid_output_send_keyboard(uint8_t modifiers, const uint8_t keys[4],
                                   uint16_t hold_ms);

/**
 * @brief  Queue a consumer-control (media key) press+release.
 * @param code  HID_CONSUMER_* value, e.g. HID_CONSUMER_PLAY_PAUSE (205).
 */
esp_err_t hid_output_send_consumer(uint8_t code, uint16_t hold_ms);

/**
 * @brief  Queue one mouse report (buttons + relative motion). Fire-and-forget:
 *         a follow-up all-zero report is sent to release the buttons.
 */
esp_err_t hid_output_send_mouse(uint8_t buttons, int8_t dx, int8_t dy);

#ifdef __cplusplus
}
#endif

#endif /* HID_OUTPUT_H_ */
