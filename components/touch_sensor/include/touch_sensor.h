#ifndef TOUCH_SENSOR_H_
#define TOUCH_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * touch_sensor — single-channel mouse-left-click button.
 *
 * Bridges the ESP32-S3 hardware touch sensor (esp_driver_touch_sens) to the
 * existing BLE HID mouse output. The reference example
 * (reference/touch_sens_basic/) printed raw benchmark data; here we collapse
 * the whole "active/inactive" pair into "left mouse button down / up" so the
 * touch pad acts as a click-and-hold button.
 *
 * BOOT ORDER
 *     hid_output_init();               // brings the BLE HID profile up
 *     ble_stack_start("<device-name>"); // powers the radio
 *     touch_sensor_init(<chan_id>);    // after the link is reachable
 *
 * Why after ble_stack_start(): the press/release worker pokes
 * esp_hidd_send_mouse_value() directly with hid_output_conn_id(), which only
 * makes sense once the link is up. Calling earlier is safe — the worker just
 * drops events when `hid_output_is_ready()` is false.
 *
 * INITIAL SCAN COST
 *     touch_sensor_init() does sync oneshot scans (3 × up to 2 s) to read the
 *     baseline before continuous scanning starts. Boot takes ~6 s longer. The
 *     example does this synchronously and so do we — the only alternative is
 *     deferring real "active/inactive" semantics until the baseline is known,
 *     which complicates the public API for no functional gain.
 *
 * THREAD MODEL
 *     - ISR-ish: on_active / on_inactive run from the touch sensor interrupt,
 *       which means NO blocking or logging at ESP_LOG*. They post a one-byte
 *       command to a small queue and return.
 *     - Worker: a task drains the queue and calls esp_hidd_send_mouse_value()
 *       directly. Only one press (or release) is in flight at a time — the
 *       worker keeps a `pressed` flag so duplicate active events don't
 *       re-send the press, and an inactive event without a matching active
 *       is a no-op.
 */
esp_err_t touch_sensor_init(int chan_id);

#ifdef __cplusplus
}
#endif

#endif /* TOUCH_SENSOR_H_ */
