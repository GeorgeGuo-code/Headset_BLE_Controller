#ifndef BLE_CONSOLE_H_
#define BLE_CONSOLE_H_

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ble_console — a tiny Nordic-UART-Service (NUS) style BLE GATT server used as
 * a wireless debug console for the head-gesture device.
 *
 *   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (Write / Write-No-Response, central -> device) : 6E400002-...
 *       Carries ASCII command lines: "c", "ca", "ct", "p", "sp", "sr".
 *   TX (Notify, device -> central) : 6E400003-...
 *       Streams UTF-8 log text (chunked to the negotiated MTU-3).
 *
 * Only a single central connection is supported at a time. The device
 * advertises as "HMBC-Console".
 */

/**
 * @brief  Callback invoked when the central writes a command to the RX
 *         characteristic. Runs in the BLE stack task context, so it MUST NOT
 *         block or run long work — post to a queue and return.
 *
 * @param cmd  Pointer to the received bytes (NUL-terminated for convenience;
 *             a trailing '\n'/'\r' is stripped before the callback).
 * @param len  Number of bytes in @p cmd (excluding the added NUL).
 */
typedef void (*ble_console_cmd_cb_t)(const char *cmd, size_t len);

/**
 * @brief  Bring up the BT controller + Bluedroid + GAP + the console GATT
 *         service and start advertising. Call once at boot, after NVS init.
 *
 * @param on_cmd  Command callback (may be NULL to ignore RX writes).
 * @return ESP_OK on success, or the first failing esp_* error.
 */
esp_err_t ble_console_init(ble_console_cmd_cb_t on_cmd);

/**
 * @brief  Queue a log line for transmission over the TX notify characteristic
 *         (and mirror it to the UART console). Non-blocking; safe to call from
 *         any task. Dropped silently if the internal buffer is full.
 */
void ble_console_log(const char *s);

/** @brief printf-style variant of ble_console_log(). */
void ble_console_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** @brief True while a central is connected AND has subscribed to TX notify. */
bool ble_console_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONSOLE_H_ */
