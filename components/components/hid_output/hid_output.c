/*
 * hid_output.c — BLE HID execution layer. See include/hid_output.h.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "esp_hidd_prf_api.h"
#include "hidd_le_prf_int.h"
#include "hid_dev.h"

#include "ble_stack.h"
#include "hid_output.h"

#define TAG "hid_output"

#define HID_OUT_QUEUE_LEN   8
#define HID_OUT_TASK_STACK  3072
#define HID_OUT_TASK_PRIO   5    /* above the gesture engine (4) — a queued
                                  * report should go out promptly, and the
                                  * worker spends its life in vTaskDelay */
#define HID_CONN_ID_NONE    0xFFFF

typedef enum {
    REQ_KEYBOARD,
    REQ_CONSUMER,
    REQ_MOUSE,
} req_kind_t;

typedef struct {
    req_kind_t kind;
    uint8_t    modifiers;
    uint8_t    keys[4];
    int8_t     dx, dy;
    uint16_t   hold_ms;
} hid_req_t;

static QueueHandle_t      s_req_q      = NULL;
static volatile uint16_t  s_conn_id    = HID_CONN_ID_NONE;
static volatile bool      s_connected  = false;

/* ── HID profile event callback ──────────────────────────────────────────── */
static void hidd_event_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH:
        /* The demo configures the device name + advertising here. We do not:
         * ble_stack owns one merged advertising payload for all profiles, and
         * letting HID reconfigure it would drop the NUS UUID from the scan
         * response and break the Electron config tool's device filter. */
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            ESP_LOGI(TAG, "HID profile registered");
        } else {
            ESP_LOGE(TAG, "HID profile registration failed");
        }
        break;

    case ESP_HIDD_EVENT_BLE_CONNECT:
        s_conn_id   = param->connect.conn_id;
        s_connected = true;
        ESP_LOGI(TAG, "HID connected, conn_id = %u", (unsigned)s_conn_id);
        break;

    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        s_connected = false;
        s_conn_id   = HID_CONN_ID_NONE;
        ESP_LOGI(TAG, "HID disconnected");
        /* ble_stack restarts advertising — see its ESP_GATTS_DISCONNECT_EVT
         * handling. Doing it here too would race with the console profile. */
        break;

    default:
        break;
    }
}

/* ── Worker: one request at a time, press → hold → release ───────────────── */
static void hid_worker_task(void *arg)
{
    (void)arg;
    hid_req_t req;

    for (;;) {
        if (xQueueReceive(s_req_q, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        /* Re-check readiness: the link may have dropped while this request sat
         * in the queue. */
        if (!hid_output_is_ready()) {
            ESP_LOGW(TAG, "dropping queued report — link no longer ready");
            continue;
        }

        uint16_t hold = req.hold_ms ? req.hold_ms : HID_OUTPUT_DEFAULT_HOLD_MS;
        uint16_t id   = s_conn_id;

        switch (req.kind) {
        case REQ_KEYBOARD: {
            uint8_t n = 0;
            while (n < 4 && req.keys[n] != 0) {
                n++;
            }
            /* esp_hidd_send_keyboard_value takes a non-const pointer but only
             * reads it; the local copy in `req` keeps that safe. */
            esp_hidd_send_keyboard_value(id, req.modifiers, req.keys, n);
            vTaskDelay(pdMS_TO_TICKS(hold));
            /* Release: no modifiers, no keys. */
            esp_hidd_send_keyboard_value(id, 0, NULL, 0);
            break;
        }

        case REQ_CONSUMER:
            esp_hidd_send_consumer_value(id, req.keys[0], true);
            vTaskDelay(pdMS_TO_TICKS(hold));
            esp_hidd_send_consumer_value(id, req.keys[0], false);
            break;

        case REQ_MOUSE:
            esp_hidd_send_mouse_value(id, req.keys[0], req.dx, req.dy);
            if (req.keys[0] != 0) {
                vTaskDelay(pdMS_TO_TICKS(hold));
                esp_hidd_send_mouse_value(id, 0, 0, 0);   /* release buttons */
            }
            break;
        }
    }
}

/* ── Enqueue helper ──────────────────────────────────────────────────────── */
static esp_err_t submit(const hid_req_t *req)
{
    if (s_req_q == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!hid_output_is_ready()) {
        ESP_LOGW(TAG, "HID not ready (connected=%d bonded=%d) — report dropped",
                 (int)s_connected, (int)ble_stack_is_bonded());
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_req_q, req, 0) != pdTRUE) {
        ESP_LOGW(TAG, "report queue full — dropped");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */
esp_err_t hid_output_init(void)
{
    esp_err_t ret;

    s_req_q = xQueueCreate(HID_OUT_QUEUE_LEN, sizeof(hid_req_t));
    if (s_req_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(hid_worker_task, "hid_out", HID_OUT_TASK_STACK, NULL,
                    HID_OUT_TASK_PRIO, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_profile_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_hidd_register_callbacks(hidd_event_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_register_callbacks failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Only HIDD_APP_ID gets a slot — deliberately NOT BATTRAY_APP_ID.
     *
     * The demo registers both, but the battery attribute table is created on
     * the *HID* gatts_if (hid_device_le_prf.c: hidd_le_create_service() calls
     * esp_ble_gatts_create_attr_tab(bas_att_db, gatts_if, ...) before building
     * the HID service, so the HID service can include it). The BATTRAY
     * registration only produces a second gatts_if whose ESP_BAT_EVENT_REG
     * branch does nothing.
     *
     * Registering it here would be actively harmful: ble_stack dispatches by
     * gatts_if, so every CONNECT/DISCONNECT would reach esp_hidd_prf_cb_hdl
     * twice — calling esp_ble_set_encryption() twice and burning the single
     * hidd_clcb slot (HID_MAX_APPS == 1). The demo never hit this because its
     * one-entry table silently dropped whichever app_id registered first. */
    if ((ret = ble_stack_register_profile(HIDD_APP_ID, esp_hidd_prf_cb_hdl, NULL)) != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "hid_output initialised (app_id 0x%04x)", HIDD_APP_ID);
    return ESP_OK;
}

bool hid_output_is_ready(void)
{
    /* Both conditions matter: the HID report characteristics are declared with
     * ESP_GATT_PERM_*_ENCRYPTED, so an unencrypted link accepts the write at
     * the API level but the host never sees the report. */
    return s_connected && ble_stack_is_bonded();
}

uint16_t hid_output_conn_id(void)
{
    return s_conn_id;
}

esp_err_t hid_output_send_keyboard(uint8_t modifiers, const uint8_t keys[4],
                                   uint16_t hold_ms)
{
    hid_req_t req = {
        .kind      = REQ_KEYBOARD,
        .modifiers = modifiers,
        .hold_ms   = hold_ms,
    };
    if (keys != NULL) {
        memcpy(req.keys, keys, sizeof(req.keys));
    }
    return submit(&req);
}

esp_err_t hid_output_send_consumer(uint8_t code, uint16_t hold_ms)
{
    hid_req_t req = {
        .kind    = REQ_CONSUMER,
        .hold_ms = hold_ms,
    };
    req.keys[0] = code;
    return submit(&req);
}

esp_err_t hid_output_send_mouse(uint8_t buttons, int8_t dx, int8_t dy)
{
    hid_req_t req = {
        .kind    = REQ_MOUSE,
        .dx      = dx,
        .dy      = dy,
        .hold_ms = HID_OUTPUT_DEFAULT_HOLD_MS,
    };
    req.keys[0] = buttons;
    return submit(&req);
}
