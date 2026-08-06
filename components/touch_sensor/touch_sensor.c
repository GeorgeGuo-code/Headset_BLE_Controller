/*
 * touch_sensor.c — single-channel "touch button → mouse left" bridge.
 *
 * Built from the IDF touch_sens_basic example (see
 * reference/touch_sens_basic/). The example's app_main prints benchmark data
 * in a loop; here we replace the print loop with one binding: on_active →
 * press left, on_inactive → release left.
 *
 * The reference app does the initial scan synchronously and prints channel
 * info early. We keep that flow but skip the print loop — the calibration
 * steps (new_controller → new_channel → config_filter → enable → 3× oneshot
 * scans → read benchmark → reset threshold) stay verbatim, then we register
 * the callbacks and start continuous scanning.
 *
 * Mouse-button bitmask is the standard HID one (1 = left, 2 = right, 4 =
 * middle), same convention ble_hid uses internally — see esp_hidd_prf_api.h
 * and hid_output.c. We don't go through hid_output_send_mouse() because that
 * always does press → hold 30 ms → release, which makes a click rather than a
 * hold. The user-facing behaviour is "press while touched, release on
 * release", so we send the raw reports directly from the worker.
 */

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/touch_sens.h"
#include "esp_hidd_prf_api.h"

#include "hid_output.h"
#include "touch_sensor.h"

#define TAG "touch_sensor"

/* The example does 3 oneshot scans to settle the channel baseline. Each can
 * take up to 2 s, so this dominates the touch_sensor_init() wall time. */
#define INIT_SCAN_TIMES 3

/* The active threshold is set to `benchmark * TOUCH_BM_RATIO` after the
 * initial scans. 1.5% matches the example's value (it works well for the
 * V2 sample configuration in the SDK example); lower = more sensitive,
 * higher = more immune to electrical noise. */
#define TOUCH_BM_RATIO 0.015f

/* HID mouse button bitmask. 1 = left button. Same convention used by
 * hid_output.c. */
#define MOUSE_BTN_LEFT 0x01

/* Worker task sizing: the loop is `xQueueReceive` + at most one HID write.
 * 2 KiB is plenty; bumped to 3 KiB to keep headroom for ESP_LOGI. */
#define WORKER_STACK_WORDS 3072
#define WORKER_PRIORITY     5    /* above the main task (3); below the BLE
                                  * stack task. Press/release is a single
                                  * GATT write so it should go out promptly */
#define WORKER_QUEUE_LEN    4

typedef enum {
    CMD_PRESS,
    CMD_RELEASE,
} worker_cmd_t;

static touch_sensor_handle_t   s_sens_handle = NULL;
static touch_channel_handle_t  s_chan_handle = NULL;
static QueueHandle_t           s_cmd_q       = NULL;
static int                     s_chan_id     = -1;

/* ── Press/Release worker ─────────────────────────────────────────────────
 *
 * The touch sensor callbacks post a command here. We marshal HID mouse
 * writes off the ISR-ish callback context, and fold consecutive presses /
 * releases through a single `pressed` flag so back-to-back active events
 * don't fire the press report twice.
 *
 * The worker checks hid_output_is_ready() on every iteration — if the link
 * drops mid-gesture, the press (or release) we held is silently dropped.
 * That's the correct behaviour: the host already saw the in-air state of
 * the button when the link was up, and the next report we manage to send
 * will be the post-reconnect state. */
static void press_release_worker(void *arg)
{
    (void)arg;
    bool pressed = false;

    for (;;) {
        worker_cmd_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!hid_output_is_ready()) {
            ESP_LOGW(TAG, "HID link not ready — dropping %s",
                     cmd == CMD_PRESS ? "press" : "release");
            /* Keep `pressed` unchanged so the next valid event still
             * represents the real button state. */
            continue;
        }

        uint16_t conn_id = hid_output_conn_id();
        if (cmd == CMD_PRESS) {
            if (!pressed) {
                esp_hidd_send_mouse_value(conn_id, MOUSE_BTN_LEFT, 0, 0);
                pressed = true;
                ESP_LOGD(TAG, "CH %d: mouse LEFT down", s_chan_id);
            }
        } else { /* CMD_RELEASE */
            if (pressed) {
                esp_hidd_send_mouse_value(conn_id, 0, 0, 0);
                pressed = false;
                ESP_LOGD(TAG, "CH %d: mouse LEFT up", s_chan_id);
            }
        }
    }
}

/* ── Touch callbacks (fire from the touch sensor ISR) ─────────────────────
 *
 * Don't log here at ESP_LOGI: the framework's reference hint says use
 * ESP_EARLY_LOGI at most — we're inside an ISR-context callback. All we do
 * is post a one-byte command to the worker queue. Failure is only possible
 * if the queue is full, which means the worker is genuinely stuck and the
 * right thing is to drop the event. */
static bool on_active_cb(touch_sensor_handle_t sens_handle,
                         const touch_active_event_data_t *event,
                         void *user_ctx)
{
    (void)sens_handle;
    (void)user_ctx;
    worker_cmd_t cmd = CMD_PRESS;
    (void)xQueueSendFromISR(s_cmd_q, &cmd, NULL);
    return false;
}

static bool on_inactive_cb(touch_sensor_handle_t sens_handle,
                           const touch_inactive_event_data_t *event,
                           void *user_ctx)
{
    (void)sens_handle;
    (void)user_ctx;
    worker_cmd_t cmd = CMD_RELEASE;
    (void)xQueueSendFromISR(s_cmd_q, &cmd, NULL);
    return false;
}

/* Reconfig the threshold based on the freshly-measured baseline. Called
 * between the initial oneshot scans and re-enable, so the controller must
 * be in INIT state (touch_sensor_disable() was called above). */
static esp_err_t apply_initial_threshold(touch_channel_handle_t chan)
{
    uint32_t benchmark[TOUCH_SAMPLE_CFG_NUM] = {0};
    ESP_RETURN_ON_ERROR(
        touch_channel_read_data(chan, TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark),
        TAG, "read benchmark");

    touch_channel_config_t chan_cfg = {
        .active_thresh[0] = (uint32_t)(benchmark[0] * TOUCH_BM_RATIO),
        .charge_speed      = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt  = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    for (int i = 0; i < TOUCH_SAMPLE_CFG_NUM; i++) {
        ESP_LOGI(TAG, "CH %d sample_cfg[%d]: benchmark=%" PRIu32
                      " thresh=%" PRIu32,
                      s_chan_id, i, benchmark[i], chan_cfg.active_thresh[i]);
    }
    return touch_sensor_reconfig_channel(chan, &chan_cfg);
}

esp_err_t touch_sensor_init(int chan_id)
{
    if (chan_id < TOUCH_MIN_CHAN_ID || chan_id > TOUCH_MAX_CHAN_ID) {
        ESP_LOGE(TAG, "chan_id %d out of range [%d, %d]",
                 chan_id, (int)TOUCH_MIN_CHAN_ID, (int)TOUCH_MAX_CHAN_ID);
        return ESP_ERR_INVALID_ARG;
    }
    s_chan_id = chan_id;

    /* Step 1: controller. SOC_TOUCH_SAMPLE_CFG_NUM is 1 on V2 (ESP32-S3),
     * so the sample config array is exactly one entry. The V2 default
     * sample config (500 charge times, 0.5V-2.2V) is the same one the IDF
     * example uses. */
    touch_sensor_sample_config_t sample_cfg =
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2);
    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, &sample_cfg);
    ESP_RETURN_ON_ERROR(touch_sensor_new_controller(&sens_cfg, &s_sens_handle),
                        TAG, "new_controller");

    /* Step 2: one channel on the caller's pin. Default V2 channel config
     * mirrors the IDF example. */
    touch_channel_config_t chan_cfg = {
        .active_thresh[0] = 2000,                 /* seed value; overwritten after init scan */
        .charge_speed     = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(
        touch_sensor_new_channel(s_sens_handle, chan_id, &chan_cfg, &s_chan_handle),
        TAG, "new_channel");

    touch_chan_info_t info = {0};
    ESP_RETURN_ON_ERROR(touch_sensor_get_channel_info(s_chan_handle, &info),
                        TAG, "get_channel_info");
    ESP_LOGI(TAG, "touch CH %d on GPIO%d", chan_id, info.chan_gpio);

    /* Step 3: default filter. Same as the example; provides software
     * smoothing + 2-tick debounce on the active event. */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_RETURN_ON_ERROR(touch_sensor_config_filter(s_sens_handle, &filter_cfg),
                        TAG, "config_filter");

    /* Step 4: worker queue + task. Created before the initial scan so the
     * very first user touch (already in flight before we enable continuous
     * scanning) is caught. */
    s_cmd_q = xQueueCreate(WORKER_QUEUE_LEN, sizeof(worker_cmd_t));
    if (s_cmd_q == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(press_release_worker, "touch_hid", WORKER_STACK_WORDS,
                    NULL, WORKER_PRIORITY, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    /* Step 5: initial scan, then disable so we can re-apply the threshold.
     * The example calls trigger_oneshot_scanning() with `timeout_ms=2000`,
     * which (per the IDF docs) is "wait up to 2 s, then give up with
     * ESP_ERR_TIMEOUT". 3 iterations is enough for the smooth data to
     * settle around the real baseline. */
    ESP_RETURN_ON_ERROR(touch_sensor_enable(s_sens_handle), TAG, "enable (init)");
    for (int i = 0; i < INIT_SCAN_TIMES; i++) {
        ESP_RETURN_ON_ERROR(touch_sensor_trigger_oneshot_scanning(s_sens_handle, 2000),
                            TAG, "oneshot_scan %d", i);
    }
    /* Must disable before reconfiguring the channel threshold. */
    ESP_RETURN_ON_ERROR(touch_sensor_disable(s_sens_handle), TAG, "disable (init)");
    ESP_RETURN_ON_ERROR(apply_initial_threshold(s_chan_handle),
                        TAG, "apply_initial_threshold");

    /* Step 6: register callbacks. Must be called while the controller is
     * disabled (INIT state) per the IDF docs. */
    touch_event_callbacks_t cbs = {
        .on_active   = on_active_cb,
        .on_inactive = on_inactive_cb,
    };
    ESP_RETURN_ON_ERROR(touch_sensor_register_callbacks(s_sens_handle, &cbs, NULL),
                        TAG, "register_callbacks");

    /* Step 7: enable + start continuous scanning. Events will fire on the
     * first real touch. */
    ESP_RETURN_ON_ERROR(touch_sensor_enable(s_sens_handle), TAG, "enable");
    ESP_RETURN_ON_ERROR(touch_sensor_start_continuous_scanning(s_sens_handle),
                        TAG, "start_continuous_scanning");

    ESP_LOGI(TAG, "touch_sensor ready on CH %d (GPIO %d)", chan_id, info.chan_gpio);
    return ESP_OK;
}
