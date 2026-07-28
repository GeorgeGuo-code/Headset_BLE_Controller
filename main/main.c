/*
 * main.c — boot sequence: NVS -> MPU/DMP -> gesture detector -> BLE console.
 *
 * Calibration is triggered over BLE (Nordic-UART-Service style console): a
 * central (e.g. the Electron tool or nRF Connect) writes a one-line command
 * ("c"/"ca"/"ct"/"p"/"sp"/"sr") to the RX characteristic. Gesture events and
 * calibration prompts/results are streamed back over the TX notify
 * characteristic (and mirrored to the UART console for local debugging).
 *
 * This replaces the old UART-command trigger from the reference firmware. The
 * BOOT button remains as an offline fallback trigger.
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "MPU6050.h"
#include "inv_mpu.h"
#include "gesture_detect.h"
#include "ble_console.h"

#define TAG "main"

#define EVENT_QUEUE_LEN 8
#define CMD_QUEUE_LEN   4
#define CMD_MAX_LEN     16

/* BOOT button (GPIO0 on most ESP32-S3 dev boards). Active-low. Hold to
 * trigger a full guided calibration without a BLE central attached. */
#define BOOT_GPIO_NUM     GPIO_NUM_0
#define BOOT_ACTIVE_LEVEL 0
#define BOOT_HOLD_MS      1000

/* Commands are posted here from the BLE callback (and the BOOT button) and
 * executed serially by cal_worker_task, so the long (~11 s) calibration never
 * runs in the BLE stack task context. */
static QueueHandle_t s_cmd_q;

/* False until gesture_detect_init() succeeds. The console (and therefore the
 * command worker) comes up before the sensor, so commands can arrive while the
 * detector is not initialised — or never will be, if the MPU is dead. */
static volatile bool s_detector_ready = false;

/* ── Gesture consumer: stream each event over BLE (and UART). ─────────────── */
static void gesture_bridge_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;
    for (;;) {
        gesture_event_t ev;
        if (xQueueReceive(q, &ev, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        const char *name = "NONE";
        switch (ev.type) {
        case GESTURE_NOD:        name = "NOD";        break;
        case GESTURE_LOOK_UP:    name = "LOOK_UP";    break;
        case GESTURE_TILT_LEFT:  name = "TILT_LEFT";  break;
        case GESTURE_TILT_RIGHT: name = "TILT_RIGHT"; break;
        case GESTURE_NONE:
        default:                 name = "NONE";       break;
        }
        ble_console_logf("GESTURE %s ts=%u peak=%.1f vel=%.1f\n",
                         name, (unsigned)ev.timestamp_ms,
                         ev.peak_angle_deg, ev.peak_velocity_deg_s);
    }
}

/* Guided three-phase calibration (neutral -> nod_axis -> tilt_axis). Prompts
 * and results go over BLE so the connected tool can display them. Ported from
 * the reference firmware; ~11.4 s total. */
static void run_guided_calibration(void)
{
    ble_console_log("== calibration 1/3: keep your head STILL ==\n");
    esp_err_t err = gesture_detect_calibrate_neutral(2000);
    if (err != ESP_OK) {
        ble_console_logf("neutral capture failed: %s — aborting\n", esp_err_to_name(err));
        return;
    }
    ble_console_log("== calibration 2/3: do a few slow NODS now ==\n");
    vTaskDelay(pdMS_TO_TICKS(700));
    err = gesture_detect_calibrate_axes(4000);
    if (err != ESP_OK) {
        ble_console_logf("nod-axis capture failed: %s — aborting\n", esp_err_to_name(err));
        return;
    }
    ble_console_log("== calibration 3/3: do slow LEFT and RIGHT tilts now ==\n");
    vTaskDelay(pdMS_TO_TICKS(700));
    err = gesture_detect_calibrate_tilt(4000);
    ble_console_logf("calibration result: %s\n", err == ESP_OK ? "OK" : esp_err_to_name(err));
}

/* Execute one console command (dispatch mirrors the reference UART command
 * set). Runs in cal_worker_task, so blocking calibration is fine here. */
static void handle_command(const char *cmd)
{
    if (!s_detector_ready) {
        ble_console_log("sensor not initialised — commands unavailable\n");
        return;
    }

    if (strcmp(cmd, "c") == 0 || strcmp(cmd, "cal") == 0) {
        ble_console_log("triggering guided calibration...\n");
        run_guided_calibration();
    } else if (strcmp(cmd, "ca") == 0) {
        ble_console_log("nod calibration only (do slow nods)...\n");
        esp_err_t err = gesture_detect_calibrate_axes(4000);
        ble_console_logf("nod calibration result: %s\n", err == ESP_OK ? "OK" : esp_err_to_name(err));
    } else if (strcmp(cmd, "ct") == 0) {
        ble_console_log("tilt calibration only (do slow LEFT and RIGHT tilts)...\n");
        esp_err_t err = gesture_detect_calibrate_tilt(4000);
        ble_console_logf("tilt calibration result: %s\n", err == ESP_OK ? "OK" : esp_err_to_name(err));
    } else if (strcmp(cmd, "p") == 0) {
        const gesture_params_t *p = gesture_detect_get_params();
        ble_console_logf("params: trigger=%.1f vel=%.1f zone=%.1f debounce=%u "
                         "sign_pitch=%u sign_roll=%u\n",
                         p->trigger_deg, p->trigger_velocity_deg_s,
                         p->neutral_zone_deg, (unsigned)p->debounce_ms,
                         (unsigned)p->sign_pitch, (unsigned)p->sign_roll);
        ble_console_logf("  q_neutral=[%.3f %.3f %.3f %.3f] nod=[%.2f %.2f %.2f] "
                         "tilt=[%.2f %.2f %.2f]\n",
                         p->neutral.q_neutral[0], p->neutral.q_neutral[1],
                         p->neutral.q_neutral[2], p->neutral.q_neutral[3],
                         p->neutral.nod_axis[0], p->neutral.nod_axis[1], p->neutral.nod_axis[2],
                         p->neutral.tilt_axis[0], p->neutral.tilt_axis[1], p->neutral.tilt_axis[2]);
        /* Phase 5: also report the runtime sliding baseline so the user can
         * see how much佩戴微调 has been absorbed. */
        float qd[4];
        gesture_detect_get_q_drift(qd);
        ble_console_logf("  q_drift =[%.3f %.3f %.3f %.3f]\n",
                         qd[0], qd[1], qd[2], qd[3]);
        /* Phase 6: nod/tilt axes rotated into the current q_drift frame.
         * These are what the detector actually projects motion against; if
         * they differ noticeably from the q_neutral-frame axes printed
         * above, q_drift has absorbed佩戴微调 and the projection is still
         * geometrically correct. */
        float nod_eff[3], tilt_eff[3];
        gesture_detect_get_effective_axes(nod_eff, tilt_eff);
        ble_console_logf("  nod_eff  =[%.2f %.2f %.2f]\n",
                         nod_eff[0], nod_eff[1], nod_eff[2]);
        ble_console_logf("  tilt_eff =[%.2f %.2f %.2f]\n",
                         tilt_eff[0], tilt_eff[1], tilt_eff[2]);
    } else if (strcmp(cmd, "q") == 0) {
        /* Phase 5: standalone q_drift diagnostic. Reports the angle between
         * q_drift and q_neutral — the larger this gets, the more佩戴微调
         * the device has absorbed. A persistently large value (e.g. >15°)
         * means the still-snap isn't catching up; run `q reset` to force
         * a re-sync from the next DMP sample. */
        const gesture_params_t *p = gesture_detect_get_params();
        float qd[4];
        gesture_detect_get_q_drift(qd);
        float qn_conj[4], qdiff[4];
        /* Use the same quat helpers the detector does. They aren't exposed
         * in the public header, so we re-derive the angle inline: the angle
         * between two unit quaternions is 2·acos(|dot|) degrees. */
        float d = qd[0]*p->neutral.q_neutral[0] +
                  qd[1]*p->neutral.q_neutral[1] +
                  qd[2]*p->neutral.q_neutral[2] +
                  qd[3]*p->neutral.q_neutral[3];
        if (d < 0.0f) d = -d;
        if (d > 1.0f) d = 1.0f;
        float angle_deg = 2.0f * acosf(d) * 57.29578f;
        ble_console_logf("q_drift  : [%.3f %.3f %.3f %.3f]\n",
                         qd[0], qd[1], qd[2], qd[3]);
        ble_console_logf("q_neutral: [%.3f %.3f %.3f %.3f]\n",
                         p->neutral.q_neutral[0], p->neutral.q_neutral[1],
                         p->neutral.q_neutral[2], p->neutral.q_neutral[3]);
        ble_console_logf("angle(deg)=%.2f  (use 'q reset' to force re-sync)\n",
                         angle_deg);
    } else if (strcmp(cmd, "q reset") == 0) {
        gesture_detect_reset_q_drift();
        ble_console_log("q_drift reset — next DMP sample becomes new baseline\n");
    } else if (strcmp(cmd, "sp") == 0) {
        gesture_params_t params = *gesture_detect_get_params();
        params.sign_pitch ^= 1;
        ESP_ERROR_CHECK(gesture_detect_apply_params(&params));
        ESP_ERROR_CHECK(gesture_params_save_to_nvs(&params));
        ble_console_logf("sign_pitch flipped -> positive_pitch_is_nod=%u\n",
                         (unsigned)params.sign_pitch);
    } else if (strcmp(cmd, "sr") == 0) {
        gesture_params_t params = *gesture_detect_get_params();
        params.sign_roll ^= 1;
        ESP_ERROR_CHECK(gesture_detect_apply_params(&params));
        ESP_ERROR_CHECK(gesture_params_save_to_nvs(&params));
        ble_console_logf("sign_roll flipped -> positive_roll_is_right=%u\n",
                         (unsigned)(params.sign_roll == 1));
    } else if (strcmp(cmd, "cd") == 0) {
        /* Capture diagnostics for the v3 prototype. Shows how stable the last
         * calibration was, which is the main knob we don't yet verify at
         * runtime. Re-run after `c` to compare runs. */
        gesture_detect_capture_t cap;
        gesture_detect_get_last_capture(&cap);
        ble_console_logf("cap diag: valid=%u used=%u sum_mag=%.1f drift=%.1f\n",
                         (unsigned)cap.valid, (unsigned)cap.used,
                         cap.sum_mag_deg, cap.drift_deg);
        ble_console_logf("  nod =[%.2f %.2f %.2f]\n",
                         cap.nod_axis[0], cap.nod_axis[1], cap.nod_axis[2]);
        ble_console_logf("  tilt=[%.2f %.2f %.2f]\n",
                         cap.tilt_axis[0], cap.tilt_axis[1], cap.tilt_axis[2]);
    } else if (cmd[0] != '\0') {
        ble_console_logf("unknown command: '%s' (try c, ca, ct, p, q, sp, sr, cd)\n", cmd);
    }
}

/* Drains the command queue and runs each command serially. */
static void cal_worker_task(void *arg)
{
    (void)arg;
    char cmd[CMD_MAX_LEN];
    for (;;) {
        if (xQueueReceive(s_cmd_q, cmd, portMAX_DELAY) == pdTRUE) {
            handle_command(cmd);
        }
    }
}

/* BLE RX write callback — runs in the BLE stack task, so just enqueue. */
static void on_console_cmd(const char *cmd, size_t len)
{
    (void)len;
    char buf[CMD_MAX_LEN];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    if (s_cmd_q != NULL) {
        (void)xQueueSend(s_cmd_q, buf, 0);  /* drop if worker is busy */
    }
}

/* BOOT-button task — offline fallback trigger. Posts "c" to the command
 * queue when held for BOOT_HOLD_MS. */
static void boot_button_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_GPIO_NUM),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    bool       pressed     = false;
    TickType_t press_start = 0;
    while (1) {
        bool is_down = (gpio_get_level(BOOT_GPIO_NUM) == BOOT_ACTIVE_LEVEL);
        if (is_down && !pressed) {
            pressed     = true;
            press_start = xTaskGetTickCount();
        } else if (!is_down && pressed) {
            pressed = false;
        } else if (is_down && pressed) {
            if (pdTICKS_TO_MS(xTaskGetTickCount() - press_start) >= BOOT_HOLD_MS) {
                while (gpio_get_level(BOOT_GPIO_NUM) == BOOT_ACTIVE_LEVEL) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                char c[CMD_MAX_LEN] = "c";
                if (s_cmd_q != NULL) {
                    (void)xQueueSend(s_cmd_q, c, 0);
                }
                pressed = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Human-readable mpu_dmp_init() return codes (see inv_mpu.c). */
static const char *dmp_err_str(uint8_t code)
{
    switch (code) {
    case 1:  return "mpu_set_sensors";
    case 2:  return "mpu_configure_fifo";
    case 3:  return "mpu_set_sample_rate";
    case 4:  return "dmp_load_motion_driver_firmware";
    case 5:  return "dmp_set_orientation";
    case 6:  return "dmp_enable_feature";
    case 7:  return "dmp_set_fifo_rate";
    case 8:  return "run_self_test (keep the board still)";
    case 9:  return "mpu_set_dmp_state";
    case 10: return "MPU_Init / I2C — sensor not responding";
    default: return "unknown";
    }
}

void app_main(void)
{
    /* 1. NVS — required for gesture params AND the BLE stack. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. MPU6050 + DMP, BEFORE the BLE stack comes up.
     *
     *    Order matters: bringing Bluedroid up first starves the I2C driver
     *    (its controller tasks sit at near-top priority), which made
     *    mpu_dmp_init fail randomly at whatever register write happened to be
     *    in flight. DMP firmware load is ~1 k transfers, so it wants a quiet
     *    bus. A failure here no longer halts boot — the console still starts
     *    below, in degraded mode. */
    uint8_t dmp_res = 0;
    for (int attempt = 1; attempt <= 5; attempt++) {
        dmp_res = mpu_dmp_init();
        if (dmp_res == 0) {
            break;
        }
        ESP_LOGW(TAG, "mpu_dmp_init attempt %d failed (%u: %s)", attempt,
                 (unsigned)dmp_res, dmp_err_str(dmp_res));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (dmp_res != 0) {
        ESP_LOGE(TAG, "mpu_dmp_init failed after 5 attempts (%u: %s)",
                 (unsigned)dmp_res, dmp_err_str(dmp_res));
    }

    /* 3. Detector — loads params from NVS (or installs defaults). Skipped when
     *    the sensor is dead. */
    if (dmp_res == 0) {
        ESP_ERROR_CHECK(gesture_detect_init());
        s_detector_ready = true;
    }

    /* 4. Command queue + BLE console. Comes up even in degraded mode, so the
     *    failure is visible in the config tool instead of only on UART. */
    s_cmd_q = xQueueCreate(CMD_QUEUE_LEN, CMD_MAX_LEN);
    ESP_ERROR_CHECK(s_cmd_q == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(ble_console_init(on_console_cmd));
    xTaskCreate(cal_worker_task,  "cal_worker",  4096, NULL, 3, NULL);
    xTaskCreate(boot_button_task, "boot_button", 4096, NULL, 3, NULL);

    if (!s_detector_ready) {
        ble_console_logf("SENSOR FAIL: mpu_dmp_init=%u (%s)\n",
                         (unsigned)dmp_res, dmp_err_str(dmp_res));
        if (dmp_res == 10) {
            ble_console_log("no I2C answer at 0x68 — check SDA=5 / SCL=6, 3V3, GND, AD0\n");
            MPU_Bus_Scan();   /* results go to UART */
        }
        ble_console_log("BLE console is up, but gesture detection is disabled\n");
        return;
    }

    ESP_LOGI(TAG, "wear device, connect over BLE (\"HMBC-Console\"), then send `c` "
                  "to calibrate — or hold BOOT 1 s");

    /* 5. Event queue + detector + consumer. */
    QueueHandle_t q = xQueueCreate(EVENT_QUEUE_LEN, sizeof(gesture_event_t));
    ESP_ERROR_CHECK(q == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(gesture_detect_start(q));

    xTaskCreate(gesture_bridge_task, "gesture_bridge", 3072, q, 4, NULL);

    ble_console_log("boot complete — gestures stream here; send `c` to calibrate\n");
}
