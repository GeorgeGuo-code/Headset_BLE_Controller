/*
 * main.c — boot sequence: NVS -> MPU/DMP -> gesture detector -> BLE stack
 * (NUS console + HID) -> UART console.
 *
 * Commands ("c"/"ca"/"ct"/"p"/"sp"/"sr"/"hs"/"ac"/"ak"/...) arrive on two
 * channels that share one queue and one dispatcher:
 *   - BLE: a central (the Electron tool or nRF Connect) writes a line to the
 *     NUS RX characteristic. Results stream back over the TX notify char.
 *   - UART: `hmbc <cmd...>` in the serial REPL. Added in Phase 7 so HID can be
 *     smoke-tested without a BLE central attached.
 * Both feed s_cmd_q; handle_command() is the single implementation.
 *
 * The BOOT button remains as an offline fallback calibration trigger.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_console.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "MPU6050.h"
#include "inv_mpu.h"
#include "gesture_detect.h"
#include "ble_console.h"
#include "ble_stack.h"
#include "hid_output.h"
#include "hid_dev.h"

#define TAG "main"

#define BLE_DEVICE_NAME "HMBC-Console"

#define EVENT_QUEUE_LEN 8
#define CMD_QUEUE_LEN   4
/* Phase 7: was 16. Commands now take arguments ("ak 8 4", "af 12"), and Step 4
 * will add rule names, so the buffer needs room beyond a two-letter opcode. */
#define CMD_MAX_LEN     32

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

/* Parse up to `max` whitespace-separated integers following the opcode.
 * Returns how many were parsed. Accepts decimal or 0x-prefixed hex. */
static int parse_args(const char *cmd, long *out, int max)
{
    int n = 0;
    const char *p = cmd;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        char *end = NULL;
        long v = strtol(p, &end, 0);
        if (end == p) {          /* not a number — this is the opcode, skip it */
            while (*p && *p != ' ' && *p != '\t') {
                p++;
            }
            continue;
        }
        out[n++] = v;
        p = end;
    }
    return n;
}

/* HID / output-layer commands. Handled before the sensor-ready gate below so
 * HID stays testable when the MPU failed to come up (degraded mode).
 * Returns true if the command was consumed. */
static bool handle_hid_command(const char *cmd)
{
    long args[4];

    if (strcmp(cmd, "hs") == 0) {
        ble_console_logf("hid: connected=%u bonded=%u ready=%u conn_id=%u\n",
                         (unsigned)ble_stack_is_connected(),
                         (unsigned)ble_stack_is_bonded(),
                         (unsigned)hid_output_is_ready(),
                         (unsigned)hid_output_conn_id());
        return true;
    }

    if (strncmp(cmd, "ac", 2) == 0 && (cmd[2] == '\0' || cmd[2] == ' ')) {
        /* ac <consumer_code> — e.g. `ac 205` = PLAY_PAUSE, `ac 233` = VOL_UP */
        int n = parse_args(cmd, args, 1);
        if (n < 1) {
            ble_console_logf("usage: ac <code>  (%u=play/pause %u=vol+ %u=next)\n",
                             (unsigned)HID_CONSUMER_PLAY_PAUSE,
                             (unsigned)HID_CONSUMER_VOLUME_UP,
                             (unsigned)HID_CONSUMER_SCAN_NEXT_TRK);
            return true;
        }
        esp_err_t err = hid_output_send_consumer((uint8_t)args[0], 0);
        ble_console_logf("consumer %ld -> %s\n", args[0], esp_err_to_name(err));
        return true;
    }

    if (strncmp(cmd, "ak", 2) == 0 && (cmd[2] == '\0' || cmd[2] == ' ')) {
        /* ak <modifiers> <key> — e.g. `ak 8 4` = LeftGUI+A (Win+A) */
        int n = parse_args(cmd, args, 2);
        if (n < 2) {
            ble_console_log("usage: ak <modifiers> <keycode>   e.g. 'ak 8 4' = Win+A\n"
                            "  modifiers: 1=LCtrl 2=LShift 4=LAlt 8=LGui\n");
            return true;
        }
        uint8_t keys[4] = { (uint8_t)args[1], 0, 0, 0 };
        esp_err_t err = hid_output_send_keyboard((uint8_t)args[0], keys, 0);
        ble_console_logf("keyboard mod=0x%02lx key=%ld -> %s\n",
                         args[0], args[1], esp_err_to_name(err));
        return true;
    }

    return false;
}

/* Execute one console command. Runs in cal_worker_task, so blocking
 * calibration is fine here. */
static void handle_command(const char *cmd)
{
    if (handle_hid_command(cmd)) {
        return;
    }

    if (!s_detector_ready) {
        ble_console_log("sensor not initialised — gesture commands unavailable\n");
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
        ble_console_logf("unknown command: '%s'\n", cmd);
        ble_console_log("  gestures: c ca ct p q 'q reset' sp sr cd\n");
        ble_console_log("  hid     : hs | ac <code> | ak <mods> <key>\n");
        ble_console_log("  prefix 'hmbc' optional on BLE (e.g. 'hmbc p')\n");
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
    size_t n = 0;
    /* Phase 7.1: BLE NUS accepts both the bare command ("c", "p", ...) used
     * by the Electron config tool and the `hmbc <cmd>` prefix used by the
     * UART REPL. Strip the prefix if present so a single command set
     * survives both transports — and so the user can keep using the same
     * syntax when the UART console times out (e.g. idf.py monitor on a chip
     * whose USB-Serial-JTAG is the monitor's default port while the app's
     * REPL lives on UART0). */
    while (n < sizeof(buf) - 1 && cmd[n] != '\0') {
        buf[n] = cmd[n];
        n++;
    }
    buf[n] = '\0';
    char *p = buf;
    if (strncmp(p, "hmbc", 4) == 0) {
        p += 4;
        while (*p == ' ' || *p == '\t') {
            p++;
        }
    }
    if (*p == '\0') {
        return;  /* nothing left after stripping — ignore */
    }
    if (s_cmd_q != NULL) {
        (void)xQueueSend(s_cmd_q, p, 0);  /* drop if worker is busy */
    }
}

/* ── UART console: `hmbc <cmd...>` ───────────────────────────────────────────
 *
 * Phase 7 addition. Re-joins argv into the same one-line form the BLE RX path
 * produces and posts it to the SAME queue, so handle_command() stays the only
 * dispatcher. Posting (rather than calling directly) also keeps the ~11 s
 * calibration off the REPL task, which would otherwise stop echoing.
 *
 * esp_console picks UART vs USB-Serial-JTAG from CONFIG_ESP_CONSOLE_*, so this
 * works on both ESP32-S3 board wirings without a code change.
 */
static int cmd_hmbc(int argc, char **argv)
{
    char buf[CMD_MAX_LEN];
    size_t pos = 0;

    for (int i = 1; i < argc; i++) {
        size_t need = strlen(argv[i]) + (pos ? 1 : 0);
        if (pos + need >= sizeof(buf)) {
            printf("command too long (max %d chars)\n", CMD_MAX_LEN - 1);
            return 1;
        }
        if (pos) {
            buf[pos++] = ' ';
        }
        strcpy(buf + pos, argv[i]);
        pos += strlen(argv[i]);
    }
    buf[pos] = '\0';

    if (pos == 0) {
        printf("usage: hmbc <command>   e.g. 'hmbc hs', 'hmbc ac 205'\n");
        return 1;
    }
    if (s_cmd_q == NULL || xQueueSend(s_cmd_q, buf, 0) != pdTRUE) {
        printf("command queue full — worker busy\n");
        return 1;
    }
    return 0;
}

static esp_err_t start_uart_console(void)
{
    esp_console_repl_t        *repl        = NULL;
    esp_console_repl_config_t  repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt          = "hmbc>";
    repl_config.max_cmdline_length = 128;

    const esp_console_cmd_t cmd = {
        .command = "hmbc",
        .help    = "Send a command to the gesture/HID controller "
                   "(hs | ac <code> | ak <mods> <key> | c | p | ...)",
        .hint    = NULL,
        .func    = &cmd_hmbc,
    };

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t dev_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_serial_jtag(&dev_config, &repl_config, &repl),
                        TAG, "usb-serial-jtag repl");
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t dev_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_cdc(&dev_config, &repl_config, &repl),
                        TAG, "usb-cdc repl");
#else
    esp_console_dev_uart_config_t dev_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&dev_config, &repl_config, &repl),
                        TAG, "uart repl");
#endif

    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&cmd), TAG, "register hmbc");
    ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "register help");
    return esp_console_start_repl(repl);
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

    /* 4. Command queue, then the BLE profiles, then the radio.
     *
     *    Order matters: ble_stack_start() registers each profile's app_id with
     *    Bluedroid, so every ble_stack_register_profile() call (made from
     *    hid_output_init / ble_console_init) has to happen first. This all
     *    comes up even in degraded mode, so a sensor failure is visible in the
     *    config tool instead of only on UART. */
    s_cmd_q = xQueueCreate(CMD_QUEUE_LEN, CMD_MAX_LEN);
    ESP_ERROR_CHECK(s_cmd_q == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(hid_output_init());                 /* app_id 0x1812 */
    ESP_ERROR_CHECK(ble_console_init(on_console_cmd));  /* app_id 0x0055 */
    ESP_ERROR_CHECK(ble_stack_start(BLE_DEVICE_NAME));

    xTaskCreate(cal_worker_task,  "cal_worker",  4096, NULL, 3, NULL);
    xTaskCreate(boot_button_task, "boot_button", 4096, NULL, 3, NULL);

    /* 5. UART REPL — HID smoke tests without a BLE central. Started last so
     *    its prompt lands after the noisy boot logs. */
    esp_err_t cerr = start_uart_console();
    if (cerr != ESP_OK) {
        ESP_LOGW(TAG, "UART console unavailable: %s", esp_err_to_name(cerr));
    }

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

    /* 6. Event queue + detector + consumer. */
    QueueHandle_t q = xQueueCreate(EVENT_QUEUE_LEN, sizeof(gesture_event_t));
    ESP_ERROR_CHECK(q == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(gesture_detect_start(q));

    xTaskCreate(gesture_bridge_task, "gesture_bridge", 3072, q, 4, NULL);

    ble_console_log("boot complete — gestures stream here; send `c` to calibrate\n");
}
