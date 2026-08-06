/*
 * main.c — boot sequence: NVS -> MPU/DMP -> gesture detector -> BLE stack
 * (NUS console + HID) -> UART console.
 *
 * Commands ("c"/"ca"/"ct"/"p"/"sp"/"sr"/"hs"/"ac"/"ak"/"o"/...) arrive on two
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
#include "cmd_config.h"
#include "touch_sensor.h"
#include "driver/touch_sens.h"

#define TAG "main"

#define BLE_DEVICE_NAME "HMBC-Console"

#define EVENT_QUEUE_LEN 8
#define CMD_QUEUE_LEN   4
/* Phase 7: was 16. Commands now take arguments ("ak 8 4", "af 12"), and Step 4
 * will add rule names, so the buffer needs room beyond a two-letter opcode. */
#define CMD_MAX_LEN     1024

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

        /* Execute any configs triggered by this gesture. */
        cmd_config_execute_by_trigger(TRIGGER_GESTURE, (uint16_t)ev.type);
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

/* Range-bounded integer parsers for the `seq` command. We can't strtol
 * directly on [p, limit) because it's not NUL-terminated, so each integer
 * is copied into a small stack buffer first. 16 bytes is plenty for any
 * HID parameter we accept (max is 60000 ms sleep = 5 digits, plus sign). */
#define SEQ_INT_BUF  16

/* Parse one signed integer at p (after skipping leading whitespace), staying
 * within [p, limit). On success, returns the pointer to the first char past
 * the consumed integer (still inside [p, limit) or one-past-the-end). On
 * failure (no integer found), returns NULL. */
static const char *seq_parse_int(const char *p, const char *limit, long *out)
{
    while (p < limit && (*p == ' ' || *p == '\t')) p++;
    if (p >= limit) return NULL;

    char buf[SEQ_INT_BUF];
    size_t n = limit - p;
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, p, n);
    buf[n] = '\0';

    char *endp = NULL;
    long v = strtol(buf, &endp, 0);
    if (endp == buf) {
        return NULL;  /* no digits */
    }
    *out = v;
    return p + (endp - buf);
}

/* Parse two signed integers in sequence. On success, returns the pointer
 * past the second one. On any failure, returns NULL — *a and *b are
 * undefined. */
static const char *seq_parse_two_ints(const char *p, const char *limit,
                                      long *a, long *b)
{
    p = seq_parse_int(p, limit, a);
    if (p == NULL) return NULL;
    p = seq_parse_int(p, limit, b);
    return p;
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

    if (strncmp(cmd, "o", 1) == 0 && (cmd[1] == '\0' || cmd[1] == ' ')) {
        /* o [path] — open Windows app via Win+R. No args = "notepad".
         * Path is the rest of the line verbatim; spaces and ASCII punctuation
         * are passed through unchanged. */
        const char *path = cmd + 1;
        while (*path == ' ' || *path == '\t') {
            path++;
        }
        if (*path == '\0') {
            path = "notepad";
        }
        esp_err_t err = hid_output_open_path(path);
        ble_console_logf("open \"%s\" -> %s\n", path, esp_err_to_name(err));
        return true;
    }

    if (strncmp(cmd, "seq", 3) == 0 && (cmd[3] == '\0' || cmd[3] == ' ')) {
        /* seq <step>; <step>; ...   (Phase 8)
         *
         * The whole line after "seq" is split on ';' into steps. Each step
         * starts with one of:
         *   sleep <ms>          — worker-side delay, no HID traffic
         *   key  <mod> <kc>     — keyboard press, hold HID_OUTPUT_DEFAULT_HOLD_MS, release
         *   type <text>         — type ASCII (text is everything up to the next ';')
         *   click left|right|middle|0..7  — mouse press+release
         *   move <dx> <dy>      — single relative mouse report
         *
         * Notes:
         *  - `type` deliberately eats the rest of the step (including spaces),
         *    so a path with spaces must be split: `seq type hello world;` works.
         *  - ';' inside `type` text is not escapable; for paths use `o` instead.
         *  - The whole command is bounded by CMD_MAX_LEN (1024) — about 100
         *    short steps, which is enough for the typical scripted open.
         *  - GOTCHA: any `key` step that triggers an async window open (Win+R,
         *    Win+E, Win+D, Win+L, ...) MUST be followed by a `sleep 350` (or
         *    500 on slow hosts) before the next `type`/`key`. The Run dialog
         *    takes ~150-250 ms to appear and grab focus, so the next step's
         *    HID reports race the host's focus change. For "open app" use the
         *    bundled `o <path>` command — it has the 350 ms wait built in. */
        const char *p = cmd + 3;
        while (*p == ' ' || *p == '\t') p++;

        hid_seq_step_t steps[HID_SEQ_MAX_STEPS];
        size_t n = 0;

        while (*p && n < HID_SEQ_MAX_STEPS) {
            /* End of this step = next ';' (exclusive) or end of string. */
            const char *end = strchr(p, ';');
            if (end == NULL) end = p + strlen(p);

            /* Skip leading whitespace in the step. Empty steps (e.g. from a
             * double ";;") are silently skipped — that way "seq sleep 100;;key 0 40"
             * doesn't error out. */
            const char *tok = p;
            while (tok < end && (*tok == ' ' || *tok == '\t')) tok++;
            if (tok >= end) {
                p = (*end == ';') ? end + 1 : end;
                continue;
            }

            /* Find end of first token. */
            const char *tok_end = tok;
            while (tok_end < end && *tok_end != ' ' && *tok_end != '\t') tok_end++;
            size_t tlen = (size_t)(tok_end - tok);

            if (tlen == 5 && strncmp(tok, "sleep", 5) == 0) {
                long ms;
                if (seq_parse_int(tok_end, end, &ms) == NULL || ms <= 0 || ms > 60000) {
                    ble_console_logf("seq: bad sleep arg (want 1..60000 ms)\n");
                    return true;
                }
                steps[n].kind      = HID_SEQ_SLEEP;
                steps[n].u.sleep.ms = (uint16_t)ms;
                n++;
            } else if (tlen == 3 && strncmp(tok, "key", 3) == 0) {
                long mod, kc;
                if (seq_parse_two_ints(tok_end, end, &mod, &kc) == NULL) {
                    ble_console_log("seq: usage: key <mod> <keycode>   e.g. 'key 8 4' = Win+A\n");
                    return true;
                }
                steps[n].kind            = HID_SEQ_KEY;
                steps[n].u.key.modifiers  = (uint8_t)mod;
                steps[n].u.key.keycode    = (uint8_t)kc;
                steps[n].u.key.hold_ms    = 0;   /* default */
                n++;
            } else if (tlen == 4 && strncmp(tok, "type", 4) == 0) {
                /* `type` consumes the rest of the step, including any leading
                 * whitespace after the token. This is what lets users type
                 * "hello world" with a space inside the typed string. */
                const char *text = tok_end;
                while (text < end && (*text == ' ' || *text == '\t')) text++;
                size_t text_len = (size_t)(end - text);
                if (text_len == 0) {
                    ble_console_log("seq: usage: type <text>   (text goes to next ';')\n");
                    return true;
                }
                if (text_len > HID_SEQ_TEXT_MAX) {
                    ble_console_logf("seq: type text too long (max %d, got %u) — "
                                     "use multiple 'type' steps or 'o <path>'\n",
                                     HID_SEQ_TEXT_MAX, (unsigned)text_len);
                    return true;
                }
                steps[n].kind = HID_SEQ_TYPE;
                memcpy(steps[n].u.type.text, text, text_len);
                steps[n].u.type.text[text_len] = '\0';
                steps[n].u.type.len = (uint8_t)text_len;
                n++;
            } else if (tlen == 5 && strncmp(tok, "click", 5) == 0) {
                const char *arg = tok_end;
                while (arg < end && (*arg == ' ' || *arg == '\t')) arg++;
                size_t alen = (size_t)(end - arg);
                uint8_t btn = 0;
                if      (alen == 4 && strncmp(arg, "left",   4) == 0) btn = 1;
                else if (alen == 5 && strncmp(arg, "right",  5) == 0) btn = 2;
                else if (alen == 6 && strncmp(arg, "middle", 6) == 0) btn = 4;
                else {
                    /* Numeric fallback: 1=left, 2=right, 4=middle, or any combo. */
                    long nb;
                    if (seq_parse_int(arg, end, &nb) != NULL && nb >= 0 && nb <= 7) {
                        btn = (uint8_t)nb;
                    } else {
                        ble_console_log("seq: usage: click <left|right|middle|0..7>\n");
                        return true;
                    }
                }
                steps[n].kind            = HID_SEQ_CLICK;
                steps[n].u.click.buttons = btn;
                n++;
            } else if (tlen == 4 && strncmp(tok, "move", 4) == 0) {
                long dx, dy;
                if (seq_parse_two_ints(tok_end, end, &dx, &dy) == NULL) {
                    ble_console_log("seq: usage: move <dx> <dy>   (-128..127)\n");
                    return true;
                }
                if (dx < -128 || dx > 127 || dy < -128 || dy > 127) {
                    ble_console_logf("seq: move dx/dy out of range (-128..127), got %ld %ld\n",
                                     dx, dy);
                    return true;
                }
                steps[n].kind       = HID_SEQ_MOVE;
                steps[n].u.move.dx  = (int8_t)dx;
                steps[n].u.move.dy  = (int8_t)dy;
                n++;
            } else {
                ble_console_logf("seq: unknown step '%.*s'\n", (int)tlen, tok);
                ble_console_log("  known: sleep <ms> | key <mod> <kc> | type <text> | "
                                "click <btn> | move <dx> <dy>\n");
                return true;
            }

            p = (*end == ';') ? end + 1 : end;
        }

        if (n == 0) {
            ble_console_log("seq: no valid steps. usage: seq sleep 100; key 0 40; type hello; "
                            "click left; move 10 20\n");
            return true;
        }
        if (*p) {
            /* We filled the array before consuming all input. */
            ble_console_logf("seq: too many steps (max %d, more remain)\n", HID_SEQ_MAX_STEPS);
            return true;
        }

        esp_err_t err = hid_output_send_seq(steps, n);
        ble_console_logf("seq: %u steps -> %s\n", (unsigned)n, esp_err_to_name(err));
        return true;
    }

    /* ── command config management ────────────────────────────────────── */

#ifdef ENABLE_SERIAL_TRIGGER
    if (strncmp(cmd, "command ", 8) == 0) {
        /* command <id> — shorthand for "cmd run <id>" (debug build only) */
        long id;
        const char *p = cmd + 8;
        while (*p == ' ') p++;
        char *endp;
        id = strtol(p, &endp, 10);
        if (endp == p || id < 1 || id > CMD_CFG_MAX) {
            ble_console_logf("usage: command <id>  (1..%d)\n", CMD_CFG_MAX);
            return true;
        }
        esp_err_t err = cmd_config_execute((uint8_t)id);
        ble_console_logf("cmd %ld -> %s\n", id, esp_err_to_name(err));
        return true;
    }
#endif

    if (strncmp(cmd, "cmd ", 4) == 0) {
        const char *p = cmd + 4;
        while (*p == ' ') p++;

        /* cmd list */
        if (strncmp(p, "list", 4) == 0 && (p[4] == '\0' || p[4] == ' ')) {
            char buf[2048];
            esp_err_t err = cmd_config_list(buf, sizeof(buf));
            if (err == ESP_OK) {
                ble_console_log(buf);
            } else {
                ble_console_log("cmd list: buffer too small\n");
            }
            return true;
        }

        /* cmd get <id> */
        if (strncmp(p, "get ", 4) == 0) {
            long id;
            const char *q = p + 4;
            while (*q == ' ') q++;
            char *endp;
            id = strtol(q, &endp, 10);
            if (endp == q || id < 1 || id > CMD_CFG_MAX) {
                ble_console_logf("usage: cmd get <id>  (1..%d)\n", CMD_CFG_MAX);
                return true;
            }
            const cmd_config_t *cfg = cmd_config_get((uint8_t)id);
            if (!cfg) {
                ble_console_logf("cmd get %ld: not found\n", id);
                return true;
            }
            const char *tname = "none";
            if (cfg->trigger_type == TRIGGER_GESTURE) {
                switch (cfg->trigger_value) {
                case GESTURE_NOD:        tname = "gesture:nod";       break;
                case GESTURE_LOOK_UP:    tname = "gesture:look_up";   break;
                case GESTURE_TILT_LEFT:  tname = "gesture:tilt_left"; break;
                case GESTURE_TILT_RIGHT: tname = "gesture:tilt_right"; break;
                default:                 tname = "gesture:?";         break;
                }
            }
#ifdef ENABLE_SERIAL_TRIGGER
            else if (cfg->trigger_type == TRIGGER_COMMAND) {
                static char tbuf[24];
                snprintf(tbuf, sizeof(tbuf), "command:%u", (unsigned)cfg->trigger_value);
                tname = tbuf;
            }
#endif
            ble_console_logf("cfg: id=%u name=\"%s\" trigger=%s n_steps=%u\n",
                             (unsigned)cfg->id, cfg->name, tname, (unsigned)cfg->n_steps);
            char seq_buf[512];
            if (cmd_config_format_seq(cfg, seq_buf, sizeof(seq_buf)) == ESP_OK) {
                ble_console_logf("cfg: steps=%s\n", seq_buf);
            }
            return true;
        }

        /* cmd set <id> <name> <type> <value> <seq_text...> */
        if (strncmp(p, "set ", 4) == 0) {
            const char *q = p + 4;
            while (*q == ' ') q++;

            long id;
            char *endp;
            id = strtol(q, &endp, 10);
            if (endp == q || id < 1 || id > CMD_CFG_MAX) {
                ble_console_logf("usage: cmd set <id> <name> <type> <value> <seq>\n"
                                 "  type: none / gesture / command\n"
                                 "  gesture values: 1=nod 2=look_up 3=tilt_left 4=tilt_right\n");
                return true;
            }
            q = endp;

            /* parse name (next token, no spaces) */
            while (*q == ' ') q++;
            const char *name_start = q;
            while (*q && *q != ' ') q++;
            size_t name_len = (size_t)(q - name_start);
            if (name_len == 0 || name_len >= CMD_CFG_NAME_MAX) {
                ble_console_log("cmd set: invalid name\n");
                return true;
            }

            /* parse trigger type */
            while (*q == ' ') q++;
            const char *type_start = q;
            while (*q && *q != ' ') q++;
            size_t type_len = (size_t)(q - type_start);

            cmd_trigger_type_t ttype = TRIGGER_NONE;
            uint16_t tvalue = 0;
            if (type_len == 4 && strncmp(type_start, "none", 4) == 0) {
                ttype = TRIGGER_NONE;
            } else if (type_len == 7 && strncmp(type_start, "gesture", 7) == 0) {
                ttype = TRIGGER_GESTURE;
            }
#ifdef ENABLE_SERIAL_TRIGGER
            else if (type_len == 7 && strncmp(type_start, "command", 7) == 0) {
                ttype = TRIGGER_COMMAND;
            }
#endif
            else {
                ble_console_log("cmd set: trigger type must be none/gesture"
#ifdef ENABLE_SERIAL_TRIGGER
                                "/command"
#endif
                                "\n");
                return true;
            }

            /* parse trigger value */
            while (*q == ' ') q++;
            long tval = 0;
            if (ttype != TRIGGER_NONE) {
                tval = strtol(q, &endp, 10);
                if (endp == q) {
                    ble_console_log("cmd set: invalid trigger value\n");
                    return true;
                }
                q = endp;
            }

            /* parse seq text (rest of line) */
            while (*q == ' ') q++;

            cmd_config_t cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.id = (uint8_t)id;
            memcpy(cfg.name, name_start, name_len);
            cfg.name[name_len] = '\0';
            cfg.trigger_type = ttype;
            cfg.trigger_value = (uint16_t)tval;

            if (*q) {
                esp_err_t err = cmd_config_parse_seq(q, cfg.steps, (size_t *)&cfg.n_steps);
                if (err != ESP_OK) {
                    ble_console_logf("cmd set: bad seq text: %s\n", esp_err_to_name(err));
                    return true;
                }
            }

            esp_err_t err = cmd_config_set(&cfg);
            ble_console_logf("cmd set id=%u -> %s\n", (unsigned)id, esp_err_to_name(err));
            return true;
        }

        /* cmd del <id> */
        if (strncmp(p, "del ", 4) == 0) {
            long id;
            const char *q = p + 4;
            while (*q == ' ') q++;
            char *endp;
            id = strtol(q, &endp, 10);
            if (endp == q || id < 1 || id > CMD_CFG_MAX) {
                ble_console_logf("usage: cmd del <id>  (1..%d)\n", CMD_CFG_MAX);
                return true;
            }
            esp_err_t err = cmd_config_delete((uint8_t)id);
            ble_console_logf("cmd del %ld -> %s\n", id, esp_err_to_name(err));
            return true;
        }

        /* cmd run <id> */
        if (strncmp(p, "run ", 4) == 0) {
            long id;
            const char *q = p + 4;
            while (*q == ' ') q++;
            char *endp;
            id = strtol(q, &endp, 10);
            if (endp == q || id < 1 || id > CMD_CFG_MAX) {
                ble_console_logf("usage: cmd run <id>  (1..%d)\n", CMD_CFG_MAX);
                return true;
            }
            esp_err_t err = cmd_config_execute((uint8_t)id);
            ble_console_logf("cmd run %ld -> %s\n", id, esp_err_to_name(err));
            return true;
        }

        /* unknown cmd subcommand */
        ble_console_log("cmd: list | get <id> | set <id> <name> <type> <value> <seq> | del <id> | run <id>\n");
        ble_console_log("  type: none / gesture / command\n");
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
        ble_console_log("  hid     : hs | ac <code> | ak <mods> <key> | o [path] | seq <steps>\n");
        ble_console_log("  configs : cmd list|get|set|del|run\n");
#ifdef ENABLE_SERIAL_TRIGGER
        ble_console_log("            command <id> (debug)\n");
#endif
        ble_console_log("  seq     : sleep <ms>; key <mod> <kc>; type <text>; click <btn>; move <dx> <dy>\n");
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
                   "(hs | ac <code> | ak <mods> <key> | o [path] | seq <steps> | c | p | ...)",
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

    /* 1b. Command configs — loaded from NVS, used by `cmd`/`command` handler. */
    cmd_config_init();

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

    /* 5b. Touch sensor → mouse-left button. Default channel is T2 (GPIO2 on
     *     ESP32-S3). Placed after the BLE stack starts so the press/release
     *     worker can find a valid conn_id, and after the UART console so
     *     the REPL prompt is reachable during the ~6 s initial scan.
     *     Independent of the MPU — runs in both healthy and degraded mode. */
    ESP_ERROR_CHECK(touch_sensor_init(TOUCH_MIN_CHAN_ID + 1));

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
