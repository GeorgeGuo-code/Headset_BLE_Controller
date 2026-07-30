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

/* Forward decl: defined later in this file, used by the REQ_SEQ branch. */
static bool ascii_to_hid(char c, uint8_t *mod, uint8_t *keycode);

typedef enum {
    REQ_KEYBOARD,
    REQ_CONSUMER,
    REQ_MOUSE,
    REQ_SEQ,                  /* multi-step script (see hid_output_send_seq) */
} req_kind_t;

/* REQ_SEQ's payload is large (HID_SEQ_MAX_STEPS × ~40 B ≈ 640 B), so the
 * whole struct is ~660 B. 8 queue slots × 660 B = ~5 KB; acceptable given
 * the script saves the caller from issuing 8+ separate HID reports. The
 * assert below catches accidental growth (e.g. someone bumping
 * HID_SEQ_MAX_STEPS without thinking about queue memory). */
typedef struct {
    req_kind_t     kind;
    uint8_t        modifiers;
    uint8_t        keys[4];
    int8_t         dx, dy;
    uint16_t       hold_ms;
    size_t         n_steps;                    /* only used when kind == REQ_SEQ */
    hid_seq_step_t steps[HID_SEQ_MAX_STEPS];   /* only used when kind == REQ_SEQ */
} hid_req_t;
_Static_assert(sizeof(hid_req_t) <= 1024,
               "hid_req_t bloated — review HID_SEQ_MAX_STEPS or HID_SEQ_TEXT_MAX");

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

        case REQ_SEQ: {
            /* Walk the step array inline. We do NOT re-submit() per step —
             * the worker is busy with this request, so any new submit() would
             * block the worker itself, deadlock. Direct esp_hidd_send_*_value
             * calls bypass the queue and don't compete for slots.
             *
             * Each iteration re-checks hid_output_is_ready() so a disconnect
             * mid-seq aborts cleanly instead of silently dropping reports.
             *
             * ── CALLER RESPONSIBILITY: post-action delays ──
             * The seq worker is fast — steps run back-to-back with only the
             * step's own hold/delay between them. If a `key` step triggers
             * something asynchronous on the host (window open, app launch,
             * focus change), the NEXT step's HID reports race the host's
             * state machine and get lost.
             *
             * Concrete case: `key 8 21` (Win+R) tells Windows to open the
             * Run dialog, but the dialog takes ~150-250 ms to actually
             * appear and take focus. A `type <path>` step that follows
             * immediately will type into whatever window was focused
             * BEFORE the dialog opened, not into the dialog itself.
             *
             * Fix: insert a `sleep 350` between Win+R and the next type/key
             * step. For longer-lived app launches, push the sleep to 500 ms
             * on slower hosts. Same applies to Win+E (Explorer), Win+D
             * (desktop), Win+L (lock), etc. — anything that opens a new
             * window asynchronously.
             *
             * The `o` command (hid_output_open_path) is the bundled
             * "Win+R + 350 ms wait + type + Enter" helper for the common
             * case. seq + manual sleep is for the less common flows where
             * `o` doesn't fit. */
            ESP_LOGI(TAG, "seq: running %u steps", (unsigned)req.n_steps);
            bool completed = true;
            for (size_t i = 0; i < req.n_steps; i++) {
                if (!hid_output_is_ready()) {
                    ESP_LOGW(TAG, "seq: aborting at step %u/%u — link no longer ready",
                             (unsigned)(i + 1), (unsigned)req.n_steps);
                    completed = false;
                    break;
                }
                const hid_seq_step_t *s = &req.steps[i];
                switch (s->kind) {
                case HID_SEQ_SLEEP:
                    vTaskDelay(pdMS_TO_TICKS(s->u.sleep.ms));
                    break;

                case HID_SEQ_KEY: {
                    uint8_t k[4] = { s->u.key.keycode, 0, 0, 0 };
                    uint16_t h = s->u.key.hold_ms
                                     ? s->u.key.hold_ms
                                     : HID_OUTPUT_DEFAULT_HOLD_MS;
                    esp_hidd_send_keyboard_value(id, s->u.key.modifiers, k, 1);
                    vTaskDelay(pdMS_TO_TICKS(h));
                    esp_hidd_send_keyboard_value(id, 0, NULL, 0);
                    break;
                }

                case HID_SEQ_TYPE:
                    /* Press → 30 ms → release per char. Unmappable bytes
                     * (non-ASCII) are skipped with a per-char warn, matching
                     * hid_output_type_string()'s user-visible behaviour. */
                    for (uint8_t j = 0; j < s->u.type.len; j++) {
                        uint8_t mod = 0, kc = 0;
                        if (!ascii_to_hid(s->u.type.text[j], &mod, &kc)) {
                            ESP_LOGW(TAG, "seq type: skip byte 0x%02x", (unsigned)s->u.type.text[j]);
                            continue;
                        }
                        uint8_t k[4] = { kc, 0, 0, 0 };
                        esp_hidd_send_keyboard_value(id, mod, k, 1);
                        vTaskDelay(pdMS_TO_TICKS(HID_OUTPUT_DEFAULT_HOLD_MS));
                        esp_hidd_send_keyboard_value(id, 0, NULL, 0);
                    }
                    break;

                case HID_SEQ_CLICK: {
                    /* Press the button, hold HID_OUTPUT_DEFAULT_HOLD_MS, release. */
                    uint8_t btn = s->u.click.buttons;
                    esp_hidd_send_mouse_value(id, btn, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(HID_OUTPUT_DEFAULT_HOLD_MS));
                    esp_hidd_send_mouse_value(id, 0, 0, 0);
                    break;
                }

                case HID_SEQ_MOVE:
                    /* Pure relative motion, no press/release. Buttons stay 0. */
                    esp_hidd_send_mouse_value(id, 0, s->u.move.dx, s->u.move.dy);
                    break;
                }
            }
            if (completed) {
                ESP_LOGI(TAG, "seq: done (%u steps)", (unsigned)req.n_steps);
            }
            break;
        }
        }
    }
}

/* ── Enqueue helper ──────────────────────────────────────────────────────── */
/* Cap on how long submit() will wait for queue space. Worker drains at
 * ~30 ms/item (HID_OUTPUT_DEFAULT_HOLD_MS), so 2 s covers ~65 items queued
 * in front — enough for a typical Windows path (260 chars) plus slack. If
 * the worker is dead, the caller still gets ESP_ERR_NO_MEM in bounded time
 * instead of hanging forever. */
#define HID_SUBMIT_TIMEOUT_MS 2000

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
    /* Block until the queue has space, instead of dropping. Blocking is
     * safe here because the worker always drains at a known rate (one
     * 30-ms hold per item); the worst-case wait is the queue depth ×
     * 30 ms. The earlier "drop if full" semantics silently truncated
     * `o <long-path>` at the first Enter — the issue was just that the
     * type-the-path loop enqueues 10+ items back-to-back while the
     * worker is still holding the 350 ms Run-dialog delay. */
    if (xQueueSend(s_req_q, req, pdMS_TO_TICKS(HID_SUBMIT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "report queue full after %d ms — worker stalled?",
                 HID_SUBMIT_TIMEOUT_MS);
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

/* ── ASCII → HID (modifier, keycode) ─────────────────────────────────────── */

/* Map one ASCII char to a (modifier, keycode) pair using US-QWERTY layout
 * (the only layout the BLE HID descriptor in hid_device_le_prf.c declares).
 * Returns false if the char has no mapping — caller skips it. Caps Lock
 * state is ignored; shift is what matters for case. */
static bool ascii_to_hid(char c, uint8_t *mod, uint8_t *keycode)
{
    *mod = 0;

    if (c >= 'a' && c <= 'z') {
        *keycode = HID_KEY_A + (c - 'a');
        return true;
    }
    if (c >= 'A' && c <= 'Z') {
        *mod     = LEFT_SHIFT_KEY_MASK;
        *keycode = HID_KEY_A + (c - 'A');
        return true;
    }
    if (c >= '1' && c <= '9') {
        *keycode = HID_KEY_1 + (c - '1');
        return true;
    }
    if (c == '0') {
        *keycode = HID_KEY_0;
        return true;
    }

    /* Punctuation that needs shift. Order doesn't matter, just exhaustive. */
    switch (c) {
    case ' ':  *keycode = HID_KEY_SPACEBAR;               return true;
    case '.':  *keycode = HID_KEY_DOT;                    return true;
    case ',':  *keycode = HID_KEY_COMMA;                  return true;
    case '-':  *keycode = HID_KEY_MINUS;                  return true;
    case '_':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_MINUS;  return true;
    case '/':  *keycode = HID_KEY_FWD_SLASH;              return true;
    case '\\': *keycode = HID_KEY_BACK_SLASH;             return true;
    case ':':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_SEMI_COLON; return true;
    case '\'': *keycode = HID_KEY_SGL_QUOTE;              return true;
    case '"':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_SGL_QUOTE; return true;
    case '!':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_1;       return true;
    case '@':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_2;       return true;
    case '#':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_3;       return true;
    case '$':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_4;       return true;
    case '%':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_5;       return true;
    case '^':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_6;       return true;
    case '&':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_7;       return true;
    case '*':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_8;       return true;
    case '(':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_9;       return true;
    case ')':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_0;       return true;
    case '[':  *keycode = HID_KEY_LEFT_BRKT;              return true;
    case ']':  *keycode = HID_KEY_RIGHT_BRKT;             return true;
    case '{':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_LEFT_BRKT;  return true;
    case '}':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_RIGHT_BRKT; return true;
    case '=':  *keycode = HID_KEY_EQUAL;                  return true;
    case '+':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_EQUAL;       return true;
    case '`':  *keycode = HID_KEY_GRV_ACCENT;             return true;
    case '~':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_GRV_ACCENT; return true;
    case ';':  *keycode = HID_KEY_SEMI_COLON;             return true;
    case '<':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_COMMA;       return true;
    case '>':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_DOT;         return true;
    case '?':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_FWD_SLASH;   return true;
    case '|':  *mod = LEFT_SHIFT_KEY_MASK; *keycode = HID_KEY_BACK_SLASH;  return true;
    case '\n': *keycode = HID_KEY_RETURN;                 return true;
    case '\t': *keycode = HID_KEY_TAB;                    return true;
    default:   return false;
    }
}

/* The worker does press → hold (HID_OUTPUT_DEFAULT_HOLD_MS = 30 ms) →
 * release, so back-to-back calls already give ~30 ms inter-char gap without
 * the caller waiting. Runs of `o <path>` feel snappy at this rate. */

esp_err_t hid_output_type_string(const char *str)
{
    if (!hid_output_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    while (*str) {
        uint8_t mod = 0, keycode = 0;
        if (!ascii_to_hid(*str, &mod, &keycode)) {
            /* Unmappable byte (e.g. non-ASCII / control). Skip rather than
             * abort — paths can contain CJK that the host user types
             * directly later anyway. */
            ESP_LOGW(TAG, "type_string: skip unmappable byte 0x%02x", (unsigned)*str);
            str++;
            continue;
        }
        uint8_t key[4] = { keycode, 0, 0, 0 };
        esp_err_t err = hid_output_send_keyboard(mod, key, 0);
        if (err != ESP_OK) {
            return err;
        }
        str++;
    }
    return ESP_OK;
}

/* How long to wait between Win+R and the first path char. Windows pops the
 * Run dialog in ~150–250 ms on a fresh boot; 350 ms is enough headroom for
 * older machines without making the path-open feel sluggish. */
#define HID_RUN_DIALOG_OPEN_MS 350

esp_err_t hid_output_open_path(const char *path)
{
    if (!path || !*path) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!hid_output_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Win+R: left GUI + R. Empty key array, leftover bytes zero. */
    uint8_t win_r[4] = { HID_KEY_R, 0, 0, 0 };
    esp_err_t err = hid_output_send_keyboard(LEFT_GUI_KEY_MASK, win_r, 0);
    if (err != ESP_OK) {
        return err;
    }

    /* The worker is single-threaded, so we can't just sleep here while other
     * HID requests trickle in. We rely on the sequence of native cmd queue
     * ordering: subsequent hid_output_send_*() calls happen after Win+R is
     * fully sent. The host side needs ~350 ms to pop the Run dialog before
     * it starts buffering the typed path, so we enqueue a delay as a no-op
     * keyboard release (modifiers=0, keys=0) with a long hold_ms. */
    uint8_t none[4] = { 0, 0, 0, 0 };
    err = hid_output_send_keyboard(0, none, HID_RUN_DIALOG_OPEN_MS);
    if (err != ESP_OK) {
        return err;
    }

    err = hid_output_type_string(path);
    if (err != ESP_OK) {
        return err;
    }

    /* Enter to dispatch the Run command. */
    uint8_t enter[4] = { HID_KEY_RETURN, 0, 0, 0 };
    return hid_output_send_keyboard(0, enter, 0);
}

esp_err_t hid_output_send_seq(const hid_seq_step_t *steps, size_t n)
{
    if (steps == NULL || n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (n > HID_SEQ_MAX_STEPS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!hid_output_is_ready()) {
        ESP_LOGW(TAG, "seq: HID not ready (connected=%d bonded=%d) — script dropped",
                 (int)s_connected, (int)ble_stack_is_bonded());
        return ESP_ERR_INVALID_STATE;
    }

    hid_req_t req = {
        .kind    = REQ_SEQ,
        .n_steps = n,
    };
    memcpy(req.steps, steps, n * sizeof(hid_seq_step_t));

    /* Reuse submit() — the worker is idle here, the queue has space (8 slots
     * and we only consume one). The 2 s timeout covers the rare case where
     * another script is still draining; longer than 2 s means the worker
     * itself is wedged and we'd rather fail fast. */
    return submit(&req);
}
