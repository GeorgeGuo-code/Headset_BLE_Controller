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

/**
 * @brief  Queue a sequence of keyboard keypresses that types the given ASCII
 *         string char-by-char. Each char is press → hold → release. The
 *         queue is fed a short inter-char delay between chars so the host
 *         text box can keep up.
 *
 *         Only ASCII printable + a few path-relevant chars are supported
 *         (a-z, A-Z, 0-9, space, '.', '-', '_', '\\', '/', ':'). Non-ASCII
 *         bytes are silently skipped — the friend command `o` accepts Chinese
 *         paths only when the host user types them directly.
 *
 *         Newlines and double quotes are passed through unchanged (newline
 *         = HID Enter, double quote = Shift+'). ASCII-only because the HID
 *         protocol can't carry Unicode without an Input Manager surrogate.
 *
 *         No NUL support — only `strlen(str)` chars are sent.
 *
 * @return ESP_OK on enqueue, ESP_ERR_INVALID_STATE if HID not ready.
 */
esp_err_t hid_output_type_string(const char *str);

/**
 * @brief  Open a Windows app via Win+R (Run dialog): types Win+R, waits for
 *         the dialog to appear, types the path, presses Enter.
 *
 *         Win+R is system-wide; the path can be either an exe name on PATH
 *         (e.g. "notepad") or a full path. The path is typed as-is; spaces
 *         and other special chars are sent via HID, no shell quoting
 *         happens (the Run dialog treats the string verbatim).
 *
 *         No NVS read here — caller hands in the path string. NVS resolution
 *         lives in main.c (`o` command) once the config tool is wired up.
 *
 * @param path  ASCII path; treated as the text to enter into the Run dialog.
 * @return ESP_OK on enqueue, ESP_ERR_INVALID_STATE if HID not ready.
 */
esp_err_t hid_output_open_path(const char *path);

/* ── Multi-step sequences (Phase 8) ──────────────────────────────────────── */

/** Max characters in a single `type` step's payload. ASCII paths, short
 *  search queries, etc. Longer strings are the job of the dedicated `o`
 *  command (hid_output_open_path), which has no in-band length limit. */
#define HID_SEQ_TEXT_MAX  32

/** Max steps in a single `seq` command. The whole sequence is one queue
 *  item; this caps the per-item memory budget at ~640 B. 16 is enough for
 *  realistic scripts ("open app → wait → type → enter" = 4 steps). */
#define HID_SEQ_MAX_STEPS 16

/** Step kinds inside a `seq` sequence. */
typedef enum {
    HID_SEQ_SLEEP,    /**< wait `ms` milliseconds (worker-side delay, not HID) */
    HID_SEQ_KEY,      /**< press modifier+keycode, hold, release */
    HID_SEQ_TYPE,     /**< type an ASCII string char-by-char */
    HID_SEQ_CLICK,    /**< press + release a mouse button (left/right/middle) */
    HID_SEQ_MOVE,     /**< relative mouse motion (dx, dy) — single report, no release */
} hid_seq_kind_t;

/** A single step in a sequence. Union layout keeps the step ~40 B so 16
 *  steps fit one queue slot cheaply. `type.text` is NUL-terminated for
 *  convenience but `type.len` is the source of truth. */
typedef struct {
    hid_seq_kind_t kind;
    union {
        struct {
            uint16_t ms;
        } sleep;
        struct {
            uint8_t  modifiers;  /**< key_mask_t bitmask */
            uint8_t  keycode;    /**< single HID_KEY_* code */
            uint16_t hold_ms;    /**< 0 -> HID_OUTPUT_DEFAULT_HOLD_MS */
        } key;
        struct {
            char     text[HID_SEQ_TEXT_MAX + 1];  /**< NUL-terminated */
            uint8_t  len;
        } type;
        struct {
            uint8_t  buttons;    /**< 1=left 2=right 4=middle, see hid_dev.h */
        } click;
        struct {
            int8_t   dx, dy;     /**< relative motion */
        } move;
    } u;
} hid_seq_step_t;

/**
 * @brief  Queue a multi-step HID script. The worker walks the step array
 *         inline (no per-step queue re-entry) so a 16-step seq doesn't
 *         compete with itself for queue slots. Steps that fire HID reports
 *         still gate on hid_output_is_ready() each iteration, so a mid-seq
 *         disconnect aborts cleanly.
 *
 *         `seq` is a fire-and-forget script — there is no progress
 *         reporting and no abort API. The link dropping mid-seq will be
 *         logged and the rest of the script is skipped. Callers that need
 *         backpressure should use the single-shot hid_output_send_*()
 *         helpers instead.
 *
 * @param  steps  array of steps; copied into the queue item.
 * @param  n      number of steps (clamped to HID_SEQ_MAX_STEPS).
 *
 * @return ESP_OK on enqueue, ESP_ERR_INVALID_ARG if n==0, ESP_ERR_INVALID_STATE
 *         if HID is not ready (link not connected or not bonded).
 */
esp_err_t hid_output_send_seq(const hid_seq_step_t *steps, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* HID_OUTPUT_H_ */
