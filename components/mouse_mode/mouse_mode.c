/*
 * mouse_mode.c — head-tracking mouse cursor control.
 *
 * Active when the user triggers mouse mode via the tilt_left + tilt_right
 * toggle gesture. Head pitch/roll directly controls the cursor via a
 * dead-zone + linear speed mapping.
 *
 * Cursor displacement is derived from the raw rotation vector r (q_drift
 * frame, in degrees) projected onto the calibrated signature axes.  Both
 * r and the axes share the same frame, so no rotation is needed.  r carries
 * actual angle information and returns to ~0 at rest, which is essential
 * for position-based cursor control.
 *
 * The module does NOT own a FreeRTOS task or read the DMP. It is called
 * from gesture_detect's detector_task at 50 Hz via mouse_mode_tick().
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mouse_mode.h"
#include "hid_output.h"
#include "ble_console.h"
#include "touch_sensor.h"

static const char *TAG = "mouse_mode";

/* Forward declaration: gesture_detect_get_sig_axes() returns a pointer to
 * the 3 calibrated signature axes (in q_drift frame). Defined in
 * gesture_detect.c. The struct matches gesture_sig_axes_t in
 * gesture_detect.h — declared here to avoid circular include deps. */
typedef struct {
    float sig_nod[3];
    float sig_tiltL[3];
    float sig_tiltR[3];
} msig_axes_t;
extern const msig_axes_t *gesture_detect_get_sig_axes(void);

/* Forward declaration: sign_pitch / sign_roll are read from gesture params
 * to normalise the direction convention for mouse cursor axes. */
extern uint8_t gesture_detect_get_sign_pitch(void);
extern uint8_t gesture_detect_get_sign_roll(void);

/* ===== Default parameters ================================================ */

#define MOUSE_DEFAULT_DEAD_ZONE_DEG   2.0f   /*!< position dead zone ° — filters natural sway */
#define MOUSE_DEFAULT_SPEED_REF_DEG   8.0f   /*!< ° from rest that maps to max_speed */
#define MOUSE_DEFAULT_MAX_SPEED      60.0f
#define MOUSE_DEFAULT_DWELL_MS     0       /*!< 0 = deactivate immediately */

#define FOUR_DIR_SPEED             10.0f   /*!< four-dir: constant px/frame */

/* ===== Module state ====================================================== */

typedef enum {
    MM_IDLE = 0,        /*!< not active */
    MM_ACTIVE,          /*!< actively controlling cursor */
} mm_state_t;

typedef struct {
    mm_state_t          state;
    bool                enabled;        /*!< toggle detection enabled */
    bool                four_dir;       /*!< four-direction (d-pad) mode */
    mouse_mode_params_t params;

    /* Dwell detection for deactivation */
    uint32_t            dwell_since_ms; /*!< tick when dwell began; 0 = not dwelling */
    bool                dwell_active;   /*!< true while monitoring for dwell exit */
} mm_t;

static mm_t s_mm;

static inline float absf(float v) { return v < 0.0f ? -v : v; }

static inline float v3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* ===== Public API ======================================================== */

esp_err_t mouse_mode_init(void)
{
    memset(&s_mm, 0, sizeof(s_mm));
    s_mm.state = MM_IDLE;
    s_mm.enabled = false;  /* off by default — enable via "mouse on" / config tool */
    s_mm.dwell_since_ms = 0;
    s_mm.dwell_active = false;

    /* Default parameters */
    s_mm.params.dead_zone_deg = MOUSE_DEFAULT_DEAD_ZONE_DEG;
    s_mm.params.speed_ref_deg = MOUSE_DEFAULT_SPEED_REF_DEG;
    s_mm.params.max_speed     = MOUSE_DEFAULT_MAX_SPEED;
    s_mm.params.dwell_ms      = MOUSE_DEFAULT_DWELL_MS;

    ESP_LOGI(TAG, "mouse_mode init: enabled=%d dz=%.1f ref=%.1f max=%.0f dwell=%u",
             (int)s_mm.enabled, s_mm.params.dead_zone_deg, s_mm.params.speed_ref_deg,
             s_mm.params.max_speed, (unsigned)s_mm.params.dwell_ms);
    return ESP_OK;
}

esp_err_t mouse_mode_activate(void)
{
    if (s_mm.state == MM_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_mm.dwell_since_ms = 0;
    s_mm.dwell_active = false;
    s_mm.state = MM_ACTIVE;
    ESP_LOGI(TAG, "[MOUSE] ACTIVATED — enabled=%d", (int)s_mm.enabled);
    ble_console_logf("[MOUSE] ACTIVATED\n");
    return ESP_OK;
}

esp_err_t mouse_mode_deactivate(void)
{
    if (s_mm.state == MM_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_mm.state = MM_IDLE;
    s_mm.dwell_since_ms = 0;
    s_mm.dwell_active = false;
    ESP_LOGI(TAG, "mouse_mode DEACTIVATED");
    ble_console_logf("[MOUSE] DEACTIVATED\n");
    return ESP_OK;
}

bool mouse_mode_is_active(void)
{
    return s_mm.state == MM_ACTIVE;
}

bool mouse_mode_is_enabled(void)
{
    return s_mm.enabled;
}

void mouse_mode_set_enabled(bool enable)
{
    s_mm.enabled = enable;
    ESP_LOGI(TAG, "toggle detection %s", enable ? "ENABLED" : "DISABLED");
}

void mouse_mode_set_four_dir(bool enable)
{
    s_mm.four_dir = enable;
    ESP_LOGI(TAG, "four-dir mode %s", enable ? "ENABLED" : "DISABLED");
}

bool mouse_mode_is_four_dir(void)
{
    return s_mm.four_dir;
}

const mouse_mode_params_t *mouse_mode_get_params(void)
{
    return &s_mm.params;
}

void mouse_mode_set_params(const mouse_mode_params_t *params)
{
    if (params == NULL) return;
    s_mm.params = *params;
    ESP_LOGI(TAG, "params updated: dz=%.1f ref=%.1f max=%.0f dwell=%u",
             s_mm.params.dead_zone_deg, s_mm.params.speed_ref_deg,
             s_mm.params.max_speed, (unsigned)s_mm.params.dwell_ms);
}

/* ===== Tick: called from detector_task at 50 Hz ========================= */

void mouse_mode_tick(const float r[3], bool r_valid,
                     float r_mag, float vel)
{
    if (s_mm.state != MM_ACTIVE) {
        return;
    }

    if (!r_valid) {
        return;
    }

    const msig_axes_t *sigs = gesture_detect_get_sig_axes();
    if (sigs == NULL) return;

    /* Read sign convention so cursor direction matches the calibrated
     * gesture directions (same sign logic as gesture_detect). */
    const uint8_t sp = gesture_detect_get_sign_pitch();
    const uint8_t sr = gesture_detect_get_sign_roll();

    /* Project r onto signature axes → angular displacement (degrees).
     * r is in the q_drift frame; the signature axes are also in the
     * q_drift frame, so no rotation is needed.  r carries actual angle
     * information and returns to ~0 at rest. */
    float proj_nod   = v3_dot(r, sigs->sig_nod);
    float proj_tiltL = v3_dot(r, sigs->sig_tiltL);

    /* Apply sign convention:
     *   pitch: positive = NOD (chin down)
     *   roll:  positive = LEFT tilt
     *
     * NOTE: only proj_tiltL is used for roll direction.  sign_roll was
     * designed for the tiltL axis; applying it to tiltR would invert
     * the direction. */
    float pitch_disp = sp ? proj_nod : -proj_nod;
    float roll_disp  = sr ? proj_tiltL : -proj_tiltL;

    float abs_pitch = fabsf(pitch_disp);
    float abs_roll  = fabsf(roll_disp);

    int dx = 0, dy = 0;

    if (s_mm.four_dir) {
        /* ── Four-direction (d-pad) mode ──
         * Determine the dominant axis; only move in that axis at a
         * constant slow speed.  This avoids diagonal movement and
         * gives a predictable, easy-to-control cursor. */
        bool pitch_active = abs_pitch > s_mm.params.dead_zone_deg;
        bool roll_active  = abs_roll  > s_mm.params.dead_zone_deg;

        if (pitch_active || roll_active) {
            int speed = (int)FOUR_DIR_SPEED;
            if (pitch_active && abs_pitch >= abs_roll) {
                /* Dominant axis is pitch → vertical only */
                dy = (pitch_disp > 0.0f) ? speed : -speed;
            } else if (roll_active) {
                /* Dominant axis is roll → horizontal only */
                dx = (roll_disp > 0.0f) ? -speed : speed;
            }
        }
    } else {
        /* ── Proportional mode: same constant speed as four-dir,
         *     but both axes can move simultaneously. ── */
        int speed = (int)FOUR_DIR_SPEED;

        if (abs_pitch > s_mm.params.dead_zone_deg) {
            dy = (pitch_disp > 0.0f) ? speed : -speed;
        }

        if (abs_roll > s_mm.params.dead_zone_deg) {
            /* Positive roll_disp = left tilt.
             * In proportional mode the simultaneous pitch movement can
             * make the perceived direction feel wrong, so we negate here
             * to match the user's expectation: tilt left → cursor left. */
            dx = (roll_disp > 0.0f) ? speed : -speed;
        }
    }

    /* Clamp to HID range [-127, 127] */
    if (dx < -127) dx = -127;
    if (dx >  127) dx =  127;
    if (dy < -127) dy = -127;
    if (dy >  127) dy =  127;

    /* Periodic debug: every 50 frames (1 s @ 50 Hz) */
    {
        static uint32_t s_frame = 0;
        s_frame++;
        if ((s_frame % 50) == 1) {
            ESP_LOGI(TAG, "MOUSE r=[%.2f %.2f %.2f] "
                     "p=%.1f r=%.1f dx=%d dy=%d%s",
                     r[0], r[1], r[2],
                     pitch_disp, roll_disp, dx, dy,
                     s_mm.four_dir ? " [4dir]" : "");
        }
    }

    /* Send HID mouse report if there's movement */
    if (dx != 0 || dy != 0) {
        esp_err_t err = hid_output_send_mouse(0, (int8_t)dx, (int8_t)dy);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "mouse report failed: %s", esp_err_to_name(err));
        }
        /* Movement resets the dwell timer — user is still actively moving */
        s_mm.dwell_since_ms = 0;
        s_mm.dwell_active = false;
    } else {
        /* No movement — head is in the dead zone or very close.
         * If the deactivation toggle has been detected (dwell_active),
         * start/continue the dwell timer. */
        if (s_mm.dwell_active) {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (s_mm.dwell_since_ms == 0) {
                s_mm.dwell_since_ms = now_ms;
                ESP_LOGI(TAG, "dwell timer started");
            } else if ((now_ms - s_mm.dwell_since_ms) >= s_mm.params.dwell_ms) {
                ESP_LOGI(TAG, "dwell complete — deactivating mouse mode");
                mouse_mode_deactivate();
                return;
            }
        }
    }
}

/* ===== Toggle detection (called from gesture_detect) ===================== */

/**
 * Toggle state machine for the left+right tilt activation/deactivation
 * gesture. Runs inside gesture_detect's detector_task.
 *
 * States:
 *   IDLE        → TILT_LEFT detected → LEFT_SEEN
 *   LEFT_SEEN   → TILT_RIGHT detected within 1500ms → toggle!
 *                 timeout → reset to IDLE
 *   (any)       → mouse_mode toggled → reset to IDLE
 *
 * In mouse_mode, after toggling, set dwell_active so mouse_mode_tick()
 * monitors for stillness before final deactivation.
 */
typedef enum {
    TG_IDLE = 0,
    TG_LEFT_SEEN,
} toggle_state_t;

typedef struct {
    toggle_state_t state;
    uint32_t       left_seen_ms;   /*!< when TILT_LEFT was detected */
} toggle_ctx_t;

static toggle_ctx_t s_tg;

#define TOGGLE_WINDOW_MS  5000  /*!< max ms between left and right tilt */

void mouse_mode_toggle_reset(void)
{
    s_tg.state = TG_IDLE;
    s_tg.left_seen_ms = 0;
}

/**
 * @brief Process a tilt gesture for toggle detection.
 *
 *        Called from gesture_detect when a TILT_LEFT or TILT_RIGHT event
 *        fires (or would fire — during mouse_mode events are suppressed
 *        but the state machine still runs).
 *
 * @param gesture   GESTURE_TILT_LEFT or GESTURE_TILT_RIGHT
 * @param now_ms    current tick in ms
 */
void mouse_mode_toggle_step(int gesture, uint32_t now_ms)
{
    /* When disabled and not active, don't process toggle gestures.
     * When active, always allow toggle (for deactivation). */
    if (!s_mm.enabled && !mouse_mode_is_active()) {
        return;
    }
    switch (s_tg.state) {
    case TG_IDLE:
        if (gesture == 3) {  /* GESTURE_TILT_LEFT */
            s_tg.state = TG_LEFT_SEEN;
            s_tg.left_seen_ms = now_ms;
        }
        break;

    case TG_LEFT_SEEN:
        if (gesture == 4) {  /* GESTURE_TILT_RIGHT */
            /* Check window */
            if ((now_ms - s_tg.left_seen_ms) <= TOGGLE_WINDOW_MS) {
                /* Toggle! */
                if (mouse_mode_is_active()) {
                    /* Deactivation requires touch held (left click pressed) */
                    if (!touch_sensor_is_pressed()) {
                        ESP_LOGD(TAG, "toggle deactivate ignored — touch not held");
                        s_tg.state = TG_IDLE;
                        break;
                    }
                    if (s_mm.params.dwell_ms == 0) {
                        /* Immediate deactivation */
                        ESP_LOGI(TAG, "toggle deactivate (immediate)");
                        ble_console_logf("[MOUSE] toggle: deactivated\n");
                        mouse_mode_deactivate();
                    } else {
                        /* Dwell-based deactivation */
                        ESP_LOGI(TAG, "toggle deactivate (dwell %u ms)",
                                 (unsigned)s_mm.params.dwell_ms);
                        ble_console_logf("[MOUSE] toggle: deactivating — "
                                         "dwell %u ms\n",
                                         (unsigned)s_mm.params.dwell_ms);
                        s_mm.dwell_active = true;
                        s_mm.dwell_since_ms = 0;
                    }
                } else {
                    /* Activating */
                    ESP_LOGI(TAG, "toggle activate");
                    ble_console_logf("[MOUSE] toggle: activated\n");
                    mouse_mode_activate();
                }
            }
            s_tg.state = TG_IDLE;
        } else if (gesture == 3) {
            /* Another left tilt — restart the window */
            s_tg.left_seen_ms = now_ms;
        } else {
            /* Non-tilt gesture — reset */
            s_tg.state = TG_IDLE;
        }
        /* Also check timeout */
        if (s_tg.state == TG_LEFT_SEEN &&
            (now_ms - s_tg.left_seen_ms) > TOGGLE_WINDOW_MS) {
            s_tg.state = TG_IDLE;
        }
        break;
    }
}
