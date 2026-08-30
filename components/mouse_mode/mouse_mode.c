/*
 * mouse_mode.c — head-tracking mouse cursor control.
 *
 * Active when the user triggers mouse mode via the tilt_left + tilt_right
 * toggle gesture. Head pitch/roll directly controls the cursor via a
 * dead-zone + acceleration response curve.
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

/* ===== Default parameters ================================================ */

#define MOUSE_DEFAULT_DEAD_ZONE_DEG   0.3f   /*!< velocity dead zone °/frame — filters jitter */
#define MOUSE_DEFAULT_SPEED_REF_DEG   3.0f   /*!< °/frame that maps to max_speed */
#define MOUSE_DEFAULT_MAX_SPEED      60.0f
#define MOUSE_DEFAULT_DWELL_MS     0       /*!< 0 = deactivate immediately */

/* ===== Module state ====================================================== */

typedef enum {
    MM_IDLE = 0,        /*!< not active */
    MM_ACTIVE,          /*!< actively controlling cursor */
} mm_state_t;

typedef struct {
    mm_state_t          state;
    bool                enabled;        /*!< toggle detection enabled */
    float               q_prev[4];      /*!< previous frame's quaternion (for incremental delta) */
    bool                q_prev_valid;
    mouse_mode_params_t params;

    /* Dwell detection for deactivation */
    uint32_t            dwell_since_ms; /*!< tick when dwell began; 0 = not dwelling */
    bool                dwell_active;   /*!< true while monitoring for dwell exit */
} mm_t;

static mm_t s_mm;

/* ===== Quaternion helpers (local copies — same as gesture_detect) ======== */

static inline float absf(float v) { return v < 0.0f ? -v : v; }

static void quat_conj(const float a[4], float out[4])
{
    out[0] = a[0]; out[1] = -a[1]; out[2] = -a[2]; out[3] = -a[3];
}

static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void quat_normalize(float a[4])
{
    float n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
    if (n < 1e-9f) { a[0] = 1.0f; a[1] = a[2] = a[3] = 0.0f; return; }
    float inv = 1.0f / n;
    a[0] *= inv; a[1] *= inv; a[2] *= inv; a[3] *= inv;
}

static inline float v3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/* Rotate vector v by quaternion q: out = q ⊗ (0,v) ⊗ conj(q). */
static void quat_rotate_vec(const float q[4], const float v[3], float out[3])
{
    float qv[4]  = { 0.0f, v[0], v[1], v[2] };
    float qc[4]; quat_conj(q, qc);
    float t[4];  quat_mul(q, qv, t);
    float r[4];  quat_mul(t, qc, r);
    out[0] = r[1]; out[1] = r[2]; out[2] = r[3];
}

/**
 * @brief Rotate 3 signature axes from q_drift frame to a target frame.
 *
 *        Signatures are captured during calibration in the q_drift frame.
 *        For mouse mode, r_delta is computed relative to q_prev (≈ qcur),
 *        so we rotate the axes into the same frame.
 *
 *        Rotation: q_td = conj(q_target) ⊗ q_drift
 *        For each axis: axis_target = rotate(axis_drift, q_td)
 */
static void rotate_sigs_to_mouse_frame(const float q_mouse_rest[4],
                                       const float q_drift[4],
                                       const msig_axes_t *sigs,
                                       float nod_out[3],
                                       float tiltL_out[3],
                                       float tiltR_out[3])
{
    float qr_conj[4]; quat_conj(q_mouse_rest, qr_conj);
    float q_md[4];    quat_mul(qr_conj, q_drift, q_md);
    quat_normalize(q_md);
    quat_rotate_vec(q_md, sigs->sig_nod,   nod_out);
    quat_rotate_vec(q_md, sigs->sig_tiltL, tiltL_out);
    quat_rotate_vec(q_md, sigs->sig_tiltR, tiltR_out);
}

/* ===== Public API ======================================================== */

esp_err_t mouse_mode_init(void)
{
    memset(&s_mm, 0, sizeof(s_mm));
    s_mm.state = MM_IDLE;
    s_mm.enabled = true;   /* toggle detection on by default */
    s_mm.dwell_since_ms = 0;
    s_mm.dwell_active = false;

    /* Default parameters */
    s_mm.params.dead_zone_deg = MOUSE_DEFAULT_DEAD_ZONE_DEG;
    s_mm.params.speed_ref_deg = MOUSE_DEFAULT_SPEED_REF_DEG;
    s_mm.params.max_speed     = MOUSE_DEFAULT_MAX_SPEED;
    s_mm.params.dwell_ms      = MOUSE_DEFAULT_DWELL_MS;

    ESP_LOGI(TAG, "mouse_mode init: dz=%.1f ref=%.1f max=%.0f dwell=%u",
             s_mm.params.dead_zone_deg, s_mm.params.speed_ref_deg,
             s_mm.params.max_speed, (unsigned)s_mm.params.dwell_ms);
    return ESP_OK;
}

esp_err_t mouse_mode_activate(void)
{
    if (s_mm.state == MM_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_mm.q_prev_valid = false;
    s_mm.dwell_since_ms = 0;
    s_mm.dwell_active = false;
    s_mm.state = MM_ACTIVE;
    ESP_LOGI(TAG, "mouse_mode ACTIVATED — head controls cursor");
    ble_console_logf("[MOUSE] ACTIVATED\n");
    return ESP_OK;
}

esp_err_t mouse_mode_deactivate(void)
{
    if (s_mm.state == MM_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }
    s_mm.state = MM_IDLE;
    s_mm.q_prev_valid = false;
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

/**
 * @brief Check if the head is still (for dwell detection).
 *
 *        "Still" means the rotation magnitude from the rest reference
 *        is small AND the angular velocity is low.
 */
static bool is_still(const float r[3], float r_mag, float vel)
{
    /* Use a relaxed still threshold — slightly larger than the dead zone
     * so the cursor stops moving but the head doesn't have to be frozen. */
    const float STILL_ANGLE = 5.0f;   /* degrees from rest */
    const float STILL_VEL   = 20.0f;  /* °/s */
    return (r_mag < STILL_ANGLE) && (vel < STILL_VEL);
}

void mouse_mode_tick(const float qcur[4], const float q_drift[4],
                     float r_mag, float vel)
{
    if (s_mm.state != MM_ACTIVE) {
        return;
    }

    /* First frame: capture baseline. Don't send HID — just establish
     * the starting point for incremental rotation tracking. */
    if (!s_mm.q_prev_valid) {
        memcpy(s_mm.q_prev, qcur, sizeof(s_mm.q_prev));
        s_mm.q_prev_valid = true;
        ESP_LOGI(TAG, "baseline captured: q=[%.3f %.3f %.3f %.3f]",
                 qcur[0], qcur[1], qcur[2], qcur[3]);
        return;
    }

    /* Compute incremental rotation: delta = conj(q_prev) × qcur.
     * This gives the rotation that happened in the last 20 ms.
     * Converted to a rotation vector, its magnitude is the angular
     * velocity (degrees/frame) and its direction is the rotation axis. */
    float qc[4];
    quat_conj(s_mm.q_prev, qc);
    float qdelta[4];
    quat_mul(qc, qcur, qdelta);
    quat_normalize(qdelta);

    /* Save current quaternion for next frame */
    memcpy(s_mm.q_prev, qcur, sizeof(s_mm.q_prev));

    /* Convert qdelta to rotation vector (degrees) */
    float w = qdelta[0];
    if (w < 0.0f) { w = -w; }
    if (w > 1.0f) w = 1.0f;
    float s = sqrtf(1.0f - w*w);
    float angle_deg = 2.0f * acosf(w) * 57.29578f;
    float r_delta[3];
    if (s < 1e-6f) {
        r_delta[0] = qdelta[1] * 2.0f * 57.29578f;
        r_delta[1] = qdelta[2] * 2.0f * 57.29578f;
        r_delta[2] = qdelta[3] * 2.0f * 57.29578f;
    } else {
        float k = angle_deg / s;
        r_delta[0] = qdelta[1] * k;
        r_delta[1] = qdelta[2] * k;
        r_delta[2] = qdelta[3] * k;
    }

    /* Rotate the 3 signature axes from q_drift frame to current qcur frame.
     * We need the axes in the same frame as r_delta (which is relative to
     * q_prev, approximately qcur for small deltas). */
    const msig_axes_t *sigs = gesture_detect_get_sig_axes();
    if (sigs == NULL) return;
    float nod_a[3], tiltL_a[3], tiltR_a[3];
    rotate_sigs_to_mouse_frame(qcur, q_drift, sigs,
                               nod_a, tiltL_a, tiltR_a);

    /* Project incremental rotation onto axes → angular velocity (°/frame) */
    float vel_nod   = v3_dot(r_delta, nod_a);
    float vel_tiltL = v3_dot(r_delta, tiltL_a);
    float vel_tiltR = v3_dot(r_delta, tiltR_a);

    /* Pitch velocity: positive = chin-down = cursor down */
    float pitch_vel = vel_nod;

    /* Roll velocity: tiltL - tiltR, negated so left = cursor left */
    float roll_vel = -(vel_tiltL - vel_tiltR);

    /* Speed mapping: angular velocity → pixel velocity.
     * The raw values are small (~0.1-5°/frame at 50 Hz).
     * dead_zone = velocity dead zone (°/frame), not angle.
     * speed_ref = velocity that gives max_speed (°/frame). */
    float dx_f = 0.0f;
    float dy_f = 0.0f;

    float ramp_range = s_mm.params.speed_ref_deg - s_mm.params.dead_zone_deg;
    if (ramp_range < 0.1f) ramp_range = 0.1f;

    float abs_roll = fabsf(roll_vel);
    if (abs_roll > s_mm.params.dead_zone_deg) {
        float speed = s_mm.params.max_speed *
                      (abs_roll - s_mm.params.dead_zone_deg) / ramp_range;
        if (speed > s_mm.params.max_speed) speed = s_mm.params.max_speed;
        dx_f = (roll_vel > 0.0f) ? speed : -speed;
    }

    float abs_pitch = fabsf(pitch_vel);
    if (abs_pitch > s_mm.params.dead_zone_deg) {
        float speed = s_mm.params.max_speed *
                      (abs_pitch - s_mm.params.dead_zone_deg) / ramp_range;
        if (speed > s_mm.params.max_speed) speed = s_mm.params.max_speed;
        dy_f = (pitch_vel > 0.0f) ? speed : -speed;
    }

    /* Convert to integers, clamping to HID range [-127, 127] */
    int dx = (int)dx_f;
    int dy = (int)dy_f;
    if (dx < -127) dx = -127;
    if (dx >  127) dx =  127;
    if (dy < -127) dy = -127;
    if (dy >  127) dy =  127;

    /* Periodic debug: every 50 frames (1 s @ 50 Hz) */
    {
        static uint32_t s_frame = 0;
        s_frame++;
        if ((s_frame % 50) == 1) {
            ESP_LOGI(TAG, "MOUSE dv=[%.2f %.2f %.2f] "
                     "p_v=%.2f r_v=%.2f dx=%d dy=%d",
                     r_delta[0], r_delta[1], r_delta[2],
                     pitch_vel, roll_vel, dx, dy);
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
            ESP_LOGD(TAG, "toggle: LEFT_SEEN at %u", (unsigned)now_ms);
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
                        ESP_LOGD(TAG, "toggle: deactivation ignored — "
                                      "touch not held");
                        s_tg.state = TG_IDLE;
                        break;
                    }
                    if (s_mm.params.dwell_ms == 0) {
                        /* Immediate deactivation */
                        ESP_LOGI(TAG, "toggle: deactivation sequence detected");
                        ble_console_logf("[MOUSE] toggle: deactivation sequence\n");
                        mouse_mode_deactivate();
                    } else {
                        /* Dwell-based deactivation */
                        ESP_LOGI(TAG, "toggle: deactivation sequence detected — "
                                      "waiting for dwell");
                        ble_console_logf("[MOUSE] toggle: deactivation sequence — "
                                         "waiting for dwell (%u ms)\n",
                                         (unsigned)s_mm.params.dwell_ms);
                        s_mm.dwell_active = true;
                        s_mm.dwell_since_ms = 0;
                    }
                } else {
                    /* Activating */
                    ESP_LOGI(TAG, "toggle: activation sequence detected");
                    ble_console_logf("[MOUSE] toggle: activation sequence detected\n");
                    mouse_mode_activate();
                }
            } else {
                ESP_LOGD(TAG, "toggle: window expired (left was %u ms ago)",
                         (unsigned)(now_ms - s_tg.left_seen_ms));
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
            ESP_LOGD(TAG, "toggle: LEFT_SEEN timeout");
            s_tg.state = TG_IDLE;
        }
        break;
    }
}
