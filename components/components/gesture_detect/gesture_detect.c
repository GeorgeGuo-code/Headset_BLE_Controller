/*
 * gesture_detect.c — per-axis state-machine detector.
 *
 * The DMP pipeline (inv_mpu.c) feeds pitch / roll / yaw at the FIFO
 * rate (100 Hz). We poll at ~50 Hz. For each axis (pitch, roll) we
 * run an independent three-state machine:
 *
 *      NEUTRAL  ──( |rel| > trigger AND |vel| > trigger_vel )──>  LOCKED_POS/NEG
 *      LOCKED_* ──( |rel| < neutral_zone AND dt > debounce )──>  NEUTRAL
 *
 * Events are pushed to a caller-owned FreeRTOS queue (drop-newest on
 * overflow). Yaw is computed by the DMP but NEVER consumed here.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"

#include "MPU6050.h"
#include "inv_mpu.h"
#include "gesture_detect.h"
#include "gesture_params.h"

static const char *TAG = "gesture_detect";

/* ===== Task timing ======================================================= */
#define GD_TASK_PERIOD_MS    20      /*!< 50 Hz polling */
#define GD_TASK_STACK_WORDS  4096
#define GD_TASK_PRIORITY     5

/* ===== Module state ====================================================== */

typedef enum {
    GD_AXIS_NEUTRAL = 0,
    GD_AXIS_LOCKED_POS,
    GD_AXIS_LOCKED_NEG,
} axis_state_t;

typedef struct {
    axis_state_t    state;
    int32_t         t_enter_ms;       /*!< when current state was entered */
    float           peak_abs_rel;     /*!< max |rel| while in current LOCKED_* */
    float           peak_abs_vel;     /*!< max |vel| while in current LOCKED_* */
} axis_ctx_t;

typedef struct {
    gesture_params_t params;
    axis_ctx_t      pitch;
    axis_ctx_t      roll;
    QueueHandle_t   event_queue;
    TaskHandle_t    task;
    uint32_t        cooldown_until_ms;   /*!< tick-count ms; no axis may emit_event() before this */
    volatile bool   calibrating;         /*!< set by calibrate_* during their DMP-loop window */
    bool            running;
    bool            calibrated;   /*!< true once neutral + axes (and, ideally, tilt) calibration finished */

    /* ===== Phase 5: q_drift sliding baseline ============================
     * `q_neutral` is captured once at calibration and never updated.
     * After wearing the device for a while the head settles a few degrees
     * away from that snapshot, the nod projects partly onto the tilt axis,
     * and the detector starts firing the wrong gesture. q_drift is the
     * detector's runtime baseline: seeded from q_neutral at apply_params(),
     * snapped to the current quaternion after STILL_DURATION_MS of rest,
     * and held fixed during motion so gestures register as relative
     * displacement. The nod/tilt axes stay expressed in the q_neutral
     * frame (still meaningful as long as q_drift stays close to it). */
    float           q_drift[4];          /*!< runtime "neutral" pose (w,x,y,z) */
    bool            q_drift_valid;       /*!< false until detector has a fresh sample after apply_params */
    uint32_t        still_since_ms;      /*!< ms tick at which stillness began; 0 = not still */
} gd_t;

/* Diagnostic snapshot of the most recent axis calibration, exposed via the
 * `cd` BLE command. Used in the v3 prototype to verify that the Δr-average
 * algorithm produces a stable direction across runs without needing to
 * instrument the detector task itself. */
typedef struct {
    uint32_t valid;             /*!< samples kept after DMP-glitch rejection */
    uint32_t used;              /*!< samples contributing to the average */
    float    nod_axis[3];       /*!< persisted nod_axis (device frame) */
    float    tilt_axis[3];      /*!< persisted tilt_axis (device frame) */
    float    sum_mag_deg;       /*!< Σ|Δr| over the kept samples */
    float    drift_deg;         /*!< max |Δr − mean(Δr)| — a small drift means the user's
                                     motion was consistent; large means the user wasn't doing the
                                     same gesture repeatedly. */
} last_capture_t;

static last_capture_t s_last_cap;

static gd_t s_gd;

/* ===== small math ======================================================== */

static inline float absf(float v) { return v < 0.0f ? -v : v; }

/* ---- vec3 helpers (arrays: {x,y,z}) ---- */
static inline float v3_dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline void v3_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float v3_norm(const float a[3])
{
    return sqrtf(v3_dot(a, a));
}
/* Normalize in place; returns the original length (0 if degenerate). */
static float v3_normalize(float a[3])
{
    float n = v3_norm(a);
    if (n < 1e-9f) { return 0.0f; }
    float inv = 1.0f / n;
    a[0] *= inv; a[1] *= inv; a[2] *= inv;
    return n;
}

/* ---- quaternion helpers (arrays: {w,x,y,z}, Hamilton product) ---- */
static void quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
static void quat_conj(const float a[4], float out[4])
{
    out[0] = a[0]; out[1] = -a[1]; out[2] = -a[2]; out[3] = -a[3];
}
static void quat_normalize(float a[4])
{
    float n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
    if (n < 1e-9f) { a[0] = 1.0f; a[1] = a[2] = a[3] = 0.0f; return; }
    float inv = 1.0f / n;
    a[0] *= inv; a[1] *= inv; a[2] *= inv; a[3] *= inv;
}

/* Rotation angle of a (unit) quaternion, in degrees, always in [0,180]. */
static float quat_angle_deg(const float q[4])
{
    float w = absf(q[0]);
    if (w > 1.0f) { w = 1.0f; }
    return 2.0f * acosf(w) * 57.29578f;
}

/* Convert a (unit) quaternion to a rotation vector (axis * angle) in
 * degrees. w is forced non-negative so the result is the shortest
 * rotation. Small-angle branch avoids the sin() blow-up. */
static void quat_to_rotvec_deg(const float q_in[4], float out[3])
{
    float q[4] = { q_in[0], q_in[1], q_in[2], q_in[3] };
    if (q[0] < 0.0f) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
    float w = q[0] > 1.0f ? 1.0f : q[0];
    float s = sqrtf(1.0f - w*w);              /* |vector part| = sin(angle/2) */
    float angle_deg = 2.0f * acosf(w) * 57.29578f;
    if (s < 1e-6f) {
        /* angle ~ 0: rotvec ≈ 2 * vector part (in radians) → degrees */
        out[0] = q[1] * 2.0f * 57.29578f;
        out[1] = q[2] * 2.0f * 57.29578f;
        out[2] = q[3] * 2.0f * 57.29578f;
        return;
    }
    float k = angle_deg / s;
    out[0] = q[1] * k;
    out[1] = q[2] * k;
    out[2] = q[3] * k;
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

/* Apply sign convention: flips the projection so that a positive value
 * always corresponds to the user's gesture of choice. */
static inline float apply_sign_pitch(float rel, uint8_t sign_pitch)
{
    return sign_pitch ? rel : -rel;
}
static inline float apply_sign_roll(float rel, uint8_t sign_roll)
{
    return sign_roll ? rel : -rel;
}

/* ===== Defaults / NVS coupling =========================================== */

esp_err_t gesture_detect_init(void)
{
    memset(&s_gd, 0, sizeof(s_gd));
    s_gd.pitch.state = GD_AXIS_NEUTRAL;
    s_gd.roll.state  = GD_AXIS_NEUTRAL;

    gesture_params_t loaded;
    esp_err_t err = gesture_params_load_from_nvs(&loaded);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS params unavailable (%s); using defaults",
                 err == ESP_ERR_NVS_NOT_FOUND ? "empty" : "corrupt");
        gesture_params_load_default(&loaded);
    } else {
        ESP_LOGI(TAG, "params loaded from NVS: trigger=%.1f vel=%.1f zone=%.1f debounce=%u",
                 loaded.trigger_deg, loaded.trigger_velocity_deg_s,
                 loaded.neutral_zone_deg, (unsigned)loaded.debounce_ms);
    }
    return gesture_detect_apply_params(&loaded);
}

esp_err_t gesture_detect_apply_params(const gesture_params_t *params)
{
    if (params == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (params->magic != GESTURE_PARAMS_MAGIC ||
        params->version != GESTURE_PARAMS_VERSION) {
        return ESP_ERR_INVALID_ARG;
    }
    s_gd.params = *params;
    /* Phase 5: seed the sliding baseline from the freshly-applied neutral
     * pose and force the detector to re-sync q_drift from the next DMP
     * sample. Without the q_drift_valid reset the detector would still be
     * using the old q_drift from the previous calibration run, and the
     * very first tick would produce a huge r-vector (old baseline vs. new
     * q_neutral) and either miss-fire or instantly snap to the wrong pose. */
    memcpy(s_gd.q_drift, params->neutral.q_neutral, sizeof(s_gd.q_drift));
    s_gd.q_drift_valid  = false;
    s_gd.still_since_ms = 0;
    /* The neutral pose is packed; copy to an aligned mirror before passing
     * to log helpers. */
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    ESP_LOGI(TAG, "params applied: trigger=%.1f vel=%.1f zone=%.1f debounce=%u "
                  "sign_pitch=%u sign_roll=%u",
             s_gd.params.trigger_deg, s_gd.params.trigger_velocity_deg_s,
             s_gd.params.neutral_zone_deg, s_gd.params.debounce_ms,
             (unsigned)s_gd.params.sign_pitch, (unsigned)s_gd.params.sign_roll);
    ESP_LOGI(TAG, "  q_neutral=[%.3f %.3f %.3f %.3f] nod_axis=[%.2f %.2f %.2f] tilt_axis=[%.2f %.2f %.2f]",
             np.q_neutral[0], np.q_neutral[1], np.q_neutral[2], np.q_neutral[3],
             np.nod_axis[0], np.nod_axis[1], np.nod_axis[2],
             np.tilt_axis[0], np.tilt_axis[1], np.tilt_axis[2]);
    return ESP_OK;
}

const gesture_params_t *gesture_detect_get_params(void)
{
    return &s_gd.params;
}

void gesture_detect_set_sign(bool positive_pitch_is_nod, bool positive_roll_is_right)
{
    s_gd.params.sign_pitch = positive_pitch_is_nod ? 1 : 0;
    s_gd.params.sign_roll  = positive_roll_is_right ? 1 : 0;
}

void gesture_detect_get_q_drift(float out[4])
{
    if (out == NULL) {
        return;
    }
    memcpy(out, s_gd.q_drift, sizeof(s_gd.q_drift));
}

/**
 * @brief Phase 5: force q_drift to re-sync from the next DMP sample.
 *        Use when the device佩戴微调 has drifted far enough that the
 *        still-snap will take too long to catch up (e.g. the user took
 *        the device off and put it back on at a very different angle).
 *        The next detector tick copies qcur into q_drift and the still
 *        counter resets. nod/tilt axes are unchanged. */
void gesture_detect_reset_q_drift(void)
{
    s_gd.q_drift_valid  = false;
    s_gd.still_since_ms = 0;
}

/* ===== Event helper ====================================================== */

static void emit_event(gesture_type_t type, float peak_angle, float peak_vel)
{
    if (s_gd.event_queue == NULL) {
        return;
    }
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    gesture_event_t ev = {
        .type              = type,
        .timestamp_ms      = now_ms,
        .peak_angle_deg    = peak_angle,
        .peak_velocity_deg_s = peak_vel,
    };
    if (xQueueSend(s_gd.event_queue, &ev, 0) != pdTRUE) {
        /* queue full — drop newest. Bridge task is too slow; nothing
         * the detector can do but keep state machine coherent. */
        ESP_LOGW(TAG, "event queue full, dropping type=%d", (int)type);
        return;
    }
    /* After firing, lock all axes for GESTURE_DEFAULT_COOLDOWN_MS. This
     * kills both (a) the same-tick race that the per-tick mutex already
     * covers and (b) the multi-tick case where a return-swing or
     * immediately-following motion would have re-entered the trigger
     * threshold on a different axis within a few hundred milliseconds.
     * The state machines continue to update normally — only emits are
     * suppressed — and step_axis() rewinds a suppressed lock to NEUTRAL
     * so each axis re-arms cleanly once cooldown ends. */
    uint32_t cooldown_ms = GESTURE_DEFAULT_COOLDOWN_MS;
    s_gd.cooldown_until_ms = now_ms + cooldown_ms;
}

/* ===== Axis step ========================================================= */

/**
 * @brief One sample's worth of work for a single axis. Updates peak
 *        tracking, transitions states, fires events on NEUTRAL→LOCKED.
 *
 * @param[in] allow_fire when false, the axis still updates its state
 *                       (so peak tracking and lock return work normally)
 *                       but emit_event() is suppressed. Two things set
 *                       `allow_fire = false`:
 *                         - the single-event mutex in detector_task:
 *                           the other axis is the dominant one this tick
 *                         - the global cooldown (see emit_event): any
 *                           axis that tries to fire within
 *                           GESTURE_DEFAULT_COOLDOWN_MS of a prior fire
 *                           is suppressed here, regardless of allow_fire
 *
 * @return true if a NEUTRAL→LOCKED transition fired an event this tick.
 */
static bool step_axis(axis_ctx_t *ctx,
                      float signed_rel,
                      float abs_velocity,
                      uint32_t now_ms,
                      gesture_type_t pos_gesture,
                      gesture_type_t neg_gesture,
                      bool allow_fire)
{
    const float trigger    = s_gd.params.trigger_deg;
    const float trigger_v  = s_gd.params.trigger_velocity_deg_s;
    const float zone       = s_gd.params.neutral_zone_deg;
    const uint32_t debounce = s_gd.params.debounce_ms;

    switch (ctx->state) {
    case GD_AXIS_NEUTRAL:
        /* Skip trigger check entirely during global cooldown — do NOT rewind
         * to NEUTRAL (we're already there) and do NOT enter LOCKED. This
         * prevents the suppressed axis from immediately re-competing on the
         * very next tick after the dominant axis fired. */
        if (now_ms < s_gd.cooldown_until_ms) {
            break;
        }
        if (absf(signed_rel) > trigger && abs_velocity > trigger_v) {
            ctx->state         = (signed_rel > 0) ? GD_AXIS_LOCKED_POS
                                                  : GD_AXIS_LOCKED_NEG;
            ctx->t_enter_ms    = now_ms;
            ctx->peak_abs_rel  = absf(signed_rel);
            ctx->peak_abs_vel  = abs_velocity;
            if (allow_fire) {
                emit_event((signed_rel > 0) ? pos_gesture : neg_gesture,
                           ctx->peak_abs_rel, ctx->peak_abs_vel);
                return true;
            }
            /* Suppressed by cross-axis mutex: rewind so this axis re-arms
             * cleanly for the next genuine gesture. */
            ctx->state = GD_AXIS_NEUTRAL;
            ctx->peak_abs_rel = 0.0f;
            ctx->peak_abs_vel = 0.0f;
        }
        break;

    case GD_AXIS_LOCKED_POS:
    case GD_AXIS_LOCKED_NEG:
        if (absf(signed_rel) > ctx->peak_abs_rel) {
            ctx->peak_abs_rel = absf(signed_rel);
        }
        if (abs_velocity > ctx->peak_abs_vel) {
            ctx->peak_abs_vel = abs_velocity;
        }
        if (absf(signed_rel) < zone &&
            (now_ms - ctx->t_enter_ms) >= debounce) {
            ctx->state = GD_AXIS_NEUTRAL;
        }
        break;
    }
    return false;
}

/* ===== Detector task ==================================================== */

static void detector_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    float prev_nod = 0.0f, prev_tilt = 0.0f;
    bool  prev_valid = false;

    while (s_gd.running) {
        /* During a calibration (calibrate_neutral / _axes / _tilt) the
         * detector must stay out of the DMP FIFO or it races the
         * calibrator. vTaskSuspend turned out to break FIFO reads in
         * this environment, so we use a plain flag: the detector
         * skips this tick and yields back. DMP FIFO packets produced
         * during the skip window sit in the FIFO buffer, never lost. */
        if (s_gd.calibrating) {
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        float qcur[4];
        if (mpu_dmp_get_quat(&qcur[0], &qcur[1], &qcur[2], &qcur[3]) != 0) {
            /* FIFO miss — skip this tick */
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* Phase 5: first sample after apply_params() seeds q_drift from
         * the current pose and skips processing for this tick — we have
         * no prior projection state for the velocity estimate and r itself
         * would be qcur-vs-qcur (zero) anyway. Subsequent ticks compute r
         * against q_drift, which slowly tracks佩戴微调. */
        if (!s_gd.q_drift_valid) {
            memcpy(s_gd.q_drift, qcur, sizeof(s_gd.q_drift));
            s_gd.q_drift_valid  = true;
            s_gd.still_since_ms = 0;
            prev_valid = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        /* Pull a stack-local aligned copy of the neutral pose so the
         * quaternion/vector helpers can take its members as float* without
         * tripping -Werror=address-of-packed-member. The pose is stable
         * for the duration of one tick (calibration only writes when
         * detector_task is between samples). The nod/tilt axes are still
         * expressed in the q_neutral frame — that's fine as long as
         * q_drift doesn't stray far from q_neutral, which the still-snap
         * guarantees by keeping q_drift ≈ current pose whenever the user
         * is at rest. */
        neutral_pose_aligned_t np;
        gesture_params_get_neutral_aligned(&np);

        /* Relative rotation from the runtime baseline q_drift, expressed
         * as a rotation vector (axis * angle, degrees). Replaces the old
         * q_neutral-relative r — same projection math, but the baseline
         * now follows佩戴微调 instead of being frozen at calibration. */
        float qd_conj[4]; quat_conj(s_gd.q_drift, qd_conj);
        float qrel[4];    quat_mul(qd_conj, qcur, qrel);
        quat_normalize(qrel);
        float r[3];       quat_to_rotvec_deg(qrel, r);

        /* Project onto the calibrated gesture axes, then apply sign. The
         * projection replaces the old rel_pitch / rel_roll channels. */
        float proj_nod  = v3_dot(r, np.nod_axis);
        float proj_tilt = v3_dot(r, np.tilt_axis);
        proj_nod  = apply_sign_pitch(proj_nod,  s_gd.params.sign_pitch);
        proj_tilt = apply_sign_roll (proj_tilt, s_gd.params.sign_roll);

        /* Derivative-based velocity (degrees/sec). First sample uses 0. */
        float vel_nod = 0.0f, vel_tilt = 0.0f;
        if (prev_valid) {
            float dt = (float)GD_TASK_PERIOD_MS / 1000.0f;
            vel_nod  = (proj_nod  - prev_nod)  / dt;
            vel_tilt = (proj_tilt - prev_tilt) / dt;
        }
        prev_nod   = proj_nod;
        prev_tilt  = proj_tilt;
        prev_valid = true;

        /* ----- diagnostic dump (rate-limited, motion-gated) -----------
         * Logs all four signals at 10 Hz, but only while one of the
         * projections exceeds 2° from neutral. Suppressed entirely
         * until gesture_detect_calibrate_axes() has completed — the
         * pre-calibration projections reference placeholder axes and
         * are not meaningful. Purpose: diagnose left/right tilt
         * asymmetry by comparing the actual peak magnitudes each side
         * achieves on raw input. Remove this block once the asymmetry
         * is resolved. */
        if (s_gd.calibrated) {
            static uint32_t last_dbg_ms = 0;
            static float    dbg_peak_nod  = 0.0f;
            static float    dbg_peak_tilt = 0.0f;
            if (absf(proj_nod) > 2.0f || absf(proj_tilt) > 2.0f) {
                if (absf(proj_nod)  > dbg_peak_nod)  dbg_peak_nod  = absf(proj_nod);
                if (absf(proj_tilt) > dbg_peak_tilt) dbg_peak_tilt = absf(proj_tilt);
                if ((now_ms - last_dbg_ms) >= 100) {
                    ESP_LOGI(TAG, "DBG t=%u nod=%+.1f(v%+.0f) tilt=%+.1f(v%+.0f) peak:nod=%.1f tilt=%.1f",
                             now_ms, proj_nod, vel_nod, proj_tilt, vel_tilt,
                             dbg_peak_nod, dbg_peak_tilt);
                    last_dbg_ms = now_ms;
                }
            } else {
                /* Returned to rest: report peaks then reset, so the user
                 * sees "this gesture reached X° / Y°" after each motion. */
                if (dbg_peak_nod > 2.0f || dbg_peak_tilt > 2.0f) {
                    ESP_LOGI(TAG, "DBG rest  peak:nod=%.1f tilt=%.1f (sign_tilt=%s)",
                             dbg_peak_nod, dbg_peak_tilt,
                             (proj_tilt > 0.0f) ? "+" :
                             ((proj_tilt < 0.0f) ? "-" : "0"));
                }
                dbg_peak_nod  = 0.0f;
                dbg_peak_tilt = 0.0f;
                last_dbg_ms   = now_ms;
            }
        }

        /* Single-event mutex. A nod with a small roll component used to
         * fire BOTH NOD and TILT_RIGHT in the same tick. We pick the
         * axis with the higher peak velocity (the dominant component of
         * the motion) and suppress the other axis's emit_event() this
         * tick — but only on the very first transition. If the other
         * axis is already mid-lock from a previous gesture, that one
         * isn't suppressed (its fire already happened earlier). The
         * step_axis() rewinds suppressed axes to NEUTRAL so they
         * remain ready for the next genuine gesture.
         *
         * Before both calibration steps have completed, the gesture axes
         * are placeholder values, so the projections are meaningless.
         * Suppress all event emission until then. */
        if (!s_gd.calibrated) {
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        /* Dominance gate. A real nod/look-up carries a tilt crosstalk that can
         * be 40-70% of the nod magnitude (a person doesn't nod on a perfect
         * vertical axis). The old per-tick mutex picked whichever axis was
         * faster, but a tilt axis can briefly cross its trigger threshold
         * before the nod does on a nod motion, firing the wrong label.
         *
         * Require the dominant axis to exceed the other by 1.4× before any
         * NEUTRAL→LOCKED transition. Mixed-direction motions (e.g. nodding
         * with a clear left lean) get re-decided next tick when one axis
         * clearly leads. */
        const float DOMINANCE = 1.4f;
        bool nod_dominant;
        if (absf(vel_nod) < 1.0f && absf(vel_tilt) < 1.0f) {
            nod_dominant = absf(vel_nod) >= absf(vel_tilt);
        } else {
            float ratio_nt = absf(vel_nod)  / (absf(vel_tilt) + 1e-3f);
            float ratio_tn = absf(vel_tilt) / (absf(vel_nod)  + 1e-3f);
            if (ratio_nt >= DOMINANCE) {
                nod_dominant = true;
            } else if (ratio_tn >= DOMINANCE) {
                nod_dominant = false;
            } else {
                /* Neither axis dominates — mixed motion. Hold fire this tick
                 * and let the per-axis velocity check resolve on the next one. */
                vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
                continue;
            }
        }
        if (nod_dominant) {
            step_axis(&s_gd.pitch, proj_nod, absf(vel_nod), now_ms,
                      GESTURE_NOD, GESTURE_LOOK_UP, /* allow_fire = */ true);
            step_axis(&s_gd.roll,  proj_tilt, absf(vel_tilt), now_ms,
                      GESTURE_TILT_RIGHT, GESTURE_TILT_LEFT,
                      /* allow_fire = */ false);
        } else {
            step_axis(&s_gd.roll,  proj_tilt, absf(vel_tilt), now_ms,
                      GESTURE_TILT_RIGHT, GESTURE_TILT_LEFT, /* allow_fire = */ true);
            step_axis(&s_gd.pitch, proj_nod, absf(vel_nod), now_ms,
                      GESTURE_NOD, GESTURE_LOOK_UP, /* allow_fire = */ false);
        }

        /* Phase 5: sliding baseline snap. After STILL_DURATION_MS of rest
         * (both projections inside the dead zone), snap q_drift to the
         * current quaternion so佩戴微调 is absorbed. The snap respects
         * the q/-q hemisphere to avoid the wrap-around the DMP occasionally
         * produces across the ±π boundary. We also reset the still timer
         * after each snap so the next snap requires another full window of
         * stillness — otherwise a long stationary period would snap on
         * every tick, which is harmless but noisy in the still_since_ms
         * accounting. */
        const uint32_t STILL_DURATION_MS = 500;
        const float zone_for_still = s_gd.params.neutral_zone_deg;
        bool is_still = (absf(proj_nod) < zone_for_still) &&
                        (absf(proj_tilt) < zone_for_still);
        if (is_still) {
            if (s_gd.still_since_ms == 0) {
                s_gd.still_since_ms = now_ms;
            } else if ((now_ms - s_gd.still_since_ms) >= STILL_DURATION_MS) {
                float d = qcur[0]*s_gd.q_drift[0] + qcur[1]*s_gd.q_drift[1] +
                          qcur[2]*s_gd.q_drift[2] + qcur[3]*s_gd.q_drift[3];
                if (d < 0.0f) {
                    s_gd.q_drift[0] = -qcur[0]; s_gd.q_drift[1] = -qcur[1];
                    s_gd.q_drift[2] = -qcur[2]; s_gd.q_drift[3] = -qcur[3];
                } else {
                    memcpy(s_gd.q_drift, qcur, sizeof(s_gd.q_drift));
                }
                s_gd.still_since_ms = now_ms;   /* re-arm the next still window */
            }
        } else {
            s_gd.still_since_ms = 0;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
    }
    s_gd.task = NULL;
    vTaskDelete(NULL);
}

esp_err_t gesture_detect_start(QueueHandle_t event_queue)
{
    if (event_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_gd.running) {
        return ESP_ERR_INVALID_STATE;
    }
    s_gd.event_queue = event_queue;
    s_gd.running     = true;
    BaseType_t ok = xTaskCreate(detector_task, "gesture_det",
                                GD_TASK_STACK_WORDS, NULL,
                                GD_TASK_PRIORITY, &s_gd.task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

/* ===== Neutral calibration ============================================== */

/**
 * @brief Block for `duration_ms`, averaging the DMP quaternion, and store
 *        the result as the new neutral orientation `q_neutral`.
 *
 *        Motion guard uses the relative rotation angle of each sample vs
 *        the first sample:
 *          - max deviation > 25°       : abort (user is moving too much)
 *          - max deviation in 10°..25° : WARN but still save (settling)
 *          - < 10°                     : save cleanly
 *
 *        The user MUST trigger this AFTER putting the device on. Follow it
 *        with gesture_detect_calibrate_axes() so the gesture axes match the
 *        (arbitrary) mounting angle.
 */
esp_err_t gesture_detect_calibrate_neutral(uint32_t duration_ms)
{
    if (duration_ms < 200) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "calibrating neutral for %u ms — keep head still...",
             (unsigned)duration_ms);

    const uint32_t period_ms = 20;
    const uint32_t ticks     = duration_ms / period_ms;
    if (ticks < 5) {
        return ESP_ERR_INVALID_ARG;
    }

    float qref[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    bool  have_ref = false;
    float acc[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
    float max_dev_deg = 0.0f;
    uint32_t valid = 0;

    /* Tell the detector to stay out of the FIFO for the duration of the
     * DMP loop (the FIFO is a single-consumer stream — two callers at
     * 50 Hz each desyncs packets). We use a plain flag rather than
     * vTaskSuspend so we don't touch the scheduler state-machine at
     * all; the detector just yields this tick and resumes next time
     * the flag drops. */
    s_gd.calibrating = true;
    esp_err_t result = ESP_OK;

    for (uint32_t i = 0; i < ticks; i++) {
        float q[4];
        if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
            if (!have_ref) {
                memcpy(qref, q, sizeof(qref));
                have_ref = true;
            }
            /* Align hemisphere to qref so the running average doesn't
             * cancel (q and -q represent the same rotation). */
            float d = q[0]*qref[0] + q[1]*qref[1] + q[2]*qref[2] + q[3]*qref[3];
            if (d < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
            acc[0]+=q[0]; acc[1]+=q[1]; acc[2]+=q[2]; acc[3]+=q[3];

            float qc[4]; quat_conj(qref, qc);
            float qd[4]; quat_mul(qc, q, qd); quat_normalize(qd);
            float dev = quat_angle_deg(qd);
            if (dev > max_dev_deg) { max_dev_deg = dev; }
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    /* Drop the flag BEFORE validation/save so even an aborted calibration
     * leaves the detector responsive. The detector resumes on its next
     * 20 ms tick. */
    s_gd.calibrating = false;

    if (valid < ticks / 2) {
        ESP_LOGE(TAG, "calibration failed: too few valid samples (%u/%u)",
                 (unsigned)valid, (unsigned)ticks);
        return ESP_FAIL;
    }
    if (max_dev_deg > 25.0f) {
        ESP_LOGE(TAG, "calibration aborted: too much motion (dev %.1f°)", max_dev_deg);
        return ESP_FAIL;
    }

    quat_normalize(acc);   /* averaged & renormalized neutral quaternion */
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    memcpy(np.q_neutral, acc, sizeof(np.q_neutral));
    gesture_params_set_neutral_aligned(&np);
    /* set_neutral_aligned already persists to NVS. */

    if (max_dev_deg > 10.0f) {
        ESP_LOGW(TAG, "neutral captured with motion (dev=%.1f°): "
                      "q=[%.3f %.3f %.3f %.3f] — consider re-calibrating while still",
                 max_dev_deg, acc[0], acc[1], acc[2], acc[3]);
    } else {
        ESP_LOGI(TAG, "neutral captured: q=[%.3f %.3f %.3f %.3f] (%u valid samples)",
                 acc[0], acc[1], acc[2], acc[3], (unsigned)valid);
    }
    return ESP_OK;
}

/**
 * @brief Derive the two gesture axes from a live nod. Requires a valid
 *        `q_neutral` (call gesture_detect_calibrate_neutral first). The
 *        user should perform one or more slow, deliberate nods during the
 *        window.
 *
 *        Method: for each sample compute the relative rotation vector
 *        r = rotvec(conj(q_neutral) ⊗ q). Keep the sample with the largest
 *        |r| (the nod peak). Project out the vertical ("up") component so
 *        nod_axis lies in the horizontal plane, then tilt_axis = up × nod.
 *        + nod_axis points in the direction the user actually nodded
 *        (chin-down), so positive projection = NOD.
 */
esp_err_t gesture_detect_calibrate_axes(uint32_t duration_ms)
{
    if (duration_ms < 500) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "calibrating axes for %u ms — do a few slow, full nods "
                  "(v3: q_neutral-relative r vectors, averaged)",
             (unsigned)duration_ms);

    /* "up" in the q_neutral body frame: world up rotated into the device's
     * body frame at the moment the user calibrated neutral. Used to project
     * the averaged rotation vector onto the horizontal plane so nod_axis
     * has no vertical component. The detector still consumes nod_axis in
     * this same q_neutral-relative frame, so as long as q_neutral hasn't
     * changed since calibration the projections will be correct. */
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    float qn_conj[4]; quat_conj(np.q_neutral, qn_conj);
    float up[3];      quat_rotate_vec(qn_conj, world_up, up);
    if (v3_normalize(up) == 0.0f) {
        ESP_LOGE(TAG, "axis calibration aborted: bad neutral quaternion");
        return ESP_FAIL;
    }

    const uint32_t period_ms = 20;
    const uint32_t ticks     = duration_ms / period_ms;
    const float W_MIN           = 0.05f;
    const float R_MAX_PER_FRAME = 90.0f;
    const float R_MOTION_MIN    = 4.0f;   /* |r| above this counts as "in motion" */
    const float R_PROJ_MIN      = 2.0f;   /* |proj_horizontal| above this counts as a nod-direction sample */
    const float SUM_MAG_MIN_DEG = 10.0f;  /* sanity gate: total accumulated horizontal nod motion must exceed this */

    /* Capture buffer: r (rotvec from q_neutral) per valid sample, plus a copy
     * of r projected onto the horizontal plane (so we don't recompute). */
    typedef struct { float r[3]; float r_h[3]; float r_h_mag; } cal_sample_t;
    cal_sample_t *samples = calloc(ticks, sizeof(cal_sample_t));
    if (samples == NULL) {
        ESP_LOGE(TAG, "axis calibration aborted: out of memory for %u samples",
                 (unsigned)ticks);
        return ESP_ERR_NO_MEM;
    }

    s_gd.calibrating = true;

    /* Provisional nod_axis = up × [1,0,0], normalized. Used only to gate which
     * samples count as "the user was nodding" in the average; the average
     * itself becomes the real nod_axis. */
    float prov_nod[3] = { up[1]*0.0f - up[2]*0.0f,
                          -up[2],
                           up[1] };
    if (v3_normalize(prov_nod) == 0.0f) {
        prov_nod[0] = 1.0f; prov_nod[1] = 0.0f; prov_nod[2] = 0.0f;
    }

    uint32_t valid_q   = 0;
    uint32_t rejected  = 0;
    uint32_t too_large = 0;

    for (uint32_t i = 0; i < ticks; i++) {
        float q[4];
        if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
            float w_abs = (q[0] < 0.0f) ? -q[0] : q[0];
            if (w_abs < W_MIN) {
                rejected++;
            } else {
                float qrel[4]; quat_mul(qn_conj, q, qrel); quat_normalize(qrel);
                float r[3];    quat_to_rotvec_deg(qrel, r);
                float mag = v3_norm(r);
                if (mag > R_MAX_PER_FRAME) {
                    too_large++;
                } else {
                    /* Always retain the sample — even "still" frames help
                     * define the average. */
                    memcpy(samples[valid_q].r, r, sizeof(r));
                    float d_up = v3_dot(r, up);
                    samples[valid_q].r_h[0] = r[0] - d_up*up[0];
                    samples[valid_q].r_h[1] = r[1] - d_up*up[1];
                    samples[valid_q].r_h[2] = r[2] - d_up*up[2];
                    samples[valid_q].r_h_mag = v3_norm(samples[valid_q].r_h);
                    valid_q++;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    s_gd.calibrating = false;

    if (rejected > 0 || too_large > 0) {
        ESP_LOGW(TAG, "axis calibration: rejected %u near-singular DMP samples, "
                      "%u out-of-range rotations",
                 (unsigned)rejected, (unsigned)too_large);
    }

    if (valid_q < ticks / 4) {
        ESP_LOGE(TAG, "axis calibration failed: too few valid samples (%u/%u) — "
                      "keep head very still between nods and the device firmly mounted",
                 (unsigned)valid_q, (unsigned)ticks);
        free(samples);
        return ESP_FAIL;
    }

    /* ----- nod_axis: mean direction of in-motion horizontal r_h's. -----
     * We average signed-by-direction r_h's so that an up-phase (negative
     * along prov_nod) and a down-phase (positive) collapse into a single
     * direction that points "where the user actually moved". For a typical
     * calibration this is close to the down-phase mean because users pause
     * at the bottom of a nod. The sign convention is therefore "positive
     * projection = chin-down = NOD", matching the sign_pitch=1 default. */
    float  sum_nod[3]    = { 0.0f, 0.0f, 0.0f };
    float  peak_r_h_mag = 0.0f;   /*!< max |r_h| over in-motion samples — what we use for trigger tuning */
    uint32_t nod_used    = 0;
    for (uint32_t k = 0; k < valid_q; k++) {
        float r_h_mag = samples[k].r_h_mag;
        if (r_h_mag < R_PROJ_MIN) continue;            /* still — don't dilute the average */
        if (v3_norm(samples[k].r) < R_MOTION_MIN) continue;
        /* Track peak (direction-agnostic) for trigger threshold tuning. The
         * signed-by-direction sum_nod_mag that used to live here is wrong:
         * it grows linearly with both intensity and duration (a 4-second
         * vigorous calibration can hit 1000°+), so the trigger ends up
         * unreachable. Peak is the unambiguous "how big is one nod". */
        if (r_h_mag > peak_r_h_mag) peak_r_h_mag = r_h_mag;
        float p = v3_dot(samples[k].r_h, prov_nod);
        if (p == 0.0f) continue;
        float sign = (p > 0.0f) ? 1.0f : -1.0f;
        sum_nod[0] += sign * samples[k].r_h[0];
        sum_nod[1] += sign * samples[k].r_h[1];
        sum_nod[2] += sign * samples[k].r_h[2];
        nod_used++;
    }
    if (nod_used < 3) {
        ESP_LOGE(TAG, "axis calibration aborted: only %u nod-direction samples — "
                      "do more pronounced nods", (unsigned)nod_used);
        free(samples);
        return ESP_FAIL;
    }

    float nod[3] = { sum_nod[0] / nod_used, sum_nod[1] / nod_used, sum_nod[2] / nod_used };
    /* (Re-)project onto horizontal — averaging reintroduces a tiny up component. */
    {
        float d = v3_dot(nod, up);
        nod[0] -= d*up[0]; nod[1] -= d*up[1]; nod[2] -= d*up[2];
    }
    if (v3_normalize(nod) == 0.0f) {
        ESP_LOGE(TAG, "axis calibration aborted: motion had no horizontal (nod) component");
        free(samples);
        return ESP_FAIL;
    }
    if (peak_r_h_mag < SUM_MAG_MIN_DEG) {
        ESP_LOGE(TAG, "axis calibration aborted: peak nod motion only %.1f° — nod harder",
                 peak_r_h_mag);
        free(samples);
        return ESP_FAIL;
    }

    /* ----- tilt_axis: same approach on the up × nod plane. ----- */
    float prov_tilt[3] = { up[1]*nod[2] - up[2]*nod[1],
                           up[2]*nod[0] - up[0]*nod[2],
                           up[0]*nod[1] - up[1]*nod[0] };
    if (v3_normalize(prov_tilt) == 0.0f) {
        ESP_LOGE(TAG, "axis calibration aborted: nod parallel to up");
        free(samples);
        return ESP_FAIL;
    }
    float  sum_tilt[3]  = { 0.0f, 0.0f, 0.0f };
    float  sum_tilt_mag = 0.0f;
    uint32_t tilt_used  = 0;
    for (uint32_t k = 0; k < valid_q; k++) {
        float r_h_mag = samples[k].r_h_mag;
        if (r_h_mag < R_PROJ_MIN) continue;
        if (v3_norm(samples[k].r) < R_MOTION_MIN) continue;
        /* Drop the up and nod components so what's left is pure tilt. */
        float d_up   = v3_dot(samples[k].r_h, up);
        float d_nod  = v3_dot(samples[k].r_h, nod);
        float r_t[3] = { samples[k].r_h[0] - d_up*up[0] - d_nod*nod[0],
                         samples[k].r_h[1] - d_up*up[1] - d_nod*nod[1],
                         samples[k].r_h[2] - d_up*up[2] - d_nod*nod[2] };
        float r_t_mag = v3_norm(r_t);
        if (r_t_mag < R_PROJ_MIN) continue;
        float p = v3_dot(r_t, prov_tilt);
        if (p == 0.0f) continue;
        float sign = (p > 0.0f) ? 1.0f : -1.0f;
        sum_tilt[0] += sign * r_t[0];
        sum_tilt[1] += sign * r_t[1];
        sum_tilt[2] += sign * r_t[2];
        sum_tilt_mag += sign * r_t_mag;
        tilt_used++;
    }
    float tilt[3];
    if (tilt_used >= 3 && sum_tilt_mag >= SUM_MAG_MIN_DEG * 0.5f) {
        tilt[0] = sum_tilt[0] / tilt_used;
        tilt[1] = sum_tilt[1] / tilt_used;
        tilt[2] = sum_tilt[2] / tilt_used;
    } else {
        /* Not enough tilt motion during a nod-only calibration. Fall back to
         * the geometric up × nod. The user can run `ct` later to refine. */
        ESP_LOGW(TAG, "tilt motion during nod calibration only %.1f° "
                      "(from %u samples) — falling back to up×nod",
                 sum_tilt_mag, (unsigned)tilt_used);
        tilt[0] = up[1]*nod[2] - up[2]*nod[1];
        tilt[1] = up[2]*nod[0] - up[0]*nod[2];
        tilt[2] = up[0]*nod[1] - up[1]*nod[0];
    }
    if (v3_normalize(tilt) == 0.0f) {
        ESP_LOGE(TAG, "axis calibration aborted: degenerate tilt axis");
        free(samples);
        return ESP_FAIL;
    }
    free(samples);

    memcpy(np.nod_axis,  nod,  sizeof(nod));
    memcpy(np.tilt_axis, tilt, sizeof(tilt));
    gesture_params_set_neutral_aligned(&np);

    /* Thresholds. peak_r_h_mag is the largest horizontal-rotation magnitude
     * observed across all in-motion samples during calibration — i.e. how
     * big a single nod actually was. Setting the trigger at ~35% of the
     * typical peak means the user clears it roughly a quarter of the way
     * into the gesture (responsive) but well above any still-frame noise
     * floor (~1-2°). Velocity uses a fixed floor because we don't track
     * per-frame velocity during calibration and inferring it from peak /
     * time-to-peak is too noisy to be worth the complexity. */
    float new_trig_deg = peak_r_h_mag * 0.35f;
    if (new_trig_deg < 5.0f) new_trig_deg = 5.0f;
    s_gd.params.trigger_deg            = new_trig_deg;
    s_gd.params.trigger_velocity_deg_s = 40.0f;   /* fixed; nod peak vel ≈ 60-100°/s */
    s_gd.params.neutral_zone_deg       = new_trig_deg * 0.30f;
    gesture_params_save_to_nvs(&s_gd.params);

    s_gd.calibrated = true;

    /* Diagnostic snapshot for `cd`. */
    s_last_cap.valid        = valid_q;
    s_last_cap.used         = nod_used;
    s_last_cap.sum_mag_deg  = peak_r_h_mag;   /*!< semantic update: now means peak in-motion magnitude */
    memcpy(s_last_cap.nod_axis,  nod,  sizeof(nod));
    memcpy(s_last_cap.tilt_axis, tilt, sizeof(tilt));
    s_last_cap.drift_deg    = 0.0f;

    ESP_LOGI(TAG, "axes captured (r-avg): nod=[%.2f %.2f %.2f] tilt=[%.2f %.2f %.2f] "
                  "nod_used=%u/%u peak_r_h=%.1f° → trigger=%.1f° vel=%.1f°/s zone=%.1f°",
             nod[0], nod[1], nod[2], tilt[0], tilt[1], tilt[2],
             (unsigned)nod_used, (unsigned)valid_q, peak_r_h_mag,
             new_trig_deg, s_gd.params.trigger_velocity_deg_s,
             s_gd.params.neutral_zone_deg);
    return ESP_OK;
}

/**
 * @brief Measure the user's actual left/right tilt axis and persist it.
 *
 *        The fall-back after gesture_detect_calibrate_axes() is to set
 *        `tilt_axis = up × nod_axis`, which is purely geometric and only
 *        matches reality when the user's natural tilt direction is
 *        exactly perpendicular to their nod in the horizontal plane. For
 *        most wearers the tilt direction has a forward- or backward-
 *        leaning component, so left/right tilts project partly onto
 *        `nod_axis` and partly onto the perpendicular — which is what
 *        makes the detector mis-fire as NOD/LOOK_UP.
 *
 *        We instead measure it directly:
 *          1. for each DMP sample compute q_rel = conj(q_neutral) ⊗ q
 *             and its rotation vector r (degrees);
 *          2. keep the sample with the largest component of r that lies
 *             in the plane perpendicular to `nod_axis` within the
 *             horizontal plane (we explicitly drop the along-nod part so
 *             a nod-leaning tilt can't leak through);
 *          3. set `tilt_axis = normalize(that vector)`;
 *          4. sign-correct so it points in the same direction as the
 *             geometric `up × nod_axis` (positive projection =
 *             right tilt when sign_roll = 1; if the sign comes out
 *             backwards the user just runs the `sr` UART command).
 *
 *        The detector must already have a `q_neutral` and a `nod_axis`;
 *        running this before calibrate_neutral / calibrate_axes gives
 *        undefined results. The function temporarily clears
 *        `s_gd.calibrated` so the user's calibration gestures don't fire
 *        as real events, and restores it on failure / keeps it true on
 *        success.
 */
esp_err_t gesture_detect_calibrate_tilt(uint32_t duration_ms)
{
    if (duration_ms < 500) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "calibrating tilt for %u ms — do a few slow LEFT and RIGHT tilts...",
             (unsigned)duration_ms);

    const uint32_t period_ms = 20;
    const uint32_t ticks     = duration_ms / period_ms;
    if (ticks < 5) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read the calibrated neutral + nod_axis. We deliberately IGNORE the
     * existing tilt_axis — we're overwriting it. */
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    const float *nod = np.nod_axis;

    /* "up" in the neutral body frame. Same definition as detector_task
     * uses (world Z rotated back through the neutral quaternion). */
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    float qn_conj[4]; quat_conj(np.q_neutral, qn_conj);
    float up[3];      quat_rotate_vec(qn_conj, world_up, up);
    if (v3_normalize(up) == 0.0f) {
        ESP_LOGE(TAG, "tilt calibration aborted: bad neutral quaternion");
        return ESP_FAIL;
    }

    /* Reference direction we want tilt_axis to align with when sign_roll
     * is the natural default (positive projection → right tilt). */
    float ref_right[3];
    v3_cross(up, nod, ref_right);
    if (v3_normalize(ref_right) == 0.0f) {
        ESP_LOGE(TAG, "tilt calibration aborted: nod_axis is parallel to up");
        return ESP_FAIL;
    }

    /* Suppress detector events for the duration of the capture so the
     * user's calibration tilts don't get reported as real tilts. Restore
     * on failure; leave enabled on success. */
    const bool prior_calibrated = s_gd.calibrated;
    s_gd.calibrated = false;

    /* v3: average the in-motion tilt-direction r_perp's instead of taking
     * the single largest. Same approach as calibrate_axes' tilt pass. */
    const float W_MIN           = 0.05f;
    const float R_MAX_PER_FRAME = 90.0f;
    const float R_MOTION_MIN    = 4.0f;
    const float R_PROJ_MIN      = 2.0f;
    const float SUM_MAG_MIN_DEG = 10.0f;

    float  sum_tilt[3]  = { 0.0f, 0.0f, 0.0f };
    float  sum_tilt_mag = 0.0f;
    uint32_t valid_q    = 0;
    uint32_t tilt_used  = 0;
    uint32_t rejected   = 0;
    uint32_t too_large  = 0;

    s_gd.calibrating = true;
    for (uint32_t i = 0; i < ticks; i++) {
        float q[4];
        if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
            float w_abs = (q[0] < 0.0f) ? -q[0] : q[0];
            if (w_abs < W_MIN) {
                rejected++;
            } else {
                float qrel[4]; quat_mul(qn_conj, q, qrel); quat_normalize(qrel);
                float r[3];    quat_to_rotvec_deg(qrel, r);
                float mag = v3_norm(r);
                if (mag > R_MAX_PER_FRAME) {
                    too_large++;
                } else if (mag >= R_MOTION_MIN) {
                    /* Horizontal component (drop vertical), then drop nod. */
                    float d_up  = v3_dot(r, up);
                    float r_h[3] = { r[0] - d_up*up[0],
                                     r[1] - d_up*up[1],
                                     r[2] - d_up*up[2] };
                    float d_nod = v3_dot(r_h, nod);
                    float r_perp[3] = { r_h[0] - d_nod*nod[0],
                                        r_h[1] - d_nod*nod[1],
                                        r_h[2] - d_nod*nod[2] };
                    float r_perp_mag = v3_norm(r_perp);
                    if (r_perp_mag >= R_PROJ_MIN) {
                        float p = v3_dot(r_perp, ref_right);
                        if (p != 0.0f) {
                            float sign = (p > 0.0f) ? 1.0f : -1.0f;
                            sum_tilt[0] += sign * r_perp[0];
                            sum_tilt[1] += sign * r_perp[1];
                            sum_tilt[2] += sign * r_perp[2];
                            sum_tilt_mag += sign * r_perp_mag;
                            tilt_used++;
                        }
                    }
                }
                valid_q++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    s_gd.calibrating = false;

    if (rejected > 0 || too_large > 0) {
        ESP_LOGW(TAG, "tilt calibration: rejected %u near-singular DMP samples, "
                      "%u out-of-range rotations",
                 (unsigned)rejected, (unsigned)too_large);
    }

    if (valid_q < ticks / 4) {
        ESP_LOGE(TAG, "tilt calibration failed: too few valid samples (%u/%u) — "
                      "keep head very still between tilts and the device firmly mounted",
                 (unsigned)valid_q, (unsigned)ticks);
        s_gd.calibrated = prior_calibrated;
        return ESP_FAIL;
    }
    if (tilt_used < 3) {
        ESP_LOGE(TAG, "tilt calibration aborted: only %u tilt-direction samples — "
                      "tilt more during the capture window", (unsigned)tilt_used);
        s_gd.calibrated = prior_calibrated;
        return ESP_FAIL;
    }
    if (sum_tilt_mag < SUM_MAG_MIN_DEG) {
        ESP_LOGE(TAG, "tilt calibration aborted: total tilt motion only %.1f — "
                      "tilt harder or more sideways (less forward/back)",
                 sum_tilt_mag);
        s_gd.calibrated = prior_calibrated;
        return ESP_FAIL;
    }

    float tilt[3] = { sum_tilt[0] / tilt_used,
                      sum_tilt[1] / tilt_used,
                      sum_tilt[2] / tilt_used };
    /* Sign-align with ref_right. The signed-by-direction average already
     * collapses left/right into one direction; this is just safety. */
    if (v3_dot(tilt, ref_right) < 0.0f) {
        tilt[0] = -tilt[0]; tilt[1] = -tilt[1]; tilt[2] = -tilt[2];
    }
    if (v3_normalize(tilt) == 0.0f) {
        ESP_LOGE(TAG, "tilt calibration aborted: degenerate tilt axis");
        s_gd.calibrated = prior_calibrated;
        return ESP_FAIL;
    }

    memcpy(np.tilt_axis, tilt, sizeof(tilt));
    gesture_params_set_neutral_aligned(&np);

    s_gd.calibrated = true;

    ESP_LOGI(TAG, "tilt captured (r-avg): tilt=[%.2f %.2f %.2f] used=%u/%u sum_mag=%.1f",
             tilt[0], tilt[1], tilt[2], (unsigned)tilt_used, (unsigned)valid_q,
             sum_tilt_mag);
    return ESP_OK;
}

void gesture_detect_get_last_capture(gesture_detect_capture_t *out)
{
    if (out == NULL) {
        return;
    }
    out->valid       = s_last_cap.valid;
    out->used        = s_last_cap.used;
    out->sum_mag_deg = s_last_cap.sum_mag_deg;
    out->drift_deg   = s_last_cap.drift_deg;
    memcpy(out->nod_axis,  s_last_cap.nod_axis,  sizeof(out->nod_axis));
    memcpy(out->tilt_axis, s_last_cap.tilt_axis, sizeof(out->tilt_axis));
}
