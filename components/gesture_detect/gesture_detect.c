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
#include "mouse_mode.h"

static const char *TAG = "gesture_detect";

/* ===== Task timing ======================================================= */
#define GD_TASK_PERIOD_MS    20      /*!< 50 Hz polling */
#define GD_TASK_STACK_WORDS  4096
#define GD_TASK_PRIORITY     5

/* 1 = log every fire / suppress / cooldown / dominance-skip / snap decision.
 * Very useful for tuning thresholds; set to 0 for production builds.
 * When data-capture mode is active (s_dc_active), DBG logs are suppressed
 * so only DC lines appear — makes it easy to copy clean data. */
#define GD_DEBUG_FIRING      0

#if GD_DEBUG_FIRING
#define GD_DBGI(fmt, ...) do { if (!s_dc_active) ESP_LOGI(TAG, fmt, ##__VA_ARGS__); } while(0)
#define GD_DBGW(fmt, ...) do { if (!s_dc_active) ESP_LOGW(TAG, fmt, ##__VA_ARGS__); } while(0)
#else
#define GD_DBGI(...) do {} while(0)
#define GD_DBGW(...) do {} while(0)
#endif

/* ===== Accumulation-based detection thresholds ============================ */
#define TRIG_VEL_FRAC     0.35f  /*!< trigger velocity = peak × this fraction */
#define END_VEL_FRAC      0.20f  /*!< end velocity = peak × this fraction */
#define END_HOLD_FRAMES   2      /*!< frames below end_vel before firing (2×20ms = 40ms) */
#define MIN_FIRE_R_MAG    3.0f   /*!< minimum smooth_r magnitude to fire (°) — prevents
                                      firing at neutral position after return motion */
#define FIRE_COOLDOWN_MS  1000   /*!< minimum ms between two fired events */
#define MIN_ACC_MAG       3.0f   /*!< minimum |accumulated| to fire (filters tiny noisy motions) */
#define MIN_PITCH_SUM     0.05f  /*!< minimum |pitch_sum| to decide NOD vs LOOK_UP direction */
#define AXIS_DOMINANCE    0.3f   /*!< winning axis dot must exceed runner-up by this margin */

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

    /* ===== Smoothed velocity for dominance gate ===========================
     * Raw frame-to-frame velocity is noisy — a single DMP glitch can
     * produce a large spike on the tilt axis during a nod, causing the
     * dominance gate to pick the wrong axis. Exponential moving average
     * (α=0.3) smooths out spikes while staying responsive (~100ms
     * time constant at 50 Hz). */
    float           smooth_vel_nod;      /*!< EMA-smoothed |proj_nod| velocity */
    float           smooth_vel_tilt;     /*!< EMA-smoothed |proj_tilt| velocity */

    /* Direction-based dominance signatures */
    gesture_signatures_t sig;

    /* Cross-product instantaneous rotation axis */
    float           prev_fwd[3];
    bool            prev_fwd_valid;
    float           smooth_axis[3];
    bool            smooth_axis_valid;
    /* Accumulation-based trigger */
    float           prev_proj_axis;
    float           accum[3];
    bool            accum_armed;
    /* Smoothed rotation vector for classification (EMA of r).
     * Replaces smooth_axis (EMA of cross-product) for the dot-product
     * classification.  The rotation vector r aligns with the calibrated
     * PCA signatures; the cross-product cp is perpendicular to the
     * rotation axis and caused misclassification (nod → tilt). */
    float           smooth_r[3];
    bool            smooth_r_valid;
    /* Per-gesture confidence snapshot (carried to emit_event) */
    float           last_conf[4];   /*!< [0]=NOD [1]=LOOK_UP [2]=TILTL [3]=TILTR */
} gd_t;

#if 0
typedef struct {
    uint32_t valid;
    uint32_t used;
    float    nod_axis[3];
    float    tilt_axis[3];
    float    sum_mag_deg;
    float    drift_deg;
} last_capture_t;

static last_capture_t s_last_cap;
#endif

/* ===== Data-capture mode (Phase: diagnostics) ============================
 * When active, detector_task logs every frame's raw metrics at 50 Hz so
 * the user can collect gesture data for offline analysis.  Enabled by
 * gesture_detect_start_capture(); auto-expires after the requested
 * duration.  The BLE `dc` command triggers it from main.c.
 */
static volatile bool     s_dc_active    = false;
static volatile uint32_t s_dc_until_ms  = 0;

static gd_t s_gd;

static float s_cal_rest_q[4] = {1, 0, 0, 0};
static bool  s_cal_rest_valid = false;
static bool  s_cal_just_completed = false;  /*!< true after first tick post-calibration (FIFO flushed) */

/* Accumulation-path state — declared at file scope so the post-calibration
 * flush code can reset them.  Originally function-local statics inside
 * detector_task. */
static uint32_t s_end_hold = 0;
static float    s_peak_sign_dot = 0.0f;
static uint32_t s_consec_above_trigger = 0;
static int      s_consistent_idx = -1;
static int      s_consistent_count = 0;
static float    s_pitch_dot_sum = 0.0f;
static float    s_tilt_dot_sum  = 0.0f;   /*!< accumulated r·sig_tiltL for tilt direction */
static bool     s_had_high_vel  = false;  /*!< set when vel > trigger_vel; gates fire */

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

/* Maximum allowed angle (degrees) between q_drift and q_neutral.
 * After many snap-on-return events during rapid gestures, q_drift
 * can accumulate >100° of rotation from q_neutral.  This causes
 * compute_effective_axes to rotate the calibrated nod/tilt axes into
 * orientations where a tilt gesture projects equally onto both axes,
 * making the dominance gate misclassify tilts as nods.  Clamping the
 * drift keeps the effective axes close to the well-calibrated raw
 * axes.  15° is enough to absorb typical DMP drift (5-10°) while
 * keeping the effective axes well-aligned with the calibrated raw
 * axes — at 25° the tilt projection from a nod was already ~7° and
 * growing, risking misclassification. */
#define Q_DRIFT_MAX_DEG  15.0f

/* After any snap that sets q_drift = qcur, call this to ensure
 * q_drift doesn't rotate more than Q_DRIFT_MAX_DEG from q_neutral.
 * Uses slerp to pull q_drift back toward q_neutral if needed. */
static void constrain_q_drift(gd_t *gd, const neutral_pose_aligned_t *np)
{
    float d = gd->q_drift[0]*np->q_neutral[0] + gd->q_drift[1]*np->q_neutral[1] +
              gd->q_drift[2]*np->q_neutral[2] + gd->q_drift[3]*np->q_neutral[3];
    /* Ensure same hemisphere for slerp */
    float sign = 1.0f;
    if (d < 0.0f) { sign = -1.0f; d = -d; }
    if (d > 1.0f) d = 1.0f;

    float angle_deg = 2.0f * acosf(d) * (180.0f / (float)M_PI);
    if (angle_deg > Q_DRIFT_MAX_DEG) {
        float t = Q_DRIFT_MAX_DEG / angle_deg;  /* fraction toward q_drift */
        float angle = acosf(d);
        float sin_angle = sinf(angle);
        float a_coeff, b_coeff;
        if (sin_angle < 1e-6f) {
            a_coeff = 1.0f - t; b_coeff = t;
        } else {
            a_coeff = sinf((1.0f - t) * angle) / sin_angle;
            b_coeff = sinf(t * angle) / sin_angle;
        }
        /* slerp from q_neutral toward (sign * q_drift) by factor t */
        for (int i = 0; i < 4; i++)
            gd->q_drift[i] = a_coeff * np->q_neutral[i] + b_coeff * sign * gd->q_drift[i];
        quat_normalize(gd->q_drift);
        GD_DBGI("DBG-CONSTRAIN drift %.1f° → %.1f° from q_neutral",
                 angle_deg, Q_DRIFT_MAX_DEG);
    }
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

/* Phase 6: rotate the q_neutral-frame nod_axis / tilt_axis into the
 * current q_drift frame so they can be dotted with r (which the detector
 * already computes as rotvec(conj(q_drift) ⊗ qcur), i.e. in q_drift frame).
 * The rotation is q_diff = conj(q_drift) ⊗ q_neutral — identity when
 * there's no佩戴微调, full rotation when q_drift has snapped far away.
 * Recomputed each tick (cost is two quat-vec rotations, ~100 flops at
 * 50 Hz — negligible) so q_drift updates are reflected immediately without
 * state-synchronisation bugs. Also used by the `p` command for diagnostics. */
static void compute_effective_axes(const neutral_pose_aligned_t *np,
                                   const float q_drift[4],
                                   float out_nod[3],
                                   float out_tilt[3])
{
    float qd_conj[4]; quat_conj(q_drift, qd_conj);
    float q_diff[4];  quat_mul(qd_conj, np->q_neutral, q_diff);
    quat_normalize(q_diff);
    quat_rotate_vec(q_diff, np->nod_axis,  out_nod);
    quat_rotate_vec(q_diff, np->tilt_axis, out_tilt);
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
    esp_err_t ret = gesture_detect_apply_params(&loaded);

    /* Load previously calibrated gesture signatures from NVS.
     * These are the cross-product rotation axes captured during
     * cn/ctl/ctr calibration.  They persist across reboots. */
    esp_err_t sig_err = gesture_signatures_load_from_nvs(&s_gd.sig);
    if (sig_err == ESP_OK && s_gd.sig.calibrated != 0) {
        ESP_LOGI(TAG, "signatures loaded from NVS: calibrated=0x%02x "
                 "nod=[%.3f %.3f %.3f] tiltL=[%.3f %.3f %.3f] tiltR=[%.3f %.3f %.3f]",
                 (unsigned)s_gd.sig.calibrated,
                 s_gd.sig.sig_nod[0], s_gd.sig.sig_nod[1], s_gd.sig.sig_nod[2],
                 s_gd.sig.sig_tiltL[0], s_gd.sig.sig_tiltL[1], s_gd.sig.sig_tiltL[2],
                 s_gd.sig.sig_tiltR[0], s_gd.sig.sig_tiltR[1], s_gd.sig.sig_tiltR[2]);
    }

    return ret;
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
    s_gd.smooth_vel_nod  = 0.0f;
    s_gd.smooth_vel_tilt = 0.0f;
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

#if 0
void gesture_detect_set_sign(bool positive_pitch_is_nod, bool positive_roll_is_right)
{
    s_gd.params.sign_pitch = positive_pitch_is_nod ? 1 : 0;
    s_gd.params.sign_roll  = positive_roll_is_right ? 1 : 0;
}
#endif

void gesture_detect_get_q_drift(float out[4])
{
    if (out == NULL) {
        return;
    }
    memcpy(out, s_gd.q_drift, sizeof(s_gd.q_drift));
}

void gesture_detect_get_effective_axes(float out_nod[3], float out_tilt[3])
{
    if (out_nod == NULL || out_tilt == NULL) {
        return;
    }
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    compute_effective_axes(&np, s_gd.q_drift, out_nod, out_tilt);
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
    s_gd.smooth_vel_nod  = 0.0f;
    s_gd.smooth_vel_tilt = 0.0f;
}

void gesture_detect_reset_calibration(void)
{
    ESP_LOGI(TAG, "RESET_CAL: calibrated was %d, sig.calibrated was 0x%02x — clearing all",
             (int)s_gd.calibrated, (unsigned)s_gd.sig.calibrated);
    s_gd.calibrated = false;
    s_gd.sig.calibrated = 0;
    s_cal_rest_valid = false;
    s_cal_just_completed = false;
}

/* ===== Event helper ====================================================== */

static void emit_event(gesture_type_t type, float peak_angle, float peak_vel,
                       const float conf[4], int8_t best_idx)
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
        .best_idx          = best_idx,
    };
    if (conf) {
        memcpy(ev.conf, conf, sizeof(ev.conf));
    } else {
        memset(ev.conf, 0, sizeof(ev.conf));
    }
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
            if (absf(signed_rel) > trigger && abs_velocity > trigger_v) {
                GD_DBGI("DBG-COOL proj=%+.1f vel=%.1f remaining=%ums",
                         signed_rel, abs_velocity,
                         (unsigned)(s_gd.cooldown_until_ms - now_ms));
            }
            break;
        }
        if (absf(signed_rel) > trigger && abs_velocity > trigger_v) {
            ctx->state         = (signed_rel > 0) ? GD_AXIS_LOCKED_POS
                                                  : GD_AXIS_LOCKED_NEG;
            ctx->t_enter_ms    = now_ms;
            ctx->peak_abs_rel  = absf(signed_rel);
            ctx->peak_abs_vel  = abs_velocity;
            if (allow_fire) {
                gesture_type_t gt = (signed_rel > 0) ? pos_gesture : neg_gesture;
                GD_DBGI("DBG-FIRE type=%d proj=%+.1f vel=%.1f "
                         "peak_a=%.1f peak_v=%.1f "
                         "trig=%.1f trig_v=%.1f zone=%.1f deb=%u",
                         (int)gt, signed_rel, abs_velocity,
                         ctx->peak_abs_rel, ctx->peak_abs_vel,
                         trigger, trigger_v, zone, (unsigned)debounce);
                emit_event(gt, ctx->peak_abs_rel, ctx->peak_abs_vel, NULL, -1);
                return true;
            }
            GD_DBGI("DBG-SUPP proj=%+.1f vel=%.1f "
                     "(dominant axis took priority)",
                     signed_rel, abs_velocity);
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
    float prev_r_mag = 0.0f;
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

        /* Require a full calibration before gesture detection is active.
         * Reset on each BLE connection via gesture_detect_reset_calibration(). */
        if (!s_gd.calibrated) {
            s_cal_just_completed = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        /* First tick after calibration — flush the DMP FIFO that
         * accumulated stale samples during calibration (DMP runs at
         * 100 Hz but the calibrator only drains at ~50 Hz).
         * Step 1: reset FIFO hardware.  Step 2: wait 100 ms for the
         * DMP to produce fresh quaternion data.  Step 3: read and
         * discard up to 5 samples to flush any residual stale data
         * and prime prev_fwd / prev_r_mag with current values.
         * Step 4: reset all detector state so stale accum / smooth /
         * hold counters don't leak into the first real gesture. */
        if (!s_cal_just_completed) {
            s_cal_just_completed = true;
            mpu_reset_fifo();
            vTaskDelay(pdMS_TO_TICKS(100));
            /* Discard up to 5 samples, keeping the last one as q_drift seed. */
            float qdiscard[4];
            int discarded = 0;
            for (int d = 0; d < 5; d++) {
                if (mpu_dmp_get_quat(&qdiscard[0], &qdiscard[1],
                                     &qdiscard[2], &qdiscard[3]) == 0) {
                    discarded++;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            /* Seed q_drift from the freshest sample. */
            if (discarded > 0) {
                memcpy(s_gd.q_drift, qdiscard, sizeof(s_gd.q_drift));
                s_gd.q_drift_valid = true;
                s_gd.still_since_ms = 0;
            }
            /* Reset all detector state so stale data can't trigger. */
            prev_r_mag = 0.0f;
            prev_valid = false;
            memset(s_gd.prev_fwd, 0, sizeof(s_gd.prev_fwd));
            s_gd.prev_fwd_valid = false;
            memset(s_gd.smooth_axis, 0, sizeof(s_gd.smooth_axis));
            s_gd.smooth_axis_valid = false;
            memset(s_gd.smooth_r, 0, sizeof(s_gd.smooth_r));
            s_gd.smooth_r_valid = false;
            memset(s_gd.accum, 0, sizeof(s_gd.accum));
            s_gd.accum_armed = true;
            s_gd.smooth_vel_nod = 0.0f;
            s_gd.smooth_vel_tilt = 0.0f;
            /* Reset file-scope accumulation state. */
            s_end_hold = 0;
            s_peak_sign_dot = 0.0f;
            s_consec_above_trigger = 0;
            s_consistent_idx = -1;
            s_consistent_count = 0;
            s_pitch_dot_sum = 0.0f;
            s_tilt_dot_sum  = 0.0f;
            s_had_high_vel = false;
            GD_DBGI("DBG-CAL-FLUSH FIFO reset + %d samples discarded, "
                     "detector state cleared", discarded);
            continue;
        }

        float qcur[4];
        if (mpu_dmp_get_quat(&qcur[0], &qcur[1], &qcur[2], &qcur[3]) != 0) {
            /* FIFO miss — skip this tick */
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* Pull a stack-local aligned copy of the neutral pose so the
         * quaternion/vector helpers can take its members as float* without
         * tripping -Werror=address-of-packed-member. The pose is stable
         * for the duration of one tick (calibration only writes when
         * detector_task is between samples). The nod/tilt axes are still
         * expressed in the q_neutral frame — that's fine as long as
         * q_drift doesn't stray far from q_neutral.  constrain_q_drift()
         * (called after every snap) enforces Q_DRIFT_MAX_DEG to prevent
         * the effective axes from rotating into bad orientations. */
        neutral_pose_aligned_t np;
        gesture_params_get_neutral_aligned(&np);

        /* Phase 5: first sample after apply_params() seeds q_drift from
         * the current pose and skips processing for this tick — we have
         * no prior projection state for the velocity estimate and r itself
         * would be qcur-vs-qcur (zero) anyway. Subsequent ticks compute r
         * against q_drift, which slowly tracks佩戴微调. */
        if (!s_gd.q_drift_valid) {
            memcpy(s_gd.q_drift, qcur, sizeof(s_gd.q_drift));
            constrain_q_drift(&s_gd, &np);
            s_gd.q_drift_valid  = true;
            s_gd.still_since_ms = 0;
            prev_valid = false;
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }

        /* Relative rotation from the runtime baseline q_drift, expressed
         * as a rotation vector (axis * angle, degrees). Replaces the old
         * q_neutral-relative r — same projection math, but the baseline
         * now follows佩戴微调 instead of being frozen at calibration. */
        float qd_conj[4]; quat_conj(s_gd.q_drift, qd_conj);
        float qrel[4];    quat_mul(qd_conj, qcur, qrel);
        quat_normalize(qrel);

        /* Yaw decomposition: remove the rotation around the world vertical
         * axis before converting to a rotation vector. Without this, a
         * combined yaw+nod produces a nonlinear rotation vector whose
         * projection leaks into the tilt axis (e.g. 90° yaw + 10° nod
         * gives proj_nod ≈ proj_tilt ≈ 8°). By extracting the yaw
         * quaternion qyaw and computing qnotyaw = conj(qyaw) ⊗ qrel,
         * only the pitch/roll residual enters the rotation vector. */
        float world_up[3] = {0.0f, 0.0f, 1.0f};
        float up_body[3]; quat_rotate_vec(qd_conj, world_up, up_body);
        float r[3];
        if (v3_normalize(up_body) > 1e-6f) {
            float v_dot_up = qrel[1]*up_body[0] + qrel[2]*up_body[1] + qrel[3]*up_body[2];
            float yaw_half = atan2f(v_dot_up, qrel[0]);
            float qyaw[4] = { cosf(yaw_half),
                              sinf(yaw_half)*up_body[0],
                              sinf(yaw_half)*up_body[1],
                              sinf(yaw_half)*up_body[2] };
            float qyaw_conj[4]; quat_conj(qyaw, qyaw_conj);
            float qnotyaw[4]; quat_mul(qyaw_conj, qrel, qnotyaw);
            quat_normalize(qnotyaw);
            quat_to_rotvec_deg(qnotyaw, r);
        } else {
            quat_to_rotvec_deg(qrel, r);
        }

        /* Runtime glitch filter: reject DMP samples where the relative
         * rotation exceeds what a human head can physically produce.
         * At 50 Hz, 45°/frame = 2250°/s, already 10× the human limit
         * (~200°/s). Anything above this is either a DMP FIFO glitch
         * (single-frame spike) or a q_drift that has drifted far from
         * the current pose (persistent large |r|). We distinguish the
         * two with a streak counter: a real glitch is 1-2 frames, a
         * drift is 5+ consecutive frames. On drift, snap q_drift to
         * qcur to recover instead of rejecting forever. */
        const float R_MAX_RUNTIME = 45.0f;
        const uint32_t GLITCH_RESYNC_FRAMES = 5;
        static uint32_t s_glitch_streak = 0;
        float r_mag = v3_norm(r);
        if (r_mag > R_MAX_RUNTIME) {
            s_glitch_streak++;
            if (s_glitch_streak >= GLITCH_RESYNC_FRAMES) {
                /* Persistent offset: q_drift has drifted from the actual
                 * device orientation. Snap q_drift to qcur (same as
                 * `q reset` but automatic) so detection can resume.
                 * Note: do NOT constrain_q_drift here — if qcur is far
                 * from q_neutral, constraining would keep |r| > R_MAX
                 * and cause an infinite resync loop.  The still-snap
                 * paths will gradually pull q_drift back toward
                 * q_neutral once the user is at rest. */
                float d = qcur[0]*s_gd.q_drift[0] + qcur[1]*s_gd.q_drift[1] +
                          qcur[2]*s_gd.q_drift[2] + qcur[3]*s_gd.q_drift[3];
                if (d < 0.0f) {
                    s_gd.q_drift[0] = -qcur[0]; s_gd.q_drift[1] = -qcur[1];
                    s_gd.q_drift[2] = -qcur[2]; s_gd.q_drift[3] = -qcur[3];
                } else {
                    memcpy(s_gd.q_drift, qcur, sizeof(s_gd.q_drift));
                }
                s_glitch_streak = 0;
                prev_valid = false;
                GD_DBGW("DBG-RESYNC q_drift→qcur after %u stale frames (|r| was %.1f°)",
                         (unsigned)GLITCH_RESYNC_FRAMES, r_mag);
            } else {
                prev_valid = false;
                GD_DBGW("DBG-GLITCH |r|=%.1f° (max %.0f°) streak=%u",
                         r_mag, R_MAX_RUNTIME, (unsigned)s_glitch_streak);
            }
            vTaskDelayUntil(&last, pdMS_TO_TICKS(GD_TASK_PERIOD_MS));
            continue;
        }
        s_glitch_streak = 0;

        /* ---- Forward-vector cross product → rotation axis -------------- */
        const float FWD_BODY[3] = {1.0f, 0.0f, 0.0f};
        float fwd[3];
        quat_rotate_vec(qcur, FWD_BODY, fwd);

        float cp[3] = {0.0f, 0.0f, 0.0f};
        bool cp_valid = false;
        if (s_gd.prev_fwd_valid) {
            v3_cross(s_gd.prev_fwd, fwd, cp);
            cp_valid = (v3_norm(cp) > 0.0001f);
        }
        s_gd.prev_fwd[0] = fwd[0]; s_gd.prev_fwd[1] = fwd[1]; s_gd.prev_fwd[2] = fwd[2];
        s_gd.prev_fwd_valid = true;

        /* ---- Angular velocity (deg/s) from rotation magnitude change -- */
        float vel = 0.0f;
        if (prev_valid) {
            vel = fabsf(r_mag - prev_r_mag) / ((float)GD_TASK_PERIOD_MS / 1000.0f);
        }
        prev_r_mag = r_mag;
        prev_valid = true;

        /* Accumulation state (file-scope statics, reset in post-cal flush) */

        /* ---- 3-axis classification via dot product with signatures ----
         * Copy packed sig arrays to aligned locals to avoid
         * -Werror=address-of-packed-member. */
        float sig_nod_a[3], sig_tiltL_a[3], sig_tiltR_a[3];
        memcpy(sig_nod_a,   s_gd.sig.sig_nod,   sizeof(sig_nod_a));
        memcpy(sig_tiltL_a, s_gd.sig.sig_tiltL, sizeof(sig_tiltL_a));
        memcpy(sig_tiltR_a, s_gd.sig.sig_tiltR, sizeof(sig_tiltR_a));

        /* Capture raw r direction for NOD vs LOOK_UP direction split.
         * Uses r (rotation vector) to stay in the same space as the
         * PCA signatures. */
        float r_raw_dot_nod = 0.0f;
        if ((s_gd.sig.calibrated & GESTURE_SIG_F_NOD) && r_mag > 0.1f) {
            r_raw_dot_nod = v3_dot(r, sig_nod_a);
        }

        int best_sig_idx = -1;
        float classification_confidence = 0.0f;  /*!< dot product of best matching signature — carried to emit */
        if (s_gd.sig.calibrated != 0 && r_mag > 0.1f) {
            /* Classify using the rotation vector r (not the cross-product
             * cp).  The PCA signatures (sig_nod, sig_tiltL, sig_tiltR)
             * are rotation-vector axes, so the dot product must be in the
             * same space.  Using cp (which is perpendicular to the
             * rotation axis) caused nod to be misclassified as tilt. */
            if (!s_gd.smooth_r_valid) {
                s_gd.smooth_r[0] = r[0]; s_gd.smooth_r[1] = r[1]; s_gd.smooth_r[2] = r[2];
                s_gd.smooth_r_valid = true;
            } else {
                /* Flip r if it points opposite to smooth_r to avoid
                 * sign-flip artifacts across the singularity. */
                float d = v3_dot(r, s_gd.smooth_r);
                float ru[3] = { r[0], r[1], r[2] };
                if (d < 0.0f) { ru[0]=-ru[0]; ru[1]=-ru[1]; ru[2]=-ru[2]; }
                const float R_ALPHA = 0.65f;
                s_gd.smooth_r[0] = s_gd.smooth_r[0]*(1-R_ALPHA) + ru[0]*R_ALPHA;
                s_gd.smooth_r[1] = s_gd.smooth_r[1]*(1-R_ALPHA) + ru[1]*R_ALPHA;
                s_gd.smooth_r[2] = s_gd.smooth_r[2]*(1-R_ALPHA) + ru[2]*R_ALPHA;
                float sm = v3_norm(s_gd.smooth_r);
                if (sm > 0.001f) { s_gd.smooth_r[0]/=sm; s_gd.smooth_r[1]/=sm; s_gd.smooth_r[2]/=sm; }
            }
            /* Only classify against calibrated signatures.
             * NOD/LOOK_UP share one axis (opposite directions).
             * TILT_LEFT/TILT_RIGHT share one axis (opposite directions).
             *
             * Axis-purity weighting: the raw dot product with the PCA
             * signature can be misleading when signatures overlap across
             * axes (e.g. nod_axis has a Y component that aligns with
             * tilt motions).  We weight each confidence by how much of
             * the smooth_r's energy lies in the expected axis:
             *   pitch gestures (NOD/LOOK_UP) → X component only
             *   roll gestures (TILT_L/TILT_R) → Y+Z components only
             * This ensures a purely vertical tilt motion (Y-dominant)
             * cannot score high on the pitch axis, and vice versa. */
            float sr_x = fabsf(s_gd.smooth_r[0]);
            float sr_roll = sqrtf(s_gd.smooth_r[1]*s_gd.smooth_r[1]
                                + s_gd.smooth_r[2]*s_gd.smooth_r[2]);
            float dn  = (s_gd.sig.calibrated & GESTURE_SIG_F_NOD)
                ? fabsf(v3_dot(s_gd.smooth_r, sig_nod_a))   * sr_x    : 0.0f;
            float dtl = (s_gd.sig.calibrated & GESTURE_SIG_F_TILTL)
                ? fabsf(v3_dot(s_gd.smooth_r, sig_tiltL_a)) * sr_roll : 0.0f;
            /* Index 0=NOD 1=LOOK_UP 2=TILTL 3=TILTR
             * NOD and LOOK_UP share axis → same alignment confidence.
             * TILT_LEFT and TILT_RIGHT share axis → same alignment.
             * Direction split happens at emit time. */
            float ad[] = {dn, dn, dtl, dtl};
            float best = 0.0f; int bi = -1;
            for (int i = 0; i < 4; i++) { if (ad[i] > best) { best = ad[i]; bi = i; } }
            if (best > 0.15f) best_sig_idx = bi;
            classification_confidence = best;
            /* Save all4 for the event — carried through to emit_event.
             * At emit time, the pitch-axis pair (NOD/LOOK_UP) or the
             * roll-axis pair (TILT_L/TILT_R) is split based on direction. */
            memcpy(s_gd.last_conf, ad, sizeof(s_gd.last_conf));
        }

        /* ---- Scale cross product by calibration-derived factor ---------
         * Forward-vector cross product values are tiny (~0.01-0.1)
         * because the forward direction barely changes per frame.
         * Calibration records avg_cp per gesture; scale brings
         * average gesture to ~1.0 for usable accumulation range. */
        float cp_sc[3] = {cp[0], cp[1], cp[2]};
        {
            float avg_cp = 0.0f;
            if (best_sig_idx >= 0) {
                switch (best_sig_idx) {
                    case 0: case 1: avg_cp = s_gd.sig.avg_cp_nod;   break;
                    case 2:         avg_cp = s_gd.sig.avg_cp_tiltL; break;
                    case 3:         avg_cp = s_gd.sig.avg_cp_tiltR; break;
                }
            } else {
                /* Before classification, use overall average */
                avg_cp = (s_gd.sig.avg_cp_nod + s_gd.sig.avg_cp_tiltL + s_gd.sig.avg_cp_tiltR) / 3.0f;
            }
            if (avg_cp > 0.001f) {
                float scale = 1.0f / avg_cp;
                cp_sc[0] = cp[0] * scale;
                cp_sc[1] = cp[1] * scale;
                cp_sc[2] = cp[2] * scale;
            }
        }

        /* Data-capture mode: log frame metrics at 50 Hz. */
        if (s_dc_active) {
            if (now_ms >= s_dc_until_ms) {
                s_dc_active = false;
                ESP_LOGI(TAG, "data capture OFF");
            } else {
                /* Line 1: raw data + rotation axis + velocity */
                ESP_LOGI(TAG, "DC1 t=%u r=[%+.1f %+.1f %+.1f] rm=%.1f "
                         "cp=[%.4f %.4f %.4f] sc=[%.2f %.2f %.2f] vel=%.0f",
                         (unsigned)now_ms,
                         r[0], r[1], r[2], r_mag,
                         cp_valid ? cp[0] : 0.0f,
                         cp_valid ? cp[1] : 0.0f,
                         cp_valid ? cp[2] : 0.0f,
                         cp_sc[0], cp_sc[1], cp_sc[2],
                         vel);
                /* Line 2: accumulation + classification */
                float acc_mag = v3_norm(s_gd.accum);
                float acc_n = acc_mag > 0.01f ? v3_dot(s_gd.accum, sig_nod_a) / acc_mag : 0.0f;
                float acc_tL = acc_mag > 0.01f ? v3_dot(s_gd.accum, sig_tiltL_a) / acc_mag : 0.0f;
                float acc_tR = acc_mag > 0.01f ? v3_dot(s_gd.accum, sig_tiltR_a) / acc_mag : 0.0f;
                ESP_LOGI(TAG, "DC2 idx=%d acc=[%.1f %.1f %.1f] |acc|=%.1f "
                         "dot(n=%.2f tL=%.2f tR=%.2f) hold=%u p=%.1f "
                         "conf=[NOD=%.2f LK=%.2f TL=%.2f TR=%.2f]",
                         best_sig_idx,
                         s_gd.accum[0], s_gd.accum[1], s_gd.accum[2],
                         acc_mag, acc_n, acc_tL, acc_tR,
                         (unsigned)s_end_hold, s_pitch_dot_sum,
                         s_gd.last_conf[0], s_gd.last_conf[1],
                         s_gd.last_conf[2], s_gd.last_conf[3]);
            }
        }

        /* ---- Per-gesture velocity thresholds (from calibration) ------- */
        float trigger_vel = 50.0f, end_vel = 30.0f, peak_vel_ref = 100.0f;
        if (best_sig_idx == 0 && s_gd.sig.peak_vel_nod > 10.0f)
            peak_vel_ref = s_gd.sig.peak_vel_nod;
        else if (best_sig_idx == 1 && s_gd.sig.peak_vel_nod > 10.0f)
            peak_vel_ref = s_gd.sig.peak_vel_nod;
        else if (best_sig_idx == 2 && s_gd.sig.peak_vel_tiltL > 10.0f)
            peak_vel_ref = s_gd.sig.peak_vel_tiltL;
        else if (best_sig_idx == 3 && s_gd.sig.peak_vel_tiltR > 10.0f)
            peak_vel_ref = s_gd.sig.peak_vel_tiltR;
        trigger_vel = peak_vel_ref * TRIG_VEL_FRAC;
        end_vel     = peak_vel_ref * END_VEL_FRAC;

        /* ---- Accumulation-based trigger --------------------------------
         * Accumulate raw cross-product vectors while velocity > trigger.
         * Record the sign of the accumulation at peak velocity.
         * Fire when velocity drops below end_vel for END_HOLD_FRAMES
         * and the accumulated sign matches the peak sign.
         * The return stroke produces opposite-sign cross products,
         * so the accumulator is naturally cancelled. */

        if (s_gd.accum_armed && best_sig_idx >= 0 && cp_valid) {
            const float *msig = NULL;
            switch (best_sig_idx) {
                case 0: case 1: msig = sig_nod_a;   break;
                case 2:         msig = sig_tiltL_a;  break;
                case 3:         msig = sig_tiltR_a;  break;
            }
            /* Temporal consistency tracking (kept for diagnostics) */
            if (best_sig_idx == s_consistent_idx) {
                s_consistent_count++;
            } else {
                s_consistent_idx = best_sig_idx;
                s_consistent_count = 1;
            }
            if (vel > trigger_vel) {
                s_consec_above_trigger++;
                /* Only arm fire gate after TRIG_RESET_FRAMES consecutive
                 * high-velocity frames.  A single-frame settling spike
                 * (return-motion or micro-movement) will NOT re-arm,
                 * preventing re-fire from the same static tilted position
                 * after cooldown expires. */
                {
                    const int TRIG_ARM_FRAMES = 3;
                    if (s_consec_above_trigger >= TRIG_ARM_FRAMES) {
                        s_had_high_vel = true;
                    }
                }
                /* Cross-axis consistency: only accumulate if the raw cp
                 * direction is consistent with the detected gesture's
                 * signature.  Removed cons_ok gate — confidence-based
                 * classification (smooth_r) already handles temporal
                 * consistency; the old cons_ok gate caused accum to
                 * stall whenever the classification briefly flipped. */
                bool axis_ok = true;
                if (msig) {
                    float cp_n = v3_norm(cp);
                    if (cp_n > 0.001f) {
                        float cp_dot_msig = fabsf(cp[0]*msig[0] + cp[1]*msig[1] + cp[2]*msig[2]) / cp_n;
                        if (cp_dot_msig < 0.3f) axis_ok = false;
                    }
                }
                if (axis_ok) {
                    /* Accumulate scaled cross product. */
                    s_gd.accum[0] += cp_sc[0]; s_gd.accum[1] += cp_sc[1]; s_gd.accum[2] += cp_sc[2];
                    /* Only reset hold after N consecutive frames above trigger.
                     * A single DMP glitch (vel→0→high) won't reset hold. */
                    const int TRIG_RESET_FRAMES = 3;
                    if (s_consec_above_trigger >= TRIG_RESET_FRAMES) {
                        s_end_hold = 0;
                    }
                }
                /* Accumulate pitch direction OUTSIDE axis_ok — the cross-product
                 * direction is perpendicular to the rotation vector, so axis_ok
                 * can block accumulation during genuine nod/look-up motions. */
                s_pitch_dot_sum += apply_sign_pitch(r_raw_dot_nod,
                                                    s_gd.params.sign_pitch);
                /* Accumulate tilt direction similarly — r·sig_tiltL accumulates
                 * net roll motion.  apply_sign_roll normalises the sign so
                 * that a positive sum always means the user's chosen
                 * gesture direction. */
                if (s_gd.sig.calibrated & GESTURE_SIG_F_TILTL) {
                    s_tilt_dot_sum += apply_sign_roll(v3_dot(r, sig_tiltL_a),
                                                      s_gd.params.sign_roll);
                }
            } else {
                s_consec_above_trigger = 0;
                if (vel < end_vel && best_sig_idx >= 0 &&
                    s_gd.smooth_r_valid && s_had_high_vel) {
                    /* Velocity dropped below end_vel — start hold timer.
                     * Fire condition: confidence-based classification (smooth_r)
                     * is stable, velocity dropped, and there was recent fast
                     * motion.  Removed acc-based MIN_ACC_MAG / sign_match /
                     * axis_dominance — those used cross-product space which
                     * is perpendicular to the rotation-vector signatures. */
                    s_end_hold++;
                    if (s_end_hold >= END_HOLD_FRAMES) {
                        /* Guard: don't fire if the raw rotation vector r_mag
                         * (in degrees) is too small — the head has returned
                         * to neutral and we'd be firing on noise.
                         * NOTE: smooth_r is normalized to unit length, so
                         * we must use r_mag (raw), NOT v3_norm(smooth_r). */
                        if (r_mag < MIN_FIRE_R_MAG) {
                            /* Head at neutral — don't fire, but don't reset
                             * all state either; the gesture motion was real. */
                            s_end_hold = 0;
                        } else if (now_ms >= s_gd.cooldown_until_ms) {
                            static const gesture_type_t sig_gt[] = {
                                GESTURE_NOD, GESTURE_LOOK_UP,
                                GESTURE_TILT_LEFT, GESTURE_TILT_RIGHT
                            };
                            /* Determine gesture type. */
                            gesture_type_t gt = sig_gt[best_sig_idx];
                            if (best_sig_idx == 0) {
                                /* Pitch axis: use pitch_dot_sum for direction.
                                 * apply_sign_pitch() has already normalised the
                                 * sign so that positive = NOD (chin-down) for
                                 * any sensor orientation. */
                                if (fabsf(s_pitch_dot_sum) >= MIN_PITCH_SUM) {
                                    gt = (s_pitch_dot_sum > 0.0f)
                                        ? GESTURE_NOD : GESTURE_LOOK_UP;
                                }
                                /* else: pitch_sum too weak, keep default (NOD) */
                            }
                            if (best_sig_idx == 2) {
                                /* Roll axis: use accumulated tilt_dot_sum for
                                 * direction.  apply_sign_roll() normalises the
                                 * sign so that positive = LEFT for any sensor
                                 * orientation.  At fire time smooth_r may have
                                 * drifted back to neutral, making its projection
                                 * unreliable — the accumulated sum is more robust. */
                                if (fabsf(s_tilt_dot_sum) >= MIN_PITCH_SUM) {
                                    gt = (s_tilt_dot_sum < 0.0f)
                                        ? GESTURE_TILT_RIGHT : GESTURE_TILT_LEFT;
                                }
                                /* else: tilt_sum too weak, keep default (TILT_LEFT) */
                            }

                            /* Split confidence by direction:
                             * Zero out the opposite direction so only the
                             * matched gesture shows confidence. */
                            float split_conf[4];
                            memcpy(split_conf, s_gd.last_conf, sizeof(split_conf));
                            if (best_sig_idx == 0) {
                                if (gt == GESTURE_NOD) {
                                    split_conf[1] = 0.0f;  /* kill LOOK_UP */
                                } else {
                                    split_conf[0] = 0.0f;  /* kill NOD */
                                }
                            } else if (best_sig_idx == 2) {
                                if (gt == GESTURE_TILT_LEFT) {
                                    split_conf[3] = 0.0f;  /* kill TILT_RIGHT */
                                } else {
                                    split_conf[2] = 0.0f;  /* kill TILT_LEFT */
                                }
                            }

                            emit_event(gt, r_mag, vel,
                                       split_conf, best_sig_idx);
                            ESP_LOGI(TAG, "DETECT %s idx=%d r=%.1f vel=%.0f "
                                     "pitch_sum=%.2f tilt_sum=%.2f "
                                     "NOD=%.2f LK=%.2f TL=%.2f TR=%.2f",
                                     (gt == GESTURE_NOD) ? "NOD" :
                                     (gt == GESTURE_LOOK_UP) ? "LOOK_UP" :
                                     (gt == GESTURE_TILT_LEFT) ? "TILT_LEFT" :
                                     (gt == GESTURE_TILT_RIGHT) ? "TILT_RIGHT" : "?",
                                     best_sig_idx, r_mag, vel,
                                     s_pitch_dot_sum, s_tilt_dot_sum,
                                     s_gd.last_conf[0], s_gd.last_conf[1],
                                     s_gd.last_conf[2], s_gd.last_conf[3]);

                            /* ---- Post-fire state reset ----
                             * Reset smooth_r_valid: force smooth_r to
                             * re-initialize from the current r vector.  This is
                             * now safe because s_had_high_vel requires
                             * TRIG_ARM_FRAMES (3) consecutive high-velocity
                             * frames to re-arm — a single settling spike during
                             * return motion cannot re-arm, so the re-init'd
                             * smooth_r can't cause an immediate second fire.
                             * Keeping smooth_r valid would let it track the
                             * tilted position indefinitely, causing re-fire
                             * after cooldown from the same static posture. */
                            memset(s_gd.accum, 0, sizeof(s_gd.accum));
                            s_gd.accum_armed = true;
                            s_end_hold = 0; s_peak_sign_dot = 0.0f;
                            s_pitch_dot_sum = 0.0f;
                            s_tilt_dot_sum  = 0.0f;
                            s_consistent_count = 0; s_consistent_idx = -1;
                            s_consec_above_trigger = 0;
                            s_had_high_vel = false;
                            s_gd.smooth_r_valid = false;
                        } else {
                            /* Cooldown active — just reset hold counter. */
                            s_end_hold = 0;
                        }
                    }
                }
            }
        /* else: velocity between end_vel and trigger — do nothing */
        } else if (best_sig_idx < 0) {
            /* No classification — reset accumulator. */
            memset(s_gd.accum, 0, sizeof(s_gd.accum));
            s_gd.accum_armed = true;
            s_end_hold = 0; s_peak_sign_dot = 0.0f;
            s_pitch_dot_sum = 0.0f;
            s_tilt_dot_sum  = 0.0f;
            s_consistent_count = 0; s_consistent_idx = -1;
            s_consec_above_trigger = 0;
            s_had_high_vel = false;
        }

        /* ---- Phase 5: sliding baseline snap --------------------------- */
        const uint32_t STILL_DURATION_MS = 200;
        const float zone_for_still = s_gd.params.neutral_zone_deg;
        bool proj_still = (r_mag < zone_for_still);
        bool vel_still  = (vel < 15.0f);
        static uint32_t s_vel_still_since = 0;

        if (proj_still) {
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
                constrain_q_drift(&s_gd, &np);
                s_gd.still_since_ms = now_ms;
                s_vel_still_since = 0;
            }
        } else {
            s_gd.still_since_ms = 0;
        }

        if (vel_still && !proj_still) {
            if (s_vel_still_since == 0) {
                s_vel_still_since = now_ms;
            } else if ((now_ms - s_vel_still_since) >= 120) {
                float d = qcur[0]*s_gd.q_drift[0] + qcur[1]*s_gd.q_drift[1] +
                          qcur[2]*s_gd.q_drift[2] + qcur[3]*s_gd.q_drift[3];
                if (d < 0.0f) {
                    s_gd.q_drift[0] = -qcur[0]; s_gd.q_drift[1] = -qcur[1];
                    s_gd.q_drift[2] = -qcur[2]; s_gd.q_drift[3] = -qcur[3];
                } else {
                    memcpy(s_gd.q_drift, qcur, sizeof(s_gd.q_drift));
                }
                constrain_q_drift(&s_gd, &np);
                s_vel_still_since = now_ms;
            }
        } else if (!vel_still) {
            s_vel_still_since = 0;
        }

        /* ── Mouse mode: send cursor reports or skip gesture emission ── */
        if (mouse_mode_is_active()) {
            /* Feed tilt events into the toggle state machine even while
             * mouse_mode is active — this detects the deactivation
             * left+right tilt sequence.
             * NOTE: best_sig_idx==2 means the roll axis matched (both
             * TILT_LEFT and TILT_RIGHT share this axis). Direction is
             * determined by the sign of the projection onto the tilt
             * signature, NOT by best_sig_idx. */
            if (best_sig_idx == 2 && s_gd.smooth_r_valid) {
                float sig_tiltL_a[3];
                memcpy(sig_tiltL_a, s_gd.sig.sig_tiltL, sizeof(sig_tiltL_a));
                float roll_proj = v3_dot(s_gd.smooth_r, sig_tiltL_a);
                gesture_type_t toggle_gest = (roll_proj >= 0.0f)
                    ? GESTURE_TILT_LEFT : GESTURE_TILT_RIGHT;
                mouse_mode_toggle_step((int)toggle_gest, now_ms);
            }

            /* Send cursor movement HID reports.
             * mouse_mode_tick computes its own effective axes relative
             * to q_mouse_rest (not q_drift), so the passed-in nod_eff/
             * tilt_eff are unused. */
            mouse_mode_tick(qcur, s_gd.q_drift, r_mag, vel);

            /* Skip gesture event emission and accumulation — mouse mode
             * suppresses all gesture-triggered cmd_configs. */
            goto tick_end;
        }

        /* Non-mouse-mode: feed tilt events into toggle detection for
         * activation. This runs AFTER emit_event so normal gesture
         * processing is unaffected — toggle is purely additive.
         * Same axis+direction logic as the mouse_mode branch above. */
        if (best_sig_idx == 2 && s_gd.smooth_r_valid) {
            float sig_tiltL_a[3];
            memcpy(sig_tiltL_a, s_gd.sig.sig_tiltL, sizeof(sig_tiltL_a));
            float roll_proj = v3_dot(s_gd.smooth_r, sig_tiltL_a);
            gesture_type_t toggle_gest = (roll_proj >= 0.0f)
                ? GESTURE_TILT_LEFT : GESTURE_TILT_RIGHT;
            mouse_mode_toggle_step((int)toggle_gest, now_ms);
        }

tick_end:
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

void gesture_detect_start_capture(uint32_t duration_ms)
{
    if (duration_ms == 0) duration_ms = 30000;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_dc_active   = true;
    s_dc_until_ms = now + duration_ms;
    ESP_LOGI(TAG, "data capture ON for %u ms — perform gestures now",
             (unsigned)duration_ms);
}

const gesture_sig_axes_t *gesture_detect_get_sig_axes(void)
{
    if (!(s_gd.sig.calibrated & GESTURE_SIG_F_NOD) &&
        !(s_gd.sig.calibrated & GESTURE_SIG_F_TILTL) &&
        !(s_gd.sig.calibrated & GESTURE_SIG_F_TILTR)) {
        return NULL;  /* not calibrated */
    }
    static gesture_sig_axes_t s_axes;
    memcpy(s_axes.sig_nod,   s_gd.sig.sig_nod,   sizeof(float)*3);
    memcpy(s_axes.sig_tiltL, s_gd.sig.sig_tiltL, sizeof(float)*3);
    memcpy(s_axes.sig_tiltR, s_gd.sig.sig_tiltR, sizeof(float)*3);
    return &s_axes;
}

#if 0
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
#endif

#if 0
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
    const float R_MAX_PER_FRAME = 90.0f;   /* generous: yaw can inflate |r|; horizontal component is filtered separately */
    const float R_PEAK_MAX      = 60.0f;   /* don't let DMP glitches inflate peak_r_h_mag for trigger tuning */
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
        if (r_h_mag > peak_r_h_mag && r_h_mag < R_PEAK_MAX) peak_r_h_mag = r_h_mag;
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
#endif

#if 0
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
    const float R_MAX_PER_FRAME = 90.0f;   /* same as calibrate_axes: yaw can inflate |r| */
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
    if (fabsf(sum_tilt_mag) < SUM_MAG_MIN_DEG * 0.3f) {
        ESP_LOGE(TAG, "tilt calibration aborted: net tilt motion only %.1f° — "
                      "tilt harder or more consistently in one direction",
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
#endif

#if 0
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
#endif

/* ===== 5-step calibration: generic gesture signature capture =============== */

esp_err_t gesture_detect_calibrate_rest(uint32_t duration_ms)
{
    if (duration_ms < 500) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "calibrating REST for %u ms...", (unsigned)duration_ms);

    const uint32_t period_ms = 20;
    const uint32_t ticks = duration_ms / period_ms;
    const float W_MIN = 0.05f;

    float q_sum[4] = {0};
    uint32_t count = 0;
    s_gd.calibrating = true;

    for (uint32_t i = 0; i < ticks; i++) {
        float q[4];
        if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
            float w_abs = (q[0] < 0.0f) ? -q[0] : q[0];
            if (w_abs >= W_MIN) {
                if (count > 0) {
                    float d = q[0]*q_sum[0] + q[1]*q_sum[1] +
                              q[2]*q_sum[2] + q[3]*q_sum[3];
                    if (d < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
                }
                q_sum[0] += q[0]; q_sum[1] += q[1];
                q_sum[2] += q[2]; q_sum[3] += q[3];
                count++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    s_gd.calibrating = false;

    if (count < 3) {
        ESP_LOGE(TAG, "rest cal: too few samples (%u)", (unsigned)count);
        s_cal_rest_valid = false;
        return ESP_FAIL;
    }
    float rn = sqrtf(q_sum[0]*q_sum[0] + q_sum[1]*q_sum[1] +
                     q_sum[2]*q_sum[2] + q_sum[3]*q_sum[3]);
    s_cal_rest_q[0] = q_sum[0]/rn; s_cal_rest_q[1] = q_sum[1]/rn;
    s_cal_rest_q[2] = q_sum[2]/rn; s_cal_rest_q[3] = q_sum[3]/rn;
    s_cal_rest_valid = true;

    ESP_LOGI(TAG, "rest captured (%u samples) q=[%.3f %.3f %.3f %.3f]",
             (unsigned)count, s_cal_rest_q[0], s_cal_rest_q[1],
             s_cal_rest_q[2], s_cal_rest_q[3]);

    /* ── Quick-calibrate path ───────────────────────────────────────────
     * Update q_neutral to the freshly captured rest pose.  If gesture
     * signatures were already loaded from NVS (from a previous full
     * calibration), we can enable detection immediately — the user only
     * needed to re-do the rest pose because the headset was repositioned. */
    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    memcpy(np.q_neutral, s_cal_rest_q, sizeof(np.q_neutral));
    gesture_params_set_neutral_aligned(&np);
    /* set_neutral_aligned → apply_params already syncs q_drift
     * and resets q_drift_valid so the detector re-syncs on next tick. */

    if (s_gd.sig.calibrated & GESTURE_SIG_F_MINIMUM) {
        ESP_LOGI(TAG, "signatures already calibrated (0x%02x) — "
                 "rest pose updated, but full re-calibration (cn/ctl/ctr) required",
                 (unsigned)s_gd.sig.calibrated);
    } else {
        ESP_LOGW(TAG, "no gesture signatures yet — run cn/ctl/ctr to calibrate axes");
    }

    /* Flush FIFO so the detector doesn't process stale samples from
     * this calibration when it resumes. */
    mpu_reset_fifo();
    return ESP_OK;
}

esp_err_t gesture_detect_calibrate_gesture(gesture_type_t type, uint32_t duration_ms)
{
    if (duration_ms < 500) return ESP_ERR_INVALID_ARG;
    if (type < GESTURE_NOD || type > GESTURE_TILT_RIGHT) return ESP_ERR_INVALID_ARG;

    static const char *names[] = { "?", "NOD", "LOOK_UP", "TILT_LEFT", "TILT_RIGHT" };
    ESP_LOGI(TAG, "calibrating gesture %s for %u ms...", names[type], (unsigned)duration_ms);

    const uint32_t period_ms = 20;
    const uint32_t ticks     = duration_ms / period_ms;
    const uint32_t rest_ticks = ticks / 3;
    const uint32_t gesture_ticks = ticks - rest_ticks;
    const float W_MIN = 0.05f;

    float q_rest[4] = {0};
    uint32_t rest_count = 0;

    if (s_cal_rest_valid) {
        q_rest[0] = s_cal_rest_q[0]; q_rest[1] = s_cal_rest_q[1];
        q_rest[2] = s_cal_rest_q[2]; q_rest[3] = s_cal_rest_q[3];
        rest_count = 999;
        ESP_LOGI(TAG, "gesture cal %s: using pre-captured rest", names[type]);
    } else {
        s_gd.calibrating = true;
        for (uint32_t i = 0; i < rest_ticks; i++) {
            float q[4];
            if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
                float w_abs = (q[0] < 0.0f) ? -q[0] : q[0];
                if (w_abs >= W_MIN) {
                    if (rest_count > 0) {
                        float d = q[0]*q_rest[0] + q[1]*q_rest[1] +
                                  q[2]*q_rest[2] + q[3]*q_rest[3];
                        if (d < 0.0f) { q[0]=-q[0]; q[1]=-q[1]; q[2]=-q[2]; q[3]=-q[3]; }
                    }
                    q_rest[0] += q[0]; q_rest[1] += q[1];
                    q_rest[2] += q[2]; q_rest[3] += q[3];
                    rest_count++;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(period_ms));
        }
        s_gd.calibrating = false;
        if (rest_count < 3) {
            ESP_LOGE(TAG, "gesture cal %s: too few rest samples (%u)", names[type], (unsigned)rest_count);
            return ESP_FAIL;
        }
        float rn = sqrtf(q_rest[0]*q_rest[0] + q_rest[1]*q_rest[1] +
                         q_rest[2]*q_rest[2] + q_rest[3]*q_rest[3]);
        q_rest[0] /= rn; q_rest[1] /= rn; q_rest[2] /= rn; q_rest[3] /= rn;
    }

    float q_rest_conj[4]; quat_conj(q_rest, q_rest_conj);
    float best_mag = 0.0f;
    float best_vel = 0.0f;
    uint32_t gesture_count = 0;
    uint32_t gesture_ticks_count = (rest_count >= 999) ? ticks : gesture_ticks;

    float all_r[120][3];
    float all_mag[120];
    float all_vel[120];
    uint32_t n_frames = 0;
    float prev_r_mag = 0.0f;
    TickType_t prev_frame_tick = 0;
    /* Track forward-vector cross product magnitudes during calibration */
    const float FWD_B[3] = {1.0f, 0.0f, 0.0f};
    float prev_fwd_cal[3] = {0.0f};
    bool  prev_fwd_cal_valid = false;
    float cp_mag_sum = 0.0f;
    uint32_t cp_count = 0;

    s_gd.calibrating = true;
    for (uint32_t i = 0; i < gesture_ticks_count && n_frames < 120; i++) {
        float q[4];
        if (mpu_dmp_get_quat(&q[0], &q[1], &q[2], &q[3]) == 0) {
            float w_abs = (q[0] < 0.0f) ? -q[0] : q[0];
            if (w_abs >= W_MIN) {
                float qrel[4]; quat_mul(q_rest_conj, q, qrel); quat_normalize(qrel);
                float r[3]; quat_to_rotvec_deg(qrel, r);
                float mag = v3_norm(r);
                /* Skip frames beyond 90 degrees — quaternion wrap artifacts
                 * cause sudden jumps (e.g. 13° → 146°) that corrupt the
                 * velocity and axis statistics. */
                if (mag > 90.0f) {
                    gesture_count++;
                    vTaskDelay(pdMS_TO_TICKS(period_ms));
                    continue;
                }
                /* Compute velocity BEFORE recording this frame.
                 * Use actual elapsed ticks to handle skipped frames
                 * (>90° quaternion wrap) where time passes but
                 * prev_r_mag isn't updated. */
                float vel_now = 0.0f;
                TickType_t now_tick = xTaskGetTickCount();
                if (n_frames > 0 && prev_frame_tick > 0) {
                    float dt_s = (float)(now_tick - prev_frame_tick) * portTICK_PERIOD_MS / 1000.0f;
                    if (dt_s > 0.001f) {
                        vel_now = fabsf(mag - prev_r_mag) / dt_s;
                    }
                    if (vel_now > best_vel) best_vel = vel_now;
                }
                all_r[n_frames][0] = r[0];
                all_r[n_frames][1] = r[1];
                all_r[n_frames][2] = r[2];
                all_mag[n_frames] = mag;
                all_vel[n_frames] = vel_now;
                n_frames++;
                if (mag > best_mag) best_mag = mag;
                prev_r_mag = mag;
                prev_frame_tick = now_tick;
                gesture_count++;
                /* Compute forward-vector cross product for avg_cp */
                {
                    float fwd_cal[3];
                    quat_rotate_vec(q, FWD_B, fwd_cal);
                    if (prev_fwd_cal_valid) {
                        float cpv[3];
                        v3_cross(prev_fwd_cal, fwd_cal, cpv);
                        cp_mag_sum += v3_norm(cpv);
                        cp_count++;
                    }
                    prev_fwd_cal[0] = fwd_cal[0]; prev_fwd_cal[1] = fwd_cal[1]; prev_fwd_cal[2] = fwd_cal[2];
                    prev_fwd_cal_valid = true;
                }
                /* Debug: output every 20th frame during calibration (less BLE noise) */
                if (n_frames % 20 == 0) {
                    ESP_LOGI(TAG, "CAL-%s f=%u r=[%+.1f %+.1f %+.1f] mag=%.1f vel=%.0f "
                             "best_mag=%.1f best_vel=%.0f",
                             names[type], (unsigned)n_frames, r[0], r[1], r[2], mag, vel_now,
                             best_mag, best_vel);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }
    s_gd.calibrating = false;

    const float FRAC_MIN = 0.30f;
    const float VEL_MIN = 30.0f;  /* exclude static/near-static frames from PCA */

    /* ---- Compute gesture axis via PCA (principal component) -----------
     * The gesture axis is the direction of maximum variance in r-space.
     * PCA removes the mean (eliminating drift offset) and finds the
     * direction where the r vectors spread the most (the gesture axis).
     * Power iteration: 8 iterations on 3x3 covariance matrix. */
    /* 1. Compute centroid of qualifying frames */
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    uint32_t pca_n = 0;
    for (uint32_t i = 0; i < n_frames; i++) {
        if (all_mag[i] > best_mag * FRAC_MIN && all_mag[i] < 90.0f
            && all_vel[i] > VEL_MIN) {
            cx += all_r[i][0]; cy += all_r[i][1]; cz += all_r[i][2];
            pca_n++;
        }
    }
    float axis[3] = {0.0f, 0.0f, 0.0f};
    if (pca_n >= 2) {
        float inv_n = 1.0f / (float)pca_n;
        cx *= inv_n; cy *= inv_n; cz *= inv_n;
        /* 2. Build 3x3 covariance matrix from centered vectors */
        float c00=0,c01=0,c02=0, c11=0,c12=0, c22=0;
        for (uint32_t i = 0; i < n_frames; i++) {
            if (all_mag[i] <= best_mag * FRAC_MIN || all_mag[i] >= 90.0f
                || all_vel[i] <= VEL_MIN) continue;
            float dx = all_r[i][0] - cx;
            float dy = all_r[i][1] - cy;
            float dz = all_r[i][2] - cz;
            c00 += dx*dx; c01 += dx*dy; c02 += dx*dz;
            c11 += dy*dy; c12 += dy*dz;
            c22 += dz*dz;
        }
        /* 3. Power iteration: find dominant eigenvector */
        axis[0] = 1.0f; axis[1] = 0.0f; axis[2] = 0.0f;
        for (int iter = 0; iter < 8; iter++) {
            float x = c00*axis[0] + c01*axis[1] + c02*axis[2];
            float y = c01*axis[0] + c11*axis[1] + c12*axis[2];
            float z = c02*axis[0] + c12*axis[1] + c22*axis[2];
            float nm = sqrtf(x*x + y*y + z*z);
            if (nm < 1e-10f) break;
            axis[0] = x/nm; axis[1] = y/nm; axis[2] = z/nm;
        }
    }

    if (best_mag < 5.0f) {
        ESP_LOGE(TAG, "gesture cal %s: peak rotation too small (%.1f deg)", names[type], best_mag);
        return ESP_FAIL;
    }

    float axis_sm = sqrtf(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (axis_sm < 0.01f) {
        ESP_LOGE(TAG, "gesture cal %s: axis sum too small (cp_count=%u), no valid frames",
                 names[type], (unsigned)cp_count);
        return ESP_FAIL;
    }
    axis[0] /= axis_sm; axis[1] /= axis_sm; axis[2] /= axis_sm;

    ESP_LOGI(TAG, "gesture %s: axis PCA=[%+.3f %+.3f %+.3f] pca_n=%u",
             names[type], axis[0], axis[1], axis[2], (unsigned)pca_n);

    uint32_t avg_count = 0;
    for (uint32_t i = 0; i < n_frames; i++) {
        if (all_mag[i] > best_mag * FRAC_MIN && all_mag[i] < 90.0f) avg_count++;
    }

    uint8_t flag = 0;
    float avg_cp = (cp_count > 0) ? (cp_mag_sum / (float)cp_count) : 0.0f;
    ESP_LOGI(TAG, "gesture %s: avg_cp=%.4f (sum=%.3f cnt=%u)",
             names[type], avg_cp, cp_mag_sum, (unsigned)cp_count);
    switch (type) {
        case GESTURE_NOD:
            memcpy(s_gd.sig.sig_nod, axis, sizeof(float)*3);
            s_gd.sig.spread_nod_deg = 0.0f;
            s_gd.sig.peak_vel_nod = best_vel;
            s_gd.sig.avg_cp_nod = avg_cp;
            flag = GESTURE_SIG_F_NOD;
            break;
        case GESTURE_LOOK_UP:
            memcpy(s_gd.sig.sig_lookup, axis, sizeof(float)*3);
            s_gd.sig.spread_lookup_deg = 0.0f;
            flag = GESTURE_SIG_F_LOOKUP;
            break;
        case GESTURE_TILT_LEFT:
            memcpy(s_gd.sig.sig_tiltL, axis, sizeof(float)*3);
            s_gd.sig.spread_tiltL_deg = 0.0f;
            s_gd.sig.peak_vel_tiltL = best_vel;
            s_gd.sig.avg_cp_tiltL = avg_cp;
            flag = GESTURE_SIG_F_TILTL;
            break;
        case GESTURE_TILT_RIGHT:
            memcpy(s_gd.sig.sig_tiltR, axis, sizeof(float)*3);
            s_gd.sig.spread_tiltR_deg = 0.0f;
            s_gd.sig.peak_vel_tiltR = best_vel;
            s_gd.sig.avg_cp_tiltR = avg_cp;
            flag = GESTURE_SIG_F_TILTR;
            break;
        default: break;
    }
    s_gd.sig.calibrated |= flag;
    gesture_signatures_save_to_nvs(&s_gd.sig);

    ESP_LOGI(TAG, "gesture %s: axis=[%.3f %.3f %.3f] peak=%.1f avg_frames=%u/%u calibrated=0x%02x",
             names[type], axis[0], axis[1], axis[2], best_mag,
             (unsigned)avg_count, (unsigned)gesture_count, s_gd.sig.calibrated);

    if ((s_gd.sig.calibrated & GESTURE_SIG_F_MINIMUM) == GESTURE_SIG_F_MINIMUM) {
        ESP_LOGI(TAG, "3 axes calibrated (0x%02x) - inferring signs...",
                 (unsigned)s_gd.sig.calibrated);
        gesture_detect_infer_signs();
        if (!s_gd.calibrated && s_cal_rest_valid) {
            s_gd.calibrated = true;
            ESP_LOGI(TAG, "full calibration complete (cr + cn + ctl + ctr) — detection enabled");
        } else if (!s_cal_rest_valid) {
            ESP_LOGW(TAG, "3 axes done but no rest pose — run cr first");
        }
    } else {
        ESP_LOGI(TAG, "sig.calibrated=0x%02x, need 0x%02x — keep calibrating",
                 (unsigned)s_gd.sig.calibrated, (unsigned)GESTURE_SIG_F_MINIMUM);
    }

    /* Flush FIFO so stale samples from this calibration don't leak
     * into the detector's first real tick. */
    mpu_reset_fifo();
    return ESP_OK;
}

esp_err_t gesture_detect_infer_signs(void)
{
    if ((s_gd.sig.calibrated & GESTURE_SIG_F_MINIMUM) != GESTURE_SIG_F_MINIMUM) {
        ESP_LOGW(TAG, "cannot infer signs: only 0x%02x calibrated (need 0x%02x)",
                 s_gd.sig.calibrated, GESTURE_SIG_F_MINIMUM);
        return ESP_ERR_INVALID_STATE;
    }

    float sn[3], tl[3];
    memcpy(sn, s_gd.sig.sig_nod,   sizeof(float)*3);
    memcpy(tl, s_gd.sig.sig_tiltL, sizeof(float)*3);

    neutral_pose_aligned_t np;
    gesture_params_get_neutral_aligned(&np);
    float nod_eff[3], tilt_eff[3];
    compute_effective_axes(&np, s_gd.q_drift, nod_eff, tilt_eff);

    float d_nod = v3_dot(sn, nod_eff);
    int inferred_pitch = (d_nod >= 0.0f) ? 1 : 0;

    float d_tl = v3_dot(tl, tilt_eff);
    int inferred_roll = (d_tl < 0.0f) ? 0 : 1;

    gesture_params_t params = s_gd.params;
    params.sign_pitch = (uint8_t)inferred_pitch;
    params.sign_roll  = (uint8_t)inferred_roll;
    gesture_detect_apply_params(&params);
    gesture_params_save_to_nvs(&params);

    ESP_LOGI(TAG, "signs inferred: sign_pitch=%d sign_roll=%d (d_nod=%+.3f d_tl=%+.3f)",
             inferred_pitch, inferred_roll, d_nod, d_tl);
    return ESP_OK;
}
