#ifndef GESTURE_PARAMS_H_
#define GESTURE_PARAMS_H_

#include <stdint.h>
#include <stdbool.h>

/* ===== Compile-time defaults =============================================
 * These match the values agreed in the prior session. They are used
 * (a) at first boot when NVS is empty and
 * (b) after `gesture_params_reset_default()`.
 */
#define GESTURE_DEFAULT_TRIGGER_DEG          20.0f   /*!< |angle-neutral| to fire */
#define GESTURE_DEFAULT_TRIGGER_VEL_DEG_S    50.0f   /*!< min angular speed to fire */
#define GESTURE_DEFAULT_NEUTRAL_ZONE_DEG      6.0f   /*!< |angle-neutral| to re-arm (wider → cleaner return) */
#define GESTURE_DEFAULT_DEBOUNCE_MS          500     /*!< min ms the user must hold inside zone before re-arming */
#define GESTURE_DEFAULT_COOLDOWN_MS          500     /*!< min ms between any two fired events (kills double-fire / phantom second gesture from return-swing) */
#define GESTURE_DEFAULT_SIGN_PITCH             1     /*!< 1: positive pitch = nod (chin-down) */
#define GESTURE_DEFAULT_SIGN_ROLL              1     /*!< 1: positive roll = right-tilt */

/* ===== Magic / version ===================================================
 * Any read of `magic` that doesn't match or a `version` higher than we
 * know how to interpret is treated as "NVS is empty / corrupt" — the
 * detector falls back to defaults and (optionally) re-saves them.
 */
#define GESTURE_PARAMS_MAGIC   0xCA1178E7u
#define GESTURE_PARAMS_VERSION 2u   /*!< v2: quaternion neutral + calibrated gesture axes */

/* ===== NVS layout ========================================================
 * Persisted under namespace "gesture", key "params" as a blob.
 */
#define GESTURE_NVS_NAMESPACE "gesture"
#define GESTURE_NVS_KEY       "params"

/**
 * @brief Sensor orientation captured at "neutral head pose", plus the two
 *        gesture axes derived by the nod calibration.
 *
 *        Detection is now fully 3-D: instead of subtracting scalar Euler
 *        offsets (the old v1 "2-D translation" that coupled pitch into roll
 *        at non-zero mounting angles), the detector computes the relative
 *        rotation q_rel = conj(q_neutral) ⊗ q_current, turns it into a
 *        rotation vector (axis·angle, degrees), and projects that onto
 *        `nod_axis` / `tilt_axis`. This is mounting-angle independent and
 *        free of Euler singularities.
 *
 *        `q_neutral` is stored (w,x,y,z). `nod_axis` is the unit rotation
 *        axis of a real nod (chin-down = +); `tilt_axis` = normalize(up ×
 *        nod_axis) with + = right tilt. Both are unit vectors expressed in
 *        the neutral body frame.
 */
typedef struct {
    float q_neutral[4];   /*!< w,x,y,z  body->world quaternion at neutral pose */
    float nod_axis[3];    /*!< unit axis, + = chin-down NOD */
    float tilt_axis[3];   /*!< unit axis, + = right tilt (= up × nod_axis) */
} neutral_pose_t;         /*!< 40 B */

/**
 * @brief Unaligned mirror of `neutral_pose_t`, for in-stack use by the
 *        detector and calibration code. The packed NVS blob has unaligned
 *        members, so passing `packed.field` to a function that takes a
 *        `float*` triggers `-Werror=address-of-packed-member` on xtensa
 *        GCC. Copy the packed blob into one of these once at the start of
 *        a function, operate on the copy, and memcpy back when writing.
 */
typedef struct {
    float q_neutral[4];
    float nod_axis[3];
    float tilt_axis[3];
} neutral_pose_aligned_t;

/** Read a stack-local aligned copy of the persisted neutral pose. */
void gesture_params_get_neutral_aligned(neutral_pose_aligned_t *out);

/** Write the (calibrated) neutral pose back to in-memory params. */
void gesture_params_set_neutral_aligned(const neutral_pose_aligned_t *src);

/**
 * @brief One motion capture used by the calibration session (Phase 3).
 *        Declared here so the params struct and the session share types.
 */
typedef struct {
    float    peak_angle_deg;
    float    peak_velocity_deg_s;
    uint32_t rise_time_ms;
    uint32_t return_time_ms;
} gesture_rep_meas_t;

/**
 * @brief Persistent detector parameters. Stored as a blob in NVS.
 *
 *        Layout is `__attribute__((packed))` and ends up exactly 64 bytes
 *        (4 + 1 + 40 + 4 + 4 + 4 + 2 + 1 + 1 + 3 = 64). Grew from v1's 32 B
 *        when `neutral_pose_t` went from 2 Euler offsets to a quaternion +
 *        two gesture axes. Old v1 blobs are rejected by the version check
 *        and the detector falls back to defaults (user must re-calibrate).
 *
 *        Threshold values are SYMMETRIC across all four gestures — the
 *        calibration session derives them as the conservative-min across
 *        every per-rep peak collected.
 */
typedef struct __attribute__((packed)) {
    uint32_t     magic;                 /*!< GESTURE_PARAMS_MAGIC */
    uint8_t      version;               /*!< GESTURE_PARAMS_VERSION */
    neutral_pose_t neutral;             /*!< 40 B: q_neutral + nod/tilt axes */
    float        trigger_deg;           /*!< symmetric */
    float        trigger_velocity_deg_s;
    float        neutral_zone_deg;      /*!< symmetric */
    uint16_t     debounce_ms;
    uint8_t      sign_pitch;            /*!< 0: flip nod-axis projection sign (positive = LOOK_UP) */
    uint8_t      sign_roll;             /*!< 0: flip tilt-axis projection sign (positive = LEFT) */
    uint8_t      reserved[3];           /*!< pad to 64 B */
} gesture_params_t;

/* ===== Helpers ===========================================================
 * Compile-time check that gesture_params_t is exactly 64 bytes. If a
 * future field is added without bumping `version`, the build breaks
 * before NVS gets misaligned at runtime.
 */
_Static_assert(sizeof(gesture_params_t) == 64,
               "gesture_params_t must remain 64 bytes for NVS stability");

esp_err_t gesture_params_load_from_nvs(gesture_params_t *out);

esp_err_t gesture_params_save_to_nvs(const gesture_params_t *params);

void gesture_params_load_default(gesture_params_t *out);

#endif /* GESTURE_PARAMS_H_ */