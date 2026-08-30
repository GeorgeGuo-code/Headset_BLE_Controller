#ifndef GESTURE_DETECT_H_
#define GESTURE_DETECT_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gesture_event.h"
#include "gesture_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the detector. Loads params from NVS (or installs
 *        compile-time defaults if the slot is empty / corrupt) but does
 *        NOT start the sampling task. Safe to call once at boot.
 */
esp_err_t gesture_detect_init(void);

/**
 * @brief Replace the in-memory params at runtime. The detector picks up
 *        the new values on its next 50 Hz tick. Used by the calibration
 *        session (Phase 3) and by manual override.
 *
 *        Does NOT persist — pair with `gesture_params_save_to_nvs()` if
 *        the change should survive reboot.
 */
esp_err_t gesture_detect_apply_params(const gesture_params_t *params);

/**
 * @brief Read-only access to the currently-active params. Pointer is to
 *        the detector's internal copy; do not modify in place.
 */
const gesture_params_t *gesture_detect_get_params(void);

/**
 * @brief Start the 50 Hz sampling task. Events are pushed to
 *        `event_queue` (caller-owned). The task stops if the queue is
 *        full (drop-newest policy, see gesture_detect.c).
 */
esp_err_t gesture_detect_start(QueueHandle_t event_queue);

/* --- DEAD CODE: sign flipping done directly in main.c (sp/sr commands)
void gesture_detect_set_sign(bool positive_pitch_is_nod, bool positive_roll_is_right);
*/

/**
 * @brief Phase 5: copy the current sliding baseline `q_drift` into
 *        `out[4]` (w,x,y,z). The quaternion is in the same body frame as
 *        `params.neutral.q_neutral` and is the actual reference the
 *        detector projects motion against, so its angle-distance from
 *        `q_neutral` indicates how much佩戴微调 has been absorbed since
 *        calibration. Used by the `q` BLE console command for diagnostics.
 */
void gesture_detect_get_q_drift(float out[4]);

/**
 * @brief Phase 6: copy the nod_axis / tilt_axis rotated into the current
 *        q_drift frame into `out_nod` / `out_tilt`. These are the actual
 *        vectors the detector projects motion against — what the
 *        q_neutral-frame nod_axis rotates to as q_drift drifts. With no
 *        drift they equal `params.neutral.nod_axis` / `tilt_axis`. Used
 *        by the `p` console command for diagnostics. */
void gesture_detect_get_effective_axes(float out_nod[3], float out_tilt[3]);

/**
 * @brief Phase 5: force the sliding baseline to re-sync from the next
 *        DMP sample. Use after佩戴微调 changes faster than the still-snap
 *        can absorb (e.g. taking the device off and putting it back on
 *        at a very different angle). See gesture_detect.c. */
void gesture_detect_reset_q_drift(void);

/* --- DEAD CODE: old 2-axis calibration (neutral + axes + tilt).
 * Replaced by the rest→gesture flow (cr/cn/ctl/ctr commands).
 * Never called from main.c.
esp_err_t gesture_detect_calibrate_neutral(uint32_t duration_ms);
esp_err_t gesture_detect_calibrate_axes(uint32_t duration_ms);
esp_err_t gesture_detect_calibrate_tilt(uint32_t duration_ms);
*/

/**
 * @brief 5-step calibration: capture the (pn, pt) signature for one gesture.
 *
 *        Call once per gesture type (NOD, LOOK_UP, TILT_LEFT, TILT_RIGHT).
 *        The user performs the requested gesture during `duration_ms`.
 *        The function:
 *          1. records DMP samples and computes r = rotvec(conj(q_drift) ⊗ q)
 *          2. projects r onto (nod_eff, tilt_eff) to get (pn, pt) per frame
 *          3. averages the in-motion (pn, pt) vectors to get the signature
 *          4. stores the signature in s_gd.sig and persists to NVS
 *
 *        After all 4 gestures are calibrated, call
 *        gesture_detect_infer_signs() to auto-set sign_pitch / sign_roll.
 *
 * @param type        GESTURE_NOD, GESTURE_LOOK_UP, GESTURE_TILT_LEFT, or GESTURE_TILT_RIGHT
 * @param duration_ms capture window (≥ 1000 recommended)
 */
esp_err_t gesture_detect_calibrate_gesture(gesture_type_t type, uint32_t duration_ms);

/**
 * @brief Phase A: capture rest quaternion for subsequent gesture calibration.
 *        Call this before gesture_detect_calibrate_gesture() so the gesture
 *        phase skips its internal rest capture and uses this baseline.
 *
 * @param duration_ms how long to sample rest pose (2000 recommended)
 */
esp_err_t gesture_detect_calibrate_rest(uint32_t duration_ms);

/**
 * @brief Auto-infer sign_pitch / sign_roll from the 4 calibrated signatures.
 *
 *        Looks at the pn sign of NOD vs LOOK_UP and the pt sign of
 *        TILT_LEFT vs TILT_RIGHT to determine which sign convention
 *        matches the user's actual motion direction.
 *        Saves the result to NVS.
 */
esp_err_t gesture_detect_infer_signs(void);

/* --- DEAD CODE: legacy alias for calibrate_neutral (also dead)
static inline esp_err_t gesture_detect_calibrate_baseline(uint32_t duration_ms)
{
    return gesture_detect_calibrate_neutral(duration_ms);
}
*/

/* --- DEAD CODE: capture_t and get_last_capture — s_last_cap never populated.
 * calibrate_axes (which wrote s_last_cap) is dead code.
typedef struct {
    uint32_t valid;
    uint32_t used;
    float    nod_axis[3];
    float    tilt_axis[3];
    float    sum_mag_deg;
    float    drift_deg;
} gesture_detect_capture_t;

void gesture_detect_get_last_capture(gesture_detect_capture_t *out);
*/

/**
 * @brief Start a data-capture session.  While active, the detector task
 *        logs every frame's raw metrics (proj, vel, smooth_vel, r_mag,
 *        axis states) at 50 Hz over BLE/UART so the user can collect
 *        gesture data for offline analysis.
 *
 * @param duration_ms  How long to capture.  0 = default 30 s.
 */
void gesture_detect_start_capture(uint32_t duration_ms);

/**
 * @brief Read-only access to the 3 calibrated gesture signatures.
 *        Pointer is to the detector's internal copy; do not modify.
 *        Returns NULL if not yet calibrated.
 *
 *        Signatures are unit rotation axes in the q_drift frame:
 *          sig[0] = nod, sig[1] = tiltL, sig[2] = tiltR
 */
typedef struct {
    float sig_nod[3];
    float sig_tiltL[3];
    float sig_tiltR[3];
} gesture_sig_axes_t;

const gesture_sig_axes_t *gesture_detect_get_sig_axes(void);

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_DETECT_H_ */