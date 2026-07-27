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

/**
 * @brief Convenience sign-convention override. Equivalent to
 *        apply_params with the sign bytes flipped; does NOT touch the
 *        other thresholds.
 */
void gesture_detect_set_sign(bool positive_pitch_is_nod, bool positive_roll_is_right);

/**
 * @brief Block for `duration_ms` while sampling the DMP quaternion,
 *        average it, and store as the new neutral orientation
 *        (`q_neutral`). Aborts with ESP_FAIL if motion exceeds an internal
 *        guard (relative rotation > 25°), so the user can't accidentally
 *        lock in a non-neutral pose.
 *
 *        Updates both the in-memory params AND NVS. Pair with
 *        gesture_detect_calibrate_axes() to finish calibration.
 */
esp_err_t gesture_detect_calibrate_neutral(uint32_t duration_ms);

/**
 * @brief Block for `duration_ms` while the user performs slow nods, and
 *        derive the two gesture axes (nod_axis, tilt_axis) relative to the
 *        already-calibrated neutral pose. This is what makes detection work
 *        at an arbitrary mounting angle. Requires gesture_detect_calibrate_
 *        neutral() to have run first. Aborts if the nod is too small.
 *
 *        Updates both the in-memory params AND NVS. Specifically overwrites
 *        `nod_axis` (the user's actual nod direction) and, as a temporary
 *        fallback, `tilt_axis` (`up × nod_axis`). For accurate left / right
 *        tilt detection the user should run gesture_detect_calibrate_tilt()
 *        afterwards, which measures the actual tilt axis instead of relying
 *        on the geometric fallback.
 */
esp_err_t gesture_detect_calibrate_axes(uint32_t duration_ms);

/**
 * @brief Block for `duration_ms` while the user performs slow left and
 *        right tilts, and overwrite the persisted `tilt_axis` with a value
 *        measured from the user's actual tilt motion (instead of the
 *        geometric `up × nod_axis` fallback from calibrate_axes()). This
 *        fixes left/right tilt being misrecognised as nod/look-up when
 *        the user's natural tilt direction does not align perfectly with
 *        the geometric axis.
 *
 *        Requires gesture_detect_calibrate_neutral() and
 *        gesture_detect_calibrate_axes() to have run first (we need a
 *        neutral pose and a nod_axis to define the plane perpendicular
 *        to nod in which we measure tilt). Updates both in-memory params
 *        and NVS. Suppresses detector events for the duration of the
 *        capture so the user's calibration gestures don't fire as real
 *        events; restores the prior `calibrated` flag on failure.
 */
esp_err_t gesture_detect_calibrate_tilt(uint32_t duration_ms);

/**
 * @brief Legacy alias kept for compatibility with earlier code paths
 *        that already used the original name.
 */
static inline esp_err_t gesture_detect_calibrate_baseline(uint32_t duration_ms)
{
    return gesture_detect_calibrate_neutral(duration_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* GESTURE_DETECT_H_ */