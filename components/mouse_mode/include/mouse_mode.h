#ifndef MOUSE_MODE_H_
#define MOUSE_MODE_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file mouse_mode.h
 *
 * Head-tracking mouse cursor control.
 *
 * When active, the user's head orientation directly controls the mouse
 * cursor. Pitch (nod) maps to vertical movement, roll (tilt) maps to
 * horizontal movement. A dead zone prevents small involuntary motions
 * from moving the cursor.
 *
 * Integration:
 *   mouse_mode_init()          — call once at boot
 *   mouse_mode_tick()          — called from gesture_detect's 50 Hz task
 *   mouse_mode_activate()     — enter mouse mode
 *   mouse_mode_deactivate()   — exit mouse mode
 *
 * The module does NOT read the DMP directly. All data comes from
 * gesture_detect via mouse_mode_tick(), reusing the existing 50 Hz loop.
 */

/** Mouse mode parameters. All tunable at runtime. */
typedef struct {
    float dead_zone_deg;    /*!< Velocity dead zone (°/frame). Head rotation
                                 speed below this is filtered out.
                                 Default 0.3°/frame */
    float speed_ref_deg;    /*!< Velocity (°/frame) at which cursor reaches
                                 max_speed. Linear ramp from dead_zone to
                                 speed_ref. Default 3.0°/frame */
    float max_speed;        /*!< Maximum cursor speed (px/frame). Default 60 */
    uint32_t dwell_ms;      /*!< How long (ms) the head must stay still after
                                 the deactivation gesture before mouse mode
                                 actually exits. Set to 0 to disable dwell
                                 (deactivate immediately on left+right).
                                 Default 0 (disabled) */
} mouse_mode_params_t;

/**
 * @brief Initialise the mouse mode module with default parameters.
 *        Call once at boot, before gesture_detect_start().
 */
esp_err_t mouse_mode_init(void);

/**
 * @brief Activate mouse mode. Records the current orientation as the
 *        rest reference. Touch sensor is enabled for left-click.
 *        Returns ESP_ERR_INVALID_STATE if already active.
 */
esp_err_t mouse_mode_activate(void);

/**
 * @brief Deactivate mouse mode. Touch sensor is disabled.
 *        Returns ESP_ERR_INVALID_STATE if not active.
 */
esp_err_t mouse_mode_deactivate(void);

/**
 * @brief Query whether mouse mode toggle detection is enabled.
 *        When disabled, left+right tilt won't trigger mouse mode.
 */
bool mouse_mode_is_enabled(void);

/**
 * @brief Enable/disable mouse mode toggle detection.
 *        When disabled, left+right tilt won't trigger mouse mode.
 *        Does NOT deactivate an already-active mouse mode.
 */
void mouse_mode_set_enabled(bool enable);

/** @brief Query whether mouse mode is currently active (cursor controlling). */
bool mouse_mode_is_active(void);

/**
 * @brief Enable/disable four-direction (d-pad) mouse mode.
 *
 *        When enabled, cursor moves only in 4 cardinal directions
 *        (up/down/left/right) at a constant slow speed.  The dominant
 *        axis (pitch or roll, whichever has larger projection) determines
 *        the direction.  This mode is easier to control for basic
 *        navigation.
 */
void mouse_mode_set_four_dir(bool enable);
bool mouse_mode_is_four_dir(void);

/**
 * @brief Called from gesture_detect's detector_task at 50 Hz.
 *
 *        Projects the raw rotation vector r (q_drift frame) onto the 3
 *        calibrated signature axes (nod, tiltL, tiltR) to obtain the
 *        head's angular displacement from rest in degrees.
 *        Applies dead zone and linear speed mapping, then sends HID
 *        mouse reports.
 *
 *        r (not smooth_r) is used because smooth_r is normalised to unit
 *        length and loses magnitude information.  r carries actual angle
 *        data and returns to ~0 at rest, which is essential for
 *        position-based cursor control.
 *
 * @param r             Raw rotation vector (3, q_drift frame, degrees)
 * @param r_valid       false when r is not yet initialised
 * @param r_mag         |r| — rotation magnitude (degrees, used for dwell)
 * @param vel           Angular velocity °/s (used for dwell detection)
 */
void mouse_mode_tick(const float r[3], bool r_valid,
                     float r_mag, float vel);

/**
 * @brief Get current parameters (read-only).
 */
const mouse_mode_params_t *mouse_mode_get_params(void);

/**
 * @brief Replace parameters at runtime.
 */
void mouse_mode_set_params(const mouse_mode_params_t *params);

/**
 * @brief Feed a tilt gesture into the toggle detection state machine.
 *
 *        Called from gesture_detect when a TILT_LEFT or TILT_RIGHT event
 *        fires (or would fire — during mouse_mode events are suppressed
 *        but this still runs). The state machine detects the left+right
 *        tilt sequence and toggles mouse mode.
 *
 * @param gesture   GESTURE_TILT_LEFT (3) or GESTURE_TILT_RIGHT (4)
 * @param now_ms    current tick in milliseconds
 */
void mouse_mode_toggle_step(int gesture, uint32_t now_ms);

/**
 * @brief Reset the toggle state machine to idle.
 *        Call when mouse mode is externally activated/deactivated.
 */
void mouse_mode_toggle_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* MOUSE_MODE_H_ */
