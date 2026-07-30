#ifndef GESTURE_EVENT_H_
#define GESTURE_EVENT_H_

#include <stdint.h>

/**
 * @brief Discrete head gestures the detector can recognise.
 *
 * The mapping from DMP axis sign → gesture is configurable via
 * gesture_params_t::sign_pitch / sign_roll, so the meaning of "positive
 * pitch" can be flipped at runtime. The names below describe the
 * user's intent, not the sensor axis.
 */
typedef enum {
    GESTURE_NONE      = 0,    /*!< no event (placeholder / queue slot) */
    GESTURE_NOD       = 1,    /*!< chin-down — pitch becomes more negative */
    GESTURE_LOOK_UP   = 2,    /*!< chin-up   — pitch becomes more positive */
    GESTURE_TILT_LEFT = 3,    /*!< left ear toward shoulder — roll becomes more negative */
    GESTURE_TILT_RIGHT= 4,    /*!< right ear toward shoulder — roll becomes more positive */
} gesture_type_t;

/**
 * @brief One event handed off from the detector task to the consumer
 *        (UART logger in Phase 1, BLE HID bridge in Phase 2).
 *
 * The fields are filled at the moment the state machine fires; they are
 * diagnostic only and not consulted by downstream code, but are useful
 * for verifying calibration correctness.
 */
typedef struct {
    gesture_type_t type;             /*!< which gesture was recognised */
    uint32_t       timestamp_ms;     /*!< ms since boot at fire time */
    float          peak_angle_deg;   /*!< max |angle - neutral| since last fire */
    float          peak_velocity_deg_s; /*!< max |angular velocity| since last fire */
} gesture_event_t;

#endif /* GESTURE_EVENT_H_ */