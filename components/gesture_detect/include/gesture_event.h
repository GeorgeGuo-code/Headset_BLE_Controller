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
 * @brief One event handed off from the detector task to the consumer.
 *
 * `conf[4]` holds the dot-product match quality for each gesture type:
 *   [0]=NOD  [1]=LOOK_UP  [2]=TILT_LEFT  [3]=TILT_RIGHT
 *
 * Each value is |dot(smooth_axis, signature)| — range [0,1], where 1 =
 * perfect axis alignment and 0 = orthogonal.  The primary classification
 * is conf[best_idx]; the others show how "close" the rotation axis was
 * to each alternative gesture.  Downstream code uses these to decide
 * which gestures are eligible to fire at a given confidence threshold.
 */
typedef struct {
    gesture_type_t type;             /*!< which gesture was recognised (primary) */
    uint32_t       timestamp_ms;     /*!< ms since boot at fire time */
    float          peak_angle_deg;   /*!< max |angle - neutral| since last fire */
    float          peak_velocity_deg_s; /*!< max |angular velocity| since last fire */
    float          conf[4];          /*!< per-gesture dot-product confidence [0]=NOD..[3]=TILTR */
    int8_t         best_idx;         /*!< index of primary classification (-1 = none) */
} gesture_event_t;

#endif /* GESTURE_EVENT_H_ */