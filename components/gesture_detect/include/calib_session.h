#ifndef CALIB_SESSION_H_
#define CALIB_SESSION_H_

/**
 * @file calib_session.h
 *
 * Phase-3 STUB. Function signatures and the command-string grammar are
 * declared here so the rest of the project (and any future Phase-1/2
 * code that wants to wire the command parser) can compile against a
 * stable interface. Implementations in calib_session.c return
 * ESP_ERR_NOT_SUPPORTED for now and are no-ops.
 */

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALIB_IDLE = 0,
    CALIB_PROMPT_NOD,
    CALIB_PROMPT_LOOK_UP,
    CALIB_PROMPT_TILT_LEFT,
    CALIB_PROMPT_TILT_RIGHT,
    CALIB_COMPUTE_PARAMS,
    CALIB_WRITE_NVS,
    CALIB_DONE,
} calib_state_t;

/* ASCII command strings (newline-terminated). Used both by the Phase-3
 * HID vendor-report receiver and the Phase-1 UART console. */
#define CALIB_CMD_START       "CALIB:START"
#define CALIB_CMD_ABORT       "CALIB:ABORT"
#define CALIB_CMD_SET_PREFIX  "CALIB:SET "       /* CALIB:SET key=value */
#define CALIB_CMD_GET_PARAMS  "GET:PARAMS"
#define CALIB_CMD_RESET       "RESET:PARAMS"

esp_err_t     calib_session_start(void);
esp_err_t     calib_session_abort(void);
esp_err_t     calib_session_handle_cmd(const char *cmd, size_t len);
void          calib_session_tick(void);
calib_state_t calib_session_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_SESSION_H_ */