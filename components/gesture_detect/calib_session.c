/*
 * Phase-3 STUB. Real implementation lands when calibration session is
 * wired in. Functions are deliberately no-op so callers can compile
 * against the header today and light up later without an API churn.
 */

#include "calib_session.h"

esp_err_t calib_session_start(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t calib_session_abort(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t calib_session_handle_cmd(const char *cmd, size_t len)
{
    (void)cmd;
    (void)len;
    return ESP_ERR_NOT_SUPPORTED;
}

void calib_session_tick(void)
{
}

calib_state_t calib_session_get_state(void)
{
    return CALIB_IDLE;
}