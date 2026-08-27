/*
 * DEAD CODE: Phase-3 STUB. Never used — calib_session.h is not included
 * anywhere in the project. The rest→gesture flow (cr/cn/ctl/ctr commands
 * in main.c) replaced this approach. Kept as reference only.
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