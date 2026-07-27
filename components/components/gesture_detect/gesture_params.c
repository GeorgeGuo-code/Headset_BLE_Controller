/*
 * gesture_params.c — NVS-backed persistence for gesture_params_t.
 *
 * Layout:
 *   namespace = "gesture"
 *   key       = "params"
 *   blob      = sizeof(gesture_params_t) = 64 bytes (v2)
 *
 * Magic + version fields gate whether a read is treated as valid; a
 * corrupt or older payload triggers fallback to defaults (which is
 * never auto-saved — the caller decides whether to persist).
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "gesture_params.h"
#include "gesture_detect.h"   /* for gesture_detect_get_params / apply_params */

static const char *TAG = "gesture_params";

/* ===== default factory =================================================== */

void gesture_params_load_default(gesture_params_t *out)
{
    memset(out, 0, sizeof(*out));
    out->magic                  = GESTURE_PARAMS_MAGIC;
    out->version                = GESTURE_PARAMS_VERSION;
    /* Identity orientation + placeholder axes. Detection is meaningless
     * until the user calibrates (neutral + nod), which overwrites these. */
    out->neutral.q_neutral[0]   = 1.0f;   /* w */
    out->neutral.q_neutral[1]   = 0.0f;   /* x */
    out->neutral.q_neutral[2]   = 0.0f;   /* y */
    out->neutral.q_neutral[3]   = 0.0f;   /* z */
    out->neutral.nod_axis[0]    = 0.0f;
    out->neutral.nod_axis[1]    = 1.0f;   /* placeholder: about body Y */
    out->neutral.nod_axis[2]    = 0.0f;
    out->neutral.tilt_axis[0]   = 1.0f;   /* placeholder: about body X */
    out->neutral.tilt_axis[1]   = 0.0f;
    out->neutral.tilt_axis[2]   = 0.0f;
    out->trigger_deg            = GESTURE_DEFAULT_TRIGGER_DEG;
    out->trigger_velocity_deg_s = GESTURE_DEFAULT_TRIGGER_VEL_DEG_S;
    out->neutral_zone_deg       = GESTURE_DEFAULT_NEUTRAL_ZONE_DEG;
    out->debounce_ms            = GESTURE_DEFAULT_DEBOUNCE_MS;
    out->sign_pitch             = GESTURE_DEFAULT_SIGN_PITCH;
    out->sign_roll              = GESTURE_DEFAULT_SIGN_ROLL;
    memset(out->reserved, 0, sizeof(out->reserved));
}

/* ===== NVS helpers ======================================================= */

/**
 * @brief Open the "gesture" namespace, creating it if needed. Returns
 *        ESP_ERR_NVS_NOT_FOUND when the partition itself is missing
 *        (caller may need to call nvs_flash_init first).
 */
static esp_err_t open_namespace(nvs_handle_t *out)
{
    return nvs_open(GESTURE_NVS_NAMESPACE, NVS_READWRITE, out);
}

/**
 * @brief Read the persisted blob into *out. Returns:
 *   - ESP_OK                 : blob read and validated
 *   - ESP_ERR_NVS_NOT_FOUND  : key doesn't exist (NVS empty)
 *   - ESP_ERR_INVALID_SIZE   : blob exists but wrong size
 *   - ESP_ERR_INVALID_CRC    : blob exists but magic/version mismatch
 *   - anything else          : bubble up
 *
 * On any failure other than ESP_OK, *out is left in an undefined state
 * and the caller should load defaults.
 */
esp_err_t gesture_params_load_from_nvs(gesture_params_t *out)
{
    nvs_handle_t h;
    esp_err_t err = open_namespace(&h);
    if (err != ESP_OK) {
        return err;
    }

    size_t size = 0;
    err = nvs_get_blob(h, GESTURE_NVS_KEY, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(h);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (err != ESP_OK) {
        nvs_close(h);
        return err;
    }
    if (size != sizeof(gesture_params_t)) {
        ESP_LOGW(TAG, "blob size mismatch: got %u expected %u — "
                      "treating as old schema and using defaults",
                 (unsigned)size, (unsigned)sizeof(gesture_params_t));
        nvs_close(h);
        return ESP_ERR_INVALID_SIZE;
    }

    err = nvs_get_blob(h, GESTURE_NVS_KEY, out, &size);
    nvs_close(h);
    if (err != ESP_OK) {
        return err;
    }

    if (out->magic != GESTURE_PARAMS_MAGIC || out->version != GESTURE_PARAMS_VERSION) {
        ESP_LOGW(TAG, "magic/version mismatch (got magic=0x%08lx ver=%u)",
                 (unsigned long)out->magic, (unsigned)out->version);
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

esp_err_t gesture_params_save_to_nvs(const gesture_params_t *params)
{
    if (params->magic != GESTURE_PARAMS_MAGIC ||
        params->version != GESTURE_PARAMS_VERSION) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = open_namespace(&h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, GESTURE_NVS_KEY, params, sizeof(*params));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/**
 * @brief Convenience: load defaults into *out, then attempt to commit
 *        them to NVS (so subsequent boots see a valid blob). Used by
 *        main.c on first boot to keep NVS from staying "empty".
 */
esp_err_t gesture_params_reset_default(gesture_params_t *out)
{
    gesture_params_load_default(out);
    return gesture_params_save_to_nvs(out);
}

/* ===== Aligned mirror ====================================================
 * gesture_params_t is __attribute__((packed)) for NVS layout stability, so
 * its members may sit on unaligned addresses. xtensa GCC's
 * -Werror=address-of-packed-member forbids taking & on those members and
 * passing them as float* to helpers. The detector and calibration code
 * therefore work on a stack-local neutral_pose_aligned_t (no packed
 * attribute) — see gesture_params.h for the why. The packed struct
 * remains the single source of truth; the getter/setter forward through
 * the detector's public API. */

void gesture_params_get_neutral_aligned(neutral_pose_aligned_t *out)
{
    const gesture_params_t *p = gesture_detect_get_params();
    memcpy(out, &p->neutral, sizeof(*out));
}

void gesture_params_set_neutral_aligned(const neutral_pose_aligned_t *src)
{
    gesture_params_t params = *gesture_detect_get_params();
    memcpy(&params.neutral, src, sizeof(*src));
    /* apply_params copies the whole struct into the detector's s_gd.
     * Persist so the calibration survives reboot. */
    gesture_detect_apply_params(&params);
    gesture_params_save_to_nvs(&params);
}