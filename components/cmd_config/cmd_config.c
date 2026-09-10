/*
 * cmd_config — persistent named command configurations.
 *
 * Stores HID seq sequences in NVS, keyed by numeric id (1..CMD_CFG_MAX).
 * Each config has a trigger (gesture or command number) and a name.
 * The seq text parser is extracted from main.c so both `seq` and `cmd set`
 * share the same syntax.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "cmd_config.h"
#include "gesture_event.h"  /* for gesture_type_t values used as trigger keys */

#define TAG "cmd_cfg"

#define NVS_NAMESPACE  "cmds"
#define NVS_KEY_COUNT  "count"
#define NVS_KEY_PREFIX "cfg_"

/* ── In-memory config store ─────────────────────────────────────────────── */

static cmd_config_t s_configs[CMD_CFG_MAX];
static SemaphoreHandle_t s_mutex;

/* Per-config cooldown tracking (not persisted — reset on reboot). */
static uint32_t s_last_fire_ms[CMD_CFG_MAX];

/* Per-config gesture-fulfillment accumulator.
 * Each bit in fulfilled_mask corresponds to a gesture_type_t bit that has
 * been individually satisfied by a past gesture event.  When all bits
 * required by trigger_value are set, the config fires and the mask resets.
 * Fulfilled_ms tracks the timestamp of the last fulfillment update —
 * if it exceeds GESTURE_FULFILL_WINDOW_MS, the mask is cleared (stale). */
static uint16_t s_fulfilled_mask[CMD_CFG_MAX];
static uint32_t s_fulfilled_ms[CMD_CFG_MAX];

#define GESTURE_FULFILL_WINDOW_MS  5000   /* 5 s window to accumulate gestures */

/* ── NVS helpers ────────────────────────────────────────────────────────── */

static esp_err_t nvs_cfg_key(uint8_t id, char *buf, size_t buf_size)
{
    if (id < 1 || id > CMD_CFG_MAX || buf_size < 8) return ESP_ERR_INVALID_ARG;
    snprintf(buf, buf_size, "%s%u", NVS_KEY_PREFIX, (unsigned)id);
    return ESP_OK;
}

/* ── Seq text parser (extracted from main.c seq command) ────────────────── */

#define SEQ_INT_BUF 16

static const char *spi(const char *p, const char *limit, long *out)
{
    while (p < limit && (*p == ' ' || *p == '\t')) p++;
    if (p >= limit) return NULL;

    char buf[SEQ_INT_BUF];
    size_t n = (size_t)(limit - p);
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, p, n);
    buf[n] = '\0';

    char *endp = NULL;
    long v = strtol(buf, &endp, 0);
    if (endp == buf) return NULL;
    *out = v;
    return p + (endp - buf);
}

static const char *spi2(const char *p, const char *limit, long *a, long *b)
{
    p = spi(p, limit, a);
    if (!p) return NULL;
    p = spi(p, limit, b);
    return p;
}

esp_err_t cmd_config_parse_seq(const char *text, hid_seq_step_t *steps, size_t *n_out)
{
    if (!text || !steps || !n_out) return ESP_ERR_INVALID_ARG;

    const char *p = text;
    const char *limit = text + strlen(text);
    size_t n = 0;

    while (*p && n < HID_SEQ_MAX_STEPS) {
        const char *end = strchr(p, ';');
        if (!end) end = limit;

        /* skip leading whitespace */
        const char *tok = p;
        while (tok < end && (*tok == ' ' || *tok == '\t')) tok++;
        if (tok >= end) {
            p = (end < limit) ? end + 1 : end;
            continue;
        }

        /* find end of keyword */
        const char *te = tok;
        while (te < end && *te != ' ' && *te != '\t') te++;
        size_t tlen = (size_t)(te - tok);

        ESP_LOGI(TAG, "[PARSE] step %u: keyword='%.8s' tlen=%u", (unsigned)n, tok, (unsigned)tlen);

        if (tlen == 5 && strncmp(tok, "sleep", 5) == 0) {
            long ms;
            if (spi(te, end, &ms) == NULL || ms <= 0 || ms > 60000) {
                ESP_LOGW(TAG, "parse: bad sleep arg");
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind      = HID_SEQ_SLEEP;
            steps[n].u.sleep.ms = (uint16_t)ms;
            ESP_LOGI(TAG, "[PARSE] -> SLEEP %lu ms", ms);
            n++;
        } else if (tlen == 3 && strncmp(tok, "key", 3) == 0) {
            long mod, kc;
            if (spi2(te, end, &mod, &kc) == NULL) {
                ESP_LOGW(TAG, "parse: bad key args");
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind             = HID_SEQ_KEY;
            steps[n].u.key.modifiers  = (uint8_t)mod;
            steps[n].u.key.keycode    = (uint8_t)kc;
            steps[n].u.key.hold_ms    = 0;
            ESP_LOGI(TAG, "[PARSE] -> KEY mod=%lu kc=%lu", mod, kc);
            n++;
        } else if (tlen == 4 && strncmp(tok, "type", 4) == 0) {
            const char *text_start = te;
            while (text_start < end && (*text_start == ' ' || *text_start == '\t')) text_start++;
            size_t text_len = (size_t)(end - text_start);
            if (text_len == 0 || text_len > HID_SEQ_TEXT_MAX) {
                ESP_LOGW(TAG, "parse: bad type text (len=%u)", (unsigned)text_len);
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind = HID_SEQ_TYPE;
            memcpy(steps[n].u.type.text, text_start, text_len);
            steps[n].u.type.text[text_len] = '\0';
            steps[n].u.type.len = (uint8_t)text_len;
            ESP_LOGI(TAG, "[PARSE] -> TYPE len=%u text='%s'", (unsigned)text_len, steps[n].u.type.text);
            n++;
        } else if (tlen == 5 && strncmp(tok, "click", 5) == 0) {
            const char *arg = te;
            while (arg < end && (*arg == ' ' || *arg == '\t')) arg++;
            size_t alen = (size_t)(end - arg);
            uint8_t btn = 0;
            if      (alen == 4 && strncmp(arg, "left",   4) == 0) btn = 1;
            else if (alen == 5 && strncmp(arg, "right",  5) == 0) btn = 2;
            else if (alen == 6 && strncmp(arg, "middle", 6) == 0) btn = 4;
            else {
                long nb;
                if (spi(arg, end, &nb) != NULL && nb >= 0 && nb <= 7)
                    btn = (uint8_t)nb;
                else {
                    ESP_LOGW(TAG, "parse: bad click arg");
                    return ESP_ERR_INVALID_ARG;
                }
            }
            steps[n].kind            = HID_SEQ_CLICK;
            steps[n].u.click.buttons = btn;
            n++;
        } else if (tlen == 4 && strncmp(tok, "move", 4) == 0) {
            long dx, dy;
            if (spi2(te, end, &dx, &dy) == NULL ||
                dx < -128 || dx > 127 || dy < -128 || dy > 127) {
                ESP_LOGW(TAG, "parse: bad move args");
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind      = HID_SEQ_MOVE;
            steps[n].u.move.dx = (int8_t)dx;
            steps[n].u.move.dy = (int8_t)dy;
            n++;
        } else if (tlen == 6 && strncmp(tok, "scroll", 6) == 0) {
            long clicks;
            if (spi(te, end, &clicks) == NULL ||
                clicks < -128 || clicks > 127) {
                ESP_LOGW(TAG, "parse: bad scroll arg");
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind          = HID_SEQ_SCROLL;
            steps[n].u.scroll.clicks = (int8_t)clicks;
            n++;
        } else {
            ESP_LOGW(TAG, "parse: unknown step '%.*s'", (int)tlen, tok);
            return ESP_ERR_INVALID_ARG;
        }

        p = (end < limit) ? end + 1 : end;
    }

    *n_out = n;
    return (n > 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

/* ── Seq text formatter (steps → semicolon-delimited text) ──────────────── */

esp_err_t cmd_config_format_seq(const cmd_config_t *cfg, char *out, size_t out_size)
{
    if (!cfg || !out || out_size == 0) return ESP_ERR_INVALID_ARG;

    out[0] = '\0';
    size_t pos = 0;

    for (uint8_t i = 0; i < cfg->n_steps && i < HID_SEQ_MAX_STEPS; i++) {
        char buf[HID_SEQ_TEXT_MAX + 16];  /* "type " prefix (5) + text + NUL */
        const hid_seq_step_t *s = &cfg->steps[i];

        switch (s->kind) {
        case HID_SEQ_SLEEP:
            snprintf(buf, sizeof(buf), "sleep %u", (unsigned)s->u.sleep.ms);
            break;
        case HID_SEQ_KEY:
            snprintf(buf, sizeof(buf), "key %u %u",
                     (unsigned)s->u.key.modifiers, (unsigned)s->u.key.keycode);
            break;
        case HID_SEQ_TYPE:
            snprintf(buf, sizeof(buf), "type %s", s->u.type.text);
            break;
        case HID_SEQ_CLICK: {
            const char *name = "?";
            if      (s->u.click.buttons == 1) name = "left";
            else if (s->u.click.buttons == 2) name = "right";
            else if (s->u.click.buttons == 4) name = "middle";
            snprintf(buf, sizeof(buf), "click %s", name);
            break;
        }
        case HID_SEQ_MOVE:
            snprintf(buf, sizeof(buf), "move %d %d",
                     (int)s->u.move.dx, (int)s->u.move.dy);
            break;
        case HID_SEQ_SCROLL:
            snprintf(buf, sizeof(buf), "scroll %d",
                     (int)s->u.scroll.clicks);
            break;
        default:
            snprintf(buf, sizeof(buf), "?");
            break;
        }

        size_t len = strlen(buf);
        if (pos + len + 2 >= out_size) return ESP_ERR_NO_MEM;

        if (pos > 0) { out[pos++] = ';'; out[pos++] = ' '; }
        memcpy(out + pos, buf, len);
        pos += len;
    }
    out[pos] = '\0';
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t cmd_config_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    memset(s_configs, 0, sizeof(s_configs));
    memset(s_last_fire_ms, 0, sizeof(s_last_fire_ms));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s (first boot?)", esp_err_to_name(err));
        return ESP_OK;  /* not fatal — empty store is valid */
    }

    uint8_t count = 0;
    nvs_get_u8(h, NVS_KEY_COUNT, &count);
    if (count > CMD_CFG_MAX) count = CMD_CFG_MAX;

    for (uint8_t i = 0; i < count; i++) {
        char key[8];
        snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, (unsigned)(i + 1));

        cmd_config_t cfg;
        size_t size = sizeof(cfg);
        err = nvs_get_blob(h, key, &cfg, &size);
        if (err == ESP_OK && cfg.id == (i + 1) && size == sizeof(cfg)) {
            s_configs[i] = cfg;
            ESP_LOGI(TAG, "loaded cfg_%u: '%s' (%u steps)", (unsigned)(i + 1), cfg.name, (unsigned)cfg.n_steps);
        } else {
            ESP_LOGD(TAG, "cfg_%u: %s", (unsigned)(i + 1), esp_err_to_name(err));
        }
    }

    nvs_close(h);
    ESP_LOGI(TAG, "loaded %u configs from NVS", (unsigned)count);
    return ESP_OK;
}

const cmd_config_t *cmd_config_get(uint8_t id)
{
    if (id < 1 || id > CMD_CFG_MAX) return NULL;
    if (s_configs[id - 1].id == 0) return NULL;
    return &s_configs[id - 1];
}

esp_err_t cmd_config_set(const cmd_config_t *cfg)
{
    if (!cfg || cfg->id < 1 || cfg->id > CMD_CFG_MAX) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "set id=%u name='%s' n_steps=%u",
             (unsigned)cfg->id, cfg->name, (unsigned)cfg->n_steps);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_configs[cfg->id - 1] = *cfg;
    xSemaphoreGive(s_mutex);

    /* persist to NVS */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char key[8];
    snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, (unsigned)cfg->id);
    err = nvs_set_blob(h, key, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        /* update count */
        uint8_t count = 0;
        for (uint8_t i = 0; i < CMD_CFG_MAX; i++) {
            if (s_configs[i].id != 0) count = i + 1;
        }
        nvs_set_u8(h, NVS_KEY_COUNT, count);
        nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t cmd_config_set_fuzzy(uint8_t id, uint16_t fallback_value,
                               uint16_t cooldown_ms, uint8_t min_confidence)
{
    if (id < 1 || id > CMD_CFG_MAX) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cmd_config_t *cfg = &s_configs[id - 1];
    if (cfg->id == 0) {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "set_fuzzy: cfg_%u does not exist", (unsigned)id);
        return ESP_ERR_NOT_FOUND;
    }
    cfg->fallback_value = fallback_value;
    cfg->cooldown_ms    = cooldown_ms;
    cfg->min_confidence = min_confidence;
    xSemaphoreGive(s_mutex);

    /* persist to NVS */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char key[8];
    snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, (unsigned)id);
    err = nvs_set_blob(h, key, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    ESP_LOGI(TAG, "set_fuzzy id=%u fallback=0x%04x cooldown=%u conf=%u",
             (unsigned)id, (unsigned)fallback_value,
             (unsigned)cooldown_ms, (unsigned)min_confidence);
    return err;
}

esp_err_t cmd_config_delete(uint8_t id)
{
    if (id < 1 || id > CMD_CFG_MAX) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(&s_configs[id - 1], 0, sizeof(cmd_config_t));
    s_last_fire_ms[id - 1] = 0;
    xSemaphoreGive(s_mutex);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    char key[8];
    snprintf(key, sizeof(key), "%s%u", NVS_KEY_PREFIX, (unsigned)id);
    nvs_erase_key(h, key);

    /* update count */
    uint8_t count = 0;
    for (uint8_t i = 0; i < CMD_CFG_MAX; i++) {
        if (s_configs[i].id != 0) count = i + 1;
    }
    nvs_set_u8(h, NVS_KEY_COUNT, count);
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

esp_err_t cmd_config_execute(uint8_t id)
{
    const cmd_config_t *cfg = cmd_config_get(id);
    if (!cfg) return ESP_ERR_NOT_FOUND;
    if (cfg->n_steps == 0) return ESP_ERR_INVALID_STATE;
    return hid_output_send_seq(cfg->steps, cfg->n_steps);
}

void cmd_config_execute_by_trigger(cmd_trigger_type_t type, uint16_t value,
                                   const float conf[4])
{
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    /* ── Diagnostic: log the incoming gesture event ────────────────────── */
    static const char *gesture_names[] = {
        "NONE", "NOD", "LOOK_UP", "TILT_LEFT", "TILT_RIGHT"
    };
    const char *gname = (value <= 4) ? gesture_names[value] : "?";
    if (conf) {
        ESP_LOGI(TAG, "═══ gesture event: %s (val=%u) "
                 "conf=[NOD=%.2f LK=%.2f TL=%.2f TR=%.2f] ═══",
                 gname, (unsigned)value,
                 conf[0], conf[1], conf[2], conf[3]);
    } else {
        ESP_LOGI(TAG, "═══ command event: val=%u ═══", (unsigned)value);
    }

    for (uint8_t i = 0; i < CMD_CFG_MAX; i++) {
        cmd_config_t *cfg = &s_configs[i];
        if (cfg->id == 0) continue;
        if (cfg->trigger_type != type) continue;

        bool match = false;
        const char *match_reason = NULL;

        if (type == TRIGGER_GESTURE && conf) {
            /* ── Multi-gesture accumulator matching ──────────────────────
             *
             * A config with trigger_value = NOD | TILT_LEFT requires BOTH
             * gestures to be individually confirmed across separate events:
             *
             *   Event 1: GESTURE_NOD, conf=[NOD=0.91, TL=0.45]
             *     → NOD confidence 0.91 >= 50 → set NOD bit in fulfilled
             *     → TL confidence not checked (event is NOD, not TL)
             *     → fulfilled=0x02 (NOD only) → not all bits → no fire
             *
             *   Event 2: GESTURE_TILT_LEFT, conf=[NOD=0.20, TL=0.89]
             *     → TL confidence 0.89 >= 50 → set TL bit in fulfilled
             *     → fulfilled=0x0A (NOD|TL) → all bits → FIRE
             *
             * Single gesture_type_t events only satisfy their own type,
             * even if other confidences are also above threshold.
             */
            static const char *glabels[] = { "NOD", "LOOK_UP", "TILT_LEFT", "TILT_RIGHT" };
            uint8_t eff_conf = cfg->min_confidence > 0
                               ? cfg->min_confidence
                               : CMD_CFG_DEFAULT_MIN_CONFIDENCE;

            /* Map gesture_type_t value (1=NOD..4=TILT_RIGHT) to bit position
             * matching trigger_value layout (bit1=NOD, bit2=LOOK_UP, ...). */
            uint16_t event_bit = (1 << value);  /* e.g. GESTURE_NOD=1 → bit1 */
            uint16_t required  = cfg->trigger_value;

            /* Only the event's own gesture type is checked — not all conf[] */
            if (value >= 1 && value <= 4 && (required & event_bit)) {
                float c = conf[value - 1];  /* conf[] is 0-indexed: [0]=NOD */
                bool gate = ((c * 100.0f) >= (float)eff_conf);

                ESP_LOGD(TAG, "  cfg_%u: event=%s conf=%.2f gate=%d eff_conf=%u",
                         (unsigned)cfg->id, glabels[value - 1], c, (int)gate,
                         (unsigned)eff_conf);

                if (gate) {
                    /* Check / update fulfillment accumulator */
                    int idx = i;  /* config slot index */
                    uint32_t now_ms_f = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

                    /* Reset if window expired */
                    if (s_fulfilled_ms[idx] > 0 &&
                        (now_ms_f - s_fulfilled_ms[idx]) > GESTURE_FULFILL_WINDOW_MS) {
                        s_fulfilled_mask[idx] = 0;
                    }

                    s_fulfilled_mask[idx] |= event_bit;
                    s_fulfilled_ms[idx] = now_ms_f;

                    ESP_LOGD(TAG, "  cfg_%u: fulfilled=0x%02x required=0x%02x",
                             (unsigned)cfg->id,
                             (unsigned)s_fulfilled_mask[idx],
                             (unsigned)required);

                    if ((s_fulfilled_mask[idx] & required) == required) {
                        match = true;
                        match_reason = "all gestures fulfilled";
                        s_fulfilled_mask[idx] = 0;  /* reset after fire */
                    }
                }
            }
            if (!match) {
                /* Log which gestures are still unfulfilled */
                int idx = i;
                uint16_t missing = cfg->trigger_value & ~s_fulfilled_mask[idx];
                char buf[80];
                int pos = 0;
                for (int g = 0; g < 4; g++) {
                    uint16_t bit = (1 << (g + 1));
                    if (!(missing & bit)) continue;
                    pos += snprintf(buf + pos, sizeof(buf) - pos, "%s ", glabels[g]);
                }
                ESP_LOGD(TAG, "  cfg_%u '%s': not all gestures fulfilled "
                         "(fulfilled=0x%02x missing=[%s])",
                         (unsigned)cfg->id, cfg->name,
                         (unsigned)s_fulfilled_mask[idx], buf);
            }
        } else {
            match = (cfg->trigger_value == value);
            match_reason = match ? "exact value match" : "value mismatch";
        }

        /* ── Per-config cooldown check ──────────────────────────────────── */
        if (match && cfg->cooldown_ms > 0) {
            uint32_t elapsed = now_ms - s_last_fire_ms[i];
            if (elapsed < cfg->cooldown_ms) {
                ESP_LOGI(TAG, "  cfg_%u '%s': ⛔ SUPPRESSED — cooldown "
                         "%ums / %ums remaining",
                         (unsigned)cfg->id, cfg->name,
                         (unsigned)cfg->cooldown_ms,
                         (unsigned)(cfg->cooldown_ms - elapsed));
                continue;
            }
        }

        /* ── Final decision ────────────────────────────────────────────── */
        if (match && cfg->n_steps > 0) {
            esp_err_t err = hid_output_send_seq(cfg->steps, cfg->n_steps);
            s_last_fire_ms[i] = now_ms;
            ESP_LOGI(TAG, "  cfg_%u '%s': ✅ FIRED via %s — %u steps -> %s",
                     (unsigned)cfg->id, cfg->name, match_reason,
                     (unsigned)cfg->n_steps, esp_err_to_name(err));
        } else if (match && cfg->n_steps == 0) {
            ESP_LOGI(TAG, "  cfg_%u '%s': ⚠️ match but no steps configured",
                     (unsigned)cfg->id, cfg->name);
        } else {
            ESP_LOGD(TAG, "  cfg_%u '%s': — skip", (unsigned)cfg->id, cfg->name);
        }
    }
    ESP_LOGI(TAG, "═══ end trigger evaluation ═══");
}

esp_err_t cmd_config_list(char *out, size_t out_size)
{
    if (!out || out_size == 0) return ESP_ERR_INVALID_ARG;

    out[0] = '\0';
    size_t pos = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < CMD_CFG_MAX; i++) {
        const cmd_config_t *cfg = &s_configs[i];
        if (cfg->id == 0) continue;
        count++;

        /* Format trigger as bitmask: "gesture:5" means bits 0+2 = NOD+TILT_LEFT */
        char tbuf[32];
        if (cfg->trigger_type == TRIGGER_GESTURE) {
            snprintf(tbuf, sizeof(tbuf), "gesture:%u", (unsigned)cfg->trigger_value);
        }
#ifdef ENABLE_SERIAL_TRIGGER
        else if (cfg->trigger_type == TRIGGER_COMMAND) {
            snprintf(tbuf, sizeof(tbuf), "command:%u", (unsigned)cfg->trigger_value);
        }
#endif
        else {
            snprintf(tbuf, sizeof(tbuf), "none");
        }

        char line[260];
        int n = snprintf(line, sizeof(line),
                         "cfg: id=%u name=\"%s\" trigger=%s n_steps=%u",
                         (unsigned)cfg->id, cfg->name, tbuf, (unsigned)cfg->n_steps);

        /* Append fuzzy parameters if any are set. */
        if (cfg->fallback_value != 0 || cfg->cooldown_ms > 0 || cfg->min_confidence > 0) {
            size_t len = strlen(line);
            snprintf(line + len, sizeof(line) - len,
                     " fuzzy: fb=0x%04x cd=%ums conf=%u%%",
                     (unsigned)cfg->fallback_value,
                     (unsigned)cfg->cooldown_ms,
                     (unsigned)cfg->min_confidence);
        }
        {
            size_t len = strlen(line);
            line[len] = '\n';
            line[len + 1] = '\0';
            n = (int)strlen(line);
        }

        if (pos + (size_t)n >= out_size) return ESP_ERR_NO_MEM;
        memcpy(out + pos, line, (size_t)n);
        pos += (size_t)n;
    }

    char summary[32];
    int n = snprintf(summary, sizeof(summary), "cfg: count=%u\n", (unsigned)count);
    if (pos + (size_t)n >= out_size) return ESP_ERR_NO_MEM;
    memcpy(out + pos, summary, (size_t)n);
    pos += (size_t)n;

    out[pos] = '\0';
    return ESP_OK;
}
