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

        if (tlen == 5 && strncmp(tok, "sleep", 5) == 0) {
            long ms;
            if (spi(te, end, &ms) == NULL || ms <= 0 || ms > 60000) {
                ESP_LOGW(TAG, "parse: bad sleep arg");
                return ESP_ERR_INVALID_ARG;
            }
            steps[n].kind      = HID_SEQ_SLEEP;
            steps[n].u.sleep.ms = (uint16_t)ms;
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
        char buf[128];
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
        } else {
            ESP_LOGW(TAG, "load cfg_%u failed: %s", (unsigned)(i + 1), esp_err_to_name(err));
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

esp_err_t cmd_config_delete(uint8_t id)
{
    if (id < 1 || id > CMD_CFG_MAX) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(&s_configs[id - 1], 0, sizeof(cmd_config_t));
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

void cmd_config_execute_by_trigger(cmd_trigger_type_t type, uint16_t value)
{
    for (uint8_t i = 0; i < CMD_CFG_MAX; i++) {
        const cmd_config_t *cfg = &s_configs[i];
        if (cfg->id == 0) continue;
        if (cfg->trigger_type != type) continue;

        bool match = false;
        if (type == TRIGGER_GESTURE) {
            /* Bitmask match: value is a single gesture (1<<gesture_type),
             * trigger_value is a bitmask of allowed gestures.
             * Any bit matching = trigger (OR logic). */
            match = (cfg->trigger_value & (1 << value)) != 0;
        } else {
            match = (cfg->trigger_value == value);
        }

        if (match && cfg->n_steps > 0) {
            esp_err_t err = hid_output_send_seq(cfg->steps, cfg->n_steps);
            ESP_LOGI(TAG, "trigger cfg_%u '%s' -> %s",
                     (unsigned)cfg->id, cfg->name, esp_err_to_name(err));
        }
    }
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

        char line[200];
        int n = snprintf(line, sizeof(line), "cfg: id=%u name=\"%s\" trigger=%s n_steps=%u\n",
                         (unsigned)cfg->id, cfg->name, tbuf, (unsigned)cfg->n_steps);

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
