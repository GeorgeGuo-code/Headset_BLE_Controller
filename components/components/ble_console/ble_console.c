/*
 * ble_console.c — a minimal Nordic-UART-Service (NUS) style BLE GATT server
 * used as a wireless debug console for the head-gesture device.
 *
 *   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (Write / Write-No-Response, central -> device) : ...0002...
 *   TX (Notify, device -> central)                    : ...0003...  (+ CCCD)
 *
 * Phase 7: this file used to own the whole radio (controller + Bluedroid +
 * the global GATTS/GAP callbacks + advertising). It no longer does. Bluedroid
 * keeps a single global callback of each kind, so owning them here made it
 * impossible to also run BLE HID — whichever component registered last won.
 * `ble_stack` now owns all of that; this file is reduced to one profile in
 * ble_stack's table (app_id 0x0055) plus the log TX plumbing.
 *
 * Attribute-table + create_attr_tab pattern mirrors
 * components/ble_hid/hid_device_le_prf.c; the send path uses the same
 * esp_ble_gatts_send_indicate() primitive as hid_dev.c.
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"

#include "ble_stack.h"
#include "ble_console.h"

#define TAG "ble_console"

#define CONSOLE_APP_ID        0x0055
#define CONSOLE_RX_MAX        1024       /* max single RX write or a prepared-
                                         * write reassembly buffer (bytes). The
                                         * 32-byte default from Phase 7 was tuned
                                         * for stub commands like "p" / "c"; the
                                         * `o <path>` and `cfg <json>` commands
                                         * from Phase 8 need 10× that. See the
                                         * prepared-write handler below — single
                                         * GATT writes are still capped at one
                                         * MTU payload (~244 B), so a > 244-byte
                                         * command must be sent as a series of
                                         * prepared-write chunks followed by an
                                         * execute-prepared-write request. */
#define CONSOLE_TX_STREAM_LEN 2048       /* log StreamBuffer capacity (bytes) */
#define CONSOLE_TX_CHUNK_MAX  244        /* hard cap for one notification */

/* ── 128-bit NUS UUIDs (little-endian byte order for the stack) ──────────── */
static const uint8_t nus_svc_uuid[16] = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E };
static const uint8_t nus_rx_uuid[16]  = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E };
static const uint8_t nus_tx_uuid[16]  = {
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E };

/* ── Attribute table indices ─────────────────────────────────────────────── */
enum {
    IDX_SVC,
    IDX_RX_CHAR,   /* RX characteristic declaration */
    IDX_RX_VAL,    /* RX value (written by central)  */
    IDX_TX_CHAR,   /* TX characteristic declaration */
    IDX_TX_VAL,    /* TX value (notified to central) */
    IDX_TX_CCC,    /* TX client characteristic configuration descriptor */
    IDX_NB,
};

/* ── Declaration constants (shared by attr table) ────────────────────────── */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t char_decl_uuid       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_ccc_uuid        = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  char_prop_write      = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t  char_prop_notify     = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t        tx_ccc_val[2]        = {0x00, 0x00};

/* Full attribute database, created in one shot via create_attr_tab. */
static const esp_gatts_attr_db_t console_gatt_db[IDX_NB] = {
    /* Service declaration — 128-bit service UUID stored as the value. */
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(nus_svc_uuid), sizeof(nus_svc_uuid), (uint8_t *)nus_svc_uuid}},

    /* RX characteristic declaration (write / write-no-response). */
    [IDX_RX_CHAR] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},
    /* RX value — stack-managed buffer (AUTO_RSP), max CONSOLE_RX_MAX bytes. */
    [IDX_RX_VAL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)nus_rx_uuid, ESP_GATT_PERM_WRITE,
         CONSOLE_RX_MAX, 0, NULL}},

    /* TX characteristic declaration (notify only). */
    [IDX_TX_CHAR] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_notify}},
    /* TX value — notified; central does not read it. */
    [IDX_TX_VAL] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, (uint8_t *)nus_tx_uuid, ESP_GATT_PERM_READ,
         CONSOLE_TX_CHUNK_MAX, 0, NULL}},
    /* TX CCCD — central writes 0x0001 to subscribe to notifications. */
    [IDX_TX_CCC] = {{ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_ccc_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(tx_ccc_val), sizeof(tx_ccc_val), (uint8_t *)tx_ccc_val}},
};

/* Advertising, the device name and the security parameters all moved to
 * ble_stack — the NUS service UUID now lives in ble_stack's scan response. */

/* ── Runtime state ───────────────────────────────────────────────────────── */
static esp_gatt_if_t        s_gatts_if      = ESP_GATT_IF_NONE;
static uint16_t             s_conn_id       = 0;
static uint16_t             s_handles[IDX_NB];
static volatile bool        s_connected     = false;
static volatile bool        s_notify_en     = false;
static volatile uint16_t    s_mtu           = 23;   /* default ATT MTU */
static ble_console_cmd_cb_t s_cmd_cb        = NULL;
static StreamBufferHandle_t s_tx_stream     = NULL;

/* Prepared-write reassembly. The central sends "prepare write" chunks (each
 * ≤ MTU−3 bytes) and then "execute prepared writes" to commit. We buffer
 * each chunk at `param->write.offset` and only forward to s_cmd_cb on the
 * EXEC event. A single connection at a time keeps this state simple — the
 * ESP32 only runs one NUS central anyway. */
static struct {
    char     buf[CONSOLE_RX_MAX + 1];
    uint16_t len;
    uint16_t conn_id;
} s_prep = { 0 };

static void prep_reset(void)
{
    s_prep.len     = 0;
    s_prep.conn_id = 0xFFFF;
}

/* ── GATTS event handler — one profile in ble_stack's table ──────────────── */
static void console_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                             esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        /* ble_stack already validated reg.status and claimed the slot; the
         * device name and advertising are its business now. */
        s_gatts_if = gatts_if;
        esp_ble_gatts_create_attr_tab(console_gatt_db, gatts_if, IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "create attr tab failed, status %d", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(TAG, "attr tab handle count mismatch: %d != %d",
                     param->add_attr_tab.num_handle, IDX_NB);
        } else {
            memcpy(s_handles, param->add_attr_tab.handles, sizeof(s_handles));
            esp_ble_gatts_start_service(s_handles[IDX_SVC]);
            ESP_LOGI(TAG, "console service started, svc handle = %d", s_handles[IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id   = param->connect.conn_id;
        s_connected = true;
        s_mtu       = 23;
        ESP_LOGI(TAG, "central connected, conn_id = %d", s_conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_notify_en = false;
        prep_reset();
        ESP_LOGI(TAG, "central disconnected, reason 0x%x",
                 param->disconnect.reason);
        /* ble_stack owns the re-advertise — it gets one DISCONNECT_EVT per
         * registered app_id and must not fire three start_advertising calls. */
        break;

    case ESP_GATTS_MTU_EVT:
        s_mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "MTU negotiated = %d", s_mtu);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.is_prep) {
            /* Prepared-write chunk: stash into s_prep.buf at the requested
             * offset. The execute-prepared-write event (EXEC_WRITE_EVT below)
             * is what finally dispatches the full command. AUTO_RSP handles
             * the protocol response; we just don't ship the chunk as a
             * command on its own. */
            if (param->write.handle != s_handles[IDX_RX_VAL]) {
                break;
            }
            if (param->write.offset == 0) {
                /* offset 0 = start of a fresh reassembly sequence. Reset
                 * even if the previous one was never executed — the only
                 * race here is "central starts a new prepare without
                 * EXEC'ing the old one", which is a malformed client we
                 * don't need to be nice to. */
                s_prep.len       = 0;
                s_prep.conn_id   = param->write.conn_id;
            }
            if (param->write.offset + param->write.len > CONSOLE_RX_MAX) {
                ESP_LOGE(TAG, "prep write overflow (%u + %u > %d) — discarding",
                         (unsigned)param->write.offset,
                         (unsigned)param->write.len,
                         CONSOLE_RX_MAX);
                prep_reset();
                break;
            }
            memcpy(s_prep.buf + param->write.offset,
                   param->write.value, param->write.len);
            s_prep.len = param->write.offset + param->write.len;
            break;
        }
        if (param->write.handle == s_handles[IDX_TX_CCC] && param->write.len == 2) {
            uint16_t cccd = param->write.value[0] | (param->write.value[1] << 8);
            s_notify_en = (cccd & 0x0001) != 0;
            ESP_LOGI(TAG, "TX notifications %s", s_notify_en ? "enabled" : "disabled");
        } else if (param->write.handle == s_handles[IDX_RX_VAL] && s_cmd_cb != NULL) {
            char cmd[CONSOLE_RX_MAX + 1];
            uint16_t n = param->write.len;
            if (n > CONSOLE_RX_MAX) {
                n = CONSOLE_RX_MAX;
            }
            memcpy(cmd, param->write.value, n);
            /* strip trailing CR/LF then NUL-terminate */
            while (n > 0 && (cmd[n - 1] == '\n' || cmd[n - 1] == '\r')) {
                n--;
            }
            cmd[n] = '\0';
            s_cmd_cb(cmd, n);
        }
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        /* The central said "commit". EXEC dispatches the buffered chunks as
         * one command; CANCEL just drops them. Either way the buffer is
         * freed for the next prepared-write sequence. AUTO_RSP handles the
         * protocol response. */
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC &&
            s_prep.len > 0 && s_cmd_cb != NULL) {
            /* Strip trailing CR/LF (the central may have appended them to
             * the last chunk) before NUL-terminating. */
            uint16_t n = s_prep.len;
            while (n > 0 &&
                   (s_prep.buf[n - 1] == '\n' || s_prep.buf[n - 1] == '\r')) {
                n--;
            }
            s_prep.buf[n] = '\0';
            s_cmd_cb(s_prep.buf, n);
        } else if (param->exec_write.exec_write_flag != ESP_GATT_PREP_WRITE_CANCEL) {
            /* Unknown flag — treat defensively. */
            ESP_LOGW(TAG, "exec_write with unknown flag %d",
                     (int)param->exec_write.exec_write_flag);
        }
        prep_reset();
        break;

    default:
        break;
    }
}

/* ── Log TX task: drain the StreamBuffer and notify in MTU-sized chunks ───── */
static void tx_task(void *arg)
{
    (void)arg;
    uint8_t chunk[CONSOLE_TX_CHUNK_MAX];
    for (;;) {
        /* Batch: block up to 50 ms accumulating bytes, then flush one chunk. */
        size_t max = s_mtu > 3 ? (size_t)(s_mtu - 3) : 20;
        if (max > CONSOLE_TX_CHUNK_MAX) {
            max = CONSOLE_TX_CHUNK_MAX;
        }
        size_t n = xStreamBufferReceive(s_tx_stream, chunk, max, pdMS_TO_TICKS(50));
        if (n == 0) {
            continue;
        }
        if (s_connected && s_notify_en && s_gatts_if != ESP_GATT_IF_NONE) {
            esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id, s_handles[IDX_TX_VAL],
                                        (uint16_t)n, chunk, false);
        }
        /* If nobody is subscribed the bytes are simply consumed & dropped
         * (already mirrored to UART at produce time). */
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */
void ble_console_log(const char *s)
{
    if (s == NULL) {
        return;
    }
    /* Always mirror to the UART console so local `idf.py monitor` still works. */
    fputs(s, stdout);
    if (s_tx_stream != NULL) {
        (void)xStreamBufferSend(s_tx_stream, s, strlen(s), 0);  /* non-blocking */
    }
}

void ble_console_logf(const char *fmt, ...)
{
    char line[160];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (len < 0) {
        return;
    }
    ble_console_log(line);
}

bool ble_console_is_connected(void)
{
    return s_connected && s_notify_en;
}

esp_err_t ble_console_init(ble_console_cmd_cb_t on_cmd)
{
    esp_err_t ret;

    s_cmd_cb = on_cmd;

    s_tx_stream = xStreamBufferCreate(CONSOLE_TX_STREAM_LEN, 1);
    if (s_tx_stream == NULL) {
        ESP_LOGE(TAG, "failed to create TX stream buffer");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(tx_task, "ble_con_tx", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create TX task");
        return ESP_ERR_NO_MEM;
    }

    /* Claim a slot in ble_stack's profile table. The radio itself is not
     * touched here — the caller brings it up with ble_stack_start() once every
     * profile has registered. No GAP callback: the console has no GAP-level
     * business now that advertising lives in ble_stack. */
    if ((ret = ble_stack_register_profile(CONSOLE_APP_ID, console_gatts_cb, NULL)) != ESP_OK) {
        ESP_LOGE(TAG, "profile registration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ble_console profile registered (app_id 0x%04x)", CONSOLE_APP_ID);
    return ESP_OK;
}
