/*
 * ble_console.c — a minimal Nordic-UART-Service (NUS) style BLE GATT server
 * used as a wireless debug console for the head-gesture device.
 *
 * Self-contained: brings up the BT controller + Bluedroid + GAP advertising +
 * one GATT service (a single GATTS "profile"/app_id). No BLE HID here — that
 * can be added later as a second app_id.
 *
 *   Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX (Write / Write-No-Response, central -> device) : ...0002...
 *   TX (Notify, device -> central)                    : ...0003...  (+ CCCD)
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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#include "ble_console.h"

#define TAG "ble_console"

#define CONSOLE_DEVICE_NAME   "HMBC-Console"
#define CONSOLE_APP_ID        0x0055
#define CONSOLE_RX_MAX        128        /* max RX write payload we accept */
#define CONSOLE_TX_STREAM_LEN 2048       /* log StreamBuffer capacity (bytes) */
#define CONSOLE_TX_CHUNK_MAX  244        /* hard cap for one notification */
#define CONSOLE_MTU_REQUEST   247

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

/* ── GAP advertising: name in adv, 128-bit service UUID in scan response ──── */
#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)
static uint8_t s_adv_config_done = 0;

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp     = false,
    .include_name     = true,
    .include_txpower  = true,
    .min_interval     = 0x0006,
    .max_interval     = 0x0010,
    .appearance       = 0x0000,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data   = NULL,
    .service_uuid_len = 0,
    .p_service_uuid   = NULL,
    .flag             = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp     = true,
    .include_name     = false,
    .include_txpower  = true,
    .service_uuid_len = sizeof(nus_svc_uuid),
    .p_service_uuid   = (uint8_t *)nus_svc_uuid,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ── Runtime state ───────────────────────────────────────────────────────── */
static esp_gatt_if_t        s_gatts_if      = ESP_GATT_IF_NONE;
static uint16_t             s_conn_id       = 0;
static uint16_t             s_handles[IDX_NB];
static volatile bool        s_connected     = false;
static volatile bool        s_notify_en     = false;
static volatile uint16_t    s_mtu           = 23;   /* default ATT MTU */
static ble_console_cmd_cb_t s_cmd_cb        = NULL;
static StreamBufferHandle_t s_tx_stream     = NULL;

/* ── GAP event handler ───────────────────────────────────────────────────── */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        s_adv_config_done &= ~ADV_CONFIG_FLAG;
        if (s_adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        s_adv_config_done &= ~SCAN_RSP_CONFIG_FLAG;
        if (s_adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "advertising start failed, status %d", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "advertising as \"%s\"", CONSOLE_DEVICE_NAME);
        }
        break;
    default:
        break;
    }
}

/* ── GATTS event handler (single profile) ────────────────────────────────── */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTS reg failed, status %d", param->reg.status);
            return;
        }
        s_gatts_if = gatts_if;
        esp_ble_gap_set_device_name(CONSOLE_DEVICE_NAME);
        s_adv_config_done |= ADV_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&adv_data);
        s_adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&scan_rsp_data);
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
        ESP_LOGI(TAG, "central disconnected, reason 0x%x — re-advertising",
                 param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_MTU_EVT:
        s_mtu = param->mtu.mtu;
        ESP_LOGI(TAG, "MTU negotiated = %d", s_mtu);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.is_prep) {
            break;  /* long writes not expected for short commands */
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

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "bt controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(TAG, "bt controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "gatts register cb failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register cb failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_ble_gatts_app_register(CONSOLE_APP_ID)) != ESP_OK) {
        ESP_LOGE(TAG, "gatts app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_ble_gatt_set_local_mtu(CONSOLE_MTU_REQUEST);
    ESP_LOGI(TAG, "ble_console initialised");
    return ESP_OK;
}
