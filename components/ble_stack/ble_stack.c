/*
 * ble_stack.c — single owner of the BLE radio, the global GATTS/GAP callbacks
 * and the merged advertising payload. See include/ble_stack.h for the why.
 *
 * Phase 7.5: dual advertising sets to solve the Windows HID + config tool
 * coexistence problem.
 *
 * The core issue: legacy ADV_IND stops broadcasting on the first CONNECT_EVT.
 * When Windows auto-pairs as a HID keyboard, it consumes the only advertising
 * set, making the device invisible to the Electron config tool.
 *
 * Solution: two legacy ADV_IND advertising sets:
 *   Instance 0 (HID):  appearance 0x03C0 + HID UUID 0x1812 + device name
 *                       → Windows auto-pairs as keyboard
 *   Instance 1 (NUS):  device name (ADV) + NUS 128-bit UUID (scan response)
 *                       → Electron config tool discovers and connects
 *
 * Each instance is a separate legacy ADV_IND. When one central connects to
 * instance 0, it stops — but instance 1 keeps broadcasting. The other
 * central (config tool) can still discover and connect via instance 1.
 * Both connections coexist on the same BLE controller (BT_ACL_CONNECTIONS).
 */

#include <string.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#include "ble_stack.h"
#include "gesture_detect.h"

#define TAG "ble_stack"

#define BLE_STACK_MTU_REQUEST 247

/* ── Profile table ───────────────────────────────────────────────────────── */
typedef struct {
    bool                   used;
    uint16_t               app_id;
    esp_gatt_if_t          gatts_if;
    ble_profile_gatts_cb_t gatts_cb;
    ble_profile_gap_cb_t   gap_cb;
} profile_slot_t;

static profile_slot_t s_profiles[BLE_STACK_MAX_PROFILES];
static int            s_profile_count = 0;
static bool           s_started       = false;

/* ── Runtime link state ──────────────────────────────────────────────────── */
static volatile bool     s_bonded     = false;
static volatile bool     s_connected  = false;
static volatile uint8_t  s_conn_count = 0;

/* ── Dual advertising instances ──────────────────────────────────────────── */
#define HID_INSTANCE  0       /* Windows HID auto-pair */
#define NUS_INSTANCE  1       /* Electron config tool */

/* Shared device name — must fit in 31-byte ADV budget with other AD elements. */
#define DEV_NAME_AD_LEN  14   /* length of "HMBC-Console" + type byte */
#define DEV_NAME_STR     'H','M','B','C','-','C','o','n','s','o','l','e'

/* ── Instance 0 (HID) — for Windows auto-pairing ────────────────────────── */

/* ADV payload (25 B): flags + appearance(HID Generic) + UUID16(HID) + name */
static const uint8_t s_hid_adv_raw[] = {
    0x02, 0x01, 0x06,                         /* Flags: GEN_DISC | BREDR_NOT_SPT */
    0x03, 0x19, 0xC0, 0x03,                   /* Appearance: 0x03C0 HID Generic */
    0x03, 0x02, 0x12, 0x18,                   /* Complete List 16-bit UUIDs: HID 0x1812 */
    0x0D, 0x09, DEV_NAME_STR,                 /* Complete Local Name */
};

static const esp_ble_gap_ext_adv_params_t s_hid_adv_params = {
    .type           = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY,
    .interval_min   = 0x20,            /* 20 ms */
    .interval_max   = 0x40,            /* 40 ms */
    .channel_map    = ADV_CHNL_ALL,
    .own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power       = 0,
    .primary_phy    = ESP_BLE_GAP_PRI_PHY_1M,
    .max_skip       = 0,
    .secondary_phy  = ESP_BLE_GAP_PHY_1M,
    .sid            = HID_INSTANCE,
    .scan_req_notif = false,
};

static const esp_ble_gap_ext_adv_t s_hid_adv_start = {
    .instance   = HID_INSTANCE,
    .duration   = 0,
    .max_events = 0,
};

/* ── Instance 1 (NUS) — for Electron config tool ────────────────────────── */

/* ADV payload (17 B): flags + name.
 * NUS UUID goes in scan response so active scanners (Electron, nRF Connect)
 * see it via scan request. */
static const uint8_t s_nus_adv_raw[] = {
    0x02, 0x01, 0x06,                         /* Flags: GEN_DISC | BREDR_NOT_SPT */
    0x0D, 0x09, DEV_NAME_STR,                 /* Complete Local Name */
};

/* Scan response (18 B): complete 128-bit NUS service UUID. */
static const uint8_t s_nus_scan_rsp_raw[] = {
    0x11, 0x07,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static const esp_ble_gap_ext_adv_params_t s_nus_adv_params = {
    .type           = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY,
    .interval_min   = 0x30,            /* 30 ms — slightly slower than HID */
    .interval_max   = 0x60,            /* 60 ms */
    .channel_map    = ADV_CHNL_ALL,
    .own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power       = 0,
    .primary_phy    = ESP_BLE_GAP_PRI_PHY_1M,
    .max_skip       = 0,
    .secondary_phy  = ESP_BLE_GAP_PHY_1M,
    .sid            = NUS_INSTANCE,
    .scan_req_notif = false,
};

static const esp_ble_gap_ext_adv_t s_nus_adv_start = {
    .instance   = NUS_INSTANCE,
    .duration   = 0,
    .max_events = 0,
};

/* ── Advertising state machine ─────────────────────────────────────────────
 * The two chains run sequentially: HID first, then NUS. Each chain is
 * four steps: set_params → set_data → set_scan_rsp → start.
 * The GAP callback advances the chain based on instance ID. */
static volatile bool s_hid_adv_active = false;
static volatile bool s_nus_adv_active = false;

/* Forward declaration — the GAP callback references this. */
static void start_nus_adv_chain(void);

static void start_hid_adv_chain(void)
{
    esp_err_t ret = esp_ble_gap_ext_adv_set_params(HID_INSTANCE, &s_hid_adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID ext_adv_set_params failed: %s", esp_err_to_name(ret));
    }
}

static void start_nus_adv_chain(void)
{
    esp_err_t ret = esp_ble_gap_ext_adv_set_params(NUS_INSTANCE, &s_nus_adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NUS ext_adv_set_params failed: %s", esp_err_to_name(ret));
    }
}

/* Re-arm both advertising sets on disconnect. */
static void rearm_all_advertising(void)
{
    s_hid_adv_active = false;
    s_nus_adv_active = false;
    start_hid_adv_chain();
}

/* ── Global GAP callback ─────────────────────────────────────────────────── */
static void stack_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    /* ── set_params completion ─────────────────────────────────────────── */
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
            uint8_t inst = param->ext_adv_set_params.instance;
            const uint8_t *data = (inst == HID_INSTANCE) ? s_hid_adv_raw : s_nus_adv_raw;
            size_t len = (inst == HID_INSTANCE) ? sizeof(s_hid_adv_raw) : sizeof(s_nus_adv_raw);
            esp_ble_gap_config_ext_adv_data_raw(inst, len, data);
        } else {
            ESP_LOGE(TAG, "ext_adv_set_params[%d] status %d",
                     param->ext_adv_set_params.instance,
                     param->ext_adv_set_params.status);
        }
        break;

    /* ── adv_data completion ───────────────────────────────────────────── */
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            uint8_t inst = param->ext_adv_data_set.instance;
            if (inst == HID_INSTANCE) {
                /* HID instance has no scan response — go straight to start. */
                esp_ble_gap_ext_adv_start(1, &s_hid_adv_start);
            } else {
                /* NUS instance has scan response with NUS UUID. */
                esp_ble_gap_config_ext_scan_rsp_data_raw(NUS_INSTANCE,
                                                         sizeof(s_nus_scan_rsp_raw),
                                                         s_nus_scan_rsp_raw);
            }
        } else {
            ESP_LOGE(TAG, "ext_adv_data_set[%d] status %d",
                     param->ext_adv_data_set.instance,
                     param->ext_adv_data_set.status);
        }
        break;

    /* ── scan_rsp completion (NUS instance only) ───────────────────────── */
    case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        if (param->scan_rsp_set.status == ESP_BT_STATUS_SUCCESS) {
            /* NUS scan response is ready — start NUS advertising. */
            esp_ble_gap_ext_adv_start(1, &s_nus_adv_start);
        } else {
            ESP_LOGE(TAG, "ext_scan_rsp_set[%d] status %d",
                     param->scan_rsp_set.instance,
                     param->scan_rsp_set.status);
        }
        break;

    /* ── ext_adv_start completion ──────────────────────────────────────── */
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
            /* ext_adv_start reports num_set (count of started sets) but not
             * which instance. We infer: if HID was not yet active, this is
             * HID; otherwise it's NUS. */
            if (!s_hid_adv_active) {
                s_hid_adv_active = true;
                ESP_LOGI(TAG, "HID advertising started (instance %d)", HID_INSTANCE);
                /* Now kick off the NUS chain. */
                start_nus_adv_chain();
            } else {
                s_nus_adv_active = true;
                ESP_LOGI(TAG, "NUS advertising started (instance %d)", NUS_INSTANCE);
            }
        } else {
            ESP_LOGE(TAG, "ext_adv_start status %d",
                     param->ext_adv_start.status);
        }
        break;

    /* HID over GATT mandates an encrypted link; accept the central's pairing
     * request. Mirrors reference/ble_hidd_demo_main.c. */
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "[SEC] SEC_REQ from "ESP_BD_ADDR_STR"",
                 ESP_BD_ADDR_HEX(param->ble_security.ble_req.bd_addr));
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        s_bonded = param->ble_security.auth_cmpl.success;
        ESP_LOGI(TAG, "[SEC] AUTH_CMPL: success=%d fail_reason=0x%x addr="ESP_BD_ADDR_STR"",
                 (int)param->ble_security.auth_cmpl.success,
                 param->ble_security.auth_cmpl.fail_reason,
                 ESP_BD_ADDR_HEX(param->ble_security.auth_cmpl.bd_addr));
        if (s_bonded) {
            ESP_LOGI(TAG, "pairing OK — link encrypted, HID reports enabled");
        } else {
            ESP_LOGE(TAG, "pairing failed, reason 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        /* Restore NO_BOND mode so that NUS (config-tool) connections do not
         * trigger a security request.  See ble_stack_request_bonding(). */
        {
            esp_ble_auth_req_t no_bond = ESP_LE_AUTH_NO_BOND;
            esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE,
                                           &no_bond, sizeof(no_bond));
            ESP_LOGI(TAG, "[SEC] auth mode restored to NO_BOND");
        }
        break;

    default:
        break;
    }

    /* GAP events carry no gatts_if, so every profile sees every event. */
    for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
        if (s_profiles[i].used && s_profiles[i].gap_cb != NULL) {
            s_profiles[i].gap_cb(event, param);
        }
    }
}

/* ── Global GATTS callback ───────────────────────────────────────────────── */
static void stack_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "app_id 0x%04x registration failed, status %d",
                     param->reg.app_id, param->reg.status);
            return;
        }
        for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
            if (s_profiles[i].used && s_profiles[i].app_id == param->reg.app_id) {
                s_profiles[i].gatts_if = gatts_if;
                ESP_LOGI(TAG, "app_id 0x%04x registered, gatts_if = %d",
                         param->reg.app_id, gatts_if);
                break;
            }
        }
    }

    /* Dual advertising: each legacy ADV_IND instance stops independently
     * when a central connects to it. We re-arm ALL instances on disconnect
     * so the device is fully discoverable again. */
    if (event == ESP_GATTS_CONNECT_EVT) {
        if (s_conn_count < 0xFF) s_conn_count++;
        s_connected = true;
        ESP_LOGI(TAG, "[CONN] CONNECT conn_id=%u gatts_if=%d addr="ESP_BD_ADDR_STR" count=%d",
                 (unsigned)param->connect.conn_id, (int)gatts_if,
                 ESP_BD_ADDR_HEX(param->connect.remote_bda), (int)s_conn_count);
        gesture_detect_reset_calibration();
    } else if (event == ESP_GATTS_DISCONNECT_EVT) {
        ESP_LOGI(TAG, "[CONN] DISCONNECT conn_id=%u reason=0x%x count=%d",
                 (unsigned)param->disconnect.conn_id,
                 param->disconnect.reason, (int)s_conn_count - 1);
        if (s_conn_count > 0) s_conn_count--;
        if (s_conn_count == 0) {
            s_connected = false;
            s_bonded    = false;
            rearm_all_advertising();
        }
    }

    for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
        if (!s_profiles[i].used || s_profiles[i].gatts_cb == NULL) {
            continue;
        }
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == s_profiles[i].gatts_if) {
            s_profiles[i].gatts_cb(event, gatts_if, param);
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */
esp_err_t ble_stack_register_profile(uint16_t app_id,
                                     ble_profile_gatts_cb_t gatts_cb,
                                     ble_profile_gap_cb_t gap_cb)
{
    if (s_started) {
        ESP_LOGE(TAG, "register_profile(0x%04x) after start", app_id);
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
        if (s_profiles[i].used && s_profiles[i].app_id == app_id) {
            ESP_LOGE(TAG, "duplicate app_id 0x%04x", app_id);
            return ESP_ERR_INVALID_ARG;
        }
    }
    for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
        if (!s_profiles[i].used) {
            s_profiles[i].used     = true;
            s_profiles[i].app_id   = app_id;
            s_profiles[i].gatts_if = ESP_GATT_IF_NONE;
            s_profiles[i].gatts_cb = gatts_cb;
            s_profiles[i].gap_cb   = gap_cb;
            s_profile_count++;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "profile table full (%d slots)", BLE_STACK_MAX_PROFILES);
    return ESP_ERR_NO_MEM;
}

esp_err_t ble_stack_start(const char *dev_name)
{
    esp_err_t ret;

    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_profile_count == 0) {
        ESP_LOGE(TAG, "no profiles registered");
        return ESP_ERR_INVALID_STATE;
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

    if ((ret = esp_ble_gatts_register_callback(stack_gatts_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gatts register cb failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_ble_gap_register_callback(stack_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register cb failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Security: NO_BOND mode — the device does NOT force pairing on every
     * connection.  HID still gets encrypted because hid_device_le_prf.c
     * explicitly calls esp_ble_set_encryption() on connect (line 573),
     * which makes Windows initiate pairing.  The NUS config-tool connection
     * stays unencrypted, which avoids the Web-Bluetooth bonding failures
     * that caused every GATT write to fail and the link to drop. */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    uint8_t            key_size = 16;
    uint8_t            init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t            rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(uint8_t));

    esp_ble_gatt_set_local_mtu(BLE_STACK_MTU_REQUEST);

    if (dev_name != NULL) {
        esp_ble_gap_set_device_name(dev_name);
    }

    /* Register every profile. */
    for (int i = 0; i < BLE_STACK_MAX_PROFILES; i++) {
        if (!s_profiles[i].used) {
            continue;
        }
        if ((ret = esp_ble_gatts_app_register(s_profiles[i].app_id)) != ESP_OK) {
            ESP_LOGE(TAG, "app_register(0x%04x) failed: %s",
                     s_profiles[i].app_id, esp_err_to_name(ret));
            return ret;
        }
    }

    /* Phase 7.5: start dual advertising chains. HID first, then NUS
     * (NUS is kicked off by the GAP callback when HID completes). */
    start_hid_adv_chain();

    s_started = true;
    ESP_LOGI(TAG, "started with %d profile(s), name \"%s\" (dual adv: HID=%d NUS=%d)",
             s_profile_count, dev_name ? dev_name : "(unset)",
             HID_INSTANCE, NUS_INSTANCE);
    return ESP_OK;
}

bool ble_stack_is_bonded(void)
{
    return s_bonded;
}

bool ble_stack_is_connected(void)
{
    return s_connected;
}

void ble_stack_request_bonding(void)
{
    esp_ble_auth_req_t bond = ESP_LE_AUTH_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE,
                                   &bond, sizeof(bond));
    ESP_LOGI(TAG, "auth mode temporarily set to BOND (for HID pairing)");
}
