/*
 * ble_stack.c — single owner of the BLE radio, the global GATTS/GAP callbacks
 * and the merged advertising payload. See include/ble_stack.h for the why.
 *
 * Phase 7.3: advertising switched from legacy esp_ble_gap_config_adv_data() /
 * esp_ble_gap_start_advertising() to BLE 5.0 extended advertising
 * (esp_ble_gap_ext_adv_*). Legacy ADV_IND stops automatically on the first
 * CONNECT_EVT, which made it impossible to host more than one central at a
 * time — Windows auto-pairing as a HID keyboard would consume the only
 * connection slot and lock out the Electron config tool. BLE 5.0 extended
 * advertising keeps broadcasting while connections exist, so multiple
 * centrals can coexist (up to BT_ACL_CONNECTIONS, default 4). Advertising
 * payload is now raw-encoded AD elements; the BASE_UUID trick needed for
 * legacy 16-bit service UUIDs is no longer required.
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
static volatile uint8_t  s_conn_count = 0;   /* Phase 7.3: tracks live connections
                                              * for multi-connection support */

/* ── Advertising (BLE 5.0 extended, non-legacy) ──────────────────────────── */
#define ADV_INSTANCE 0

/* Raw-encoded ADV payload (25 B):
 *   flags(3) + appearance(4) + complete uuid16 HID(4) + complete local name(14)
 * 0x03C0 HID Generic is intentionally restored here — see the design note in
 * docs/PHASE7_ACTION_HID_DESIGN.md: dropping it in Phase 7.2 stopped Windows
 * from auto-pairing, but also stopped the device from being usable as a HID
 * keyboard at all without a manual Settings → Bluetooth flow. With ext_adv
 * we get both: Windows auto-pairs AND the config tool can connect
 * simultaneously (BT_ACL_CONNECTIONS, default 4). */
static const uint8_t s_adv_raw[] = {
    0x02, 0x01, 0x06,                                              /* Flags: GEN_DISC | BREDR_NOT_SPT */
    0x03, 0x19, 0xC0, 0x03,                                        /* Appearance: 0x03C0 HID Generic */
    0x03, 0x02, 0x12, 0x18,                                        /* Complete List 16-bit UUIDs: HID 0x1812 */
    0x0D, 0x09, 'H','M','B','C','-','C','o','n','s','o','l','e',   /* Complete Local Name */
};

/* Raw-encoded scan response (18 B):
 *   complete 128-bit NUS UUID. Active scanners (nRF Connect, the Electron
 * config tool) see this so they can pick us out among other "HMBC-Console"
 * devices by service. */
static const uint8_t s_scan_rsp_raw[] = {
    0x11, 0x07,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

/* LEGACY + CONNECTABLE + SCANNABLE = legacy ADV_IND, the standard
 * connectable+scannable PDU type. All scanners (Windows, nRF Connect,
 * the Electron config tool) see this; the previous attempt to drop
 * SCANNABLE alone failed because non-LEGACY ext_adv has the
 * CONNECTABLE-/-SCANNABLE-exclusion rule, and the LEGACY-/-CONNECTABLE-
 * alone combination has no legacy PDU type to map to (the controller
 * rejected with HCI 0x12 "Invalid Param"). The full triple is the only
 * way to keep both `connectable` and `scannable` semantics in legacy mode.
 *
 * BLE 5.0 ext_adv rule: BTM (not the controller) rejects CONNECTABLE +
 * SCANNABLE without LEGACY — see `btm_ble_ext_adv_params_validate`. With
 * LEGACY set, the constraint flips to "CONNECTABLE requires SCANNABLE",
 * which is exactly what ADV_IND encodes.
 *
 * Trade-off: legacy ADV_IND stops at the controller level on the first
 * CONNECT_EVT (re-armed on DISCONNECT_EVT below). Two centrals (Windows
 * HID + config tool) can no longer coexist; Phase 7.3's multi-connection
 * goal is deferred — would need two advertising sets to recover. */
static const esp_ble_gap_ext_adv_params_t s_ext_adv_params = {
    .type           = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_SCANNABLE |
                      ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY,
    .interval_min   = 0x20,            /* 32 * 0.625 ms = 20 ms */
    .interval_max   = 0x40,            /* 64 * 0.625 ms = 40 ms */
    .channel_map    = ADV_CHNL_ALL,
    .own_addr_type  = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .tx_power       = 0,               /* controller default */
    .primary_phy    = ESP_BLE_GAP_PRI_PHY_1M,
    .max_skip       = 0,
    .secondary_phy  = ESP_BLE_GAP_PHY_1M,
    .sid            = 0,
    .scan_req_notif = false,
};

static const esp_ble_gap_ext_adv_t s_ext_adv_start = {
    .instance   = ADV_INSTANCE,
    .duration   = 0,        /* 0 = advertise indefinitely */
    .max_events = 0,        /* 0 = unlimited */
};

/* True once ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT comes back SUCCESS.
 * With ext_adv we don't need the old "pending vs active" pair — each
 * ext_adv step is its own command with its own completion event, and the
 * state machine in stack_gap_cb chains them in order. We only need this
 * flag so /hs/ can report "we're actually broadcasting" rather than
 * "we asked the controller to start". */
static volatile bool s_adv_active = false;

/* Kick off the four-step setup: set_params → set_data → set_scan_rsp →
 * start. Called once from ble_stack_start(); the GAP completion events
 * advance the chain. The same chain is re-triggered on DISCONNECT_EVT
 * because legacy ADV_IND stops at the controller level on connect. */
static void start_ext_adv_chain(void)
{
    esp_err_t ret = esp_ble_gap_ext_adv_set_params(ADV_INSTANCE, &s_ext_adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ext_adv_set_params failed: %s", esp_err_to_name(ret));
    }
}

/* ── Global GAP callback ─────────────────────────────────────────────────── */
static void stack_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
            esp_ble_gap_config_ext_adv_data_raw(ADV_INSTANCE,
                                                sizeof(s_adv_raw), s_adv_raw);
        } else {
            ESP_LOGE(TAG, "ext_adv_set_params status %d",
                     param->ext_adv_set_params.status);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            esp_ble_gap_config_ext_scan_rsp_data_raw(ADV_INSTANCE,
                                                    sizeof(s_scan_rsp_raw),
                                                    s_scan_rsp_raw);
        } else {
            ESP_LOGE(TAG, "ext_adv_data_set status %d",
                     param->ext_adv_data_set.status);
        }
        break;

    case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        if (param->scan_rsp_set.status == ESP_BT_STATUS_SUCCESS) {
            esp_ble_gap_ext_adv_start(1, &s_ext_adv_start);
        } else {
            ESP_LOGE(TAG, "ext_scan_rsp_set status %d",
                     param->scan_rsp_set.status);
        }
        break;

    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
            s_adv_active = true;
            ESP_LOGI(TAG, "advertising");
        } else {
            ESP_LOGE(TAG, "ext_adv_start status %d",
                     param->ext_adv_start.status);
        }
        break;

    /* HID over GATT mandates an encrypted link; accept the central's pairing
     * request. Mirrors reference/ble_hidd_demo_main.c. */
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        s_bonded = param->ble_security.auth_cmpl.success;
        if (s_bonded) {
            ESP_LOGI(TAG, "pairing OK — link encrypted, HID reports enabled");
        } else {
            ESP_LOGE(TAG, "pairing failed, reason 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
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
    /* Claim the gatts_if for the slot whose app_id matches. Matching on app_id
     * (not on table index like the IDF demo does) is what lets three profiles
     * coexist — see the header comment. */
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

    /* Phase 7.3 wrote this off because non-legacy ext_adv keeps broadcasting
     * across connects — the controller doesn't stop on CONNECT_EVT. That's
     * no longer true now that LEGACY is set: legacy ADV_IND stops at the
     * controller level on the first CONNECT_EVT, so the device becomes
     * invisible again the moment the central disconnects. Re-arm the chain
     * on every disconnect so paired hosts can reconnect without a reboot.
     * s_adv_active is set back to false so the EXT_ADV_START_COMPLETE_EVT
     * that follows can flip it back to true (and so /hs/ accurately reports
     * "not currently broadcasting" during the brief re-arm window). */
    if (event == ESP_GATTS_CONNECT_EVT) {
        if (s_conn_count < 0xFF) s_conn_count++;
        s_connected = true;
    } else if (event == ESP_GATTS_DISCONNECT_EVT) {
        if (s_conn_count > 0) s_conn_count--;
        if (s_conn_count == 0) {
            s_connected = false;
            s_bonded    = false;
            s_adv_active = false;
            start_ext_adv_chain();
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

    /* Security: bond with the central, no IO capability (Just Works). HID
     * report characteristics are permission-gated on an encrypted link, so
     * without this the host silently drops every report. */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
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

    /* Register every profile. Each produces one ESP_GATTS_REG_EVT, which
     * stack_gatts_cb routes to the matching slot. */
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

    /* Phase 7.3: kick off the four-step BLE 5.0 ext_adv setup chain. The
     * GAP callback advances set_data → set_scan_rsp → start on each
     * completion event. */
    start_ext_adv_chain();

    s_started = true;
    ESP_LOGI(TAG, "started with %d profile(s), name \"%s\"",
             s_profile_count, dev_name ? dev_name : "(unset)");
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