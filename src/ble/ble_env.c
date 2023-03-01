/******************************************************************************
 * @file    ble_env.c
 * @author  Insight SiP
 * @brief   ble environment module implementation
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include "ble_env.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_ENV_CONFIG_CHAR 0x1201      /**< The UUID of the config Characteristic. */
#define BLE_UUID_ENV_TEMPERATURE_CHAR 0x1202 /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_ENV_PRESSURE_CHAR 0x1203    /**< The UUID of the pressure Characteristic. */
#define BLE_UUID_ENV_HUMIDITY_CHAR 0x1204    /**< The UUID of the humidity Characteristic. */
#define BLE_UUID_ENV_LIGHT_CHAR 0x1205       /**< The UUID of the light Characteristic. */

#define BLE_ENV_MAX_RX_CHAR_LEN BLE_ENV_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_ENV_MAX_TX_CHAR_LEN BLE_ENV_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

// b8c7xxxx-dd70-4c5a-b872-184eac50d00b
#define BASE_UUID                                                                                          \
    {                                                                                                      \
        { 0x0B, 0xD0, 0x50, 0xAC, 0x4E, 0x18, 0x72, 0xB8, 0x5A, 0x4C, 0x70, 0xDD, 0x00, 0x00, 0xC7, 0xB8 } \
    } /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_env     Environment Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_env_t *p_env, ble_evt_t const *p_ble_evt) {
    p_env->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_env     Environment Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_env_t *const p_env, ble_evt_t const *p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_env->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_tes     Environment Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_env_t *p_env, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_env->temperature_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_env->is_temperature_notif_enabled != notif_enabled) {
            p_env->is_temperature_notif_enabled = notif_enabled;

            if (p_env->evt_handler != NULL) {
                p_env->evt_handler(p_env, BLE_ENV_EVT_NOTIF_TEMPERATURE, (uint8_t *)p_evt_write->data, (uint16_t)p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_env->pressure_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_env->is_pressure_notif_enabled != notif_enabled) {
            p_env->is_pressure_notif_enabled = notif_enabled;

            if (p_env->evt_handler != NULL) {
                p_env->evt_handler(p_env, BLE_ENV_EVT_NOTIF_PRESSURE, (uint8_t *)p_evt_write->data, (uint16_t)p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_env->humidity_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (notif_enabled != p_env->is_humidity_notif_enabled) {
            p_env->is_humidity_notif_enabled = notif_enabled;

            if (p_env->evt_handler != NULL) {
                p_env->evt_handler(p_env, BLE_ENV_EVT_NOTIF_HUMIDITY, (uint8_t *)p_evt_write->data, (uint16_t)p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_env->light_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (notif_enabled != p_env->is_light_notif_enabled) {
            p_env->is_light_notif_enabled = notif_enabled;

            if (p_env->evt_handler != NULL) {
                p_env->evt_handler(p_env, BLE_ENV_EVT_NOTIF_LIGHT, (uint8_t *)p_evt_write->data, (uint16_t)p_evt_write->len);
            }
        }
    } else {
        // Do Nothing. This event is not relevant for this service.
    }
}

static void on_authorize_req(ble_env_t *p_env, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_rw_authorize_request_t const *p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
        if (p_evt_rw_authorize_request->request.write.handle == p_env->config_handles.value_handle) {
            ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
            bool valid_data = true;

            // Check for valid data
            if (p_evt_rw_authorize_request->request.write.len != sizeof(ble_env_config_t)) {
                valid_data = false;
            } else {
                ble_env_config_t *p_config = (ble_env_config_t *)p_evt_rw_authorize_request->request.write.data;

                if ((p_config->temperature_interval_ms < BLE_ENV_CONFIG_TEMPERATURE_INT_MIN) ||
                    (p_config->temperature_interval_ms > BLE_ENV_CONFIG_TEMPERATURE_INT_MAX) ||
                    (p_config->pressure_interval_ms < BLE_ENV_CONFIG_PRESSURE_INT_MIN) ||
                    (p_config->pressure_interval_ms > BLE_ENV_CONFIG_PRESSURE_INT_MAX) ||
                    (p_config->humidity_interval_ms < BLE_ENV_CONFIG_HUMIDITY_INT_MIN) ||
                    (p_config->humidity_interval_ms > BLE_ENV_CONFIG_HUMIDITY_INT_MAX)) {
                    valid_data = false;
                }
            }

            rw_authorize_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;

            if (valid_data) {
                rw_authorize_reply.params.write.update = 1;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                rw_authorize_reply.params.write.p_data = p_evt_rw_authorize_request->request.write.data;
                rw_authorize_reply.params.write.len = p_evt_rw_authorize_request->request.write.len;
                rw_authorize_reply.params.write.offset = p_evt_rw_authorize_request->request.write.offset;
            } else {
                rw_authorize_reply.params.write.update = 0;
                rw_authorize_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;
            }

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);

            if (valid_data && (p_env->evt_handler != NULL)) {
                p_env->evt_handler(p_env,
                    BLE_ENV_EVT_CONFIG_RECEIVED,
                    (uint8_t *)p_evt_rw_authorize_request->request.write.data,
                    (uint16_t)p_evt_rw_authorize_request->request.write.len);
            }
        }
    }
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_env       Environment Service structure.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t config_char_add(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_CONFIG_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_env_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_env_init->p_init_config;
    attr_char_value.max_len = sizeof(ble_env_config_t);

    return sd_ble_gatts_characteristic_add(p_env->service_handle,
        &char_md,
        &attr_char_value,
        &p_env->config_handles);
}

/**@brief Function for adding temperature characteristic.
 *
 * @param[in] p_env       Environment Service structure.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t temperature_char_add(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_TEMPERATURE_CHAR;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = ble_uuid.uuid;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(ble_env_temperature_t);
    add_char_params.init_len = sizeof(ble_env_temperature_t);
    add_char_params.p_init_value = (uint8_t *)p_env_init->p_init_temperature;
    ;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_env->service_handle, &add_char_params, &(p_env->temperature_handles));
}

/**@brief Function for adding pressure characteristic.
 *
 * @param[in] p_env       Environment Service structure.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pressure_char_add(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_PRESSURE_CHAR;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = ble_uuid.uuid;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(ble_env_pressure_t);
    add_char_params.init_len = sizeof(ble_env_pressure_t);
    add_char_params.p_init_value = (uint8_t *)p_env_init->p_init_pressure;
    ;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_env->service_handle, &add_char_params, &(p_env->pressure_handles));
}

/**@brief Function for adding humidity characteristic.
 *
 * @param[in] p_env       Environment Service structure.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t humidity_char_add(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_HUMIDITY_CHAR;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = ble_uuid.uuid;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(ble_env_humidity_t);
    add_char_params.init_len = sizeof(ble_env_humidity_t);
    add_char_params.p_init_value = (uint8_t *)p_env_init->p_init_pressure;
    ;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_env->service_handle, &add_char_params, &(p_env->humidity_handles));
}

/**@brief Function for adding light characteristic.
 *
 * @param[in] p_env       Environment Service structure.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t light_char_add(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    ble_uuid_t ble_uuid;
    ble_add_char_params_t add_char_params;

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_LIGHT_CHAR;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = ble_uuid.uuid;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(ble_env_light_t);
    add_char_params.init_len = sizeof(ble_env_light_t);
    add_char_params.p_init_value = (uint8_t *)p_env_init->p_init_pressure;
    ;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_env->service_handle, &add_char_params, &(p_env->light_handles));
}

void ble_env_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    ble_env_t *p_env = (ble_env_t *)p_context;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_env, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_env, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_env, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_authorize_req(p_env, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

uint32_t ble_env_init(ble_env_t *p_env, const ble_env_init_t *p_env_init) {
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_env);
    VERIFY_PARAM_NOT_NULL(p_env_init);

    // Initialize the service structure.
    p_env->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_env->evt_handler = p_env_init->evt_handler;
    p_env->is_temperature_notif_enabled = false;
    p_env->is_pressure_notif_enabled = false;
    p_env->is_humidity_notif_enabled = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_env->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_env->uuid_type;
    ble_uuid.uuid = BLE_UUID_ENV_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_env->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    if (p_env_init->p_init_config != NULL) {
        err_code = config_char_add(p_env, p_env_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the temperature Characteristic.
    if (p_env_init->p_init_temperature != NULL) {
        err_code = temperature_char_add(p_env, p_env_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the pressure Characteristic.
    if (p_env_init->p_init_pressure != NULL) {
        err_code = pressure_char_add(p_env, p_env_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the humidity Characteristic.
    if (p_env_init->p_init_humidity != NULL) {
        err_code = humidity_char_add(p_env, p_env_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the light Characteristic.
    if (p_env_init->p_init_light != NULL) {
        err_code = light_char_add(p_env, p_env_init);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_env_temperature_set(ble_env_t *p_env, ble_env_temperature_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_env_temperature_t);

    VERIFY_PARAM_NOT_NULL(p_env);

    if ((p_env->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_env->is_temperature_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_env->temperature_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_env->conn_handle, &hvx_params);
}

uint32_t ble_env_pressure_set(ble_env_t *p_env, ble_env_pressure_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_env_pressure_t);

    VERIFY_PARAM_NOT_NULL(p_env);

    if ((p_env->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_env->is_pressure_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_env->pressure_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_env->conn_handle, &hvx_params);
}

uint32_t ble_env_humidity_set(ble_env_t *p_env, ble_env_humidity_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_env_humidity_t);

    VERIFY_PARAM_NOT_NULL(p_env);

    if ((p_env->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_env->is_humidity_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_env->humidity_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_env->conn_handle, &hvx_params);
}

uint32_t ble_env_light_set(ble_env_t *p_env, ble_env_light_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_env_light_t);

    VERIFY_PARAM_NOT_NULL(p_env);

    if ((p_env->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_env->is_light_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_env->light_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_env->conn_handle, &hvx_params);
}