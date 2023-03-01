/******************************************************************************
 * @file    ble_isp4580.c
 * @author  Insight SiP
 * @brief   LoRa mngt ble service implementation file.
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

#include "ble_isp4580.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_ISP4580_CONFIG_CHAR 0x1101 /**< The UUID of the lora config Characteristic. */
#define BLE_UUID_ISP4580_RXDIAG_CHAR 0x1102 /**< The UUID of the lora rx diagnostic Characteristic. */

#define BLE_ISP4580_MAX_RX_CHAR_LEN BLE_ISP4580T_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_ISP4580_MAX_TX_CHAR_LEN BLE_ISP4580_MAX_DATA_LEN  /**< Maximum length of the TX Characteristic (in bytes). */

// 7ed3xxxx-c005-11e8-a355-529269fb1459
#define ISP4580_BASE_UUID                                                                                  \
    {                                                                                                      \
        { 0x59, 0x14, 0xFB, 0x69, 0x92, 0x52, 0x55, 0xA3, 0xE8, 0x11, 0x05, 0xC0, 0x00, 0x00, 0xD3, 0x7E } \
    } /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_isp4580    isp4580 service structure.
 * @param[in] p_ble_evt     Pointer to the event received from BLE stack.
 */
static void on_connect(ble_isp4580_t *p_isp4580, ble_evt_t const *p_ble_evt) {
    p_isp4580->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_isp4580    isp4580 Service structure.
 * @param[in] p_ble_evt    Pointer to the event received from BLE stack.
 */
static void on_write(ble_isp4580_t *p_isp4580, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_isp4580->rxdiag_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled;

        notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_isp4580->is_rxdiag_notif_enabled != notif_enabled) {
            p_isp4580->is_rxdiag_notif_enabled = notif_enabled;

            if (p_isp4580->evt_handler != NULL) {
                p_isp4580->evt_handler(p_isp4580, BLE_ISP4580_EVT_NOTIF_RXDIAG, (uint8_t *)p_evt_write->data, (uint16_t)p_evt_write->len);
            }
        }
    }
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_isp4580         isp4580 service structure.
 * @param[in] p_ble_evt 	Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_isp4580_t *p_isp4580, ble_evt_t const *p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_isp4580->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_authorize_req(ble_isp4580_t *p_isp4580, ble_evt_t const *p_ble_evt) {
    ble_gatts_evt_rw_authorize_request_t const *p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
        ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
        bool valid_data = true;
        bool reply = true;
        ble_isp4580_evt_type_t evt_type = BLE_ISP4580_EVT_CONFIG_RECEIVED;

        if (p_evt_rw_authorize_request->request.write.handle == p_isp4580->config_handles.value_handle) {
            // Check for valid data
            if (p_evt_rw_authorize_request->request.write.len != sizeof(ble_isp4580_config_t)) {
                valid_data = false;
            } else {
                ble_isp4580_config_t *p_data = (ble_isp4580_config_t *)p_evt_rw_authorize_request->request.write.data;

                evt_type = BLE_ISP4580_EVT_CONFIG_RECEIVED;

                if ((p_data->data_rate > ISP4580_DATA_RATE_MAX) ||
                    (p_data->tx_power > ISP4580_TX_POWER_MAX) ||
                    (p_data->tx_request_interval > ISP4580_TX_INTERVAL_MAX) ||
                    (p_data->tx_request_interval < ISP4580_TX_INTERVAL_MIN) ||
                    (p_data->enable_adr > ISP4580_ADR_MAX)) {
                    valid_data = false;
                }
            }
        } else {
            valid_data = false;
            reply = false;
        }

        if (reply) {
            // Reply depending on valid data or not
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

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);
        }

        // Call event handler
        if (valid_data && (p_isp4580->evt_handler != NULL)) {
            p_isp4580->evt_handler(p_isp4580,
                evt_type,
                (uint8_t *const)p_evt_rw_authorize_request->request.write.data,
                p_evt_rw_authorize_request->request.write.len);
        }
    }
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_isp4580     	isp4580 service structure.
 * @param[in] p_lisp4580_init  	Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t config_char_add(ble_isp4580_t *p_isp4580, const ble_isp4580_init_t *p_isp4580_init) {
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

    ble_uuid.type = p_isp4580->uuid_type;
    ble_uuid.uuid = BLE_UUID_ISP4580_CONFIG_CHAR;

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
    attr_char_value.init_len = sizeof(ble_isp4580_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_isp4580_init->p_init_config;
    attr_char_value.max_len = sizeof(ble_isp4580_config_t);

    return sd_ble_gatts_characteristic_add(p_isp4580->service_handle,
        &char_md,
        &attr_char_value,
        &p_isp4580->config_handles);
}

/**@brief Function for adding rx diagnostic characteristic.
 *
 * @param[in] p_isp4580     	isp4580 service structure.
 * @param[in] p_isp4580_init  	Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rxdiag_char_add(ble_isp4580_t *p_isp4580, const ble_isp4580_init_t *p_isp4580_init) {
    ble_add_char_params_t add_char_params;
    ble_uuid_t ble_uuid;

    ble_uuid.type = p_isp4580->uuid_type;
    ble_uuid.uuid = BLE_UUID_ISP4580_RXDIAG_CHAR;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid = ble_uuid.uuid;
    add_char_params.uuid_type = ble_uuid.type;
    add_char_params.max_len = sizeof(ble_isp4580_rxdiag_t);
    add_char_params.init_len = sizeof(ble_isp4580_rxdiag_t);
    add_char_params.p_init_value = (uint8_t *)p_isp4580_init->p_init_rxdiag;
    ;
    add_char_params.is_var_len = false;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_isp4580->service_handle, &add_char_params, &(p_isp4580->rxdiag_handles));
}

void ble_isp4580_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    if ((p_context == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    ble_isp4580_t *p_isp4580 = (ble_isp4580_t *)p_context;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_isp4580, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_isp4580, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_isp4580, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_authorize_req(p_isp4580, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

uint32_t ble_isp4580_init(ble_isp4580_t *p_isp4580, const ble_isp4580_init_t *p_isp4580_init) {
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t base_uuid = ISP4580_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_isp4580);
    VERIFY_PARAM_NOT_NULL(p_isp4580_init);

    // Initialize the service structure.
    p_isp4580->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_isp4580->evt_handler = p_isp4580_init->evt_handler;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_isp4580->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_isp4580->uuid_type;
    ble_uuid.uuid = BLE_UUID_ISP4580_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_isp4580->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Characteristics.
    err_code = config_char_add(p_isp4580, p_isp4580_init);
    VERIFY_SUCCESS(err_code);

    if (p_isp4580_init->p_init_rxdiag != NULL) {
        err_code = rxdiag_char_add(p_isp4580, p_isp4580_init);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_isp4580_rxdiag_set(ble_isp4580_t *p_isp4580, ble_isp4580_rxdiag_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_isp4580_rxdiag_t);

    VERIFY_PARAM_NOT_NULL(p_isp4580);

    if ((p_isp4580->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_isp4580->is_rxdiag_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_isp4580->rxdiag_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_isp4580->conn_handle, &hvx_params);
}