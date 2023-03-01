/******************************************************************************
 * @file    ble_motion.c
 * @author  Insight SiP
 * @brief   ble motion module implementation
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

#include "ble_motion.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_MOTION_CONFIG_CHAR 0x1301           /**< The UUID of the config Characteristic. */
#define BLE_UUID_MOTION_GRAVITY_CHAR 0x1302          /**< The UUID of the gravity vector Characteristic. */
#define BLE_UUID_MOTION_ANGULAR_ROTATION_CHAR 0x1303 /**< The UUID of the raw data Characteristic. */
#define BLE_UUID_MOTION_MAGNETIZATION_CHAR 0x1304    /**< The UUID of the raw data Characteristic. */
#define BLE_UUID_MOTION_QUATERNION_CHAR 0x1305       /**< The UUID of the quaternion Characteristic. */
#define BLE_UUID_MOTION_EULER_CHAR 0x1306            /**< The UUID of the euler Characteristic. */
#define BLE_UUID_MOTION_ROT_MAT_CHAR 0x1307          /**< The UUID of the rotation matrix Characteristic. */
#define BLE_UUID_MOTION_HEADING_CHAR 0x1308          /**< The UUID of the heading Characteristic. */
#define BLE_UUID_MOTION_CALIB_CHAR 0x1309            /**< The UUID of the calib Characteristic. */

// b8c7xxxx-dd70-4c5a-b872-184eac50d00b
#define BASE_UUID                                                                                          \
    {                                                                                                      \
        { 0x0B, 0xD0, 0x50, 0xAC, 0x4E, 0x18, 0x72, 0xB8, 0x5A, 0x4C, 0x70, 0xDD, 0x00, 0x00, 0xC7, 0xB8 } \
    } /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_motion     	Motion Service structure.
 * @param[in] p_ble_evt 	Pointer to the event received from BLE stack.
 */
static void on_connect(ble_motion_t *p_motion, ble_evt_t *p_ble_evt) {
    p_motion->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_motion     	Motion Service structure.
 * @param[in] p_ble_evt 	Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_motion_t *p_motion, ble_evt_t *p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_motion->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_motion     	Motion Service structure.
 * @param[in] p_ble_evt 	Pointer to the event received from BLE stack.
 */
static void on_write(ble_motion_t *p_motion, ble_evt_t *p_ble_evt) {
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_motion->gravity_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_gravity_notif_enabled != notif_enabled) {
            p_motion->is_gravity_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_GRAVITY, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->angular_rotation_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_angular_velocity_notif_enabled != notif_enabled) {
            p_motion->is_angular_velocity_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_ANGULAR_ROTATION, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->magnetization_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_magnetization_notif_enabled != notif_enabled) {
            p_motion->is_magnetization_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_MAGNETIZATION, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->quat_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_quat_notif_enabled != notif_enabled) {
            p_motion->is_quat_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_QUAT, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->euler_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_euler_notif_enabled != notif_enabled) {
            p_motion->is_euler_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_EULER, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->rot_mat_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_rot_mat_notif_enabled != notif_enabled) {
            p_motion->is_rot_mat_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_ROT_MAT, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->heading_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_heading_notif_enabled != notif_enabled) {
            p_motion->is_heading_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_HEADING, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->gravity_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_gravity_notif_enabled != notif_enabled) {
            p_motion->is_gravity_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_GRAVITY, p_evt_write->data, p_evt_write->len);
            }
        }
    } else if ((p_evt_write->handle == p_motion->calib_handles.cccd_handle) && (p_evt_write->len == 2)) {
        bool notif_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

        if (p_motion->is_calibration_notif_enabled != notif_enabled) {
            p_motion->is_calibration_notif_enabled = notif_enabled;

            if (p_motion->evt_handler != NULL) {
                p_motion->evt_handler(p_motion, BLE_MOTION_EVT_NOTIF_CALIB, p_evt_write->data, p_evt_write->len);
            }
        }
    } else {
        // Do Nothing. This event is not relevant for this service.
    }
}

static void on_authorize_req(ble_motion_t *p_motion, ble_evt_t *p_ble_evt) {
    ble_gatts_evt_rw_authorize_request_t *p_evt_rw_authorize_request = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    uint32_t err_code;

    if (p_evt_rw_authorize_request->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
        if (p_evt_rw_authorize_request->request.write.handle == p_motion->config_handles.value_handle) {
            ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
            bool valid_data = true;

            // Check for valid data
            if (p_evt_rw_authorize_request->request.write.len != sizeof(ble_motion_config_t)) {
                valid_data = false;
            } else {
                ble_motion_config_t *p_config = (ble_motion_config_t *)p_evt_rw_authorize_request->request.write.data;

                if ((p_config->interval_ms < BLE_MOTION_CONFIG_INT_MIN) ||
                    (p_config->interval_ms > BLE_MOTION_CONFIG_INT_MAX)) {
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

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);

            if (valid_data && (p_motion->evt_handler != NULL)) {
                p_motion->evt_handler(p_motion,
                    BLE_MOTION_EVT_CONFIG_RECEIVED,
                    p_evt_rw_authorize_request->request.write.data,
                    p_evt_rw_authorize_request->request.write.len);
            }
        }

        else if (p_evt_rw_authorize_request->request.write.handle == p_motion->calib_handles.value_handle) {
            ble_gatts_rw_authorize_reply_params_t rw_authorize_reply;
            bool valid_data = true;

            // Check for valid data
            if (p_evt_rw_authorize_request->request.write.len != sizeof(ble_motion_calib_t)) {
                valid_data = false;
            } else {
                ble_motion_calib_t *p_calib = (ble_motion_calib_t *)p_evt_rw_authorize_request->request.write.data;

                if ((p_calib->is_mag_calibrated != BLE_MOTION_CALIB_NOT_DONE) ||
                    (p_calib->is_mag_calibrated != BLE_MOTION_CALIB_DONE)) {
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

            err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &rw_authorize_reply);
            APP_ERROR_CHECK(err_code);

            if (valid_data && (p_motion->evt_handler != NULL)) {
                p_motion->evt_handler(p_motion,
                    BLE_MOTION_EVT_CALIB_RECEIVED,
                    p_evt_rw_authorize_request->request.write.data,
                    p_evt_rw_authorize_request->request.write.len);
            }
        }
    }
}

/**@brief Function for adding quaternion characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t quat_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_quat_t quat_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_QUATERNION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_quat_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&quat_init;
    attr_char_value.max_len = sizeof(ble_motion_quat_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->quat_handles);
}

/**@brief Function for adding euler angle data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t euler_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_euler_t euler_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_EULER_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_euler_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&euler_init;
    attr_char_value.max_len = sizeof(ble_motion_euler_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->euler_handles);
}

/**@brief Function for adding rotation matrix data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rot_mat_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_rot_mat_t rot_mat_init;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_ROT_MAT_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_rot_mat_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&rot_mat_init;
    attr_char_value.max_len = sizeof(ble_motion_rot_mat_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->rot_mat_handles);
}

/**@brief Function for adding compass heading data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t heading_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_heading_t heading_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_HEADING_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_heading_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&heading_init;
    attr_char_value.max_len = sizeof(ble_motion_heading_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->heading_handles);
}

/**@brief Function for adding gravity data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t gravity_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_gravity_t gravity_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_GRAVITY_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_gravity_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&gravity_init;
    attr_char_value.max_len = sizeof(ble_motion_gravity_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->gravity_handles);
}

/**@brief Function for adding angular velocity data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t angular_velocity_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_angular_velocity_t angular_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_ANGULAR_ROTATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_angular_velocity_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&angular_init;
    attr_char_value.max_len = sizeof(ble_motion_angular_velocity_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->angular_rotation_handles);
}

/**@brief Function for adding magnetization data characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t magnetization_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_motion_magnetization_t magnetization_init = {0};

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_MAGNETIZATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_magnetization_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)&magnetization_init;
    attr_char_value.max_len = sizeof(ble_motion_magnetization_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->magnetization_handles);
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t config_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
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
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_CONFIG_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_motion_init->p_init_config;
    attr_char_value.max_len = sizeof(ble_motion_config_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->config_handles);
}

/**@brief Function for adding calibration characteristic.
 *
 * @param[in] p_motion       Motion Service structure.
 * @param[in] p_motion_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t calib_char_add(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.notify = 1;
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_CALIB_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 1;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(ble_motion_calib_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value = (uint8_t *)p_motion_init->p_init_calib;
    attr_char_value.max_len = sizeof(ble_motion_calib_t);

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
        &char_md,
        &attr_char_value,
        &p_motion->calib_handles);
}

void ble_motion_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
    if ((p_context == NULL) || (p_ble_evt == NULL)) {
        return;
    }

    ble_motion_t *p_motion = (ble_motion_t *)p_context;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_motion, (ble_evt_t *)p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_motion, (ble_evt_t *)p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_motion, (ble_evt_t *)p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_authorize_req(p_motion, (ble_evt_t *)p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

uint32_t ble_motion_init(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init) {
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t motion_base_uuid = BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_motion);
    VERIFY_PARAM_NOT_NULL(p_motion_init);

    // Initialize the service structure.
    p_motion->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_motion->evt_handler = p_motion_init->evt_handler;
    p_motion->is_gravity_notif_enabled = false;
    p_motion->is_angular_velocity_notif_enabled = false;
    p_motion->is_magnetization_notif_enabled = false;
    p_motion->is_quat_notif_enabled = false;
    p_motion->is_euler_notif_enabled = false;
    p_motion->is_rot_mat_notif_enabled = false;
    p_motion->is_heading_notif_enabled = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&motion_base_uuid, &p_motion->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_motion->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics
    if (p_motion_init->p_init_config != NULL) {
        err_code = config_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_gravity != NULL) {
        err_code = gravity_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_angular_velocity != NULL) {
        err_code = angular_velocity_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_magnetization != NULL) {
        err_code = magnetization_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_quat != NULL) {
        err_code = quat_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_euler != NULL) {
        err_code = euler_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_rot_mat != NULL) {
        err_code = rot_mat_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_heading != NULL) {
        err_code = heading_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }
    if (p_motion_init->p_init_calib != NULL) {
        err_code = calib_char_add(p_motion, p_motion_init);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t ble_motion_quat_set(ble_motion_t *p_motion, ble_motion_quat_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_quat_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_quat_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->quat_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_euler_set(ble_motion_t *p_motion, ble_motion_euler_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_euler_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_euler_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->euler_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_rot_mat_set(ble_motion_t *p_motion, ble_motion_rot_mat_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_rot_mat_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_rot_mat_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->rot_mat_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_heading_set(ble_motion_t *p_motion, ble_motion_heading_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_heading_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_heading_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->heading_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_gravity_set(ble_motion_t *p_motion, ble_motion_gravity_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_gravity_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_gravity_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->gravity_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_angular_velocity_set(ble_motion_t *p_motion, ble_motion_angular_velocity_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_angular_velocity_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_angular_velocity_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->angular_rotation_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_magnetization_set(ble_motion_t *p_motion, ble_motion_magnetization_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_magnetization_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_magnetization_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->magnetization_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}

uint32_t ble_motion_calibration_set(ble_motion_t *p_motion, ble_motion_calib_t *p_data) {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_motion_calib_t);

    VERIFY_PARAM_NOT_NULL(p_motion);

    if ((p_motion->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_motion->is_calibration_notif_enabled)) {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_motion->calib_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len = &length;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);
}