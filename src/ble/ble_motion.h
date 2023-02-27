/******************************************************************************
 * @file    ble_motion.h
 * @author  Insight SiP
 * @brief   ble motion module header
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

#ifndef BLE_MOTION_H__
#define BLE_MOTION_H__

#include "app_util_platform.h"
#include "ble.h"
#include "ble_srv_common.h"
#include <stdbool.h>
#include <stdint.h>

#define BLE_UUID_MOTION_SERVICE 0x1300                         /**< The UUID of the Motion Service. */
#define BLE_MOTION_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Motion service module. */

#ifdef __GNUC__
#ifdef PACKED
#undef PACKED
#endif

#define PACKED(TYPE) TYPE __attribute__((packed))
#endif

/**@brief   Macro for defining a ble_motion instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_MOTION_DEF(_name)         \
    static ble_motion_t _name;        \
    NRF_SDH_BLE_OBSERVER(_name##_obs, \
        APP_BLE_OBSERVER_PRIO,        \
        ble_motion_on_ble_evt, &_name)

typedef uint8_t ble_motion_orientation_t;

typedef PACKED(struct
    {
        float w;
        float x;
        float y;
        float z;
    }) ble_motion_quat_t;

typedef PACKED(struct
    {
        float roll;
        float pitch;
        float yaw;
    }) ble_motion_euler_t;

typedef PACKED(struct
    {
        float matrix[9];
    }) ble_motion_rot_mat_t;

typedef float ble_motion_heading_t;

typedef PACKED(struct
    {
        float x;
        float y;
        float z;
    }) ble_motion_gravity_t;

typedef PACKED(struct
    {
        float x;
        float y;
        float z;
    }) ble_motion_angular_velocity_t;

typedef PACKED(struct
    {
        float x;
        float y;
        float z;
    }) ble_motion_magnetization_t;

typedef PACKED(struct
    {
        uint16_t interval_ms;
    }) ble_motion_config_t;

typedef PACKED(struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t is_mag_calibrated;
    }) ble_motion_calib_t;

#define BLE_MOTION_CONFIG_INT_MIN 50
#define BLE_MOTION_CONFIG_INT_MAX 5000
#define BLE_MOTION_CALIB_NOT_DONE 0
#define BLE_MOTION_CALIB_DONE 1

typedef enum {
    BLE_MOTION_EVT_CONFIG_RECEIVED,
    BLE_MOTION_EVT_NOTIF_GRAVITY,
    BLE_MOTION_EVT_NOTIF_ANGULAR_ROTATION,
    BLE_MOTION_EVT_NOTIF_MAGNETIZATION,
    BLE_MOTION_EVT_NOTIF_QUAT,
    BLE_MOTION_EVT_NOTIF_EULER,
    BLE_MOTION_EVT_NOTIF_ROT_MAT,
    BLE_MOTION_EVT_NOTIF_HEADING,
    BLE_MOTION_EVT_NOTIF_CALIB,
    BLE_MOTION_EVT_CALIB_RECEIVED
} ble_motion_evt_type_t;

/* Forward declaration of the ble_motion_t type. */
typedef struct ble_motion_s ble_motion_t;

/**@brief Motion Service event handler type. */
typedef void (*ble_motion_evt_handler_t)(ble_motion_t *p_motion,
    ble_motion_evt_type_t evt_type,
    uint8_t *p_data,
    uint16_t length);

/**@brief Motion Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_motion_init function.
 */
typedef struct
{
    ble_motion_config_t *p_init_config;                     /**< configuration characteristic default value. */
    ble_motion_gravity_t *p_init_gravity;                   /**< gravity characteristic default value. */
    ble_motion_magnetization_t *p_init_magnetization;       /**< magnetization characteristic default value. */
    ble_motion_angular_velocity_t *p_init_angular_velocity; /**< angular velocity characteristic default value. */
    ble_motion_gravity_t *p_init_quat;                      /**< quaternion characteristic default value. */
    ble_motion_gravity_t *p_init_euler;                     /**< euler characteristic default value. */
    ble_motion_gravity_t *p_init_rot_mat;                   /**< rotation matric characteristic default value. */
    ble_motion_gravity_t *p_init_heading;                   /**< heading characteristic default value. */
    ble_motion_calib_t *p_init_calib;                       /**< calibration characteristic default value. */
    ble_motion_evt_handler_t evt_handler;                   /**< Event handler to be called for handling received data. */
} ble_motion_init_t;

/**@brief Motion Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_motion_s {
    uint8_t uuid_type;                                 /**< UUID type for Motion Service Base UUID. */
    uint16_t service_handle;                           /**< Handle of Motion Service (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t config_handles;           /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t gravity_handles;          /**< Handles related to the gravity vector characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t angular_rotation_handles; /**< Handles related to the angular rotation vector characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t magnetization_handles;    /**< Handles related to the  magnetization vector characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t quat_handles;             /**< Handles related to the quaternion characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t euler_handles;            /**< Handles related to the euler angles characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t rot_mat_handles;          /**< Handles related to the rotation matrix characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t heading_handles;          /**< Handles related to the compass heading characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t calib_handles;            /**< Handles related to the compass calib characteristic (as provided by the S132 SoftDevice). */
    uint16_t conn_handle;                              /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool is_gravity_notif_enabled;                     /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_angular_velocity_notif_enabled;            /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_magnetization_notif_enabled;               /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_quat_notif_enabled;                        /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_euler_notif_enabled;                       /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_rot_mat_notif_enabled;                     /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_heading_notif_enabled;                     /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_calibration_notif_enabled;                 /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    ble_motion_evt_handler_t evt_handler;              /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Motion Service.
 *
 * @param[out] p_motion     Motion Service structure. This structure must be supplied
 *                        	by the application. It is initialized by this function and will
 *                        	later be used to identify this particular service instance.
 * @param[in] p_motion_init Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_wss or p_tms_init is NULL.
 */
uint32_t ble_motion_init(ble_motion_t *p_motion, const ble_motion_init_t *p_motion_init);

/**@brief Function for handling the Motion Service's BLE events.
 *
 * @details The Motion Service expects the application to call this function each time an
 * event is received from the S132 SoftDevice. This function processes the event if it
 * is relevant and calls the Motion Service event handler of the
 * application if necessary.
 *
 * @param[in] p_motion    Motion Service structure.
 * @param[in] p_ble_evt   Event received from the S132 SoftDevice.
 */
void ble_motion_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for sending quaternion data.
 *
 * @details This function sends the input quaternion as an quaternion characteristic notification to the peer.
 *
 * @param[in] p_ble_evt   	Event received from the S132 SoftDevice.
 * @param[in] p_context      Configuration Service structure.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_quat_set(ble_motion_t *p_tmotion, ble_motion_quat_t *p_data);

/**@brief Function for sending euler angle data.
 *
 * @details This function sends the input pitch, roll and yaw as an euler characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_euler_set(ble_motion_t *p_motion, ble_motion_euler_t *p_data);

/**@brief Function for sending rotation matrix data.
 *
 * @details This function sends the input as a rotation matrix characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_rot_mat_set(ble_motion_t *p_motion, ble_motion_rot_mat_t *p_data);

/**@brief Function for sending compass heading data.
 *
 * @details This function sends the input as a compass heading characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_heading_set(ble_motion_t *p_motion, ble_motion_heading_t *p_data);

/**@brief Function for sending gravity vector data.
 *
 * @details This function sends the input as a gravity vector characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_gravity_set(ble_motion_t *p_motion, ble_motion_gravity_t *p_data);

/**@brief Function for sending angular rotation vector data.
 *
 * @details This function sends the input as an angular rotation vector characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_angular_velocity_set(ble_motion_t *p_motion, ble_motion_angular_velocity_t *p_data);

/**@brief Function for sending magnetization vector data.
 *
 * @details This function sends the input as a magnetization vector characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_magnetization_set(ble_motion_t *p_motion, ble_motion_magnetization_t *p_data);

/**@brief Function for sending calibration data.
 *
 * @details This function sends the input as a calibration characteristic notification to the peer.
 *
 * @param[in] p_motion    Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_motion_calibration_set(ble_motion_t *p_motion, ble_motion_calib_t *p_data);

#endif // BLE_MOTION_H__