/******************************************************************************
 * @file    ble_env.h
 * @author  Insight SiP
 * @brief   ble environment module header
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

#ifndef BLE_ENV_H__
#define BLE_ENV_H__

#include "app_util_platform.h"
#include "ble.h"
#include "ble_srv_common.h"
#include <stdbool.h>
#include <stdint.h>

#define BLE_UUID_ENV_SERVICE 0x1200                         /**< The UUID of the Envronment Service. */
#define BLE_ENV_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Environment service module. */

#ifdef __GNUC__
#ifdef PACKED
#undef PACKED
#endif

#define PACKED(TYPE) TYPE __attribute__((packed))
#endif

/**@brief   Macro for defining a ble_env instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ENV_DEF(_name)            \
    static ble_env_t _name;           \
    NRF_SDH_BLE_OBSERVER(_name##_obs, \
        APP_BLE_OBSERVER_PRIO,        \
        ble_env_on_ble_evt, &_name)

typedef float ble_env_temperature_t;
typedef float ble_env_pressure_t;
typedef float ble_env_humidity_t;
typedef float ble_env_light_t;
typedef PACKED(struct
    {
        uint16_t temperature_interval_ms;
        uint16_t pressure_interval_ms;
        uint16_t humidity_interval_ms;
        uint16_t light_interval_ms;
    }) ble_env_config_t;

#define BLE_ENV_CONFIG_TEMPERATURE_INT_MIN 100
#define BLE_ENV_CONFIG_TEMPERATURE_INT_MAX 5000
#define BLE_ENV_CONFIG_PRESSURE_INT_MIN 50
#define BLE_ENV_CONFIG_PRESSURE_INT_MAX 5000
#define BLE_ENV_CONFIG_HUMIDITY_INT_MIN 100
#define BLE_ENV_CONFIG_HUMIDITY_INT_MAX 5000
#define BLE_ENV_CONFIG_LIGHT_INT_MIN 100
#define BLE_ENV_CONFIG_LIGHT_INT_MAX 5000

typedef enum {
    BLE_ENV_EVT_NOTIF_TEMPERATURE,
    BLE_ENV_EVT_NOTIF_PRESSURE,
    BLE_ENV_EVT_NOTIF_HUMIDITY,
    BLE_ENV_EVT_NOTIF_LIGHT,
    BLE_ENV_EVT_CONFIG_RECEIVED
} ble_env_evt_type_t;

/* Forward declaration of the ble_env_t type. */
typedef struct ble_env_s ble_env_t;

/**@brief Environment Service event handler type. */
typedef void (*ble_env_evt_handler_t)(ble_env_t *p_env,
    ble_env_evt_type_t evt_type,
    uint8_t *p_data,
    uint16_t length);

/**@brief H2O Environment Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_env_init function.
 */
typedef struct
{
    ble_env_temperature_t *p_init_temperature;
    ble_env_pressure_t *p_init_pressure;
    ble_env_humidity_t *p_init_humidity;
    ble_env_light_t *p_init_light;
    ble_env_config_t *p_init_config;
    ble_env_evt_handler_t evt_handler; /**< Event handler to be called for handling received data. */
} ble_env_init_t;

/**@brief Environment Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_env_s {
    uint8_t uuid_type;                            /**< UUID type for Environment Service Base UUID. */
    uint16_t service_handle;                      /**< Handle of Environment Service (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t temperature_handles; /**< Handles related to the temperature characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t pressure_handles;    /**< Handles related to the pressure characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t humidity_handles;    /**< Handles related to the humidity characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t light_handles;       /**< Handles related to the light characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t config_handles;      /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    uint16_t conn_handle;                         /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool is_temperature_notif_enabled;            /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_pressure_notif_enabled;               /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_humidity_notif_enabled;               /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool is_light_notif_enabled;                  /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    ble_env_evt_handler_t evt_handler;            /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Environment Service.
 *
 * @param[out] p_env      Environment Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_env_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_tes or p_tes_init is NULL.
 */
uint32_t ble_env_init(ble_env_t *p_env, const ble_env_init_t *p_env_init);

/**@brief Function for handling the Environment Service's BLE events.
 *
 * @details The Environment Service expects the application to call this function each time an
 * event is received from the S132 SoftDevice. This function processes the event if it
 * is relevant and calls the Environment Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt   	Event received from the S132 SoftDevice.
 * @param[in] p_context      Configuration Service structure.
 */
void ble_env_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for setting the temperature.
 *
 * @details This function sends the input temperature as an temperature characteristic notification to the
 *          peer.
 *
 * @param[in] p_env       Pointer to the Environment Service structure.
 * @param[in] p_data      Pointer to the temperature data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_env_temperature_set(ble_env_t *p_env, ble_env_temperature_t *p_data);

/**@brief Function for setting the pressure.
 *
 * @details This function sends the input pressure as an pressure characteristic notification to the peer.
 *
 * @param[in] p_env       Pointer to the Environment Service structure.
 * @param[in] p_data      Pointer to the pressure data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_env_pressure_set(ble_env_t *p_env, ble_env_pressure_t *p_data);

/**@brief Function for setting the humidity.
 *
 * @details This function sends the input humidity as an humidity characteristic notification to the peer.
 *
 * @param[in] p_env       Pointer to the Environment Service structure.
 * @param[in] p_data      Pointer to the humidity data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_env_humidity_set(ble_env_t *p_env, ble_env_humidity_t *p_data);

/**@brief Function for setting the light.
 *
 * @details This function sends the input light as an light characteristic notification to the peer.
 *
 * @param[in] p_env       Pointer to the Environment Service structure.
 * @param[in] p_data      Pointer to the light data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_env_light_set(ble_env_t *p_env, ble_env_light_t *p_data);

#endif // BLE_ENV_H__