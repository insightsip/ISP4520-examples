/******************************************************************************
 * @file    ble_isp4580.h
 * @author  Insight SiP
 * @brief   LoRa mngt ble service header file.
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

#include "app_util_platform.h"
#include "ble_srv_common.h"

#ifndef BLE_ISP4580_H__
#define BLE_ISP4580_H__

#ifdef __GNUC__
#ifdef PACKED
#undef PACKED
#endif

#define PACKED(TYPE) TYPE __attribute__((packed))
#endif

/**@brief   Macro for defining a ble_isp4580 instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ISP4580_DEF(_name)        \
    static ble_isp4580_t _name;       \
    NRF_SDH_BLE_OBSERVER(_name##_obs, \
        APP_BLE_OBSERVER_PRIO,        \
        ble_isp4580_on_ble_evt, &_name)

#define BLE_UUID_ISP4580_SERVICE 0x1000 /**< The UUID of the isp4580 Service. */

typedef PACKED(struct
    {
        uint8_t data_rate;
        uint8_t tx_power;
        uint16_t tx_request_interval;
        uint8_t enable_adr;
    }) ble_isp4580_config_t;

typedef PACKED(struct
    {
        uint8_t frame_number;
        int16_t rssi;
        int8_t snr;
    }) ble_isp4580_rxdiag_t;

#define ISP4580_DATA_RATE_MAX 6
#define ISP4580_TX_POWER_MAX 7
#define ISP4580_TX_INTERVAL_MIN 5
#define ISP4580_TX_INTERVAL_MAX 3600
#define ISP4580_ADR_MAX 1

typedef enum {
    BLE_ISP4580_EVT_NOTIF_RXDIAG,
    BLE_ISP4580_EVT_CONFIG_RECEIVED
} ble_isp4580_evt_type_t;

/* Forward declaration of the ble_isp4580_t type. */
typedef struct ble_isp4580_s ble_isp4580_t;

/**@brief isp4580 service event handler type. */
typedef void (*ble_isp4580_evt_handler_t)(ble_isp4580_t *p_isp4580,
    ble_isp4580_evt_type_t evt_type,
    uint8_t *p_data,
    uint16_t length);

/**@brief isp4580 configuration Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_isp4580_init function.
 */
typedef struct
{
    ble_isp4580_config_t *p_init_config;
    ble_isp4580_rxdiag_t *p_init_rxdiag;
    ble_isp4580_evt_handler_t evt_handler; /**< Event handler to be called for handling received data. */
} ble_isp4580_init_t;

/**@brief isp4580 service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_isp4580_s {
    uint8_t uuid_type;                       /**< UUID type for isp4580 Service Base UUID. */
    uint16_t service_handle;                 /**< Handle of isp4580 Service (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t config_handles; /**< Handles related to the configuration characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t rxdiag_handles; /**< Handles related to the rx diagnotics characteristic (as provided by the S132 SoftDevice). */
    bool is_rxdiag_notif_enabled;            /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    uint16_t conn_handle;                    /**< Handle of the current connection (as provided by the S132 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_isp4580_evt_handler_t evt_handler;   /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the isp4580 service.
 *
 * @param[out] p_isp4580 	isp4580 service structure. This structure must be supplied
 *                        	by the application. It is initialized by this function and will
 *                        	later be used to identify this particular service instance.
 * @param[in] p_isp4580_init  	Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_tes or p_tes_init is NULL.
 */
uint32_t ble_isp4580_init(ble_isp4580_t *p_isp4580, const ble_isp4580_init_t *p_isp4580_init);

/**@brief Function for handling the isp4580 Service's BLE events.
 *
 * @details The isp4580 Service expects the application to call this function each time an
 * event is received from the S132 SoftDevice. This function processes the event if it
 * is relevant and calls the isp4580 Service event handler of the
 * application if necessary.
 *
 * @param[in] p_isp4580     	isp4580 service structure.
 * @param[in] p_ble_evt   	Event received from the S132 SoftDevice.
 */
void ble_isp4580_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for setting the rxdiag.
 *
 * @details This function sends the input rxdiag as an rxdiag characteristic notification to the peer.
 *
 * @param[in] p_isp4580  Pointer to the isp4580 Service structure.
 * @param[in] p_data      Pointer to the rxdiag data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_isp4580_rxdiag_set(ble_isp4580_t *p_isp4580, ble_isp4580_rxdiag_t *p_data);

#endif