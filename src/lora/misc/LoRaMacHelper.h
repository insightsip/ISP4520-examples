/******************************************************************************
 * @file LoRaMacHelper.h
 * @author  Insight SiP
 * @brief LoRaMacHelper declaration file.
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

#ifndef __LORAMACHELPER_H__
#define __LORAMACHELPER_H__

#include "stdint.h"
#include "Commissioning.h"
#include "board.h"
#include "LoRaMac.h"
#include "Region.h"


#define LORAWAN_CONFIRMED_MSG_ON 	0               /**< LoRaWAN confirmed messages */
#define LORAWAN_CERTIF_PORT 		224             /**< LoRaWAN certification port */
#define LORAWAN_CERTIF_DELAY 		5000             /**< LoRaWAN certification delay */
#define LORAWAN_APP_DATA_MAX_SIZE       242             /**< LoRaWAN User application data buffer size*/
#define LORAWAN_DEFAULT_DATARATE        DR_5            /**< LoRaWAN Default datarate*/
#define LORAWAN_DEFAULT_TX_POWER        TX_POWER_0      /**< LoRaWAN Default tx power*/

typedef struct lmh_param_s
{
    bool    adr_enable;                 /**< Activation state of adaptative Datarate */
    int8_t  tx_data_rate;               /**< Uplink datarate, if AdrEnable is off */
    bool    enable_public_network;	/**< Enable or disable a public network */
    int8_t tx_power;                    /**< Uplink power */
} lmh_param_t;



// Forward declaration of the DeviceClass_t type.
typedef DeviceClass_t lmh_device_class_t;

// Forward declaration of the ActivationType_t type.
typedef ActivationType_t lmh_activation_type_t;

// Forward declaration of the LoRaMacStatus_t type.
typedef LoRaMacStatus_t lmh_error_code_t;

// Forward declaration of the ChannelParams_t type.
typedef ChannelParams_t lmh_channel_param_t;

/**
 * @brief Confirmed/Unconfirmed type.
 */
typedef enum 
{
    LMH_UNCONFIRMED_MSG = 0, 
    LMH_CONFIRMED_MSG = !LMH_UNCONFIRMED_MSG
} lmh_confirm_t;

/**
 * @brief Types of LHM events.
 */
typedef enum
{
    LHM_EVT_RX_ACK,                 ///< Confirmed message is acknowledged by the server
    LHM_EVT_NWK_JOINED,             ///< The device has joined a network
    LHM_EVT_CLASS_CHANGED,          ///< The device class has changed
    LMH_EVT_BEACON_LOST,            ///< Beacon lost
    LMH_EVT_BEACON_RECEIVED,        ///< Beacon received
    LMH_EVT_BEACON_NOT_RECEIVED,    ///< Beacon not received
} lmh_evt_type_t;

/**@brief Application Data structure
 */
typedef struct
{ 
    uint8_t* buffer; 		/**< point to the tx data buffer */ 
    uint8_t buffsize; 		/**< tx data buffer size */
    uint8_t port; 		/**< Port on which the data is sent */ 
    uint8_t confirmed;          /**< =1 if frame must be confirmed */ 
    uint8_t nb_trials;          /**< number of tries (confirmed packets only) */ 
} lmh_app_tx_data_t;

/**@brief Application Data structure
 */
typedef struct
{ 
    uint8_t* buffer; 		/**< point to the LoRa rx data buffer */ 
    uint8_t buffsize; 		/**< rx data buffer size */
    uint8_t port; 		/**< Port on which the data is received */ 
    int16_t rssi;               /**< Signal strength */ 
    uint8_t snr;                /**< Signal to noise ratio */
} lmh_app_rx_data_t;

/**@brief LoRaMac Helper Callbacks
 */
typedef struct lmh_callback_s
{
/**@brief Get the current battery level
 * @retval value  	battery level ( 0: very low, 254: fully charged )
 */
    uint8_t (*BoardGetBatteryLevel) (void);

/**@brief Gets the board 64 bits unique ID 
 * @param[in] id	Pointer to an array that will contain the Unique ID
 */
    void (*BoardGetUniqueId) (uint8_t *id);

/**@brief Returns a pseudo random seed generated using the MCU Unique ID
 * @retval seed Generated pseudo random seed
 */
    uint32_t (*BoardGetRandomSeed) (void);
	
/**@brief Process Rx Data received from Lora network 
 * @param[in] AppData 	Rx structure
 */
    void (*lmh_RxData) (lmh_app_rx_data_t *appdata);

/**@brief lmh events that are notified to the upper layer
 */
    void (*lmh_evt_handler) (lmh_evt_type_t type, void* data);
  
} lmh_callback_t;

/**@brief LoRaWAN compliance tests support data
 */
typedef struct LoraMacHelper_ComplianceTest_s
{
    uint16_t downlink_counter;
    bool link_check;
    bool running;
    bool is_tx_confirmed;
    uint8_t state;
    uint8_t demod_margin;
    uint8_t nb_gateways;
    uint8_t app_port;
    uint8_t data_buffer_size;
    uint8_t data_buffer[242];
} lmh_compliance_test_t;

/**@brief LoWaWan stack Initialisation
 *
 * @param[in] callbacks	Pointer to structure containing the callback functions
 * @param[in] LoRaParam	Pointer to structure containing the parameters
 */
lmh_error_code_t lmh_init (lmh_callback_t *callbacks);

void lmh_process(void);

/**
 * @brief  Set device EUI
 * @param[in]  pointer to the provided dev_eui
 */
lmh_error_code_t lmh_device_eui_set(uint8_t *dev_eui);

/**
 * @brief  Get device EUI
 * @param[out]  pointer to dev_eui
 */
lmh_error_code_t lmh_device_eui_get(uint8_t *dev_eui);

/**
 * @brief  Set the App/Join server EUI
 * @param[in]  pointer to provided join_eui
 */
lmh_error_code_t lmh_join_eui_set(uint8_t *join_eui);

/**
 * @brief  Get the App/Join server EUI
 * @param[out]   pointer to join_eui
 */
lmh_error_code_t lmh_join_eui_get(uint8_t *join_eui);

/**
 * @brief  Set the Network root key
 * @param[out]   pointer to nwk_key
 */
lmh_error_code_t lmh_nwk_key_set(uint8_t *nwk_key);

/**
 * @brief  Get the Network root key
 * @param[out]   pointer to nwk_key
 */
lmh_error_code_t lmh_nwk_key_get(uint8_t *nwk_key);

/**
 * @brief  Set the Forwarding Network session integrity key
 * @param[out]   pointer to f_nwk_s_int_key
 */
lmh_error_code_t lmh_f_nwk_s_int_key_set(uint8_t *f_nwk_s_int_key);

/**
 * @brief  Get the Forwarding Network session integrity key
 * @param[out]   pointer to f_nwk_s_int_key
 */
void lmh_f_nwk_s_int_key_get(uint8_t *f_nwk_s_int_key);

/**
 * @brief  Set the Serving Network session integrity key
 * @param[out]   pointer to s_nwk_s_int_key
 */
lmh_error_code_t lmh_s_nwk_s_int_key_set(uint8_t *s_nwk_s_int_key);

/**
 * @brief  Get the Serving Network session integrity key
 * @param[out]   pointer to s_nwk_s_int_key
 */
void lmh_s_nwk_s_int_key_get(uint8_t *s_nwk_s_int_key);

/**
 * @brief  Set the Network session encryption key
 * @param[out]   pointer to nwk_s_enc_key
 */
lmh_error_code_t lmh_nwk_s_enc_key_set(uint8_t *nwk_s_enc_key);

/**
 * @brief  Get the Network session encryption key
 * @param[out]   pointer to nwk_s_enc_key
 */
void lmh_nwk_s_enc_key_get(uint8_t *nwk_s_enc_key);

/**
 * @brief  Set the Application root key (Used to derive Multicast keys on 1.0.x devices)
 * @param[in]  pointer to provided gen_app_key
 */
lmh_error_code_t lmh_gen_app_key_set(uint8_t *gen_app_key);

/**
 * @brief  Get the Application root key (Used to derive Multicast keys on 1.0.x devices)
 * @param[out]  pointer to gen_app_key
 */
void lmh_gen_app_key_get(uint8_t *gen_app_key);

/**
 * @brief  Set the Application root key
 * @param[in]  pointer to provided app_key
 */
lmh_error_code_t lmh_app_key_set(uint8_t *app_key);

/**
 * @brief  Get the Application root key
 * @param[out]  pointer to app_key
 */
void lmh_app_key_get(uint8_t *app_key);

/**
 * @brief  Set the Application session key
 * @param[in]  pointer to provided app_key
 */
lmh_error_code_t lmh_app_s_key_set(uint8_t *app_s_key);

/**
 * @brief  Get the Application session key
 * @param[out]  pointer to app_key
 */
void lmh_app_s_key_get(uint8_t *app_s_key);

/**
 * @brief  Set the device address
 * @param[in]  pointer to provided device_address
 */
void lmh_device_address_set(uint32_t device_address);

/**
 * @brief  Get the device address
 * @param[out]  pointer to device_address
 */
void lmh_device_address_get(uint32_t *device_address);

/**
 * @brief  Set the Network ID
 * @param[in]  pointer to the provided network_id
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_network_id_set(uint32_t network_id);

/**
 * @brief  Get the Network ID
 * @param[out]  pointer to the network_id
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_network_id_get(uint32_t *network_id);

/**
 * @brief  Set the adaptative data rate
 * @param[in]  pointer to the provided adrEnable
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_adaptative_datarate_set(uint8_t adr);

/**
 * @brief  Get the adaptative data rate
 * @param[out]  pointer to the adrEnable
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_adaptative_datarate_get(uint8_t *adr);

/**@brief Set Lora Class
 * @Note Only switch from class A to class B/C OR from  class B/C to class A is allowed
 * @param[in] new_class class to switch to
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_class_set(lmh_device_class_t new_class);

/**@brief get the current Lora Class
 * @param[in] DeviceClass_t Current class
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_class_get(lmh_device_class_t *current_class);

/**@brief Join a Lora Network
 * @param[in] activation mode (true=OTAA, false=ABP)
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_join(bool otaa_mode);

/**@brief Check if the network is joined
 * @param[out] type 0 id not joined, 1 if joined as ABP device, 2 if joined as OTAA
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_join_status_get(lmh_activation_type_t *type);

/**@brief Send data
 * @param[in] app_data Pointer to data structure to be sent
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_send(lmh_app_tx_data_t* app_data);

/**@brief Request device time
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_device_time_req(void);

/**@brief Request beacon
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_beacon_req(void);

/**@brief Request ping slot periodicity
 * @param[in] periodicity Is equal to 2^periodicity seconds.
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_ping_slot_req(uint8_t periodicity);

/**@brief Set tx power
 * @param[in] tx_power power (as defined in lorawan spec)
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_tx_power_set(uint8_t tx_power);

/**@brief Get the tx power
 * @param[out] pointer to tx_power
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_tx_power_get(uint8_t *tx_power);

/**@brief Set the data rate
 * @param[in] data_rate data rate
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_tx_datarate_set(uint8_t data_rate);

/**@brief Get the data rate
 * @param[out] pointer to data_rate
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_tx_datarate_get(uint8_t *data_rate);

/**@brief Set the join accept delay 1
 * @param[in] delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_joinAcceptDelay1_set(uint32_t delay);

/**@brief Get the join accept delay 1
 * @param[out] pointer to delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_joinAcceptDelay1_get(uint32_t *delay);

/**@brief Set the join accept delay 2
 * @param[in] delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_joinAcceptDelay2_set(uint32_t delay);

/**@brief Get the join accept delay 2
 * @param[out] pointer to delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_joinAcceptDelay2_get(uint32_t *delay);

/**@brief Set the public Network mode
 * @param[in] public_mode
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_publicNetwork_set(uint8_t public_mode);

/**@brief Get the public Network mode
 * @param[out] pointer to public_mode
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_publicNetwork_get(uint8_t *public_mode);

/**@brief Set the rx delay 1
 * @param[in] delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDelay1_set(uint32_t delay);

/**@brief Get the rx delay 1
 * @param[out] pointer to delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDelay1_get(uint32_t *delay);

/**@brief Set the rx delay 2
 * @param[in] delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDelay2_set(uint32_t delay);

/**@brief Get the rx delay 2
 * @param[out] pointer to delay
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDelay2_get(uint32_t *delay);

/**@brief Set the rx data rate 2
 * @param[in] dr
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDataRate2_set(uint8_t dr);

/**@brief Get the rx data rate 2
 * @param[out] pointer to dr
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxDataRate2_get(uint8_t *dr);

/**@brief Set the rx frequency 2
 * @param[in] freq
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxFrequency2_set(uint32_t freq);

/**@brief Get the rx frequency 2
 * @param[out] pointer to freq
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_rxFrequency2_get(uint32_t *freq);

/**@brief Set the duty cycle
 * @param[in] duty_cycle
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_duty_cycle_set(bool duty_cycle);

/**@brief Get the duty cycle
 * @param[out] pointer to duty_cycle
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_duty_cycle_get(bool *duty_cycle);

/**@brief Set a channel info
 * @param[in] id                id of the channel
 * @param[in] channel_param     Parameters of the channel, frequency, dr min and  dr max
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_channel_set(uint8_t id, lmh_channel_param_t channel_param);

/**@brief Get all channel info
 * @param[out] channel_param     Parameters of the channel, frequency, dr min and  dr max
 * @retval lmh_error_code_t
 */
lmh_error_code_t lmh_channels_get(lmh_channel_param_t *channel_param);

#endif
