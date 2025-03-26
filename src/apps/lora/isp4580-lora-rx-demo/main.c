/******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   RX side ISP4580 demo
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

// Standards
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// nRF
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_serial.h"

// BLE
#include "ble.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_dis.h"
#include "ble_isp4580.h"
#include "ble_env.h"
#include "ble_motion.h"

// LoRa
#include "board.h"
#include "radio.h"

// logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define FW_VERSION "3.1.8.1"
#define HW_REVISION "C"
#define FIRMWARE_VERSION 0x03010800                               // 3.1.8.0
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                       /**< Maximum number of events in the scheduler queue. */
#define APP_TX_DUTYCYCLE 10000                                    /**< Data transmission duty cycle in ms. */
#define APP_TX_DUTYCYCLE_RND 1000                                 /**< Random delay in ms for application data transmission duty cycle. */
#define LORA_CODINGRATE 1                                         // [1: 4/5,  2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH 8                                    // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0                                     // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define APP_BLE_CONN_CFG_TAG 1                               /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3                             /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#if ISP4520_US
#define DEVICE_NAME "ISP4520-US-GW"  
#elif ISP4520_EU
#define DEVICE_NAME "ISP4520-EU-GW"
#endif
                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME "Insight SiP"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL 300                                 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)  /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY 0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define PIN_UART_RX 8
#define PIN_UART_TX 6
#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32
#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
    PIN_UART_RX, PIN_UART_TX,
    NRF_UART_PSEL_DISCONNECTED, NRF_UART_PSEL_DISCONNECTED,
    NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
    NRF_UART_BAUDRATE_115200,
    UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_UART_DEF(serial_uart, 0);
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ, &serial_queues, &serial_buffs, NULL, NULL);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
BLE_ADVERTISING_DEF(m_advertising);                      /**< Advertising module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                  /**< Context for the Queued Write module.*/
BLE_ISP4580_DEF(m_isp4580);                              /**< ISP4580 BLE Service instance. */
BLE_MOTION_DEF(m_motion);    
BLE_ENV_DEF(m_env);      
static RadioEvents_t LoRaEvents;                         /**< LoRa events */
static ble_isp4580_config_t m_ble_isp4580_config = {     /**< LoRa configuration pointer. */
    .data_rate = 5,
    .tx_power = 0,
    .tx_request_interval = APP_TX_DUTYCYCLE / 1000,
    .enable_adr = 0};

/**@brief Function to be executed on Radio Tx Done event
 */
void lora_tx_done_evt_handler(void) {
    Radio.Sleep();
}

/**@brief Function to be executed on Radio Rx Done event
 */
void lora_rx_done_evt_handler(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    uint16_t pressure, humidity, light;
    int16_t temperature;
    int16_t acc[3];
    int16_t mag[3];
    int16_t gyro[3];
    float temperature_f32, humidity_f32, pressure_f32, light_f32;
    float acc_f32[3], mag_f32[3], gyro_f32[3];
    uint8_t cpt;
    ble_isp4580_rxdiag_t rxdiag;
    char tx_message[100];

    Radio.Sleep();

    if (size == 27) {
        // Parse LoRa frame
        pressure_f32    = (float)((uint16_t)(payload[1] << 8) | payload[2]) /  10;
        temperature_f32 = (float)( (int16_t)(payload[3] << 8) | payload[4]) / 100;
        humidity_f32    = (float)((uint16_t)(payload[5] << 8) | payload[6]) / 100;
        light_f32       = (float)((uint16_t)(payload[7] << 8) | payload[8]) / 100;
        acc_f32[0]      = (float)( (int16_t)(payload[9] << 8) | payload[10]) / 100;
        acc_f32[1]      = (float)( (int16_t)(payload[11] << 8) | payload[12]) / 100;
        acc_f32[2]      = (float)( (int16_t)(payload[13] << 8) | payload[14]) / 100;
        mag_f32[0]      = (float)( (int16_t)(payload[15] << 8) | payload[16]) / 100;
        mag_f32[1]      = (float)( (int16_t)(payload[17] << 8) | payload[18]) / 100;
        mag_f32[2]      = (float)( (int16_t)(payload[19] << 8) | payload[20]) / 100;

        rxdiag.frame_number =  payload[0];
        rxdiag.rssi = rssi;
        rxdiag.snr = snr;

        // Update BLE service
        ble_isp4580_rxdiag_set(&m_isp4580, &rxdiag);
        ble_env_temperature_set(&m_env, &temperature_f32);
        ble_env_pressure_set(&m_env, &pressure_f32);
        ble_env_humidity_set(&m_env, &humidity_f32);
        ble_env_light_set(&m_env, &light_f32);
        ble_motion_gravity_set(&m_motion, (ble_motion_gravity_t*)&acc_f32);
        ble_motion_angular_velocity_set(&m_motion, (ble_motion_angular_velocity_t*)&gyro_f32);
        ble_motion_magnetization_set(&m_motion, (ble_motion_magnetization_t*)&mag_f32);


        // Send by UART
        sprintf(tx_message, 
            "Frame %d, rssi= %d dBm, pressure= %.1f hPa, temp= %.1f degC, humidity= %.1f %%, light= %.1f lux, acc= %.1f %.1f %.1f g, mag=%.1f %.1f %.1f uT\r\n",                              \
            rxdiag.frame_number, rssi,    \
            pressure_f32, temperature_f32, humidity_f32, light_f32,                                 \
            acc_f32[0], acc_f32[1], acc_f32[2], mag_f32[0], mag_f32[1], mag_f32[2]);

        nrf_serial_write(&serial_uart, tx_message, strlen(tx_message), NULL, NRF_SERIAL_MAX_TIMEOUT);
    }
    Radio.Rx(0);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void lora_tx_timeout_evt_handler(void) {
    Radio.Sleep();
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void lora_rx_timeout_evt_handler(void) {
    Radio.Sleep();
}

/**@brief Function to be executed on Radio Rx Error event
 */
void lora_rx_error_evt_handler(void) {
    Radio.Sleep();
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static uint32_t isp4580_config_apply(ble_isp4580_config_t *p_config) {
    ret_code_t err_code;
    uint32_t bw;
    uint32_t sf;
    int8_t power;

    m_ble_isp4580_config = *p_config;

    // set new tx power
    power = 14 - 2 * p_config->tx_power;
    NRF_LOG_DEBUG("tx power: %d", p_config->tx_power);

    // set new dr
    switch (p_config->data_rate) {
    case 0:
        bw = 0;
        sf = 12;
        break;
    case 1:
        bw = 0;
        sf = 11;
        break;
    case 2:
        bw = 0;
        sf = 10;
        break;
    case 3:
        bw = 0;
        sf = 9;
        break;
    case 4:
        bw = 0;
        sf = 8;
        break;
    case 5:
        bw = 0;
        sf = 7;
        break;
    case 6:
        bw = 1;
        sf = 7;
        break;
    default:
        bw = 0;
        sf = 12;
        break;
    }
    NRF_LOG_DEBUG("data rate: %d", p_config->data_rate);

    Radio.SetRxConfig(MODEM_LORA, bw, sf,
        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    return NRF_SUCCESS;
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling event from the isp4580 service.
 *
 * @details This function will process the data received from the isp4580 BLE Service.
 *
 * @param[in] p_isp4580   	isp4580 service structure.
 * @param[in] evt_type 		isp4580 service event type.
 * @param[in] p_data   		Event data.
 * @param[in] length   		Length of the data.
 */
static void ble_isp4580_evt_handler(ble_isp4580_t *p_isp4580,
    ble_isp4580_evt_type_t evt_type,
    uint8_t *p_data,
    uint16_t length) {
    switch (evt_type) {
    case BLE_ISP4580_EVT_CONFIG_RECEIVED: {
        NRF_LOG_INFO("BLE_ISP4580_EVT_CONFIG_RECEIVED: %d", length);
        APP_ERROR_CHECK_BOOL(length == sizeof(ble_isp4580_config_t));

        isp4580_config_apply((ble_isp4580_config_t *)p_data);
        break;
    }

    default:
        break;
    }
}

/**@brief Function for handling event from the environment service.
 *
 * @details This function will process the data received from the environment BLE Service.
 *
 * @param[in] p_loraconfig  	environment service structure.
 * @param[in] evt_type 		environment service event type.
 * @param[in] p_data   		Event data.
 * @param[in] length   		Length of the data.
 */
static void ble_env_evt_handler(ble_env_t *p_env, ble_env_evt_type_t evt_type, uint8_t *p_data, uint16_t length)
{
    //Nothing to do here
}

/**@brief Function for handling event from the motion service.
 *
 * @details This function will process the data received from the motion BLE Service.
 *
 * @param[in] p_loraconfig  	motion service structure.
 * @param[in] evt_type 		motion service event type.
 * @param[in] p_data   		Event data.
 * @param[in] length   		Length of the data.
 */
static void ble_motion_evt_handler(ble_motion_t *p_motion, ble_motion_evt_type_t evt_type, uint8_t *p_data, uint16_t length)
{
    //Nothing to do here
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Device Information services.
 */
static void services_init(void) {
    ret_code_t err_code;
    ble_dis_init_t dis_init;
    ble_env_init_t          env_init;
    ble_motion_init_t       motion_init;
    ble_isp4580_init_t isp4580_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_isp4580_rxdiag_t   rxdiag_init = {0};
    ble_env_temperature_t   temperature_init = 0.0;
    ble_env_pressure_t      pressure_init = 0.0;
    ble_env_humidity_t      humidity_init = 0.0;
    ble_env_light_t         light_init = 0.0;
    ble_motion_gravity_t            gravityt_init = {0.0};
    ble_motion_magnetization_t      magnetization_init = {0.0};
    ble_motion_angular_velocity_t   angular_velocity_init = {0.0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    memset(&env_init, 0, sizeof(ble_env_init_t));
    env_init.p_init_temperature = &temperature_init;
    env_init.p_init_pressure    = &pressure_init;
    env_init.p_init_humidity    = &humidity_init;
    env_init.p_init_light       = &light_init;
    env_init.evt_handler        = ble_env_evt_handler;
    err_code = ble_env_init(&m_env, &env_init);
    APP_ERROR_CHECK(err_code);

    memset(&motion_init, 0, sizeof(ble_motion_init_t));
    motion_init.p_init_gravity          = &gravityt_init;
    motion_init.p_init_magnetization    = &magnetization_init;
    motion_init.p_init_angular_velocity = &angular_velocity_init;
    motion_init.evt_handler             = ble_motion_evt_handler; 
    err_code = ble_motion_init(&m_motion, &motion_init);
    APP_ERROR_CHECK(err_code);

    // Initialize isp4580 service
    memset(&isp4580_init, 0, sizeof(ble_isp4580_init_t));
    isp4580_init.p_init_config = &m_ble_isp4580_config;
    isp4580_init.p_init_rxdiag     = &rxdiag_init;
    isp4580_init.evt_handler = ble_isp4580_evt_handler;
    err_code = ble_isp4580_init(&m_isp4580, &isp4580_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HW_REVISION);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
    ret_code_t err_code;
    ble_advertising_init_t init;
    ble_uuid_t adv_uuid = {BLE_UUID_ISP4580_SERVICE, m_isp4580.uuid_type};

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(adv_uuid) / sizeof(adv_uuid);
    init.srdata.uuids_complete.p_uuids = &adv_uuid;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = 0;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
    ret_code_t err_code;

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for application main entry.
 */
int main(void) {
    uint32_t err_code;
    uint32_t rf_freq;

    // Initialize logs.
    log_init();
    NRF_LOG_INFO("ISP4580 LoRa demo RX started.");

    // Initialize power
    nrf_pwr_mgmt_init();

    // Initialize Scheduler and timer
    timers_init();

     //Initialize Serial.
    err_code = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("nrf_serial_init failed - %d", err_code);
        return err_code;
    }

    // Initialize BLE
    ble_stack_init();
    gap_params_init();
    conn_params_init();
    services_init();
    advertising_init();

    // Initialize LoRa
#if ISP4520_US
    rf_freq = 915000000;
#elif ISP4520_EU
    rf_freq = 868000000;
#endif

    lora_hardware_init();
    LoRaEvents.TxDone = lora_tx_done_evt_handler;
    LoRaEvents.RxDone = lora_rx_done_evt_handler;
    LoRaEvents.TxTimeout = lora_tx_timeout_evt_handler;
    LoRaEvents.RxTimeout = lora_rx_timeout_evt_handler;
    LoRaEvents.RxError = lora_rx_error_evt_handler;
    Radio.Init(&LoRaEvents);
    Radio.SetChannel(rf_freq);
    Radio.SetRxConfig(MODEM_LORA, 0, 7,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    isp4580_config_apply(&m_ble_isp4580_config);

    // Start Lora
    Radio.Rx(0);

    // Start BLE
    ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    // Enter main loop.
    for (;;) {
        // Process the LoRaMac events
        Radio.IrqProcess();
        // Process events managed by the scheduler
        app_sched_execute();

        if (NRF_LOG_PROCESS() == false) {
            nrf_pwr_mgmt_run();
        }
    }
}