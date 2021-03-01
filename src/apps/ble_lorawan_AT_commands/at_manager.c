 /******************************************************************************
 * @file    at_manager.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-may-2019
 * @brief  at commands for LoRaWan
 *
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

#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "at_hal_transport.h"
#include "at_manager.h"
#include "at_hal_transport.h"
#include "LoRaMacHelper.h"
#include "board.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define LORAWAN_APP_DATA_BUFF_SIZE	64      ///< Size of the LoRa data to be allocate.

/**
 * @brief Macro for creating a new AT Command
 */
#define AT_COMMAND_DEF(_name, _set, _read, _test)   \
{                                                   \
    .name = _name,                                  \
    .name_size = sizeof(_name) - 1,                 \
    .set = _set,                                    \
    .read = _read,                                  \
    .test = _test,                                  \
}

#define CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error)        \
do                                                          \
{                                                           \
    if (lmh_error == LORAMAC_STATUS_OK)                     \
    {                                                       \
        at_error = AT_OK;                                   \
    }                                                       \
    else if (lmh_error == LORAMAC_STATUS_PARAMETER_INVALID) \
    {                                                       \
        at_error = AT_ERROR_PARAM;                          \
    }                                                       \
    else if (lmh_error == LORAMAC_STATUS_BUSY)              \
    {                                                       \
        at_error = AT_ERROR_BUSY;                           \
    }                                                       \
    else if (lmh_error == LORAMAC_STATUS_NO_NETWORK_JOINED) \
    {                                                       \
        at_error = AT_ERROR_NOT_JOINED;                     \
    }                                                       \
    else if (lmh_error == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED)  \
    {                                                       \
        at_error = AT_ERROR_DUTY_CYCLE;                     \
    }                                                       \
    else                                                    \
    {                                                       \
        at_error = AT_ERROR_OTHER;                          \
    }                                                       \
} while (0)

#define AT_VERIFY_SUCCESS(err_code)         \
do                                          \
{                                           \
    if (!(err_code == AT_OK))               \
    {                                       \
        return err_code;                    \
    }                                       \
} while (0)


// Foward declarations
static void lmh_rx_data_handler(lmh_app_rx_data_t *app_data);
static void lmh_evt_handler(lmh_evt_type_t type, void *data);

at_error_code_t at_error_not_supported (const uint8_t *param);
at_error_code_t at_reset (const uint8_t *param);
at_error_code_t at_deveui_set (const uint8_t *param);
at_error_code_t at_deveui_read (const uint8_t *param);
at_error_code_t at_deveui_test (const uint8_t *param);
at_error_code_t at_appeui_set (const uint8_t *param);
at_error_code_t at_appeui_read (const uint8_t *param);
at_error_code_t at_appeui_test (const uint8_t *param);
at_error_code_t at_joineui_set (const uint8_t *param);
at_error_code_t at_joineui_read (const uint8_t *param);
at_error_code_t at_joineui_test (const uint8_t *param);
at_error_code_t at_appkey_set (const uint8_t *param);
at_error_code_t at_appkey_read (const uint8_t *param);
at_error_code_t at_appkey_test (const uint8_t *param);
at_error_code_t at_genappkey_set (const uint8_t *param);
at_error_code_t at_genappkey_read (const uint8_t *param);
at_error_code_t at_genappkey_test (const uint8_t *param);
at_error_code_t at_nwkkey_set (const uint8_t *param);
at_error_code_t at_nwkkey_read (const uint8_t *param);
at_error_code_t at_nwkkey_test (const uint8_t *param);
at_error_code_t at_fnwksintkey_set (const uint8_t *param);
at_error_code_t at_fnwksintkey_read (const uint8_t *param);
at_error_code_t at_fnwksintkey_test (const uint8_t *param);
at_error_code_t at_snwksintkey_set (const uint8_t *param);
at_error_code_t at_snwksintkey_read (const uint8_t *param);
at_error_code_t at_snwksintkey_test (const uint8_t *param);
at_error_code_t at_nwksenckey_set (const uint8_t *param);
at_error_code_t at_nwksenckey_read (const uint8_t *param);
at_error_code_t at_nwksenckey_test (const uint8_t *param);
at_error_code_t at_appskey_set (const uint8_t *param);
at_error_code_t at_appskey_read (const uint8_t *param);
at_error_code_t at_appskey_test (const uint8_t *param);
at_error_code_t at_devaddr_set (const uint8_t *param);
at_error_code_t at_devaddr_read (const uint8_t *param);
at_error_code_t at_devaddr_test (const uint8_t *param);
at_error_code_t at_netid_set (const uint8_t *param);
at_error_code_t at_netid_read (const uint8_t *param);
at_error_code_t at_netid_test (const uint8_t *param);
at_error_code_t at_joinrq_set (const uint8_t *param);
at_error_code_t at_joinstat_read (const uint8_t *param);
at_error_code_t at_rcv_read (const uint8_t *param);
at_error_code_t at_send_set (const uint8_t *param);
at_error_code_t at_adr_set (const uint8_t *param);
at_error_code_t at_adr_read (const uint8_t *param);
at_error_code_t at_class_set (const uint8_t *param);
at_error_code_t at_class_read (const uint8_t *param);
at_error_code_t at_class_test (const uint8_t *param);
at_error_code_t at_dr_set (const uint8_t *param);
at_error_code_t at_dr_read (const uint8_t *param);
at_error_code_t at_dr_test (const uint8_t *param);
at_error_code_t at_joindly1_set (const uint8_t *param);
at_error_code_t at_joindly1_read (const uint8_t *param);
at_error_code_t at_joindly2_set (const uint8_t *param);
at_error_code_t at_joindly2_read (const uint8_t *param);
at_error_code_t at_pnet_set (const uint8_t *param);
at_error_code_t at_pnet_read (const uint8_t *param);
at_error_code_t at_rxdly1_set (const uint8_t *param);
at_error_code_t at_rxdly1_read (const uint8_t *param);
at_error_code_t at_rxdly2_set (const uint8_t *param);
at_error_code_t at_rxdly2_read (const uint8_t *param);
at_error_code_t at_rxdr2_set (const uint8_t *param);
at_error_code_t at_rxdr2_read (const uint8_t *param);
at_error_code_t at_rxdr2_test (const uint8_t *param);
at_error_code_t at_rxfq2_set (const uint8_t *param);
at_error_code_t at_rxfq2_read (const uint8_t *param);
at_error_code_t at_txp_set (const uint8_t *param);
at_error_code_t at_txp_read (const uint8_t *param);
at_error_code_t at_txp_test (const uint8_t *param);
at_error_code_t at_batt_read (const uint8_t *param);
at_error_code_t at_rssi_read (const uint8_t *param);
at_error_code_t at_snr_read (const uint8_t *param);
at_error_code_t at_hw_read (const uint8_t *param);
at_error_code_t at_fw_read (const uint8_t *param);
at_error_code_t at_dutyc_set (const uint8_t *param);
at_error_code_t at_dutyc_read (const uint8_t *param);
//at_error_code_t at_counter_read (const uint8_t *param);
at_error_code_t at_channel_set (const uint8_t *param);
at_error_code_t at_channel_read (const uint8_t *param);

/**
 * @brief  Structure defining an AT Command
 */
typedef struct  
{
    const uint8_t *name;                                /*< command name, after the "AT" */
    const uint8_t name_size;                            /*< size of the command name, not including the final \r or \n */
    at_error_code_t (*set)(const uint8_t *param);       /*< = after the string to set a new value, or \0 if not parameters*/
    at_error_code_t (*read)(const uint8_t *param);      /*< ? after the name to get the current value*/
    at_error_code_t (*test)(const uint8_t *param);      /*< =? test command */
} at_command_t;

/**@brief Structure containing lmh callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                            lmh_rx_data_handler, lmh_evt_handler};

static uint8_t m_lora_rx_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                    ///< Lora RX data buffer.
static lmh_app_rx_data_t m_lora_rx_data = {m_lora_rx_buffer, 0 ,0, 0, 0};       ///< Lora RX data structure.
static uint8_t m_lora_tx_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                    ///< Lora TX data buffer.
static lmh_app_tx_data_t m_lora_tx_data = {m_lora_tx_buffer, 0 ,0, 0};          ///< Lora TX data structure.
static bool m_at_command_ready = false;
static bool m_lora_ack_received = false;
static bool m_otaa = false;
uint8_t m_rx_at_command[128] = {0};

/**
 * @brief  Array corresponding to the description of each possible AT Error
 */
static const uint8_t *at_error_description[] =
{
    "OK\r\n",                    /* AT_OK */
    "UNKNOWN_CMD\r\n",           /* AT_UNKNOWN_CMD */
    "ERROR_NOT_SUPPORTED\r\n",    /* AT_ERROR_NOT_SUPPORTED */
    "ERROR_PARAM\r\n",           /* AT_ERROR_PARAM */
    "ERROR_BUSY\r\n",            /* AT_BUSY_ERROR_BUSY */
    "ERROR_NOT_JOINED\r\n",      /* AT_ERROR_NOT_JOINED */
    "ERROR_DUTY_CYCLE\r\n",      /* AT_ERROR_DUTY_CYCLE */
    "ERROR\r\n",                 /* AT_MAX */
};

/**
 * @brief  List of all supported AT Commands
 */
static at_command_t at_commands[] =
{
    AT_COMMAND_DEF (AT_RESET,       at_reset,               at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_DEVEUI,      at_deveui_set,          at_deveui_read,         at_deveui_test),
    AT_COMMAND_DEF (AT_APPEUI,      at_appeui_set,          at_appeui_read,         at_appeui_test),
    AT_COMMAND_DEF (AT_JOINEUI,     at_joineui_set,         at_joineui_read,        at_joineui_test),
    AT_COMMAND_DEF (AT_APPKEY,      at_appkey_set,          at_appkey_read,         at_appkey_test),
    AT_COMMAND_DEF (AT_GENAPPKEY,   at_genappkey_set,       at_genappkey_read,      at_genappkey_test),
    AT_COMMAND_DEF (AT_NWKKEY,      at_nwkkey_set,          at_nwkkey_read,         at_nwkkey_test),
    AT_COMMAND_DEF (AT_FNWKSINTKEY, at_fnwksintkey_set,     at_fnwksintkey_read,    at_fnwksintkey_test),
    AT_COMMAND_DEF (AT_SNWKSINTKEY, at_snwksintkey_set,     at_snwksintkey_read,    at_snwksintkey_test),
    AT_COMMAND_DEF (AT_NWKSENCKEY,  at_nwksenckey_set,      at_nwksenckey_read,     at_nwksenckey_test),
    AT_COMMAND_DEF (AT_APPSKEY,     at_appskey_set,         at_appskey_read,        at_appskey_test),
    AT_COMMAND_DEF (AT_DEVADDR,     at_devaddr_set,         at_devaddr_read,        at_devaddr_test),
    AT_COMMAND_DEF (AT_NETID,       at_netid_set,           at_netid_read,          at_netid_test),
    AT_COMMAND_DEF (AT_JOINRQ,      at_joinrq_set,          at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_JOINSTAT,    at_error_not_supported, at_joinstat_read,       at_error_not_supported),
    AT_COMMAND_DEF (AT_RCV,         at_error_not_supported, at_rcv_read,            at_error_not_supported),
    AT_COMMAND_DEF (AT_SEND,        at_send_set,            at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_ADR,         at_adr_set,             at_adr_read,            at_error_not_supported),
    AT_COMMAND_DEF (AT_CLASS,       at_class_set,           at_class_read,          at_class_test),
    AT_COMMAND_DEF (AT_DR,          at_dr_set,              at_dr_read,             at_dr_test),
    AT_COMMAND_DEF (AT_JOINDLY1,    at_joindly1_set,        at_joindly1_read,       at_error_not_supported),
    AT_COMMAND_DEF (AT_JOINDLY2,    at_joindly2_set,        at_joindly2_read,       at_error_not_supported),
    AT_COMMAND_DEF (AT_PNET,        at_pnet_set,            at_pnet_read,           at_error_not_supported),
    AT_COMMAND_DEF (AT_RXDLY1,      at_rxdly1_set,          at_rxdly1_read,         at_error_not_supported),
    AT_COMMAND_DEF (AT_RXDLY2,      at_rxdly2_set,          at_rxdly2_read,         at_error_not_supported),
    AT_COMMAND_DEF (AT_RXDR2,       at_rxdr2_set,           at_rxdr2_read,          at_rxdr2_test),
    AT_COMMAND_DEF (AT_RXFQ2,       at_rxfq2_set,           at_rxfq2_read,          at_error_not_supported),
    AT_COMMAND_DEF (AT_TXP,         at_txp_set,             at_txp_read,            at_txp_test),
    AT_COMMAND_DEF (AT_BATT,        at_error_not_supported, at_batt_read,           at_error_not_supported),
    AT_COMMAND_DEF (AT_RSSI,        at_error_not_supported, at_rssi_read,           at_error_not_supported),
    AT_COMMAND_DEF (AT_SNR,         at_error_not_supported, at_snr_read,            at_error_not_supported),
    AT_COMMAND_DEF (AT_FW,          at_error_not_supported, at_fw_read,             at_error_not_supported),
    AT_COMMAND_DEF (AT_HW,          at_error_not_supported, at_hw_read,             at_error_not_supported),
    AT_COMMAND_DEF (AT_DUTYC,       at_dutyc_set,           at_dutyc_read,          at_error_not_supported),
 //   AT_COMMAND_DEF (AT_COUNTER,     at_error_not_supported, at_counter_read,        at_error_not_supported),
    AT_COMMAND_DEF (AT_CHANNEL,     at_channel_set,         at_channel_read,        at_error_not_supported),
};

at_error_code_t at_error_not_supported (const uint8_t *param)
{
    return AT_ERROR_NOT_SUPPORTED;
}

at_error_code_t at_reset (const uint8_t *param)
{
    NVIC_SystemReset();

    return AT_OK;
}

at_error_code_t at_appeui_set (const uint8_t *param)
{
    uint8_t key[8];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3],
                &key[4], &key[5], &key[6], &key[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    lmh_join_eui_set(key);

    return AT_OK;
}

at_error_code_t at_appeui_read (const uint8_t *param)
{
    uint8_t key[8];
    uint8_t text[35];
    
    lmh_join_eui_get(key);

    sprintf(text, "+APPEUI: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3],
                key[4], key[5], key[6], key[7]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_appeui_test (const uint8_t *param)
{
    uint8_t text[35] = "+APPEUI: hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_joineui_set (const uint8_t *param)
{
    uint8_t key[8];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3],
                &key[4], &key[5], &key[6], &key[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    lmh_join_eui_set(key);

    return AT_OK;
}

at_error_code_t at_joineui_read (const uint8_t *param)
{
    uint8_t key[8];
    uint8_t text[35];
    
    lmh_join_eui_get(key);

    sprintf(text, "+JOINEUI: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3],
                key[4], key[5], key[6], key[7]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_joineui_test (const uint8_t *param)
{
    uint8_t text[35] = "+JOINEUI: hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}


at_error_code_t at_nwk_key_set(const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_nwk_key_set(key);

    return AT_OK;
}

at_error_code_t at_nwk_key_read(const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[60];
    
    lmh_nwk_key_get(key);

    sprintf(text, "+NWKKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_nwk_key_test (const uint8_t *param)
{
    uint8_t text[60] = "+NWKKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_f_nwk_s_int_key_set(const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_f_nwk_s_int_key_set(key);

    return AT_OK;
}

at_error_code_t at_f_nwk_s_int_key_read(const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_f_nwk_s_int_key_get(key);

    sprintf(text, "+FNWKSINTKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_f_nwk_s_int_key_test (const uint8_t *param)
{
    uint8_t text[70] = "+FNWKSINTKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_s_nwk_s_int_key_set(const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_s_nwk_s_int_key_set(key);

    return AT_OK;
}

at_error_code_t at_s_nwk_s_int_key_read(const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_s_nwk_s_int_key_get(key);

    sprintf(text, "+SNWKSINTKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_s_nwk_s_int_key_test (const uint8_t *param)
{
    uint8_t text[70] = "+SNWKSINTKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_nwk_s_enc_key_set(const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_nwk_s_enc_key_set(key);

    return AT_OK;
}

at_error_code_t at_nwk_s_enc_key_read(const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_s_nwk_s_int_key_get(key);

    sprintf(text, "+NWKSENCKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_nwk_s_enc_key_test (const uint8_t *param)
{
    uint8_t text[70] = "+NWKSENCKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_gen_app_key_set (const uint8_t *param)
{
    uint8_t appkey[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &appkey[0], &appkey[1], &appkey[2], &appkey[3], &appkey[4], &appkey[5], &appkey[6], &appkey[7],
                &appkey[8], &appkey[9], &appkey[10], &appkey[11], &appkey[12], &appkey[13], &appkey[14], &appkey[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_gen_app_key_set(appkey);

    return AT_OK;
}

at_error_code_t at_gen_app_key_read (const uint8_t *param)
{
    uint8_t appkey[16];
    uint8_t text[70];
    
    lmh_gen_app_key_get(appkey);

    sprintf(text, "+GENAPPKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                appkey[0], appkey[1], appkey[2], appkey[3], appkey[4], appkey[5], appkey[6], appkey[7],
                appkey[8], appkey[9], appkey[10], appkey[11], appkey[12], appkey[13], appkey[14], appkey[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_gen_app_key_test (const uint8_t *param)
{
    uint8_t text[70] = "+GENAPPKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}


at_error_code_t at_appkey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_app_key_set(key);

    return AT_OK;
}

at_error_code_t at_appkey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[60];
    
    lmh_app_key_get(key);

    sprintf(text, "+APPKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_appkey_test (const uint8_t *param)
{
    uint8_t text[60] = "+APPKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_genappkey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_gen_app_key_set(key);

    return AT_OK;
}

at_error_code_t at_genappkey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_gen_app_key_get(key);

    sprintf(text, "+GENAPPKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_genappkey_test (const uint8_t *param)
{
    uint8_t text[70] = "+GENAPPKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_nwkkey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_nwk_key_set(key);

    return AT_OK;
}

at_error_code_t at_nwkkey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_nwk_key_get(key);

    sprintf(text, "+NWKKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_nwkkey_test (const uint8_t *param)
{
    uint8_t text[70] = "+NWKKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_fnwksintkey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_f_nwk_s_int_key_set(key);

    return AT_OK;
}

at_error_code_t at_fnwksintkey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_f_nwk_s_int_key_get(key);

    sprintf(text, "+FNWKSINTKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_fnwksintkey_test (const uint8_t *param)
{
    uint8_t text[70] = "+FNWKSINTKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_snwksintkey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_s_nwk_s_int_key_set(key);

    return AT_OK;
}

at_error_code_t at_snwksintkey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_s_nwk_s_int_key_get(key);

    sprintf(text, "+SNWKSINTKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_snwksintkey_test (const uint8_t *param)
{
    uint8_t text[70] = "+SNWKSINTKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_nwksenckey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_nwk_s_enc_key_set(key);

    return AT_OK;
}

at_error_code_t at_nwksenckey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_nwk_s_enc_key_get(key);

    sprintf(text, "+NWKSENCKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_nwksenckey_test (const uint8_t *param)
{
    uint8_t text[70] = "+NWKSENCKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_appskey_set (const uint8_t *param)
{
    uint8_t key[16];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    lmh_app_s_key_set(key);

    return AT_OK;
}

at_error_code_t at_appskey_read (const uint8_t *param)
{
    uint8_t key[16];
    uint8_t text[70];
    
    lmh_app_s_key_get(key);

    sprintf(text, "+APPSKEY: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
                key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_appskey_test (const uint8_t *param)
{
    uint8_t text[70] = "+APPSKEY: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_devaddr_set (const uint8_t *param)
{
    uint8_t devaddr[4];
    uint32_t devaddr1;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx",
             &devaddr[0], &devaddr[1], &devaddr[2], &devaddr[3]) != 4)
    {
        return AT_ERROR_PARAM;
    }

    devaddr1  = ((uint32_t)devaddr[0]) << 24;
    devaddr1 |= ((uint32_t)devaddr[1]) << 16;
    devaddr1 |= ((uint32_t)devaddr[2]) << 8;
    devaddr1 |=  (uint32_t)devaddr[3];

    lmh_device_address_set(devaddr1);

    return AT_OK;
}

at_error_code_t at_devaddr_read (const uint8_t *param)
{
    uint8_t devaddr[4];
    uint32_t devaddr1;
    uint8_t text[24];
    
    lmh_device_address_get(&devaddr1);

    devaddr[0] = devaddr1 >> 24;
    devaddr[1] = devaddr1 >> 16;
    devaddr[2] = devaddr1 >> 8;
    devaddr[3] = devaddr1;

    sprintf(text, "+DEVADDR: %02x-%02x-%02x-%02x\r\n", 
            devaddr[0], devaddr[1], devaddr[2], devaddr[3]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_devaddr_test (const uint8_t *param)
{
    uint8_t text[24] = "+DEVADDR: hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_deveui_set (const uint8_t *param)
{
    uint8_t deveui[8];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
            &deveui[0], &deveui[1], &deveui[2], &deveui[3],
            &deveui[4], &deveui[5], &deveui[6], &deveui[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    lmh_device_eui_set(deveui);

    return AT_OK;
}

at_error_code_t at_deveui_read (const uint8_t *param)
{
    uint8_t deveui[8];
    uint8_t text[35];
    
    lmh_device_eui_get(deveui);

    sprintf(text, "+DEVEUI: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
            deveui[0], deveui[1], deveui[2], deveui[3],
            deveui[4], deveui[5], deveui[6], deveui[7]);

    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_deveui_test (const uint8_t *param)
{
    uint8_t text[35] = "+DEVEUI: hh-hh-hh-hh-hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(text, sizeof(text)-1);

    return AT_OK;
}

at_error_code_t at_netid_set (const uint8_t *param)
{
    uint8_t netid[3];
    uint32_t netid1;

    if (sscanf(param, "%hhx-%hhx-%hhx",
                &netid[0], &netid[1], &netid[2]) != 3)
    {
        return AT_ERROR_PARAM;
    }
    
    netid1  = 0;
    netid1 |= ((uint32_t)netid[0]) << 16;
    netid1 |= ((uint32_t)netid[1]) << 8;
    netid1 |=  (uint32_t)netid[2];

    lmh_network_id_set(netid1);

    return AT_OK;
}

at_error_code_t at_netid_read (const uint8_t *param)
{
    uint8_t netid[4];
    uint32_t netid1;
    uint8_t text[24];
    
    lmh_network_id_get(&netid1);

    netid[0] = netid1 >> 16;
    netid[1] = netid1 >> 8;
    netid[2] = netid1;

    sprintf(text, "+NETID: %02x-%02x-%02x\r\n", 
                    netid[0], netid[1], netid[2]);

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_netid_test (const uint8_t *param)
{
    uint8_t text[20] = "+NETID: hh-hh-hh\r\n";

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_joinrq_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t otaa;

    if (sscanf(param, "%u", &otaa) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (otaa > 1)
    {
        return AT_ERROR_PARAM;
    }

    m_otaa = otaa;
    lmh_error = lmh_join(m_otaa);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_joinstat_read(const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t join_status;
    uint8_t text[15];

    lmh_error = lmh_join_status_get(&join_status);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // Send response
    sprintf(text, "+JOINSTAT: %u\r\n", join_status);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rcv_read (const uint8_t *param)
{
    uint8_t text[100];

    sprintf(text, "+RCV: %u,%u,%s\r\n", m_lora_ack_received, m_lora_rx_data.port, m_lora_rx_data.buffer);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_send_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t text[128];
    uint8_t buffer[128];
    uint8_t buffersize = strlen(param);
    uint8_t *p_data = (uint8_t*)param;
    uint32_t is_confirmed;
    uint32_t port;
    
    memcpy(buffer, param, buffersize); 

    // Check if there is a confirmed/unconfirmed mode and application port
    if (sscanf(param, "%u,%u,", &is_confirmed, &port) != 2)
    {
        return AT_ERROR_PARAM;
    }   
    // Search for the payload
    // Skip confirm param
    while (*p_data != ',')
    {
        buffersize--;
        p_data++;
    }
    // Skip ','
    buffersize--;
    p_data++;
    // Skip port
    while (*p_data != ',')
    {
        buffersize--;
        p_data++;
    }
    // Skip ','
    buffersize--;
    p_data++;

    if (buffersize > LORAWAN_APP_DATA_BUFF_SIZE)
    {
        buffersize = LORAWAN_APP_DATA_BUFF_SIZE;
    }

    memcpy1(m_lora_tx_data.buffer, (uint8_t *)p_data, buffersize);
    m_lora_tx_data.buffsize = buffersize;
    m_lora_tx_data.port = port;
    m_lora_tx_data.confirmed = is_confirmed;

    m_lora_ack_received = false;
    lmh_error = lmh_send(&m_lora_tx_data);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_adr_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t adr = *param;

    if (sscanf(param, "%u", &adr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (adr > 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_adaptative_datarate_set(adr);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_adr_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t is_adr_enabled;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_adaptative_datarate_get(&is_adr_enabled);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+ADR: %d\r\n", is_adr_enabled);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_class_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t class_param;
    lmh_device_class_t new_class;

    if (sscanf(param, "%c", &class_param) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (class_param != 'a' &&  class_param != 'A' &&
        class_param != 'b' &&  class_param != 'B' &&
        class_param != 'c' &&  class_param != 'C')
    {
        return AT_ERROR_PARAM;
    }

    if      (class_param == 'a' ||  class_param = 'A') new_class = CLASS_A;
    else if (class_param == 'b' ||  class_param = 'B') new_class = CLASS_B;
    else if (class_param == 'c' ||  class_param = 'C') new_class = CLASS_C;


    // run AT command
    lmh_error = lmh_class_set(new_class);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_class_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t lora_class;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_class_get(&lora_class);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+CLASS: %c\r\n", "ABC"[lora_class]);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_class_test (const uint8_t *param)
{
    uint8_t text[15] = "+CLASS: A,B,C\r\n";

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_dr_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t dr;

    if (sscanf(param, "%u", &dr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (dr > 7)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_tx_datarate_set(dr);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_dr_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t dr;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_tx_datarate_get(&dr);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+DR: %u\r\n", dr);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_dr_test (const uint8_t *param)
{
    uint8_t text[25] = "+DR: 0,1,2,3,4,5,6,7\r\n";

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_joindly1_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_joinAcceptDelay1_set(delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_joindly1_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_joinAcceptDelay1_get(&delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+JOINDLY1: %u\r\n", delay);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_joindly2_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_joinAcceptDelay2_set(delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_joindly2_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_joinAcceptDelay2_get(&delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);


    // send response
    sprintf(text, "+JOINDLY2: %u\r\n", delay);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_pnet_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t public_mode;

    if (sscanf(param, "%u", &public_mode) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (public_mode > 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_publicNetwork_set(public_mode);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_pnet_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t public_mode;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_publicNetwork_get(&public_mode);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+PNET: %u\r\n", public_mode);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rxdly1_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_rxDelay1_set(delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_rxdly1_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_rxDelay1_get(&delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+RXDLY1: %u\r\n", delay);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rxdly2_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_rxDelay2_set(delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_rxdly2_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t delay;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_rxDelay2_get(&delay);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+RXDLY2: %u\r\n", delay);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rxdr2_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t dr = *param;

    if (sscanf(param, "%u", &dr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (dr > 7)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_rxDataRate2_set(dr);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_rxdr2_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t dr;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_rxDataRate2_get(&dr);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+RXDR2: %u\r\n", dr);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rxdr2_test (const uint8_t *param)
{
    uint8_t text[26] = "+RXDR2: 0,1,2,3,4,5,6,7\r\n";

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rxfq2_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t freq;

    if (sscanf(param, "%u", &freq) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_rxFrequency2_set(freq);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_rxfq2_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t freq;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_rxFrequency2_get(&freq);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+RXFQ2: %u\r\n", freq);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_txp_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t txp;

    if (sscanf(param, "%u", &txp) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (txp > 5)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_error = lmh_tx_power_set(txp);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_txp_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t txp;
    uint8_t text[15];

    // run AT command
    lmh_error = lmh_tx_power_get(&txp);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    sprintf(text, "+TXP: %u\r\n", txp);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_txp_test (const uint8_t *param)
{
    uint8_t text[25] = "+TXP: 0,1,2,3,4,5\r\n";

    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_batt_read (const uint8_t *param)
{
    uint8_t batt;
    uint8_t text[25];

    // run AT command
    batt = BoardGetBatteryLevel();

    // send response
    sprintf(text, "+BATT: %u\r\n", batt);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_rssi_read (const uint8_t *param)
{
    uint8_t text[25];

    // send response
    sprintf(text, "+RSSI: %d\r\n", m_lora_rx_data.rssi);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_snr_read (const uint8_t *param)
{
    uint8_t text[25];

    // send response
    sprintf(text, "+SNR: %d\r\n", m_lora_rx_data.snr);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_hw_read (const uint8_t *param)
{
    uint8_t text[25];

    // send response
#if defined(ISP4520_EU)
    sprintf(text, "+HW: EU-%c\r\n", BoardGetRevision());
#elif defined(ISP4520_AS) 
    sprintf(text, "+HW: AS-%c\r\n", BoardGetRevision());
#elif defined(ISP4520_US)
    sprintf(text, "+HW: US-%c\r\n", BoardGetRevision());
#else
    #error "Please define a ISP4520 configuration"
#endif 
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_fw_read (const uint8_t *param)
{
    uint8_t text[25];

    // send response
    sprintf(text, "+FW: %s\r\n", FW_REVISION);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

at_error_code_t at_dutyc_set (const uint8_t *param)
{
    uint8_t duty;

    if (sscanf(param, "%u", &duty) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (duty >1)
    {
        return AT_ERROR_PARAM;
    }

    // run AT command
    lmh_duty_cycle_set(duty ? true : false);

    return AT_OK;
}

at_error_code_t at_dutyc_read (const uint8_t *param)
{
    uint8_t text[25];
    bool duty;

    // run AT command
    lmh_duty_cycle_get(&duty);

    // send response
    sprintf(text, "+DUTYC: %u\r\n", duty? 1 : 0);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    return AT_OK;
}

//at_error_code_t at_counter_read (const uint8_t *param)
//{
//    uint8_t text[25];
//    uint8_t up_counter;
//    uint8_t down_counter;
//
//    // run AT command
//    lmh_counter_get(&up_counter, &down_counter);
//
//    // send response
//    sprintf(text, "+COUNTER: %u, %u\r\n", up_counter, down_counter);
//    at_hal_transport_tx_pkt_send(text, strlen(text));
//
//    return AT_OK;
//}

at_error_code_t at_channel_set (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint32_t id;
    lmh_channel_param_t channel_param;
    uint32_t drmin, drmax;
    uint32_t freq;


    if (sscanf(param, "%u,%u,%u,%u", &id, &freq, &drmin, &drmax) != 4)
    {
        return AT_ERROR_PARAM;
    }
    channel_param.DrRange.Value = (drmin & 0x0F) | (drmax << 4);
    channel_param.Frequency = freq;

    // run AT command
    lmh_error = lmh_channel_set(id, channel_param);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    return AT_OK;
}

at_error_code_t at_channel_read (const uint8_t *param)
{
    lmh_error_code_t lmh_error;
    at_error_code_t at_error;
    uint8_t text[40];
    lmh_channel_param_t channels[16];

    // run AT command
    lmh_error = lmh_channels_get(channels);
    CONVERT_LMH_TO_AT_ERROR(lmh_error, at_error);
    AT_VERIFY_SUCCESS(at_error);

    // send response
    for (int i=0; i<16; i++)
    {
        sprintf(text, "+CHANNEL: %u, %u, %u, %u\r\n", i, channels[i].Frequency, channels[i].DrRange.Fields.Min, channels[i].DrRange.Fields.Max);
        at_hal_transport_tx_pkt_send(text, strlen(text));
    }

    return AT_OK;
}




/**
 * @brief  Send final response
 * @param[in] 
 */
static void final_response_send(at_error_code_t err_code)
{
    if (err_code > AT_ERROR_OTHER)
    {
        err_code = AT_ERROR_OTHER;
    }
    at_hal_transport_tx_pkt_send(at_error_description[err_code], strlen(at_error_description[err_code]));
}

/**@brief A function for processing the HAL Transport layer events.
 *
 * @param[in] event    HAL Transport layer event.
 */
static void at_conn_hal_transport_event_handle (at_hal_transport_evt_t event)
{   
    switch (event.evt_type)
    {
        case AT_HAL_TRANSP_EVT_TX_PKT_SENT:
        {
            break;
        }

        case AT_HAL_TRANSP_EVT_RX_PKT_RECEIVED:
        {
            memcpy1(m_rx_at_command, event.evt_params.rx_pkt_received.p_buffer, event.evt_params.rx_pkt_received.num_of_bytes);
            m_at_command_ready = true;
            break;
        }

        case AT_HAL_TRANSP_EVT_PHY_ERROR:
        {
           // APP_ERROR_CHECK(NRF_ERROR_FORBIDDEN);
            NRF_LOG_ERROR("AT_HAL_TRANSP_EVT_PHY_ERROR");
            break;
        }

        default:
        {
            /* do nothing */
            break;
        }
    }
}


/**@brief Function for handling event from the LMH layer
 *
 * @param[in] type  event type 
 * @param[in] data  event data
 */
static void lmh_evt_handler(lmh_evt_type_t type, void *data)
{
    switch(type)
    {
        case LHM_EVT_NWK_JOINED:
        {
            if (m_otaa)
            {
                uint8_t text[10] = "+JOINED\r\n";
                at_hal_transport_tx_pkt_send(text, strlen(text));
            }
            NRF_LOG_INFO("Network Joined");
        } 
        break;

        case LHM_EVT_RX_ACK:
        {
            uint8_t text[10] = "+RXACK\r\n";
            at_hal_transport_tx_pkt_send(text, strlen(text));

            m_lora_ack_received = true;
            NRF_LOG_INFO("Ack received");
        } 
        break;

        case LHM_EVT_CLASS_CHANGED:
        {
            int new_class = *(int *)data; 
            NRF_LOG_INFO("Device class changed to %c", "ABC"[new_class]);
        }
        break;

        default:
            break;
    }
}

/**@brief LoRa function for handling received data from server
 *
 * @param[in] app_data  Pointer to rx data
 */
static void lmh_rx_data_handler (lmh_app_rx_data_t *app_data)
{
    uint8_t text[128];

    memcpy(&m_lora_rx_data, app_data, sizeof(lmh_app_rx_data_t));

     sprintf(text, "+RXDATA: %u,%s\r\n", m_lora_rx_data.port, m_lora_rx_data.buffer);
    at_hal_transport_tx_pkt_send(text, strlen(text));

    NRF_LOG_INFO("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d", app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
}

at_error_code_t at_manager_execute()
{
    at_error_code_t err_code;
    at_command_t *current_at_command;
    uint8_t *p_data;

    if (m_at_command_ready)
    {
        if ((m_rx_at_command[0] != 'A') || (m_rx_at_command[1] != 'T'))
        {
            err_code = AT_ERROR_UNKNOWN_CMD;
        }
        else
        {
            err_code = AT_ERROR_UNKNOWN_CMD;
            p_data = m_rx_at_command+2;
    
            for (int i=0; i < (sizeof(at_commands) / sizeof(at_command_t)); i++)
            {
                if (strncmp(p_data, at_commands[i].name, at_commands[i].name_size) == 0)
                {
                    // command found 
                    current_at_command = &(at_commands[i]);
                    p_data += current_at_command->name_size;
                        
                    //  parse the type (set, read or test), and jump to the correcponding function
                    if (p_data[0] == '\r' || p_data[0] == '\n')
                    {
                        err_code = current_at_command->set(p_data+1);
                    }
                    else if (p_data[0] == '?')
                    {
                        err_code = current_at_command->read(p_data+1);
                    }
                    else if (p_data[0] == '=')
                    {
                        if (p_data[1] == '?')
                        {
                            err_code = current_at_command->test(p_data+1);
                        }
                        else if (p_data[1] != '\r' && p_data[1] != '\n')
                        {
                            err_code = current_at_command->set(p_data+1);
                        }
                    }
                    //  we end the loop as the command was found 
                    break;
                }
            }
        }

        // Send final response
        final_response_send(err_code);


        memset1(m_rx_at_command, 0, 128);
        m_at_command_ready = false;
    }
}

at_error_code_t at_manager_init()
{
    uint32_t err_code;

    // Initialize LoRaWan
    err_code = lmh_init(&lora_callbacks);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Initialize transport
    err_code = at_hal_transport_open(at_conn_hal_transport_event_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return err_code;
}