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

// LoRa
#include "board.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "Commissioning.h"
#include "RegionCommon.h"
#include "NvmDataMgmt.h"
#include "RegionCommon.h"
#include "LoRaMacTest.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE        242                             /**< Size of the LoRa data to be allocate. */
#define APP_TX_DUTYCYCLE                        5000                            /**< Data transmission duty cycle in ms. */
#define APP_TX_DUTYCYCLE_RND                    1000                            /**< Random delay in ms for application data transmission duty cycle. */
#define LORAWAN_DEFAULT_CLASS                   CLASS_A                         /**< LoRaWAN default end-device class. */
#define LORAWAN_ADR_STATE                       LORAMAC_HANDLER_ADR_ON          /**< LoRaWan Default Adaptive Data Rate. */
#define LORAWAN_DEFAULT_DATARATE                DR_0                            /**< LoRaWan Default Data Rate. */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE     LORAMAC_HANDLER_UNCONFIRMED_MSG /**< LoRaWan Default confirmed messages. */
#define LORAWAN_DUTYCYCLE_ON                    true                            /**< LoRaWAN ETSI duty cycle control enable/disable. */

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

/**
 * @brief Macro for converting LoRaMac error type into AT commands error type
 */
#define CONVERT_LORAMAC_TO_AT_ERROR(loramac_error, at_error) \
do                                                          \
{                                                           \
    if (loramac_error == LORAMAC_STATUS_OK)                     \
    {                                                       \
        at_error = AT_OK;                                   \
    }                                                       \
    else if (loramac_error == LORAMAC_STATUS_PARAMETER_INVALID) \
    {                                                       \
        at_error = AT_ERROR_PARAM;                          \
    }                                                       \
    else if (loramac_error == LORAMAC_STATUS_BUSY)              \
    {                                                       \
        at_error = AT_ERROR_BUSY;                           \
    }                                                       \
    else if (loramac_error == LORAMAC_STATUS_NO_NETWORK_JOINED) \
    {                                                       \
        at_error = AT_ERROR_NOT_JOINED;                     \
    }                                                       \
    else if (loramac_error == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED)  \
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
static void OnTxPeriodicityChanged(uint32_t periodicity);
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);
static void OnNetworkParametersChange(CommissioningParams_t* params);
static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size);
static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn);
static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn);
static void OnJoinRequest(LmHandlerJoinParams_t* params);
static void OnMacProcessNotify(void);
static void OnTxData(LmHandlerTxParams_t* params);
static void OnRxData(LmHandlerAppData_t* appData, LmHandlerRxParams_t* params);
static void OnClassChange(DeviceClass_t deviceClass);
static void OnBeaconStatusChange(LoRaMacHandlerBeaconParams_t* params);
#if(LMH_SYS_TIME_UPDATE_NEW_API == 1)
static void OnSysTimeUpdate(bool isSynchronized, int32_t timeCorrection);
#else
static void OnSysTimeUpdate(void);
#endif

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


static MibRequestConfirm_t mibReq;   
static TimerEvent_t TxCertifTimer;                                      /**< Timer to handle the data transmission duty cycle for certification. */  
static volatile uint32_t TxPeriodicity = 0;    
static volatile uint8_t IsTxFramePending = 0;                                     
static volatile uint8_t IsMacProcessPending = 0;       
static uint8_t TxAppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];       /*< Lora TX data buffer. */
static LmHandlerAppData_t TxAppData = {0, 0, TxAppDataBuffer};          /*< Lora TX data structure. */
static uint8_t RxAppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];       /*< Lora RX data buffer. */
static LmHandlerAppData_t RxAppData = {0 ,0, RxAppDataBuffer};          /*< Lora RX data structure. */
static int8_t LastRssi = 0;
static int8_t LastSnr = 0;
static bool m_at_command_ready = false;
static bool m_lora_ack_received = false;
static bool m_otaa = false;
uint8_t m_rx_buffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE+MAX_AT_PREFIX_SIZE] = {0};
uint8_t m_tx_buffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE+MAX_AT_PREFIX_SIZE] = {0};
static uint8_t m_echo = 0;

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = NULL,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
{
#if defined (REGION_AS923)
    .Region = LORAMAC_REGION_AS923,
#elif defined (REGION_AU915)
    .Region = LORAMAC_REGION_AU915,
#elif defined (REGION_CN470)
    .Region = LORAMAC_REGION_CN470,
#elif defined (REGION_CN779)
    .Region = LORAMAC_REGION_CN779,
#elif defined (REGION_EU433)
    .Region = LORAMAC_REGION_EU433,
#elif defined (REGION_IN865)
    .Region = LORAMAC_REGION_IN865,
#elif defined (REGION_EU868)
    .Region = LORAMAC_REGION_EU868,
#elif defined (REGION_KR920)
    .Region = LORAMAC_REGION_KR920,
#elif defined (REGION_US915)
    .Region = LORAMAC_REGION_US915,
#elif defined (REGION_US915_HYBRID)
    .Region = LORAMAC_REGION_US915_HYBRID,
#else
     #error "Please define a region in the compiler options."
#endif	
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = TxAppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FW_VERSION_NUM,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

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


static void OnTxPeriodicityChanged(uint32_t periodicity)
{
    TxPeriodicity = periodicity;

    if (TxPeriodicity == 0)
    { 
        // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    }
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

static void OnMacProcessNotify(void)
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state, uint16_t size)
{
}

static void OnNetworkParametersChange(CommissioningParams_t* params)
{
}

static void OnMacMcpsRequest(LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn)
{
}

static void OnMacMlmeRequest(LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn)
{
}

static void OnJoinRequest(LmHandlerJoinParams_t* params)
{
    if(params->Status == LORAMAC_HANDLER_SUCCESS)
    {
        sprintf(m_tx_buffer, "+JOINED\r\n");
        at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));
    }
}

static void OnTxData(LmHandlerTxParams_t* params)
{
}

static void OnRxData(LmHandlerAppData_t* appData, LmHandlerRxParams_t* params)
{
    memcpy(&RxAppData, appData, sizeof(RxAppData));
    LastRssi = params->Rssi;
    LastSnr = params->Snr;

    sprintf(m_tx_buffer, "+RXDATA: %u,%s\r\n", RxAppData.Port, RxAppData.Buffer);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));
}

static void OnClassChange(DeviceClass_t deviceClass)
{
    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend(&appData, LORAMAC_HANDLER_UNCONFIRMED_MSG);
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{
}
#else
static void OnSysTimeUpdate( void )
{
}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame(void)
{
    if (LmHandlerIsBusy() == true)
    {
        return;
    }

    uint8_t msg[] = "Test";
    TxAppData.Port = 2;
    strcpy(TxAppData.Buffer, msg);
    TxAppData.BufferSize = sizeof(msg);
    LmHandlerSend(&TxAppData, LmHandlerParams.IsTxConfirmed);
}

static void UplinkProcess(void)
{
    uint8_t isPending = 0;
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    if (isPending == 1)
    {
        PrepareTxFrame();
    }
}


/*!
 * Function executed on TxTimer event
 */
static void OnTxCertifTimerEvent(void* conm_tx_buffer)
{
    TimerStop(&TxCertifTimer);

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue(&TxCertifTimer, TxPeriodicity);
    TimerStart(&TxCertifTimer);
}

static void StartTxCertifProcess(void)
{
    // Schedule 1st packet transmission
    TimerInit(&TxCertifTimer, OnTxCertifTimerEvent);
    TimerSetValue(&TxCertifTimer, TxPeriodicity);
    OnTxCertifTimerEvent(NULL);
}

static void StopTxCertifProcess(void)
{
    TimerStop(&TxCertifTimer);
    IsTxFramePending = 0;
}

at_error_code_t at_error_not_supported(const uint8_t *param)
{
    return AT_ERROR_NOT_SUPPORTED;
}

at_error_code_t at_reset(const uint8_t *param)
{
    NRF_LOG_FINAL_FLUSH();

    BoardResetMcu();

    return AT_OK;
}

static at_error_code_t at_echo_set(const uint8_t *param)
{
    int echo;

    if (sscanf(param, "%u", &echo) != 1)
    {
        return AT_ERROR_PARAM;
    }

    echo = (uint8_t)echo;

    if (echo > 1)
    {
        return AT_ERROR_PARAM;
    }

    m_echo = echo;

    return AT_OK;
}

at_error_code_t at_echo_read(const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_ECHO, m_echo);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_info_read(const uint8_t *param)
{
    // Send module name
    sprintf(m_tx_buffer, "%s\r\n", MODULE_NAME);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    // Send device id
    sprintf(m_tx_buffer, "%X%X\r\n", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    // Send firmware version
    sprintf(m_tx_buffer, "%s\r\n", FW_VERSION_STR);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_appeui_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t key[8];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3],
                &key[4], &key[5], &key[6], &key[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_EUI;
    mibReq.Param.JoinEui = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_appeui_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t key[8];
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_EUI;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    memcpy(key, mibReq.Param.JoinEui, SE_EUI_SIZE);
    sprintf(m_tx_buffer, "%s: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
            AT_APPEUI, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]);

    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_appeui_test (const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_APPEUI);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_joineui_set (const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t key[8];

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3],
                &key[4], &key[5], &key[6], &key[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_EUI;
    mibReq.Param.JoinEui = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_joineui_read (const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t key[8];
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_EUI;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    memcpy(key, mibReq.Param.JoinEui, SE_EUI_SIZE);
    sprintf(m_tx_buffer, "%s: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
            AT_JOINEUI, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]);

    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_joineui_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_JOINEUI);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_nwk_key_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_NWK_KEY;
    mibReq.Param.NwkKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_nwk_key_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_NWKKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_f_nwk_s_int_key_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_F_NWK_S_INT_KEY;
    mibReq.Param.FNwkSIntKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_f_nwk_s_int_key_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_FNWKSINTKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_s_nwk_s_int_key_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_S_NWK_S_INT_KEY;
    mibReq.Param.SNwkSIntKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_s_nwk_s_int_key_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_SNWKSINTKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_nwk_s_enc_key_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_NWK_S_ENC_KEY;
    mibReq.Param.NwkSEncKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_nwk_s_enc_key_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_NWKSENCKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_appkey_set (const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_APP_KEY;
    mibReq.Param.AppKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_appkey_test(const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_APPKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_nwkkey_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_NWK_KEY;
    mibReq.Param.NwkKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_nwkkey_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_NWKKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_fnwksintkey_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_F_NWK_S_INT_KEY;
    mibReq.Param.FNwkSIntKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_fnwksintkey_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_FNWKSINTKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_snwksintkey_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_S_NWK_S_INT_KEY;
    mibReq.Param.SNwkSIntKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_snwksintkey_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_SNWKSINTKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_nwksenckey_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_NWK_S_ENC_KEY;
    mibReq.Param.NwkSEncKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_nwksenckey_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_NWKSENCKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_appskey_set(const uint8_t *param)
{
    uint8_t key[16];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
                &key[0], &key[1], &key[2], &key[3], &key[4], &key[5], &key[6], &key[7],
                &key[8], &key[9], &key[10], &key[11], &key[12], &key[13], &key[14], &key[15]) != 16)
    {
        return AT_ERROR_PARAM;
    }
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_APP_S_KEY;
    mibReq.Param.AppSKey = key;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_appskey_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_APPSKEY);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_devaddr_set(const uint8_t *param)
{
    uint8_t devaddr[4];
    uint32_t devaddr1;
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx",
             &devaddr[0], &devaddr[1], &devaddr[2], &devaddr[3]) != 4)
    {
        return AT_ERROR_PARAM;
    }

    devaddr1  = ((uint32_t)devaddr[0]) << 24;
    devaddr1 |= ((uint32_t)devaddr[1]) << 16;
    devaddr1 |= ((uint32_t)devaddr[2]) << 8;
    devaddr1 |=  (uint32_t)devaddr[3];

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_DEV_ADDR;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.DevAddr = devaddr1;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_devaddr_read(const uint8_t *param)
{
    uint8_t devaddr[4];
    uint32_t devaddr1;
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    
    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_DEV_ADDR;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    devaddr1 = mibReq.Param.DevAddr;
    devaddr[0] = devaddr1 >> 24;
    devaddr[1] = devaddr1 >> 16;
    devaddr[2] = devaddr1 >> 8;
    devaddr[3] = devaddr1;

    sprintf(m_tx_buffer, "%s: %02x-%02x-%02x-%02x\r\n", 
            AT_DEVADDR, devaddr[0], devaddr[1], devaddr[2], devaddr[3]);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_devaddr_test(const uint8_t *param)
{
    uint8_t m_tx_buffer[24] = "+DEVADDR: hh-hh-hh-hh\r\n";
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_deveui_set(const uint8_t *param)
{
    uint8_t key[8];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx-%hhx",
            &key[0], &key[1], &key[2], &key[3],
            &key[4], &key[5], &key[6], &key[7]) != 8)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_DEV_EUI;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    memcpy(mibReq.Param.DevEui, key, SE_EUI_SIZE);
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_deveui_read(const uint8_t *param)
{
    uint8_t key[8];
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    
    mibReq.Type = MIB_DEV_EUI;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    memcpy(key, mibReq.Param.DevEui, SE_EUI_SIZE);
    sprintf(m_tx_buffer, "%s: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", 
            AT_DEVEUI, key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7]);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_deveui_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh-hh-hh-hh-hh-hh\r\n", AT_DEVEUI);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_netid_set (const uint8_t *param)
{
    uint8_t netid[3];
    uint32_t netid1;
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;

    if (sscanf(param, "%hhx-%hhx-%hhx",
                &netid[0], &netid[1], &netid[2]) != 3)
    {
        return AT_ERROR_PARAM;
    }
    
    netid1  = 0;
    netid1 |= ((uint32_t)netid[0]) << 16;
    netid1 |= ((uint32_t)netid[1]) << 8;
    netid1 |=  (uint32_t)netid[2];

    mibReq.Type = MIB_NET_ID;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.NetID = netid1;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_netid_read(const uint8_t *param)
{
    uint8_t netid[4];
    uint32_t netid1;
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    
    mibReq.Type = MIB_NET_ID;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    netid1 = mibReq.Param.NetID;
    netid[0] = netid1 >> 16;
    netid[1] = netid1 >> 8;
    netid[2] = netid1;

    sprintf(m_tx_buffer, "%s: %02x-%02x-%02x\r\n",  AT_NETID, netid[0], netid[1], netid[2]);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_netid_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: hh-hh-hh\r\n", AT_NETID);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_joinrq_set (const uint8_t *param)
{
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
    LmHandlerJoinRequest(m_otaa);

    return AT_OK;
}

at_error_code_t at_joinstat_read(const uint8_t *param)
{
    uint8_t join_status;

    join_status = LmHandlerJoinStatus();

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_JOINSTAT, join_status);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rcv_read(const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: %u,%u,%s\r\n", AT_RCV, m_lora_ack_received, RxAppData.Port, RxAppData.Buffer);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_send_set(const uint8_t *param)
{
    LmHandlerErrorStatus_t lm_err_code;
    at_error_code_t at_err_code;
    uint8_t buffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];
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

    if (buffersize > LORAWAN_APP_DATA_BUFFER_MAX_SIZE)
    {
        buffersize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE;
    }

    memcpy(TxAppData.Buffer, (uint8_t *)p_data, buffersize);
    TxAppData.BufferSize = buffersize;
    TxAppData.Port = port;
    LmHandlerParams.IsTxConfirmed = is_confirmed;

    // Check if device is activated
    if(LmHandlerJoinStatus() != LORAMAC_HANDLER_SET)
    {
        // The network isn't joined
        return AT_ERROR_NOT_JOINED;
    }

    m_lora_ack_received = false;
    lm_err_code = LmHandlerSend(&TxAppData, LmHandlerParams.IsTxConfirmed);
    if (lm_err_code != LORAMAC_HANDLER_SUCCESS)
    {
        return AT_ERROR_OTHER;
    }

    return AT_OK;
}

at_error_code_t at_adr_set (const uint8_t *param)
{
    LmHandlerErrorStatus_t lm_err_code;
    at_error_code_t at_err_code;
    uint8_t adr = *param;

    if (sscanf(param, "%u", &adr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (adr > 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    LoRaMacStatus_t status;
    mibReq.Type = MIB_ADR;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.AdrEnable = adr;
    status = LoRaMacMibSetRequestConfirm(&mibReq);	
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_adr_read (const uint8_t *param)
{
    LmHandlerErrorStatus_t lm_err_code;
    at_error_code_t at_err_code;
    uint8_t is_adr_enabled;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    LoRaMacStatus_t status;
    mibReq.Type = MIB_ADR;
    status = LoRaMacMibGetRequestConfirm(&mibReq);	
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    is_adr_enabled = mibReq.Param.AdrEnable;
    sprintf(m_tx_buffer, "%s: %d\r\n", AT_ADR, is_adr_enabled);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_class_set (const uint8_t *param)
{
    LmHandlerErrorStatus_t lm_err_code;
    at_error_code_t at_error;
    uint8_t class_param;
    DeviceClass_t new_class;

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

    if      (class_param == 'a' ||  class_param == 'A') new_class = CLASS_A;
    else if (class_param == 'b' ||  class_param == 'B') new_class = CLASS_B;
    else if (class_param == 'c' ||  class_param == 'C') new_class = CLASS_C;

    lm_err_code = LmHandlerRequestClass(new_class);
    if (lm_err_code != LORAMAC_HANDLER_SUCCESS)
    {
        return AT_ERROR_OTHER;
    }

    return AT_OK;
}

at_error_code_t at_class_read (const uint8_t *param)
{
    uint8_t lora_class;

    lora_class = LmHandlerGetCurrentClass();

    sprintf(m_tx_buffer, "%s: %c\r\n", AT_CLASS, "ABC"[lora_class]);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_class_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: A,B,C\r\n", AT_CLASS);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_dr_set (const uint8_t *param)
{
    uint8_t dr;

    if (sscanf(param, "%u", &dr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (dr > 7)
    {
        return AT_ERROR_PARAM;
    }

    LmHandlerParams.TxDatarate = dr;

    return AT_OK;
}

at_error_code_t at_dr_read(const uint8_t *param)
{
    uint8_t dr;

    dr = LmHandlerGetCurrentDatarate();

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_DR, dr);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_dr_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: 0,1,2,3,4,5,6,7\r\n", AT_DR);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_joindly1_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.JoinAcceptDelay1 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_joindly1_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_JOINDLY1, delay);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_joindly2_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.JoinAcceptDelay1 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_joindly2_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_JOINDLY2, delay);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_pnet_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t public_mode;

    if (sscanf(param, "%u", &public_mode) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (public_mode > 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_PUBLIC_NETWORK;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.EnablePublicNetwork = public_mode;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_pnet_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t public_mode;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_PUBLIC_NETWORK;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    public_mode =  mibReq.Param.EnablePublicNetwork;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_PNET, public_mode);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rxdly1_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RECEIVE_DELAY_1;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.ReceiveDelay1 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_rxdly1_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RECEIVE_DELAY_1;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    delay =  mibReq.Param.ReceiveDelay1;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_RXDLY1, delay);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rxdly2_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    if (sscanf(param, "%u", &delay) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RECEIVE_DELAY_2;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.ReceiveDelay2 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_rxdly2_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t delay;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RECEIVE_DELAY_2;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    delay =  mibReq.Param.ReceiveDelay2;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_RXDLY2, delay);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rxdr2_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t dr;

    if (sscanf(param, "%u", &dr) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (dr > 7)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RX2_CHANNEL;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.Rx2Channel.Datarate = dr;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_rxdr2_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t dr;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    dr =  mibReq.Param.Rx2Channel.Datarate;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_RXDR2, dr);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rxdr2_test(const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: 0,1,2,3,4,5,6,7\r\n", AT_RXDR2);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rxfq2_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t freq;

    if (sscanf(param, "%u", &freq) != 1)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RX2_CHANNEL;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.Rx2Channel.Frequency = freq;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_rxfq2_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t freq;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    freq =  mibReq.Param.Rx2Channel.Frequency;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_RXFQ2, freq);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_txp_set(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t txp;

    if (sscanf(param, "%u", &txp) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (txp > 5)
    {
        return AT_ERROR_PARAM;
    }

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    LoRaMacMibGetRequestConfirm(&mibReq);
    mibReq.Param.ChannelsTxPower = txp;
    status = LoRaMacMibSetRequestConfirm(&mibReq);	
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    return AT_OK;
}

at_error_code_t at_txp_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint8_t txp;

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    txp =  mibReq.Param.ChannelsTxPower;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_TXP, txp);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_txp_test (const uint8_t *param)
{
    sprintf(m_tx_buffer,"%s: 0,1,2,3,4,5\r\n", AT_TXP);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_batt_read(const uint8_t *param)
{
    uint8_t batt;

    batt = BoardGetBatteryLevel();

    sprintf(m_tx_buffer, "%s: %u\r\n", AT_BATT, batt);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_rssi_read(const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: %d\r\n", AT_RSSI, LastRssi);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_snr_read(const uint8_t *param)
{
    sprintf(m_tx_buffer, "%s: %d\r\n", AT_SNR, LastSnr);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_dutyc_set(const uint8_t *param)
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

#if defined (REGION_EU868)
    LmHandlerParams.DutyCycleEnabled = (duty?  true : false);
    LoRaMacTestSetDutyCycleOn(LmHandlerParams.DutyCycleEnabled);
#endif

    return AT_OK;
}

at_error_code_t at_dutyc_read(const uint8_t *param)
{
    bool duty;

    duty = LmHandlerParams.DutyCycleEnabled;
    sprintf(m_tx_buffer, "%s: %u\r\n", AT_DUTYC, duty? 1 : 0);
    at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

    return AT_OK;
}

at_error_code_t at_channel_set (const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    uint32_t id;
    ChannelParams_t channel_param;
    uint32_t drmin, drmax;
    uint32_t freq;

    if (sscanf(param, "%u,%u,%u,%u", &id, &freq, &drmin, &drmax) != 4)
    {
        return AT_ERROR_PARAM;
    }
    channel_param.DrRange.Value = (drmin & 0x0F) | (drmax << 4);
    channel_param.Frequency = freq;

    // Not implemented by LmHandler, so we call LoRaMacChannelAdd and LoRaMacChannelRemove directly
    if (channel_param.Frequency != 0)
    {
        status = LoRaMacChannelAdd(id, (ChannelParams_t){ channel_param.Frequency, 0, channel_param.DrRange, 0 });
        CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
        AT_VERIFY_SUCCESS(at_err_code);
    }
    else
    {
        status = LoRaMacChannelRemove(id);
        CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
        AT_VERIFY_SUCCESS(at_err_code);
    }

    return AT_OK;
}

at_error_code_t at_channel_read(const uint8_t *param)
{
    LoRaMacStatus_t status;
    at_error_code_t at_err_code;
    ChannelParams_t channels[16];

    // Not implemented by LmHandler, so we call LoRaMacMibSetRequestConfirm directly
    mibReq.Type = MIB_CHANNELS;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    CONVERT_LORAMAC_TO_AT_ERROR(status, at_err_code);
    AT_VERIFY_SUCCESS(at_err_code);

    memcpy(channels, mibReq.Param.ChannelList, 16*sizeof(ChannelParams_t));
    for (int i=0; i<16; i++)
    {
        sprintf(m_tx_buffer, "%s: %u, %u, %u, %u\r\n", AT_CHANNEL, i, channels[i].Frequency, channels[i].DrRange.Fields.Min, channels[i].DrRange.Fields.Max);
        at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));
    }

    return AT_OK;
}

at_error_code_t at_certif_set(const uint8_t *param)
{
    uint8_t start;

    if (sscanf(param, "%u", &start) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (start >1)
    {
        return AT_ERROR_PARAM;
    }

    if (start == 1)
    {
#if defined (REGION_EU868)
        LoRaMacTestSetDutyCycleOn(false);
#endif
        StartTxCertifProcess();
    }
    else
    {
#if defined (REGION_EU868)
        LoRaMacTestSetDutyCycleOn(true);
#endif
        StopTxCertifProcess();
    }

    return AT_OK;
}

at_error_code_t at_ctxrst_set(const uint8_t *param)
{
    uint8_t confirm;

    if (sscanf(param, "%u", &confirm) != 1)
    {
        return AT_ERROR_PARAM;
    }

    if (confirm!=1)
    {
        return AT_ERROR_PARAM;
    }

    NvmDataMgmtFactoryReset();
    NRF_LOG_FINAL_FLUSH();
    BoardResetMcu();

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
            memcpy(m_rx_buffer, event.evt_params.rx_pkt_received.p_buffer, event.evt_params.rx_pkt_received.num_of_bytes);
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
//static void lmh_evt_handler(lmh_evt_type_t type, void *data)
//{
//    switch(type)
//    {

//        case LHM_EVT_RX_ACK:
//        {
//            uint8_t m_tx_buffer[10] = "+RXACK\r\n";
//            at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));

//            m_lora_ack_received = true;
//            NRF_LOG_INFO("Ack received");
//        } 
//        break;



//        default:
//            break;
//    }
//}

/**
 * @brief  List of all supported AT Commands
 */
static at_command_t at_commands[] =
{
    AT_COMMAND_DEF (AT_RESET,       at_reset,               at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_ECHO,        at_echo_set,            at_echo_read,           at_error_not_supported),
    AT_COMMAND_DEF (AT_INFO,        at_error_not_supported, at_info_read,           at_error_not_supported),
    AT_COMMAND_DEF (AT_DEVEUI,      at_deveui_set,          at_deveui_read,         at_deveui_test),
    AT_COMMAND_DEF (AT_APPEUI,      at_appeui_set,          at_appeui_read,         at_appeui_test),
    AT_COMMAND_DEF (AT_JOINEUI,     at_joineui_set,         at_joineui_read,        at_joineui_test),
    AT_COMMAND_DEF (AT_APPKEY,      at_appkey_set,          at_error_not_supported, at_appkey_test),
    AT_COMMAND_DEF (AT_GENAPPKEY,   at_error_not_supported, at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_NWKKEY,      at_nwkkey_set,          at_error_not_supported, at_nwkkey_test),
    AT_COMMAND_DEF (AT_FNWKSINTKEY, at_fnwksintkey_set,     at_error_not_supported, at_fnwksintkey_test),
    AT_COMMAND_DEF (AT_SNWKSINTKEY, at_snwksintkey_set,     at_error_not_supported, at_snwksintkey_test),
    AT_COMMAND_DEF (AT_NWKSENCKEY,  at_nwksenckey_set,      at_error_not_supported, at_nwksenckey_test),
    AT_COMMAND_DEF (AT_APPSKEY,     at_appskey_set,         at_error_not_supported, at_appskey_test),
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
    AT_COMMAND_DEF (AT_DUTYC,       at_dutyc_set,           at_dutyc_read,          at_error_not_supported),
    AT_COMMAND_DEF (AT_CHANNEL,     at_channel_set,         at_channel_read,        at_error_not_supported),
    AT_COMMAND_DEF (AT_CERTIF,      at_certif_set,          at_error_not_supported, at_error_not_supported),
    AT_COMMAND_DEF (AT_CTXRST,      at_ctxrst_set,          at_error_not_supported, at_error_not_supported),
};

uint32_t at_manager_execute()
{
    at_error_code_t err_code;
    at_command_t *current_at_command;
    uint8_t *p_data;

    // Process the LoRaMac events
    LmHandlerProcess();

    UplinkProcess();

    // Process AT command
    if (m_at_command_ready)
    {
        // If enabled echo command
        if (m_echo)
        {
            sprintf(m_tx_buffer, "%s",  m_rx_buffer);
            m_tx_buffer[strcspn(m_tx_buffer, "\r\n")] = 0; 
            strcat(m_tx_buffer, "\r\n");
            at_hal_transport_tx_pkt_send(m_tx_buffer, strlen(m_tx_buffer));
        }

        // Verify that command starts with AT
        if ((m_rx_buffer[0] != 'A') || (m_rx_buffer[1] != 'T'))
        {
            err_code = AT_ERROR_UNKNOWN_CMD;
        }
        // Search AT commands in the list and execute correponding functio
        else
        {
            err_code = AT_ERROR_UNKNOWN_CMD;
            p_data = m_rx_buffer+2;

            if (p_data[0] == '\r' || p_data[0] == '\n')
            {
                err_code = AT_OK;
            }
            else
            {
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
        }

        // Send final response
        final_response_send(err_code);

        memset1(m_rx_buffer, 0, 128);
        m_at_command_ready = false;
    }

    if (IsMacProcessPending == 1)
    {
        // Clear flag and prevent MCU to go into low power modes.
        IsMacProcessPending = 0;
        return 1;
    }

    return 0;
}

uint32_t at_manager_init()
{
    uint32_t err_code;
    LmHandlerErrorStatus_t lm_err_code;

    // Initialize NVM
    NvmDataMgmtInit();

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);

    // Initialize LoRaWan
    lm_err_code = LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams);
    if (lm_err_code != LORAMAC_HANDLER_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    // Fill mibReq structure
    mibReq.Type = MIB_DEV_EUI;
    LoRaMacMibGetRequestConfirm(&mibReq);

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError(20);

    // The LoRa-Alliance Compliance protocol package should always be initialized and activated.
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);
    
    // Initialize transport
    err_code = at_hal_transport_open(at_conn_hal_transport_event_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return err_code;
}