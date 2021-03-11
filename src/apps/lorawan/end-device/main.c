 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   LoRaWAN End-device project. 
 *          The is inspired from the periodic-uplink-lpp project from stackforce 
 *          https://github.com/Lora-net/LoRaMac-node
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

// Standards
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// nRF
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"

// LoRa
#include "board.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "Commissioning.h"
#include "RegionCommon.h"
#include "NvmDataMgmt.h"

//logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define FIRMWARE_VERSION                        0x03010400 // 3.1.4.0
#define SCHED_MAX_EVENT_DATA_SIZE               APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                        60                              /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DEFAULT_CLASS                   CLASS_A                         /**< LoRaWAN default end-device class. */
#define LORAWAN_ADR_STATE                       LORAMAC_HANDLER_ADR_ON          /**< LoRaWan Default Adaptive Data Rate. */
#define LORAWAN_DEFAULT_DATARATE                DR_0                            /**< LoRaWan Default Data Rate. */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE     LORAMAC_HANDLER_UNCONFIRMED_MSG /**< LoRaWan Default confirmed messages. */
#define LORAWAN_DUTYCYCLE_ON                    true                            /**< LoRaWAN ETSI duty cycle control enable/disable. */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE        242                             /**< LoRaWAN User application data buffer size. */
#define APP_TX_DUTYCYCLE                        5000                            /**< Data transmission duty cycle in ms. */
#define APP_TX_DUTYCYCLE_RND                    1000                            /**< Random delay in ms for application data transmission duty cycle. */

typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

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

static char strlog1[64];
static char strlog2[64];
static char strlog3[64];
static TimerEvent_t TxTimer;                                                    /**< Timer to handle the application data transmission duty cycle. */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];                 /**< User application data. */
static volatile uint8_t IsMacProcessPending = 0;                               
static volatile uint8_t IsTxFramePending = 0;
static volatile uint32_t TxPeriodicity = 0;
static LmHandlerAppData_t AppData =                                              /**< User application data structure. */
{
    .Buffer = AppDataBuffer,
    .BufferSize = 0,
    .Port = 0,
};

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
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/**
 * MAC status strings
 */
const char* MacStatusStrings[] =
{
    "OK",                            // LORAMAC_STATUS_OK
    "Busy",                          // LORAMAC_STATUS_BUSY
    "Service unknown",               // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",             // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",             // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",              // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid", // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",             // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                  // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",          // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",              // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",         // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",              // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",         // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",     // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",    // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",         // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                  // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",            // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",             // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                  // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",           // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Multicast group undefined",     // LORAMAC_STATUS_MC_GROUP_UNDEFINED
    "Unknown error",                 // LORAMAC_STATUS_ERROR
};

/**
 * RX Slot strings
 */
const char* RxSlotStrings[] =
{ 
    "1",                            // RX_SLOT_WIN_1
    "2",                            // RX_SLOT_WIN_2
    "CLASS C",                      // RX_SLOT_WIN_CLASS_C
    "CLASS C MULTICAST",            // RX_SLOT_WIN_CLASS_C_MULTICAST
    "CLASS B PING SLOT",            // RX_SLOT_WIN_CLASS_B_PING_SLOT
    "CLASS B PING MULTICAST SLOT",  // RX_SLOT_WIN_CLASS_B_MULTICAST_SLOT
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

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    if( state == LORAMAC_HANDLER_NVM_STORE )
    {
        NRF_LOG_INFO("Data stored on NVM: %d bytes", size);
    }
    else
    {
        NRF_LOG_INFO("Data restored from NVM: %d bytes", size);
    }
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    uint8_t temp[4];

    sprintf(strlog1, "DevEui=%02X", params->DevEui[0]);
    for (int i = 1; i < 8; i++)
    {
        sprintf(temp, ":%02X", params->DevEui[i]);
        strcat(strlog1, temp);
    }  
    NRF_LOG_INFO("%s", (uint32_t)strlog1);

    sprintf(strlog2, "JoinEui=%02X", params->JoinEui[0]);
    for (int i = 1; i < 8; i++)
    {
        sprintf(temp, ":%02X", params->JoinEui[i]);
        strcat(strlog2, temp);
    }  
    NRF_LOG_INFO("%s", (uint32_t)strlog2);

    sprintf(strlog3, "SePin=%02X", params->SePin[0]);
    for (int i = 1; i < 4; i++)
    {
        sprintf(temp, ":%02X", params->SePin[i]);
        strcat(strlog3, temp);
    }  
    NRF_LOG_INFO("%s", (uint32_t)strlog3);
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    switch (mcpsReq->Type)
    {
        case MCPS_CONFIRMED:
        {
            NRF_LOG_INFO("MCPS-Request: MCPS_CONFIRMED, %s", MacStatusStrings[status]);
            break;
        }
        case MCPS_UNCONFIRMED:
        {
            NRF_LOG_INFO("MCPS-Request: MCPS_UNCONFIRMED,  %s", MacStatusStrings[status]);
            break;
        }
        case MCPS_PROPRIETARY:
        {
            NRF_LOG_INFO("MCPS-Request: MCPS_PROPRIETARY, %s", MacStatusStrings[status]);
            break;
        }
        default:
        {
            NRF_LOG_INFO("MCPS-Request: MLME_UNKNOWN", MacStatusStrings[status]);
            break;
        }
    }
    if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED)
    {
         NRF_LOG_INFO("Next Tx in  : %lu [ms]", nextTxIn );
    }
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    switch (mlmeReq->Type)
    {
        case MLME_JOIN:
        {
            NRF_LOG_INFO("MLME-Request: MLME_JOIN, %s", MacStatusStrings[status]);
            break;
        }
        case MLME_LINK_CHECK:
        {
            NRF_LOG_INFO("MLME-Request: MLME_LINK_CHECK,  %s", MacStatusStrings[status]);
            break;
        }
        case MLME_DEVICE_TIME:
        {
            NRF_LOG_INFO("MLME-Request: MLME_DEVICE_TIME, %s", MacStatusStrings[status]);
            break;
        }
        case MLME_TXCW:
        {
            NRF_LOG_INFO("MLME-Request: MLME_TXCW, %s", MacStatusStrings[status]);
            break;
        }
        default:
        {
            NRF_LOG_INFO("MLME-Request: MLME_UNKNOWN", MacStatusStrings[status]);
            break;
        }
    }
    if (status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED)
    {
         NRF_LOG_INFO("Next Tx in  : %lu [ms]", nextTxIn );
    }
}

static void OnJoinRequest(LmHandlerJoinParams_t* params)
{
    if(params->Status == LORAMAC_HANDLER_ERROR)
    {
        NRF_LOG_INFO("Network not joined, Retrying...");
        LmHandlerJoin( );
    }
    else
    {
        NRF_LOG_INFO("Network joined with DevAddr=%08X", params->CommissioningParams->DevAddr);
        LmHandlerRequestClass(LORAWAN_DEFAULT_CLASS);
    }
}

static void OnTxData(LmHandlerTxParams_t* params)
{
    NRF_LOG_DEBUG("Uplink counter   : %u",      params->UplinkCounter);
    NRF_LOG_DEBUG("Uplink Data rate : DR_%d",   params->Datarate);
    NRF_LOG_DEBUG("Uplink TX power  : %d",      params->TxPower );

    MibRequestConfirm_t mibReq;
    mibReq.Type  = MIB_CHANNELS;
    if (LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK)
    {
        NRF_LOG_DEBUG("Uplink frequency: %lu", mibReq.Param.ChannelList[params->Channel].Frequency);
    }
}

static void OnRxData(LmHandlerAppData_t* appData, LmHandlerRxParams_t* params)
{
    NRF_LOG_DEBUG("Downlink counter : %u",    params->DownlinkCounter);
    NRF_LOG_DEBUG("Downlink slot    : %s",    RxSlotStrings[params->RxSlot]);
    NRF_LOG_DEBUG("Downlink RSSI    : %d",    params->Rssi);
    NRF_LOG_DEBUG("Downlink SNR     : %d",    params->Snr);
}

static void OnClassChange(DeviceClass_t deviceClass)
{
    NRF_LOG_INFO("Switch to Class %c done.", "ABC"[deviceClass]);

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
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            NRF_LOG_INFO("Beacon received");
            NRF_LOG_DEBUG( "Timestamp       : %lu", params->Info.Time.Seconds);
            NRF_LOG_DEBUG( "GW Descriptor   : %d", params->Info.GwSpecific.InfoDesc);
            NRF_LOG_DEBUG( "GW Info         : %X-%X-%X-%X-%X-%X",   params->Info.GwSpecific.Info[0],
                                                                    params->Info.GwSpecific.Info[1],
                                                                    params->Info.GwSpecific.Info[2],
                                                                    params->Info.GwSpecific.Info[3],
                                                                    params->Info.GwSpecific.Info[4],
                                                                    params->Info.GwSpecific.Info[5]);
            NRF_LOG_DEBUG( "Frequncy        : %lu", params->Info.Frequency);
            NRF_LOG_DEBUG( "Data rate       : DR_%d", params->Info.Datarate);
            NRF_LOG_DEBUG( "RSSI            : %d", params->Info.Rssi);
            NRF_LOG_DEBUG( "SNR             : %d", params->Info.Snr);
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        {
            NRF_LOG_INFO("Beacon Lost");
            break;
        }
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            NRF_LOG_INFO("Beacon not received");
            break;
        }
        case LORAMAC_HANDLER_BEACON_ACQUIRING:
        {
            NRF_LOG_INFO("Beacon acquiring");
            break;
        }
        default:
        {
            break;
        }
    }
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
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent(void* context)
{
    TimerStop(&TxTimer);

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue(&TxTimer, TxPeriodicity);
    TimerStart(&TxTimer);
}

static void StartTxProcess(LmHandlerTxEvents_t txEvent)
{
    switch (txEvent)
    {
        default:
            // Intentional fall through
        case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit(&TxTimer, OnTxTimerEvent);
            TimerSetValue(&TxTimer, TxPeriodicity);
            OnTxTimerEvent(NULL);
        }
        break;
        case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame(void)
{
    if (LmHandlerIsBusy() == true)
    {
        return;
    }

    uint8_t msg[] = "Hello world!";
    AppData.Port = 2;
    strcpy(AppData.Buffer, msg);
    AppData.BufferSize = sizeof(msg);

    LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed);
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

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;
	
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for application main entry.
 */
int main(void)
{	
    uint32_t err_code;
    LmHandlerErrorStatus_t lm_Error_code;
	
    // Initialize logs.
    log_init();
    NRF_LOG_INFO("LoRaWan end-device started.");
	
    // Initialize clocks & power
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_pwr_mgmt_init();

    // Enable nRF52 DCDC
    NRF_POWER->DCDCEN = 1;

    // Initialize Scheduler and timer
    timers_init();

    // Initialize NVM
    NvmDataMgmtInit();
  		
    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
	
    // Initialize LoRaWan
    lm_Error_code = LmHandlerInit(&LmHandlerCallbacks, &LmHandlerParams);
    APP_ERROR_CHECK_BOOL(lm_Error_code == LORAMAC_HANDLER_SUCCESS);

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError(20);

    // The LoRa-Alliance Compliance protocol package should always be initialized and activated.
    LmHandlerPackageRegister(PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams);

    // Start Join procedure
    LmHandlerJoin();

    StartTxProcess(LORAMAC_HANDLER_TX_ON_TIMER);
		
    // Enter main loop.
    for (;;)
    {
        // Process the LoRaMac events
        LmHandlerProcess();

        // Process application uplinks management
        UplinkProcess( );

        // Process events managed by the scheduler
        app_sched_execute();

        if (IsMacProcessPending == 1)
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            if (NRF_LOG_PROCESS() == false)
            {
                nrf_pwr_mgmt_run();
            }	
        }
    }
}

