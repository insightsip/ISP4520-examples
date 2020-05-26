/******************************************************************************
 * @file LoRaMacHelper.c
 * @author  Insight SiP
 * @brief LoRaMacHelper implementation file.
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
 
#include "LoRaMacHelper.h"
#include "NvmCtxMgmt.h"
#include "nrf_delay.h"

//logs
#define NRF_LOG_MODULE_NAME lmh
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

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
 * MAC event info status strings.
 */
const char* EventInfoStatusStrings[] =
{ 
    "OK",                            // LORAMAC_EVENT_INFO_STATUS_OK
    "Error",                         // LORAMAC_EVENT_INFO_STATUS_ERROR
    "Tx timeout",                    // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT
    "Rx 1 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
    "Rx 2 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
    "Rx1 error",                     // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
    "Rx2 error",                     // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
    "Join failed",                   // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL
    "Downlink repeated",             // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED
    "Tx DR payload size error",      // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR
    "Downlink too many frames loss", // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS
    "Address fail",                  // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL
    "MIC fail",                      // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL
    "Multicast fail",                // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL
    "Beacon locked",                 // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED
    "Beacon lost",                   // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST
    "Beacon not found"               // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND
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

#define LMH_VERIFY_SUCCESS(err_code)        \
do                                          \
{                                           \
    if (!(err_code == LORAMAC_STATUS_OK))   \
    {                                       \
        return err_code;                    \
    }                                       \
} while (0)

#if defined (REGION_EU868)
#include "LoRaMacTest.h"
#define LORAWAN_DUTYCYCLE_ON  true                /**< LoRaWAN ETSI duty cycle control enable/disable. Please note that ETSI mandates duty cycled transmissions. Use only for test purposes */
#endif
#define LORAWAN_ADR_ON        1                   /**< LoRaWAN Adaptive Data Rate enabled (the end-device should be static here). */
#define LORAWAN_ADR_OFF       0                   /**< LoRaWAN Adaptive Data Rate disabled. */

static char strlog1[64];
static char strlog2[64];
static char strlog3[64];

static uint8_t m_device_eui[]       = LORAWAN_DEVICE_EUI;       /**< Mote device IEEE EUI */
static uint8_t m_join_eui[]         = LORAWAN_JOIN_EUI;         /**< App/Join server IEEE EUI */
static uint8_t m_nwk_key[]          = LORAWAN_NWK_KEY;          /**< Network root key */
static uint8_t m_f_nwk_s_int_key[]  = LORAWAN_F_NWK_S_INT_KEY;  /**< Forwarding Network session integrity key */
static uint8_t m_s_nwk_s_int_key[]  = LORAWAN_S_NWK_S_INT_KEY;  /**< Serving Network session integrity key */
static uint8_t m_nwk_s_enc_key[]    = LORAWAN_NWK_S_ENC_KEY;    /**< Network session encryption key */
static uint8_t m_gen_app_key[]      = LORAWAN_GEN_APP_KEY;      /**< Application root key - Used to derive Multicast keys on 1.0.x devices */
static uint8_t m_app_key[]          = LORAWAN_APP_KEY;          /**< Application root key */
static uint8_t m_app_s_key[]        = LORAWAN_APP_S_KEY;        /**< Application session key */
static uint32_t m_device_address    = LORAWAN_DEVICE_ADDRESS;   /**< End-device address */
static uint32_t m_network_id        = LORAWAN_NETWORK_ID;       /**< Network ID */

static LoRaMacPrimitives_t LoRaMacPrimitives;                   /**< LoRaMAC events variable */
static LoRaMacCallback_t LoRaMacCallbacks;                      /**< LoRaMAC callback variable */
static MibRequestConfirm_t mibReq;                              /**< LoRaMAC MIB-RequestConfirm variable */

static bool m_is_classB_switch_pending = false;                 /**< Indicates if a switch to Class B operation is pending */
static lmh_callback_t *m_callbacks;                             /**< Callback functions pointer */
static lmh_compliance_test_t m_compliance_test;                 /**< LoRaWAN compliance tests data */
static lmh_param_t m_param = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, LORAWAN_DEFAULT_TX_POWER};
static bool m_duty_cycle;      

static MlmeReqJoin_t m_certif_join_backup;                      /**< Last used join parameters, used to restore after compliance test */
static bool m_certif_adr_backup;                                /**< Last used adr, used to restore after compliance test */

static TimerEvent_t ComplianceTestTxNextPacketTimer;            /**< Timer to handle the tx packets for compliance test */

                     
static bool compliance_test_tx(void)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
  
    if (m_compliance_test.link_check == true)
    {
        m_compliance_test.link_check = false;
        m_compliance_test.data_buffer_size = 3;
        m_compliance_test.data_buffer[0] = 5;
        m_compliance_test.data_buffer[1] = m_compliance_test.demod_margin;
        m_compliance_test.data_buffer[2] = m_compliance_test.nb_gateways;
        m_compliance_test.state = 1;
    }
    else
    {
        switch (m_compliance_test.state)
        {
            case 4:
                m_compliance_test.state = 1;
                break;

            case 1:
                m_compliance_test.data_buffer_size = 2;
                m_compliance_test.data_buffer[0] = m_compliance_test.downlink_counter >> 8;
                m_compliance_test.data_buffer[1] = m_compliance_test.downlink_counter;
                break;
        }
    }

    lmh_app_tx_data_t app_data =
    {
        .buffer = m_compliance_test.data_buffer,
        .buffsize = m_compliance_test.data_buffer_size,
        .port = LORAWAN_CERTIF_PORT,
        .confirmed = m_compliance_test.is_tx_confirmed,
        .nb_trials = 8,
    };
    lmh_send(&app_data);

    // certification test on-going
    TimerStart(&ComplianceTestTxNextPacketTimer);	
	
    return true;
}

/**@brief Function executed on TxNextPacket Timeout event
 */
static void OnComplianceTestTxNextPacketTimerEvent(void* context)
{
    compliance_test_tx();
}


/**@brief MCPS-Confirm event function
 *
 * @param[in] McpsConfirm - Pointer to the confirm structure, containing confirm attributes.
 */
static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
    NRF_LOG_DEBUG("McpsConfirm event, status: %s", EventInfoStatusStrings[mcpsConfirm->Status]);

    if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
    {
        switch (mcpsConfirm->McpsRequest)
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }

            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }

            case MCPS_PROPRIETARY:
            {
                break;
            }

            default:
                break;
        }
    
        NRF_LOG_DEBUG("Uplink counter   : %u",      mcpsConfirm->UpLinkCounter);
        NRF_LOG_DEBUG("Uplink Data rate : DR_%d",   mcpsConfirm->Datarate);
        NRF_LOG_DEBUG("Uplink TX power  : %d",      mcpsConfirm->TxPower );
    }
}

/**@brief MCPS-Indication event function
 *
 * @param[in] mcpsIndication	Pointer to the indication structure, containing indication attributes.
 */
static void McpsIndication(McpsIndication_t *mcpsIndication)
{
    NRF_LOG_DEBUG("McpsIndication event, status: %s", EventInfoStatusStrings[mcpsIndication->Status]);
	
    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
        return;
    }

    // Check Multicast
    // Check Port
    // Check Datarate

    // Check FramePending
    if (mcpsIndication->FramePending == true)
    {   
        lmh_device_class_t current_class;
        lmh_class_get(&current_class);
        if (current_class == CLASS_A)
        {
            // The server signals that it has pending data to be sent.
            // We schedule an uplink as soon as possible to flush the server.

            // Send an empty message
            lmh_app_tx_data_t app_data =
            {
                .buffer = NULL,
                .buffsize = 0,
                .port = 0,
                .confirmed = LMH_UNCONFIRMED_MSG,
                .nb_trials = 8,
            };
            lmh_send(&app_data);
        }
    }

    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if (m_compliance_test.running == true)
    {
	 m_compliance_test.downlink_counter++;
    }

    if (mcpsIndication->AckReceived == true)
    {
        m_callbacks->lmh_evt_handler(LHM_EVT_RX_ACK, NULL);
    }

    if (mcpsIndication->RxData == true)
    {
        switch (mcpsIndication->Port)
        {
            case LORAWAN_CERTIF_PORT:
            {
                // Compliance not started yet, start it
                if (m_compliance_test.running == false)
                {
                    // Check compliance test enable command (i)
                    if (    (mcpsIndication->BufferSize == 4) 		&&
                            (mcpsIndication->Buffer[0] == 0x01) 	&&
                            (mcpsIndication->Buffer[1] == 0x01) 	&&
                            (mcpsIndication->Buffer[2] == 0x01) 	&&
                            (mcpsIndication->Buffer[3] == 0x01) )
                    {
                        m_compliance_test.is_tx_confirmed = LMH_UNCONFIRMED_MSG;
                        m_compliance_test.data_buffer_size = 2;
                        m_compliance_test.downlink_counter = 0;
                        m_compliance_test.link_check = false;
                        m_compliance_test.demod_margin = 0;
                        m_compliance_test.nb_gateways = 0;
                        m_compliance_test.running = true;
                        m_compliance_test.state = 1;

                        // Get current ADR, to be restored after compliance tests
                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        LoRaMacMibGetRequestConfirm(&mibReq);
                        m_certif_adr_backup = mibReq.Param.AdrEnable;
					  
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = true;
                        LoRaMacMibSetRequestConfirm(&mibReq);

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
                        LoRaMacTestSetDutyCycleOn(false);
#endif

                        TimerInit(&ComplianceTestTxNextPacketTimer, OnComplianceTestTxNextPacketTimerEvent);
                        TimerSetValue(&ComplianceTestTxNextPacketTimer,  LORAWAN_CERTIF_DELAY); 
					
                        // confirm test mode activation
                        compliance_test_tx();			
                    }
                }
                // Compliance is started, check which stage we are at and take action
                else
                {
                    m_compliance_test.state = mcpsIndication->Buffer[0];
                    switch (m_compliance_test.state)
                    {
                        case 0: // Check compliance test disable command (ii)
                        {
                            MibRequestConfirm_t mibReq;

                            TimerStop( &ComplianceTestTxNextPacketTimer );

                            // Disable compliance test mode and reset the downlink counter.
                            m_compliance_test.downlink_counter = 0;
                            m_compliance_test.running = false;

                            // Restore previous ADR seeting
                            mibReq.Type = MIB_ADR;
                            mibReq.Param.AdrEnable = m_certif_adr_backup;
                            LoRaMacMibSetRequestConfirm(&mibReq);

                            // Enable duty cycle
#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
                            LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif
                        } 
                        break;

                        case 1: // (iii, iv)
                        {
                            m_compliance_test.data_buffer_size = 2; 
                        } break;

                        case 2: // Enable confirmed messages (v)
                        {
                            m_compliance_test.is_tx_confirmed = LMH_CONFIRMED_MSG;
                            m_compliance_test.state = 1;
                        } break;

                        case 3:  // Disable confirmed messages (vi)
                        {
                            m_compliance_test.is_tx_confirmed = LMH_UNCONFIRMED_MSG;
                            m_compliance_test.state = 1;
                        } break;
						
                        case 4: // (vii)
                        {
                            m_compliance_test.data_buffer_size = mcpsIndication->BufferSize;
                            m_compliance_test.data_buffer[0] = 4;
                            for( uint8_t i = 1; i < MIN(m_compliance_test.data_buffer_size, LORAWAN_APP_DATA_MAX_SIZE); i++ )
                            {
                                m_compliance_test.data_buffer[i] = mcpsIndication->Buffer[i] + 1;
                            }
                        } break;

                        case 5: // (viii)
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_LINK_CHECK;
                            LoRaMacMlmeRequest(&mlmeReq);
                        } break;

                        case 6: // (ix)
                        {
                            MlmeReq_t mlmeReq;

                            // Disable TestMode and revert back to normal operation
                            // Disable compliance test mode and reset the downlink counter.
                            m_compliance_test.is_tx_confirmed = LORAWAN_CONFIRMED_MSG_ON;
                            m_compliance_test.downlink_counter = 0;
                            m_compliance_test.running = false;

                            // Restore previous ADR setting
                            MibRequestConfirm_t mibReq;
                            mibReq.Type = MIB_ADR;
                            mibReq.Param.AdrEnable = m_certif_adr_backup;
                            LoRaMacMibSetRequestConfirm(&mibReq);

                            // Enable duty cycle
#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
                            LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif
                            mlmeReq.Type = MLME_JOIN;
                            mlmeReq.Req.Join = m_certif_join_backup;
                            LoRaMacMlmeRequest(&mlmeReq);
                        } break;

                        case 7: // (x)
                        {
                            if (mcpsIndication->BufferSize == 3)
                            {
                                    MlmeReq_t mlmeReq;
                                    mlmeReq.Type = MLME_TXCW;
                                    mlmeReq.Req.TxCw.Timeout = (uint16_t)( (mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2] );
                                    LoRaMacMlmeRequest(&mlmeReq);
                            }
                            else if (mcpsIndication->BufferSize == 7)
                            {
                                MlmeReq_t mlmeReq;
                                mlmeReq.Type = MLME_TXCW_1;
                                mlmeReq.Req.TxCw.Timeout = (uint16_t)( (mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2]);
                                mlmeReq.Req.TxCw.Frequency = (uint32_t)( (mcpsIndication->Buffer[3] << 16) | (mcpsIndication->Buffer[4] << 8) | mcpsIndication->Buffer[5] ) * 100;
                                mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                                LoRaMacMlmeRequest(&mlmeReq);
                            }
                            m_compliance_test.state = 1;
                        } break;

                        case 8: // Send DeviceTimeReq
                        {
                            MlmeReq_t mlmeReq;

                            mlmeReq.Type = MLME_DEVICE_TIME;
                            LoRaMacMlmeRequest( &mlmeReq );
                        } break;

                        case 9: // Switch end device Class
                        {
                            MibRequestConfirm_t mibReq;

                            mibReq.Type = MIB_DEVICE_CLASS;
                            // CLASS_A = 0, CLASS_B = 1, CLASS_C = 2
                            mibReq.Param.Class = ( DeviceClass_t )mcpsIndication->Buffer[1];;
                            LoRaMacMibSetRequestConfirm( &mibReq );
                        } break;

                        case 10: // Send PingSlotInfoReq
                        {
                            MlmeReq_t mlmeReq;

                            mlmeReq.Type = MLME_PING_SLOT_INFO;
                            mlmeReq.Req.PingSlotInfo.PingSlot.Value = mcpsIndication->Buffer[1];
                            LoRaMacMlmeRequest( &mlmeReq );
                        } break;
                        
                        case 11: // Send BeaconTimingReq
                        {
                            MlmeReq_t mlmeReq;

                            mlmeReq.Type = MLME_BEACON_TIMING;
                            LoRaMacMlmeRequest( &mlmeReq );
                        } break;

                        default:
                            break;
                    }
                }  
                  
                if (m_compliance_test.running == false)
                {
                    // cerification test stops
                    TimerStop(&ComplianceTestTxNextPacketTimer);
                }  
            }
            break;
			
            default:
            {
                lmh_app_rx_data_t app_data;
                app_data.port = mcpsIndication->Port;
                app_data.buffsize = mcpsIndication->BufferSize;
                app_data.buffer = mcpsIndication->Buffer;
                app_data.rssi = mcpsIndication->Rssi;
                app_data.snr = mcpsIndication->Snr;
                m_callbacks->lmh_RxData(&app_data);
            }
            break;
        }
    }

    NRF_LOG_DEBUG("Downlink counter : %u",    mcpsIndication->DownLinkCounter);
    NRF_LOG_DEBUG("Downlink slot    : %s",    RxSlotStrings[mcpsIndication->RxSlot]);
    NRF_LOG_DEBUG("Downlink RSSI    : %d",    mcpsIndication->Rssi);
    NRF_LOG_DEBUG("Downlink SNR     : %d",    mcpsIndication->Snr);
}

/**@brief MLME-Confirm event function
 *
 * @param[in] MlmeConfirm	Pointer to the confirm structure, containing confirm attributes.
 */
static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
    NRF_LOG_DEBUG("MlmeConfirm event, status: %s", EventInfoStatusStrings[mlmeConfirm->Status]);

    switch (mlmeConfirm->MlmeRequest)
    {
        case MLME_JOIN:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                // Status is OK, node has joined the network
                m_callbacks->lmh_evt_handler(LHM_EVT_NWK_JOINED, NULL);
            }
        } 
        break;

        case MLME_LINK_CHECK:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                // Check DemodMargin
                // Check NbGateways
                if (m_compliance_test.running == true)
                {
                    m_compliance_test.link_check = true;
                    m_compliance_test.demod_margin = mlmeConfirm->DemodMargin;
                    m_compliance_test.nb_gateways = mlmeConfirm->NbGateways;
                }
            }
        } 
        break;

        case MLME_DEVICE_TIME:
        {   
            NRF_LOG_INFO("Received device time");
            if( m_is_classB_switch_pending == true )
            {
                lmh_beacon_req();
            }
        } 
        break;

        case MLME_BEACON_TIMING:
        {
            // deprecated in v1.0.3
            // The device may use DeviceTimeReq&Ans commands as a substitute
        } 
        break;

        case MLME_BEACON_ACQUISITION:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {  
                NRF_LOG_INFO("Beacon acquired");
                // Beacon successfully acquired. Now request ping slot
                lmh_ping_slot_req(0);
            }
            else
            {   
                NRF_LOG_INFO("Beacon not acquired");
                // Failed to acquire beacon. Start again by requesting device time
                lmh_device_time_req();
            }
            
        }
        break;

        case MLME_PING_SLOT_INFO:
        {
            if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            {
                MibRequestConfirm_t mibReq;

                // Class B is now activated
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_B;
                LoRaMacMibSetRequestConfirm(&mibReq);

                // Notify upper layer
                int data = CLASS_B;
                m_callbacks->lmh_evt_handler(LHM_EVT_CLASS_CHANGED, &data);
                m_is_classB_switch_pending = false;
            }
            else
            {
                // Failed to get ping slot info. Start again by requesting ping slot
                lmh_ping_slot_req(0);
            }
        }
        break;

        default:
            break;
    }
}

/**
 * @brief   MLME-Indication event function
 *
 * @param[in] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication(MlmeIndication_t *mlmeIndication)
{
    if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
    {
        NRF_LOG_DEBUG("MlmeIndication event, status: %s", EventInfoStatusStrings[mlmeIndication->Status]);
    }

    switch (mlmeIndication->MlmeIndication)
    {
        case MLME_SCHEDULE_UPLINK:
        {
            // The MAC signals that we shall provide an uplink as soon as possible
            // Send an empty message
            lmh_app_tx_data_t app_data =
            {
                .buffer = NULL,
                .buffsize = 0,
                .port = 0,
                .confirmed = LMH_UNCONFIRMED_MSG,
                .nb_trials = 8,
            };
            if (m_compliance_test.running == false) lmh_send(&app_data);
        } 
        break;

        case MLME_BEACON_LOST:
        {
            NRF_LOG_INFO("Beacon lost, switching back to class A");

            m_callbacks->lmh_evt_handler(LMH_EVT_BEACON_LOST, NULL);

            // we need to switch to class A
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm(&mibReq);
            int data = CLASS_A;
            m_callbacks->lmh_evt_handler(LHM_EVT_CLASS_CHANGED, &data);
        } 
        break;

         case MLME_BEACON:
        {
            if( mlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
            {
                NRF_LOG_INFO("Beacon locked");
                NRF_LOG_DEBUG( "Timestamp       : %lu", mlmeIndication->BeaconInfo.Time.Seconds);
                NRF_LOG_DEBUG( "GW Descriptor   : %d\r\n", mlmeIndication->BeaconInfo.GwSpecific.InfoDesc);
                NRF_LOG_DEBUG( "GW Info         : %X-%X-%X-%X-%X-%X",   mlmeIndication->BeaconInfo.GwSpecific.Info[0],
                                                                        mlmeIndication->BeaconInfo.GwSpecific.Info[1],
                                                                        mlmeIndication->BeaconInfo.GwSpecific.Info[2],
                                                                        mlmeIndication->BeaconInfo.GwSpecific.Info[3],
                                                                        mlmeIndication->BeaconInfo.GwSpecific.Info[4],
                                                                        mlmeIndication->BeaconInfo.GwSpecific.Info[5]);
                NRF_LOG_DEBUG( "Frequncy        : %lu\r\n", mlmeIndication->BeaconInfo.Frequency);
                NRF_LOG_DEBUG( "Data rate       : DR_%d\r\n", mlmeIndication->BeaconInfo.Datarate);
                NRF_LOG_DEBUG( "RSSI            : %d\r\n", mlmeIndication->BeaconInfo.Rssi);
                NRF_LOG_DEBUG( "SNR             : %d\r\n", mlmeIndication->BeaconInfo.Snr);

                 m_callbacks->lmh_evt_handler(LMH_EVT_BEACON_RECEIVED, NULL);
            }
            else
            {
                NRF_LOG_INFO("Beacon not received");

                m_callbacks->lmh_evt_handler(LMH_EVT_BEACON_NOT_RECEIVED, NULL);
            }
        } 
        break;
            
        default:
            break;
    }
}


lmh_error_code_t lmh_init (lmh_callback_t *callbacks)
{
    LoRaMacStatus_t error_status;

    m_callbacks = callbacks;

    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
    LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
    LoRaMacCallbacks.GetBatteryLevel = m_callbacks->BoardGetBatteryLevel;
    LoRaMacCallbacks.GetTemperatureLevel = NULL;
    LoRaMacCallbacks.NvmContextChange = NvmCtxMgmtEvent;
    LoRaMacCallbacks.MacProcessNotify = NULL;

    m_is_classB_switch_pending = false; 
	
    LoRaMacRegion_t region;
#if defined (REGION_AS923)
    region = LORAMAC_REGION_AS923;
#elif defined (REGION_AU915)
    region = LORAMAC_REGION_AU915;
#elif defined (REGION_CN470)
    region = LORAMAC_REGION_CN470;
#elif defined (REGION_CN779)
    region = LORAMAC_REGION_CN779;
#elif defined (REGION_EU433)
    region = LORAMAC_REGION_EU433;
#elif defined (REGION_IN865)
    region = LORAMAC_REGION_IN865;
#elif defined (REGION_EU868)
    region = LORAMAC_REGION_EU868;
#elif defined (REGION_KR920)
    region = LORAMAC_REGION_KR920;
#elif defined (REGION_US915)
    region = LORAMAC_REGION_US915;
#elif defined (REGION_US915_HYBRID)
    region = LORAMAC_REGION_US915_HYBRID;
#else
     #error "Please define a region in the compiler options."
#endif	

    // Initialize LoRaMAc stack
    error_status = LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, region);
    if (error_status != LORAMAC_STATUS_OK)
    {
        NRF_LOG_ERROR("Error in LoRaMacInitialization: %s", MacStatusStrings[error_status]);
        return error_status;
    }

    // Try to restore from NVM and query the mac if possible.
    if (NvmCtxMgmtRestore() == NVMCTXMGMT_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("LoRaWan context restored");
    }
    else
    {
        // Tell the MAC layer which network server version are we connecting too.
        mibReq.Type = MIB_ABP_LORAWAN_VERSION;
        mibReq.Param.AbpLrWanVersion.Value = ABP_ACTIVATION_LRWAN_VERSION;
        LoRaMacMibSetRequestConfirm( &mibReq );
        
        // For v1.0.x ABP devices
        mibReq.Type = MIB_GEN_APP_KEY;
        mibReq.Param.GenAppKey = m_gen_app_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        // For v1.1.x ABP devices
        mibReq.Type = MIB_APP_KEY;
        mibReq.Param.AppKey = m_app_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        // Set network key
        mibReq.Type = MIB_NWK_KEY;
        mibReq.Param.NwkKey = m_nwk_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        // Initialize Device EUI if not already defined in Commissioning.h
        if (( m_device_eui[0] == 0 ) && ( m_device_eui[1] == 0) &&
           ( m_device_eui[2] == 0 ) && ( m_device_eui[3] == 0 ) &&
           ( m_device_eui[4] == 0 ) && ( m_device_eui[5] == 0 ) &&
           ( m_device_eui[6] == 0 ) && ( m_device_eui[7] == 0 ))
        {
            m_callbacks->BoardGetUniqueId(m_device_eui);
        }

        // Set device EUI
        mibReq.Type = MIB_DEV_EUI;
        mibReq.Param.DevEui = m_device_eui;
        LoRaMacMibSetRequestConfirm( &mibReq );
        sprintf(strlog1, "DevEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", m_device_eui[0], m_device_eui[1], m_device_eui[2], m_device_eui[3], m_device_eui[4], m_device_eui[5], m_device_eui[6], m_device_eui[7]);
        NRF_LOG_INFO("%s", (uint32_t)strlog1);

        // Set Join EUI
        mibReq.Type = MIB_JOIN_EUI;
        mibReq.Param.JoinEui = m_join_eui;
        LoRaMacMibSetRequestConfirm( &mibReq );
        sprintf(strlog2, "JoinEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", m_join_eui[0], m_join_eui[1], m_join_eui[2], m_join_eui[3], m_join_eui[4], m_join_eui[5], m_join_eui[6], m_join_eui[7]);
        NRF_LOG_INFO("%s", (uint32_t)strlog2);

#if (OVER_THE_AIR_ACTIVATION != 0)
        NRF_LOG_INFO("OTAA mode"); 

        sprintf(strlog3, "NwkKey=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",                              \
                        m_nwk_key[0], m_nwk_key[1], m_nwk_key[2], m_nwk_key[3], m_nwk_key[4], m_nwk_key[5], m_nwk_key[6], m_nwk_key[7],     \
                        m_nwk_key[8], m_nwk_key[9], m_nwk_key[10], m_nwk_key[11], m_nwk_key[12], m_nwk_key[13], m_nwk_key[14], m_nwk_key[15]);	
        NRF_LOG_INFO ("%s", (uint32_t)strlog3);
    
#else
        // Choose a random device address if not already defined in Commissioning.h
        if (m_device_address == 0)
        {
          // Random seed initialization
          srand1(BoardGetRandomSeed());

          // Choose a random device address
          m_device_address = randr(0, 0x01FFFFFF);
        }

        mibReq.Type = MIB_DEV_ADDR;
        mibReq.Param.DevAddr = m_device_address;
        LoRaMacMibSetRequestConfirm(&mibReq);
        NRF_LOG_INFO("DevAdd=%08X", m_device_address);

        mibReq.Type = MIB_NET_ID;
        mibReq.Param.NetID = LORAWAN_NETWORK_ID;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_F_NWK_S_INT_KEY;
        mibReq.Param.FNwkSIntKey = m_f_nwk_s_int_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_S_NWK_S_INT_KEY;
        mibReq.Param.SNwkSIntKey = m_s_nwk_s_int_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_NWK_S_ENC_KEY;
        mibReq.Param.NwkSEncKey = m_nwk_s_enc_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_APP_S_KEY;
        mibReq.Param.AppSKey = m_app_s_key;
        LoRaMacMibSetRequestConfirm(&mibReq);
#endif
    }

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = m_param.adr_enable;
    LoRaMacMibSetRequestConfirm(&mibReq);
	
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = m_param.tx_power;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = m_param.enable_public_network;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mibReq.Param.SystemMaxRxError = 20;
    LoRaMacMibSetRequestConfirm(&mibReq);
                      
    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class= CLASS_A;
    LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_ANTENNA_GAIN;
    mibReq.Param.AntennaGain = 0;
    LoRaMacMibSetRequestConfirm(&mibReq);
	
#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
    m_duty_cycle = true;
    LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#else
    m_duty_cycle = false;
#endif
	
    LoRaMacStart();

    return LORAMAC_STATUS_OK;
}

void lmh_process(void)
{
    Radio.IrqProcess();

    LoRaMacProcess();
}

void lmh_device_eui_set(uint8_t *dev_eui)
{
    memcpy(m_device_eui, dev_eui, sizeof(m_device_eui));
}

void lmh_device_eui_get(uint8_t *dev_eui)
{
    memcpy(dev_eui, m_device_eui, sizeof(m_device_eui));
}

void lmh_join_eui_set(uint8_t *join_eui)
{
    memcpy(m_join_eui, join_eui, sizeof(m_join_eui));
}

void lmh_join_eui_get(uint8_t *join_eui)
{
    memcpy(join_eui, m_join_eui, sizeof(m_join_eui));
}

void lmh_nwk_key_set(uint8_t *nwk_key)
{
    memcpy(m_nwk_key, nwk_key, sizeof(m_nwk_key));
}

void lmh_nwk_key_get(uint8_t *nwk_key)
{
    memcpy(nwk_key, m_nwk_key, sizeof(m_nwk_key));
}

void lmh_f_nwk_s_int_key_set(uint8_t *f_nwk_s_int_key)
{
    memcpy(m_f_nwk_s_int_key, f_nwk_s_int_key, sizeof(m_f_nwk_s_int_key));
}

void lmh_f_nwk_s_int_key_get(uint8_t *f_nwk_s_int_key)
{
    memcpy(f_nwk_s_int_key, m_f_nwk_s_int_key, sizeof(m_s_nwk_s_int_key));
}

void lmh_s_nwk_s_int_key_set(uint8_t *s_nwk_s_int_key)
{
    memcpy(m_s_nwk_s_int_key, s_nwk_s_int_key, sizeof(m_s_nwk_s_int_key));
}

void lmh_s_nwk_s_int_key_get(uint8_t *s_nwk_s_int_key)
{
    memcpy(s_nwk_s_int_key, m_s_nwk_s_int_key, sizeof(m_s_nwk_s_int_key));
}

void lmh_nwk_s_enc_key_set(uint8_t *nwk_s_enc_key)
{
    memcpy(m_nwk_s_enc_key, nwk_s_enc_key, sizeof(m_nwk_s_enc_key));
}

void lmh_nwk_s_enc_key_get(uint8_t *nwk_s_enc_key)
{
    memcpy(nwk_s_enc_key, m_nwk_s_enc_key, sizeof(m_nwk_s_enc_key));
}

void lmh_gen_app_key_set(uint8_t *gen_app_key)
{
    memcpy(m_gen_app_key, gen_app_key, sizeof(m_gen_app_key));
}

void lmh_gen_app_key_get(uint8_t *gen_app_key)
{
    memcpy(gen_app_key, m_gen_app_key, sizeof(m_gen_app_key));
}

void lmh_app_key_set(uint8_t *app_key)
{
    memcpy(m_app_key, app_key, sizeof(m_app_key));
}

void lmh_app_key_get(uint8_t *app_key)
{
    memcpy(app_key, m_app_key, sizeof(m_app_key));
}

void lmh_app_s_key_set(uint8_t *app_s_key)
{
    memcpy(m_app_s_key, app_s_key, sizeof(m_app_s_key));
}

void lmh_app_s_key_get(uint8_t *app_s_key)
{
    memcpy(app_s_key, m_app_s_key, sizeof(m_app_s_key));
}

void lmh_device_address_set (uint32_t device_address)
{
    m_device_address = device_address;
}

void lmh_device_address_get (uint32_t *device_address)
{
    *device_address = m_device_address;
}

lmh_error_code_t lmh_network_id_set (uint32_t network_id)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = network_id;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    m_network_id = network_id;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_network_id_get (uint32_t *network_id)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NET_ID;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *network_id = mibReq.Param.NetID;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_adaptative_datarate_set(uint8_t adrEnable)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = adrEnable;
    status = LoRaMacMibSetRequestConfirm(&mibReq);	
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_adaptative_datarate_get(uint8_t *adrEnable)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_ADR;
    status = LoRaMacMibGetRequestConfirm(&mibReq);	
    LMH_VERIFY_SUCCESS(status);

    *adrEnable = mibReq.Param.AdrEnable;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_class_set(lmh_device_class_t new_class)
{
    LoRaMacStatus_t status;
    DeviceClass_t current_class;
 
    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm(&mibReq);
    current_class = mibReq.Param.Class;
	
    // attempt to swicth only if class update
    if (current_class != new_class)
    {
        switch (new_class)
        {
            case CLASS_A:
            {
                if( current_class != CLASS_A )
                {
                    mibReq.Param.Class = CLASS_A;
                    status = LoRaMacMibSetRequestConfirm(&mibReq);
                    LMH_VERIFY_SUCCESS(status);
                
                    // Switch is instantaneous, Notify upper layer
                    int data = CLASS_A;
                    m_callbacks->lmh_evt_handler(LHM_EVT_CLASS_CHANGED, &data);
                }
            } 
            break;

            case CLASS_B:
            {
                if (current_class != CLASS_A)
                {
                    return LORAMAC_STATUS_ERROR;
                }
                // Beacon must first be acquired
                status = lmh_device_time_req();
                m_is_classB_switch_pending = true;
            }
            break;

            case CLASS_C:
            {
                if (current_class != CLASS_A)
                {
                    return LORAMAC_STATUS_ERROR;
                }
                // Switch is instantaneous
                mibReq.Param.Class = CLASS_C;
                status = LoRaMacMibSetRequestConfirm(&mibReq);
                LMH_VERIFY_SUCCESS(status);

                // Switch is instantaneous, Notify upper layer
                int data = CLASS_C;
                m_callbacks->lmh_evt_handler(LHM_EVT_CLASS_CHANGED, &data);
            }
            break;

            default:
                break;
        }
        
    }

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_class_get(lmh_device_class_t *current_class)
{
    LoRaMacStatus_t status;
    DeviceClass_t currentClass;
  
    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm (&mibReq);
    if (status != LORAMAC_STATUS_OK)
    {
        return status;
    }
  
    *current_class = mibReq.Param.Class;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_join(bool otaa_mode)
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;
    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.Datarate = m_param.tx_data_rate;
    m_certif_join_backup = mlmeReq.Req.Join;

    // Starts the join procedure
    if (otaa_mode)  // OTAA
    {
        status = LoRaMacMlmeRequest(&mlmeReq);
        NRF_LOG_INFO("Send OTAA Join Request, status: %s", MacStatusStrings[status]);
        LMH_VERIFY_SUCCESS(status);
    }
    else            // ABP
    {
        mibReq.Type = MIB_NET_ID;
        mibReq.Param.NetID = m_network_id;
        status = LoRaMacMibSetRequestConfirm(&mibReq);
        LMH_VERIFY_SUCCESS(status);

        mibReq.Type = MIB_NET_ID;
        mibReq.Param.NetID = LORAWAN_NETWORK_ID;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_DEV_ADDR;
        mibReq.Param.DevAddr = m_device_address;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_F_NWK_S_INT_KEY;
        mibReq.Param.FNwkSIntKey = m_f_nwk_s_int_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_S_NWK_S_INT_KEY;
        mibReq.Param.SNwkSIntKey = m_s_nwk_s_int_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_NWK_S_ENC_KEY;
        mibReq.Param.NwkSEncKey = m_nwk_s_enc_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_APP_S_KEY;
        mibReq.Param.AppSKey = m_app_s_key;
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_NETWORK_ACTIVATION;
        mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
        status = LoRaMacMibSetRequestConfirm(&mibReq);
        LMH_VERIFY_SUCCESS(status);

        m_callbacks->lmh_evt_handler(LHM_EVT_NWK_JOINED, NULL);
    }

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_join_status_get(lmh_activation_type_t *join_status)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NETWORK_ACTIVATION;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *join_status = mibReq.Param.NetworkActivation;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_send(lmh_app_tx_data_t* app_data)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    LoRaMacStatus_t status;
    uint8_t activation_status;

    lmh_join_status_get(&activation_status);
    if (activation_status == ACTIVATION_TYPE_NONE)
    {
        //Not joined
        lmh_join(OVER_THE_AIR_ACTIVATION);  
        return LORAMAC_STATUS_NO_NETWORK_JOINED;
    }

  
    /*if certification test are on going, application data is not sent*/
    if (m_compliance_test.running == true && app_data->port != LORAWAN_CERTIF_PORT)
    {
        return LORAMAC_STATUS_BUSY;
    }
    
    if (LoRaMacQueryTxPossible(app_data->buffsize, &txInfo) != LORAMAC_STATUS_OK)
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = m_param.tx_data_rate;
    }
    else
    {
        if (app_data->confirmed == LMH_UNCONFIRMED_MSG)
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = app_data->port;
            mcpsReq.Req.Unconfirmed.fBufferSize = app_data->buffsize;
            mcpsReq.Req.Unconfirmed.fBuffer = app_data->buffer;
            mcpsReq.Req.Unconfirmed.Datarate = m_param.tx_data_rate;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = app_data->port;
            mcpsReq.Req.Confirmed.fBufferSize = app_data->buffsize;
            mcpsReq.Req.Confirmed.fBuffer = app_data->buffer;
            mcpsReq.Req.Confirmed.NbTrials = app_data->nb_trials;
            mcpsReq.Req.Confirmed.Datarate = m_param.tx_data_rate;
        }
    }

    status = LoRaMacMcpsRequest(&mcpsReq);
    NRF_LOG_INFO("Send data request, status: %s", MacStatusStrings[status]);
    LMH_VERIFY_SUCCESS(status);

    if (NvmCtxMgmtStore() == NVMCTXMGMT_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("CTXS STORED");
    }

    return LORAMAC_STATUS_OK;
}  

lmh_error_code_t lmh_device_time_req(void)
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;

    mlmeReq.Type = MLME_DEVICE_TIME;

    status = LoRaMacMlmeRequest(&mlmeReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_beacon_req(void)
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;

    mlmeReq.Type = MLME_BEACON_ACQUISITION;

    status = LoRaMacMlmeRequest(&mlmeReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_ping_slot_req(uint8_t periodicity)
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;

    mlmeReq.Type = MLME_PING_SLOT_INFO;
    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.Periodicity = periodicity;
    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.RFU = 0;

    status = LoRaMacMlmeRequest(&mlmeReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_tx_power_set(uint8_t tx_power)
{	
    LoRaMacStatus_t status;

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = tx_power;
    LoRaMacMibSetRequestConfirm(&mibReq);	
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_tx_power_get(uint8_t *tx_power)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *tx_power =  mibReq.Param.ChannelsTxPower;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_tx_datarate_set(uint8_t data_rate)
{
    LoRaMacStatus_t status;

    m_param.tx_data_rate = data_rate;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_tx_datarate_get (uint8_t *data_rate)
{
    *data_rate = m_param.tx_data_rate;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_joinAcceptDelay1_set (uint32_t delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    mibReq.Param.JoinAcceptDelay1 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_joinAcceptDelay1_get (uint32_t *delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *delay =  mibReq.Param.JoinAcceptDelay1;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_joinAcceptDelay2_set (uint32_t delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    mibReq.Param.JoinAcceptDelay2 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_joinAcceptDelay2_get (uint32_t *delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *delay =  mibReq.Param.JoinAcceptDelay2;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_publicNetwork_set (uint8_t public_mode)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = public_mode;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_publicNetwork_get (uint8_t *public_mode)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_PUBLIC_NETWORK;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *public_mode =  mibReq.Param.EnablePublicNetwork;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDelay1_set (uint32_t delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RECEIVE_DELAY_1;
    mibReq.Param.ReceiveDelay1 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDelay1_get (uint32_t *delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RECEIVE_DELAY_1;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *delay =  mibReq.Param.ReceiveDelay1;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDelay2_set (uint32_t delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RECEIVE_DELAY_2;
    mibReq.Param.ReceiveDelay2 = delay;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDelay2_get (uint32_t *delay)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RECEIVE_DELAY_2;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *delay =  mibReq.Param.ReceiveDelay2;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDataRate2_set (uint8_t dr)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    mibReq.Param.Rx2Channel.Datarate = dr;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxDataRate2_get (uint8_t *dr)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *dr =  mibReq.Param.Rx2Channel.Datarate;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxFrequency2_set (uint32_t freq)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    mibReq.Param.Rx2Channel.Frequency = freq;
    status = LoRaMacMibSetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_rxFrequency2_get (uint32_t *freq)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_RX2_CHANNEL;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

    *freq =  mibReq.Param.Rx2Channel.Frequency;

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_duty_cycle_set (bool duty_cycle)
{   
#if defined (REGION_EU868)
    m_duty_cycle = duty_cycle;
    LoRaMacTestSetDutyCycleOn(duty_cycle);
#endif
    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_duty_cycle_get (bool *duty_cycle)
{
    *duty_cycle = m_duty_cycle;
     return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_channel_set (uint8_t id, lmh_channel_param_t channel_param)
{
    LoRaMacStatus_t status;

    if (channel_param.Frequency != 0)
    {
        status = LoRaMacChannelAdd(id, (ChannelParams_t){ channel_param.Frequency, 0, channel_param.DrRange, 0 });
        LMH_VERIFY_SUCCESS(status);
    }
    else
    {
        status = LoRaMacChannelRemove(id);
        LMH_VERIFY_SUCCESS(status);
    }

    return LORAMAC_STATUS_OK;
}

lmh_error_code_t lmh_channels_get (lmh_channel_param_t *channel_param)
{
    LoRaMacStatus_t status;

    mibReq.Type = MIB_CHANNELS;
    status = LoRaMacMibGetRequestConfirm(&mibReq);
    LMH_VERIFY_SUCCESS(status);

   // channel_param = mibReq.Param.ChannelList;
    memcpy(channel_param, mibReq.Param.ChannelList, 16*sizeof(lmh_channel_param_t));

    return LORAMAC_STATUS_OK;
}