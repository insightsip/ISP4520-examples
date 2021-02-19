 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   LoRa ping-pong example
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
#include <stdint.h>
#include <string.h>

// nRF
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"

// LoRa
#include "radio.h"
#include "board.h"

// logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined(ISP4520_EU) 
#define TX_OUTPUT_POWER                             14          // dBm
#define RF_FREQUENCY                                868000000   // Hz
#elif defined(ISP4520_AS) 
#define TX_OUTPUT_POWER                             14          // dBm
#define RF_FREQUENCY                                923000000   // Hz
#elif defined(ISP4520_US)
#define TX_OUTPUT_POWER                             22          // dBm
#define RF_FREQUENCY                                915000000   // Hz
#else
    #error "Please define a ISP4520 configuration"
#endif

#define LORA_BANDWIDTH                              0       // [0: 125 kHz,
                                                            //  1: 250 kHz,
                                                            //  2: 500 kHz,
                                                            //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7       // [SF7..SF12]
#define LORA_CODINGRATE                             1       // [1: 4/5,
                                                            //  2: 4/6,
                                                            //  3: 4/7,
                                                            //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8       // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            5000
#define BUFFER_SIZE                                 64      // Define the payload size here

static RadioEvents_t RadioEvents;
static uint16_t BufferSize = BUFFER_SIZE;
static uint8_t Buffer[BUFFER_SIZE];
static bool isMaster = true;
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";


/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone (void)
{
//    Radio.Sleep();
    NRF_LOG_INFO("OnTxDone");
    Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone (uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    //Radio.Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);

    NRF_LOG_INFO("OnRxDone");
    NRF_LOG_INFO("RssiValue=%d dBm, SnrValue=%d", rssi, snr);

    if (isMaster == true)
    {
        if (BufferSize > 0)
        {
            if (strncmp((const char*)Buffer, (const char*)PongMsg, 4) == 0)
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                // We fill the buffer with numbers for the payload
                for (int i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                DelayMs(500);
                Radio.Send(Buffer, BufferSize);
            }
            else if (strncmp((const char*)Buffer, (const char*)PingMsg, 4) == 0)
            { // A master already exists then become a slave
                isMaster = false;
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
            else // valid reception but neither a PING or a PONG message
            {    // Set device as master ans start again
                isMaster = true;
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
        }
    }
    else
    {
        if (BufferSize > 0)
        {
            if(strncmp((const char* )Buffer, ( const char* )PingMsg, 4) == 0)
            {
                // Send the reply to the PONG string
                Buffer[0] = 'P';
                Buffer[1] = 'O';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                // We fill the buffer with numbers for the payload
                for (int i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                DelayMs (500);
                Radio.Send(Buffer, BufferSize);
            }
            else // valid reception but not a PING as expected
            {    // Set device as master and start again
                isMaster = true;
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
        }
    }
}


/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout (void)
{
   // Radio.Sleep();
    NRF_LOG_INFO("OnTxTimeout");

    Radio.Rx(RX_TIMEOUT_VALUE);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout (void)
{
   // Radio.Sleep();
    NRF_LOG_INFO("OnRxTimeout");

    if (isMaster == true)
    {
	// Send the next PING frame
	Buffer[0] = 'P';
	Buffer[1] = 'I';
	Buffer[2] = 'N';
	Buffer[3] = 'G';
	for(int i = 4; i < BufferSize; i++)
	{
            Buffer[i] = i - 4;
	}
	DelayMs(1);
	Radio.Send(Buffer, BufferSize);
    }
    else
    {
	Radio.Rx(RX_TIMEOUT_VALUE);
    }
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError (void)
{
  // Radio.Sleep();
    NRF_LOG_INFO("OnRxError");

    if (isMaster == true)
    {
        // Send the next PING frame
        Buffer[0] = 'P';
        Buffer[1] = 'I';
	Buffer[2] = 'N';
	Buffer[3] = 'G';
	for(int i = 4; i < BufferSize; i++)
	{
            Buffer[i] = i - 4;
	}
	DelayMs(1);
	Radio.Send(Buffer, BufferSize);
    }
    else
    {
        Radio.Rx(RX_TIMEOUT_VALUE);
    }
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init (void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power manager.
 */
static void power_manage (void)
{
    __WFE();
    __SEV();
    __WFE();
}

/**@brief Function for application main entry.
 */
int main (void)
{
    ret_code_t err_code;

    // Initialize logs.
    log_init();
    NRF_LOG_INFO("LoRa ping-pong example started.");

    // Initialize Clocks and DCDC
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_drv_clock_hfclk_request(NULL);
    NRF_POWER->DCDCEN = 1;

    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LoRa driver.
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    // Start LoRa
    Radio.Rx(RX_TIMEOUT_VALUE);

    // Enter main loop.
    for (;;)
    {
        Radio.IrqProcess();
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}
