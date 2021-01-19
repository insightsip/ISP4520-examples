/******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   Rx temperature main file.
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
#include "app_timer.h"
#include "app_scheduler.h"

// LoRa
#include "radio.h"
#include "board.h"

//logs
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

#define SCHED_MAX_EVENT_DATA_SIZE                   APP_TIMER_SCHED_EVENT_DATA_SIZE     ///< Maximum size of scheduler events. 
#define SCHED_QUEUE_SIZE                            60                                  ///< Maximum number of events in the scheduler queue.


APP_TIMER_DEF(lora_tx_timer_id);                                        ///< LoRa tranfer timer instance.
static RadioEvents_t RadioEvents;                                       ///< LoRa driver instance.


/**@brief Function executed on Radio Rx Timeout event
 */
void OnRadioRxTimeout (void)
{
    // Should never be called
}

 /**@brief Function executed on Radio Rx Error event
 */
void OnRadioRxError (void)
{
    // Restart Rx
    Radio.Rx(0);
}

 /**@brief Function executed on Radio Rx Done event
 */
void OnRadioRxdone (uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    uint8_t frame_counter;
    int32_t temperature_reg = 0;
    float temperature;

    if (size == 6 && payload[0] == 'T')
    {
        frame_counter   = payload[1];
        temperature_reg |= payload[2] << 24;
        temperature_reg |= payload[3] << 16;
        temperature_reg |= payload[4] << 8;
        temperature_reg |= payload[5] << 0;
         
         // Temperature in °C (0.25° steps)
        temperature = temperature_reg * 0.25;
        NRF_LOG_INFO("LoRa frame %d received with RSSI: %d dBm.\r\nRemote temperature = "NRF_LOG_FLOAT_MARKER " \°C", frame_counter, rssi, NRF_LOG_FLOAT(temperature));
    }

    // Restart Rx
    Radio.Rx(0);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    __WFE();
    __SEV();
    __WFE();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;

    // Initialize logs.
    log_init();
    NRF_LOG_INFO("LoRa rx temperature example started.");

    // Initialize Clocks and DCDC	
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_drv_clock_hfclk_request(NULL);
    NRF_POWER->DCDCEN = 1;
	
    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LoRa driver.
    RadioEvents.RxDone      = OnRadioRxdone;
    RadioEvents.RxError     = OnRadioRxError;
    RadioEvents.RxTimeout   = OnRadioRxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    // start Rx
    Radio.Rx(0);

    // Set green LED on
    nrf_gpio_pin_write(13, 0);
    nrf_gpio_cfg_output(13);

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

