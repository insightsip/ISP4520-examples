 /******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   LoRaMac classA project main file.
 *          This file contains initialization for starting device in LoRaMac classA 
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
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"

// LoRa
#include "board.h"

//logs
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// LoRaMac
#include "loramachelper.h"

#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60                              /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_APP_PORT            2       /**< LoRaWAN application port, do not use 224. It is reserved for certification */
#define LORAWAN_APP_DATA_BUFF_SIZE  64      /**< Size of the data buffer. */  
#define LORAWAN_APP_TX_DUTYCYCLE    10000   /**< Defines the application data transmission duty cycle. 10s, value in [ms]. */  


// Foward declaration
static void lmh_rx_data_handler(lmh_app_rx_data_t *app_data);
static void lmh_evt_handler(lmh_evt_type_t type, void *data);

APP_TIMER_DEF(lora_tx_timer_id);                                                    ///< LoRa tx timer instance.
static uint8_t m_lora_rx_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                   ///< LoRa user application rx data buffer.
static lmh_app_rx_data_t m_lora_rx_data = {m_lora_rx_data_buffer, 0 ,0, 0, 0};      ///< LoRa user application rx data structure.
static uint8_t m_lora_tx_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];                   ///< LoRa user application tx data buffer.
static lmh_app_tx_data_t m_lora_tx_data = {m_lora_tx_data_buffer, 0 ,0, 0, 0};      ///< LoRa user application tx data structure.

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                            lmh_rx_data_handler, lmh_evt_handler};



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
#if (OVER_THE_AIR_ACTIVATION != 0)
            NRF_LOG_INFO("Network Joined");
#endif
        } 
        break;

        case LHM_EVT_RX_ACK:
        {
            NRF_LOG_INFO("Acknowledge received");
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

/**@brief Function for handling LoRaWan received data from the server
 *
 * @param[in] app_data  Pointer to rx data
 */
static void lmh_rx_data_handler(lmh_app_rx_data_t *app_data)
{
    NRF_LOG_DEBUG("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d", app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

    switch (app_data->port)
    {
        case LORAWAN_APP_PORT:
            // Take action on received data 
        break;
		
        default:
            break;
    }
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
static void tx_lora_periodic_handler(void * unused)
{
    uint32_t i = 0;
    m_lora_tx_data.port = LORAWAN_APP_PORT;
    m_lora_tx_data.buffer[i++] = 'H';
    m_lora_tx_data.buffer[i++] = 'e';
    m_lora_tx_data.buffer[i++] = 'l';
    m_lora_tx_data.buffer[i++] = 'l';
    m_lora_tx_data.buffer[i++] = 'o';
    m_lora_tx_data.buffer[i++] = ' ';
    m_lora_tx_data.buffer[i++] = 'w';
    m_lora_tx_data.buffer[i++] = 'o';
    m_lora_tx_data.buffer[i++] = 'r';
    m_lora_tx_data.buffer[i++] = 'l';
    m_lora_tx_data.buffer[i++] = 'd';
    m_lora_tx_data.buffer[i++] = '!';
    m_lora_tx_data.buffsize = i;
    m_lora_tx_data.confirmed = 0;
    m_lora_tx_data.nb_trials = 8;
  
    lmh_send(&m_lora_tx_data);
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
	
    // Initialize timers
    err_code = app_timer_create(&lora_tx_timer_id, APP_TIMER_MODE_REPEATED, tx_lora_periodic_handler);
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
	
    // Initialize logs.
    log_init();
    NRF_LOG_INFO("LoRaWan Class A example started.");
	
    // Initialize clocks & power
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    nrf_pwr_mgmt_init();

    // Enable nRF52 DCDC
    NRF_POWER->DCDCEN = 1;

    // Initialize Scheduler and timer
    timers_init();
		
    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);
	
    // Initialize LoRaWan
    err_code = lmh_init(&lora_callbacks);
    APP_ERROR_CHECK(err_code);  

    lmh_duty_cycle_set(0);

    // Start Join procedure
    lmh_join(OVER_THE_AIR_ACTIVATION);
    app_timer_start(lora_tx_timer_id, APP_TIMER_TICKS(LORAWAN_APP_TX_DUTYCYCLE), NULL);
		
    // Enter main loop.
    for (;;)
    {
        lmh_process();

        app_sched_execute();

        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }		
    }
}

