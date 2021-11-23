/******************************************************************************
 * @file    main.c
 * @author  Insight SiP
 * @brief   TX CW main file.
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
#define TX_OUTPUT_POWER                             20          // dBm
#define RF_FREQUENCY                                915000000   // Hz
#else
    #error "Please define a ISP4520 configuration"
#endif

#define TX_TIMEOUT                                  3         // in seconds

static RadioEvents_t RadioEvents;


 /**@brief Function executed on Radio Tx Timeout event
 */
void OnRadioTxTimeout(void)
{
    NRF_LOG_INFO("OnRadioTxTimeout");
    Radio.Sleep();
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
    NRF_LOG_INFO("LoRa tx-cw example started.");
	
    // Initialize Clocks and DCDC
    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    NRF_POWER->DCDCEN = 1;
	
    // Initialize LoRa chip.
    err_code = lora_hardware_init();
    APP_ERROR_CHECK(err_code);

    // Initialize LoRa driver.
    RadioEvents.TxTimeout = OnRadioTxTimeout;
    Radio.Init(&RadioEvents);
	
    // Start LORA
    Radio.SetTxContinuousWave(RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT);

    // Enter main loop.
    for (;;)
    {
        // sx126x drivers needs Radio.IrqProcess(); to be called regulary to poll IRQs
        Radio.IrqProcess();

        // Process logs and sleep 
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}

