/******************************************************************************
 * @file    drv_humidity.c
 * @author  Insight SiP
 * @brief   humidity driver implementation file.
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

#include "drv_humidity.h"
#include "app_scheduler.h"
#include "drv_shtc1.h"
#include "nrf_drv_gpiote.h"
#include "twi_manager.h"
#include <string.h>

/**@brief Humidity configuration struct.
 */
typedef struct
{
    drv_shtc1_twi_cfg_t cfg; ///< TWI configuraion.
    bool enabled;            ///< Driver enabled.
} drv_humidity_t;

/**@brief Stored configuration.
 */
static drv_humidity_t m_drv_humidity;

uint32_t drv_humidity_init(drv_humidity_init_t *p_params) {
    uint32_t err_code;

    drv_shtc1_init();

    m_drv_humidity.cfg.twi_addr = SHTC1_ADDR;
    m_drv_humidity.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_humidity.cfg.p_twi_config = p_params->p_twi_cfg;
    m_drv_humidity.enabled = false;

    // Request twi bus
    drv_shtc1_open(&m_drv_humidity.cfg);

    err_code = drv_shtc1_verify();

    // Release twi bus
    drv_shtc1_close();

    return err_code;
}

uint32_t drv_humidity_enable(void) {
    if (m_drv_humidity.enabled) {
        return NRF_SUCCESS;
    }

    m_drv_humidity.enabled = true;

    return NRF_SUCCESS;
}

uint32_t drv_humidity_disable(void) {
    if (m_drv_humidity.enabled == false) {
        return NRF_SUCCESS;
    }

    m_drv_humidity.enabled = false;

    return NRF_SUCCESS;
}

uint32_t drv_humidity_reset(void) {
    uint32_t err_code;

    // Request twi bus
    drv_shtc1_open(&m_drv_humidity.cfg);

    err_code = drv_shtc1_reboot();

    // Release twi bus
    drv_shtc1_close();

    return err_code;
}

uint32_t drv_temperature_and_humidity_get(float *p_temperature, float *p_humidity) {
    uint32_t err_code;

    // Request twi bus
    drv_shtc1_open(&m_drv_humidity.cfg);

    err_code = drv_shtc1_temp_humi_get(p_temperature, p_humidity);

    // Release twi bus
    drv_shtc1_close();

    return err_code;
}