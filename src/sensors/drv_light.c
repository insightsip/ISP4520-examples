/******************************************************************************
 * @file    drv_light.c
 * @author  Insight SiP
 * @brief   light driver implementation file.
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

#include "nrf_error.h"
#include "sdk_common.h"

#include "app_scheduler.h"
#include "drv_light.h"
#include "drv_ltr303als.h"
#include "nrf_drv_gpiote.h"
#include "twi_manager.h"
#include <string.h>

/**@brief Light configuration struct.
 */
typedef struct
{
    drv_ltr303als_twi_cfg_t cfg;         ///< TWI configuraion.
    drv_light_evt_handler_t evt_handler; ///< Event handler called by gpiote_evt_sceduled.
    bool enabled;                        ///< Driver enabled.
} drv_light_t;

/**@brief Stored configuration.
 */
static drv_light_t m_drv_light;

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled(void *p_event_data, uint16_t event_size) {
    // Data ready
    drv_light_evt_t evt;
    evt.type = DRV_LIGHT_EVT_DATA;

    m_drv_light.evt_handler(&evt);
}

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    uint32_t err_code;

    if ((pin == m_drv_light.cfg.pin_int) && (nrf_gpio_pin_read(m_drv_light.cfg.pin_int) == 0)) {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Initialize the GPIO tasks and events system to catch pin data ready interrupts.
 */
static uint32_t gpiote_init(uint32_t pin) {
    uint32_t err_code;

    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;
    gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    err_code = nrf_drv_gpiote_in_init(pin, &gpiote_in_config, gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    nrf_drv_gpiote_in_event_enable(pin, true);

    return NRF_SUCCESS;
}

/**@brief Uninitialize the GPIO tasks and events system.
 */
static void gpiote_uninit(uint32_t pin) {
    nrf_drv_gpiote_in_uninit(pin);
}

uint32_t drv_light_init(drv_light_init_t *p_params) {
    uint32_t err_code;
    uint8_t who_am_i;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_light.evt_handler = p_params->evt_handler;
    m_drv_light.cfg.twi_addr = LTR303ALS_ADDR;
    m_drv_light.cfg.pin_int = p_params->pin_int;
    m_drv_light.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_light.cfg.p_twi_cfg = p_params->p_twi_cfg;
    m_drv_light.enabled = false;

    // Request twi bus
    drv_ltr303als_open(&m_drv_light.cfg);

    err_code = drv_ltr303als_verify(&who_am_i);
    if (err_code != NRF_SUCCESS) {
        //  err_code = drv_lps22hb_odr_set(DRV_LPS22HB_ODR_PowerDown);
    }

    // Release twi bus
    drv_ltr303als_close();

    return err_code;
}

uint32_t drv_light_enable(void) {
    uint32_t err_code;

    if (m_drv_light.enabled) {
        return NRF_SUCCESS;
    }
    m_drv_light.enabled = true;

    static const drv_ltr303als_cfg_t ltr303als_cfg =
        {
            .als_contr = ALS_CONTR_REG_DEFAULT | (ALS_CONTR_REG_ALS_MODE_Active << ALS_CONTR_REG_ALS_MODE_Pos) | (ALS_CONTR_REG_ALS_GAIN_1 << ALS_CONTR_REG_ALS_GAIN_Pos),
            .als_meas_rate = (ALS_MEAS_RATE_REG_ALS_MRR_100MS << ALS_MEAS_RATE_REG_ALS_MRR_Pos) | (ALS_MEAS_RATE_REG_ALS_IT_100MS << ALS_MEAS_RATE_REG_ALS_IT_Pos),
            .interrupt = INTERRUPT_REG_DEFAULT,
            .interrupt_persist = INTERRUPT_PERSIST_REG_DEFAULT};

    if (m_drv_light.cfg.pin_int != DRV_LIGHT_PIN_NOT_USED) {
        err_code = gpiote_init(m_drv_light.cfg.pin_int);
        VERIFY_SUCCESS(err_code);
    }

    // Request twi bus
    drv_ltr303als_open(&m_drv_light.cfg);

    err_code = drv_ltr303als_cfg_set(&ltr303als_cfg);

    // Release twi bus
    drv_ltr303als_close();

    return err_code;
}

uint32_t drv_light_disable(void) {
    uint32_t err_code;

    if (m_drv_light.enabled == false) {
        return NRF_SUCCESS;
    }
    m_drv_light.enabled = false;

    static const drv_ltr303als_cfg_t ltr303als_cfg =
        {
            .als_contr = ALS_CONTR_REG_DEFAULT,
            .als_meas_rate = ALS_MEAS_RATE_REG_DEFAULT,
            .interrupt = INTERRUPT_REG_DEFAULT,
            .interrupt_persist = INTERRUPT_PERSIST_REG_DEFAULT};

    if (m_drv_light.cfg.pin_int != DRV_LIGHT_PIN_NOT_USED) {
        gpiote_uninit(m_drv_light.cfg.pin_int);
    }

    // Request twi bus
    drv_ltr303als_open(&m_drv_light.cfg);

    err_code = drv_ltr303als_cfg_set(&ltr303als_cfg);

    // Release twi bus
    drv_ltr303als_close();

    return err_code;
}

uint32_t drv_light_reset(void) {
    uint32_t err_code;

    static const drv_ltr303als_cfg_t ltr303als_cfg =
        {
            .als_contr = ALS_CONTR_REG_SW_RST_Active << ALS_CONTR_REG_SW_RST_Pos,
            .als_meas_rate = ALS_MEAS_RATE_REG_DEFAULT,
            .interrupt = INTERRUPT_REG_DEFAULT,
            .interrupt_persist = INTERRUPT_PERSIST_REG_DEFAULT};

    // Request twi bus
    drv_ltr303als_open(&m_drv_light.cfg);

    err_code = drv_ltr303als_cfg_set(&ltr303als_cfg);

    // Release twi bus
    drv_ltr303als_close();

    return err_code;
}

uint32_t drv_light_get(double *f_light) {
    uint32_t err_code;
    // uint16_t light;

    // Request twi bus
    drv_ltr303als_open(&m_drv_light.cfg);

    err_code = drv_ltr303als_light_get(f_light);

    // Release twi bus
    drv_ltr303als_close();

    return err_code;
}