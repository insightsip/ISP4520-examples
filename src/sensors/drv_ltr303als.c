/******************************************************************************
 * @file    drv_ltr303als.c
 * @author  Insight SiP
 * @brief   ltr303als driver implementation file.
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

#include "drv_ltr303als.h"
#include "twi_manager.h"

#include <math.h>

/**@brief Check if the driver is open, if not return NRF_ERROR_INVALID_STATE.
 */
#define DRV_CFG_CHECK(PARAM)            \
    if ((PARAM) == NULL) {              \
        return NRF_ERROR_INVALID_STATE; \
    }

/**@brief TWI configuration.
 */
static struct
{
    drv_ltr303als_twi_cfg_t const *p_cfg;
} m_ltr303als;

/**@brief Open the TWI bus for communication.
 */
static __inline uint32_t twi_open(void) {
    uint32_t err_code;

    err_code = twi_manager_request(m_ltr303als.p_cfg->p_twi_instance,
        m_ltr303als.p_cfg->p_twi_cfg,
        NULL,
        NULL);
    VERIFY_SUCCESS(err_code);

    nrf_drv_twi_enable(m_ltr303als.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void) {
    nrf_drv_twi_disable(m_ltr303als.p_cfg->p_twi_instance);

    nrf_drv_twi_uninit(m_ltr303als.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function for reading a sensor register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_read(uint8_t reg_addr, uint8_t *p_reg_val) {
    uint32_t err_code;

    err_code = nrf_drv_twi_tx(m_ltr303als.p_cfg->p_twi_instance,
        m_ltr303als.p_cfg->twi_addr,
        &reg_addr,
        1,
        true);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_twi_rx(m_ltr303als.p_cfg->p_twi_instance,
        m_ltr303als.p_cfg->twi_addr,
        p_reg_val,
        1);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for writing to a sensor register.
 *
 * @param[in]  reg_addr            Address of the register to write to.
 * @param[in]  reg_val             Value to write to the register.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_write(uint8_t reg_addr, uint8_t reg_val) {
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx(m_ltr303als.p_cfg->p_twi_instance,
        m_ltr303als.p_cfg->twi_addr,
        buffer,
        2,
        false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_ltr303als_open(drv_ltr303als_twi_cfg_t const *const p_cfg) {
    m_ltr303als.p_cfg = p_cfg;

    return twi_open();
}

uint32_t drv_ltr303als_close(void) {
    uint32_t err_code = twi_close();

    m_ltr303als.p_cfg = NULL;

    return err_code;
}

uint32_t drv_ltr303als_cfg_set(drv_ltr303als_cfg_t const *const p_cfg) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_ltr303als.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_cfg);

    err_code = reg_write(ALS_CONTR_REG, p_cfg->als_contr);
    VERIFY_SUCCESS(err_code);

    err_code = reg_write(ALS_MEAS_RATE_REG, p_cfg->als_meas_rate);
    VERIFY_SUCCESS(err_code);

    err_code = reg_write(INTERRUPT_REG, p_cfg->interrupt);
    VERIFY_SUCCESS(err_code);

    err_code = reg_write(INTERRUPT_PERSIST_REG, p_cfg->interrupt_persist);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_ltr303als_cfg_get(drv_ltr303als_cfg_t *p_cfg) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_ltr303als.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_cfg);

    err_code = reg_read(ALS_CONTR_REG, &p_cfg->als_contr);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(ALS_MEAS_RATE_REG, &p_cfg->als_meas_rate);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(INTERRUPT_REG, &p_cfg->interrupt);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(INTERRUPT_PERSIST_REG, &p_cfg->interrupt_persist);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_ltr303als_light_get(double *p_lux) {
    uint32_t err_code;
    uint8_t light_ch1_l, light_ch1_h, light_ch0_l, light_ch0_h, status;
    uint8_t gain;
    uint16_t light_ch1, light_ch0;
    double f_light_ch1, f_light_ch0, ratio;

    DRV_CFG_CHECK(m_ltr303als.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_lux);

    // Read status
    err_code = reg_read(ALS_STATUS_REG, &status);
    VERIFY_SUCCESS(err_code);

    // Check if there is new data available
    if (!(status & ALS_STATUS_REG_DATA_STATUS_Msk)) {
        *p_lux = 0.0;
        return NRF_SUCCESS;
    }

    gain = (status & ALS_STATUS_REG_GAIN_Msk) >> ALS_STATUS_REG_GAIN_Pos;

    // Read data
    err_code = reg_read(ALS_DATA_CH1_0_REG, &light_ch1_l);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(ALS_DATA_CH1_1_REG, &light_ch1_h);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(ALS_DATA_CH0_0_REG, &light_ch0_l);
    VERIFY_SUCCESS(err_code);

    err_code = reg_read(ALS_DATA_CH0_1_REG, &light_ch0_h);
    VERIFY_SUCCESS(err_code);

    light_ch1 = (light_ch1_h << 8) | light_ch1_l;
    light_ch0 = (light_ch0_h << 8) | light_ch0_l;

    // Convert to lux...

    // Check if sensor is saturated
    if ((light_ch0 == 0xFFFF) || (light_ch1 == 0xFFFF)) {
        *p_lux = 0.0;
        return NRF_SUCCESS;
    }

    f_light_ch0 = light_ch0;
    f_light_ch1 = light_ch1;

    ratio = f_light_ch1 / f_light_ch0;
    f_light_ch0 *= 402.0 / 100; // 100 = intergration time ms
    f_light_ch1 *= 402.0 / 100; // 100 = intergration time ms

    if (!gain) {
        f_light_ch0 *= 16;
        f_light_ch1 *= 16;
    }

    if (ratio < 0.5) {
        *p_lux = 0.0304 * f_light_ch0 - 0.062 * f_light_ch0 * pow(ratio, 1.4);
        return NRF_SUCCESS;
    }

    if (ratio < 0.61) {
        *p_lux = 0.0224 * f_light_ch0 - 0.031 * f_light_ch1;
        return NRF_SUCCESS;
    }

    if (ratio < 0.80) {
        *p_lux = 0.0128 * f_light_ch0 - 0.0153 * f_light_ch1;
        return NRF_SUCCESS;
    }

    // if (ratio < 1.30)
    {
        *p_lux = 0.00146 * f_light_ch0 - 0.00112 * f_light_ch1;
        return NRF_SUCCESS;
    }

    *p_lux = 0.0;

    return NRF_SUCCESS;
}

uint32_t drv_ltr303als_verify(uint8_t *who_am_i) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_ltr303als.p_cfg);

    err_code = reg_read(PART_ID_REG, who_am_i);
    VERIFY_SUCCESS(err_code);

    return (*who_am_i == PART_ID_REG_DEFAULT) ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}

uint32_t drv_ltr303al_init(void) {
    m_ltr303als.p_cfg = NULL;

    return NRF_SUCCESS;
}