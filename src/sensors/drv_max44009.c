 /******************************************************************************
 * @file    drv_max44009.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    06-november-2018
 * @brief   max44009 driver implementation file.
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

#include "drv_max44009.h"
#include "twi_manager.h"

#include "nrf_error.h"

#include "macros_common.h"

/**@brief Check if the driver is open, if not return NRF_ERROR_INVALID_STATE.
 */
#define DRV_CFG_CHECK(PARAM)                                                                      \
        if ((PARAM) == NULL)                                                                      \
        {                                                                                         \
            return NRF_ERROR_INVALID_STATE;                                                       \
        }

/**@brief TWI configuration.
 */
static struct
{
    drv_max44009_twi_cfg_t const * p_cfg;
} m_max44009;

/**@brief Open the TWI bus for communication.
 */
static __inline uint32_t twi_open(void)
{
    uint32_t err_code;

    err_code = twi_manager_request(m_max44009.p_cfg->p_twi_instance,
                                   m_max44009.p_cfg->p_twi_cfg,
                                   NULL,
                                   NULL);
    RETURN_IF_ERROR(err_code);

    nrf_drv_twi_enable(m_max44009.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}


/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void)
{
    nrf_drv_twi_disable(m_max44009.p_cfg->p_twi_instance);

    nrf_drv_twi_uninit(m_max44009.p_cfg->p_twi_instance);

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
static uint32_t reg_read(uint8_t reg_addr, uint8_t * p_reg_val)
{
    uint32_t err_code;

    err_code = nrf_drv_twi_tx( m_max44009.p_cfg->p_twi_instance,
                               m_max44009.p_cfg->twi_addr,
                               &reg_addr,
                               1,
                               true );
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_rx( m_max44009.p_cfg->p_twi_instance,
                               m_max44009.p_cfg->twi_addr,
                               p_reg_val,
                               1 );
    RETURN_IF_ERROR(err_code);

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
static uint32_t reg_write(uint8_t reg_addr, uint8_t reg_val)
{
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx( m_max44009.p_cfg->p_twi_instance,
                               m_max44009.p_cfg->twi_addr,
                               buffer,
                               2,
                               false );
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t drv_max44009_open(drv_max44009_twi_cfg_t const * const p_cfg)
{
    m_max44009.p_cfg = p_cfg;

    return twi_open();
}


uint32_t drv_max44009_close(void)
{
    uint32_t err_code = twi_close();

    m_max44009.p_cfg = NULL;

    return err_code;
}

uint32_t drv_max44009_cfg_set(drv_max44009_cfg_t const * const p_cfg)
{
    uint32_t err_code;

    DRV_CFG_CHECK(m_max44009.p_cfg);
    NULL_PARAM_CHECK(p_cfg);

    err_code = reg_write(INTERRUPT_ENABLE_REG, p_cfg->int_en_reg);
    RETURN_IF_ERROR(err_code);

    err_code = reg_write(CONFIGURATION_REG, p_cfg->cfg_reg);
    RETURN_IF_ERROR(err_code);

    err_code = reg_write(THRESHOLD_SET_H_REG, p_cfg->upper_threshold_lim_reg);
    RETURN_IF_ERROR(err_code);
	
    err_code = reg_write(THRESHOLD_SET_L_REG, p_cfg->lower_threshold_lim_reg);
    RETURN_IF_ERROR(err_code);
 
    err_code = reg_write(THRESHOLD_SET_TIMER_REG, p_cfg->threshold_timer_reg);
    RETURN_IF_ERROR(err_code);
	
    return NRF_SUCCESS;
}


uint32_t drv_max44009_cfg_get(drv_max44009_cfg_t *  p_cfg)
{
    uint32_t err_code;

    DRV_CFG_CHECK(m_max44009.p_cfg);
    NULL_PARAM_CHECK(p_cfg);

    err_code = reg_read(INTERRUPT_ENABLE_REG, &p_cfg->int_en_reg);
    RETURN_IF_ERROR(err_code);
	
    err_code = reg_read(CONFIGURATION_REG, &p_cfg->cfg_reg);
    RETURN_IF_ERROR(err_code);

    err_code = reg_read(THRESHOLD_SET_H_REG, &p_cfg->upper_threshold_lim_reg);
    RETURN_IF_ERROR(err_code);
	
    err_code = reg_read(THRESHOLD_SET_L_REG, &p_cfg->lower_threshold_lim_reg);
    RETURN_IF_ERROR(err_code);
 
    err_code = reg_read(THRESHOLD_SET_TIMER_REG, &p_cfg->threshold_timer_reg);
    RETURN_IF_ERROR(err_code);	
	
    return NRF_SUCCESS;
}

uint32_t drv_max44009_int_status_get(uint8_t * p_status)
{
    uint32_t err_code;

    DRV_CFG_CHECK(m_max44009.p_cfg);
    NULL_PARAM_CHECK(p_status);

    err_code = reg_read(INTERRUPT_STATUS_REG, p_status);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_max44009_light_get(uint16_t * p_light)
{
    uint32_t err_code;
    uint8_t exponent, mantissa;
    uint8_t  light_l;
    uint8_t  light_h;

    DRV_CFG_CHECK(m_max44009.p_cfg);
    NULL_PARAM_CHECK(p_light);

    err_code = reg_read(LUX_READING_H_REG, &light_h);
    RETURN_IF_ERROR(err_code);

    err_code = reg_read(LUX_READING_L_REG, &light_l);
    RETURN_IF_ERROR(err_code);

	
    exponent = (( light_h >> 4 )  & 0x0F);
    mantissa = (( light_h & 0x0F ) << 4) | ((light_l & 0x0F));

    *p_light = ( (uint16_t) exponent << 8 ) | ( (uint16_t) mantissa << 0);

    return NRF_SUCCESS;
}

float drv_max44009_convert_light(uint16_t lux) 
{
    uint8_t exponent, mantissa;
    float result = 0.045;

    exponent = (lux >> 8) & 0xFF;    
    exponent = (exponent == 0x0F ? exponent & 0x0E : exponent);

    mantissa = (lux >> 0) & 0xFF;

    result *= 2^exponent * mantissa;

    return result;
}

uint32_t drv_max44009_odr_set(drv_max44009_odr_t odr)
{
    uint32_t err_code;
    uint8_t  reg_val;
    uint8_t  reg_val_new;

    DRV_CFG_CHECK(m_max44009.p_cfg);
	
    err_code = reg_read(CONFIGURATION_REG, &reg_val);
    RETURN_IF_ERROR(err_code);

    reg_val_new = reg_val;

    reg_val_new &= ~CONFIGURATION_REG_INTEGRATION_Msk;
    reg_val_new |= (odr << CONFIGURATION_REG_INTEGRATION_Pos);

    if (reg_val_new != reg_val)
    {
        err_code = reg_write(CONFIGURATION_REG, reg_val);
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t drv_max44009_init(void)
{
    m_max44009.p_cfg = NULL;

    return NRF_SUCCESS;
}
