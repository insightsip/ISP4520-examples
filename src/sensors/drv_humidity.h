/******************************************************************************
 * @file    drv_humidity.h
 * @author  Insight SiP
 * @brief   humidity driver header file.
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

#ifndef __DRV_HUMIDITY_H__
#define __DRV_HUMIDITY_H__

#include "nrf_drv_twi.h"
#include <stdint.h>

/**@brief Humidity driver event types.
 */
typedef enum {
    DRV_HUMIDITY_EVT_DATA, /**<Converted value ready to be read.*/
    DRV_HUMIDITY_EVT_ERROR /**<HW error on the communication bus.*/
} drv_humidity_evt_t;

/**@brief Humidity driver event handler callback type.
 */
typedef void (*drv_humidity_evt_handler_t)(drv_humidity_evt_t evt);

/**@brief Initialization struct for humidity driver.
 */
typedef struct
{
    uint8_t twi_addr;
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
    drv_humidity_evt_handler_t evt_handler;
} drv_humidity_init_t;

/**@brief Function for initializing the humidity driver.
 *
 * @param[in] p_params      Pointer to init parameters.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE If the driver is in invalid state.
 */
uint32_t drv_humidity_init(drv_humidity_init_t *p_params);

/**@brief Function for enabling the humidity sensor.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
uint32_t drv_humidity_enable(void);

/**@brief Function for disabling the humidity sensor.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 */
uint32_t drv_humidity_disable(void);

/**@brief Function for resetting the chip to all default register values
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_humidity_reset(void);

/**@brief Function for getting the humidity data.
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_temperature_and_humidity_get(float *p_temperature, float *p_humidity);

#endif