 /******************************************************************************
 * @file    drv_max44009.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    06-november-2018
 * @brief   max44009 driver header file.
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

#ifndef __MAX44009_H__
#define __MAX44009_H__

#include "nrf_drv_twi.h"
#include <stdint.h>

#define MAX44009_ADDR           0x4A // Light

/**@brief Interrupt Status Register. */
#define INTERRUPT_STATUS_REG                    0x00
#define INTERRUPT_STATUS_REG_DEFAULT            0x00

/**@brief Bitmasks for INTS. */
#define INTERRUPT_STATUS_REG_INTS_Pos           0
#define INTERRUPT_STATUS_REG_INTS_Msk           (1 << INTERRUPT_STATUS_REG_INTS_Pos)


/**@brief Interrupt Enable Register. */
#define INTERRUPT_ENABLE_REG                    0x01
#define INTERRUPT_ENABLE_REG_DEFAULT            0x00

/**@brief Bitmasks for INTS. */
#define INTERRUPT_ENABLE_REG_INTS_Pos           0
#define INTERRUPT_ENABLE_REG_INTS_Msk           (1 << INTERRUPT_ENABLE_REG_INTS_Pos)
#define INTERRUPT_ENABLE_REG_INTS_Disable       0
#define INTERRUPT_ENABLE_REG_INTS_Enable        1


/**@brief Configuration Register. */
#define CONFIGURATION_REG                       0x02
#define CONFIGURATION_REG_DEFAULT               0x03

/**@brief Bitmasks for Continuous Mode. */
#define CONFIGURATION_REG_CONTMODE_Pos          7
#define CONFIGURATION_REG_CONTMODE_Msk          (1 << CONFIGURATION_REG_CONTMODE_Pos)
#define CONFIGURATION_REG_CONTMODE_Disable      0
#define CONFIGURATION_REG_CONTMODE_Enable       1

/**@brief Bitmasks for Manual Configuration. */
#define CONFIGURATION_REG_MANUALCONF_Pos        6
#define CONFIGURATION_REG_MANUALCONF_Msk        (1 << CONFIGURATION_REG_MANUALCONF_Pos)
#define CONFIGURATION_REG_MANUALCONF_Disable    0
#define CONFIGURATION_REG_MANUALCONF_Enable     1

/**@brief Bitmasks for Current Division Ratio. */
#define CONFIGURATION_REG_CDR_Pos               3
#define CONFIGURATION_REG_CDR_Msk               (1 << CONFIGURATION_REG_CDR_Pos)
#define CONFIGURATION_REG_CDR_NODIV             0
#define CONFIGURATION_REG_CDR_DIV8              1

/**@brief Bitmasks for Integration Time. */
#define CONFIGURATION_REG_INTEGRATION_Pos       3
#define CONFIGURATION_REG_INTEGRATION_Msk       (1 << CONFIGURATION_REG_INTEGRATION_Pos)
#define CONFIGURATION_REG_INTEGRATION_800MS     0
#define CONFIGURATION_REG_INTEGRATION_400MS     1
#define CONFIGURATION_REG_INTEGRATION_200MS     2
#define CONFIGURATION_REG_INTEGRATION_100MS     3
#define CONFIGURATION_REG_INTEGRATION_50MS      4
#define CONFIGURATION_REG_INTEGRATION_25MS      5
#define CONFIGURATION_REG_INTEGRATION_12_5MS    6
#define CONFIGURATION_REG_INTEGRATION_6_25MS    7


/**@brief LUX READING Registers. */
#define LUX_READING_H_REG                       0x03
#define LUX_READING_L_REG                       0x04


/**@brief THRESHOLD SET Registers. */
#define THRESHOLD_SET_H_REG                     0x05
#define THRESHOLD_SET_H_REG_DEFAULT             0xFF

#define THRESHOLD_SET_L_REG                     0x06
#define THRESHOLD_SET_L_REG_DEFAULT             0x00

#define THRESHOLD_SET_TIMER_REG                 0x07
#define THRESHOLD_SET_TIMER_REG_DEFAULT         0xFF

/**@brief Configuration struct for max44009 light sensor.
 */
typedef struct
{
    uint8_t int_en_reg;
    uint8_t cfg_reg;
    uint8_t upper_threshold_lim_reg;	
    uint8_t lower_threshold_lim_reg;
    uint8_t threshold_timer_reg;		
}drv_max44009_cfg_t;

/**@brief Initialization struct for max44009 light sensor driver.
 */
typedef struct
{
    uint8_t                      twi_addr;        ///< TWI address.
    uint32_t                     pin_int;         ///< Interrupt pin number.
    nrf_drv_twi_t        const * p_twi_instance;  ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const * p_twi_cfg;       ///< The TWI configuration to use while the driver is enabled.
} drv_max44009_twi_cfg_t;

/**@brief Available integration time (which is equal to ODR if sensor is in Continuous Mode).
 */
typedef enum
{
    DRV_MAX44009_ODR_800MS,
    DRV_MAX44009_ODR_400MS,
    DRV_MAX44009_ODR_200MS,
    DRV_MAX44009_ODR_100MS,
    DRV_MAX44009_ODR_50MS,
    DRV_MAX44009_ODR_25MS,
    DRV_MAX44009_ODR_12_5MS,
    DRV_MAX44009_ODR_6_25MS,
}drv_max44009_odr_t;

/**@brief Inits the max44009 driver.
 */
uint32_t drv_max44009_init (void);

/**@brief Opens the max44009 driver according to the specified configuration.
 *
 * @param[in]   p_twi_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_open (drv_max44009_twi_cfg_t const * const p_twi_cfg);

/**@brief Close the max44009 driver.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_close (void);

/**@brief Configures the max44009 sensor according to the specified configuration.
 *
 * @param[in]   p_cfg Pointer to the sensor configuration.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_cfg_set (drv_max44009_cfg_t const * const p_cfg);

/**@brief Reads the configuration of the max44009 sensor.
 *
 * @param[in]   p_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_cfg_get (drv_max44009_cfg_t *  p_cfg);

/**@brief Function to get the interrupt status.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_int_status_get (uint8_t * p_status);

/**@brief Function to get the lux reading data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_light_get (uint16_t * p_light);

/**@brief Function to convert uint16 value to float.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
float drv_max44009_convert_light (uint16_t lux);

/**@brief Function to output data rate of the lps22hb.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_max44009_odr_set (drv_max44009_odr_t odr);

#endif

/** @} */
