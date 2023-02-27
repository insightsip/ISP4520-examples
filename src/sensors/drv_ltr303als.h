/******************************************************************************
 * @file    drv_ltr303als.h
 * @author  Insight SiP
 * @brief   ltr303als driver header file.
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

#ifndef __LTR303ALS_H__
#define __LTR303ALS_H__

#include "nrf_drv_twi.h"
#include <stdint.h>

#define LTR303ALS_ADDR 0x29 // I2C address

/**@brief ALS operation mode control SW reset Register */
#define ALS_CONTR_REG 0x80
#define ALS_CONTR_REG_DEFAULT 0x00

/**@brief Bitmasks for ALS_MODE. */
#define ALS_CONTR_REG_ALS_MODE_Pos 0
#define ALS_CONTR_REG_ALS_MODE_Msk (1 << ALS_CONTR_REG_ALS_MODE_Pos)
#define ALS_CONTR_REG_ALS_MODE_Stdby 0
#define ALS_CONTR_REG_ALS_MODE_Active 1

/**@brief Bitmasks for SW_RST. */
#define ALS_CONTR_REG_SW_RST_Pos 1
#define ALS_CONTR_REG_SW_RST_Msk (1 << ALS_CONTR_REG_SW_RST_Pos)
#define ALS_CONTR_REG_SW_RST_Inactive 0
#define ALS_CONTR_REG_SW_RST_Active 1

/**@brief Bitmasks for ALS_GAIN. */
#define ALS_CONTR_REG_ALS_GAIN_Pos 2
#define ALS_CONTR_REG_ALS_GAIN_Msk (7 << ALS_CONTR_REG_ALS_GAIN_Pos)
#define ALS_CONTR_REG_ALS_GAIN_1 0
#define ALS_CONTR_REG_ALS_GAIN_2 1
#define ALS_CONTR_REG_ALS_GAIN_4 2
#define ALS_CONTR_REG_ALS_GAIN_8 3
#define ALS_CONTR_REG_ALS_GAIN_48 6
#define ALS_CONTR_REG_ALS_GAIN_96 7

/**@brief ALS measurement rate in active mode */
#define ALS_MEAS_RATE_REG 0x85
#define ALS_MEAS_RATE_REG_DEFAULT 0x01

/**@brief Bitmasks for ALS_MRR. */
#define ALS_MEAS_RATE_REG_ALS_MRR_Pos 0
#define ALS_MEAS_RATE_REG_ALS_MRR_Msk (7 << ALS_CONTR_REG_ALS_MRR_Pos)
#define ALS_MEAS_RATE_REG_ALS_MRR_50MS 0
#define ALS_MEAS_RATE_REG_ALS_MRR_100MS 1
#define ALS_MEAS_RATE_REG_ALS_MRR_200MS 2
#define ALS_MEAS_RATE_REG_ALS_MRR_500MS 3
#define ALS_MEAS_RATE_REG_ALS_MRR_1000MS 4
#define ALS_MEAS_RATE_REG_ALS_MRR_2000MS 5

/**@brief Bitmasks for ALS_IT. */
#define ALS_MEAS_RATE_REG_ALS_IT_Pos 3
#define ALS_MEAS_RATE_REG_ALS_IT_Msk (7 << ALS_CONTR_REG_ALS_IT_Pos)
#define ALS_MEAS_RATE_REG_ALS_IT_100MS 0
#define ALS_MEAS_RATE_REG_ALS_IT_50MS 1
#define ALS_MEAS_RATE_REG_ALS_IT_200MS 2
#define ALS_MEAS_RATE_REG_ALS_IT_400MS 3
#define ALS_MEAS_RATE_REG_ALS_IT_150MS 4
#define ALS_MEAS_RATE_REG_ALS_IT_250MS 5
#define ALS_MEAS_RATE_REG_ALS_IT_300MS 6
#define ALS_MEAS_RATE_REG_ALS_IT_350MS 7

/**@brief Part Number ID and Revision ID */
#define PART_ID_REG 0x86
#define PART_ID_REG_DEFAULT 0xA0

/**@brief Manufacturer ID */
#define MANUFAC_ID_REG 0x87
#define MANUFAC_ID_REG_DEFAULT 0x05

/**@brief ALS measurement CH1 data, lower byte */
#define ALS_DATA_CH1_0_REG 0x88
#define ALS_DATA_CH1_0_REG_DEFAULT 0x00

/**@brief ALS measurement CH1 data, upper byte */
#define ALS_DATA_CH1_1_REG 0x89
#define ALS_DATA_CH1_1_REG_DEFAULT 0x00

/**@brief ALS measurement CH0 data, lower byte */
#define ALS_DATA_CH0_0_REG 0x8A
#define ALS_DATA_CH0_0_REG_DEFAULT 0x00

/**@brief ALS measurement CH0 data, upper byte */
#define ALS_DATA_CH0_1_REG 0x8B
#define ALS_DATA_CH0_1_REG_DEFAULT 0x00

/**@brief ALS new data status */
#define ALS_STATUS_REG 0x8C
#define ALS_STATUS_REG_DEFAULT 0x00

/**@brief Bitmasks for DATA_STATUS. */
#define ALS_STATUS_REG_DATA_STATUS_Pos 2
#define ALS_STATUS_REG_DATA_STATUS_Msk (1 << ALS_STATUS_REG_DATA_STATUS_Pos)

/**@brief Bitmasks for INT_STATUS. */
#define ALS_STATUS_REG_INT_STATUS_Pos 3
#define ALS_STATUS_REG_INT_STATUS_Msk (1 << INTERRUPT_REG_INT_STATUS_Pos)

/**@brief Bitmasks for GAIN. */
#define ALS_STATUS_REG_GAIN_Pos 4
#define ALS_STATUS_REG_GAIN_Msk (7 << ALS_STATUS_REG_GAIN_Pos)

/**@brief Bitmasks for DATA_VALID. */
#define ALS_STATUS_REG_DATA_VALID_Pos 7
#define ALS_STATUS_REG_DATA_VALID_Msk (1 << INTERRUPT_REG_DATA_VALID_Pos)

/**@brief Interrupt settings */
#define INTERRUPT_REG 0x8F
#define INTERRUPT_REG_DEFAULT 0x08

/**@brief Bitmasks for MODE. */
#define INTERRUPT_REG_MODE_Pos 1
#define INTERRUPT_REG_MODE_Msk (1 << INTERRUPT_REG_MODE_Pos)
#define INTERRUPT_REG_MODE_INACTIVE 0
#define INTERRUPT_REG_MODE_ACTIVE 1

/**@brief Bitmasks for POL. */
#define INTERRUPT_REG_POL_Pos 2
#define INTERRUPT_REG_POL_Msk (1 << INTERRUPT_REG_POL_Pos)
#define INTERRUPT_REG_POL_ACTIVE_0 0
#define INTERRUPT_REG_POL_ACTIVE_1 1

/** @brief ALS interrupt upper threshold, lower byte */
#define ALS_THRES_UP_0_REG 0x97
#define ALS_THRES_UP_0_REG_DEFAULT 0xFF

/** @brief ALS interrupt upper threshold, upper byte */
#define ALS_THRES_UP_1_REG 0x98
#define ALS_THRES_UP_1_REG_DEFAULT 0xFF

/** @brief ALS interrupt lower threshold, lower byte */
#define ALS_THRES_LOW_0_REG 0x99
#define ALS_THRES_LOW_0_REG_DEFAULT 0x00

/** @brief ALS interrupt lower threshold, upper byte */
#define ALS_THRES_LOW_1_REG 0x9A
#define ALS_THRES_LOW_1_REG_DEFAULT 0x00

/** @brief ALS Interrupt persist setting */
#define INTERRUPT_PERSIST_REG 0x9E
#define INTERRUPT_PERSIST_REG_DEFAULT 0x00

/**@brief Configuration struct for ltr303als light sensor driver.
 */
typedef struct
{
    uint8_t als_contr;
    uint8_t als_meas_rate;
    uint8_t interrupt;
    uint8_t interrupt_persist;
} drv_ltr303als_cfg_t;

/**@brief Initialization struct for ltr303als light sensor driver.
 */
typedef struct
{
    uint8_t twi_addr;                      ///< TWI address.
    uint32_t pin_int;                      ///< Interrupt pin number.
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
} drv_ltr303als_twi_cfg_t;

/**@brief Inits the ltr303als driver.
 */
uint32_t drv_ltr303als_init(void);

/**@brief Opens the ltr303als driver according to the specified configuration.
 *
 * @param[in]   p_twi_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_open(drv_ltr303als_twi_cfg_t const *const p_twi_cfg);

/**@brief Close the ltr303als driver.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_close(void);

/**@brief Configures the ltr303als sensor according to the specified configuration.
 *
 * @param[in]   p_cfg Pointer to the sensor configuration.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_cfg_set(drv_ltr303als_cfg_t const *const p_cfg);

/**@brief Reads the configuration of the ltr303als sensor.
 *
 * @param[in]   p_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_cfg_get(drv_ltr303als_cfg_t *p_cfg);

/**@brief Function to get the lux reading data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_light_get(double *p_lux);

/**@brief Function to convert uint16 value to float.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
// float drv_ltr303als_convert_light (uint16_t raw_value);

/**@brief Read and check the PART_ID register of the ltr303als sensor.
 *
 * @param[in]   who_am_i Pointer to store the data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_ltr303als_verify(uint8_t *who_am_i);

/**@brief Function to output data rate of the ltr303als.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
// uint32_t drv_ltr303als_odr_set (drv_ltr303als_odr_t odr);

#endif