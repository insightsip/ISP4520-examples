/*
    2017: Distributed by Insight SiP

        THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
        OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
        OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
        DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
        LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
        CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
        GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
        HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
        LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
        OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __DRV_SHTC1_H
#define __DRV_SHTC1_H

// Includes
#include "nrf_drv_twi.h"
#include <stdbool.h>
#include <stdint.h>

#define SHTC1_ADDR 0x70 // Humidity address

// Defines
// CRC
#define POLYNOMIAL 0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

/**@brief Initialization struct for humid driver.
 */
typedef struct
{
    uint8_t twi_addr;                         ///< TWI address on bus.
    uint32_t pin_int;                         ///< Interrupt pin.
    nrf_drv_twi_t const *p_twi_instance;      ///< TWI instance.
    nrf_drv_twi_config_t const *p_twi_config; ///< TWI configuraion.
} drv_shtc1_twi_cfg_t;

//-- Enumerations --------------------------------------------------------------
// Sensor Commands
typedef enum {
    READ_ID = 0xEFC8,            // command: read ID register
    SOFT_RESET = 0x805D,         // soft reset
    MEAS_T_RH_POLLING = 0x7866,  // meas. read T first, clock stretching disabled
    MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
    MEAS_RH_T_POLLING = 0x58E0,  // meas. read RH first, clock stretching disabled
    MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
} etCommands;

// I2C address typedef
typedef enum {
    I2C_ADR_W = 0xE0, // sensor I2C address + write bit
    I2C_ADR_R = 0xE1  // sensor I2C address + read bit
} etI2cHeader;

// Error typedef
typedef enum {
    NO_ERROR = 0UL,
    ACK_ERROR = 1UL,
    CHECKSUM_ERROR = 4UL
} etError;

/**@brief Inits the shtc1 driver. */
uint32_t drv_shtc1_init(void);

/**@brief Opens the shtc1 driver according to the specified configuration.
 *
 * @param[in]   p_cfg   Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_open(drv_shtc1_twi_cfg_t const *const p_cfg);

/**@brief Close the shtc1 driver.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_close(void);

/**@brief Verify the shtc1 READ ID register.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_verify(void);

/**@brief Reboot the shtc1.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_reboot(void);

/**@brief Function to get the humidity and temperature data.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_temp_humi_get(float *p_temperature, float *p_humidity);

/**@brief Function to get the humidity and temperature raw data.
 *
 * @return NRF_SUCCESS    If the call was successful. */
uint32_t drv_shtc1_raw_temp_humi_get(uint16_t *rawValueTemp, uint16_t *rawValueHumi);

/**@brief Function to calculates checksum for n bytes of data and compares it with expected checksum.
 *
 * @param[in] data[]           checksum is built based on this data
 * @param[in] nbrOfBytes       checksum is built for n bytes of data
 * @param[in] checksum         expected checksum
 *
 * @return NRF_SUCCESS    		If the call was successful.
 * @return CHECKSUM_ERROR    	If the checksum does not match  */
uint32_t drv_shtc1_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);

/**@brief  Calculates the temperature [C] as a floating point value from the raw data that are read from the sensor.
 *
 * @param[in] rawValue         temperature raw value (16bit scaled)
 *
 * @return temperature [C] as a floating point value */
float drv_shtc1_CalcTemperature(uint16_t rawValue);

/**@brief Calculates the relative humidity [%RH] as a floating point value from the raw data that are read from the sensor.
 *
 * @param[in] rawValue        humidity raw value (16bit scaled)
 *
 * @return relative humidity [%RH] as a floating point value   */
float drv_shtc1_CalcHumidity(uint16_t rawValue);

#endif