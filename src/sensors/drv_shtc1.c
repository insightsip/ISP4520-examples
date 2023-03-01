/******************************************************************************
 * @file    drv_shtc1.c
 * @author  Insight SiP
 * @brief   shtc1 driver implementation file.
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

#include "drv_shtc1.h"
#include "nrf_delay.h"
#include "twi_manager.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/**@brief Check if the driver is open, if not return NRF_ERROR_INVALID_STATE.
 */
#define DRV_CFG_CHECK(PARAM)            \
    if ((PARAM) == NULL) {              \
        return NRF_ERROR_INVALID_STATE; \
    }

static struct
{
    drv_shtc1_twi_cfg_t const *p_cfg;
} m_shtc1;

/**@brief Function to init the TWI module when this driver needs to communicate on the TWI bus.
 */
static __inline uint32_t twi_open(void) {
    uint32_t err_code;

    err_code = twi_manager_request(m_shtc1.p_cfg->p_twi_instance,
        m_shtc1.p_cfg->p_twi_config,
        NULL,
        NULL);
    VERIFY_SUCCESS(err_code);

    nrf_drv_twi_enable(m_shtc1.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void) {
    nrf_drv_twi_disable(m_shtc1.p_cfg->p_twi_instance);

    nrf_drv_twi_uninit(m_shtc1.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function for reading a sensor register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 * @param[in] size_to_read         Number of bytes to read
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_read(uint16_t reg_addr, uint8_t *p_reg_val, uint8_t size_to_read) {
    uint32_t err_code;

    uint8_t command_bytes[2];
    command_bytes[0] = (reg_addr & 0xFF00) >> 8; // MSB command bytes
    command_bytes[1] = reg_addr & 0xFF;          // LSB Command Byte

    err_code = nrf_drv_twi_tx(m_shtc1.p_cfg->p_twi_instance,
        m_shtc1.p_cfg->twi_addr,
        command_bytes,
        2,
        true);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_twi_rx(m_shtc1.p_cfg->p_twi_instance,
        m_shtc1.p_cfg->twi_addr,
        p_reg_val,
        size_to_read);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for writing to a sensor register.
 *
 * @param[in]  value            command
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t reg_write(uint16_t value) {
    uint32_t err_code;

    uint8_t buffer[2] = {(value & 0xFF00) >> 8, value & 0x00FF};

    err_code = nrf_drv_twi_tx(m_shtc1.p_cfg->p_twi_instance,
        m_shtc1.p_cfg->twi_addr,
        buffer,
        2,
        false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_shtc1_init(void) {
    m_shtc1.p_cfg = NULL;

    return NRF_SUCCESS;
}

uint32_t drv_shtc1_open(drv_shtc1_twi_cfg_t const *const p_cfg) {
    m_shtc1.p_cfg = p_cfg;

    return twi_open();
}

uint32_t drv_shtc1_close(void) {
    uint32_t err_code = twi_close();

    m_shtc1.p_cfg = NULL;

    return err_code;
}

uint32_t drv_shtc1_verify(void) {
    uint32_t err_code;
    uint8_t idvalue[3];
    uint8_t checksum;

    DRV_CFG_CHECK(m_shtc1.p_cfg);

    // Soft reset
    // reg_write (SOFT_RESET);

    // check sensor ID
    err_code = reg_read(READ_ID, idvalue, 3);
    VERIFY_SUCCESS(err_code);

    checksum = idvalue[2];
    err_code = drv_shtc1_CheckCrc(idvalue, 0x2, checksum);

    if (((idvalue[1] & 0x3F) == 0x7) && err_code == NO_ERROR)
        return NRF_SUCCESS;
    else
        return NRF_ERROR_NOT_FOUND;
}

uint32_t drv_shtc1_reboot(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_shtc1.p_cfg);

    err_code = reg_write(SOFT_RESET);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_shtc1_temp_humi_get(float *p_temperature, float *p_humidity) {
    uint32_t err_code;
    uint8_t data[6];
    uint8_t temp_data[3], humi_data[3];
    uint8_t checksum[2];
    uint16_t rawValueTemp; // temperature raw value from sensor
    uint16_t rawValueHumi; // humidity raw value from sensor

    DRV_CFG_CHECK(m_shtc1.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_humidity);
    VERIFY_PARAM_NOT_NULL(p_temperature);

    // measure, read temperature first, clock streching enabled
    err_code = reg_read(MEAS_T_RH_CLOCKSTR, data, 6);
    VERIFY_SUCCESS(err_code);

    for (int i = 0; i < 2; i++) {
        temp_data[i] = data[i];
        humi_data[i] = data[i + 3];
        checksum[i] = data[2 + i * 3];
    }

    // Check CRC
    err_code &= drv_shtc1_CheckCrc(temp_data, 0x2, checksum[0]);
    err_code &= drv_shtc1_CheckCrc(humi_data, 0x2, checksum[1]);

    if (err_code == NO_ERROR) {
        rawValueTemp = temp_data[0] << 8 | temp_data[1];
        rawValueHumi = humi_data[0] << 8 | humi_data[1];

        *p_temperature = drv_shtc1_CalcTemperature(rawValueTemp);
        *p_humidity = drv_shtc1_CalcHumidity(rawValueHumi);
    }

    return NRF_SUCCESS;
}

uint32_t drv_shtc1_raw_temp_humi_get(uint16_t *rawValueTemp, uint16_t *rawValueHumi) {
    uint32_t err_code;
    uint8_t data[6];
    uint8_t temp_data[3], humi_data[3];
    uint8_t checksum[2];

    DRV_CFG_CHECK(m_shtc1.p_cfg);
    VERIFY_PARAM_NOT_NULL(rawValueHumi);
    VERIFY_PARAM_NOT_NULL(rawValueTemp);

    // measure, read temperature first, clock streching enabled
    err_code = reg_read(MEAS_T_RH_CLOCKSTR, data, 6);
    VERIFY_SUCCESS(err_code);

    for (int i = 0; i < 2; i++) {
        temp_data[i] = data[i];
        humi_data[i] = data[i + 3];
        checksum[i] = data[2 + i * 3];
    }

    // Check CRC
    err_code &= drv_shtc1_CheckCrc(temp_data, 0x2, checksum[0]);
    err_code &= drv_shtc1_CheckCrc(humi_data, 0x2, checksum[1]);

    if (err_code == NO_ERROR) {
        *rawValueTemp = temp_data[0] << 8 | temp_data[1];
        *rawValueHumi = humi_data[0] << 8 | humi_data[1];
    }

    return err_code;
}

uint32_t drv_shtc1_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum) {
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for (bit = 8; bit > 0; --bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }

    // verify checksum
    if (crc != checksum)
        return CHECKSUM_ERROR;
    else
        return NO_ERROR;
}

float drv_shtc1_CalcTemperature(uint16_t rawValue) {
    // calculate temperature [C]
    // T = -45 + 175 * rawValue / 2^16
    return (float)175 * (float)rawValue / (float)65536 - 45;
}

float drv_shtc1_CalcHumidity(uint16_t rawValue) {

    // calculate relative humidity [%RH]
    // RH = rawValue / 2^16 * 100
    return 100 * (float)rawValue / 65536;
}