/******************************************************************************
 * @file    drv_lsm9ds1.c
 * @author  Insight SiP
 * @brief   lsm9ds1 driver implementation file.
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

#include "drv_lsm9ds1.h"
#include "nrf_delay.h"
#include "twi_manager.h"

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
    drv_lsm9ds1_twi_cfg_t const *p_cfg;
} m_lsm9ds1;

/**@brief Open the TWI bus for communication.
 */
static __inline uint32_t twi_open(void) {
    uint32_t err_code;

    err_code = twi_manager_request(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->p_twi_cfg,
        NULL,
        NULL);
    VERIFY_SUCCESS(err_code);

    nrf_drv_twi_enable(m_lsm9ds1.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module.
 */
static __inline uint32_t twi_close(void) {
    nrf_drv_twi_disable(m_lsm9ds1.p_cfg->p_twi_instance);

    nrf_drv_twi_uninit(m_lsm9ds1.p_cfg->p_twi_instance);

    return NRF_SUCCESS;
}

/**@brief Function for reading a an accelerometer register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t acc_reg_read(uint8_t reg_addr, uint8_t *p_reg_val) {
    uint32_t err_code;

    err_code = nrf_drv_twi_tx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->acc_twi_addr,
        &reg_addr,
        1,
        true);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_twi_rx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->acc_twi_addr,
        p_reg_val,
        1);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for writing to an accelerometer sensor register.
 *
 * @param[in]  reg_addr            Address of the register to write to.
 * @param[in]  reg_val             Value to write to the register.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t acc_reg_write(uint8_t reg_addr, uint8_t reg_val) {
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->acc_twi_addr,
        buffer,
        2,
        false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for reading a an magnetometer register.
 *
 * @param[in]  reg_addr            Address of the register to read.
 * @param[out] p_reg_val           Pointer to a buffer to receive the read value.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t mag_reg_read(uint8_t reg_addr, uint8_t *p_reg_val) {
    uint32_t err_code;

    err_code = nrf_drv_twi_tx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->mag_twi_addr,
        &reg_addr,
        1,
        true);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_drv_twi_rx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->mag_twi_addr,
        p_reg_val,
        1);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for writing to an magnetometer sensor register.
 *
 * @param[in]  reg_addr            Address of the register to write to.
 * @param[in]  reg_val             Value to write to the register.
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy.
 */
static uint32_t mag_reg_write(uint8_t reg_addr, uint8_t reg_val) {
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr, reg_val};

    err_code = nrf_drv_twi_tx(m_lsm9ds1.p_cfg->p_twi_instance,
        m_lsm9ds1.p_cfg->mag_twi_addr,
        buffer,
        2,
        false);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_init(void) {
    m_lsm9ds1.p_cfg = NULL;

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_open(drv_lsm9ds1_twi_cfg_t const *const p_cfg) {
    m_lsm9ds1.p_cfg = p_cfg;

    return twi_open();
}

uint32_t drv_lsm9ds1_close(void) {
    uint32_t err_code = twi_close();

    m_lsm9ds1.p_cfg = NULL;

    return err_code;
}

uint32_t drv_lsm9ds1_cfg_set(drv_lsm9ds1_cfg_t const *const p_cfg) {
    uint32_t err_code;

    err_code = acc_reg_write(CTRL_REG1_G, p_cfg->ctrl_reg1_g);
    VERIFY_SUCCESS(err_code);

    err_code = acc_reg_write(CTRL_REG3_G, p_cfg->ctrl_reg3_g);
    VERIFY_SUCCESS(err_code);

    err_code = acc_reg_write(CTRL_REG4, p_cfg->ctrl_reg4);
    VERIFY_SUCCESS(err_code);

    err_code = acc_reg_write(CTRL_REG5_XL, p_cfg->ctrl_reg5_xl);
    VERIFY_SUCCESS(err_code);

    err_code = acc_reg_write(CTRL_REG6_XL, p_cfg->ctrl_reg6_xl);
    VERIFY_SUCCESS(err_code);

    err_code = mag_reg_write(CTRL_REG1_M, p_cfg->ctrl_reg1_m);
    VERIFY_SUCCESS(err_code);

    err_code = mag_reg_write(CTRL_REG2_M, p_cfg->ctrl_reg2_m);
    VERIFY_SUCCESS(err_code);

    err_code = mag_reg_write(CTRL_REG3_M, p_cfg->ctrl_reg3_m);
    VERIFY_SUCCESS(err_code);

    err_code = mag_reg_write(CTRL_REG4_M, p_cfg->ctrl_reg4_m);
    VERIFY_SUCCESS(err_code);

    err_code = mag_reg_write(CTRL_REG5_M, p_cfg->ctrl_reg5_m);
    VERIFY_SUCCESS(err_code);

    err_code = acc_reg_write(CTRL_REG9, p_cfg->ctrl_reg9);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_cfg_get(drv_lsm9ds1_cfg_t *p_cfg) {
    // TO BE IMPLEMENTED

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_verify(void) {
    uint32_t err_code;
    uint8_t reg_val;
    bool found = false;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);

    // Verify Accelerometer device
    err_code = acc_reg_read(WHO_AM_I, &reg_val);
    VERIFY_SUCCESS(err_code);

    found = (reg_val == WHO_AM_I_DEFAULT) ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;

    // Verify Magnetometer device
    err_code = mag_reg_read(WHO_AM_I_M, &reg_val);
    VERIFY_SUCCESS(err_code);

    found |= (reg_val == WHO_AM_I_M_DEFAULT) ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;

    return found;
}

uint32_t drv_lsm9ds1_accelerometer_get(drv_lsm9ds1_acc_data_t *p_acc) {
    uint32_t err_code;
    uint8_t acc_l;
    uint8_t acc_h;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_acc);

    // Fetch X
    err_code = acc_reg_read(OUT_X_H_XL, &acc_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_X_L_XL, &acc_l);
    VERIFY_SUCCESS(err_code);
    p_acc->x = ((uint16_t)acc_h << 8) + acc_l;

    // Fetch Y
    err_code = acc_reg_read(OUT_Y_H_XL, &acc_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_Y_L_XL, &acc_l);
    VERIFY_SUCCESS(err_code);
    p_acc->y = ((uint16_t)acc_h << 8) + acc_l;

    // Fetch Z
    err_code = acc_reg_read(OUT_Z_H_XL, &acc_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_Z_L_XL, &acc_l);
    VERIFY_SUCCESS(err_code);
    p_acc->z = ((uint16_t)acc_h << 8) + acc_l;

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_gyroscope_get(drv_lsm9ds1_gyro_data_t *p_gyro) {
    uint32_t err_code;
    uint8_t gyro_l;
    uint8_t gyro_h;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_gyro);

    // Fetch X
    err_code = acc_reg_read(OUT_X_H_G, &gyro_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_X_L_G, &gyro_l);
    VERIFY_SUCCESS(err_code);
    p_gyro->x = ((uint16_t)gyro_h << 8) + gyro_l;

    // Fetch Y
    err_code = acc_reg_read(OUT_Y_H_G, &gyro_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_Y_L_G, &gyro_l);
    VERIFY_SUCCESS(err_code);
    p_gyro->y = ((uint16_t)gyro_h << 8) + gyro_l;

    // Fetch Z
    err_code = acc_reg_read(OUT_Z_H_G, &gyro_h);
    VERIFY_SUCCESS(err_code);
    err_code = acc_reg_read(OUT_Z_L_G, &gyro_l);
    VERIFY_SUCCESS(err_code);
    p_gyro->z = ((uint16_t)gyro_h << 8) + gyro_l;

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_magnetometer_get(drv_lsm9ds1_mag_data_t *p_mag) {
    uint32_t err_code;
    uint8_t mag_l;
    uint8_t mag_h;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);
    VERIFY_PARAM_NOT_NULL(p_mag);

    // Fetch X
    err_code = mag_reg_read(OUT_X_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OUT_X_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);
    p_mag->x = ((uint16_t)mag_h << 8) + mag_l;

    // Fetch Y
    err_code = mag_reg_read(OUT_Y_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OUT_Y_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);
    p_mag->y = ((uint16_t)mag_h << 8) + mag_l;

    // Fetch Z
    err_code = mag_reg_read(OUT_Z_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OUT_Z_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);
    p_mag->z = ((uint16_t)mag_h << 8) + mag_l;

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_reboot(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);

    // reboot Accelerometer/gyro
    err_code = acc_reg_write(CTRL_REG8, BITS_REBOOT);
    VERIFY_SUCCESS(err_code);

    // reboot Magnetometer
    err_code = acc_reg_write(CTRL_REG2_M, BITS_REBOOT_M);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_sw_reset(void) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);

    // reboot Accelerometer/gyro
    err_code = acc_reg_write(CTRL_REG8, BITS_SOFT_RST);
    VERIFY_SUCCESS(err_code);

    // reboot Magnetometer
    err_code = acc_reg_write(CTRL_REG2_M, BITS_SOFT_RST_M);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_offset_mag_set(int16_t offset_x, int16_t offset_y, int16_t offset_z) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);

    // Set X offset
    err_code = mag_reg_write(OFFSET_X_REG_L_M, offset_x & 0x00FF);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_write(OFFSET_X_REG_H_M, (offset_x & 0xFF00) >> 8);
    VERIFY_SUCCESS(err_code);

    // Set Y offset
    err_code = mag_reg_write(OFFSET_Y_REG_L_M, offset_y & 0x00FF);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_write(OFFSET_Y_REG_H_M, (offset_y & 0xFF00) >> 8);
    VERIFY_SUCCESS(err_code);

    // Set Z offset
    err_code = mag_reg_write(OFFSET_Z_REG_L_M, offset_z & 0x00FF);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_write(OFFSET_Z_REG_H_M, (offset_z & 0xFF00) >> 8);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_lsm9ds1_offset_mag_get(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z) {
    uint32_t err_code;

    DRV_CFG_CHECK(m_lsm9ds1.p_cfg);

    uint8_t mag_l;
    uint8_t mag_h;

    // Fetch X
    err_code = mag_reg_read(OFFSET_X_REG_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OFFSET_X_REG_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);

    *offset_x = ((uint16_t)mag_h << 8) + mag_l;

    // Fetch Y
    err_code = mag_reg_read(OFFSET_Y_REG_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OFFSET_Y_REG_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);

    *offset_y = ((uint16_t)mag_h << 8) + mag_l;

    // Fetch Z
    err_code = mag_reg_read(OFFSET_Z_REG_H_M, &mag_h);
    VERIFY_SUCCESS(err_code);
    err_code = mag_reg_read(OFFSET_Z_REG_L_M, &mag_l);
    VERIFY_SUCCESS(err_code);

    *offset_z = ((uint16_t)mag_h << 8) + mag_l;

    return NRF_SUCCESS;
}