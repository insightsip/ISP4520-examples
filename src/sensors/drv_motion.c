/******************************************************************************
 * @file    drv_motion.c
 * @author  Insight SiP
 * @brief   motion driver implementation file.
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

#include "nrf_delay.h"
#include "nrf_error.h"

#include "drv_lsm9ds1.h"
#include "drv_motion.h"

#include "nrf_log.h"

#define RETURN_IF_INV_ERROR(PARAM) \
    if ((PARAM) != INV_SUCCESS) {  \
        return NRF_ERROR_INTERNAL; \
    }

// calibration data
static bool m_is_magnetometer_calibrated = false;
static int16_t m_mag_offset[3] = {0};

/**@brief Motion configuration struct.
 */
typedef struct
{
    bool enabled;              ///< Driver enabled.
    drv_lsm9ds1_twi_cfg_t cfg; ///< TWI configuraion.
    bool running;
    drv_motion_feature_mask_t features;
    drv_motion_evt_handler_t evt_handler;
} drv_motion_t;

/**@brief Stored configuration.
 */
static drv_motion_t m_drv_motion;

uint32_t drv_motion_init(drv_motion_init_t *p_params) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_motion.evt_handler = p_params->evt_handler;
    m_drv_motion.cfg.acc_twi_addr = LSM9DS1_ACC_GYRO_ADDR;
    m_drv_motion.cfg.mag_twi_addr = LSM9DS1_MAG_ADDR;
    m_drv_motion.cfg.acc_pin_int = p_params->acc_pin_int;
    m_drv_motion.cfg.mag_pin_int = p_params->mag_pin_int;
    m_drv_motion.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_motion.cfg.p_twi_cfg = p_params->p_twi_cfg;
    m_drv_motion.enabled = false;
    m_drv_motion.features = 0;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_verify();
    drv_lsm9ds1_reboot();
    drv_lsm9ds1_offset_mag_set(0, 0, 0);

    // Release twi bus
    drv_lsm9ds1_close();

    //  reset_mag_calibration();

    return err_code;
}

uint32_t drv_motion_enable(drv_motion_feature_mask_t feature_mask) {
    uint32_t err_code;

    if ((feature_mask & ~(DRV_MOTION_FEATURE_MASK)) || (feature_mask == 0)) {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Set features bits.
    m_drv_motion.features |= feature_mask;

    if (m_drv_motion.enabled) {
        return NRF_SUCCESS;
    }
    m_drv_motion.enabled = true;

    static const drv_lsm9ds1_cfg_t lsm9ds1_cfg =
        {
            .ctrl_reg1_g = BITS_ODR_59_5HZ_G | BITS_FS_2000_G,
            .ctrl_reg3_g = BITS_LP_EN,
            .ctrl_reg4 = BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G,
            .ctrl_reg5_xl = BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL,
            .ctrl_reg6_xl = BITS_ODR_119HZ_XL,
            .ctrl_reg1_m = BITS_TEMP_COMP | BITS_OM_HIGH | BITS_ODR_M_80HZ,
            .ctrl_reg2_m = BITS_FS_M_4Gs,
            .ctrl_reg3_m = BITS_MD_CONTINUOUS,
            .ctrl_reg4_m = BITS_OMZ_HIGH,
            .ctrl_reg5_m = 0x00,
            .ctrl_reg9 = CTRL_REG9_DEFAULT};

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    drv_lsm9ds1_cfg_set(&lsm9ds1_cfg);

    // Release twi bus
    drv_lsm9ds1_close();

    return NRF_SUCCESS;
}

uint32_t drv_motion_disable(drv_motion_feature_mask_t feature_mask) {
    uint32_t err_code = NRF_SUCCESS;

    if ((feature_mask & ~(DRV_MOTION_FEATURE_MASK)) || (feature_mask == 0)) {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Clear feature bits
    m_drv_motion.features &= ~feature_mask;

    if (!m_drv_motion.features) {
        m_drv_motion.enabled = false;

        static const drv_lsm9ds1_cfg_t lsm9ds1_cfg =
            {
                .ctrl_reg1_g = CTRL_REG1_G_DEFAULT,
                .ctrl_reg3_g = BITS_LP_EN,
                .ctrl_reg4 = CTRL_REG4_DEFAULT,
                .ctrl_reg5_xl = CTRL_REG5_XL_DEFAULT,
                .ctrl_reg6_xl = CTRL_REG6_XL_DEFAULT,
                .ctrl_reg1_m = CTRL_REG1_M_DEFAULT,
                .ctrl_reg2_m = CTRL_REG2_M_DEFAULT,
                .ctrl_reg3_m = CTRL_REG3_M_DEFAULT,
                .ctrl_reg4_m = CTRL_REG4_M_DEFAULT,
                .ctrl_reg5_m = 0x00,
                .ctrl_reg9 = BITS_SLEEP_G};

        // Request twi bus
        drv_lsm9ds1_open(&m_drv_motion.cfg);

        err_code = drv_lsm9ds1_cfg_set(&lsm9ds1_cfg);

        // Release twi bus
        drv_lsm9ds1_close();
    }

    return err_code;
}

uint32_t drv_motion_config(drv_motion_cfg_t *p_cfg) {
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_cfg);

    if (m_drv_motion.enabled) {
        drv_motion_feature_mask_t enabled_features = m_drv_motion.features;

        err_code = drv_motion_disable(enabled_features);
        VERIFY_SUCCESS(err_code);

        return drv_motion_enable(enabled_features);
    } else {
        return NRF_SUCCESS;
    }
}

uint32_t drv_motion_raw_acc_get(drv_lsm9ds1_acc_data_t *acc_value) {
    uint32_t err_code = NRF_SUCCESS;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_accelerometer_get(acc_value);

    // Release twi bus
    drv_lsm9ds1_close();

    return err_code;
}

uint32_t drv_motion_raw_mag_get(drv_lsm9ds1_mag_data_t *mag_value) {
    uint32_t err_code = NRF_SUCCESS;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_magnetometer_get(mag_value);

    // Release twi bus
    drv_lsm9ds1_close();

    return err_code;
}

uint32_t drv_motion_raw_gyro_get(drv_lsm9ds1_gyro_data_t *gyro_value) {
    uint32_t err_code = NRF_SUCCESS;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_gyroscope_get(gyro_value);

    // Release twi bus
    drv_lsm9ds1_close();

    return err_code;
}

uint32_t drv_motion_offset_mag_set(int16_t offset_x, int16_t offset_y, int16_t offset_z) {
    uint32_t err_code = NRF_SUCCESS;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_offset_mag_set(offset_x, offset_y, offset_z);

    // Release twi bus
    drv_lsm9ds1_close();

    return err_code;
}

uint32_t drv_motion_offset_mag_get(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z) {
    uint32_t err_code = NRF_SUCCESS;

    // Request twi bus
    drv_lsm9ds1_open(&m_drv_motion.cfg);

    err_code = drv_lsm9ds1_offset_mag_get(offset_x, offset_y, offset_z);

    // Release twi bus
    drv_lsm9ds1_close();

    return err_code;
}