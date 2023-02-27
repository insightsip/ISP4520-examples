/******************************************************************************
 * @file    drv_motion.h
 * @author  Insight SiP
 * @brief   motion driver header file.
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

#ifndef __DRV_MOTION_H__
#define __DRV_MOTION_H__

#include "drv_lsm9ds1.h"
#include "nrf_drv_twi.h"
#include <stddef.h>
#include <stdint.h>

#define LSM9DS1_ACC_GYRO_ADDR 0x6A // I2C Acc/Gyro
#define LSM9DS1_MAG_ADDR 0x1C      // I2C mag

// #define PI 3.14159f
#define DRV_MOTION_PIN_NOT_USED 0xFF

#define ACC_SCALE_2G 0.000061f
#define ACC_SCALE_4G 0.000122f
#define ACC_SCALE_8G 0.000244f
#define ACC_SCALE_16G 0.000732f

#define GYRO_SCALE_245DPS 0.00875f
#define GYRO_SCALE_500DPS 0.0175f
#define GYRO_SCALE_2000DPS 0.07f

#define MAG_SCALE_4Gs_IN_UT 0.01221f
#define MAG_SCALE_8Gs_IN_UT 0.02442f
#define MAG_SCALE_12Gs_IN_UT 0.03662f
#define MAG_SCALE_16Gs_IN_UT 0.04883f

#define INV_MAG_SCALE_4Gs_IN_UT 81.9f
#define INV_MAG_SCALE_8Gs_IN_UT 40.95f
#define INV_MAG_SCALE_12Gs_IN_UT 27.3075f
#define INV_MAG_SCALE_16Gs_IN_UT 20.4792f

/**@brief Motion event types.
 */
typedef enum {
    DRV_MOTION_EVT_RAW,
    DRV_MOTION_EVT_QUAT,
    DRV_MOTION_EVT_EULER,
    DRV_MOTION_EVT_ROT_MAT,
    DRV_MOTION_EVT_HEADING,
    DRV_MOTION_EVT_GRAVITY,
    DRV_MOTION_EVT_ANGULAR_ROTATION,
    DRV_MOTION_EVT_MAGNETIZATION
} drv_motion_evt_t;

/**@brief Motion driver event handler callback type.
 */
typedef void (*drv_motion_evt_handler_t)(drv_motion_evt_t const *p_evt, void *p_data, uint32_t size);

/**@brief Initialization struct for motion driver.
 */
typedef struct
{
    uint8_t acc_twi_addr;                  ///< Accelerometer/Gyroscope TWI address.
    uint8_t mag_twi_addr;                  ///< Magnetometer TWI address.
    uint32_t acc_pin_int;                  ///< Accelerometer/Gyroscope Interrupt pin.
    uint32_t mag_pin_int;                  ///< Magnetometer Interrupt pin.
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
    drv_motion_evt_handler_t evt_handler;  ///< Event handler - called after a pin interrupt has been detected.
} drv_motion_init_t;

/**@brief Motion features.
 */
typedef enum {
    DRV_MOTION_FEATURE_RAW_ACCEL,
    DRV_MOTION_FEATURE_RAW_GYRO,
    DRV_MOTION_FEATURE_RAW_COMPASS,
    DRV_MOTION_FEATURE_GRAVITY_VECTOR,
    DRV_MOTION_FEATURE_ANGULAR_ROTATION_VECTOR,
    DRV_MOTION_FEATURE_MAGNETIZATION_VECTOR,
    DRV_MOTION_FEATURE_QUAT,
    DRV_MOTION_FEATURE_EULER,
    DRV_MOTION_FEATURE_ROT_MAT,
    DRV_MOTION_FEATURE_HEADING
} drv_motion_feature_t;

typedef uint32_t drv_motion_feature_mask_t;

#define DRV_MOTION_FEATURE_MASK_RAW ((1UL << DRV_MOTION_FEATURE_RAW_ACCEL) | (1UL << DRV_MOTION_FEATURE_RAW_COMPASS) | (1UL << DRV_MOTION_FEATURE_RAW_GYRO))
#define DRV_MOTION_FEATURE_MASK_RAW_ACCEL (1UL << DRV_MOTION_FEATURE_RAW_ACCEL)
#define DRV_MOTION_FEATURE_MASK_RAW_GYRO (1UL << DRV_MOTION_FEATURE_RAW_GYRO)
#define DRV_MOTION_FEATURE_MASK_RAW_COMPASS (1UL << DRV_MOTION_FEATURE_RAW_COMPASS)
#define DRV_MOTION_FEATURE_MASK_QUAT (1UL << DRV_MOTION_FEATURE_QUAT)
#define DRV_MOTION_FEATURE_MASK_EULER (1UL << DRV_MOTION_FEATURE_EULER)
#define DRV_MOTION_FEATURE_MASK_ROT_MAT (1UL << DRV_MOTION_FEATURE_ROT_MAT)
#define DRV_MOTION_FEATURE_MASK_HEADING (1UL << DRV_MOTION_FEATURE_HEADING)
#define DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR (1UL << DRV_MOTION_FEATURE_GRAVITY_VECTOR)
#define DRV_MOTION_FEATURE_MASK_ANGULAR_VELOCITY_VECTOR (1UL << DRV_MOTION_FEATURE_ANGULAR_ROTATION_VECTOR)
#define DRV_MOTION_FEATURE_MASK_MAGNETIZATION_VECTOR (1UL << DRV_MOTION_FEATURE_MAGNETIZATION_VECTOR)

#define DRV_MOTION_FEATURE_MASK (DRV_MOTION_FEATURE_MASK_RAW_ACCEL |               \
                                 DRV_MOTION_FEATURE_MASK_RAW_GYRO |                \
                                 DRV_MOTION_FEATURE_MASK_RAW_COMPASS |             \
                                 DRV_MOTION_FEATURE_MASK_QUAT |                    \
                                 DRV_MOTION_FEATURE_MASK_EULER |                   \
                                 DRV_MOTION_FEATURE_MASK_ROT_MAT |                 \
                                 DRV_MOTION_FEATURE_MASK_HEADING |                 \
                                 DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR |          \
                                 DRV_MOTION_FEATURE_MASK_ANGULAR_VELOCITY_VECTOR | \
                                 DRV_MOTION_FEATURE_MASK_MAGNETIZATION_VECTOR)

/**@brief Motion configuration struct.
 */
typedef struct
{
    uint16_t motion_interval_ms;
} drv_motion_cfg_t;

/**@brief Function for initializing the motion driver.
 *
 * @param[in] evt_handler       motion param.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_init(drv_motion_init_t *p_params);

/**@brief Function for enabling features in the motion driver.
 *
 * @param[in] feature_mask      Feature mask telling what features to enable.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_enable(drv_motion_feature_mask_t feature_mask);

/**@brief Function to disable features in the motion driver.
 *
 * @param[in] feature_mask      Feature mask telling what features to disable.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_disable(drv_motion_feature_mask_t feature_mask);

/**@brief Function to configure the motion driver.
 *
 * @param[in] cfg      Configuration structure.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_config(drv_motion_cfg_t *p_cfg);

/**@brief Function to prepare for sleep mode.
 *
 * @param[in] wakeup   Boolean indicating wakeup or sleep for ever.
 *
 * @retval NRF_SUCCESS.
 */
// uint32_t drv_motion_sleep_prepare(bool wakeup);

/**@brief Function for getting the acceleration raw data.
 *
 * @param[in] acc_value   acceleration raw value
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_raw_acc_get(drv_lsm9ds1_acc_data_t *acc_value);

/**@brief Function for getting the magnetometer raw data.
 *
 * @param[in] mag_value   magnetic raw value
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_raw_mag_get(drv_lsm9ds1_mag_data_t *mag_value);

/**@brief Function for getting the magnetometer raw data.
 *
 * @param[in] gyro_value   gyro raw value
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_raw_gyro_get(drv_lsm9ds1_gyro_data_t *gyro_value);

/**@brief Function for setting the magnetometer offset.
 *
 * @param[in] offset_x   x offset value
 * @param[in] offset_y   y offset value
 * @param[in] offset_z   z offset value
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_offset_mag_set(int16_t offset_x, int16_t offset_y, int16_t offset_z);

/**@brief Function for getting the magnetometer offset.
 *
 * @param[in] offset_x   x offset value
 * @param[in] offset_y   y offset value
 * @param[in] offset_z   z offset value
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_offset_mag_get(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z);

#endif