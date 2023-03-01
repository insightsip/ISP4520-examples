/******************************************************************************
 * @file    drv_lsm9ds1.h
 * @date    06-november-2018
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

#ifndef __DRV_LSM9DS1_H
#define __DRV_LSM9DS1_H

#include "nrf_drv_twi.h"
#include <stdbool.h>
#include <stdint.h>

// I2C Address
#define LSM9DS1_ACC_GYRO_I2C_ADDR 0xD4
#define LSM9DS1_MAG_I2C_ADDR 0x38

// Accelerometer and Gyro Register map
#define ACT_THS 0x04          // Default: 00000000 Type: r/w
#define ACT_DUR 0x05          // Default: 00000000 Type: r/w
#define INT_GEN_CFG_XL 0x06   // Default: 00000000 Type: r/w
#define INT_GEN_THS_X_XL 0x07 // Default: 00000000 Type: r/w
#define INT_GEN_THS_Y_XL 0x08 // Default: 00000000 Type: r/w
#define INT_GEN_THS_Z_XL 0x09 // Default: 00000000 Type: r/w
#define INT_GEN_DUR_XL 0x0A   // Default: 00000000 Type: r/w
#define REFERENCE_G 0x0B      // Default: 00000000 Type: r/w
#define INT1_CTRL 0x0C        // Default: 00000000 Type: r/w
#define INT2_CTRL 0x0D        // Default: 00000000 Type: r/w
#define WHO_AM_I 0x0F         // Default: 01101000 Type: r
#define CTRL_REG1_G 0x10      // Default: 00000000 Type: r/w
#define CTRL_REG2_G 0x11      // Default: 00000000 Type: r/w
#define CTRL_REG3_G 0x12      // Default: 00000000 Type: r/w
#define ORIENT_CFG_G 0x13     // Default: 00000000 Type: r/w
#define INT_GEN_SRC_G 0x14    // output 		  Type: r
#define OUT_TEMP_L 0x15       // output 		  Type: r
#define OUT_TEMP_H 0x16       // output 		  Type: r
#define STATUS_REG_G 0x17     // output 		  Type: r
#define OUT_X_L_G 0x18        // output 		  Type: r
#define OUT_X_H_G 0x19        // output 		  Type: r
#define OUT_Y_L_G 0x1A        // output 		  Type: r
#define OUT_Y_H_G 0x1B        // output 		  Type: r
#define OUT_Z_L_G 0x1C        // output 		  Type: r
#define OUT_Z_H_G 0x1D        // output 		  Type: r
#define CTRL_REG4 0x1E        // Default: 00111000 Type: r/w
#define CTRL_REG5_XL 0x1F     // Default: 00111000 Type: r/w
#define CTRL_REG6_XL 0x20     // Default: 00000000 Type: r/w
#define CTRL_REG7_XL 0x21     // Default: 00000000 Type: r/w
#define CTRL_REG8 0x22        // Default: 00000100 Type: r/w
#define CTRL_REG9 0x23        // Default: 00000000 Type: r/w
#define CTRL_REG10 0x24       // Default: 00000000 Type: r/w
#define INT_GEN_SRC_XL 0x26   // output 		  Type: r
#define STATUS_REG_XL 0x27    // output 		  Type: r
#define OUT_X_L_XL 0x28       // output 		  Type: r
#define OUT_X_H_XL 0x29       // output 		  Type: r
#define OUT_Y_L_XL 0x2A       // output 		  Type: r
#define OUT_Y_H_XL 0x2B       // output 		  Type: r
#define OUT_Z_L_XL 0x2C       // output 		  Type: r
#define OUT_Z_H_XL 0x2D       // output 		  Type: r
#define FIFO_CTRL 0x2E        // Default: 00000000 Type: r/w
#define FIFO_SRC 0x2F         // output 		  Type: r
#define INT_GEN_CFG_G 0x30    // Default: 00000000 Type: r/w
#define INT_GEN_THS_XH_G 0x31 // Default: 00000000 Type: r/w
#define INT_GEN_THS_XL_G 0x32 // Default: 00000000 Type: r/w
#define INT_GEN_THS_YH_G 0x33 // Default: 00000000 Type: r/w
#define INT_GEN_THS_YL_G 0x34 // Default: 00000000 Type: r/w
#define INT_GEN_THS_ZH_G 0x35 // Default: 00000000 Type: r/w
#define INT_GEN_THS_ZL_G 0x36 // Default: 00000000 Type: r/w
#define INT_GEN_DUR_G 0x37    // Default: 00000000 Type: r/w

// Magnetometer Register map
#define OFFSET_X_REG_L_M 0x05 // Default: 00000000 Type: r/w
#define OFFSET_X_REG_H_M 0x06 // Default: 00000000 Type: r/w
#define OFFSET_Y_REG_L_M 0x07 // Default: 00000000 Type: r/w
#define OFFSET_Y_REG_H_M 0x08 // Default: 00000000 Type: r/w
#define OFFSET_Z_REG_L_M 0x09 // Default: 00000000 Type: r/w
#define OFFSET_Z_REG_H_M 0x0A // Default: 00000000 Type: r/w
#define WHO_AM_I_M 0x0F       // Default: 00111101 Type: r
#define CTRL_REG1_M 0x20      // Default: 00010000 Type: r/w
#define CTRL_REG2_M 0x21      // Default: 00000000 Type: r/w
#define CTRL_REG3_M 0x22      // Default: 00000011 Type: r/w
#define CTRL_REG4_M 0x23      // Default: 00000000 Type: r/w
#define CTRL_REG5_M 0x24      // Default: 00000000 Type: r/w
#define STATUS_REG_M 0x27     // Output 		  Type: r
#define OUT_X_L_M 0x28        // Output 		  Type: r
#define OUT_X_H_M 0x29        // Output 		  Type: r
#define OUT_Y_L_M 0x2A        // Output 		  Type: r
#define OUT_Y_H_M 0x2B        // Output 		  Type: r
#define OUT_Z_L_M 0x2C        // Output 		  Type: r
#define OUT_Z_H_M 0x2D        // Output 		  Type: r
#define INT_CFG_M 0x30        // Default: 00001000 Type: rw
#define INT_SRC_M 0x31        // Default: 00000000 Type: r
#define INT_THS_L_M 0x32      // Default: 00000000 Type: r
#define INT_THS_H_M 0x33      // Default: 00000000 Type: r

// Configuration Values

/* CTRL_REG1_G Values */
#define CTRL_REG1_G_DEFAULT 0x00
#define BITS_ODR_PD_G (0x0 << 5) // Output Data rate bits [7:5]
#define BITS_ODR_14_9HZ_G (0x1 << 5)
#define BITS_ODR_59_5HZ_G (0x2 << 5)
#define BITS_ODR_119HZ_G (0x3 << 5)
#define BITS_ODR_238HZ_G (0x4 << 5)
#define BITS_ODR_476HZ_G (0x5 << 5)
#define BITS_ODR_952HZ_G (0x6 << 5)
#define BITS_FS_246_G (0x0 << 3) // Full Scale bits [4:3]
#define BITS_FS_500_G (0x1 << 3)
#define BITS_FS_2000_G (0x3 << 3)
#define BITS_BW_00_G (0x0 << 0) // Bandwidth bits [1:0]
#define BITS_BW_01_G (0x1 << 0)
#define BITS_BW_10_G (0x2 << 0)
#define BITS_BW_11_G (0x3 << 0)

/* CTRL_REG3_G Values */
#define CTRL_REG3_G_DEFAULT 0x00
#define BITS_HP_EN (0x1 << 6)
#define BITS_LP_EN (0x1 << 7)

/* CTRL_REG4 Values */
#define CTRL_REG4_DEFAULT 0x38
#define BITS_ZEN_G (0x1 << 5)
#define BITS_YEN_G (0x1 << 4)
#define BITS_XEN_G (0x1 << 3)
#define BITS_LIR_XL1 (0x1 << 1)
#define BITS_4D_XL1			0x1 << 0)

/* CTRL_REG5_XL Values */
#define CTRL_REG5_XL_DEFAULT 0x38
#define BITS_ZEN_XL (0x1 << 5)
#define BITS_YEN_XL (0x1 << 4)
#define BITS_XEN_XL (0x1 << 3)
#define BITS_DEC_1 (0x1 << 6)
#define BITS_DEC_0 (0x1 << 7)

/* CTRL_REG6_XL Values */
#define CTRL_REG6_XL_DEFAULT 0x0
#define BITS_ODR_PD_XL (0x0 << 5) // Output Data rate bits [7:4]
#define BITS_ODR_10HZ_XL (0x1 << 5)
#define BITS_ODR_50HZ_XL (0x2 << 5)
#define BITS_ODR_119HZ_XL (0x3 << 5)
#define BITS_ODR_238HZ_XL (0x4 << 5)
#define BITS_ODR_476HZ_XL (0x5 << 5)
#define BITS_ODR_952HZ_XL (0x6 << 5)
#define BITS_FS_2_XL (0x0 << 3) // Full Scale bits [4:3]
#define BITS_FS_16_XL (0x1 << 3)
#define BITS_FS_4_XL (0x2 << 3)
#define BITS_FS_8_XL (0x3 << 3)
#define BITS_BW_SCAL_ODR (0x1 << 2) // Scale Bandwidth with ODR bit 2
#define BITS_BW_408_XL (0x0 << 0)   // Bandwidth bits [1:0]
#define BITS_BW_211_XL (0x1 << 0)
#define BITS_BW_105_XL (0x2 << 0)
#define BITS_BW_50_XL (0x3 << 0)

/* CTRL_REG8 Values */
#define BITS_SOFT_RST (0x1 << 0)
#define BITS_REBOOT (0x1 << 7)

/* CTRL_REG1_M Values */
#define CTRL_REG1_M_DEFAULT 0x10
#define BITS_TEMP_COMP (0x1 << 7)
#define BITS_OM_LOW (0x0 << 5)
#define BITS_OM_MED (0x1 << 5)
#define BITS_OM_HIGH (0x2 << 5)
#define BITS_OM_ULTRA (0x3 << 5)
#define BITS_ODR_M_0_625HZ (0x0 << 2)
#define BITS_ODR_M_1_25HZ (0x1 << 2)
#define BITS_ODR_M_5_5HZ (0x2 << 2)
#define BITS_ODR_M_5HZ (0x3 << 2)
#define BITS_ODR_M_10HZ (0x4 << 2)
#define BITS_ODR_M_20HZ (0x5 << 2)
#define BITS_ODR_M_40HZ (0x6 << 2)
#define BITS_ODR_M_80HZ (0x7 << 2)

/* CTRL_REG2_M Values */
#define CTRL_REG2_M_DEFAULT 0x0
#define BITS_SOFT_RST_M (0x1 << 2)
#define BITS_REBOOT_M (0x1 << 3)
#define BITS_FS_M_4Gs (0x0 << 5)
#define BITS_FS_M_8Gs (0x1 << 5)
#define BITS_FS_M_12Gs (0x2 << 5)
#define BITS_FS_M_16Gs (0x3 << 5)

/* CTRL_REG3_M Values */
#define CTRL_REG3_M_DEFAULT 0x3
#define BITS_MD_CONTINUOUS (0x0 << 0)
#define BITS_MD_SINGLE (0x1 << 0)
#define BITS_MD_PD1 (0x2 << 0)
#define BITS_MD_PD2 (0x3 << 0)

/* CTRL_REG4_M Values */
#define CTRL_REG4_M_DEFAULT 0x0
#define BITS_OMZ_LOW (0x0 << 2)
#define BITS_OMZ_MED (0x1 << 2)
#define BITS_OMZ_HIGH (0x2 << 2)
#define BITS_OMZ_ULTRA (0x3 << 2)

/* CTRL_REG9 Values */
#define CTRL_REG9_DEFAULT 0x0
#define BITS_STOP_ON_FTH (0x1 << 0)
#define BITS_FIFO_EN (0x1 << 1)
#define BITS_I2C_DIS (0x1 << 2)
#define BITS_DRDY_EN (0x1 << 3)
#define BITS_FIFO_TEMP_EN (0x1 << 4)
#define BITS_SLEEP_G (0x1 << 6)

#define WHO_AM_I_DEFAULT 0x68
#define WHO_AM_I_M_DEFAULT 0x3D

/**@brief Configuration struct for lsm9ds1 acc sensor.
 */
typedef struct
{
    uint8_t ctrl_reg1_g;
    uint8_t ctrl_reg2_g;
    uint8_t ctrl_reg3_g;
    int8_t ctrl_reg4;
    uint8_t ctrl_reg5_xl;
    uint8_t ctrl_reg6_xl;
    uint8_t ctrl_reg1_m;
    uint8_t ctrl_reg2_m;
    uint8_t ctrl_reg3_m;
    uint8_t ctrl_reg4_m;
    uint8_t ctrl_reg5_m;
    uint8_t ctrl_reg9;
} drv_lsm9ds1_cfg_t;

/**@brief Initialization struct for lsm9ds1 accelerometer sensor driver.
 */
typedef struct
{
    uint8_t acc_twi_addr;                  ///< Accelerometer TWI address.
    uint8_t mag_twi_addr;                  ///< Magnetometer TWI address.
    uint32_t acc_pin_int;                  ///< Accelerometer Interrupt pin number.
    uint32_t mag_pin_int;                  ///< Magnetometer Interrupt pin number.
    nrf_drv_twi_t const *p_twi_instance;   ///< The instance of TWI master to be used for transactions.
    nrf_drv_twi_config_t const *p_twi_cfg; ///< The TWI configuration to use while the driver is enabled.
} drv_lsm9ds1_twi_cfg_t;

typedef enum {
    LowPowerMode = 0x0,
    MediumPerformanceMode = 0x1,
    HighPerformanceMode = 0x2,
    UltraHighPerformanceMode = 0x3
} drv_lsm9ds1_MAG_Mode_t;

typedef struct drv_lsm9ds1_acc_data {
    int16_t x;
    int16_t y;
    int16_t z;
} drv_lsm9ds1_acc_data_t;

typedef struct drv_lsm9ds1_gyro_data {
    int16_t x;
    int16_t y;
    int16_t z;
} drv_lsm9ds1_gyro_data_t;

typedef struct drv_lsm9ds1_mag_data {
    int16_t x;
    int16_t y;
    int16_t z;
} drv_lsm9ds1_mag_data_t;

/**@brief Inits the lsm9ds1 driver.
 */
uint32_t drv_lsm9ds1_init(void);

/**@brief Opens the lsm9ds1 driver according to the specified configuration.
 *
 * @param[in]   p_twi_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_open(drv_lsm9ds1_twi_cfg_t const *const p_twi_cfg);

/**@brief Close the lsm9ds1 driver.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_close(void);

/**@brief Read and check the IDs register of the lsm9ds1 sensor.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_verify(void);

/**@brief Configures the lsm9ds1 sensor according to the specified configuration.
 *
 * @param[in]   p_cfg Pointer to the sensor configuration.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_cfg_set(drv_lsm9ds1_cfg_t const *const p_cfg);

/**@brief Reads the configuration of the lsm9ds1 sensor.
 *
 * @param[in]   p_cfg Pointer to the driver configuration for the session to be opened.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_cfg_get(drv_lsm9ds1_cfg_t *p_cfg);

/**@brief Function to get the acceleration data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_accelerometer_get(drv_lsm9ds1_acc_data_t *pAccValue);

/**@brief Function to get the Gyroscope data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_gyroscope_get(drv_lsm9ds1_gyro_data_t *pGyroValue);

/**@brief Function to get the Magnetic data.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_magnetometer_get(drv_lsm9ds1_mag_data_t *p_MagValue);

/**@brief Function to reboot memory content.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_reboot(void);

/**@brief Function to perform software reset.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_sw_reset(void);

/**@brief Function to set magnetic hard-iron offset.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_offset_mag_set(int16_t offset_x, int16_t offset_y, int16_t offset_z);

/**@brief Function to get magnetic hard-iron offset.
 *
 * @return NRF_SUCCESS    If the call was successful.
 */
uint32_t drv_lsm9ds1_offset_mag_get(int16_t *offset_x, int16_t *offset_y, int16_t *offset_z);

#endif