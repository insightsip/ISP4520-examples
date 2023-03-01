/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf_error.h"
#include "sdk_common.h"

#include "app_scheduler.h"
#include "drv_lps22hb.h"
#include "drv_pressure.h"
#include "nrf_drv_gpiote.h"
#include "twi_manager.h"
#include <string.h>

/**@brief Pressure configuration struct.
 */
typedef struct
{
    drv_lps22hb_twi_cfg_t cfg;              ///< TWI configuraion.
    drv_pressure_evt_handler_t evt_handler; ///< Event handler called by gpiote_evt_sceduled.
    drv_pressure_mode_t mode;               ///< Mode of operation.
    bool enabled;                           ///< Driver enabled.
} drv_pressure_t;

/**@brief Stored configuration.
 */
static drv_pressure_t m_drv_pressure;

/**@brief GPIOTE sceduled handler, executed in main-context.
 */
static void gpiote_evt_sceduled(void *p_event_data, uint16_t event_size) {
    // Data ready
    drv_pressure_evt_t evt;
    evt.type = DRV_PRESSURE_EVT_DATA;
    evt.mode = DRV_PRESSURE_MODE_BAROMETER;

    m_drv_pressure.evt_handler(&evt);
}

/**@brief GPIOTE event handler, executed in interrupt-context.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    uint32_t err_code;

    if ((pin == m_drv_pressure.cfg.pin_int) && (nrf_gpio_pin_read(m_drv_pressure.cfg.pin_int) == 0)) {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Initialize the GPIO tasks and events system to catch pin data ready interrupts.
 */
static uint32_t gpiote_init(uint32_t pin) {
    uint32_t err_code;

    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        VERIFY_SUCCESS(err_code);
    }

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;
    gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    err_code = nrf_drv_gpiote_in_init(pin, &gpiote_in_config, gpiote_evt_handler);
    VERIFY_SUCCESS(err_code);

    nrf_drv_gpiote_in_event_enable(pin, true);

    return NRF_SUCCESS;
}

/**@brief Uninitialize the GPIO tasks and events system.
 */
static void gpiote_uninit(uint32_t pin) {
    nrf_drv_gpiote_in_uninit(pin);
}

uint32_t drv_pressure_init(drv_pressure_init_t *p_params) {
    uint32_t err_code;
    uint8_t who_am_i;

    VERIFY_PARAM_NOT_NULL(p_params);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_instance);
    VERIFY_PARAM_NOT_NULL(p_params->p_twi_cfg);
    VERIFY_PARAM_NOT_NULL(p_params->evt_handler);

    m_drv_pressure.mode = p_params->mode;
    m_drv_pressure.evt_handler = p_params->evt_handler;
    m_drv_pressure.cfg.twi_addr = LPS22HB_ADDR;
    m_drv_pressure.cfg.pin_int = p_params->pin_int;
    m_drv_pressure.cfg.p_twi_instance = p_params->p_twi_instance;
    m_drv_pressure.cfg.p_twi_cfg = p_params->p_twi_cfg;
    m_drv_pressure.enabled = false;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_verify(&who_am_i);
    if (err_code != NRF_SUCCESS) {
        err_code = drv_lps22hb_odr_set(DRV_LPS22HB_ODR_PowerDown);
    }

    // Release twi bus
    drv_lps22hb_close();

    return err_code;
}

uint32_t drv_pressure_enable(void) {
    uint32_t err_code;
    uint8_t status;
    uint16_t dummy_temp;
    uint32_t dummy_pressure;

    if (m_drv_pressure.enabled) {
        return NRF_SUCCESS;
    }
    m_drv_pressure.enabled = true;

    static const drv_lps22hb_cfg_t lps22hb_cfg =
        {
            .int_cfg_reg = ((INTERRUPT_CFG_REG_DEFAULT) | (INTERRUPT_CFG_REG_LIR_Enable << INTERRUPT_CFG_REG_LIR_Pos)),
            .int_threshold = 0,
            .ctrl_reg1 = CTRL_REG1_DEFAULT,
            .ctrl_reg2 = CTRL_REG2_DEFAULT,
            .ctrl_reg3 = (CTRL_REG3_DEFAULT | (CTRL_REG3_DRDY_Enable << CTRL_REG3_DRDY_Pos) | (CTRL_REG3_INT_H_L_ActiveLow << CTRL_REG3_INT_H_L_Pos) /* | (CTRL_REG3_INT_S_HighOrLow << CTRL_REG3_INT_S_Pos)*/),
            .fifo_ctrl = FIFO_CTRL_REG_DEFAULT,
            .res_conf = (RES_CONF_REG_DEFAULT | (RES_CONF_REG_LC_EN_Enable << RES_CONF_REG_LC_EN_Pos))};

    if (m_drv_pressure.cfg.pin_int != DRV_PRESSURE_PIN_NOT_USED) {
        err_code = gpiote_init(m_drv_pressure.cfg.pin_int);
        VERIFY_SUCCESS(err_code);
    }

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code |= drv_lps22hb_cfg_set(&lps22hb_cfg);

    err_code |= drv_lps22hb_status_get(&status);

    if (((status & STATUS_REG_P_DA_Msk) >> STATUS_REG_P_DA_Pos) == STATUS_REG_P_DA_Available) {
        err_code |= drv_lps22hb_pressure_get(&dummy_pressure);
    }

    if (((status & STATUS_REG_T_DA_Msk) >> STATUS_REG_T_DA_Pos) == STATUS_REG_T_DA_Available) {

        err_code |= drv_lps22hb_temperature_get(&dummy_temp);
    }

    // Release twi bus
    drv_lps22hb_close();

    return err_code;
}

uint32_t drv_pressure_disable(void) {
    uint32_t err_code;

    if (m_drv_pressure.enabled == false) {
        return NRF_SUCCESS;
    }
    m_drv_pressure.enabled = false;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_odr_set(DRV_LPS22HB_ODR_PowerDown);

    // Release twi bus
    drv_lps22hb_close();

    if (m_drv_pressure.cfg.pin_int != DRV_PRESSURE_PIN_NOT_USED) {
        gpiote_uninit(m_drv_pressure.cfg.pin_int);
    }

    return err_code;
}

uint32_t drv_pressure_reset(void) {
    // Not implemented. Reserved for later.
    return NRF_SUCCESS;
}

uint32_t drv_pressure_mode_set(drv_pressure_mode_t mode) {
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t drv_pressure_get(float *p_pressure) {
    uint32_t err_code;
    uint32_t pressure;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_pressure_get(&pressure);

    // Release twi bus
    drv_lps22hb_close();

    // p(hPa) = pressure(LSB) / 4096(LSB/hPa).
    *p_pressure = (((float)pressure) / 4096.0f);

    return err_code;
}

uint32_t drv_pressure_sample(void) {
    uint32_t err_code;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_one_shot();

    // Release twi bus
    drv_lps22hb_close();

    return err_code;
}

uint32_t drv_pressure_sleep(void) {
    uint32_t err_code;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_odr_set(DRV_LPS22HB_ODR_PowerDown);

    // Release twi bus
    drv_lps22hb_close();

    return err_code;
}

uint32_t drv_pressure_odr_set(drv_pressure_odr_t odr) {
    uint32_t err_code;

    // Request twi bus
    drv_lps22hb_open(&m_drv_pressure.cfg);

    err_code = drv_lps22hb_odr_set((drv_lps22hb_odr_t)odr);

    // Release twi bus
    drv_lps22hb_close();

    return err_code;
}