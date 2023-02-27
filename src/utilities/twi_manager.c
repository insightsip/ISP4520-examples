
#include "twi_manager.h"
#include "nrf_error.h"
#include "nrf_log.h"

static app_irq_priority_t s_context_limit = APP_IRQ_PRIORITY_HIGHEST;
static uint32_t s_collisions = 0;

uint32_t twi_manager_request(nrf_drv_twi_t const *p_instance,
    nrf_drv_twi_config_t const *p_config,
    nrf_drv_twi_evt_handler_t event_handler,
    void *p_context) {
    uint32_t err_code;
    uint8_t current_context = current_int_priority_get();

    //    if (current_context < s_context_limit)
    //    {
    //        NRF_LOG_ERROR("twi_manager_request: current_context < s_context_limit %d", current_context);
    //        return NRF_ERROR_FORBIDDEN;
    //    }

    err_code = nrf_drv_twi_init(p_instance,
        p_config,
        event_handler,
        p_context);
    if (err_code != NRF_SUCCESS) {
        s_collisions++;

        NRF_LOG_ERROR("twi_manager_request: collision %d", s_collisions);
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t twi_manager_release(nrf_drv_twi_t const *p_instance) {
    nrf_drv_twi_uninit(p_instance);

    return NRF_SUCCESS;
}

uint32_t twi_manager_collision_get(void) {
    return s_collisions;
}

uint32_t twi_manager_collision_reset(void) {
    s_collisions = 0;

    return NRF_SUCCESS;
}

uint32_t twi_manager_init(app_irq_priority_t context_limit) {
    s_context_limit = context_limit;
    s_collisions = 0;

    return NRF_SUCCESS;
}