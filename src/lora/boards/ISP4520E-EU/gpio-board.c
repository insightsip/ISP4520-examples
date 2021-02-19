/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

/******************************************************************************
 * @attention
 *      Modified work 2021 Insight SiP  
 *
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

#include "gpio-board.h"
#include "board.h"

static GpioIrqHandler *GpioIrq[32];


/** @brief GPIOTE event handler.
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t unused)
{
    if (GpioIrq[pin] != NULL)
    {
        GpioIrq[pin](NULL);
    }
}

void GpioInit(Gpio_t *obj, uint32_t pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value)
{
    nrf_gpio_pin_pull_t pull_config;

    if (pin >= NRF_NUM_GPIO_PINS)
        return;

    obj->pin = pin;
    obj->pull = type;
    
    if (mode == PIN_INPUT)
    {
        if (type == PIN_NO_PULL) 
            pull_config = NRF_GPIO_PIN_NOPULL;
        else if (type == PIN_PULL_UP) 
            pull_config = NRF_GPIO_PIN_PULLUP;
        else if (type == PIN_PULL_DOWN) 
            pull_config = NRF_GPIO_PIN_PULLDOWN;

        nrf_gpio_cfg_input (obj->pin, pull_config);
    }
    else // mode output
    {
        nrf_gpio_cfg_output (obj->pin);
    }

    // Sets initial output value
    if (mode == PIN_OUTPUT)
    {
        nrf_gpio_pin_write (pin, value);
    }
}


void GpioSetInterrupt (Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler)
{
    uint32_t err_code;

    if (obj->pin >= NRF_NUM_GPIO_PINS)
        return;

    if (irqHandler == NULL)
    {
        return;
    }

    GpioIrq[obj->pin] = irqHandler;

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher  = false;
    gpiote_in_config.hi_accuracy = false;

    // Check if gpiote lib is already initialized
    // Initiliaze it if not
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
    }

    // Configure pin number, polarity and pull configuration
    nrf_drv_gpiote_pin_t pin = obj->pin;

    if (irqMode == IRQ_RISING_EDGE)
    {
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    }
    else if (irqMode == IRQ_FALLING_EDGE)
    {
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    }
    else
    {
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
    }

    if (obj->pull == PIN_NO_PULL)
    {
        gpiote_in_config.pull = NRF_GPIO_PIN_NOPULL;
    }
    else if (obj->pull == PIN_PULL_UP)
    {
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;
    }
    else
    {
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    }

    err_code = nrf_drv_gpiote_in_init (pin, &gpiote_in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(pin, true);
}

void GpioRemoveInterrupt (Gpio_t *obj)
{
    if(obj->pin >= NRF_NUM_GPIO_PINS)
        return;
  
     nrf_drv_gpiote_in_event_disable(obj->pin);
     nrf_drv_gpiote_in_uninit(obj->pin);
}

void GpioWrite (Gpio_t *obj, uint32_t value)
{
    if(obj->pin >= NRF_NUM_GPIO_PINS)
        return;

    nrf_gpio_pin_write (obj->pin, value);
}

void GpioToggle (Gpio_t *obj)
{
    if(obj->pin >= NRF_NUM_GPIO_PINS)
        return;

    nrf_gpio_pin_toggle (obj->pin);
}

uint32_t GpioRead (Gpio_t *obj)
{
    if(obj->pin >= NRF_NUM_GPIO_PINS)
        return 0;

    return nrf_gpio_pin_read (obj->pin);
}
