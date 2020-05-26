 /******************************************************************************
 * @file    at_hal_transport.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-may-2019
 * @brief  hal transport upper level
 *
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

#include <stdbool.h>
#include <string.h>
#include "app_error.h"
#include "at_phy.h"
#include "at_hal_transport.h"

#define AT_HAL_TRANSPORT_TX_MAX_PKT_SIZE    (384UL)
#define AT_HAL_TRANSPORT_RX_MAX_PKT_SIZE    (384UL)

/**
 * @brief States of the RX state machine.
 */
typedef enum
{
    HAL_TRANSP_RX_STATE_CLOSED = 0,
    HAL_TRANSP_RX_STATE_IDLE,
    HAL_TRANSP_RX_STATE_RECEIVED,
    HAL_TRANSP_RX_STATE_MAX
}at_hal_transp_rx_states_t;

/**
 * @brief States of the TX state machine.
 */
typedef enum
{
    HAL_TRANSP_TX_STATE_CLOSED = 0,
    HAL_TRANSP_TX_STATE_IDLE,
    HAL_TRANSP_TX_STATE_TRANSMITTED,
    HAL_TRANSP_TX_STATE_MAX
}at_hal_transp_tx_states_t;

/** @brief RX state. */
static at_hal_transp_rx_states_t m_rx_state = HAL_TRANSP_RX_STATE_CLOSED;

/** @brief TX state. */
static at_hal_transp_tx_states_t m_tx_state = HAL_TRANSP_TX_STATE_CLOSED;

/** @brief Callback function handler for HAL Transport layer events. */
static at_hal_transport_events_handler_t m_events_handler = NULL;

/**
 * @brief A callback function to be used to handle a PHY module events. This function is called in an interrupt context.
 */
static void phy_events_handler (at_phy_evt_t phy_event)
{
    uint32_t               err_code = 0;
    at_hal_transport_evt_t hal_transp_event;

    memset(&hal_transp_event, 0, sizeof (at_hal_transport_evt_t));
    hal_transp_event.evt_type = AT_HAL_TRANSP_EVT_TYPE_MAX;

    switch (phy_event.evt_type)
    {
        case AT_PHY_EVT_TX_PKT_SENT:
        {

            m_tx_state = HAL_TRANSP_TX_STATE_TRANSMITTED;
            /* An event to an upper layer that a packet has been transmitted. */
            hal_transp_event.evt_type = AT_HAL_TRANSP_EVT_TX_PKT_SENT;
            m_events_handler(hal_transp_event);

            break;
        }

        case AT_PHY_EVT_RX_PKT_RECEIVED:
        {
            m_rx_state = HAL_TRANSP_RX_STATE_RECEIVED;
            /* Generate the event to an upper layer. */
            hal_transp_event.evt_type = AT_HAL_TRANSP_EVT_RX_PKT_RECEIVED;
            hal_transp_event.evt_params.rx_pkt_received.p_buffer = phy_event.evt_params.rx_pkt_received.p_buffer;
            hal_transp_event.evt_params.rx_pkt_received.num_of_bytes = phy_event.evt_params.rx_pkt_received.num_of_bytes;
            m_events_handler(hal_transp_event);
            break;
        }

        case AT_PHY_EVT_HW_ERROR:
        {
            /* Generate the event to an upper layer. */
            hal_transp_event.evt_type                        = AT_HAL_TRANSP_EVT_PHY_ERROR;
            hal_transp_event.evt_params.phy_error.error_type = AT_HAL_TRANSP_PHY_ERROR_HW_ERROR;
            hal_transp_event.evt_params.phy_error.hw_error_code =  phy_event.evt_params.hw_error.error_code;
            m_events_handler(hal_transp_event);

            break;
        }

        default:
        {
            APP_ERROR_CHECK_BOOL(false);
            break;
        }
    }
}

uint32_t at_hal_transport_open (at_hal_transport_events_handler_t events_handler)
{
    uint32_t err_code = NRF_SUCCESS;

    if ((HAL_TRANSP_RX_STATE_CLOSED != m_rx_state) || (HAL_TRANSP_TX_STATE_CLOSED != m_tx_state))
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    else if (NULL == events_handler)
    {
        err_code = NRF_ERROR_NULL;
    }
    else
    {
        /* We have to change states before calling lower layer because ser_phy_open() function is
         * going to enable interrupts. On success an event from PHY layer can be emitted immediately
         * after return from ser_phy_open(). */
        m_rx_state = HAL_TRANSP_RX_STATE_IDLE;
        m_tx_state = HAL_TRANSP_TX_STATE_IDLE;

        m_events_handler = events_handler;

        /* Initialize a PHY module. */
        err_code = at_phy_open(phy_events_handler);

        if (NRF_SUCCESS != err_code)
        {
            m_rx_state       = HAL_TRANSP_RX_STATE_CLOSED;
            m_tx_state       = HAL_TRANSP_TX_STATE_CLOSED;
            m_events_handler = NULL;

            if (NRF_ERROR_INVALID_PARAM != err_code)
            {
                err_code = NRF_ERROR_INTERNAL;
            }
        }
    }

    return err_code;
}

void at_hal_transport_close(void)
{
    /* Reset generic handler for all events, reset internal states and close PHY module. */
    m_rx_state = HAL_TRANSP_RX_STATE_CLOSED;
    m_tx_state = HAL_TRANSP_TX_STATE_CLOSED;

    m_events_handler = NULL;

    at_phy_close();
}

uint32_t at_hal_transport_tx_pkt_send (const uint8_t * p_buffer, uint16_t num_of_bytes)
{
    uint32_t err_code = NRF_SUCCESS;

    if (NULL == p_buffer)
    {
        err_code = NRF_ERROR_NULL;
    }
    else if (0 == num_of_bytes)
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }

    err_code = at_phy_tx_pkt_send((uint8_t *)p_buffer, num_of_bytes);

    return err_code;
}