 /******************************************************************************
 * @file    at_phy_uart.c
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-may-2019
 * @brief  hal transport phy level (uart)
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
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_delay.h"
#include "board.h"

#define UART_TX_BUF_SIZE 256                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                                        /**< UART RX buffer size. */

static uint8_t tx_buffer[UART_TX_BUF_SIZE];
static uint8_t m_rx_buffer[UART_RX_BUF_SIZE];
static at_phy_events_handler_t m_at_phy_event_handler;
static at_phy_evt_t m_at_phy_rx_event;

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handler (app_uart_evt_t * p_event)
{
    at_phy_evt_t evt;
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&m_rx_buffer[index]));
            index++;

            if ((m_rx_buffer[index - 1] == '\r') || (m_rx_buffer[index - 1] == '\n') || (index >= (UART_RX_BUF_SIZE)))
            {
               // Generate AT_PHY_EVT_RX_PKT_RECEIVED event
                evt.evt_type = AT_PHY_EVT_RX_PKT_RECEIVED;
                evt.evt_params.rx_pkt_received.num_of_bytes = index;
                evt.evt_params.rx_pkt_received.p_buffer = m_rx_buffer;
                m_at_phy_event_handler(evt);

                index = 0;
            }
            break;

        case APP_UART_TX_EMPTY:
            // Generate AT_PHY_EVT_TX_PKT_SENT event
            evt.evt_type = AT_PHY_EVT_TX_PKT_SENT;
            m_at_phy_event_handler(evt);
            break;

        case APP_UART_COMMUNICATION_ERROR:
            // Generate AT_PHY_EVT_HW_ERROR event
            evt.evt_type = AT_PHY_EVT_HW_ERROR;
            m_at_phy_event_handler(evt);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

uint32_t at_phy_open(at_phy_events_handler_t events_handler)
{
    uint32_t err_code;

    if (events_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check if function was not called before.
    if (m_at_phy_event_handler != NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    // Initialize app_uart
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = PIN_UART_RX,
        .tx_pin_no    = PIN_UART_TX,
        .rts_pin_no   = PIN_UART_RTS,
        .cts_pin_no   = PIN_UART_CTS,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handler,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

    // Register event
    m_at_phy_event_handler = events_handler;

    return err_code;
}

void at_phy_close(void)
{
    // Unitialize app_uart
    app_uart_close();

    // Unregister event
    m_at_phy_event_handler = NULL;
}

uint32_t at_phy_tx_pkt_send (uint8_t *p_buffer, uint8_t num_of_bytes)
{
    for (uint32_t i=0; i<num_of_bytes; i++) 
    {
        while(app_uart_put(p_buffer[i]) != NRF_SUCCESS);
       // nrf_delay_us(500);
    }
}
