 /******************************************************************************
 * @file    at_phy.h
 * @author  Insight SiP
 * @version V1.0.0
 * @date    15-may-2019
 * @brief  hal transport phy level 
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

#ifndef AT_PHY_H__
#define AT_PHY_H__

#include <stdbool.h>
#include <string.h>
#include "app_error.h"

/**@brief Serialization PHY module event types. */
typedef enum
{
    AT_PHY_EVT_TX_PKT_SENT = 0,   /**< Obligatory to implement. An event indicating that a TX packet has been transmitted. */
    AT_PHY_EVT_RX_PKT_RECEIVED,   /**< Obligatory to implement. An event indicating that an RX packet has been successfully received. */
    AT_PHY_EVT_RX_PKT_DROPPED,    /**< Obligatory to implement. An event indicating that the RX packet
                                    *   receiving has been finished but the packet was discarded because
                                    *   it was longer than available the buffer. */

    AT_PHY_EVT_RX_OVERFLOW_ERROR, /**< Optional to implement. An event indicating that more
                                    *   information has been transmitted than the PHY module could
                                    *   handle. */
    AT_PHY_EVT_TX_OVERREAD_ERROR, /**< Optional to implement. An event indicating that the PHY module
                                    *   was forced to transmit more information than possessed. */
    AT_PHY_EVT_HW_ERROR,          /**< Optional to implement. An event indicating a hardware error in the PHY module.  */
    AT_PHY_EVT_TYPE_MAX           /**< Enumeration upper bound. */
} at_phy_evt_type_t;

/**@brief A struct containing parameters of event of type @ref AT_PHY_EVT_RX_PKT_RECEIVED. */
typedef struct
{
    uint8_t * p_buffer;     /**< Pointer to a buffer containing the received packet. */
    uint16_t  num_of_bytes; /**< Length of the received packet in octets. */
} at_phy_evt_rx_pkt_received_params_t;


/**@brief A struct containing parameters of event of type @ref AT_PHY_EVT_HW_ERROR. */
typedef struct
{
    uint32_t error_code; /**< Hardware error code - specific for a microcontroller. */
    uint8_t * p_buffer;  /**< Pointer to the buffer that was processed when error occured. */
} at_phy_evt_hw_error_params_t;

/**@brief A struct containing events from a PHY module.
 *
 * @note  Some events do not have parameters, then whole information is contained in the evt_type.
 */
typedef struct
{
    at_phy_evt_type_t                       evt_type; /**< Type of event. */

    union  /**< Union alternative identified by evt_type in enclosing struct. */
    {
        /** Parameters of event of type @ref AT_PHY_EVT_RX_PKT_RECEIVED. */
        at_phy_evt_rx_pkt_received_params_t rx_pkt_received;
        /** Parameters of the event of type @ref AT_PHY_EVT_HW_ERROR. */
        at_phy_evt_hw_error_params_t        hw_error;
    } evt_params;
} at_phy_evt_t;

/**@brief A type of generic callback function handler to be used by all PHY module events.
 *
 * @param[in] event    PHY module event.
 */
typedef void (*at_phy_events_handler_t)(at_phy_evt_t event);


/**@brief Function for opening and initializing the PHY module.
 *
 * @note  The function initializes hardware and internal module states, and registers callback
 *        function to be used by all PHY module events.
 *
 * @warning If the function has been already called, the function @ref at_phy_close has to be
 *          called before at_phy_open can be called again.
 *
 * @param[in] events_handler    Generic callback function handler to be used by all PHY module
 *                              events.
 *
 * @retval NRF_SUCCESS                Operation success.
 * @retval NRF_ERROR_INVALID_STATE    Operation failure. The function has been already called.
 *                                    To call it again, the function @ref at_phy_close has to be
 *                                    called first.
 * @retval NRF_ERROR_NULL             Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM    Operation failure. Hardware initialization parameters are not
 *                                    supported.
 */
uint32_t at_phy_open(at_phy_events_handler_t events_handler);

/**@brief Function for closing the PHY module.
 *
 * @note  The function disables hardware, resets internal module states, and unregisters the events
 *        callback function.
 */
void at_phy_close(void);

/**@brief Function for transmitting a packet.
 *
 * @note  The function adds a packet pointed by p_buffer parameter to a transmission queue and
 *        schedules generation of an event of type @ref AT_PHY_EVT_TX_PKT_SENT upon transmission
 *        completion.
 *
 * @param[in] p_buffer        Pointer to a buffer to transmit.
 * @param[in] num_of_bytes    Number of octets to transmit. Must be more than 0.
 *
 * @retval NRF_SUCCESS                Operation success. Packet was added to the transmission queue
 *                                    and event will be send upon transmission completion.
 * @retval NRF_ERROR_NULL             Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM    Operation failure. The num_of_bytes parameter equal to 0.
 * @retval NRF_ERROR_BUSY             Operation failure. Transmitting of a packet in progress.
 */
uint32_t at_phy_tx_pkt_send (uint8_t *p_buffer, uint8_t num_of_bytes);

#endif
