 /******************************************************************************
 * @file    at_hal_transport.h
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

#ifndef AT_HAL_TRANSPORT_H__
#define AT_HAL_TRANSPORT_H__


#include <stdint.h>

/**@brief AT HAL Transport layer event types. */
typedef enum
{
    AT_HAL_TRANSP_EVT_TX_PKT_SENT = 0,     /**< An event indicating that TX packet has been transmitted. */
    AT_HAL_TRANSP_EVT_RX_PKT_RECEIVING,    /**< An event indicating that RX packet is being scheduled to receive or to drop. */
    AT_HAL_TRANSP_EVT_RX_PKT_RECEIVED,     /**< An event indicating that RX packet is ready for read. */
    AT_HAL_TRANSP_EVT_RX_PKT_DROPPED,      /**< An event indicating that RX packet was dropped because it was longer than available buffer. */
    AT_HAL_TRANSP_EVT_PHY_ERROR,           /**< An event indicating error on PHY layer. */
    AT_HAL_TRANSP_EVT_TYPE_MAX             /**< Enumeration upper bound. */
} at_hal_transport_evt_type_t;

/**@brief Serialization PHY layer error types. */
typedef enum
{
    AT_HAL_TRANSP_PHY_ERROR_RX_OVERFLOW = 0, /**< An error indicating that more information has been transmitted than the PHY module could handle. */
    AT_HAL_TRANSP_PHY_ERROR_TX_OVERREAD,     /**< An error indicating that the PHY module was forced to transmit more information than possessed. */
    AT_HAL_TRANSP_PHY_ERROR_HW_ERROR,        /**< An error indicating a hardware error in the PHY module. */
    AT_HAL_TRANSP_PHY_ERROR_TYPE_MAX         /**< Enumeration upper bound. */
} at_hal_transport_phy_error_type_t;

/**@brief Struct containing parameters of event of type @ref AT_HAL_TRANSP_EVT_RX_PKT_RECEIVED.
 */
typedef struct
{
    uint8_t * p_buffer;         /**< Pointer to a buffer containing a packet to read. */
    uint16_t  num_of_bytes;     /**< Length of a received packet in octets. */
} at_hal_transport_evt_rx_pkt_received_params_t;


/**@brief Struct containing parameters of event of type @ref AT_HAL_TRANSP_EVT_PHY_ERROR. */
typedef struct
{
    at_hal_transport_phy_error_type_t error_type;   /**< Type of the PHY error. */
    uint32_t hw_error_code;                         /**< Hardware error code - specific for a microcontroller. Parameter
                                                        is valid only for the PHY error of type @ref AT_HAL_TRANSP_PHY_ERROR_HW_ERROR. */
} at_hal_transport_evt_phy_error_params_t;

/**@brief Struct containing events from the  HAL Transport layer.
 *
 * @note  Some events do not have parameters, then the whole information is contained in the evt_type.
 */
typedef struct
{
    at_hal_transport_evt_type_t evt_type;   /**< Type of event. */
    union    /**< Union alternative identified by evt_type in the enclosing struct. */
    {
        at_hal_transport_evt_rx_pkt_received_params_t  rx_pkt_received; /**< Parameters of event of type @ref AT_HAL_TRANSP_EVT_RX_PKT_RECEIVED. */
        at_hal_transport_evt_phy_error_params_t        phy_error;       /**< Parameters of event of type @ref AT_HAL_TRANSP_EVT_PHY_ERROR. */
    } evt_params;
} at_hal_transport_evt_t;


/**@brief Generic callback function type to be used by all  HAL Transport layer events.
 *
 * @param[in] event     HAL Transport layer event.
 */
typedef void (*at_hal_transport_events_handler_t)(at_hal_transport_evt_t event);


/**@brief Function for opening and initializing the HAL Transport layer.
 *
 * @note The function opens the transport channel, initializes a PHY layer, and registers the callback
 *       function to be used by all HAL Transport layer events.
 *
 * @warning If the function has been already called, the function @ref at_hal_transport_close has
 *          to be called before at_hal_transport_open can be called again.
 *
 * @param[in] events_handler    Generic callback function to be used by all HAL Transport layer events.
 *
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. Hardware initialization parameters taken from
 *                                  the configuration file are wrong.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. The function has been already called. To call
 *                                  it again the function @ref at_hal_transport_close has to be called first.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t at_hal_transport_open(at_hal_transport_events_handler_t events_handler);

/**@brief Function for closing a transport channel.
 *
 * @note The function disables the hardware, resets internal module states, and unregisters the events
 *       callback function. Can be called multiple times, also for a channel that is not opened.
 */
void ser_hal_transport_close(void);

/**@brief Function for transmitting a packet.
 *
 * @note The function adds a packet pointed by the p_buffer parameter to a transmission queue. A buffer
 *       provided to this function must be allocated by the @ref at_hal_transport_tx_pkt_alloc function.
 *
 * @warning Completion of this method does not guarantee that actual peripheral transmission will be completed.
 *
 * @param[in] p_buffer        Pointer to the buffer to transmit.
 * @param[in] num_of_bytes    Number of octets to transmit. Must be more than 0.
 *
 * @retval NRF_SUCCESS              Operation success. Packet was added to the transmission queue.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_PARAM  Operation failure. num_of_bytes is equal to 0.
 * @retval NRF_ERROR_INVALID_ADDR   Operation failure. Not a valid pointer (provided address is not
 *                                  the starting address of a buffer managed by HAL Transport layer).
 * @retval NRF_ERROR_DATA_SIZE      Operation failure. Packet size exceeds limit.
 * @retval NRF_ERROR_BUSY           Operation failure. Transmission queue is full so packet was not
 *                                  added to the transmission queue.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Transmittion channel was not opened by
 *                                  @ref ser_hal_transport_open function or provided buffer was not
 *                                  allocated by @ref ser_hal_transport_tx_pkt_alloc function.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 */
uint32_t at_hal_transport_tx_pkt_send (const uint8_t * p_buffer, uint16_t num_of_bytes);

#endif