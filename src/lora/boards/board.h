/*!
 * \file      board.h
 *
 * \brief     Target board general functions implementation
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
 *      Modified work 2020 Insight SiP  
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
 

#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "nrf_drv_gpiote.h"

#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "nvmm.h"
#include "gpio-board.h"
#include "spi-board.h"


#include "radio.h"
#include "sx126x/sx126x.h"
#include "sx126x-board.h"


/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME  2

#define LORA_RESET 	PIN_LORA_RESET
#define SPI_INSTANCE 	0
#define NRF_NUM_GPIO_PINS       32

// External pin configuration
#define PIN_UART_RTS            5       // UART RTS for AT commands
#define PIN_UART_TX             6       // UART TX for AT commands
#define PIN_UART_CTS            7       // UART CTS for AT commands
#define PIN_UART_RX             8       // UART RX for AT commands
#define PIN_NVM_ERASE           12      // Non-volatile data erase

// Internal pin configuration
#define PIN_LORA_DIO_1          11      // LORA DIO_1
#define PIN_LORA_RESET          19      // LORA RESET
#define PIN_LORA_SCLK           23      // LORA SPI CLK
#define PIN_LORA_NSS            24      // LORA SPI CS
#define PIN_LORA_MISO           25      // LORA SPI MISO
#define PIN_LORA_MOSI           26      // LORA SPI MOSI
#define PIN_LORA_BUSY           27      // LORA SPI BUSY

/**@brief Initializes the target board peripherals.
 */
uint32_t lora_hardware_init(void);

/**@brief De-initializes the target board peripherals to decrease power consumption.
 */
void lora_hardware_uninit(void);

/**@brief Reset CPU.
 */
void BoardResetMcu(void);

/**@brief Returns a pseudo random seed generated using the MCU Unique ID
 *
 * @retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed(void);

/**@brief Gets the board 64 bits unique ID
 *
 * @param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId(uint8_t *id);

/**@brief   Get battery value
 */
uint8_t BoardGetBatteryLevel(void);

/**@brief   Get module revision 
 */
char BoardGetRevision(void);

/**@brief Disable interrupts
 *
 * @remark IRQ nesting is managed
 */
void BoardDisableIrq(void);

/**@brief Enable interrupts
 *
 * @remark IRQ nesting is managed
 */
void BoardEnableIrq(void);

#ifdef __cplusplus
}
#endif

#endif // __BOARD_H__
