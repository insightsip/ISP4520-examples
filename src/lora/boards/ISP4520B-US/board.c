/*!
 * \file      board.c
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
 
#include "board.h"

/**@brief Unique Devices IDs register set (nRF52)
 */
#define ID1     (0x10000060)
#define ID2     (0x10000064)

uint32_t lora_hardware_init (void)
{
    TimerRtcInit();

    SpiInit(&SX126x.Spi, PIN_LORA_MOSI, PIN_LORA_MISO, PIN_LORA_SCLK, NRF_SPI_PIN_NOT_CONNECTED);
    SX126xIoInit();

    return NRF_SUCCESS;
}

void lora_hardware_uninit (void)
{
    SpiDeInit(&SX126x.Spi);
    SX126xIoDeInit();
}

uint32_t BoardGetRandomSeed (void)
{
    return ((*(uint32_t*)ID1) ^ (*(uint32_t*)ID2));
}

void BoardGetUniqueId (uint8_t *id)
{
    id[7] = ( (*(uint32_t*)ID1) );
    id[6] = ( (*(uint32_t*)ID1) ) >> 8;
    id[5] = ( (*(uint32_t*)ID1) ) >> 16;
    id[4] = ( (*(uint32_t*)ID1) ) >> 24;
    id[3] = ( (*(uint32_t*)ID2) );
    id[2] = ( (*(uint32_t*)ID2) ) >> 8;
    id[1] = ( (*(uint32_t*)ID2) ) >> 16;
    id[0] = ( (*(uint32_t*)ID2) ) >> 24;
}

uint8_t BoardGetBatteryLevel (void)
{
    return 0;
}

char BoardGetRevision(void)
{
    return 'B';
}

void BoardCriticalSectionBegin(uint32_t *mask)
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd(uint32_t *mask)
{
    __set_PRIMASK( *mask );
}
