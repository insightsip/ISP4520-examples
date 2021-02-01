/*!
 * \file      nvmm.c
 *
 * \brief     None volatile memory management module
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
 *              (C)2013-2020 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
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

#include <stdint.h>

#include "utilities.h"
#include "eeprom-board.h"
#include "nvmm.h"

uint16_t NvmmInit(void)
{
   return EepromMcuInit();
}

uint16_t NvmmWrite( uint8_t* src, uint16_t size, uint16_t offset )
{
    if( EepromMcuWriteBuffer(offset, src, size) == SUCCESS )
    {
        return size;
    }
    return 0;
}

uint16_t NvmmRead( uint8_t* dest, uint16_t size, uint16_t offset  )
{
    if( EepromMcuReadBuffer( offset, dest, size ) == SUCCESS )
    {
        return size;
    }
    return 0;
}

bool NvmmCrc32Check( uint16_t size, uint16_t offset )
{
    // Not required with the current EEPROM-board implementation
    // CRC is checked by the FDS library

    return true;
}

bool NvmmReset( uint16_t size, uint16_t offset )
{
    if( EepromMcuDeleteBuffer(offset) == SUCCESS )
    {
        return true;
    }
    return false;
}
