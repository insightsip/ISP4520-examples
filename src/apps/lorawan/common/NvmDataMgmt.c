/*!
 * \file      NvmDataMgmt.c
 *
 * \brief     NVM context management implementation
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
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

#include <stdio.h>
#include "utilities.h"
#include "nvmm.h"
#include "LoRaMac.h"
#include "NvmDataMgmt.h"

/*!
 * Enables/Disables the context storage management storage.
 * Must be enabled for LoRaWAN 1.0.4 or later.
 */
#ifndef CONTEXT_MANAGEMENT_ENABLED
#define CONTEXT_MANAGEMENT_ENABLED         1
#endif

#define NVM_ID_CRYPTO               1   /**< ID for NVM group crypto. */
#define NVM_ID_MAC_GROUP1           2   /**< ID for NVM group MAC 1. */
#define NVM_ID_MAC_GROUP2           3   /**< ID for NVM group MAC 2. */
#define NVM_ID_SECURE_ELEMENT       4   /**< ID for NVM group secure element. */
#define NVM_ID_REGION_GROUP1        5   /**< ID for NVM group 1 region. */
#define NVM_ID_REGION_GROUP2        6   /**< ID for NVM group 2 region. */
#define NVM_ID_CLASS_B              7   /**< ID for NVM group class b. */


static uint16_t NvmNotifyFlags = 0;

void NvmDataMgmtEvent( uint16_t notifyFlags )
{
    NvmNotifyFlags = notifyFlags;
}

uint16_t NvmDataMgmtInit( void )
{
#if( CONTEXT_MANAGEMENT_ENABLED == 1 )
    return NvmmInit();
#else
    return 0;
#endif
}

uint16_t NvmDataMgmtStore( void )
{
#if(CONTEXT_MANAGEMENT_ENABLED == 1)
    uint16_t dataSize = 0;
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm( &mibReq );
    LoRaMacNvmData_t* nvm = mibReq.Param.Contexts;

    // Input checks
    if( NvmNotifyFlags == LORAMAC_NVM_NOTIFY_FLAG_NONE )
    {
        // There was no update.
        return 0;
    }
    if( LoRaMacStop( ) != LORAMAC_STATUS_OK )
    {
        return 0;
    }

    // Crypto
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_CRYPTO) ==  LORAMAC_NVM_NOTIFY_FLAG_CRYPTO )
    {
        dataSize += NvmmWrite( (uint8_t*)&nvm->Crypto, sizeof(nvm->Crypto), NVM_ID_CRYPTO );
    }

    // MacGroup1
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_MAC_GROUP1) == LORAMAC_NVM_NOTIFY_FLAG_MAC_GROUP1 )
    {
        dataSize += NvmmWrite( (uint8_t*)&nvm->MacGroup1, sizeof(nvm->MacGroup1), NVM_ID_MAC_GROUP1 );
    }

    // MacGroup2
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_MAC_GROUP2) == LORAMAC_NVM_NOTIFY_FLAG_MAC_GROUP2 )
    {
        dataSize += NvmmWrite( (uint8_t*)&nvm->MacGroup2, sizeof(nvm->MacGroup2), NVM_ID_MAC_GROUP2 );
    }

    // Secure element
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_SECURE_ELEMENT) ==  LORAMAC_NVM_NOTIFY_FLAG_SECURE_ELEMENT )
    {
        dataSize += NvmmWrite( (uint8_t*)&nvm->SecureElement, sizeof(nvm->SecureElement), NVM_ID_SECURE_ELEMENT );
    }

    // Region group 1
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_REGION_GROUP1) == LORAMAC_NVM_NOTIFY_FLAG_REGION_GROUP1 )
    {
        dataSize += NvmmWrite( (uint8_t*)&nvm->RegionGroup1, sizeof(nvm->RegionGroup1), NVM_ID_REGION_GROUP1 );
    }

    // Region group 2
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_REGION_GROUP2) == LORAMAC_NVM_NOTIFY_FLAG_REGION_GROUP2 )
    {
        dataSize += NvmmWrite( (uint8_t*) &nvm->RegionGroup2, sizeof(nvm->RegionGroup2), NVM_ID_REGION_GROUP2 );
    }

    // Class b
    if( (NvmNotifyFlags & LORAMAC_NVM_NOTIFY_FLAG_CLASS_B) == LORAMAC_NVM_NOTIFY_FLAG_CLASS_B )
    {
        dataSize += NvmmWrite( (uint8_t*) &nvm->ClassB, sizeof(nvm->ClassB), NVM_ID_CLASS_B );
    }

    // Reset notification flags
    NvmNotifyFlags = LORAMAC_NVM_NOTIFY_FLAG_NONE;

    // Resume LoRaMac
    LoRaMacStart();
    return dataSize;
#else
    return 0;
#endif
}

uint16_t NvmDataMgmtRestore( void )
{
#if( CONTEXT_MANAGEMENT_ENABLED == 1 )
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm( &mibReq );
    LoRaMacNvmData_t* nvm = mibReq.Param.Contexts;
    uint16_t dataSize = 0;

    // CRC Check if done by FDS - No need for NvmmCrc32Check

    // Crypto
    if( NvmmRead((uint8_t*)&nvm->Crypto, sizeof(LoRaMacCryptoNvmData_t), NVM_ID_CRYPTO) == sizeof(LoRaMacCryptoNvmData_t) )
    {
        dataSize += sizeof(LoRaMacCryptoNvmData_t);
    }

    // Mac Group 1
    if( NvmmRead((uint8_t*)&nvm->MacGroup1, sizeof(LoRaMacNvmDataGroup1_t), NVM_ID_MAC_GROUP1) == sizeof(LoRaMacNvmDataGroup1_t) )
    {
        dataSize += sizeof(LoRaMacNvmDataGroup1_t);
    }

    // Mac Group 2
    if( NvmmRead((uint8_t*)&nvm->MacGroup2, sizeof(LoRaMacNvmDataGroup2_t), NVM_ID_MAC_GROUP2) == sizeof(LoRaMacNvmDataGroup2_t) )
    {
        dataSize += sizeof(LoRaMacNvmDataGroup2_t);
    }

    // Secure element
    if( NvmmRead((uint8_t*)&nvm->SecureElement, sizeof(SecureElementNvmData_t), NVM_ID_SECURE_ELEMENT) == sizeof(SecureElementNvmData_t) )
    {
        dataSize += sizeof(SecureElementNvmData_t);
    }

    // Region group 1
    if( NvmmRead((uint8_t*)&nvm->RegionGroup1, sizeof(RegionNvmDataGroup1_t), NVM_ID_REGION_GROUP1) == sizeof(RegionNvmDataGroup1_t) )
    {
        dataSize += sizeof(RegionNvmDataGroup1_t);
    }
    
    // Region group 2
    if( NvmmRead((uint8_t*)&nvm->RegionGroup2, sizeof(RegionNvmDataGroup2_t), NVM_ID_REGION_GROUP2) == sizeof(RegionNvmDataGroup2_t) )
    {
        dataSize += sizeof(RegionNvmDataGroup2_t);
    }

    // Class b
    if( NvmmRead((uint8_t*)&nvm->ClassB, sizeof(LoRaMacClassBNvmData_t), NVM_ID_CLASS_B) == sizeof(LoRaMacClassBNvmData_t) )
    {
        dataSize += sizeof(LoRaMacClassBNvmData_t);
    }
    
    return dataSize;
#endif
    return 0;
}

bool NvmDataMgmtFactoryReset( void )
{
    uint16_t offset = 0;
#if( CONTEXT_MANAGEMENT_ENABLED == 1 )
    // Crypto
    if( NvmmReset( sizeof(LoRaMacCryptoNvmData_t), NVM_ID_CRYPTO ) == false )
    {
        return false;
    }

    // Mac Group 1
    if( NvmmReset( sizeof(LoRaMacNvmDataGroup1_t), NVM_ID_MAC_GROUP1 ) == false )
    {
        return false;
    }

    // Mac Group 2
    if( NvmmReset( sizeof(LoRaMacNvmDataGroup2_t), NVM_ID_MAC_GROUP2 ) == false )
    {
        return false;
    }

    // Secure element
    if( NvmmReset( sizeof(SecureElementNvmData_t), NVM_ID_SECURE_ELEMENT ) == false )
    {
        return false;
    }

    // Region group 1
    if( NvmmReset( sizeof(RegionNvmDataGroup1_t), NVM_ID_REGION_GROUP1 ) == false )
    {
        return false;
    }

    // Region group 2
    if( NvmmReset( sizeof( RegionNvmDataGroup2_t ), NVM_ID_REGION_GROUP2 ) == false )
    {
        return false;
    }

    // Class b
    if( NvmmReset( sizeof( LoRaMacClassBNvmData_t ), NVM_ID_CLASS_B ) == false )
    {
        return false;
    }
#endif
    return true;
}
