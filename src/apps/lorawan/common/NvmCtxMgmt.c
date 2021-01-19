/*!
 * \file      NvmCtxMgmt.c
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
 *      2020 Insight SiP  
 *
 *      This file is left as placeholder.
 *      Neither nvmc nor eeprom are implemented yet.
 *      Keep #define CONTEXT_MANAGEMENT_ENABLED  0
 *****************************************************************************/

#include <stdio.h>
#include "NvmCtxMgmt.h"
#include "utilities.h"
//#include "eeprom.h"
#include "nvmm.h"

/*!
 * Enables/Disables the context storage management storage at all. Must be enabled for LoRaWAN 1.1.x.
 * WARNING: Still under development and not tested yet.
 */
#define CONTEXT_MANAGEMENT_ENABLED         0

/*!
 * Enables/Disables maximum persistent context storage management. All module contexts will be saved on a non-volatile memory.
 * WARNING: Still under development and not tested yet.
 */
#define MAX_PERSISTENT_CTX_MGMT_ENABLED    0

#if ( MAX_PERSISTENT_CTX_MGMT_ENABLED == 1 )
#define NVM_CTX_STORAGE_MASK               0xFF
#else
#define NVM_CTX_STORAGE_MASK               0x8C
#endif

#if ( CONTEXT_MANAGEMENT_ENABLED == 1 )
/*!
 * LoRaMAC Structure holding contexts changed status
 * in case of a \ref MLME_NVM_CTXS_UPDATE indication.
 */
typedef union uLoRaMacCtxsUpdateInfo
{
    /*!
     * Byte-access to the bits
     */
    uint8_t Value;
    /*!
     * The according context bit will be set to one
     * if the context changed or 0 otherwise.
     */
    struct sElements
    {
        /*!
         * Mac core nvm context
         */
        uint8_t Mac : 1;
        /*!
         * Region module nvm contexts
         */
        uint8_t Region : 1;
        /*!
         * Cryto module context
         */
        uint8_t Crypto : 1;
        /*!
         * Secure Element driver context
         */
        uint8_t SecureElement : 1;
        /*!
         * MAC commands module context
         */
        uint8_t Commands : 1;
        /*!
         * Class B module context
         */
        uint8_t ClassB : 1;
        /*!
         * Confirm queue module context
         */
        uint8_t ConfirmQueue : 1;
        /*!
         * FCnt Handler module context
         */
        uint8_t FCntHandlerNvmCtx : 1;
    }Elements;
}LoRaMacCtxUpdateStatus_t;

LoRaMacCtxUpdateStatus_t CtxUpdateStatus = { .Value = 0 };

/*
 * Nvmm handles
 */
static NvmmDataBlock_t SecureElementNvmCtxDataBlock;
static NvmmDataBlock_t CryptoNvmCtxDataBlock;
#if ( MAX_PERSISTENT_CTX_MGMT_ENABLED == 1 )
static NvmmDataBlock_t MacNvmCtxDataBlock;
static NvmmDataBlock_t RegionNvmCtxDataBlock;
static NvmmDataBlock_t CommandsNvmCtxDataBlock;
static NvmmDataBlock_t ConfirmQueueNvmCtxDataBlock;
static NvmmDataBlock_t ClassBNvmCtxDataBlock;
#endif
#endif

void NvmCtxMgmtEvent( LoRaMacNvmCtxModule_t module )
{
#if ( CONTEXT_MANAGEMENT_ENABLED == 1 )
    switch( module )
    {
        case LORAMAC_NVMCTXMODULE_MAC:
        {
            CtxUpdateStatus.Elements.Mac = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_REGION:
        {
            CtxUpdateStatus.Elements.Region = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_CRYPTO:
        {
            CtxUpdateStatus.Elements.Crypto = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_SECURE_ELEMENT:
        {
            CtxUpdateStatus.Elements.SecureElement = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_COMMANDS:
        {
            CtxUpdateStatus.Elements.Commands = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_CLASS_B:
        {
            CtxUpdateStatus.Elements.ClassB = 1;
            break;
        }
        case LORAMAC_NVMCTXMODULE_CONFIRM_QUEUE:
        {
            CtxUpdateStatus.Elements.ConfirmQueue = 1;
            break;
        }
        default:
        {
            break;
        }
    }
#endif
}

NvmCtxMgmtStatus_t NvmCtxMgmtStore( void )
{
#if ( CONTEXT_MANAGEMENT_ENABLED == 1 )
	//TODO To be implemented
	return NVMCTXMGMT_STATUS_FAIL;
#else
    return NVMCTXMGMT_STATUS_FAIL;
#endif
}

NvmCtxMgmtStatus_t NvmCtxMgmtRestore( void )
{
#if ( CONTEXT_MANAGEMENT_ENABLED == 1 )
	//TODO To be implemented
	return NVMCTXMGMT_STATUS_FAIL;
#else
    return NVMCTXMGMT_STATUS_FAIL;
#endif
}
