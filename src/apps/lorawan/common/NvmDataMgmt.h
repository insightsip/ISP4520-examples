/*!
 * \file      NvmDataMgmt.h
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
 *
 * \defgroup  NVMDATAMGMT NVM context management implementation
 *            This module implements the NVM context handling
 * \{
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

#ifndef __NVMDATAMGMT_H__
#define __NVMDATAMGMT_H__

/*!
 * \brief NVM Management event.
 *
 * \param [IN] notifyFlags Bitmap which contains the information about modules that
 *                         changed.
 */
void NvmDataMgmtEvent( uint16_t notifyFlags );

/*!
 * \brief Function which initiliaze the NvmDataMgnt.
 *
 * \retval Returns true, if successful.
 */
uint16_t NvmDataMgmtInit( void );


/*!
 * \brief Function which stores the MAC data into NVM, if required.
 *
 * \retval Number of bytes which were stored.
 */
uint16_t NvmDataMgmtStore( void );

/*!
 * \brief Function which restores the MAC data from NVM, if required.
 *
 * \retval Number of bytes which were restored.
 */
uint16_t NvmDataMgmtRestore(void );

/*!
 * \brief Resets the NVM data.
 *
 * \retval Returns true, if successful.
 */
bool NvmDataMgmtFactoryReset( void );

/* \} */

#endif // __NVMDATAMGMT_H__
