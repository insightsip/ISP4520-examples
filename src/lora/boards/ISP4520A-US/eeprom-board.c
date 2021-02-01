/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
 * @note 
 *      This implementation is the FDS (Flash Dara Storage) library of Nordic Semiconductor.
 *      The impacts are:
 *          - Higher flash lifetime
 *          - Easy implementation (we manipulate files and not firectly the flash)
 *          - Should not be conflict with other nordic tools that require flash operation (peer manager, bootloader; etc..)
 *
 *****************************************************************************/

#include "utilities.h"
#include "eeprom-board.h"
#include "fds.h"

#include "nrf_log.h"

#define CTX_FILE 1000                       //!< Context File ID.

static bool volatile m_fds_initialized;     //!< Flag to check fds initialization.
static bool volatile m_fds_write_done;      //!< Flag to check fds writting or updating is done.
static bool volatile m_fds_delete_done;     //!< Flag to check fds deleting is done.
static bool volatile m_fds_gc_done;         //!< Flag to check fds gc is done.


/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("FDS write success! FileId: 0x%x RecKey:0x%x", p_evt->write.file_id, p_evt->write.record_key);
            }
            m_fds_write_done = true;
        } break;

        case FDS_EVT_UPDATE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("FDS update success! FileId: 0x%x RecKey:0x%x", p_evt->write.file_id, p_evt->write.record_key);
            }
            m_fds_write_done = true;
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("FDS delete success! FileId: 0x%x RecKey:0x%x", p_evt->write.file_id, p_evt->write.record_key);
            }
            m_fds_delete_done = true;
        } break;

        case FDS_EVT_GC:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("FDS Garbage collection success!");
            }
            m_fds_gc_done = true;
        } break;

        default:
            NRF_LOG_DEBUG("Event: %s received (%s)", fds_evt_str[p_evt->id], fds_err_str(p_evt->result));
            break;
    }
}

uint8_t EepromMcuInit(void)
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);

    NRF_LOG_INFO("Initializing fds...");
    
    /* Initialize fds */
    rc = fds_init();
    if (rc != NRF_SUCCESS)
    {   
        return FAIL;
    }

    /* Wait for fds to initialize. */
    while (!m_fds_initialized)
    {
        power_manage();
    }

    /* Reading flash usage statistics */
    NRF_LOG_INFO("Reading flash usage statistics...");

    fds_stat_t stat = {0};
    rc = fds_stat(&stat);
    if (rc != NRF_SUCCESS)
    {   
        return FAIL;
    }

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    while (fds_record_find_in_file(CTX_FILE, &desc, &tok) != FDS_ERR_NOT_FOUND)
    {
        NRF_LOG_DEBUG("Record %d at %x", desc.record_id, desc.p_record);
    }

    return SUCCESS;
}

uint8_t EepromMcuWriteBuffer( uint16_t record_key, uint8_t *buffer, uint16_t size )
{
    ret_code_t rc_find, rc_write;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
    fds_record_t record;

    record.file_id = CTX_FILE;
    record.key = record_key;
    record.data.p_data = buffer,
    record.data.length_words = CEIL_DIV(size, sizeof(uint32_t));

    m_fds_write_done = false;
    rc_find = fds_record_find(CTX_FILE, record_key, &desc, &tok);
    if (rc_find != NRF_SUCCESS && rc_find != FDS_ERR_NOT_FOUND)
    {
        return FAIL;
    }

    if (rc_find == NRF_SUCCESS)
    {
        /* Write the updated record to flash. */
        rc_write = fds_record_update(&desc, &record);
    }
    else if (rc_find == FDS_ERR_NOT_FOUND)
    {
        /* Record not found; write a new one. */
        rc_write = fds_record_write(&desc, &record);
    }

    if (rc_write == NRF_SUCCESS)
    { 
        /* Write/update started, let's wait for fds to finish. */
        while (!m_fds_write_done)
        {
            power_manage();
        }
        // Here writting is successfully done so we can leave the function
        return SUCCESS;
    }
    else if (rc_write == FDS_ERR_NO_SPACE_IN_FLASH)
    {
        /* No more space, so we run garbage collector to free some space and then try again */
        NRF_LOG_INFO("No space in flash, running garbage collector.");
        m_fds_gc_done = false;
        fds_gc();

         /* Garbage collection in progress, let's wait for fds to finish. */
        while (!m_fds_gc_done)
        {
            power_manage();
        }
     
        /* try again */
        m_fds_write_done = false;
        rc_find = fds_record_find(CTX_FILE, record_key, &desc, &tok);
        if (rc_find != NRF_SUCCESS && rc_find != FDS_ERR_NOT_FOUND)
        {
            return FAIL;
        }

        if (rc_find == NRF_SUCCESS)
        {
           /* Write the updated record to flash. */
           rc_write = fds_record_update(&desc, &record);
        }
        else if (rc_find == FDS_ERR_NOT_FOUND)
        {
           /* Record not found; write a new one. */
           rc_write = fds_record_write(&desc, &record);
        }

        if (rc_write == NRF_SUCCESS)
        { 
            /* Write/update started, let's wait for fds to finish. */
            while (!m_fds_write_done)
            {
                power_manage();
            }
             return SUCCESS;
        }
    }
    return FAIL;
}

uint8_t EepromMcuReadBuffer( uint16_t record_key, uint8_t *buffer, uint16_t size )
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
    fds_flash_record_t flash_rec;

    rc = fds_record_find(CTX_FILE, record_key, &desc, &tok);
    if (rc != NRF_SUCCESS)
    {
        return FAIL;
    }

    rc = fds_record_open(&desc, &flash_rec);
    if (rc != NRF_SUCCESS)
    {
        return FAIL;
    }

    memcpy(buffer, flash_rec.p_data, size);

    fds_record_close(&desc);

    return SUCCESS;
}

uint8_t EepromMcuDeleteBuffer(uint16_t record_key)
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(CTX_FILE, record_key, &desc, &tok);
    if (rc != NRF_SUCCESS)
    {
        return FAIL;
    }

    /* Delete record. */
    m_fds_delete_done = false;
    rc = fds_record_delete(&desc);
    if (rc != NRF_SUCCESS)
    {
        return FAIL;
    }

    /* Deletinge started, let's wait for fds to finish. */
    while (!m_fds_delete_done)
    {
        power_manage();
    }

    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
   // assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
   // assert_param( FAIL );
    return 0;
}
