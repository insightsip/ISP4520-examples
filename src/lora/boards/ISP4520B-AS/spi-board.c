


/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
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

#include "spi-board.h"
#include "board.h"


void SpiInit (Spi_t *obj, uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t nss)
{
    uint32_t err_code;
    nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG;
	
    const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
    obj->Spi = spi;
	
    config.frequency	= NRF_DRV_SPI_FREQ_8M;
    config.miso_pin     = miso;
    config.mosi_pin     = mosi;
    config.sck_pin      = sclk;
    config.ss_pin       = nss;
    err_code = nrf_drv_spi_init (&(obj->Spi), &config, NULL, NULL);	
    APP_ERROR_CHECK (err_code);
}

void SpiDeInit (Spi_t *obj)
{
    nrf_drv_spi_uninit (&(obj->Spi));	
}

uint16_t SpiInOut (Spi_t *obj, uint16_t outData)
{
    uint32_t err_code;
    uint8_t inData ;

    if ((obj == NULL) || (&(obj->Spi) == NULL))
    {
        APP_ERROR_CHECK_BOOL (false);
    }

    err_code = nrf_drv_spi_transfer (&(obj->Spi), (uint8_t*)&outData, 1, (uint8_t*)&inData, 1);
    APP_ERROR_CHECK(err_code);

    return (uint16_t)inData;
}

