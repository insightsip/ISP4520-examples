/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *              (C)2013-2017 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team (C)( STMicroelectronics International )
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

#include <math.h>
#include <time.h>
#include "app_util.h"
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "rtc-board.h"


#define MIN_ALARM_DELAY                 3                   /**< MCU Wake Up Time */
#define RTC2_PRESCALER                  31                  /**< Prescaler for the RTC Timer */
#define RTC2_CLOCK_FREQ                 32768               /**< Clock frequency of the RTC timer. */


/*!
 * RTC timer context 
 */
typedef struct
{
    uint32_t        Time;         // Reference time
}RtcTimerContext_t;

/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * \brief Indicates the number of overflows
 */
static uint8_t m_ovrflw_cnt = 0;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

//Backup data
static uint32_t m_data0;
static uint32_t m_data1;

rtc_evt_handler_t m_rtc_handler;




void RtcInit(rtc_evt_handler_t handler)
{
    if (RtcInitialized == false)
    {
        m_rtc_handler = handler;

        NRF_RTC2->PRESCALER = RTC2_PRESCALER;    
        NRF_RTC2->INTENSET = RTC_INTENSET_OVRFLW_Msk;

        NVIC_ClearPendingIRQ(RTC2_IRQn);
        NVIC_SetPriority(RTC2_IRQn, 2);
        NVIC_EnableIRQ(RTC2_IRQn);

        NRF_RTC2->TASKS_START = 1;

        RtcSetTimerContext( );
        m_ovrflw_cnt = 0;
        RtcInitialized = true;
    }
}

uint32_t RtcGetMinimumTimeout( void )
{
    return MIN_ALARM_DELAY;
}

uint32_t RtcMs2Tick( uint32_t milliseconds )
{
     return ((uint32_t)ROUNDED_DIV((milliseconds) * ((uint64_t)RTC2_CLOCK_FREQ), 1000 * (RTC2_PRESCALER + 1)));
}

uint32_t RtcTick2Ms( uint32_t tick )
{
     return ((uint32_t)ROUNDED_DIV((tick) * ((uint64_t)(1000 * (RTC2_PRESCALER + 1))), (uint64_t)RTC2_CLOCK_FREQ));
}

void RtcSetAlarm(uint32_t timeout)
{
    TimerTime_t now = NRF_RTC2->COUNTER;
    NRF_RTC2->CC[0] = RtcTimerContext.Time + timeout;

    // Enable RTC2 CC[0] interrupts
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
}

void RtcStopAlarm(void)
{
    // Disable RTC2 CC[0] interrupts
    NRF_RTC2->INTENCLR = RTC_INTENSET_COMPARE0_Msk;
}

uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = NRF_RTC2->COUNTER;
    return ( uint32_t )RtcTimerContext.Time;  
}

uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

uint32_t RtcGetCalendarTime(uint16_t *milliseconds)
{
    uint32_t ticks;
    uint32_t temp_milliseconds;

    ticks = RtcGetTimerValue();
    ticks += m_ovrflw_cnt*0xFFFFFFUL; 

    temp_milliseconds = RtcTick2Ms(ticks);

    uint32_t seconds = (uint32_t)(temp_milliseconds/1000);

    *milliseconds = (uint16_t)(temp_milliseconds % 1000); 

    return seconds;
}

uint32_t RtcGetTimerValue(void)
{
    return NRF_RTC2->COUNTER;
}

uint32_t RtcGetTimerElapsedTime(void)
{
    TimerTime_t now_in_ticks = NRF_RTC2->COUNTER;
    return (now_in_ticks - RtcTimerContext.Time);
}

void RtcBkupWrite(uint32_t data0, uint32_t data1)
{
    //TODO a backup should be done on flash memory
    m_data0 = data0;
    m_data1 = data1;
}

void RtcBkupRead(uint32_t *data0, uint32_t *data1)
{
    //TODO a backup should be done on flash memory
    *data0 = m_data0;
    *data1 = m_data1;
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature)
{
    float k = RTC_TEMP_COEFFICIENT;
    float kDev = RTC_TEMP_DEV_COEFFICIENT;
    float t = RTC_TEMP_TURNOVER;
    float tDev = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0;
    float ppm = 0.0;

    if( k < 0.0 )
    {
        ppm = ( k - kDev );
    }
    else
    {
        ppm = ( k + kDev );
    }
    interim = ( temperature - ( t - tDev ) );
    ppm *=  interim * interim;

    // Calculate the drift in time
    interim = ( ( float ) period * ppm ) / 1e6;
    // Calculate the resulting time period
    interim += period;
    interim = floor( interim );

    if( interim < 0.0 )
    {
        interim = ( float )period;
    }

    // Calculate the resulting period
    return ( TimerTime_t ) interim;
}

/*!
 * \brief RTC2 IRQ handler
 */
void RTC2_IRQHandler(void)
{
    if (NRF_RTC2->EVENTS_COMPARE[0] == 1)
    {  
        NRF_RTC2->EVENTS_COMPARE[0] = 0;
        m_rtc_handler();       
    }

    if (NRF_RTC2->EVENTS_OVRFLW == 1)
    {  
        NRF_RTC2->EVENTS_OVRFLW = 0;
        m_ovrflw_cnt++;
    }

}