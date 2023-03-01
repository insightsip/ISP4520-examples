 /******************************************************************************
 * @file    at_manager.h
 * @author  Insight SiP
 * @brief  at commands for LoRaWan
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

#ifndef AT_MANAGER_H__
#define AT_MANAGER_H__

#include <stdint.h>

#define MODULE_NAME             "ISP4520"
#define MAX_AT_PREFIX_SIZE      20

typedef enum 
{
    AT_OK = 0,
    AT_ERROR_UNKNOWN_CMD,
    AT_ERROR_NOT_SUPPORTED,
    AT_ERROR_PARAM,
    AT_ERROR_BUSY,
    AT_ERROR_NOT_JOINED,
    AT_ERROR_DUTY_CYCLE,
    AT_ERROR_OTHER,
} at_error_code_t;

/* AT Command list */
#define AT_RESET        "Z"
#define AT_ECHO         "E"
#define AT_INFO         "I"
#define AT_DEVEUI       "+DEVEUI"
#define AT_APPEUI       "+APPEUI"
#define AT_JOINEUI      "+JOINEUI"
#define AT_APPKEY       "+APPKEY"
#define AT_GENAPPKEY    "+GENAPPKEY"
#define AT_NWKKEY       "+NWKKEY"
#define AT_FNWKSINTKEY  "+FNWKSINTKEY"
#define AT_SNWKSINTKEY  "+SNWKSINTKEY"
#define AT_NWKSENCKEY   "+NWKSENCKEY"
#define AT_APPSKEY      "+APPSKEY"
#define AT_DEVADDR      "+DEVADDR"
#define AT_NETID        "+NETID"
#define AT_JOINRQ       "+JOINRQ"
#define AT_JOINSTAT     "+JOINSTAT"
#define AT_RCV          "+RCV"
#define AT_SEND         "+SEND"
#define AT_ADR          "+ADR"
#define AT_CLASS        "+CLASS"
#define AT_DR           "+DR"
#define AT_JOINDLY1     "+JOINDLY1"
#define AT_JOINDLY2     "+JOINDLY2"
#define AT_PNET         "+PNET"
#define AT_RXDLY1       "+RXDLY1"
#define AT_RXDLY2       "+RXDLY2"
#define AT_RXDR2        "+RXDR2"
#define AT_RXFQ2        "+RXFQ2"
#define AT_TXP          "+TXP"
#define AT_BATT         "+BATT"
#define AT_RSSI         "+RSSI"
#define AT_SNR          "+SNR"
#define AT_DUTYC        "+DUTYC"
#define AT_CHANNEL      "+CHANNEL"
#define AT_CERTIF       "+CERTIF"
#define AT_CTXRST       "+CTXRST"

uint32_t at_manager_init();
uint32_t at_manager_execute();


#endif
