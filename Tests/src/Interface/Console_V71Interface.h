/*!*****************************************************************************
 * @file    Console_V71InterfaceSync.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.1.0
 * @date    04/06/2023
 * @brief   Console interface for the Console Transmit and Receive
 *          This unit interface the Console API with the current hardware
 *          This interface implements the synchronous use of the API on a SAMV71
 *          and is also specific with the SAMV71 Xplained Ultra board
*******************************************************************************/
/* @page License
 *
 * Copyright (c) 2020-2023 Fabien MAILLY
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS,
 * IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/

/* Revision history:
 * 1.1.0    Adapt the interface to new console library
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef CONSOLE_V71INTERFACESYNC_H_INC
#define CONSOLE_V71INTERFACESYNC_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "Console.h"
#include "Main.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_CONSOLE_TX

#  define LOGTITLE(format, ...)            LOG(CONSOLE_TX, LogSeverity_Title,   LogMode_Line, format, ##__VA_ARGS__) //!< Log Title
#  define LOGFATAL(format, ...)            LOG(CONSOLE_TX, LogSeverity_Fatal,   LogMode_Line, format, ##__VA_ARGS__) //!< Log Fatal
#  define LOGERROR(format, ...)            LOG(CONSOLE_TX, LogSeverity_Error,   LogMode_Line, format, ##__VA_ARGS__) //!< Log Error
#  define LOGWARN(format, ...)             LOG(CONSOLE_TX, LogSeverity_Warning, LogMode_Line, format, ##__VA_ARGS__) //!< Log Warning
#  define LOGINFO(format, ...)             LOG(CONSOLE_TX, LogSeverity_Info,    LogMode_Line, format, ##__VA_ARGS__) //!< Log Information
#  define LOGTRACE(format, ...)            LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_Line, format, ##__VA_ARGS__) //!< Log Trace
#  if (defined(DEBUG) || defined(_DEBUG))
#    define LOGDEBUG(format, ...)          LOG(CONSOLE_TX, LogSeverity_Debug,   LogMode_Line, format, ##__VA_ARGS__) //!< Log Debug
#    define LOGSPECIAL(format, ...)        LOG(CONSOLE_TX, LogSeverity_Special, LogMode_Line, format, ##__VA_ARGS__) //!< Log Special
#    define HEXDUMP(context, src, size)    __HexDump(CONSOLE_TX, context, src, size) //!< Hexadecimal dump of memory
#    define BINDUMP(context, src, size)    __BinDump(CONSOLE_TX, context, src, size) //!< Binary dump of memory
#  else
#    define LOGDEBUG(api, format, ...)
#    define LOGSPECIAL(api, format, ...)
#    define HEXDUMP(api, context, src, size)
#    define BINDUMP(api, context, src, size)
#  endif

#  define LOG_START_INFO(format, ...)      LOG(CONSOLE_TX, LogSeverity_Info,    LogMode_StartPartial, format, ##__VA_ARGS__) //!< Start partial Log Information
#  define LOG_START_TRACE(format, ...)     LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_StartPartial, format, ##__VA_ARGS__) //!< Start partial Log Trace
#  define LOG_START_PARTIAL_INFO           LOG(CONSOLE_TX, LogSeverity_Info,    LogMode_StartPartial, "%s", "")              //!< Start partial Log Information, no string
#  define LOG_START_PARTIAL_TRACE          LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_StartPartial, "%s", "")              //!< Start partial Log Trace, no string
#  define LOG_PARTIAL(format, ...)         LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_Partial,      format, ##__VA_ARGS__) //!< Continue partial Log
#  define LOG_END(format, ...)             LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_EndPartial,   format, ##__VA_ARGS__) //!< End partial Log
#  define LOG_END_PARTIAL                  LOG(CONSOLE_TX, LogSeverity_Trace,   LogMode_EndPartial,   "%s", "")              //!< End partial Log, no string
#  if (defined(DEBUG) || defined(_DEBUG))
#    define LOG_START_DEBUG(format, ...)   LOG(CONSOLE_TX, LogSeverity_Debug,   LogMode_StartPartial, format, ##__VA_ARGS__) //!< Start partial Log Debug
#    define LOG_START_PARTIAL_DEBUG        LOG(CONSOLE_TX, LogSeverity_Debug,   LogMode_StartPartial, "%s", "")              //!< Start partial Log Debug, no string
#  else
#    define LOG_START_DEBUG(format, ...)   //!< No start partial Log Debug
#    define LOG_START_PARTIAL_DEBUG        //!< No start partial Log Debug, no string
#  endif

#else

#  define LOGTITLE(format, ...)         //!< No Log Title
#  define LOGFATAL(format, ...)         //!< No Log Fatal
#  define LOGERROR(format, ...)         //!< No Log Error
#  define LOGWARN(format, ...)          //!< No Log Warning
#  define LOGINFO(format, ...)          //!< No Log Information
#  define LOGTRACE(format, ...)         //!< No Log Trace
#  define LOGDEBUG(format, ...)         //!< No Log Debug
#  define LOGSPECIAL(format, ...)       //!< No Log Special
#  define HEXDUMP(context, src, size)   //!< Hexadecimal dump of memory
#  define BINDUMP(context, src, size)   //!< Binary dump of memory

#  define LOG_START_INFO(format, ...)   //!< No Start partial Log Information
#  define LOG_START_TRACE(format, ...)  //!< No Start partial Log Trace
#  define LOG_START_DEBUG(format, ...)  //!< No Start partial Log Debug
#  define LOG_START_PARTIAL_INFO        //!< No start partial Log Information, no string
#  define LOG_START_PARTIAL_TRACE       //!< No start partial Log Trace, no string
#  define LOG_START_PARTIAL_DEBUG       //!< No start partial Log Debug, no string

#  define LOG_PARTIAL(format, ...)      //!< No continue partial Log
#  define LOG_END(format, ...)          //!< No End partial Log
#  define LOG_END_PARTIAL               //!< End partial Log, no string

#endif
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// UART of V71
//********************************************************************************************************************

//! @brief Console UART Tx initialization for the ATSAMV71
void ConsoleUART_TxInit_V71(void);

/*! @brief UART transmit char function interface of the ATSAMV71
 *
 * This function will be called to try to transmit data over the UART
 * This function only try to transmit and it is not intend to transmit all the data. To transmit all the data, repeat calling this function until size == 0
 * @param[in] *pIntDev Is the UART interface container structure used for the UART transmit
 * @param[in] *data Is the data array to send to the UART transmiter through the transmit FIFO
 * @param[in] size Is the count of data to send to the UART transmitter through the transmit FIFO
 * @param[out] *actuallySent Is the count of data actually sent to the transmit FIFO
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT UARTtransmit_V71(UART_Interface *pIntDev, uint8_t *data, size_t size, size_t *actuallySent);

//-----------------------------------------------------------------------------


//! @brief Console UART Rx initialization for the ATSAMV71
void ConsoleUART_RxInit_V71(void);

/*! @brief UART receive char function interface of the ATSAMV71
 *
 * @param[in] *pIntDev Is the UART interface container structure used for the UART receive
 * @param[out] *data Is where the data will be stored
 * @param[in] size Is the count of data that the data buffer can hold
 * @param[out] *actuallyReceived Is the count of data actually received from the received FIFO
 * @param[out] *lastCharError Is the last char received error. Set to UART_NO_ERROR (0) if no errors
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT UARTreceive_V71(UART_Interface *pIntDev, uint8_t *data, size_t size, size_t *actuallyReceived, uint8_t *lastCharError);

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Transmit API
//********************************************************************************************************************
#ifdef USE_CONSOLE_TX

#  define CONSOLE_TX_BUFFER_SIZE    200          //!< Define the console transmission buffer size, must be determined according to the max length of a string and the UART speed
   char ConsoleTxBuffer[CONSOLE_TX_BUFFER_SIZE]; //!< The actual console transmission buffer

   extern ConsoleTx Console_TxConf;   //!< The console transmission configuration
#  define CONSOLE_TX  &Console_TxConf //!< Define to simplify the naming at the functions calling

#endif // USE_CONSOLE_TX
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************

//! Command buffer size
#define COMMAND_BUFFER_SIZE   256

/*! @brief Command Input buffer
 *
 * This structure contains infos concerning the input data for the received command
 */
typedef struct
{
	volatile uint32_t BufPos;         //!< Position in the buffer
	char Buffer[COMMAND_BUFFER_SIZE]; //!< Raw buffer with the frame to be processed
	volatile bool ToProcess;          //!< Indicate that the frame in buffer should be processed or not
} CommandInputBuf;

//! The current Command Input buffer
extern CommandInputBuf CommandInput;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************
#ifdef USE_CONSOLE_RX
//-----------------------------------------------------------------------------

# ifdef CONSOLE_RX_USE_COMMAND_RECALL
#   define CONSOLE_RX_COMMAND_BUFFER_SIZE    400               //!< Define the console command recall buffer size, this set the character count of commands that can be recall
    char ConsoleRxCommandBuffer[CONSOLE_RX_COMMAND_BUFFER_SIZE]; //!< The actual console command recall buffer
# endif

  extern ConsoleRx Console_RxConf;   //!< The console reception configuration
# define CONSOLE_RX  &Console_RxConf //!< Define to simplify the naming at the functions calling

//-----------------------------------------------------------------------------
#endif // USE_CONSOLE_RX



//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CONSOLE_V71INTERFACESYNC_H_INC */