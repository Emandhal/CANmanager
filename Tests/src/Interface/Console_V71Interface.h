/*******************************************************************************
    File name:    Console_V71InterfaceSync.h
    Author:       FMA
    Version:      1.0
    Date (d/m/y): 28/04/2020
    Description:  Console interface for the Console Transmit
                  This unit interface the Console API with the current hardware
                  This interface implements the synchronous use of the API on a SAMV71
                  and is also specific with the SAMV71 Xplained Ultra board
    History :
*******************************************************************************/
#ifndef CONSOLE_V71INTERFACESYNC_H_INC
#define CONSOLE_V71INTERFACESYNC_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "Console.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------

//! Log Title, use it instead of LOG!
#define LOGTITLE(format, ...)           LOG(CONSOLE_TX, lsTitle, format, ##__VA_ARGS__)
//! Log Fatal, use it instead of LOG!
#define LOGFATAL(format, ...)           LOG(CONSOLE_TX, lsFatal, format, ##__VA_ARGS__)
//! Log Error, use it instead of LOG!
#define LOGERROR(format, ...)           LOG(CONSOLE_TX, lsError, format, ##__VA_ARGS__)
//! Log Warning, use it instead of LOG!
#define LOGWARN(format, ...)            LOG(CONSOLE_TX, lsWarning, format, ##__VA_ARGS__)
//! Log Information, use it instead of LOG!
#define LOGINFO(format, ...)            LOG(CONSOLE_TX, lsInfo, format, ##__VA_ARGS__)
//! Log Trace, use it instead of LOG!
#define LOGTRACE(format, ...)           LOG(CONSOLE_TX, lsTrace, format, ##__VA_ARGS__)
//! Log Debug, use it instead of LOG!
#	define LOGDEBUG(format, ...)          LOG(CONSOLE_TX, lsDebug, format, ##__VA_ARGS__)
//! Log Special, use it instead of LOG!
#	define LOGSPECIAL(format, ...)        LOG(CONSOLE_TX, lsSpecial, format, ##__VA_ARGS__)
//! Hexadecimal dump of memory
#define HEXDUMP(context, src, size)     __HexDump(CONSOLE_TX, context, src, size)
//! Binary dump of memory
#define BINDUMP(context, src, size)     __BinDump(CONSOLE_TX, context, src, size)

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

#define CONSOLE_TX_BUFFER_SIZE    200         //!< Define the console transmission buffer size, must be determined according to the max length of a string and the UART speed
char ConsoleTxBuffer[CONSOLE_TX_BUFFER_SIZE]; //!< The actual console transmission buffer

extern ConsoleTx Console_TxConf;    //!< The console transmission configuration
#define CONSOLE_TX  &Console_TxConf //!< Define to simplify the naming at the functions calling

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************

#ifdef CONSOLE_RX_USE_COMMAND_RECALL
#  define CONSOLE_RX_COMMAND_BUFFER_SIZE    400              //!< Define the console command recall buffer size, this set the character count of commands that can be recall
char ConsoleRxCommandBuffer[CONSOLE_RX_COMMAND_BUFFER_SIZE]; //!< The actual console command recall buffer
#endif

extern ConsoleRx Console_RxConf;    //!< The console reception configuration
#define CONSOLE_RX  &Console_RxConf //!< Define to simplify the naming at the functions calling

//-----------------------------------------------------------------------------

/*! @brief Specific command function
 * @param[in] *pCmd Is the command string first char (NULL terminated string)
 * @param[in] size Is the char count of the command string pCmd
 * @return Returns an #eERRORRESULT value enum
 */
typedef eERRORRESULT (*RxCommand_Func)(const uint8_t* pCmd, size_t size);

/*! Console command hash + function tuple
 * @details The first member of each supported commands is a hash of a specific string.
 * This string is the first parameter of the string (ie. from index 0 to the first space or null character
 * The second is the function that will be called if the hash match
 */
typedef struct ConsoleCommand
{
  uint32_t Hash;                   //!< Hash of the first parameter of the command
  RxCommand_Func fnCommandProcess; //!< This function will be called when the hash will match
} ConsoleCommand;



/*! @brief Process received command Callback
 *
 * @param[in] *pCmd Is the command string first char (NULL terminated string)
 * @param[in] size Is the char count of the command string pCmd
 */
//void ProcessReceivedCommandCallBack(const uint8_t* pCmd, size_t size);

//-----------------------------------------------------------------------------


/*! @brief Process GPIO command
 * @details Commands that can be parsed are the following:
 * GPIO <action> <PORT/Pin>[ <value>]
 * Where:
 *  - <action> can be:
 *    - RD, READ: Read <PORT/Pin>
 *    - WR, WRITE: Write <value> to <PORT/Pin>
 *    - SET: Set bitset of <value> to <PORT/Pin>
 *    - CLR, CLEAR: Clear bitset of <value> to <PORT/Pin>
 *    - TG, TOGGLE: Toggle the value of <PORT/Pin>
 *  - <PORT/Pin> can be:
 *    - PORTx: where 'x' can be any of the ports name of the MCU
 *    - Rxy: where 'x' can be any of the ports name and 'y' the number of the pin of the MCU (ex: RA0)
 *  - <value> is the value to apply in case of Write, Set, or Clear. The value can be binary (0b prefix), decimal, hexadecimal (0x prefix). The default value will be 0
 *
 * @param[in] *pCmd Is the command string first char (NULL terminated string)
 * @param[in] size Is the char count of the command string pCmd
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT ProcessGPIOcommand(const uint8_t* pCmd, size_t size);

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CONSOLE_V71INTERFACESYNC_H_INC */