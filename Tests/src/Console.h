/*!*****************************************************************************
 * @file    Console.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/12/2017
 * @brief   Some functions for RS-232 console communication
 ******************************************************************************/
#ifndef CONSOLE_H_
#define CONSOLE_H_
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdio.h>
#include <stdarg.h>
#include "UART_Interface.h"

#if !defined(__cplusplus)
# include "asf.h"
#else
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#if !defined(__cplusplus)

#  define CONSOLE_MEMBER(name)  .name =
#  define CONSOLE_WEAK          __attribute__((weak))
#  define __FORMATPRINTF23__    __attribute__((__format__(__printf__, 2, 3))) // 2: Format at second argument ; 3: args at third argument (...)
#  define __FORMATPRINTF30__    __attribute__((__format__(__printf__, 3, 0))) // 3: Format at third  argument ; 0: for va_list
#  define __FORMATPRINTF34__    __attribute__((__format__(__printf__, 3, 4))) // 3: Format at third  argument ; 4: args at fourth argument (...)
#  define __FORMATPRINTF40__    __attribute__((__format__(__printf__, 4, 0))) // 4: Format at fourth argument ; 0: for va_list
#  define vprintf               viprintf

#else

#  define CONSOLE_MEMBER(name)
#  define CONSOLE_WEAK          __attribute__((weak))
#  define __FORMATPRINTF23__
#  define __FORMATPRINTF30__
#  define __FORMATPRINTF34__
#  define __FORMATPRINTF40__

#endif

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Transmit API
//********************************************************************************************************************
#ifdef USE_CONSOLE_TX

//! Circular Buffer for Console transmit structure
typedef struct ConsoleTx ConsoleTx;
struct ConsoleTx
{
  void *UserAPIData;      //!< Optional, can be used to store API data or NULL

  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  UART_Interface* UART;   //!< This is the UART_RxInterface descriptor pointer that will be used to communicate with the device
#else
  UART_Interface  UART;   //!< This is the UART_RxInterface descriptor that will be used to communicate with the device
#endif

  //--- Transmit buffer ---
  volatile size_t InPos;  //!< This is the input position in the buffer (where data will be write before being send to UART)
  volatile size_t OutPos; //!< This is the output position in the buffer (where data will be read and send to UART)
  char *Buffer;           //!< The buffer itself (should be the same size as BufferSize)
  size_t BufferSize;      //!< The buffer size
};

//-----------------------------------------------------------------------------


/*! @brief Initialize the Console transmit
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT InitConsoleTx(ConsoleTx* pApi);

/*! @brief Set char to print buffer
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] aChar Is the char to be sent
 */
void SetCharToConsoleBuffer(ConsoleTx* pApi, const char aChar);

/*! @brief Set char array to print buffer
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] *string Pointer to one-line string to be sent (will stop at the first '\0' char)
 */
void SetStrToConsoleBuffer(ConsoleTx* pApi, const char* string);

/*! @brief Try to send next char in the console print buffer
 *
 * This function is for programs that need to send only in a main loop like in a while(true){} in the main
 * @param[in] *pApi Is the Console transmit API to work with
 * @return If the next char is actually sending
 */
bool TrySendingNextCharToConsole(ConsoleTx* pApi);

/*! @brief Indicate if there is a char to send to console
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @return If there is a char to send to console
 */
inline bool IsCharToSendToConsole(ConsoleTx* pApi)
{
  if (pApi == NULL) return false;
  return (pApi->OutPos != pApi->InPos);
}

/*! @brief Convert a char to a digit value
 *
 * @param[in] aChar The char to convert
 * @return return the digit value
 */
inline uint32_t CharToDigit(const char aChar)
{
  return (uint32_t)(aChar - '0');
}

//-----------------------------------------------------------------------------

//! Macro to get the lower case of a char
#define LowerCase(x)  (((((int_fast8_t)(x)) - 'A') < 26) ? (x) + 32 : (x))

//-----------------------------------------------------------------------------


//! Log type, sorted by severity.
typedef enum
{
  lsTitle   = 0, //! Show a title
  lsFatal   = 1, //! Fatal error! application will abort shortly
  lsError   = 2, //! Error! the application may work improperly
  lsWarning = 3, //! Warning! There's something wrong
  lsInfo    = 4, //! For your information, the application is safe
  lsTrace   = 5, //! Trace log only
  lsDebug   = 6, //! For Debugging purpose. Emitted only when DEBUG is defined
  lsSpecial = 7, //! For Debugging purpose. Emitted only when DEBUG is defined
  lsLast_,       //! Special value. Do not use and keep this the last value
} eSeverity;

//! Windows console colors
typedef enum
{
  wccBLACK   = 0,
  wccNAVY    = 1,
  wccGREEN   = 2,
  wccTEAL    = 3,
  wccMAROON  = 4,
  wccPURPLE  = 5,
  wccOLIVE   = 6,
  wccSILVER  = 7,
  wccGRAY    = 8,
  wccBLUE    = 9,
  wccLIME    = 10,
  wccAQUA    = 11,
  wccRED     = 12,
  wccFUCHISA = 13,
  wccYELLOW  = 14,
  wccWHITE   = 15,
  wccLast_, // KEEP LAST!
} eWinConsoleColor;

//-----------------------------------------------------------------------------


/*! @brief Send a formated Logs to console
 *
 * @note DO NOT USE DIRECTLY, use LOG*() instead.
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] *context Context string (usually, the system name emitting the log).
 * @param[in] whiteText If the text after the time counter change to white
 * @param[in] *format Format string (printf format), followed by arguments.
 * @param[in] args Arguments of the formated string.
 */
void __LOG(ConsoleTx* pApi, const char* context, bool whiteText, const char* format, va_list args) __FORMATPRINTF40__;

/*! @brief Send a formated Logs to console
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] severity This is the log severity.
 * @param[in] *format Format string (printf format), followed by arguments.
 * @param[in] ... Arguments of the formated string.
 */
void LOG(ConsoleTx* pApi, eSeverity severity, const char* format, ...) __FORMATPRINTF34__;

//-----------------------------------------------------------------------------

#ifdef __cplusplus
/*! @brief Set the Windows console color
 *
 * @param[in] text Is the text color of the Windows console
 * @param[in] fond Is the background color of the Windows console
 */
void SetConsoleColor(eWinConsoleColor text, eWinConsoleColor background);

/*! @brief Send a formated Simulation Logs to console
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] *format Format string (printf format), followed by arguments.
 * @param[in] ... Arguments of the formated string.
 */
void LOGSIM(ConsoleTx* pApi, const char* format, ...);
#endif

//-----------------------------------------------------------------------------

#if (defined(DEBUG) || defined(_DEBUG))
/*! @brief Show the hexadecimal dump of the memory to console
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] *context Is the text to show for the dump
 * @param[in] *src This is the source pointer of the begining of data to dump
 * @param[in] size The size of data to dump.
 */
void __HexDump(ConsoleTx* pApi, const char* context, const void* src, unsigned int size);

/*! @brief Show the binary dump of the memory to console
 *
 * @param[in] *pApi Is the Console transmit API to work with
 * @param[in] *context Is the text to show for the dump
 * @param[in] *src This is the source pointer of the begining of data to dump
 * @param[in] size The size of data to dump
 */
void __BinDump(ConsoleTx* pApi, const char* context, const void* src, unsigned int size);
#endif

//-----------------------------------------------------------------------------

//! Log Title, use it instead of LOG!
#define LOGTITLE_(api, format, ...)             LOG(api, lsTitle, format, ##__VA_ARGS__)
//! Log Fatal, use it instead of LOG!
#define LOGFATAL_(api, format, ...)             LOG(api, lsFatal, format, ##__VA_ARGS__)
//! Log Error, use it instead of LOG!
#define LOGERROR_(api, format, ...)             LOG(api, lsError, format, ##__VA_ARGS__)
//! Log Warning, use it instead of LOG!
#define LOGWARN_(api, format, ...)              LOG(api, lsWarning, format, ##__VA_ARGS__)
//! Log Information, use it instead of LOG!
#define LOGINFO_(api, format, ...)              LOG(api, lsInfo, format, ##__VA_ARGS__)
//! Log Trace, use it instead of LOG!
#define LOGTRACE_(api, format, ...)             LOG(api, lsTrace, format, ##__VA_ARGS__)
#if (defined(DEBUG) || defined(_DEBUG))
  //! Log Debug, use it instead of LOG!
  #define LOGDEBUG_(api, format, ...)           LOG(api, lsDebug, format, ##__VA_ARGS__)
  //! Log Special, use it instead of LOG!
  #define LOGSPECIAL_(api, format, ...)         LOG(api, lsSpecial, format, ##__VA_ARGS__)
  //! Hexadecimal dump of memory
  #define HEXDUMP_(api, context, src, size)     __HexDump(api, context, src, size)
  //! Binary dump of memory
  #define BINDUMP_(api, context, src, size)     __BinDump(api, context, src, size)
#else
  #define LOGDEBUG_(api, format, ...)           do{}while(false)
  #define LOGSPECIAL_(api, format, ...)         do{}while(false)
  #define HEXDUMP_(api, context, src, size)     do{}while(false)
  #define BINDUMP_(api, context, src, size)     do{}while(false)
#endif

//-----------------------------------------------------------------------------
#endif /* USE_CONSOLE_TX */





//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************

/*! @defgroup ConsoleHash Calculus of the Rx console hash for commands
 * @details The hash of commands allows to do compiler time hash generation for all commands supported by a RX console
 */
//! @addtogroup ConsoleHash
//! @{
#define CONSOLE_HASH_INITIAL_VAL              ( 0x00000000u )                                                          //!< Initial value of the ROL5 XOR hash
#define CONSOLE_UPPER_CASE(aChar)             ( (((aChar) >= 'a') && ((aChar) <= 'z')) ? ((aChar) - 32) : (aChar) )    //!< Uppercase the char in argument, else let it untouch
#define CONSOLE_ROL5_XOR_CHAR(value,newData)  ( (((value) >> 27) | ((value) << 5)) ^ (newData) )                       //!< Perform a Rol 5 of hash following a xor of the char of the string

/*! @brief Get the ROL5 XOR hash of a maximum 8-char string
 * @warning Use this macro only on constant value of n to use the compiler simplification, else create a function with the code in details
 * @details This is the equivalent of: @code{ uint32_t Hash = CONSOLE_HASH_INITIAL_VAL; for (size_t idx = 0; idx < 8; ++idx) { uint32_t newData = (idx < (sizeof(str) - 1) ? str[idx] : 0); newData = (((newData >= 'a') && (newData <= 'z')) ? (newData - 32) : newData); Hash = ((Hash >> 27) | (Hash << 5)) ^ newData; } return Hash; }@endcode
 * @note Unfortunately, it is impossible to do a str[idx] in C pre-processing so all the first 8 chars shall be set manually
 * @param[in] char1 Is the first char of the string for which to generate a hash
 * @param[in] char2 Is the second char of the string for which to generate a hash
 * @param[in] char3 Is the third char of the string for which to generate a hash
 * @param[in] char4 Is the fourth char of the string for which to generate a hash
 * @param[in] char5 Is the fifth char of the string for which to generate a hash
 * @param[in] char6 Is the sixth char of the string for which to generate a hash
 * @param[in] char7 Is the seventh char of the string for which to generate a hash
 * @param[in] char8 Is the eighth char of the string for which to generate a hash
 * @return The ROL5 XOR hash of the 8 chars set in argument if any
 */
#define CONSOLE_ROL5XOR_HASH(char1, char2, char3, char4, char5, char6, char7, char8)  ( CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(                                                      \
                                                                                        CONSOLE_ROL5_XOR_CHAR(CONSOLE_HASH_INITIAL_VAL, CONSOLE_UPPER_CASE(char1)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char2)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char3)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char4)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char5)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char6)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char7)), \
                                                                                                                                        CONSOLE_UPPER_CASE(char8)) )
//! @}

//-----------------------------------------------------------------------------

#ifdef USE_CONSOLE_RX

/*! @defgroup ConsoleReceive Console receive API
 * @details The console receive API helps to perform received commands from a console
 */
//! @addtogroup ConsoleReceive
//! @{
#define CONSOLE_NULL  0x00 //!< Null
#define CONSOLE_SOH   0x01 //!< Start Of Heading
#define CONSOLE_STX   0x02 //!< Start of TeXt
#define CONSOLE_ETX   0x03 //!< End of TeXt
#define CONSOLE_EOT   0x04 //!< End Of Transmission
#define CONSOLE_ENQ   0x05 //!< ENQuiry (End of Line)
#define CONSOLE_ACK   0x06 //!< ACKnowledge
#define CONSOLE_BEL   0x07 //!< BELl
#define CONSOLE_BS    0x08 //!< BackSpace
#define CONSOLE_HT    0x09 //!< Horizontal Tab
#define CONSOLE_LF    0x0A //!< Line Feed
#define CONSOLE_VT    0x0B //!< Vertical Tab
#define CONSOLE_FF    0x0C //!< Form Feed
#define CONSOLE_CR    0x0D //!< Carriage Return
#define CONSOLE_SO    0x0E //!< Shift Out
#define CONSOLE_SI    0x0F //!< Shift In
#define CONSOLE_DLE   0x10 //!< Data Link Escape
#define CONSOLE_DC1   0x11 //!< Device Control 1
#define CONSOLE_DC2   0x12 //!< Device Control 2
#define CONSOLE_DC3   0x13 //!< Device Control 3
#define CONSOLE_DC4   0x14 //!< Device Control 4
#define CONSOLE_NAK   0x15 //!< Negative AcKnowledge
#define CONSOLE_SYN   0x16 //!< Synchronous Idle
#define CONSOLE_ETB   0x17 //!< End of Transmission Block
#define CONSOLE_CAN   0x18 //!< CANcel
#define CONSOLE_SUB   0x1A //!< SUBstitute
#define CONSOLE_ESC   0x1B //!< ESCape
#define CONSOLE_FS    0x1C //!< File Separator
#define CONSOLE_GS    0x1D //!< Group Separator
#define CONSOLE_RS    0x1E //!< Record Separator
#define CONSOLE_US    0x1F //!< Unit Separator
#define CONSOLE_DEL   0x7F //!< DELete

//-----------------------------------------------------------------------------

#if !defined(CONSOLE_RX_CURRENT_BUFFER_SIZE)
#  define CONSOLE_RX_CURRENT_BUFFER_SIZE  50 //!< Console Rx command acquisition default buffer size (for CurrentBuff)
#endif

//! Circular Buffer for Console receive structure
typedef struct ConsoleRx ConsoleRx;
struct ConsoleRx
{
  void *UserAPIData;          //!< Optional, can be used to store API data or NULL

  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  UART_Interface* UART;       //!< This is the UART_RxInterface descriptor pointer that will be used to communicate with the device
#else
  UART_Interface  UART;       //!< This is the UART_RxInterface descriptor that will be used to communicate with the device
#endif

  //--- Receive buffer ---
  volatile size_t RxIdx;      //!< This is the receive input index in the buffer (where data will be write after receive from UART)
  volatile size_t ProcessIdx; //!< This is the processing index in the buffer (where data will be read from CurrentBuff when processed)
  size_t CursorIdx;           //!< This is the cursor index in the buffer (where data will be inserted into CurrentBuff when processed)
  uint8_t CurrentBuff[CONSOLE_RX_CURRENT_BUFFER_SIZE]; // Current receive buffer (working buffer)

#ifdef CONSOLE_RX_USE_COMMAND_RECALL
  //--- Command buffer ---
  size_t StartIdx;            //!< This is the start of command line index in the buffer
  size_t CurrentIdx;          //!< This is the current index of the command line in the buffer
  char *CommandBuffer;        //!< The buffer itself (should be the same size as BufferSize)
  size_t BufferSize;          //!< The buffer size
#endif
};

//-----------------------------------------------------------------------------



/*! @brief Initialize the Console receive
 *
 * @param[in] *pApi Is the Console receive API to work with
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT InitConsoleRx(ConsoleRx* pApi);

/*! @brief Receive char from console
 * @note This can be used in an interrupt
 *
 * @param[in] *pApi Is the Console receive API to work with
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT ReceiveCharFromConsole(ConsoleRx* pApi);

/*! @brief Process received char from console
 *
 * @param[in] *pApi Is the Console receive API to work with
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT ProcessReceivedCharFromConsole(ConsoleRx* pApi);

/*! @brief Process received command Callback
 * @note This function is weak. Thus the used shall implement is own function
 *
 * @param[in] *pCmd Is the command string first char (NULL terminated string)
 * @param[in] size Is the char count of the command string pCmd
 * @return Returns an #eERRORRESULT value enum
 */
void ProcessReceivedCommandCallBack(const uint8_t* pCmd, size_t size) CONSOLE_WEAK;

//-----------------------------------------------------------------------------
//! @}
#endif /* USE_CONSOLE_RX */
//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CONSOLE_H_ */