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

//-----------------------------------------------------------------------------
#include "Console.h"
#include "Console_V71Interface.h"
#include "StringTools.h"
//-----------------------------------------------------------------------------
#if !defined(__cplusplus)
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// UART of V71
//********************************************************************************************************************
//! UART console interface definition for dynamic interfaces
#ifdef USE_DYNAMIC_INTERFACE
static UART_Interface Console_UART =
{
  UART_MEMBER(InterfaceDevice) CONSOLE_UART,
  UART_MEMBER(fnUART_Transmit) UARTtransmit_V71,
  UART_MEMBER(fnUART_Receive)  UARTreceive_V71,
  UART_MEMBER(Channel)         0,
};
#endif

//-----------------------------------------------------------------------------


//=============================================================================
// Console UART Tx initialization for the ATSAMV71
//=============================================================================
void ConsoleUART_TxInit_V71(void)
{
  usart_serial_options_t uart_serial_options = {};
  uart_serial_options.baudrate   = CONF_CONSOLE_BAUDRATE;
  uart_serial_options.charlength = CONF_CONSOLE_CHAR_LENGTH;
  uart_serial_options.paritytype = CONF_CONSOLE_PARITY;
  uart_serial_options.stopbits   = CONF_CONSOLE_STOP_BITS;

  //--- Enable the peripheral clock in the PMC ---
  sysclk_enable_peripheral_clock(CONSOLE_UART_ID);

  //--- Configure console UART ---
  //  usart_serial_init(CONSOLE_UART, &uart_serial_options); // For Console API
  stdio_serial_init(CONSOLE_UART, &uart_serial_options); // For printf

  //--- Enable Tx function ---
  usart_enable_tx(CONSOLE_UART);

  //--- Configure and enable interrupt of USART ---
  usart_disable_interrupt(CONSOLE_UART, US_IER_TXRDY | US_IER_TXEMPTY);
  NVIC_EnableIRQ(USART1_IRQn);                                          // *** USE WITH INTERRUPT CHAR SEND. IN SEND WHILE IDLE (while(true) in the main()) COMMENT THIS LINE
}


//=============================================================================
// UART transmit char function interface of the ATSAMV71
//=============================================================================
eERRORRESULT UARTtransmit_V71(UART_Interface *pIntDev, uint8_t *data, size_t size, size_t *actuallySent)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Usart* pUART = (Usart*)(pIntDev->InterfaceDevice); // Get the V71 USART device of this UART port
  *actuallySent = 0;
  if (size <= 0) return ERR_OK;
  if ((pUART->US_CSR & US_CSR_TXRDY  ) == 0) return ERR__NOT_READY; // Character is in the US_THR
  if ((pUART->US_CSR & US_CSR_TXEMPTY) == 0) return ERR__NOT_READY;
//  if ((pUART->US_IMR & US_IMR_TXRDY) >  0) return ERR__NOT_READY; // TX Ready interrupt is set  // *** USE WITH INTERRUPT CHAR SEND. IN SEND WHILE IDLE (while(true) in the main()) COMMENT THIS LINE

  pUART->US_THR = US_THR_TXCHR(*data);             // Send the char
  pUART->US_IER = (US_IER_TXRDY | US_IER_TXEMPTY); // Enable interrupts
  *actuallySent = 1;                               // Always 1 by 1 with this USART
  return ERR_OK;
}

//-----------------------------------------------------------------------------


//=============================================================================
// Console UART Rx initialization for the ATSAMV71
//=============================================================================
void ConsoleUART_RxInit_V71(void)
{
  usart_serial_options_t uart_serial_options = {};
  uart_serial_options.baudrate   = CONF_CONSOLE_BAUDRATE;
  uart_serial_options.charlength = CONF_CONSOLE_CHAR_LENGTH;
  uart_serial_options.paritytype = CONF_CONSOLE_PARITY;
  uart_serial_options.stopbits   = CONF_CONSOLE_STOP_BITS;

  //--- Enable the peripheral clock in the PMC ---
  sysclk_enable_peripheral_clock(CONSOLE_UART_ID);

  //--- Configure console UART ---
  //  usart_serial_init(CONSOLE_UART, &uart_serial_options); // For Console API
  stdio_serial_init(CONSOLE_UART, &uart_serial_options); // For printf

  //--- Enable Rx function ---
  usart_enable_rx(CONSOLE_UART);

  //--- Configure and enable interrupt of USART ---
  usart_enable_interrupt(CONSOLE_UART, US_IER_RXRDY);
  NVIC_EnableIRQ(USART1_IRQn);
}


//=============================================================================
// UART receive char function interface of the ATSAMV71
//=============================================================================
eERRORRESULT UARTreceive_V71(UART_Interface *pIntDev, uint8_t *data, size_t size, size_t *actuallyReceived, uint8_t *lastCharError)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Usart* pUART = (Usart*)(pIntDev->InterfaceDevice); // Get the V71 USART device of this UART port
  *actuallyReceived = 0;
  if (size <= 0) return ERR_OK;
  if ((pUART->US_CSR & US_CSR_RXRDY) == 0) return ERR__NO_DATA_AVAILABLE;

  *data = (char)(pUART->US_RHR & US_RHR_RXCHR_Msk); // Get the char
  *actuallyReceived = 1;                            // Always 1 by 1 with this USART
  *lastCharError = pUART->US_CSR & 0xE4;            // Get only Rx errors
  return ERR_OK;
}

//-----------------------------------------------------------------------------


//=============================================================================
// Handler for Console USART interrupt.
//=============================================================================
void USART1_Handler(void)
{
  //--- Transmission interrupts ---
  if ((CONSOLE_UART->US_CSR & (US_CSR_TXRDY | US_CSR_TXEMPTY)) > 0) // Transmit interrupt rises
  {
    CONSOLE_UART->US_IDR = (US_IDR_TXRDY | US_IDR_TXEMPTY);         // Disable interrupts
    TrySendingNextCharToConsole(CONSOLE_TX);
  }

  //--- Reception interrupts ---
  if ((CONSOLE_UART->US_CSR & US_CSR_RXRDY) > 0)                    // Receive interrupt rises
  {
//    char ReceivedChar;
//    ConsoleRx_GetChar_V71(CONSOLE_RX, &ReceivedChar);
  }
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Transmit API
//********************************************************************************************************************

//! Console Tx configuration
ConsoleTx Console_TxConf =
{
  CONSOLE_MEMBER(UserAPIData) NULL,
  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  CONSOLE_MEMBER(UART) &Console_UART,
#else
  CONSOLE_MEMBER(UART)
  {
    UART_MEMBER(InterfaceDevice) CONSOLE_UART,
    UART_MEMBER(fnUART_Transmit) UARTtransmit_V71,
    UART_MEMBER(fnUART_Receive)  NULL,             // Not used for Tx
    UART_MEMBER(Channel)         0,
  },
#endif
  //--- Transmit buffer ---
  CONSOLE_MEMBER(Buffer    ) &ConsoleTxBuffer[0],
  CONSOLE_MEMBER(BufferSize) CONSOLE_TX_BUFFER_SIZE,
};

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************

//! Console Rx configuration
ConsoleRx Console_RxConf =
{
  CONSOLE_MEMBER(UserAPIData) NULL,
  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  CONSOLE_MEMBER(UART) &Console_UART,
#else
  CONSOLE_MEMBER(UART)
  {
    UART_MEMBER(InterfaceDevice) CONSOLE_UART,
    UART_MEMBER(fnUART_Transmit) NULL,            // Not used for Rx
    UART_MEMBER(fnUART_Receive)  UARTreceive_V71,
    UART_MEMBER(Channel)         0,
  },
#endif

#ifdef CONSOLE_RX_USE_COMMAND_RECALL
  //--- Command buffer ---
  CONSOLE_MEMBER(CommandBuffer) &ConsoleRxCommandBuffer[0],
  CONSOLE_MEMBER(BufferSize   ) CONSOLE_RX_COMMAND_BUFFER_SIZE,
#endif
};

//-----------------------------------------------------------------------------


//! @brief List of supported console receive commands
const ConsoleCommand ConsoleCommandsList[] =
{
  { CONSOLE_ROL5XOR_HASH('G','P','I','O','\0','\0','\0','\0'), ProcessGPIOcommand },
};

//=============================================================================
// Process received command Callback
//=============================================================================
void ProcessReceivedCommandCallBack(const uint8_t* pCmd, size_t size)
{
  eERRORRESULT Error = ERR__NOT_SUPPORTED;
  
  //--- Generate hash of first parameter of the string ---
  uint32_t Hash = CONSOLE_HASH_INITIAL_VAL;
  bool EndCmd = false;
  for (size_t idx = 0; idx < 8; ++idx)
  {
    EndCmd |= (idx >= size) || (pCmd[idx] == ' ') || (pCmd[idx] == CONSOLE_NULL);  // End of command only if space or null char, or index out of string
    uint32_t newData = (EndCmd == false ? pCmd[idx] : 0);                          // If after end of command, force 0
    newData = (((newData >= 'a') && (newData <= 'z')) ? (newData - 32) : newData); // Force uppercase
    Hash = ((Hash >> 27) | (Hash << 5)) ^ newData;                                 // Do Rol 5 xor data
  }

  //--- Search into command list matching command and execute ---
  for (size_t zIdx = 0; zIdx < (sizeof(ConsoleCommandsList) / sizeof(ConsoleCommandsList[0])); ++zIdx)
  {
    if (Hash == ConsoleCommandsList[zIdx].Hash)
    {
      Error = ConsoleCommandsList[zIdx].fnCommandProcess(pCmd, size); // Process command in associated function
      if (Error == ERR_OK) break;                                     // Exit if command have been successfully processed, else give if to another hash
    }    
  }
  if (Error == ERR__NOT_SUPPORTED)
  {
    LOGDEBUG("Unknown command (Key: %s; Hash: 0x%8X", pCmd, (unsigned int)Hash);
  }
}

//-----------------------------------------------------------------------------


//=============================================================================
// Process GPIO command
//=============================================================================
eERRORRESULT ProcessGPIOcommand(const uint8_t* pCmd, size_t size)
{
  if (size < 9) return ERR__PARSE_ERROR;
  uint32_t Value = 0;

  //--- Parse "GPIO" command ---
  size_t Index = 0;
  while ((*pCmd != ' ') && (*pCmd != 0)) { ++pCmd; ++Index; } // Search end of "GPIO" command
  if ((Index != 5) || (*pCmd == 0)) return ERR__PARSE_ERROR;  // Check the good position of the parse
  ++pCmd;

  if ((pCmd[0] >= '0') && (pCmd[0] <= '9'))                          // Is a digit?
  {
    switch (pCmd[1])
    {
      case 'x':
        pCmd += 2;                                                   // Start with "0x"? Pass these chars
        Value = HexStringToUint((char**)&pCmd);                      // Extract data
        if ((*pCmd != ' ') && (*pCmd != 0)) return ERR__PARSE_ERROR; // Check the good position of the parse
        break;

      case 'b':
        pCmd += 2;                                                   // Start with "0b"? Pass these chars
        Value = BinStringToUint((char**)&pCmd);                      // Extract data
        if ((*pCmd != ' ') && (*pCmd != 0)) return ERR__PARSE_ERROR; // Check the good position of the parse
        break;

      default:
        Value = StringToInt((char**)&pCmd);                          // Extract data
        if ((*pCmd != ' ') && (*pCmd != 0)) return ERR__PARSE_ERROR; // Check the good position of the parse
        break;
    }
  }
  else                                                               // Is a string
  {
    
    
  }

  //--- Set GPIO command ---
  
  return ERR_OK;
}

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------