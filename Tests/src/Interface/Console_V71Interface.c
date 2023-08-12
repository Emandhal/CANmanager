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

//-----------------------------------------------------------------------------
#include "Console_V71Interface.h"
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
  if (size <= 0) return ERR_NONE;
  if ((pUART->US_CSR & US_CSR_TXRDY  ) == 0) return ERR__NOT_READY; // Character is in the US_THR
  if ((pUART->US_CSR & US_CSR_TXEMPTY) == 0) return ERR__NOT_READY;
//  if ((pUART->US_IMR & US_IMR_TXRDY) >  0) return ERR__NOT_READY; // TX Ready interrupt is set  // *** USE WITH INTERRUPT CHAR SEND. IN SEND WHILE IDLE (while(true) in the main()) COMMENT THIS LINE

  pUART->US_THR = US_THR_TXCHR(*data);             // Send the char
  pUART->US_IER = (US_IER_TXRDY | US_IER_TXEMPTY); // Enable interrupts
  *actuallySent = 1;                               // Always 1 by 1 with this USART
  return ERR_NONE;
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
  if (size <= 0) return ERR_NONE;
  if ((pUART->US_CSR & US_CSR_RXRDY) == 0) return ERR__NO_DATA_AVAILABLE;

  *data = (char)(pUART->US_RHR & US_RHR_RXCHR_Msk); // Get the char
  *actuallyReceived = 1;                            // Always 1 by 1 with this USART
  *lastCharError = pUART->US_CSR & 0xE4;            // Get only Rx errors
  return ERR_NONE;
}

//-----------------------------------------------------------------------------


//! The current Command Input buffer
CommandInputBuf CommandInput;

//=============================================================================
// Handler for Console USART interrupt.
//=============================================================================
void USART1_Handler(void)
{
  //--- Transmission interrupts ---
#ifdef USE_CONSOLE_TX
  if ((CONSOLE_UART->US_CSR & (US_CSR_TXRDY | US_CSR_TXEMPTY)) > 0) // Transmit interrupt rises
  {
    CONSOLE_UART->US_IDR = (US_IDR_TXRDY | US_IDR_TXEMPTY);         // Disable interrupts
    TrySendingNextCharToConsole(CONSOLE_TX);
  }
#endif

  //--- Reception interrupts ---
#ifdef USE_CONSOLE_RX
  if ((CONSOLE_UART->US_CSR & US_CSR_RXRDY) > 0)                    // Receive interrupt rises
  {
    ConsoleRx_ReceiveChar(CONSOLE_RX);
  }
#endif
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Transmit API
//********************************************************************************************************************
#ifdef USE_CONSOLE_TX

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

#endif // USE_CONSOLE_TX
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************
#ifdef USE_CONSOLE_RX

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





//**********************************************************************************************************************************************************
#ifdef USE_CONSOLE_GPIO_COMMANDS
//=============================================================================
// Process GPIO command Callback
//=============================================================================
void ConsoleRx_GPIOcommandCallBack(eConsoleActions action, eGPIO_PortPin portPin, uint8_t pinNum, uint32_t value, uint32_t mask)
{
  //--- Set GPIO command ---
  if (portPin == No_PORTpin) return;
  uint32_t Result = 0;
  switch (action)
  {
    default:
    case Action_None:
      break;

    case Action_Read:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) Result = ioport_get_port_level(((uint32_t)portPin - (uint32_t)PORTA), mask);
      else Result = ioport_get_pin_level(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum));
      LOGINFO("GPIO Direction: 0x%x", (unsigned int)Result);
      break;

    case Action_Write:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) ioport_set_port_level(((uint32_t)portPin - (uint32_t)PORTA), mask, value);
      else ioport_set_pin_level(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum), value);
      break;

    case Action_Set:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) ioport_set_port_level(((uint32_t)portPin - (uint32_t)PORTA), mask, IOPORT_PIN_LEVEL_HIGH);
      else ioport_set_pin_level(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum), IOPORT_PIN_LEVEL_HIGH);
      break;

    case Action_Clear:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) ioport_set_port_level(((uint32_t)portPin - (uint32_t)PORTA), mask, IOPORT_PIN_LEVEL_LOW);
      else ioport_set_pin_level(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum), IOPORT_PIN_LEVEL_LOW);
      break;

    case Action_Toggle:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) ioport_toggle_port_level(((uint32_t)portPin - (uint32_t)PORTA), mask);
      else ioport_toggle_pin_level(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum));
      break;

    case Action_Dir:
      if ((portPin >= PORTA) && (portPin < PORTa_Max)) ioport_set_port_dir(((uint32_t)portPin - (uint32_t)PORTA), mask, value);
      else ioport_set_pin_dir(((((uint32_t)portPin - (uint32_t)PA) * 32) + pinNum), value);
      break;
  }
}
#endif // USE_CONSOLE_GPIO_COMMANDS
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#ifdef USE_CONSOLE_EEPROM_COMMANDS

// Generate a lookup table for 8-bits integers
#define B2(n) n, n + 1, n + 1, n + 2
#define B4(n) B2(n), B2(n + 1), B2(n + 1), B2(n + 2)
#define B6(n) B4(n), B4(n + 1), B4(n + 1), B4(n + 2)

// Lookup table of '1' in each 8-bits values
uint8_t Uint8_1Count_LUT[256] = { B6(0), B6(1), B6(1), B6(2) };

//=============================================================================
// Show the device memory
//=============================================================================
static void __ShowEEPROMmapping(uint8_t index)
{
  eERRORRESULT Error = ERR_OK;
  volatile uint32_t OneBitsCount = 0;
  uint8_t  PageBuffer[8];
  uint32_t EepromSize     = EEPROMdevices[index]->Conf->ArrayByteSize; // Get EEPROM total size
  uint32_t EepromPageSize = EEPROMdevices[index]->Conf->PageSize;      // Get EEPROM page size
  if (EepromSize == EepromPageSize)                                    // The EEPROM does not have pages, the memory is in one bloc
  {                                                                    // Search a pseudo page division
    //--- Divide by 80 (to fit inside a 80 lines console for < 24kB) ---
    EepromPageSize /= 80;
    uint32_t BestPowerOf2 = 1;
    while (BestPowerOf2 < EepromPageSize) BestPowerOf2 <<= 1;
    EepromPageSize = BestPowerOf2;
    if (EepromPageSize <   8) EepromPageSize =   8;
    if (EepromPageSize > 256) EepromPageSize = 256;
  }

  //--- Read page per page and create mapping ---
  LOGINFO("Visual mapping of device %u, page size %u:", (unsigned int)(index), (unsigned int)EepromPageSize);
  for (size_t zMem = 0; zMem < (EepromSize / EepromPageSize); ++zMem)
  {
    OneBitsCount = 0;
    
    for (size_t zPage = 0; zPage < (EepromPageSize / sizeof(PageBuffer)); ++zPage)
    {
      //--- Read a page ---
      Error = EEPROM_ReadData(EEPROMdevices[index], (zMem * EepromPageSize) + (zPage * sizeof(PageBuffer)), &PageBuffer[0], sizeof(PageBuffer));
      if (Error != ERR_OK)
      {
        LOGERROR("Unable to read EEPROM memory (error code: %u)", (unsigned int)Error);
        return;
      }
      //--- Count number of bit at '1' ---
      for (int32_t zByte = sizeof(PageBuffer); --zByte >= 0;) OneBitsCount += Uint8_1Count_LUT[(uint8_t)PageBuffer[zByte]];
    }

    //--- Fill visual mapping ---
    char CharToSend = (char)250;                                      // Char not filled (default)                                    
    if (OneBitsCount >= (EepromPageSize * 2)) CharToSend = (char)176; // Char filled a little
    if (OneBitsCount >= (EepromPageSize * 4)) CharToSend = (char)177; // Char partially filled
    if (OneBitsCount >= (EepromPageSize * 6)) CharToSend = (char)178; // Char filled mostly
    if (OneBitsCount >= (EepromPageSize * 8)) CharToSend = (char)219; // Char fully filled
    SetCharToConsoleBuffer(CONSOLE_TX, CharToSend);
  }
}


//=============================================================================
// Dump the device memory
//=============================================================================
static void __DumpEEPROMmemory(uint8_t index, uint32_t address, uint32_t size)
{
  eERRORRESULT Error = ERR_OK;
  static const char* Hexa = "0123456789ABCDEF";
  uint8_t ReadBuffer[16];
  
#define ROW_LENGTH  16           // 16 bytes per row
  char HexaDump[ROW_LENGTH * 3]; // [2 digit hexa + space] - 1 space + 1 zero terminal
  char HexaChar[ROW_LENGTH + 1]; // [1 char] + 1 zero terminal
  size_t SizeToRead = (size_t)(EEPROMdevices[index]->Conf->ArrayByteSize - address); // Set the full device size by default
  if (SizeToRead > size) SizeToRead = size;
  LOGINFO("Dump %d bytes at 0x%04X", SizeToRead, (unsigned int)address);

  //--- Dump the data read ---
  uint8_t* pSrc = &ReadBuffer[0];
  HexaChar[ROW_LENGTH] = 0;
  for (int32_t i = ((SizeToRead+ROW_LENGTH-1) / ROW_LENGTH); --i >= 0; pSrc += ROW_LENGTH, SizeToRead -= ROW_LENGTH, address += ROW_LENGTH)
  {
    //--- Read the memory ---
    Error = EEPROM_ReadData(EEPROMdevices[index], address, &ReadBuffer[0], (SizeToRead >= ROW_LENGTH ? ROW_LENGTH : SizeToRead));
    if (Error != ERR_OK)
    {
      LOGERROR("Unable to read EEPROM memory (error code: %u)", (unsigned int)Error);
      return;
    }
    
    //--- Dump ---
    memset(HexaDump, ' ', sizeof(HexaDump));
    memset(HexaChar, '.', ROW_LENGTH);
    for (int j = (SizeToRead >= ROW_LENGTH ? ROW_LENGTH : SizeToRead); --j >= 0;)
    {
      HexaDump[j * 3 + 0] = Hexa[(pSrc[j] >> 4) & 0xF];
      HexaDump[j * 3 + 1] = Hexa[(pSrc[j] >> 0) & 0xF];
      //HexaDump[j * 3 + 2] = ' ';
      HexaChar[j] = (pSrc[j] < 0x20) ? '.' : pSrc[j];
    }
    HexaDump[ROW_LENGTH * 3 - 1] = 0;
    LOGINFO("  %04X : %s \"%s\"", (unsigned int)address, HexaDump, HexaChar);
  }
#undef ROW_LENGTH
}


//=============================================================================
// Process EEPROM command Callback
//=============================================================================
void ConsoleRx_EEPROMcommandCallBack(eConsoleActions action, uint8_t index, uint32_t address, uint32_t size, char* data)
{
  if (index >= EEPROM_DEVICE_COUNT) { LOGERROR("EEPROM index out of range"); return; } // Check index
  eERRORRESULT Error;
  
  switch (action)
  {
    default:
    case Action_None:
      break;

    case Action_Read:
      break;

    case Action_Write:
      break;

    case Action_Clear:
      {
        uint8_t WriteBuffer[16];
        memset(&WriteBuffer[0], 0xFF, sizeof(WriteBuffer));
        for (size_t zMem = 0; zMem < (EEPROMdevices[index]->Conf->ArrayByteSize / sizeof(WriteBuffer)); ++zMem)
        {
          //--- Write buffer ---
          Error = EEPROM_WriteData(EEPROMdevices[index], (zMem * sizeof(WriteBuffer)), &WriteBuffer[0], sizeof(WriteBuffer));
          if (Error != ERR_OK)
          {
            LOGERROR("Unable to write EEPROM memory (error code: %u)", (unsigned int)Error);
            return;
          }
        }
        LOGINFO("The memory has been cleared");
      }
      break;

    case Action_Show:
      __ShowEEPROMmapping(index);
      break;
    
    case Action_Dump:
      __DumpEEPROMmemory(index, address, size);
      break;
  }    
}
#endif // USE_CONSOLE_EEPROM_COMMANDS
#endif // USE_CONSOLE_RX

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------