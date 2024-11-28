/*
 * Hardware setup:
 *
 * Plug the V71_XplainedUltra_CAN_Shield (Custom board: https://github.com/Emandhal/CANmanager/tree/main/V71_XplainedUltra_CAN_Shield) board on shield connectors of the SAM V71 Xplained Ultra evaluation kit.
 * For this test configuration, the MIKROBUS1 connector of the shield board will get a MCP2518FD click (3.3V, 40MHz)
 * For this test configuration, the MIKROBUS2 connector of the shield board will get a TCAN4550 CAN FD6 click (3.3V)
 * For this test configuration, the MIKROBUS3 connector of the shield board will get a MCP2517FD click (3.3V, 40MHz)
 * Power SAM V71 Xplained by connecting a USB cable to the DEBUG connector and plugging it into your PC.
 * Connect a terminal to see the console debug
 *
 * CAN bus :
 * Connect a wire on the DB9 pin 7 MCP2518FD click and the other end to the H7 pin 1 (CAN_H) of the V71_XplainedUltra_CAN_Shield
 * Connect a wire on the DB9 pin 2 MCP2518FD click and the other end to the H7 pin 2 (CAN_L) of the V71_XplainedUltra_CAN_Shield
 * Connect a wire on the DB9 pin 7 TCAN4550 click and the other end to the H6 pin 1 (CAN_H) of the V71_XplainedUltra_CAN_Shield
 * Connect a wire on the DB9 pin 2 TCAN4550 click and the other end to the H6 pin 2 (CAN_L) of the V71_XplainedUltra_CAN_Shield
 * Connect a wire on the DB9 pin 7 MCP2517FD click and the other end to the H5 pin 1 (CAN_H) of the V71_XplainedUltra_CAN_Shield
 * Connect a wire on the DB9 pin 2 MCP2517FD click and the other end to the H5 pin 2 (CAN_L) of the V71_XplainedUltra_CAN_Shield
 * Connect a terminal resistor with a jumper on H1, and another on H2
 * Connect another terminal resistor with a jumper on H3, and another on H4 if necessary
 */
//=============================================================================

//-----------------------------------------------------------------------------
#include "string.h"
#include "Main.h"
#include "TimerTicks.h"
#include "Interface/Console_V71Interface.h"
//-----------------------------------------------------------------------------

volatile uint32_t msCount; //!< Milli-seconds count from start of the system

//-----------------------------------------------------------------------------

const char ButtonStateStr[2][7+1/* \0 */] =
{
  "push",
  "release",
};

//-----------------------------------------------------------------------------
#ifdef USE_CONSOLE_GPIO_COMMANDS

//! Description of each PORTs A to Z available on the V71 Xplained Ultra board
PORT_Interface* const PORTAtoZ[] =
{
  &IOPORTA, //!< PORTA
  &IOPORTB, //!< PORTB
  &IOPORTC, //!< PORTC
  &IOPORTD, //!< PORTD
  &IOPORTE, //!< PORTE
};
const size_t PORTAtoZ_COUNT = ( sizeof(PORTAtoZ) / sizeof(PORTAtoZ[0]) ); //!< Count of PORTs available on the V71 Xplained Ultra board

//! Description of each PORTs 0 to 9 available on the V71 Xplained Ultra board with the CAN_Shield board
PORT_Interface* const PORT0to9[] =
{
  &PORTGP_U12, //!< PORT0
  &PORTGPA_U8, //!< PORT1
  &PORTGPB_U8, //!< PORT2
  &PORTGPA_U9, //!< PORT3
  &PORTGPB_U9, //!< PORT4
};
const size_t PORT0to9_COUNT = ( sizeof(PORT0to9) / sizeof(PORT0to9[0]) ); //!< Count of PORTs of external devices available

#endif
//-----------------------------------------------------------------------------
#ifdef USE_CONSOLE_EEPROM_COMMANDS
#include "EEPROM.h"

//! Description of each EEPROM devices on the V71 Xplained Ultra board
EEPROM* const EEPROMdevices[] =
{
  &AT24MAC402_V71.Eeprom,
};
const size_t EEPROM_DEVICE_COUNT = ( sizeof(EEPROMdevices) / sizeof(EEPROMdevices[0]) ); //!< Count of EEPROMs available on the V71 Xplained Ultra board

#endif // USE_CONSOLE_EEPROM_COMMANDS
//-----------------------------------------------------------------------------

#include "Console_V71Interface.h"

//! Description of each UART available on the V71 Xplained Ultra board
UART_Interface* const UARTdevices[] =
{
  &Console_UART,
};
const size_t UART_DEVICE_COUNT = ( sizeof(UARTdevices) / sizeof(UARTdevices[0]) ); //!< Count of UARTs available on the V71 Xplained Ultra board

//-----------------------------------------------------------------------------





//=============================================================================
// SysTick Handler
//=============================================================================
void SysTick_Handler(void)
{
  msCount++;
}





//=============================================================================
// Main
//=============================================================================
int main (void)
{
  wdt_disable(WDT);

  //--- Configure system clock --------------------------
  sysclk_init();
  SystemCoreClock = sysclk_get_cpu_hz();

  //--- Initialize board --------------------------------
  board_init();
  // Configure the push button 2
  ioport_set_pin_dir(GPIO_PUSH_BUTTON_2, IOPORT_DIR_INPUT);
  ioport_set_pin_mode(GPIO_PUSH_BUTTON_2, GPIO_PUSH_BUTTON_2_FLAGS);
  ioport_set_pin_sense_mode(GPIO_PUSH_BUTTON_2, GPIO_PUSH_BUTTON_2_SENSE);
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO12; // PB12 function selected (Button2 is connected on PB12)
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;  // PB4 function selected (EXT2_INT1 connected on PB4)
  SystemAndStatusLED_Init();
  SetSystemLEDblinkMode(BLINK_10Hz);
  SetStatusLEDblinkMode(BLINK_OFF);

  //--- Initialize the console UART ---------------------
#ifdef USE_CONSOLE_TX
  ConsoleUART_TxInit_V71();
  InitConsoleTx(CONSOLE_TX);
#endif
#ifdef USE_CONSOLE_RX
  ConsoleUART_RxInit_V71();
  InitConsoleRx(CONSOLE_RX);
#endif

  //--- Demo start --------------------------------------
  printf("\r\n\r\n");
  LOGTITLE("CANmanager Demo start...");

  //--- Configure SysTick base timer --------------------
  SysTick_Config(SystemCoreClock * SYSTEM_TICK_MS / 1000); // (Fmck(Hz)*1/1000)=1ms
  
  //--- Configure SPI0 ----------------------------------
  eERRORRESULT ErrorSPI = SPI_Init(SPI0, &SPI0_Config);
  if (ErrorSPI != ERR_NONE)
  {
    ioport_set_pin_level(LED0_GPIO, LED0_ACTIVE_LEVEL);
    ioport_set_pin_level(LED1_GPIO, LED1_ACTIVE_LEVEL);
    LOGFATAL("Unable to configure SPI0 (error code: %u), END OF DEMO", (unsigned int)ErrorSPI);
    while (true) TrySendingNextCharToConsole(CONSOLE_TX); // Stay stuck here
  }

  //--- Reset watchdog ----------------------------------
  wdt_restart(WDT);

  //--- Log ---------------------------------------------
  LOGTRACE("Initialization complete");
  SetSystemLEDblinkMode(BLINK_STAY_ON);

  //=== The main loop ===================================
  while(1)
  {
    //--- Flush char by char console buffer ---
#ifdef USE_CONSOLE_TX
    TrySendingNextCharToConsole(CONSOLE_TX);
#endif

    //--- Process console reception ---
#ifdef USE_CONSOLE_RX
    ConsoleRx_ProcessReceivedChars(CONSOLE_RX);
#endif

    if (ThereIs10msTick())
    {

    }

    if (ThereIs20msTick())
    {

    }

    if (ThereIs100msTick())
    {

    }

    if (ThereIs250msTick())
    {

    }

    if (ThereIs500msTick())
    {

    }

    if (ThereIs1sTick())
    {

    }

    if (ThereIs2sTick())
    {

    }

    nop();
  }
}