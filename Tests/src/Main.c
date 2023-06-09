/*
 * Hardware setup:
 *
 * 1) Plug the mikroBus Xplained Pro adapter board into connector EXT1 of the SAM V71 Xplained Ultra evaluation kit.
 * 2) Select 3.3V at the jumper selection: VDD of the MCP2517FD/MCP2518FD.
 * 3) Select 40MHz at the jumper selection of the frequency of the MCP2517FD/MCP2518FD.
 * 4) Connect 5V of the POWER connector of SAM V71 Xplained Ultra to 5V External Power Header of the adapter board (jumper wire): 5V for VCC of ATA6563.
 * 5) Plug MCP2517FD click or a MCP2518FD click into adapter board.
 * 6) Power SAM V71 Xplained by connecting a USB cable to the DEBUG connector and plugging it into your PC.
 * 7) Connect a terminal to see the console debug
 * 8) Connect a wire on the DB9 pin 7 MCP2517FD/MCP2518FD click and the other end to the J1000 pin 1 (CAN_H) of the SAM V71 Xplained Ultra
 * 9) Connect a wire on the DB9 pin 2 MCP2517FD/MCP2518FD click and the other end to the J1000 pin 2 (CAN_L) of the SAM V71 Xplained Ultra
 * 10) Make sure both ends of the CAN bus are terminated.
 * 
 * Bonus add a secondary board:
 * 11) Plug the mikroBus Xplained Pro adapter board into connector EXT2 of the SAM V71 Xplained Ultra evaluation kit.
 * 12) Select 3.3V at the jumper selection: VDD of the MCP2517FD/MCP2518FD.
 * 13) Select 40MHz at the jumper selection of the frequency of the MCP2517FD/MCP2518FD.
 * 14) Connect 5V of the POWER connector of SAM V71 Xplained Ultra to 5V External Power Header of the adapter board (jumper wire): 5V for VCC of ATA6563.
 * 15) Plug MCP2517FD click or a MCP2518FD click into adapter board.
 * 16) Connect the DB9 to a CAN to PC adapter to see some frame on the computer
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
  InitConsoleTx(CONSOLE_TX);
#endif
#ifdef USE_CONSOLE_RX
  InitConsoleRx(CONSOLE_RX);
#endif

  //--- Demo start --------------------------------------
  printf("\r\n\r\n");
  LOGTITLE("CANmanager Demo start...");

  //--- Configure SysTick base timer --------------------
  SysTick_Config(SystemCoreClock * SYSTEM_TICK_MS / 1000); // (Fmck(Hz)*1/1000)=1ms

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