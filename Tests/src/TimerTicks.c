/*!*****************************************************************************
 * @file    TimerTicks.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/02/2020
 * @brief   Timer ticks and System LED management
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "TimerTicks.h"
//-----------------------------------------------------------------------------

#define UNIT_ERR_CONTEXT  ERRCONTEXT__TIMERTICKS // Error context of this unit

//------------------------------------------------------------------------------

#if defined(__cplusplus)
  using namespace std;
  using namespace std::chrono;
  SIMULATION_Mutex Locker_Ticks;  //! Mutex for the access of timer ticks
  SIMULATION_Mutex Locker_SysLED; //! Mutex for the access of System and Status LEDs state
#endif

//-----------------------------------------------------------------------------

//--- Ticks variables ---
static uint32_t Tick1ms;
static uint32_t Tick10ms;
static uint32_t Tick20ms;
static uint32_t Tick100ms;
static uint32_t Tick200ms;
static uint32_t Tick250ms;
static uint32_t Tick500ms;
static uint32_t Tick1s;
static uint32_t Tick2s;

//--- Counting variables ---
static volatile uint32_t msTickCount;
static uint32_t TickBlinkSystemLED;
static uint32_t TickBlinkStatusLED;

//--- Watchdog variables ---
#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
  static uint32_t TickWatchdog;
  static uint32_t RefreshWatchdogTickCount;
#endif

eLEDBlinkMode SystemLEDBlinkMode; // Typically LED0
eLEDBlinkMode StatusLEDBlinkMode; // Typically LED1

//-----------------------------------------------------------------------------

//! Forward definition
void __TimerTicks_ProcessIT(void);

//-----------------------------------------------------------------------------



 

//=============================================================================
// Initialize the timer ticks
//=============================================================================
eERRORRESULT TimerTicks_Init(void)
{
  //--- Init variables ---
  Tick1ms     = 0;
  Tick10ms    = 0;
  Tick20ms    = 0;
  Tick100ms   = 0;
  Tick200ms   = 0;
  Tick250ms   = 0;
  Tick500ms   = 0;
  Tick1s      = 0;
  Tick2s      = 0;
  msTickCount = 0;

    //--- Init watchdog ---
#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
  TickWatchdog = 0;
  RefreshWatchdogTickCount = (TIMER_TICKS_PER_SECONDS / TimerTickWatchdog_Conf.WatchdogProcessFrequency);
#endif

  //--- Configure the timer ---
//  if (Error != ERR_OK) return ERR_GENERATE(ERR__CONFIGURATION);
  return ERR_OK;
}

void __TimerTicks_ProcessIT(void)
{
  RefreshSystemLEDstate(TICK_TO_ADD_FOR_MS);
  RefreshStatusLEDState(TICK_TO_ADD_FOR_MS);

  LOCK(Ticks);
  msTickCount += TICK_TO_ADD_FOR_MS;

  //--- Watchdog ticks ---
#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
  TickWatchdog++;
  if (TickWatchdog >= RefreshWatchdogTickCount)
  {
    TickWatchdog -= RefreshWatchdogTickCount;
    if (TimerTickWatchdog_Conf.fnWatchDogProcess != NULL) TimerTickWatchdog_Conf.fnWatchDogProcess();
  }
#endif
    
  //--- 1ms ticks ---
#if COUNT_1ms_IN_TIMER_TICKS != 0
  Tick1ms++;
#endif

  //--- 10ms ticks ---
#if COUNT_10ms_TIMER_TICKS != 0
  Tick10ms++;
#endif

    //--- 20ms ticks ---
#if COUNT_20ms_IN_TIMER_TICKS != 0
  Tick20ms++;
#endif

  //--- 100ms ticks ---
#if COUNT_100ms_IN_TIMER_TICKS != 0
  Tick100ms++;
#endif

  //--- 200ms ticks ---
#if COUNT_200ms_IN_TIMER_TICKS != 0
  Tick200ms++;
#endif

  //--- 250ms ticks ---
#if COUNT_250ms_IN_TIMER_TICKS != 0
  Tick250ms++;
#endif

  //--- 500ms ticks ---
#if COUNT_500ms_IN_TIMER_TICKS != 0
  Tick500ms++;
#endif

  //--- 1s ticks ---
#if COUNT_1s_IN_TIMER_TICKS != 0
  Tick1s++;
#endif

  //--- 2s ticks ---
#if COUNT_2s_IN_TIMER_TICKS != 0
  Tick2s++;
#endif

  //--- Clear timer flag ---
  // Cleared by HAL
  UNLOCK(Ticks);
}


#if COUNT_1ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 1000Hz (1ms) tick?
//=============================================================================
bool ThereIs1msTick(void)
{
  if (Tick1ms >= COUNT_1ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick1ms -= COUNT_1ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick1ms >= COUNT_1ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 1ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_10ms_TIMER_TICKS != 0
//=============================================================================
// Is there a 100Hz (10ms) tick?
//=============================================================================
bool ThereIs10msTick(void)
{
  if (Tick10ms >= COUNT_10ms_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick10ms -= COUNT_10ms_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick10ms >= COUNT_10ms_TIMER_TICKS)
    {
      LOGDEBUG("A tick 10ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_20ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 50Hz (20ms) tick?
//=============================================================================
bool ThereIs20msTick(void)
{
  if (Tick20ms >= COUNT_20ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick20ms -= COUNT_20ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick20ms >= COUNT_20ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 20ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_100ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 10Hz (100ms) tick?
//=============================================================================
bool ThereIs100msTick(void)
{
  if (Tick100ms >= COUNT_100ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick100ms -= COUNT_100ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick100ms >= COUNT_100ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 100ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_200ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 5Hz (200ms) tick?
//=============================================================================
bool ThereIs200msTick(void)
{
  if (Tick200ms >= COUNT_200ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick200ms -= COUNT_200ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick200ms >= COUNT_200ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 200ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_250ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 4Hz (250ms) tick?
//=============================================================================
bool ThereIs250msTick(void)
{
  if (Tick250ms >= COUNT_250ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick250ms -= COUNT_250ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick250ms >= COUNT_250ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 250ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_500ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 2Hz (500ms) tick?
//=============================================================================
bool ThereIs500msTick(void)
{
  if (Tick500ms >= COUNT_500ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick500ms -= COUNT_500ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick500ms >= COUNT_500ms_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 500ms was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_1s_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 1Hz (1s) tick?
//=============================================================================
bool ThereIs1sTick(void)
{
  if (Tick1s >= COUNT_1s_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick1s -= COUNT_1s_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick1s >= COUNT_1s_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 1s was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


#if COUNT_2s_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 0.5Hz (2s) tick?
//=============================================================================
bool ThereIs2sTick(void)
{
  if (Tick2s >= COUNT_2s_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick2s -= COUNT_2s_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick2s >= COUNT_2s_IN_TIMER_TICKS)
    {
      LOGDEBUG("A tick 2s was not treat in time");
    }
#endif
    return true;
  }
  return false;
}
#endif


//=============================================================================
// Get current millisecond
//=============================================================================
uint32_t GetCurrentMs(void)
{
  return msTickCount;
}


//=============================================================================
// Sleep x millisecond
//=============================================================================
void Sleep_ms(uint16_t mSec)
{ // This procedure is not accurate, the range of sleep is: mSec < Sleep < mSec+1, and depend to the timer refresh
  uint32_t Timeout;
    
  Timeout = msTickCount + mSec;
  while (Timeout > msTickCount)
  {
#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
    if (TimerTickWatchdog_Conf.fnWatchdogRestart != NULL) TimerTickWatchdog_Conf.fnWatchdogRestart();
#endif
  }
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// LEDs management
//********************************************************************************************************************
//=============================================================================
// Initialize the system and status LEDs managment
//=============================================================================
void SystemAndStatusLED_Init(void)
{
  //SYSLED_As_Output;
  //STATUSLED_As_Output;

  //--- Set initial blink mode ---
  SetSystemLEDblinkMode(BLINK_1Hz);
  SetStatusLEDblinkMode(BLINK_USER_MANAGEMENT);
}


//=============================================================================
// Set system LED blink mode
//=============================================================================
void SetSystemLEDblinkMode(eLEDBlinkMode blinkMode)
{
  if (blinkMode != SystemLEDBlinkMode)
  {
    LOCK(SysLED);
    SystemLEDBlinkMode = blinkMode;
    if (blinkMode == BLINK_USER_MANAGEMENT) /*SYSLED_Low*/; // Force OFF at initialization
    UNLOCK(SysLED);
    TickBlinkSystemLED = 0;
  }
}


//=============================================================================
// Refresh system LED state
//=============================================================================
void RefreshSystemLEDstate(uint32_t ticks)
{
  TickBlinkSystemLED += ticks;

  switch (SystemLEDBlinkMode)
  {
    case BLINK_OFF:
        //SYSLED_Low;
        break;

#if COUNT_BLINK_10Hz_IN_TIMER_TICKS != 0
    case BLINK_10Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_10Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_10Hz_IN_TIMER_TICKS;
          //SYSLED_Invert;
      }
      break;
#endif

#if COUNT_BLINK_2Hz_IN_TIMER_TICKS != 0
    case BLINK_2Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_2Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_2Hz_IN_TIMER_TICKS;
          //SYSLED_Invert;
        }
        break;
#endif

    case BLINK_1Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_1Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_1Hz_IN_TIMER_TICKS;
          //SYSLED_Invert;
        }
        break;

    case BLINK_STAY_ON:
        //SYSLED_High;
        break;

    case BLINK_USER_MANAGEMENT:
    default:
        TickBlinkSystemLED = 0;
        break;
  }
}


//=============================================================================
// Set status LED blink mode
//=============================================================================
void SetStatusLEDblinkMode(eLEDBlinkMode blinkMode)
{
  if (blinkMode != StatusLEDBlinkMode)
  {
    LOCK(SysLED);
    StatusLEDBlinkMode = blinkMode;
    if (blinkMode == BLINK_USER_MANAGEMENT) /*STATUSLED_Low*/; // Force OFF at initialization
    UNLOCK(SysLED);
    TickBlinkStatusLED = TickBlinkSystemLED; // For synchronization with the system LED
  }
}


//=============================================================================
// Refresh status LED state
//=============================================================================
void RefreshStatusLEDState(uint32_t ticks)
{
  TickBlinkStatusLED += ticks;

  switch (StatusLEDBlinkMode)
  {
    case BLINK_OFF:
        //STATUSLED_Low;
        break;

#if COUNT_BLINK_10Hz_IN_TIMER_TICKS != 0
    case BLINK_10Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_10Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_10Hz_IN_TIMER_TICKS;
          //STATUSLED_Invert;
        }
        break;
#endif

#if COUNT_BLINK_2Hz_IN_TIMER_TICKS != 0
    case BLINK_2Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_2Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_2Hz_IN_TIMER_TICKS;
          //STATUSLED_Invert;
        }
        break;
#endif

    case BLINK_1Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_1Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_1Hz_IN_TIMER_TICKS;
          //STATUSLED_Invert;
        }
        break;

    case BLINK_STAY_ON:
        //STATUSLED_High;
        break;

    case BLINK_USER_MANAGEMENT:
    default:
        TickBlinkStatusLED = 0;
        break;
  }
}