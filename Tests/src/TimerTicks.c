/*!*****************************************************************************
 * @file    TimerTicks.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/02/2020
 * @brief   Timer ticks and System LED management
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "TimerTicks.h"
#include "main.h"
//-----------------------------------------------------------------------------

#define UNIT_ERR_CONTEXT  ERRCONTEXT__TIMERTICKS // Error context of this unit

//------------------------------------------------------------------------------

#if defined(__cplusplus)
  using namespace std;
  using namespace std::chrono;
  SIMULATION_Mutex Locker_Ticks;  //! Mutex for the access of timer ticks
  SIMULATION_Mutex Locker_SysLED; //! Mutex for the access of System and Status LEDs state
#endif

#if !defined(LOGDEBUG)
#  define LOGDEBUG(...)
#endif

//-----------------------------------------------------------------------------

//--- Ticks variables ---
#if COUNT_1ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick1ms = 0;
#endif
#if COUNT_10ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick10ms = 0;
#endif
#if COUNT_20ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick20ms = 0;
#endif
#if COUNT_100ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick100ms = 0;
#endif
#if COUNT_200ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick200ms = 0;
#endif
#if COUNT_250ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick250ms = 0;
#endif
#if COUNT_500ms_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick500ms = 0;
#endif
#if COUNT_1s_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick1s = 0;
#endif
#if COUNT_2s_IN_TIMER_TICKS != 0
  static volatile uint32_t Tick2s = 0;
#endif

//--- Counting variables ---
static volatile uint32_t msTickCount = 0;

//--- Watchdog variables ---
#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
  static uint32_t TickWatchdog = 0;
  static uint32_t RefreshWatchdogTickCount = 0;
#endif

//--- SYSLED and STATUSLED variables ---
#if defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)
  static volatile uint32_t TickBlinkSystemLED = 0;
  static eLEDBlinkMode SystemLEDBlinkMode = BLINK_OFF; // Typically LED0
#endif
#if defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High)
  static volatile uint32_t TickBlinkStatusLED = 0;
  static eLEDBlinkMode StatusLEDBlinkMode = BLINK_OFF; // Typically LED1
#endif

//--- User Buttons variables ---
#if defined(USER_BUTTON1_GetState)
  static uint32_t UserButton1_DebounceTicksInt  = 0;
  static uint32_t UserButton1_DebounceTicksMask = 0;
  static ButtonState UserButton1_LastState   = BUTTON_RELEASED;
  static ButtonEvent UserButton1_EventStatus = BUTTON_PUSHED_EVENT;
#endif
#if defined(USER_BUTTON2_GetState)
  static uint32_t UserButton2_DebounceTicksInt  = 0;
  static uint32_t UserButton2_DebounceTicksMask = 0;
  static ButtonState UserButton2_LastState   = BUTTON_RELEASED;
  static ButtonEvent UserButton2_EventStatus = BUTTON_PUSHED_EVENT;
#endif

//-----------------------------------------------------------------------------
#define TIMERTICKS_TIME_DIFF(begin,end)  ( ((end) >= (begin)) ? ((end) - (begin)) : (UINT32_MAX - ((begin) - (end) - 1)) ) // Works only if time difference is strictly inferior to (UINT32_MAX/2) and call often
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// Timer ticks management
//********************************************************************************************************************
//=============================================================================
// Initialize the timer ticks
//=============================================================================
eERRORRESULT TimerTicks_Init(void)
{
  //--- Init variables ---
#if COUNT_1ms_IN_TIMER_TICKS != 0
  Tick1ms     = 0;
#endif
#if COUNT_10ms_IN_TIMER_TICKS != 0
  Tick10ms    = 0;
#endif
#if COUNT_20ms_IN_TIMER_TICKS != 0
  Tick20ms    = 0;
#endif
#if COUNT_100ms_IN_TIMER_TICKS != 0
  Tick100ms   = 0;
#endif
#if COUNT_200ms_IN_TIMER_TICKS != 0
  Tick200ms   = 0;
#endif
#if COUNT_250ms_IN_TIMER_TICKS != 0
  Tick250ms   = 0;
#endif
#if COUNT_500ms_IN_TIMER_TICKS != 0
  Tick500ms   = 0;
#endif
#if COUNT_1s_IN_TIMER_TICKS != 0
  Tick1s      = 0;
#endif
#if COUNT_2s_IN_TIMER_TICKS != 0
  Tick2s      = 0;
#endif
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


//=============================================================================
// [INTERRUPT] Process in SystemTick interrupt
//=============================================================================
void __attribute__ ((optimize("Ofast"))) TimerTicks_ProcessIT(void)
{
#if defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)
  RefreshSystemLEDstate(TICK_TO_ADD_FOR_MS);
#endif
#if defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High)
  RefreshStatusLEDState(TICK_TO_ADD_FOR_MS);
#endif

#if defined(USER_BUTTON1_GetState)
  //--- Integrate the User Button 1 read ---
  UserButton1_DebounceTicksInt <<= 1;
  if (USER_BUTTON1_GetState > 0) UserButton1_DebounceTicksInt |= 1;
  UserButton1_DebounceTicksInt &= UserButton1_DebounceTicksMask;
  //--- Get new status of the User Button 1 ---
  ButtonState UserButton1_NewState = BUTTON_UNDEF;
  if (UserButton1_DebounceTicksInt == UserButton1_DebounceTicksMask) UserButton1_NewState = BUTTON_PUSHED;
  if (UserButton1_DebounceTicksInt ==                             0) UserButton1_NewState = BUTTON_RELEASED;
  //--- Test User Button 1 status ---
  if ((UserButton1_NewState != BUTTON_UNDEF) && (UserButton1_NewState != UserButton1_LastState))
  {
    UserButton1_LastState = UserButton1_NewState;
    if ((UserButton1_NewState == BUTTON_PUSHED) && (UserButton1_EventStatus != BUTTON_RELEASED_EVENT)) UserButton1_Event(BUTTON_PUSHED_EVENT);
    if ((UserButton1_NewState == BUTTON_RELEASED) && (UserButton1_EventStatus != BUTTON_PUSHED_EVENT)) UserButton1_Event(BUTTON_RELEASED_EVENT);
  }
#endif
#if defined(USER_BUTTON2_GetState)
  //--- Integrate the User Button 2 read ---
  UserButton2_DebounceTicksInt <<= 1;
  if (USER_BUTTON2_GetState > 0) UserButton2_DebounceTicksInt |= 1;
  UserButton2_DebounceTicksInt &= UserButton2_DebounceTicksMask;
  //--- Get new status of the User Button 2 ---
  ButtonState UserButton2_NewState = BUTTON_UNDEF;
  if (UserButton2_DebounceTicksInt ==                             0) UserButton2_NewState = BUTTON_RELEASED;
  if (UserButton2_DebounceTicksInt == UserButton2_DebounceTicksMask) UserButton2_NewState = BUTTON_PUSHED;
  //--- Test User Button 2 status ---
  if ((UserButton2_NewState != BUTTON_UNDEF) && (UserButton2_NewState != UserButton1_LastState))
  {
    UserButton2_LastState = UserButton2_NewState;
    if ((UserButton2_NewState == BUTTON_PUSHED) && (UserButton2_EventStatus != BUTTON_RELEASED_EVENT)) UserButton2_Event(BUTTON_PUSHED_EVENT);
    if ((UserButton2_NewState == BUTTON_RELEASED) && (UserButton2_EventStatus != BUTTON_PUSHED_EVENT)) UserButton2_Event(BUTTON_RELEASED_EVENT);
  }
#endif

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
#if COUNT_10ms_IN_TIMER_TICKS != 0
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


#if COUNT_10ms_IN_TIMER_TICKS != 0
//=============================================================================
// Is there a 100Hz (10ms) tick?
//=============================================================================
bool ThereIs10msTick(void)
{
  if (Tick10ms >= COUNT_10ms_IN_TIMER_TICKS)
  {
    LOCK(Ticks);
    Tick10ms -= COUNT_10ms_IN_TIMER_TICKS;
    UNLOCK(Ticks);
#if (defined(DEBUG) || defined(_DEBUG))
    if (Tick10ms >= COUNT_10ms_IN_TIMER_TICKS)
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
// Wait x millisecond
//=============================================================================
void Wait_ms(uint16_t mSec)
{ // This procedure is not accurate, the range of wait is: mSec < Sleep < mSec+1, and depend to the timer refresh
  uint32_t StartTime = msTickCount;
  while (TIMERTICKS_TIME_DIFF(StartTime, msTickCount) < mSec)
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
#if (defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)) || (defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High))
//=============================================================================
// Initialize the system and status LEDs management
//=============================================================================
void SystemAndStatusLED_Init(void)
{
#if defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)
  //--- Set SYSLED initial blink mode ---
  SYSLED_As_Output;
  SetSystemLEDblinkMode(BLINK_1Hz);
#endif

#if defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High)
  //--- Set STATUSLED initial blink mode ---
  STATUSLED_As_Output;
  SetStatusLEDblinkMode(BLINK_USER_MANAGEMENT);
#endif
}
#endif

//-----------------------------------------------------------------------------



#if defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)
//=============================================================================
// Set system LED blink mode
//=============================================================================
void SetSystemLEDblinkMode(eLEDBlinkMode blinkMode)
{
  if (blinkMode != SystemLEDBlinkMode)
  {
    LOCK(SysLED);
    SystemLEDBlinkMode = blinkMode;
    if (blinkMode == BLINK_USER_MANAGEMENT) SYSLED_Low; // Force OFF at initialization
    UNLOCK(SysLED);
    TickBlinkSystemLED = 0;
  }
}


//=============================================================================
// Refresh system LED state
//=============================================================================
void __attribute__ ((optimize("Ofast"))) RefreshSystemLEDstate(uint32_t ticks)
{
  TickBlinkSystemLED += ticks;

  switch (SystemLEDBlinkMode)
  {
    case BLINK_OFF:
        SYSLED_Low;
        break;

#if COUNT_BLINK_10Hz_IN_TIMER_TICKS != 0
    case BLINK_10Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_10Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_10Hz_IN_TIMER_TICKS;
          SYSLED_Invert;
      }
      break;
#endif

#if COUNT_BLINK_2Hz_IN_TIMER_TICKS != 0
    case BLINK_2Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_2Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_2Hz_IN_TIMER_TICKS;
          SYSLED_Invert;
        }
        break;
#endif

    case BLINK_1Hz:
        if (TickBlinkSystemLED >= COUNT_BLINK_1Hz_IN_TIMER_TICKS)
        {
          TickBlinkSystemLED -= COUNT_BLINK_1Hz_IN_TIMER_TICKS;
          SYSLED_Invert;
        }
        break;

    case BLINK_STAY_ON:
        SYSLED_High;
        break;

    case BLINK_USER_MANAGEMENT:
    default:
        TickBlinkSystemLED = 0;
        break;
  }
}
#endif // #if defined(SYSLED_Low) && defined(SYSLED_Invert) && defined(SYSLED_High)

//-----------------------------------------------------------------------------



#if defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High)
//=============================================================================
// Set status LED blink mode
//=============================================================================
void SetStatusLEDblinkMode(eLEDBlinkMode blinkMode)
{
  if (blinkMode != StatusLEDBlinkMode)
  {
    LOCK(SysLED);
    StatusLEDBlinkMode = blinkMode;
    if (blinkMode == BLINK_USER_MANAGEMENT) STATUSLED_Low; // Force OFF at initialization
    UNLOCK(SysLED);
    TickBlinkStatusLED = TickBlinkSystemLED; // For synchronization with the system LED
  }
}


//=============================================================================
// Refresh status LED state
//=============================================================================
void __attribute__ ((optimize("Ofast"))) RefreshStatusLEDState(uint32_t ticks)
{
  TickBlinkStatusLED += ticks;

  switch (StatusLEDBlinkMode)
  {
    case BLINK_OFF:
        STATUSLED_Low;
        break;

#if COUNT_BLINK_10Hz_IN_TIMER_TICKS != 0
    case BLINK_10Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_10Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_10Hz_IN_TIMER_TICKS;
          STATUSLED_Invert;
        }
        break;
#endif

#if COUNT_BLINK_2Hz_IN_TIMER_TICKS != 0
    case BLINK_2Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_2Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_2Hz_IN_TIMER_TICKS;
          STATUSLED_Invert;
        }
        break;
#endif

    case BLINK_1Hz:
        if (TickBlinkStatusLED >= COUNT_BLINK_1Hz_IN_TIMER_TICKS)
        {
          TickBlinkStatusLED -= COUNT_BLINK_1Hz_IN_TIMER_TICKS;
          STATUSLED_Invert;
        }
        break;

    case BLINK_STAY_ON:
        STATUSLED_High;
        break;

    case BLINK_USER_MANAGEMENT:
    default:
        TickBlinkStatusLED = 0;
        break;
  }
}
#endif // #if defined(STATUSLED_Low) && defined(STATUSLED_Invert) && defined(STATUSLED_High)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// User Buttons management
//********************************************************************************************************************
#if defined(USER_BUTTON1_GetState)
//=============================================================================
// Initialize the User Button 1
//=============================================================================
void UserButton1_Init(uint32_t debounceTicksCount, ButtonEvent buttonEvent)
{
  //--- Set User Button 1 configuration ---
  USER_BUTTON1_As_Input;
  UserButton1_DebounceTicksInt  = 0;
  UserButton1_DebounceTicksMask = (uint32_t)(((uint64_t)1u << debounceTicksCount) - 1);
  UserButton1_LastState         = (USER_BUTTON1_GetState ? BUTTON_PUSHED : BUTTON_RELEASED);
  UserButton1_EventStatus       = buttonEvent;
}

//=============================================================================
// [WEAK] This function is called when the User Button 1 get the event expected
//=============================================================================
void UserButton1_Event(ButtonEvent buttonEvent)
{
    UNUSED(buttonEvent);
    // Do nothing special
    // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
}
#endif


#if defined(USER_BUTTON2_GetState)
//=============================================================================
// Initialize the User Button 2
//=============================================================================
void UserButton2_Init(uint32_t debounceTicksCount, ButtonEvent buttonEvent)
{
  //--- Set User Button 2 configuration ---
  USER_BUTTON2_As_Input;
  UserButton2_DebounceTicksInt  = 0;
  UserButton2_DebounceTicksMask = ((1 << debounceTicksCount) - 1);
  UserButton2_LastState         = (USER_BUTTON2_GetState ? BUTTON_PUSHED : BUTTON_RELEASED);
  UserButton2_EventStatus       = buttonEvent;
}

//=============================================================================
// [WEAK] This function is called when the User Button 2 get the event expected
//=============================================================================
void UserButton2_Event(ButtonEvent buttonEvent)
{
    UNUSED(buttonEvent);
    // Do nothing special
    // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
}
#endif

//-----------------------------------------------------------------------------