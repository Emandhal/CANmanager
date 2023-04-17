/*!*****************************************************************************
 * @file    TimerTicks.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/02/2020
 * @brief   Timer ticks and System LED management
 ******************************************************************************/
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
#ifndef __TIMERTICKS_H_INCLUDED
#define __TIMERTICKS_H_INCLUDED
//=============================================================================

//------------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "Main.h"
#include "ErrorsDef.h"
//------------------------------------------------------------------------------
#ifdef __cplusplus
#include "stdafx.h"

#if !defined(LOCK)
#  define LOCK(name)    Locker_##name.Lock()
#  define UNLOCK(name)  Locker_##name.UnLock()
#endif
extern SIMULATION_Mutex Locker_Ticks;  //!< Mutex for the access of timer ticks
extern SIMULATION_Mutex Locker_SysLED; //!< Mutex for the access of System and Status LEDs state

extern "C" {
#else
#  define LOCK(name)
#  define UNLOCK(name)
#endif
//------------------------------------------------------------------------------

#define COUNT_1ms_IN_TIMER_TICKS    (  TIMER_TICKS_PER_SECONDS / 1000 )
#define COUNT_10ms_TIMER_TICKS      (  TIMER_TICKS_PER_SECONDS /  100 )
#define COUNT_20ms_IN_TIMER_TICKS   (  TIMER_TICKS_PER_SECONDS /   50 )
#define COUNT_100ms_IN_TIMER_TICKS  (  TIMER_TICKS_PER_SECONDS /   10 )
#define COUNT_200ms_IN_TIMER_TICKS  (  TIMER_TICKS_PER_SECONDS /    5 )
#define COUNT_250ms_IN_TIMER_TICKS  (  TIMER_TICKS_PER_SECONDS /    4 )
#define COUNT_500ms_IN_TIMER_TICKS  (  TIMER_TICKS_PER_SECONDS /    2 )
#define COUNT_1s_IN_TIMER_TICKS     (  TIMER_TICKS_PER_SECONDS /    1 )
#define COUNT_2s_IN_TIMER_TICKS     ( (TIMER_TICKS_PER_SECONDS /    1 ) * 2 )

//------------------------------------------------------------------------------

//! Calculate the timer refresh millisecond count
#if COUNT_1ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_1ms_IN_TIMER_TICKS )
#elif COUNT_10ms_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_10ms_TIMER_TICKS )
#elif COUNT_20ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_20ms_IN_TIMER_TICKS )
#elif COUNT_100ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_100ms_IN_TIMER_TICKS )
#elif COUNT_200ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_200ms_IN_TIMER_TICKS )
#elif COUNT_250ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_250ms_IN_TIMER_TICKS )
#elif COUNT_500ms_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_500ms_IN_TIMER_TICKS )
#elif COUNT_1s_IN_TIMER_TICKS != 0
#  define TICK_TO_ADD_FOR_MS  ( COUNT_1s_IN_TIMER_TICKS )
#else
#  define TICK_TO_ADD_FOR_MS  ( COUNT_2s_IN_TIMER_TICKS )
#endif

//------------------------------------------------------------------------------

#define COUNT_BLINK_10Hz_IN_TIMER_TICKS  ( TIMER_TICKS_PER_SECONDS / 10 ) //!< Invert the pin state of the system and status LEDs 10 times per second
#define COUNT_BLINK_2Hz_IN_TIMER_TICKS   ( TIMER_TICKS_PER_SECONDS /  2 ) //!< Invert the pin state of the system and status LEDs  2 times per second
#define COUNT_BLINK_1Hz_IN_TIMER_TICKS   ( TIMER_TICKS_PER_SECONDS /  1 ) //!< Invert the pin state of the system and status LEDs each second

//! Enumerate System and Status LEDs blink speed
typedef enum
{
  BLINK_OFF         = 0,
#if COUNT_BLINK_10Hz_IN_TIMER_TICKS != 0
  BLINK_10Hz        = 1,
#endif
#if COUNT_BLINK_2Hz_IN_TIMER_TICKS != 0
  BLINK_2Hz         = 2,
#endif
  BLINK_1Hz         = 3,
  BLINK_STAY_ON        ,
  BLINK_USER_MANAGEMENT, //!< Set this if the user will change its state in the program (No automatic management by TimerTicks)
} eLEDBlinkMode;

extern eLEDBlinkMode SystemLEDBlinkMode;
extern eLEDBlinkMode StatusLEDBlinkMode;

//------------------------------------------------------------------------------

//! @brief Timer ticks interface procedure type when there is no parameter
typedef void(*TTT_NoParam_Proc)(void);

//------------------------------------------------------------------------------

#if defined(PROCESS_WATCHDOG_IN_TIMER_TICKS)
//! BootloaderConfig object structure
typedef struct TimerTickWatchdogConfig
{
  //--- Watchdog configuration ---
  uint32_t WatchdogProcessFrequency;  //!< Processing frequency of the watchdog when a restart is called
  //--- Interface call functions ---
  TTT_NoParam_Proc fnWatchdogRestart; //!< This procedure will be called in the Sleep_ms() procedure
  TTT_NoParam_Proc fnWatchDogProcess; //!< This procedure will be called at a frequency defined by the variable 
} TimerTickWatchdogConfig;

extern TimerTickWatchdogConfig TimerTickWatchdog_Conf; //!< Timer ticks watchdog interface that needs to be declared and filled by the application
#endif

//------------------------------------------------------------------------------





//********************************************************************************************************************
// Timer ticks management
//********************************************************************************************************************
/*! @brief Initialize the timer ticks
 *
 * Initialize the timer ticks for 1ms to 2s.
 * It configures a timer to generate the main ticks from which derives ticks from 1ms to 2s
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TimerTicks_Init(void);

/*! @brief Get current millisecond
 *
 * The maximum time before exceeding the uint32 and crash the system is: 49 day, 17h, 2min, 47s and 295ms
 * @return The current millisecond
 */
uint32_t GetCurrentMs(void);

/*! @brief Sleep x millisecond
 *
 * @warning This function is not accurate, the range of sleep is: mSec < Sleep < mSec+1, and depend to the timer refresh
 * @param mSec Is the number of millisecond to wait
 */
void Sleep_ms(uint16_t mSec);


#if COUNT_1ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 1ms tick?
 *
 * @return If there is a 1ms tick pending
 */
bool ThereIs1msTick(void);
#endif

#if COUNT_10ms_TIMER_TICKS != 0
/*! @brief Is there a 10ms tick?
 *
 * @return If there is a 10ms tick pending
 */
bool ThereIs10msTick(void);
#endif

#if COUNT_20ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 20ms tick?
 *
 * @return If there is a 20ms tick pending
 */
bool ThereIs20msTick(void);
#endif

#if COUNT_100ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 100ms tick?
 *
 * @return If there is a 100ms tick pending
 */
bool ThereIs100msTick(void);
#endif

#if COUNT_200ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 200ms tick?
 *
 * @return If there is a 200ms tick pending
 */
bool ThereIs200msTick(void);
#endif

#if COUNT_250ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 250ms tick?
 *
 * @return If there is a 250ms tick pending
 */
bool ThereIs250msTick(void);
#endif

#if COUNT_500ms_IN_TIMER_TICKS != 0
/*! @brief Is there a 500ms tick?
 *
 * @return If there is a 500ms tick pending
 */
bool ThereIs500msTick(void);
#endif

#if COUNT_1s_IN_TIMER_TICKS != 0
/*! @brief Is there a 1s tick?
 *
 * @return If there is a 1s tick pending
 */
bool ThereIs1sTick(void);
#endif

#if COUNT_2s_IN_TIMER_TICKS != 0
/*! @brief Is there a 2s tick?
 *
 * @return If there is 2s tick pending
 */
bool ThereIs2sTick(void);
#endif

//------------------------------------------------------------------------------



//********************************************************************************************************************
// LEDs management
//********************************************************************************************************************
//! @brief Initialize the system and status LEDs management
void SystemAndStatusLED_Init(void);

/*! @brief Set system LED blink mode
 *
 * @param[in] blinkMode Is the new blink mode
 */
void SetSystemLEDblinkMode(eLEDBlinkMode blinkMode);

/*! @brief Refresh system LED state
 *
 * @param[in] ticks Is the count of ticks to add to the blink
 */
void RefreshSystemLEDstate(uint32_t ticks);

/*! @brief Set status LED blink mode
 *
 * @param[in] blinkMode Is the new blink mode
 */
void SetStatusLEDblinkMode(eLEDBlinkMode blinkMode);

/*! @brief Refresh status LED state
 *
 * @param[in] ticks Is the count of ticks to add to the blink
 */
void RefreshStatusLEDState(uint32_t ticks);

//------------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//------------------------------------------------------------------------------
#endif // __TIMERTICKS_H_INCLUDED
