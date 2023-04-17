/*
 * Main.h
 *
 * Created: 09/04/2020 15:57:04
 *  Author: Fabien
 */


#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "conf_board.h"
#include "Console.h"
#include "Interface/Console_V71Interface.h"





// System defines
#define SYSTEM_TICK_MS           ( 1u ) // 1ms for system tick
#define TIMER_TICKS_PER_SECONDS  ( 1000u / SYSTEM_TICK_MS )
extern volatile uint32_t msCount; //!< Milli-seconds count from start of the system


//! Button state
typedef enum
{
  PUSHED = 0,
  RELEASED,
} ButtonState;


#endif /* MAIN_H_ */