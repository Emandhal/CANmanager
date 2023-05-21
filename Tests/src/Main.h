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

//=== General pins definitions ===============================================
#define SYSLED_As_Output  ioport_set_pin_dir(LED0_GPIO, IOPORT_DIR_OUTPUT);
#define SYSLED_Low        ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
#define SYSLED_Invert     ioport_toggle_pin_level(LED0_GPIO);
#define SYSLED_High       ioport_set_pin_level(LED0_GPIO, LED0_ACTIVE_LEVEL);

#define STATUSLED_As_Output  ioport_set_pin_dir(LED1_GPIO, IOPORT_DIR_OUTPUT);
#define STATUSLED_Low        ioport_set_pin_level(LED1_GPIO, LED1_INACTIVE_LEVEL);
#define STATUSLED_Invert     ioport_toggle_pin_level(LED1_GPIO);
#define STATUSLED_High       ioport_set_pin_level(LED1_GPIO, LED1_ACTIVE_LEVEL);

#define USER_BUTTON1_As_Input  
#define USER_BUTTON1_GetState  (ioport_get_pin_level(GPIO_PUSH_BUTTON_1) ? 0 : 1) // Default state of button 1 will return '0'

#define USER_BUTTON2_As_Input  do                                                                         \
                               {                                                                          \
                                 ioport_set_pin_dir(GPIO_PUSH_BUTTON_2, IOPORT_DIR_INPUT);                \
                                 ioport_set_pin_mode(GPIO_PUSH_BUTTON_2, GPIO_PUSH_BUTTON_2_FLAGS);       \
                                 ioport_set_pin_sense_mode(GPIO_PUSH_BUTTON_2, GPIO_PUSH_BUTTON_2_SENSE); \
                               } while (0)
#define USER_BUTTON2_GetState  (ioport_get_pin_level(GPIO_PUSH_BUTTON_2) ? 0 : 1) // Default state of button 2 will return '0'

//-----------------------------------------------------------------------------
#endif /* MAIN_H_ */