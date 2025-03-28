/*!*****************************************************************************
 * @file    Main.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    04/06/2023
 * @brief   System configuration
 ******************************************************************************/
#ifndef MAIN_H_
#define MAIN_H_
//=============================================================================
#define USE_CONSOLE_TX
#define USE_CONSOLE_RX
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <asf.h> // Use Atmel Software Framework (ASF)
#include "conf_board.h"
//-----------------------------------------------------------------------------
#include "CAN_Shield_V71Interface.h"
#include "Ultra_V71Interfaces.h"
//-----------------------------------------------------------------------------





//===  System defines ===============================================
#define SYSTEM_TICK_MS           ( 1u ) //!< 1ms for system tick
#define TIMER_TICKS_PER_SECONDS  ( 1000u / SYSTEM_TICK_MS )
extern volatile uint32_t msCount;       //!< Milli-seconds count from start of the system
#define TIMESTAMP_TICK_us        ( 25 ) //!< TimeStamp tick is 25�s
#define TIMESTAMP_TICK(sysclk)   ( ((sysclk) / 1000000) * TIMESTAMP_TICK_us )

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

#define CAN_SHIELD_BITRATE    1000000 //!< Nominal Bitrate to 1Mbps
#define CANFD_SHIELD_BITRATE  2000000 //!< Data Bitrate to 2Mbps

#define MCAN_EID_AND_RANGE_MASK  (MCAN_FILTER_ACCEPT_ALL_MESSAGES)

//-----------------------------------------------------------------------------
#endif /* MAIN_H_ */