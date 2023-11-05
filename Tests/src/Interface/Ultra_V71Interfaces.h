/*!*****************************************************************************
 * @file    Ultra_V71Interfaces.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    01/07/2023
 * @brief   V71 Xplained Ultra board interfaces
*******************************************************************************/
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

/* Revision history:
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef ULTRA_V71INTERFACE_H_
#define ULTRA_V71INTERFACE_H_
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "GPIO_Interface.h"
#ifdef USE_EEPROM_GENERICNESS
#  include "AT24MAC402.h"
#endif
#ifdef USE_V71_MCAN
#  include "MCAN_V71.h"
#endif
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SAMV71 MCAN0 peripheral
//********************************************************************************************************************
#ifdef USE_V71_MCAN
extern CAN_BitTimeStats MCAN0V71_BitTimeStats; //!< MCAN0V71 Bit Time stat

extern struct MCANV71 MCAN0V71;             //!< Component structure of the MCAN0 on SAMV71
extern struct MCAN_Config MCAN0V71_Conf; //!< Configuration structure of the MCAN0 on SAMV71
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SAMV71 MCAN1 peripheral
//********************************************************************************************************************
#ifdef USE_V71_MCAN
extern CAN_BitTimeStats MCAN1V71_BitTimeStats; //!< MCAN1V71 Bit Time stat
extern uint32_t MCAN1V71_SYSCLK;               //!< SYSCLK frequency will be stored here after using #Init_MCANV71()

extern struct MCANV71 MCAN1V71;          //!< Component structure of the MCAN1 on SAMV71
extern struct MCAN_Config MCAN1V71_Conf; //!< Configuration structure of the MCAN1 on SAMV71

#define MCAN1V71_OBJ_COUNT    6
extern CAN_RAMconfig MCAN1_FIFObuff_RAMInfos[MCAN1V71_OBJ_COUNT]; //!< FIFO/Buffer RAM informations array
extern MCAN_FIFObuff MCAN1_FIFObuffList[MCAN1V71_OBJ_COUNT];      //!< FIFO/Buffer configuration structure

#define MCAN1V71_FILTER_COUNT  (10 + 11)
extern MCAN_Filter MCAN1_FilterList[MCAN1V71_FILTER_COUNT]; //!< Filter configuration structure
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SAMV71 I2C peripheral
//********************************************************************************************************************
#ifdef USE_EEPROM_GENERICNESS
extern struct I2C_Interface I2C0_V71;
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SAMV71 SPI peripheral
//********************************************************************************************************************
extern struct SPI_Interface SPI0_V71;
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// AT24MAC402 Component
//********************************************************************************************************************
#ifdef USE_EEPROM_GENERICNESS
// Component structure of the AT24MAC402 with hard I2C on the V71
extern struct AT24MAC402 AT24MAC402_V71;
#define EEPROM_AT24MAC402  &AT24MAC402_V71
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SAN V71 GPIO Ports
//********************************************************************************************************************
extern PORT_Interface IOPORTA; //!< IOPORTA interface of the V71
extern PORT_Interface IOPORTB; //!< IOPORTB interface of the V71
extern PORT_Interface IOPORTC; //!< IOPORTC interface of the V71
extern PORT_Interface IOPORTD; //!< IOPORTD interface of the V71
extern PORT_Interface IOPORTE; //!< IOPORTE interface of the V71
//-----------------------------------------------------------------------------



//********************************************************************************************************************
/*! @brief Get millisecond
 *
 * This function will be called when the driver need to get current millisecond
 */
uint32_t GetCurrentms_V71(void);

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// PORTs interfaces of V71
//********************************************************************************************************************

/*! @brief PORT set pins direction for V71
 *
 * This function will be called to change the direction of a whole PORT
 * @param[in] *pIntDev Is the PORT interface container structure used for the PORT set direction
 * @param[in] pinsDirection Set the PORT pin direction, if bit is '1' then the corresponding GPIO is input else it's output
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT PORTSetDirection_V71(PORT_Interface *pIntDev, const uint32_t pinsDirection);

/*! @brief PORT pins input level for V71
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to get input level of a whole PORT
 * @param[out] *pinsLevel Return the actual level of the PORT pins. If bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT PORTGetInputLevel_V71(PORT_Interface *pIntDev, uint32_t *pinsLevel);

/*! @brief PORT pins output level for V71
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to set output level of a whole PORT
 * @param[in] pinsLevel Set the PORT pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT PORTSetOutputLevel_V71(PORT_Interface *pIntDev, const uint32_t pinsLevel);

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// GPIO Interfaces of V71
//********************************************************************************************************************


/*! @brief GPIO set direction for V71
 *
 * This function will be called to change the direction of the GPIO
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO set direction
 * @param[in] pinState Set the GPIO state following #eGPIO_State enumerator
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT GPIOSetState_V71(GPIO_Interface *pIntDev, const eGPIO_State pinState);

/*! @brief GPIO pin input level for V71
 *
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO get input level
 * @param[out] *pinLevel Return the actual level of the I/O pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT GPIOGetInputLevel_V71(GPIO_Interface *pIntDev, eGPIO_State *pinLevel);

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* ULTRA_V71INTERFACE_H_ */