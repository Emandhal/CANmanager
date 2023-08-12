/*!*****************************************************************************
 * @file    CAN_Shield_V71Interface.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/07/2023
 * @brief   CAN Shield board interfaces on V71
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
#ifndef CAN_SHIELD_V71INTERFACE_H_
#define CAN_SHIELD_V71INTERFACE_H_
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include "GPIO_Interface.h"
#include "MCP230XX.h"
#include "MCP23SXX.h"
#include "SJA1000.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#define CAN_SHIELD_BITRATE  1000000

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// Component structure of the MCP230XX as U12 with hard I2C on the CAN_Shield board
extern struct MCP230XX MCP23008_U12;
//#define PORT_DEVICE0  &MCP23008_U12
extern struct MCP230XX_Config MCP23008_U12_Conf;

extern PORT_Interface PORTGP_U12; //!< PORT interface of the MCP230XX as U12 PORT GP on the CAN_Shield board
//-----------------------------------------------------------------------------
// Component structure of the MCP230XX as U8 with hard I2C on the CAN_Shield board
extern struct MCP230XX MCP23017_U8;
//#define PORT_DEVICE1  &MCP23017_U8
extern struct MCP230XX_Config MCP23017_U8_Conf;

extern PORT_Interface PORTGPA_U8; //!< PORT interface of the MCP230XX as U8 PORT GPA on the CAN_Shield board
extern PORT_Interface PORTGPB_U8; //!< PORT interface of the MCP230XX as U8 PORT GPB on the CAN_Shield board
//-----------------------------------------------------------------------------
// Component structure of the MCP23SXX as U9 with hard SPI on the CAN_Shield board
extern struct MCP23SXX MCP23S17_U9;
//#define PORT_DEVICE2  &MCP23S17_U9
extern struct MCP23SXX_Config MCP23S17_U9_Conf;

extern PORT_Interface PORTGPA_U9; //!< PORT interface of the MCP23SXX as U9 PORT GPA on the CAN_Shield board
extern PORT_Interface PORTGPB_U9; //!< PORT interface of the MCP23SXX as U9 PORT GPB on the CAN_Shield board

extern GPIO_Interface SJA1000_CS;       //!< GPIO interface of the MCP23SXX as SJA1000 CS pin on the CAN_Shield board
extern GPIO_Interface SJA1000_RD_E;     //!< GPIO interface of the MCP23SXX as SJA1000 RD#/E pin on the CAN_Shield board
extern GPIO_Interface SJA1000_WR;       //!< GPIO interface of the MCP23SXX as SJA1000 WR pin on the CAN_Shield board
extern GPIO_Interface SJA1000_ALE_AS;   //!< GPIO interface of the MCP23SXX as SJA1000 ALE/AS pin on the CAN_Shield board
extern GPIO_Interface MCP2515_Trans_CS; //!< GPIO interface of the MCP23SXX as MCP2515 Transceiver CS pin on the CAN_Shield board
extern GPIO_Interface SJA1000_Trans_CS; //!< GPIO interface of the MCP23SXX as SJA1000 Transceiver CS pin on the CAN_Shield board
//-----------------------------------------------------------------------------



//********************************************************************************************************************
extern CAN_BitTimeStats SJA1000_BitTimeStats; //!< SJA1000 Bit Time stat

extern struct SJA1000 SJA1000_U6;             //!< Component structure of the SJA1000 as U6 with link to GPIO expander on the CAN_Shield board
extern struct SJA1000_Config SJA1000_U6_Conf; //!< Configuration structure of the SJA1000 as U6
//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CAN_SHIELD_V71INTERFACE_H_ */