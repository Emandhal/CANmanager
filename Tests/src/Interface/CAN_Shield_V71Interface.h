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
#include "MCP251XFD.h"
#include "SJA1000.h"
#include "MCP251X.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP230XX and MCP23SXX ports expanders
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
// MCP2515 External CAN controller
//********************************************************************************************************************
extern CAN_BitTimeStats MCP2515_BitTimeStats; //!< MCP2515 Bit Time stat

extern struct MCP251X MCP2515_U5;             //!< Component structure of the MCP2515 as U5
extern struct MCP251X_Config MCP2515_U5_Conf; //!< Configuration structure of the MCP2515 as U5
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SJA1000 External CAN controller
//********************************************************************************************************************
extern CAN_BitTimeStats SJA1000_BitTimeStats; //!< SJA1000 Bit Time stat

extern struct SJA1000 SJA1000_U6;             //!< Component structure of the SJA1000 as U6 with link to GPIO expander on the CAN_Shield board
extern struct SJA1000_Config SJA1000_U6_Conf; //!< Configuration structure of the SJA1000 as U6
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP2518FD External CAN controller
//********************************************************************************************************************
extern struct MCP251XFD MCP2518FD_MB1;                                //!< Component structure of the MCP2518FD on MIKROBUS1 on the V71_XplainedUltra_CAN_Shield board
extern MCP251XFD_BitTimeStats MCP2518FD_BTStats;                      //!< MCP2518FD Bit Time stat
extern uint32_t MCP2518FD_SYSCLK;                                     //!< SYSCLK frequency will be stored here after using #Init_MCP251XFD()
extern struct MCP251XFD_Config MCP2518FD_Config;                      //!< Configuration structure of the MCP2518FD on MIKROBUS1

#define MCP2518FD_FIFO_COUNT    1
extern MCP251XFD_RAMInfos MCP2518FD_RAMInfos;                         //!< RAM informations will be stored here after using #MCP251XFD_ConfigureFIFOList()
extern MCP251XFD_FIFO MCP2518FD_FIFOlist[MCP2518FD_FIFO_COUNT];       //!< Configuration structure for FIFO of the MCP2518FD on MIKROBUS1

#define MCP2518FD_FILTER_COUNT  1
extern MCP251XFD_Filter MCP2518FD_FilterList[MCP2518FD_FILTER_COUNT]; //!< Configuration structure for Filters of the MCP2518FD on MIKROBUS1
//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP2517FD External CAN controller
//********************************************************************************************************************
extern struct MCP251XFD MCP2517FD_MB3;                                //!< Component structure of the MCP2517FD on MIKROBUS3 on the V71_XplainedUltra_CAN_Shield board
extern MCP251XFD_BitTimeStats MCP2517FD_BTStats;                      //!< MCP2517FD Bit Time stat
extern uint32_t MCP2517FD_SYSCLK;                                     //!< SYSCLK frequency will be stored here after using #Init_MCP251XFD()
extern struct MCP251XFD_Config MCP2517FD_Config;                      //!< Configuration structure of the MCP2517FD on MIKROBUS3

#define MCP2517FD_FIFO_COUNT    1
extern MCP251XFD_RAMInfos MCP2517FD_RAMInfos;                         //!< RAM informations will be stored here after using #MCP251XFD_ConfigureFIFOList()
extern MCP251XFD_FIFO MCP2517FD_FIFOlist[MCP2517FD_FIFO_COUNT];       //!< Configuration structure for FIFO of the MCP2517FD on MIKROBUS3

#define MCP2517FD_FILTER_COUNT  1
extern MCP251XFD_Filter MCP2517FD_FilterList[MCP2517FD_FILTER_COUNT]; //!< Configuration structure for Filters of the MCP2517FD on MIKROBUS3
//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CAN_SHIELD_V71INTERFACE_H_ */