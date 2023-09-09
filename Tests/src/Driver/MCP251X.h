/*!*****************************************************************************
 * @file    MCP251X.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    06/08/2023
 * @brief   MCP2510/MCP2515 driver
 * @details Stand-Alone CAN Controller with SPI Interface
 * Follow datasheet MCP2510 Rev.F (Jan 2007)
 *                  MCP2515 Rev.K (Apr 2021)
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

/* Revision history:
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef MCP251X_H_INC
#define MCP251X_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include "CAN_common.h"
#include "ErrorsDef.h"
#include "SPI_Interface.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#  define __MCP251X_PACKED__
#  define MCP251X_PACKITEM             __pragma(pack(push, 1))
#  define MCP251X_UNPACKITEM           __pragma(pack(pop))
#  define MCP251X_PACKENUM(name,type)  typedef enum name : type
#  define MCP251X_UNPACKENUM(name)     name
#else
#  define __MCP251X_PACKED__           __attribute__((packed))
#  define MCP251X_PACKITEM
#  define MCP251X_UNPACKITEM
#  define MCP251X_PACKENUM(name,type)  typedef enum __MCP251X_PACKED__
#  define MCP251X_UNPACKENUM(name)     name
#endif

//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define MCP251X_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP251X limits definitions
//********************************************************************************************************************

// Frequencies and bitrate limits for MCP251X
#define MCP2510_FOSCFREQ_3V0_5V5_MAX   ( 16000000u ) //!< Max Fosc frequency for MCP2510 (Vdd = 3.0V to 5.5V)
#define MCP2510_FOSCFREQ_4V5_5V5_MAX   ( 25000000u ) //!< Max Fosc frequency for MCP2510 (Vdd = 4.5V to 5.5V)
#define MCP2515_FOSCFREQ_2V7_5V5_MAX   ( 25000000u ) //!< Max Fosc frequency for MCP2515 (Vdd = 2.7V to 5.5V)
#define MCP2515_FOSCFREQ_4V5_5V5_MAX   ( 40000000u ) //!< Max Fosc frequency for MCP2515 (Vdd = 4.5V to 5.5V)
#define MCP251X_PERIPHERAL_CLK_MAX     MCP2515_FOSCFREQ_4V5_5V5_MAX //!< Max Peripheral frequency
#define MCP251X_CLKOUT_MAX             ( 25000000u ) //!< Max CLKOUT frequency
#define MCP2510_SPICLOCK_3V0_4V5_MAX   (  2500000u ) //!< Max SPI clock frequency for MCP2510 (Vdd = 3.0V to 4.5V)
#define MCP2510E_SPICLOCK_4V5_5V5_MAX  (  4000000u ) //!< Max SPI clock frequency for MCP2510 (Vdd = 4.5V to 5.5V, Etemp)
#define MCP2510_SPICLOCK_4V5_5V5_MAX   (  5000000u ) //!< Max SPI clock frequency for MCP2510 (Vdd = 4.5V to 5.5V)
#define MCP2515_SPICLOCK_MAX           ( 10000000u ) //!< Max SPI clock frequency for MCP2515 (Vdd = 2.7V to 5.5V)

#define MCP251X_NOMBITRATE_MIN         (        0u ) //!< Min Nominal bitrate
#define MCP251X_NOMBITRATE_MAX         (  1000000u ) //!< Max Nominal bitrate

#define MCP251X_SYSCLOCK_DIV           ( 1 ) //!< No specific divider of system clock inside the MCP251X

//-----------------------------------------------------------------------------

// Limits Bit Rate configuration range for MCP251X
#define MCP251X_tTXDtRXD_MAX  ( 255 ) //!< tTXD-RXD is the propagation delay of the transceiver, a maximum 255ns according to ISO 11898-1:2015
#define MCP251X_tBUS_CONV     (   5 ) //!< TBUS is the delay on the CAN bus, which is approximately 5ns/m

#define MCP251X_NBRP_MIN      (  1 ) //!< Min NBRP
#define MCP251X_NBRP_MAX      ( 64 ) //!< Max NBRP
#define MCP251X_NSYNC         (  1 ) //!< NSYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define MCP251X_NPRSEG_MIN    (  1 ) //!< Min PRSEG
#define MCP251X_NPRSEG_MAX    (  8 ) //!< Max PRSEG
#define MCP251X_NTSEG1_MIN    (  1 ) //!< Min NTSEG1
#define MCP251X_NTSEG1_MAX    (  8 ) //!< Max NTSEG1
#define MCP251X_NTSEG2_MIN    (  2 ) //!< Min NTSEG2
#define MCP251X_NTSEG2_MAX    (  8 ) //!< Max NTSEG2
#define MCP251X_NSJW_MIN      (  1 ) //!< Min NSJW
#define MCP251X_NSJW_MAX      (  4 ) //!< Max NSJW
#define MCP251X_NTQBIT_MIN    ( MCP251X_NSYNC + MCP251X_PRSEG_MIN + MCP251X_NTSEG1_MIN + MCP251X_NTSEG2_MIN ) //!< Min NTQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define MCP251X_NTQBIT_MAX    ( MCP251X_NSYNC + MCP251X_PRSEG_MAX + MCP251X_NTSEG1_MAX + MCP251X_NTSEG2_MAX ) //!< Max NTQ per Bit (25-bits)

#define MCP251X_MAX_OSC_TOLERANCE  ( 1.58f ) //!< The CAN specification indicates that the worst case oscillator tolerance is 1.58% and is only suitable for low bit rates (125kb/s or less)
#define MCP251X_UINT_MAX_OSC_TOL   ( (int32_t)(MCP251X_MAX_OSC_TOLERANCE * 100.0f) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X's SPI driver definitions
//********************************************************************************************************************

// SPI commands instructions
#define MCP251X_SPI_INSTRUCTION_RESET           ( 0b11000000 ) //!< Reset instruction. Resets internal registers to the default state, sets Configuration mode
#define MCP251X_SPI_INSTRUCTION_READ            ( 0b00000011 ) //!< Read instruction. Reads data from the register beginning at selected address
#define MCP251X_SPI_INSTRUCTION_READ_RX_BUFFER  ( 0b10010000 ) /*!< [MCP2515 only] Read Rx buffer instruction. The format is 1001 0nm0.
                                                                *   When reading a receive buffer, reduces the overhead of a normal READ command by placing the Address Pointer at one of four locations, as indicated by 'n,m'.
                                                                *   @note The associated RX flag bit, RXnIF (CANINTF), will be cleared after bringing CS high
                                                                */
#define MCP251X_SPI_INSTRUCTION_WRITE           ( 0b00000010 ) //!< Write instruction. Writes data to the register beginning at the selected address
#define MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER  ( 0b01000000 ) /*!< [MCP2515 only] Load Tx Buffer instruction. The format is 0100 0abc.
                                                                *   When loading a transmit buffer, reduces the overhead of a normal WRITE command by placing the Address Pointer at one of six locations, as indicated by 'a,b,c'
                                                                */
#define MCP251X_SPI_INSTRUCTION_RTS             ( 0b10000000 ) /*!< Message Request-to-Send instruction. The format is 1000 0xyz.
                                                                *   Instructs controller to begin message transmission sequence for any of the transmit buffers. 'x' is Request-to-Send for TXB2, 'y' is Request-to-Send for TXB1, and 'z' is Request-to-Send for TXB0
                                                                */
#define MCP251X_SPI_INSTRUCTION_READ_STATUS     ( 0b10100000 ) //!< Read Status instruction. Quick polling command that reads several status bits for transmit and receive functions
#define MCP251X_SPI_INSTRUCTION_RX_STATUS       ( 0b10110000 ) //!< [MCP2515 only] Rx Status instruction. Quick polling command that indicates filter match and message type (standard, extended and/or remote) of received message
#define MCP251X_SPI_INSTRUCTION_BIT_MODIFY      ( 0b00000101 ) /*!< Bit Modify instruction. Allows the user to set or clear individual bits in a particular register
                                                                *   @note Not all registers can be bit modified with this command. Executing this command on registers that are not bit modifiable will force the mask to FFh. List of the registers that apply:
                                                                *   BFPCTRL (0x0C), TXRTSCTRL (0x0D), CANSTAT (0xXE), CANCTRL (0xXF), TEC (0x1C), REC (0x1D), CNF3 (0x28), CNF2 (0x29), CNF1 (0x2A), CANINTE (0x2B), CANINTF (0x2C), EFLG (0x2D), TXB0CTRL (0x30), TXB1CTRL (0x40), TXB2CTRL (0x50), RXB0CTRL (0x60), and RXB1CTRL (0x70)
                                                                */

//-----------------------------------------------------------------------------

//! Read Rx buffer instruction enumerator
typedef enum
{
  MCP251X_SPI_READ_START_RXB0SIDH = 0b00, //!< Receive Buffer 0, Start at RXB0SIDH (0x61)
  MCP251X_SPI_READ_START_RXB0D0   = 0b01, //!< Receive Buffer 0, Start at RXB0D0 (0x66)
  MCP251X_SPI_READ_START_RXB1SIDH = 0b10, //!< Receive Buffer 1, Start at RXB1SIDH (0x71)
  MCP251X_SPI_READ_START_RXB1D0   = 0b11, //!< Receive Buffer 1, Start at RXB1D0 (0x76)
} eMCP251X_ReadRxBuffAddr;

#define MCP251X_SPI_READ_RX_BUFF_START_Pos         1
#define MCP251X_SPI_READ_RX_BUFF_START_Mask        (0x3u << MCP251X_SPI_READ_RX_BUFF_START_Pos)
#define MCP251X_SPI_READ_RX_BUFF_START_SET(value)  (((uint8_t)(value) << MCP251X_SPI_READ_RX_BUFF_START_Pos) & MCP251X_SPI_READ_RX_BUFF_START_Mask) //!< Set Read Rx buffer Instruction Start Address

#define MCP251X_SPI_READ_RX_BUFF_RXB0SIDH     ( MCP251X_SPI_INSTRUCTION_READ_RX_BUFFER | MCP251X_SPI_READ_RX_BUFF_START_SET(MCP251X_SPI_READ_START_RXB0SIDH) ) //!< SPI intruction 0x90: Read Rx buffer 0, Start at RXB0SIDH (0x61)
#define MCP251X_SPI_READ_RX_BUFF_RXB0D0       ( MCP251X_SPI_INSTRUCTION_READ_RX_BUFFER | MCP251X_SPI_READ_RX_BUFF_START_SET(MCP251X_SPI_READ_START_RXB0D0  ) ) //!< SPI intruction 0x92: Read Rx buffer 0, Start at RXB0D0 (0x66)
#define MCP251X_SPI_READ_RX_BUFF_RXB1SIDH     ( MCP251X_SPI_INSTRUCTION_READ_RX_BUFFER | MCP251X_SPI_READ_RX_BUFF_START_SET(MCP251X_SPI_READ_START_RXB1SIDH) ) //!< SPI intruction 0x94: Read Rx buffer 1, Start at RXB1SIDH (0x71)
#define MCP251X_SPI_READ_RX_BUFF_RXB1D0       ( MCP251X_SPI_INSTRUCTION_READ_RX_BUFFER | MCP251X_SPI_READ_RX_BUFF_START_SET(MCP251X_SPI_READ_START_RXB1D0  ) ) //!< SPI intruction 0x96: Read Rx buffer 1, Start at RXB1D0 (0x76)
#define MCP251X_SPI_READ_RX_BUFF_RXBnSIDH(n)  ( MCP251X_SPI_READ_RX_BUFF_RXB0SIDH + ((n) << 2) ) //!< SPI intruction 0x90, 0x94: Read Rx buffer n, Start at RXBnSIDH

//-----------------------------------------------------------------------------

//! Load Tx buffer instruction enumerator
typedef enum
{
  MCP251X_SPI_LOAD_START_TXB0SIDH = 0b000, //!< Tx Buffer 0, Start at TXB0SIDH (0x31)
  MCP251X_SPI_LOAD_START_TXB0D0   = 0b001, //!< Tx Buffer 0, Start at TXB0D0 (0x36)
  MCP251X_SPI_LOAD_START_TXB1SIDH = 0b010, //!< Tx Buffer 1, Start at TXB1SIDH (0x41)
  MCP251X_SPI_LOAD_START_TXB1D0   = 0b011, //!< Tx Buffer 1, Start at TXB1D0 (0x46)
  MCP251X_SPI_LOAD_START_TXB2SIDH = 0b100, //!< Tx Buffer 2, Start at TXB2SIDH (0x51)
  MCP251X_SPI_LOAD_START_TXB2D0   = 0b101, //!< Tx Buffer 2, Start at TXB2D0 (0x56)
} eMCP251X_LoadRxBuffAddr;

#define MCP251X_SPI_LOAD_TX_BUFF_START_Pos         0
#define MCP251X_SPI_LOAD_TX_BUFF_START_Mask        (0x7u << MCP251X_SPI_LOAD_TX_BUFF_START_Pos)
#define MCP251X_SPI_LOAD_TX_BUFF_START_SET(value)  (((uint8_t)(value) << MCP251X_SPI_LOAD_TX_BUFF_START_Pos) & MCP251X_SPI_LOAD_TX_BUFF_START_Mask) //!< Set Load Tx buffer Instruction Start Address

#define MCP251X_SPI_LOAD_TX_BUFF_TXB0SIDH     ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB0SIDH) ) //!< SPI intruction 0x40: Load Tx buffer 0, Start at TXB0SIDH (0x31)
#define MCP251X_SPI_LOAD_TX_BUFF_TXB0D0       ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB0D0  ) ) //!< SPI intruction 0x41: Load Tx buffer 0, Start at TXB0D0 (0x36)
#define MCP251X_SPI_LOAD_TX_BUFF_TXB1SIDH     ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB1SIDH) ) //!< SPI intruction 0x42: Load Tx buffer 1, Start at TXB1SIDH (0x41)
#define MCP251X_SPI_LOAD_TX_BUFF_TXB1D0       ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB1D0  ) ) //!< SPI intruction 0x43: Load Tx buffer 1, Start at TXB1D0 (0x46)
#define MCP251X_SPI_LOAD_TX_BUFF_TXB2SIDH     ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB2SIDH) ) //!< SPI intruction 0x44: Load Tx buffer 2, Start at TXB2SIDH (0x51)
#define MCP251X_SPI_LOAD_TX_BUFF_TXB2D0       ( MCP251X_SPI_INSTRUCTION_LOAD_TX_BUFFER | MCP251X_SPI_LOAD_TX_BUFF_START_SET(MCP251X_SPI_LOAD_START_TXB2D0  ) ) //!< SPI intruction 0x45: Load Tx buffer 2, Start at TXB2D0 (0x56)
#define MCP251X_SPI_LOAD_TX_BUFF_TXBnSIDH(n)  ( MCP251X_SPI_LOAD_TX_BUFF_TXB0SIDH + ((n) << 1) ) //!< SPI intruction 0x40, 0x42, 0x44: Load Tx buffer n, Start at TXBnSIDH

//-----------------------------------------------------------------------------

#define MCP251X_SPI_RTS_TXB0      ( MCP251X_SPI_INSTRUCTION_RTS | 0x01 ) //!< SPI intruction 0x81: Request-to-Send TXB0
#define MCP251X_SPI_RTS_TXB1      ( MCP251X_SPI_INSTRUCTION_RTS | 0x02 ) //!< SPI intruction 0x82: Request-to-Send TXB1
#define MCP251X_SPI_RTS_TXB2      ( MCP251X_SPI_INSTRUCTION_RTS | 0x04 ) //!< SPI intruction 0x84: Request-to-Send TXB2
#define MCP251X_SPI_RTS_ALL_TXB   ( MCP251X_SPI_INSTRUCTION_RTS | 0x07 ) //!< SPI intruction 0x87: Request-to-Send all TXB
#define MCP251X_SPI_RTSx(txb)     ( MCP251X_SPI_INSTRUCTION_RTS | (1 << (txb)) ) //!< SPI intruction Request-to-Send by TXB index

//-----------------------------------------------------------------------------

//! Read Status instruction
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_SPI_ReadStatus
{
  uint8_t ReadStatus;
  struct
  {
    uint8_t RX0IF : 1; //!< 0 - Status of RX0IF (CANINTF[0])
    uint8_t RX1IF : 1; //!< 1 - Status of RX1IF (CANINTF[1])
    uint8_t TXREQ0: 1; //!< 2 - Status of TXREQ (TXB0CTRL[3])
    uint8_t TX0IF : 1; //!< 3 - Status of TX0IF (CANINTF[2])
    uint8_t TXREQ1: 1; //!< 4 - Status of TXREQ (TXB1CTRL[3])
    uint8_t TX1IF : 1; //!< 5 - Status of TX1IF (CANINTF[3])
    uint8_t TXREQ2: 1; //!< 6 - Status of TXREQ (TXB2CTRL[3])
    uint8_t TX2IF : 1; //!< 7 - Status of TX2IF (CANINTF[4])
  };
} MCP251X_SPI_ReadStatus;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_SPI_ReadStatus, 1);

#define MCP251X_SPI_READ_STATUS_RX0IF   (0x1u << 0) //!< Status of RX0IF (CANINTF[0])
#define MCP251X_SPI_READ_STATUS_RX1IF   (0x1u << 1) //!< Status of RX1IF (CANINTF[1])
#define MCP251X_SPI_READ_STATUS_TXREQ0  (0x1u << 2) //!< Status of TXREQ0 (TXB0CTRL[3])
#define MCP251X_SPI_READ_STATUS_TX0IF   (0x1u << 3) //!< Status of TX0IF (CANINTF[2])
#define MCP251X_SPI_READ_STATUS_TXREQ1  (0x1u << 4) //!< Status of TXREQ1 (TXB1CTRL[3])
#define MCP251X_SPI_READ_STATUS_TX1IF   (0x1u << 5) //!< Status of TX1IF (CANINTF[3])
#define MCP251X_SPI_READ_STATUS_TXREQ2  (0x1u << 6) //!< Status of TXREQ2 (TXB2CTRL[3])
#define MCP251X_SPI_READ_STATUS_TX2IF   (0x1u << 7) //!< Status of TX2IF (CANINTF[4])

#define MCP251X_INT_ALL_STATUS_EVENTS  ( MCP251X_SPI_READ_STATUS_RX0IF  | MCP251X_SPI_READ_STATUS_RX1IF | MCP251X_SPI_READ_STATUS_TXREQ0 | MCP251X_SPI_READ_STATUS_TX0IF \
                                       | MCP251X_SPI_READ_STATUS_TXREQ1 | MCP251X_SPI_READ_STATUS_TX1IF | MCP251X_SPI_READ_STATUS_TXREQ2 | MCP251X_SPI_READ_STATUS_TX2IF )

//! Status Events, can be OR'ed.
typedef enum
{
  MCP251X_NO_STATUS_EVENT                 = 0x00,                           //!< No status events
  MCP251X_RX_BUFFER0_FULL_STATUS_EVENT    = MCP251X_SPI_READ_STATUS_RX0IF,  //!< Receive Buffer 0 Full status event
  MCP251X_RX_BUFFER1_FULL_STATUS_EVENT    = MCP251X_SPI_READ_STATUS_RX1IF,  //!< Receive Buffer 1 Full status event
  MCP251X_TX_REQUEST_BUFFER0_STATUS_EVENT = MCP251X_SPI_READ_STATUS_TXREQ0, //!< Message Transmit Request Buffer 0 status event
  MCP251X_TX_BUFFER0_EMPTY_STATUS_EVENT   = MCP251X_SPI_READ_STATUS_TX0IF,  //!< Transmit Buffer 0 Empty status event
  MCP251X_TX_REQUEST_BUFFER1_STATUS_EVENT = MCP251X_SPI_READ_STATUS_TXREQ1, //!< Message Transmit Request Buffer 1 status event
  MCP251X_TX_BUFFER1_EMPTY_STATUS_EVENT   = MCP251X_SPI_READ_STATUS_TX1IF,  //!< Transmit Buffer 1 Empty status event
  MCP251X_TX_REQUEST_BUFFER2_STATUS_EVENT = MCP251X_SPI_READ_STATUS_TXREQ2, //!< Message Transmit Request Buffer 2 status event
  MCP251X_TX_BUFFER2_EMPTY_STATUS_EVENT   = MCP251X_SPI_READ_STATUS_TX2IF,  //!< Transmit Buffer 2 Empty status event

  MCP251X_INT_STATUS_EVENTS_FLAGS_MASK    = MCP251X_INT_ALL_STATUS_EVENTS,  //!< Interrupt events flags mask
} eMCP251X_StatusEvents;

typedef eMCP251X_StatusEvents setMCP251X_StatusEvents; //!< Set of device status (can be OR'ed)

//-----------------------------------------------------------------------------

//! Rx Status instruction
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_SPI_RxStatus
{
  uint8_t RxStatus;
  struct
  {
    uint8_t MATCH: 3; //!< 0-2 - Filter Match
    uint8_t TYPE : 2; //!< 3-4 - Message Type Received
    uint8_t      : 1; //!< 5
    uint8_t RXMSG: 2; //!< 6-7 - Received Message
  };
} MCP251X_SPI_RxStatus;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_SPI_RxStatus, 1);

//! Filter Match enumerator
typedef enum
{
  MCP251X_SPI_FILTER_MATCH_RXF0  = 0b000, //!< Filter Match RXF0
  MCP251X_SPI_FILTER_MATCH_RXF1  = 0b001, //!< Filter Match RXF1
  MCP251X_SPI_FILTER_MATCH_RXF2  = 0b010, //!< Filter Match RXF2
  MCP251X_SPI_FILTER_MATCH_RXF3  = 0b011, //!< Filter Match RXF3
  MCP251X_SPI_FILTER_MATCH_RXF4  = 0b100, //!< Filter Match RXF4
  MCP251X_SPI_FILTER_MATCH_RXF5  = 0b101, //!< Filter Match RXF5
  MCP251X_SPI_FILTER_MATCH_RXF0r = 0b110, //!< Filter Match RXF0 (rollover to RXB1)
  MCP251X_SPI_FILTER_MATCH_RXF1r = 0b111, //!< Filter Match RXF1 (rollover to RXB1)
} eMCP251X_SPIfilterMatch;

#define MCP251X_SPI_FILTER_MATCH_Pos         0
#define MCP251X_SPI_FILTER_MATCH_Mask        (0x7u << MCP251X_SPI_FILTER_MATCH_Pos)
#define MCP251X_SPI_FILTER_MATCH_GET(value)  (((uint8_t)(value) & MCP251X_SPI_FILTER_MATCH_Mask) >> MCP251X_SPI_FILTER_MATCH_Pos) //!< Get Filter Match

//! Message Type Received enumerator
typedef enum
{
  MCP251X_SPI_MSG_TYPE_STD_DATA = 0b00, //!< Standard data frame
  MCP251X_SPI_MSG_TYPE_STD_RTR  = 0b01, //!< Standard remote frame
  MCP251X_SPI_MSG_TYPE_EXT_DATA = 0b10, //!< Extended data frame
  MCP251X_SPI_MSG_TYPE_EXT_RTR  = 0b11, //!< Extended remote frame
} eMCP251X_MsgTypeReceived;

#define MCP251X_SPI_MSG_TYPE_Pos         3
#define MCP251X_SPI_MSG_TYPE_Mask        (0x3u << MCP251X_SPI_MSG_TYPE_Pos)
#define MCP251X_SPI_MSG_TYPE_GET(value)  (((uint8_t)(value) & MCP251X_SPI_MSG_TYPE_Mask) >> MCP251X_SPI_MSG_TYPE_Pos) //!< Get Message Type Received

//! Received Message enumerator
typedef enum
{
  MCP251X_SPI_NO_RX_MSG = 0b00, //!< No RX message
  MCP251X_SPI_MSG_RXB0  = 0b01, //!< Message in RXB0
  MCP251X_SPI_MSG_RXB1  = 0b10, //!< Message in RXB1
  MCP251X_SPI_BOTH_MSG  = 0b11, //!< Messages in both buffers
} eMCP251X_ReceivedMsg;

#define MCP251X_SPI_RX_MSG_Pos         6
#define MCP251X_SPI_RX_MSG_Mask        (0x3u << MCP251X_SPI_RX_MSG_Pos)
#define MCP251X_SPI_RX_MSG_GET(value)  (((uint8_t)(value) & MCP251X_SPI_RX_MSG_Mask) >> MCP251X_SPI_RX_MSG_Pos) //!< Get Message Type Received

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Tx, Rx Messages Objects
//********************************************************************************************************************

//! Data Length Size for the CAN message
typedef enum
{
  MCP251X_DLC_0BYTE   = 0b0000, //!< The DLC is 0 data byte
  MCP251X_DLC_1BYTE   = 0b0001, //!< The DLC is 1 data byte
  MCP251X_DLC_2BYTE   = 0b0010, //!< The DLC is 2 data bytes
  MCP251X_DLC_3BYTE   = 0b0011, //!< The DLC is 3 data bytes
  MCP251X_DLC_4BYTE   = 0b0100, //!< The DLC is 4 data bytes
  MCP251X_DLC_5BYTE   = 0b0101, //!< The DLC is 5 data bytes
  MCP251X_DLC_6BYTE   = 0b0110, //!< The DLC is 6 data bytes
  MCP251X_DLC_7BYTE   = 0b0111, //!< The DLC is 7 data bytes
  MCP251X_DLC_8BYTE   = 0b1000, //!< The DLC is 8 data bytes
  MCP251X_DLC_8BYTE1  = 0b1001, //!< The DLC is 12 data byte
  MCP251X_DLC_8BYTE2  = 0b1010, //!< The DLC is 16 data byte
  MCP251X_DLC_8BYTE3  = 0b1011, //!< The DLC is 20 data bytes
  MCP251X_DLC_8BYTE4  = 0b1100, //!< The DLC is 24 data bytes
  MCP251X_DLC_8BYTE5  = 0b1101, //!< The DLC is 32 data bytes
  MCP251X_DLC_8BYTE6  = 0b1110, //!< The DLC is 48 data bytes
  MCP251X_DLC_8BYTE7  = 0b1111, //!< The DLC is 64 data bytes
  MCP251X_DLC_COUNT,            // Keep last
  MCP251X_PAYLOAD_MIN = 8,
  MCP251X_PAYLOAD_MAX = 8,
} eMCP251X_DataLength;

static const uint8_t MCP251X_DLC_TO_VALUE[MCP251X_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8};

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Standard Identifier High (SIDH) [Address: 0x31, 0x41, 0x51]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_StdMsgIdHigh
{
  uint8_t SIDH;
  struct
  {
    uint8_t SID10_3: 8; //!< 0-7 - Standard Message Identifier SID10..3
  };
} MCP251X_CAN_StdMsgIdHigh;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_StdMsgIdHigh, 1);

#define MCP251X_CAN_SID_10_3_Pos         0
#define MCP251X_CAN_SID_10_3_Mask        (0xFFu << MCP251X_CAN_SID_10_3_Pos)
#define MCP251X_CAN_SID_10_3_SET(value)  (((uint8_t)(value) << MCP251X_CAN_SID_10_3_Pos) & MCP251X_CAN_SID_10_3_Mask) //!< Set SID 10..3
#define MCP251X_CAN_SID_10_3_GET(value)  (uint32_t)(((uint8_t)(value) & MCP251X_CAN_SID_10_3_Mask) >> MCP251X_CAN_SID_10_3_Pos) //!< Get SID 10..3

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Standard Identifier Low (SIDL) [Address: 0x32, 0x42, 0x52]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_StdMsgIdLow
{
  uint8_t SIDL;
  struct
  {
    uint8_t EID17_16: 2; //!< 0-1 - Extended Message Identifier EID17..16
    uint8_t         : 1; //!< 2
    uint8_t EXIDE   : 1; //!< 3   - Identifier Extension Flag: '1' = Message will transmit Extended Identifier ; '0' = Message will transmit Standard Identifier
    uint8_t SRR     : 1; //!< 4   - [Rx only] Standard Frame Remote Transmit Request (valid only if IDE bit = 0): '1' = Standard frame Remote Transmit Request received ; '0' = Standard data frame received
    uint8_t SID2_0  : 3; //!< 5-7 - Standard Message Identifier SID2..0
  };
} MCP251X_CAN_StdMsgIdLow;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_StdMsgIdLow, 1);

#define MCP251X_CAN_EID_17_16_Pos         0
#define MCP251X_CAN_EID_17_16_Mask        (0x3u << MCP251X_CAN_EID_17_16_Pos)
#define MCP251X_CAN_EID_17_16_SET(value)  (((uint8_t)(value) << MCP251X_CAN_EID_17_16_Pos) & MCP251X_CAN_EID_17_16_Mask) //!< Set EID 17..16
#define MCP251X_CAN_EID_17_16_GET(value)  (uint32_t)(((uint8_t)(value) & MCP251X_CAN_EID_17_16_Mask) >> MCP251X_CAN_EID_17_16_Pos) //!< Get EID 17..16
#define MCP251X_CAN_EXIDE_EXTENDED_ID     (0x1u << 3) //!< 29-bit extended identifier
#define MCP251X_CAN_EXIDE_STANDARD_ID     (0x0u << 3) //!< 11-bit standard identifier
#define MCP251X_CAN_SRR_REMOTE_FRAME      (0x1u << 4) //!< Standard frame Remote Transmit Request received
#define MCP251X_CAN_SRR_DATA_FRAME        (0x0u << 4) //!< Standard data frame received
#define MCP251X_CAN_SID_2_0_Pos           5
#define MCP251X_CAN_SID_2_0_Mask          (0x7u << MCP251X_CAN_SID_2_0_Pos)
#define MCP251X_CAN_SID_2_0_SET(value)    (((uint8_t)(value) << MCP251X_CAN_SID_2_0_Pos) & MCP251X_CAN_SID_2_0_Mask) //!< Set SID 2..0
#define MCP251X_CAN_SID_2_0_GET(value)    (uint32_t)(((uint8_t)(value) & MCP251X_CAN_SID_2_0_Mask) >> MCP251X_CAN_SID_2_0_Pos) //!< Get SID 2..0

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Extended Identifier High (EIDH) [Address: 0x33, 0x43, 0x53]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_ExtMsgIdHigh
{
  uint8_t EIDH;
  struct
  {
    uint8_t EID15_8: 8; //!< 0-7 - Extended Message Identifier EID15..8
  };
} MCP251X_CAN_ExtMsgIdHigh;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_ExtMsgIdHigh, 1);

#define MCP251X_CAN_EID_15_8_Pos         0
#define MCP251X_CAN_EID_15_8_Mask        (0xFFu << MCP251X_CAN_EID_15_8_Pos)
#define MCP251X_CAN_EID_15_8_SET(value)  (((uint8_t)(value) << MCP251X_CAN_EID_15_8_Pos) & MCP251X_CAN_EID_15_8_Mask) //!< Set EID 15..8
#define MCP251X_CAN_EID_15_8_GET(value)  (uint32_t)(((uint8_t)(value) & MCP251X_CAN_EID_15_8_Mask) >> MCP251X_CAN_EID_15_8_Pos) //!< Get EID 15..8

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Extended Identifier Low (EIDL) [Address: 0x34, 0x44, 0x54]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_ExtMsgIdLow
{
  uint8_t EIDL;
  struct
  {
    uint8_t EID7_0: 8; //!< 0-7 - Extended Message Identifier EID7..0
  };
} MCP251X_CAN_ExtMsgIdLow;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_ExtMsgIdLow, 1);

#define MCP251X_CAN_EID_7_0_Pos         0
#define MCP251X_CAN_EID_7_0_Mask        (0xFFu << MCP251X_CAN_EID_7_0_Pos)
#define MCP251X_CAN_EID_7_0_SET(value)  (((uint8_t)(value) << MCP251X_CAN_EID_7_0_Pos) & MCP251X_CAN_EID_7_0_Mask) //!< Set EID 7..0
#define MCP251X_CAN_EID_7_0_GET(value)  (uint32_t)(((uint8_t)(value) & MCP251X_CAN_EID_7_0_Mask) >> MCP251X_CAN_EID_7_0_Pos) //!< Get EID 7..0

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Data Length Code (DLC) [Address: 0x35, 0x45, 0x55]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_DataLengthCode
{
  uint8_t DLCR;
  struct
  {
    uint8_t DLC: 4; //!< 0-3 - Data Length Code
    uint8_t RB0: 1; //!< 4   - [Rx only] Reserved Bit 0 value
    uint8_t RB1: 1; //!< 5   - [Rx only] Reserved Bit 1 value
    uint8_t RTR: 1; //!< 6   - Remote Transmission Request. '1' = Transmitted message will be a remote transmit request ; '0' = Transmitted message will be a data frame
    uint8_t    : 1; //!< 7
  };
} MCP251X_CAN_DataLengthCode;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_DataLengthCode, 1);

#define MCP251X_CAN_DLC_Pos           0
#define MCP251X_CAN_DLC_Mask          (0xFu << MCP251X_CAN_DLC_Pos)
#define MCP251X_CAN_DLC_SET(value)    (((uint8_t)(value) << MCP251X_CAN_DLC_Pos) & MCP251X_CAN_DLC_Mask) //!< Set Data Length Code
#define MCP251X_CAN_DLC_GET(value)    (((uint8_t)(value) & MCP251X_CAN_DLC_Mask) >> MCP251X_CAN_DLC_Pos) //!< Get Data Length Code
#define MCP251X_CAN_R0_RECESSIVE      (0x1u << 4) //!< R0 bit is recessive '1'
#define MCP251X_CAN_R1_RECESSIVE      (0x1u << 5) //!< R1 bit is recessive '1'
#define MCP251X_CAN_RTR_REMOTE_FRAME  (0x1u << 6) //!< Transmit remote frame
#define MCP251X_CAN_RTR_DATA_FRAME    (0x0u << 6) //!< Transmit data frame

//-----------------------------------------------------------------------------

#define MCP251X_CAN_TXMSG_STDH  0
#define MCP251X_CAN_TXMSG_STDL  1
#define MCP251X_CAN_TXMSG_EIDH  2
#define MCP251X_CAN_TXMSG_EIDL  3
#define MCP251X_CAN_TXMSG_DLCR  4

//! Transmit Message Object
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_TxMessage
{
  uint8_t Bytes[5];
  struct
  {
    MCP251X_CAN_StdMsgIdHigh   STDH; //!< CAN Transmit Message Standard Identifier High (SIDH)
    MCP251X_CAN_StdMsgIdLow    STDL; //!< CAN Transmit Message Standard Identifier Low (SIDL)
    MCP251X_CAN_ExtMsgIdHigh   EIDH; //!< CAN Transmit Message Extended Identifier High (EIDH)
    MCP251X_CAN_ExtMsgIdLow    EIDL; //!< CAN Transmit Message Extended Identifier Low (EIDL)
    MCP251X_CAN_DataLengthCode DLCR; //!< CAN Transmit Message Data Length Code (DLC)
  };
} MCP251X_CAN_TxMessage;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_TxMessage, 5);

//-----------------------------------------------------------------------------

#define MCP251X_CAN_TX_MESSAGE_HEADER_SIZE  ( sizeof(MCP251X_CAN_TxMessage) )
#define MCP251X_CAN_TX_MESSAGE_SIZE_MAX     ( sizeof(MCP251X_CAN_TxMessage) + MCP251X_PAYLOAD_MAX )

//-----------------------------------------------------------------------------

#define MCP251X_CAN_RXMSG_STDH  0
#define MCP251X_CAN_RXMSG_STDL  1
#define MCP251X_CAN_RXMSG_EIDH  2
#define MCP251X_CAN_RXMSG_EIDL  3
#define MCP251X_CAN_RXMSG_DLCR  4

//! Received Message Object
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CAN_RxMessage
{
  uint8_t Bytes[5];
  struct
  {
    MCP251X_CAN_StdMsgIdHigh   STDH; //!< CAN Received Message Standard Identifier High (SIDH)
    MCP251X_CAN_StdMsgIdLow    STDL; //!< CAN Received Message Standard Identifier Low (SIDL)
    MCP251X_CAN_ExtMsgIdHigh   EIDH; //!< CAN Received Message Extended Identifier High (EIDH)
    MCP251X_CAN_ExtMsgIdLow    EIDL; //!< CAN Received Message Extended Identifier Low (EIDL)
    MCP251X_CAN_DataLengthCode DLCR; //!< CAN Received Message Data Length Code (DLC)
  };
} MCP251X_CAN_RxMessage;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CAN_RxMessage, 5);

//-----------------------------------------------------------------------------

#define MCP251X_CAN_RX_MESSAGE_HEADER_SIZE  ( sizeof(MCP251X_CAN_RxMessage) )
#define MCP251X_CAN_RX_MESSAGE_SIZE_MAX     ( sizeof(MCP251X_CAN_RxMessage) + MCP251X_PAYLOAD_MAX )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Filter Objects
//********************************************************************************************************************

//! CAN Filter Standard Identifier High (SIDH) [Address: 0x00, 0x04, 0x08, 0x10, 0x14, 0x18]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterSIDhigh
{
  uint8_t SIDH;
  struct
  {
    uint8_t SID10_3: 8; //!< 0-7 - Standard Message Identifier SID10..3
  };
} MCP251X_FilterSIDhigh;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterSIDhigh, 1);

#define MCP251X_FILTER_SID_10_3_Pos         0
#define MCP251X_FILTER_SID_10_3_Mask        (0xFFu << MCP251X_FILTER_SID_10_3_Pos)
#define MCP251X_FILTER_SID_10_3_SET(value)  (((uint8_t)(value) << MCP251X_FILTER_SID_10_3_Pos) & MCP251X_FILTER_SID_10_3_Mask) //!< Set SID 10..3

//-----------------------------------------------------------------------------

//! CAN Filter Standard Identifier Low (SIDL) [Address: 0x01, 0x05, 0x09, 0x11, 0x15, 0x19]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterSIDlow
{
  uint8_t SIDL;
  struct
  {
    uint8_t EID17_16: 2; //!< 0-1 - Extended Message Identifier EID17..16
    uint8_t         : 1; //!< 2
    uint8_t EXIDE   : 1; //!< 3   - [ID only] Identifier Extension Flag: '1' = Message will transmit Extended Identifier ; '0' = Message will transmit Standard Identifier
    uint8_t         : 1; //!< 4
    uint8_t SID2_0  : 3; //!< 5-7 - Standard Message Identifier SID2..0
  };
} MCP251X_FilterSIDlow;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterSIDlow, 1);

#define MCP251X_FILTER_EID_17_16_Pos         0
#define MCP251X_FILTER_EID_17_16_Mask        (0x3u << MCP251X_FILTER_EID_17_16_Pos)
#define MCP251X_FILTER_EID_17_16_SET(value)  (((uint8_t)(value) << MCP251X_FILTER_EID_17_16_Pos) & MCP251X_FILTER_EID_17_16_Mask) //!< Set EID 17..16
#define MCP251X_FILTER_EXIDE_EXTENDED_ID     (0x1u << 3) //!< 29-bit extended identifier
#define MCP251X_FILTER_EXIDE_STANDARD_ID     (0x0u << 3) //!< 11-bit standard identifier
#define MCP251X_FILTER_SID_2_0_Pos           5
#define MCP251X_FILTER_SID_2_0_Mask          (0x7u << MCP251X_FILTER_SID_2_0_Pos)
#define MCP251X_FILTER_SID_2_0_SET(value)    (((uint8_t)(value) << MCP251X_FILTER_SID_2_0_Pos) & MCP251X_FILTER_SID_2_0_Mask) //!< Set SID 2..0

//-----------------------------------------------------------------------------

//! CAN Filter Extended Identifier High (EIDH) [Address: 0x02, 0x06, 0x0A, 0x12, 0x16, 0x1A]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterEIDhigh
{
  uint8_t EIDH;
  struct
  {
    uint8_t EID15_8: 8; //!< 0-7 - Extended Message Identifier EID15..8
  };
} MCP251X_FilterEIDhigh;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterEIDhigh, 1);

#define MCP251X_FILTER_EID_15_8_Pos         0
#define MCP251X_FILTER_EID_15_8_Mask        (0xFFu << MCP251X_FILTER_EID_15_8_Pos)
#define MCP251X_FILTER_EID_15_8_SET(value)  (((uint8_t)(value) << MCP251X_FILTER_EID_15_8_Pos) & MCP251X_FILTER_EID_15_8_Mask) //!< Set EID 15..8

//-----------------------------------------------------------------------------

//! CAN Filter Extended Identifier Low (EIDL) [Address: 0x03, 0x07, 0x0B, 0x13, 0x17, 0x1B]
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterEIDlow
{
  uint8_t EIDL;
  struct
  {
    uint8_t EID7_0: 8; //!< 0-7 - Extended Message Identifier EID7..0
  };
} MCP251X_FilterEIDlow;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterEIDlow, 1);

#define MCP251X_FILTER_EID_7_0_Pos         0
#define MCP251X_FILTER_EID_7_0_Mask        (0xFFu << MCP251X_FILTER_EID_7_0_Pos)
#define MCP251X_FILTER_EID_7_0_SET(value)  (((uint8_t)(value) << MCP251X_FILTER_EID_7_0_Pos) & MCP251X_FILTER_EID_7_0_Mask) //!< Set EID 7..0

//-----------------------------------------------------------------------------

#define MCP251X_FILTER_ID_STDH  0
#define MCP251X_FILTER_ID_STDL  1
#define MCP251X_FILTER_ID_EIDH  2
#define MCP251X_FILTER_ID_EIDL  3

//! Filter ID Object
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterObj
{
  uint8_t Bytes[4];
  struct
  {
    MCP251X_FilterSIDhigh  STDH; //!< Filter Standard Identifier High (SIDH)
    MCP251X_FilterSIDlow   STDL; //!< Filter Standard Identifier Low (SIDL)
    MCP251X_FilterEIDhigh  EIDH; //!< Filter Extended Identifier High (EIDH)
    MCP251X_FilterEIDlow   EIDL; //!< Filter Extended Identifier Low (EIDL)
  };
} MCP251X_FilterObj;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterObj, 4);

//-----------------------------------------------------------------------------

#define MCP251X_CAN_FILTER_ID_SIZE_MAX  ( sizeof(MCP251X_FilterObj) )

//-----------------------------------------------------------------------------

//! CAN Filter Mask Standard Identifier High (SIDH) [Address: 0x20, 0x24]
typedef MCP251X_FilterSIDhigh MCP251X_FilterMaskSIDhigh;

//-----------------------------------------------------------------------------

//! CAN Filter Mask Standard Identifier Low (SIDL) [Address: 0x21, 0x25]
typedef MCP251X_FilterSIDlow MCP251X_FilterMaskSIDlow;

//-----------------------------------------------------------------------------

//! CAN Filter Mask Extended Identifier High (EIDH) [Address: 0x22, 0x26]
typedef MCP251X_FilterEIDhigh MCP251X_FilterMaskEIDhigh;

//-----------------------------------------------------------------------------

//! CAN Filter Mask Extended Identifier Low (EIDL) [Address: 0x23, 0x27]
typedef MCP251X_FilterEIDlow MCP251X_FilterMaskEIDlow;

//-----------------------------------------------------------------------------

#define MCP251X_FILTER_MASK_STDH  0
#define MCP251X_FILTER_MASK_STDL  1
#define MCP251X_FILTER_MASK_EIDH  2
#define MCP251X_FILTER_MASK_EIDL  3

//! Filter Mask Object
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_FilterMaskObj
{
  uint8_t Bytes[4];
  struct
  {
    MCP251X_FilterMaskSIDhigh STDH; //!< Filter Mask Standard Identifier High (SIDH)
    MCP251X_FilterMaskSIDlow  STDL; //!< Filter Mask Standard Identifier Low (SIDL)
    MCP251X_FilterMaskEIDhigh EIDH; //!< Filter Mask Extended Identifier High (EIDH)
    MCP251X_FilterMaskEIDlow  EIDL; //!< Filter Mask Extended Identifier Low (EIDL)
  };
} MCP251X_FilterMaskObj;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_FilterMaskObj, 4);

//-----------------------------------------------------------------------------

#define MCP251X_CAN_FILTER_MASK_SIZE_MAX  ( sizeof(MCP251X_FilterMaskObj) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Register list
//********************************************************************************************************************

//! MCP251X registers list
typedef enum
{
  RegMCP251X_RXF0SIDH  = 0b00000000, //!< (Offset: 0x00) Filter 0 Standard Identifier Register High
  RegMCP251X_RXF0SIDL  = 0b00000001, //!< (Offset: 0x01) Filter 0 Standard Identifier Register Low
  RegMCP251X_RXF0EID8  = 0b00000010, //!< (Offset: 0x02) Filter 0 Extended Identifier Register High
  RegMCP251X_RXF0EID0  = 0b00000011, //!< (Offset: 0x03) Filter 0 Extended Identifier Register Low
  RegMCP251X_RXF1SIDH  = 0b00000100, //!< (Offset: 0x04) Filter 1 Standard Identifier Register High
  RegMCP251X_RXF1SIDL  = 0b00000101, //!< (Offset: 0x05) Filter 1 Standard Identifier Register Low
  RegMCP251X_RXF1EID8  = 0b00000110, //!< (Offset: 0x06) Filter 1 Extended Identifier Register High
  RegMCP251X_RXF1EID0  = 0b00000111, //!< (Offset: 0x07) Filter 1 Extended Identifier Register Low
  RegMCP251X_RXF2SIDH  = 0b00001000, //!< (Offset: 0x08) Filter 2 Standard Identifier Register High
  RegMCP251X_RXF2SIDL  = 0b00001001, //!< (Offset: 0x09) Filter 2 Standard Identifier Register Low
  RegMCP251X_RXF2EID8  = 0b00001010, //!< (Offset: 0x0A) Filter 2 Extended Identifier Register High
  RegMCP251X_RXF2EID0  = 0b00001011, //!< (Offset: 0x0B) Filter 2 Extended Identifier Register Low
  RegMCP251X_BFPCTRL   = 0b00001100, //!< (Offset: 0x0C) RXnBF Pin Control and Status Register (can be bit modified)
  RegMCP251X_TXRTSCTRL = 0b00001101, //!< (Offset: 0x0D) TXnRTS Pin Control and Status Register (can be bit modified)
  RegMCP251X_CANSTAT0  = 0b00001110, //!< (Offset: 0x0E) CAN Status Register 0 (can be bit modified)
  RegMCP251X_CANCTRL0  = 0b00001111, //!< (Offset: 0x0F) CAN Control Register 0 (can be bit modified)

  RegMCP251X_RXF3SIDH  = 0b00010000, //!< (Offset: 0x10) Filter 3 Standard Identifier Register High
  RegMCP251X_RXF3SIDL  = 0b00010001, //!< (Offset: 0x11) Filter 3 Standard Identifier Register Low
  RegMCP251X_RXF3EID8  = 0b00010010, //!< (Offset: 0x12) Filter 3 Extended Identifier Register High
  RegMCP251X_RXF3EID0  = 0b00010011, //!< (Offset: 0x13) Filter 3 Extended Identifier Register Low
  RegMCP251X_RXF4SIDH  = 0b00010100, //!< (Offset: 0x14) Filter 4 Standard Identifier Register High
  RegMCP251X_RXF4SIDL  = 0b00010101, //!< (Offset: 0x15) Filter 4 Standard Identifier Register Low
  RegMCP251X_RXF4EID8  = 0b00010110, //!< (Offset: 0x16) Filter 4 Extended Identifier Register High
  RegMCP251X_RXF4EID0  = 0b00010111, //!< (Offset: 0x17) Filter 4 Extended Identifier Register Low
  RegMCP251X_RXF5SIDH  = 0b00011000, //!< (Offset: 0x18) Filter 5 Standard Identifier Register High
  RegMCP251X_RXF5SIDL  = 0b00011001, //!< (Offset: 0x19) Filter 5 Standard Identifier Register Low
  RegMCP251X_RXF5EID8  = 0b00011010, //!< (Offset: 0x1A) Filter 5 Extended Identifier Register High
  RegMCP251X_RXF5EID0  = 0b00011011, //!< (Offset: 0x1B) Filter 5 Extended Identifier Register Low
  RegMCP251X_TEC       = 0b00011100, //!< (Offset: 0x1C) Transmit Error Counter Register (can be bit modified)
  RegMCP251X_REC       = 0b00011101, //!< (Offset: 0x1D) Receive Error Counter Register (can be bit modified)
  RegMCP251X_CANSTAT1  = 0b00011110, //!< (Offset: 0x1E) CAN Status Register 1 (can be bit modified)
  RegMCP251X_CANCTRL1  = 0b00011111, //!< (Offset: 0x1F) CAN Control Register 1 (can be bit modified)

  RegMCP251X_RXM0SIDH  = 0b00100000, //!< (Offset: 0x20) Mask 0 Standard Identifier Register High
  RegMCP251X_RXM0SIDL  = 0b00100001, //!< (Offset: 0x21) Mask 0 Standard Identifier Register Low
  RegMCP251X_RXM0EID8  = 0b00100010, //!< (Offset: 0x22) Mask 0 Extended Identifier Register High
  RegMCP251X_RXM0EID0  = 0b00100011, //!< (Offset: 0x23) Mask 0 Extended Identifier Register Low
  RegMCP251X_RXM1SIDH  = 0b00100100, //!< (Offset: 0x24) Mask 1 Standard Identifier Register High
  RegMCP251X_RXM1SIDL  = 0b00100101, //!< (Offset: 0x25) Mask 1 Standard Identifier Register Low
  RegMCP251X_RXM1EID8  = 0b00100110, //!< (Offset: 0x26) Mask 1 Extended Identifier Register High
  RegMCP251X_RXM1EID0  = 0b00100111, //!< (Offset: 0x27) Mask 1 Extended Identifier Register Low
  RegMCP251X_CNF3      = 0b00101000, //!< (Offset: 0x28) Configuration Register 3 (can be bit modified)
  RegMCP251X_CNF2      = 0b00101001, //!< (Offset: 0x29) Configuration Register 2 (can be bit modified)
  RegMCP251X_CNF1      = 0b00101010, //!< (Offset: 0x2A) Configuration Register 1 (can be bit modified)
  RegMCP251X_CANINTE   = 0b00101011, //!< (Offset: 0x2B) CAN Interrupt Enable Register (can be bit modified)
  RegMCP251X_CANINTF   = 0b00101100, //!< (Offset: 0x2C) CAN Interrupt Flag Register (can be bit modified)
  RegMCP251X_EFLG      = 0b00101101, //!< (Offset: 0x2D) Error Flag Register (can be bit modified)
  RegMCP251X_CANSTAT2  = 0b00101110, //!< (Offset: 0x2E) CAN Status Register 2 (can be bit modified)
  RegMCP251X_CANCTRL2  = 0b00101111, //!< (Offset: 0x2F) CAN Control Register 2 (can be bit modified)

  RegMCP251X_TXB0CTRL  = 0b00110000, //!< (Offset: 0x30) Transmit Buffer 0 Control Register (can be bit modified)
  RegMCP251X_TXB0SIDH  = 0b00110001, //!< (Offset: 0x31) Transmit Buffer 0 Standard Identifier Register High
  RegMCP251X_TXB0SIDL  = 0b00110010, //!< (Offset: 0x32) Transmit Buffer 0 Standard Identifier Register Low
  RegMCP251X_TXB0EID8  = 0b00110011, //!< (Offset: 0x33) Transmit Buffer 0 Extended Identifier 8 Register High
  RegMCP251X_TXB0EID0  = 0b00110100, //!< (Offset: 0x34) Transmit Buffer 0 Extended Identifier 0 Register Low
  RegMCP251X_TXB0DLC   = 0b00110101, //!< (Offset: 0x35) Transmit Buffer 0 Data Length Code Register
  RegMCP251X_TXB0D0    = 0b00110110, //!< (Offset: 0x36) Transmit Buffer 0 Data Byte 0 Register
  RegMCP251X_TXB0D1    = 0b00110111, //!< (Offset: 0x37) Transmit Buffer 0 Data Byte 1 Register
  RegMCP251X_TXB0D2    = 0b00111000, //!< (Offset: 0x38) Transmit Buffer 0 Data Byte 2 Register
  RegMCP251X_TXB0D3    = 0b00111001, //!< (Offset: 0x39) Transmit Buffer 0 Data Byte 3 Register
  RegMCP251X_TXB0D4    = 0b00111010, //!< (Offset: 0x3A) Transmit Buffer 0 Data Byte 4 Register
  RegMCP251X_TXB0D5    = 0b00111011, //!< (Offset: 0x3B) Transmit Buffer 0 Data Byte 5 Register
  RegMCP251X_TXB0D6    = 0b00111100, //!< (Offset: 0x3C) Transmit Buffer 0 Data Byte 6 Register
  RegMCP251X_TXB0D7    = 0b00111101, //!< (Offset: 0x3D) Transmit Buffer 0 Data Byte 7 Register
  RegMCP251X_CANSTAT3  = 0b00111110, //!< (Offset: 0x3E) CAN Status Register 3 (can be bit modified)
  RegMCP251X_CANCTRL3  = 0b00111111, //!< (Offset: 0x3F) CAN Control Register 3 (can be bit modified)

  RegMCP251X_TXB1CTRL  = 0b01000000, //!< (Offset: 0x40) Transmit Buffer 1 Control Register (can be bit modified)
  RegMCP251X_TXB1SIDH  = 0b01000001, //!< (Offset: 0x41) Transmit Buffer 1 Standard Identifier Register High
  RegMCP251X_TXB1SIDL  = 0b01000010, //!< (Offset: 0x42) Transmit Buffer 1 Standard Identifier Register Low
  RegMCP251X_TXB1EID8  = 0b01000011, //!< (Offset: 0x43) Transmit Buffer 1 Extended Identifier 8 Register High
  RegMCP251X_TXB1EID0  = 0b01000100, //!< (Offset: 0x44) Transmit Buffer 1 Extended Identifier 0 Register Low
  RegMCP251X_TXB1DLC   = 0b01000101, //!< (Offset: 0x45) Transmit Buffer 1 Data Length Code Register
  RegMCP251X_TXB1D0    = 0b01000110, //!< (Offset: 0x46) Transmit Buffer 1 Data Byte 0 Register
  RegMCP251X_TXB1D1    = 0b01000111, //!< (Offset: 0x47) Transmit Buffer 1 Data Byte 1 Register
  RegMCP251X_TXB1D2    = 0b01001000, //!< (Offset: 0x48) Transmit Buffer 1 Data Byte 2 Register
  RegMCP251X_TXB1D3    = 0b01001001, //!< (Offset: 0x49) Transmit Buffer 1 Data Byte 3 Register
  RegMCP251X_TXB1D4    = 0b01001010, //!< (Offset: 0x4A) Transmit Buffer 1 Data Byte 4 Register
  RegMCP251X_TXB1D5    = 0b01001011, //!< (Offset: 0x4B) Transmit Buffer 1 Data Byte 5 Register
  RegMCP251X_TXB1D6    = 0b01001100, //!< (Offset: 0x4C) Transmit Buffer 1 Data Byte 6 Register
  RegMCP251X_TXB1D7    = 0b01001101, //!< (Offset: 0x4D) Transmit Buffer 1 Data Byte 7 Register
  RegMCP251X_CANSTAT4  = 0b01001110, //!< (Offset: 0x4E) CAN Status Register 4 (can be bit modified)
  RegMCP251X_CANCTRL4  = 0b01001111, //!< (Offset: 0x4F) CAN Control Register 4 (can be bit modified)

  RegMCP251X_TXB2CTRL  = 0b01010000, //!< (Offset: 0x50) Transmit Buffer 2 Control Register (can be bit modified)
  RegMCP251X_TXB2SIDH  = 0b01010001, //!< (Offset: 0x51) Transmit Buffer 2 Standard Identifier Register High
  RegMCP251X_TXB2SIDL  = 0b01010010, //!< (Offset: 0x52) Transmit Buffer 2 Standard Identifier Register Low
  RegMCP251X_TXB2EID8  = 0b01010011, //!< (Offset: 0x53) Transmit Buffer 2 Extended Identifier 8 Register High
  RegMCP251X_TXB2EID0  = 0b01010100, //!< (Offset: 0x54) Transmit Buffer 2 Extended Identifier 0 Register Low
  RegMCP251X_TXB2DLC   = 0b01010101, //!< (Offset: 0x55) Transmit Buffer 2 Data Length Code Register
  RegMCP251X_TXB2D0    = 0b01010110, //!< (Offset: 0x56) Transmit Buffer 2 Data Byte 0 Register
  RegMCP251X_TXB2D1    = 0b01010111, //!< (Offset: 0x57) Transmit Buffer 2 Data Byte 1 Register
  RegMCP251X_TXB2D2    = 0b01011000, //!< (Offset: 0x58) Transmit Buffer 2 Data Byte 2 Register
  RegMCP251X_TXB2D3    = 0b01011001, //!< (Offset: 0x59) Transmit Buffer 2 Data Byte 3 Register
  RegMCP251X_TXB2D4    = 0b01011010, //!< (Offset: 0x5A) Transmit Buffer 2 Data Byte 4 Register
  RegMCP251X_TXB2D5    = 0b01011011, //!< (Offset: 0x5B) Transmit Buffer 2 Data Byte 5 Register
  RegMCP251X_TXB2D6    = 0b01011100, //!< (Offset: 0x5C) Transmit Buffer 2 Data Byte 6 Register
  RegMCP251X_TXB2D7    = 0b01011101, //!< (Offset: 0x5D) Transmit Buffer 2 Data Byte 7 Register
  RegMCP251X_CANSTAT5  = 0b01011110, //!< (Offset: 0x5E) CAN Status Register 5 (can be bit modified)
  RegMCP251X_CANCTRL5  = 0b01011111, //!< (Offset: 0x5F) CAN Control Register 5 (can be bit modified)

  RegMCP251X_RXB0CTRL  = 0b01100000, //!< (Offset: 0x60) Receive Buffer 0 Control Register (can be bit modified)
  RegMCP251X_RXB0SIDH  = 0b01100001, //!< (Offset: 0x61) Receive Buffer 0 Standard Identifier Register High
  RegMCP251X_RXB0SIDL  = 0b01100010, //!< (Offset: 0x62) Receive Buffer 0 Standard Identifier Register Low
  RegMCP251X_RXB0EID8  = 0b01100011, //!< (Offset: 0x63) Receive Buffer 0 Extended Identifier 8 Register High
  RegMCP251X_RXB0EID0  = 0b01100100, //!< (Offset: 0x64) Receive Buffer 0 Extended Identifier 0 Register Low
  RegMCP251X_RXB0DLC   = 0b01100101, //!< (Offset: 0x65) Receive Buffer 0 Data Length Code Register
  RegMCP251X_RXB0D0    = 0b01100110, //!< (Offset: 0x66) Receive Buffer 0 Data Byte 0 Register
  RegMCP251X_RXB0D1    = 0b01100111, //!< (Offset: 0x67) Receive Buffer 0 Data Byte 1 Register
  RegMCP251X_RXB0D2    = 0b01101000, //!< (Offset: 0x68) Receive Buffer 0 Data Byte 2 Register
  RegMCP251X_RXB0D3    = 0b01101001, //!< (Offset: 0x69) Receive Buffer 0 Data Byte 3 Register
  RegMCP251X_RXB0D4    = 0b01101010, //!< (Offset: 0x6A) Receive Buffer 0 Data Byte 4 Register
  RegMCP251X_RXB0D5    = 0b01101011, //!< (Offset: 0x6B) Receive Buffer 0 Data Byte 5 Register
  RegMCP251X_RXB0D6    = 0b01101100, //!< (Offset: 0x6C) Receive Buffer 0 Data Byte 6 Register
  RegMCP251X_RXB0D7    = 0b01101101, //!< (Offset: 0x6D) Receive Buffer 0 Data Byte 7 Register
  RegMCP251X_CANSTAT6  = 0b01101110, //!< (Offset: 0x6E) CAN Status Register 6 (can be bit modified)
  RegMCP251X_CANCTRL6  = 0b01101111, //!< (Offset: 0x6F) CAN Control Register 6 (can be bit modified)

  RegMCP251X_RXB1CTRL  = 0b01110000, //!< (Offset: 0x70) Receive Buffer 1 Control Register (can be bit modified)
  RegMCP251X_RXB1SIDH  = 0b01110001, //!< (Offset: 0x71) Receive Buffer 1 Standard Identifier Register High
  RegMCP251X_RXB1SIDL  = 0b01110010, //!< (Offset: 0x72) Receive Buffer 1 Standard Identifier Register Low
  RegMCP251X_RXB1EID8  = 0b01110011, //!< (Offset: 0x73) Receive Buffer 1 Extended Identifier 8 Register High
  RegMCP251X_RXB1EID0  = 0b01110100, //!< (Offset: 0x74) Receive Buffer 1 Extended Identifier 0 Register Low
  RegMCP251X_RXB1DLC   = 0b01110101, //!< (Offset: 0x75) Receive Buffer 1 Data Length Code Register
  RegMCP251X_RXB1D0    = 0b01110110, //!< (Offset: 0x76) Receive Buffer 1 Data Byte 0 Register
  RegMCP251X_RXB1D1    = 0b01110111, //!< (Offset: 0x77) Receive Buffer 1 Data Byte 1 Register
  RegMCP251X_RXB1D2    = 0b01111000, //!< (Offset: 0x78) Receive Buffer 1 Data Byte 2 Register
  RegMCP251X_RXB1D3    = 0b01111001, //!< (Offset: 0x79) Receive Buffer 1 Data Byte 3 Register
  RegMCP251X_RXB1D4    = 0b01111010, //!< (Offset: 0x7A) Receive Buffer 1 Data Byte 4 Register
  RegMCP251X_RXB1D5    = 0b01111011, //!< (Offset: 0x7B) Receive Buffer 1 Data Byte 5 Register
  RegMCP251X_RXB1D6    = 0b01111100, //!< (Offset: 0x7C) Receive Buffer 1 Data Byte 6 Register
  RegMCP251X_RXB1D7    = 0b01111101, //!< (Offset: 0x7D) Receive Buffer 1 Data Byte 7 Register
  RegMCP251X_CANSTAT7  = 0b01111110, //!< (Offset: 0x7E) CAN Status Register 7 (can be bit modified)
  RegMCP251X_CANCTRL7  = 0b01111111, //!< (Offset: 0x7F) CAN Control Register 7 (can be bit modified)
} eMCP251X_Registers;

//-----------------------------------------------------------------------------

#define RegMCP251X_TXBnCTRL(n)  ( RegMCP251X_TXB0CTRL + (eMCP251X_Registers)(n << 4) ) //!< Select the TXBnCTRL register following the TXB index
#define RegMCP251X_TXBnSIDH(n)  ( RegMCP251X_TXB0SIDH + (eMCP251X_Registers)(n << 4) ) //!< Select the TXBnSIDH register following the TXB index
#define RegMCP251X_RXBnSIDH(n)  ( RegMCP251X_RXB0SIDH + (eMCP251X_Registers)(n << 4) ) //!< Select the RXBnSIDH register following the TXB index

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Specific Controller Registers
//********************************************************************************************************************

//! @brief BFPCTRL: RXnBF Pin Control and Status Register (Read/Write, Address: 0x0C, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_BFPCTRL_Register
{
  uint8_t BFPCTRL;
  struct
  {
    uint8_t B0BFM: 1; //!< 0 - RX0BF Pin Operation mode: '1' = Pin is used as an interrupt when a valid message is loaded into RXB0 ; '0' = Digital Output mode
    uint8_t B1BFM: 1; //!< 1 - RX1BF Pin Operation mode: '1' = Pin is used as an interrupt when a valid message is loaded into RXB1 ; '0' = Digital Output mode
    uint8_t B0BFE: 1; //!< 2 - RX0BF Pin Function Enable: '1' = Pin function is enabled, operation mode is determined by the B0BFM ; '0' = Pin function is disabled, pin goes to a high-impedance state
    uint8_t B1BFE: 1; //!< 3 - RX1BF Pin Function Enable: '1' = Pin function is enabled, operation mode is determined by the B1BFM ; '0' = Pin function is disabled, pin goes to a high-impedance state
    uint8_t B0BFS: 1; //!< 4 - RX0BF Pin State bit (Digital Output mode only). Reads as '0' when RX0BF is configured as an interrupt pin
    uint8_t B1BFS: 1; //!< 5 - RX1BF Pin State bit (Digital Output mode only). Reads as '0' when RX1BF is configured as an interrupt pin.
    uint8_t      : 2; //!< 6-7
  } Bits;
} MCP251X_BFPCTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_BFPCTRL_Register, 1);

#define MCP251X_RXnBF_INTERRUPT_MODE   (0x1u << 0) //!< Pin is used as an interrupt when a valid message is loaded into RXBx
#define MCP251X_RXnBF_OUTPUT_MODE      (0x0u << 0) //!< Digital Output mode
#define MCP251X_RXnBF_PIN_FUNC_ENABLE  (0x1u << 2) //!< Pin function is enabled, operation mode is determined by the BxBFM bit
#define MCP251X_RXnBF_PIN_FUNC_DISABLE (0x0u << 2) //!< Pin function is disabled, pin goes to a high-impedance state
#define MCP251X_RX0BF_GPIO_HIGH        (0x1u << 4) //!< RX0BF Latch Drive Pin High
#define MCP251X_RX0BF_GPIO_LOW         (0x0u << 4) //!< RX0BF Latch Drive Pin Low
#define MCP251X_RX1BF_GPIO_HIGH        (0x1u << 5) //!< RX1BF Latch Drive Pin High
#define MCP251X_RX1BF_GPIO_LOW         (0x0u << 5) //!< RX1BF Latch Drive Pin Low

//! RXnBF pin configuration
typedef enum
{
  MCP251X_RxPIN_DISABLE     = MCP251X_RXnBF_PIN_FUNC_DISABLE,                               //!< RXnBF pin disable. Pin goes to a high-impedance state
  MCP251X_RxPIN_AS_INT_RX   = MCP251X_RXnBF_PIN_FUNC_ENABLE | MCP251X_RXnBF_INTERRUPT_MODE, //!< RXnBF pin as RX Interrupt output (active low)
  MCP251X_RxPIN_AS_GPIO_OUT = MCP251X_RXnBF_PIN_FUNC_ENABLE | MCP251X_RXnBF_OUTPUT_MODE,    //!< RXnBF pin as GPIO output
} eMCP251X_RxPinMode;

#define MCP251X_Rx0BF_MODE_Pos         0
#define MCP251X_Rx0BF_MODE_Mask        (0x5u << MCP251X_Rx0BF_MODE_Pos)
#define MCP251X_Rx0BF_MODE_SET(value)  (((uint8_t)(value) << MCP251X_Rx0BF_MODE_Pos) & MCP251X_Rx0BF_MODE_Mask) //!< Set Rx0BF pin mode
#define MCP251X_Rx0BF_MODE_GET(value)  (eMCP251X_RxPinMode)(((uint8_t)(value) & MCP251X_Rx0BF_MODE_Mask) >> MCP251X_Rx0BF_MODE_Pos) //!< Get Rx0BF pin mode
#define MCP251X_Rx1BF_MODE_Pos         1
#define MCP251X_Rx1BF_MODE_Mask        (0x5u << MCP251X_Rx1BF_MODE_Pos)
#define MCP251X_Rx1BF_MODE_SET(value)  (((uint8_t)(value) << MCP251X_Rx1BF_MODE_Pos) & MCP251X_Rx1BF_MODE_Mask) //!< Set Rx1BF pin mode
#define MCP251X_Rx1BF_MODE_GET(value)  (eMCP251X_RxPinMode)(((uint8_t)(value) & MCP251X_Rx1BF_MODE_Mask) >> MCP251X_Rx1BF_MODE_Pos) //!< Get Rx1BF pin mode

//-----------------------------------------------------------------------------

//! @brief TXRTSCTRL: TXnRTS Pin Control and Status Register (Read/Write, Address: 0x0D, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_TXRTSCTRL_Register
{
  uint8_t TXRTSCTRL;
  struct
  {
    uint8_t B0RTSM: 1; //!< 0 - TX0RTS Pin mode: '1' = Pin is used to request message transmission of TXB0 buffer (on falling edge) ; '0' = Digital input
    uint8_t B1RTSM: 1; //!< 1 - TX1RTS Pin mode: '1' = Pin is used to request message transmission of TXB1 buffer (on falling edge) ; '0' = Digital input
    uint8_t B2RTSM: 1; //!< 2 - TX2RTS Pin mode: '1' = Pin is used to request message transmission of TXB2 buffer (on falling edge) ; '0' = Digital input
    uint8_t B0RTS : 1; //!< 3 - TX0RTS Pin State. Reads state of TX0RTS pin when in Digital Input mode, Reads as '0' when pin is in Request-to-Send mode
    uint8_t B1RTS : 1; //!< 4 - TX1RTS Pin State. Reads state of TX1RTS pin when in Digital Input mode, Reads as '0' when pin is in Request-to-Send mode
    uint8_t B2RTS : 1; //!< 5 - TX2RTS Pin State. Reads state of TX2RTS pin when in Digital Input mode, Reads as '0' when pin is in Request-to-Send mode
    uint8_t       : 2; //!< 6-7
  } Bits;
} MCP251X_TXRTSCTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_TXRTSCTRL_Register, 1);

#define MCP251X_TXnRTS_INTERRUPT_MODE   (0x1u << 0) //!< Pin is used as an interrupt when a valid message is loaded into RXBx
#define MCP251X_TXnRTS_INPUT_MODE       (0x0u << 0) //!< Digital Output mode
#define MCP251X_TX0RTS_GPIO_HIGH        (0x1u << 3) //!< TX0RTS is high level '1'
#define MCP251X_TX0RTS_GPIO_LOW         (0x0u << 3) //!< TX0RTS is low level '0'
#define MCP251X_TX1RTS_GPIO_HIGH        (0x1u << 4) //!< TX1RTS is high level '1'
#define MCP251X_TX1RTS_GPIO_LOW         (0x0u << 4) //!< TX1RTS is low level '0'
#define MCP251X_TX2RTS_GPIO_HIGH        (0x1u << 5) //!< TX2RTS is high level '1'
#define MCP251X_TX2RTS_GPIO_LOW         (0x0u << 5) //!< TX2RTS is low level '0'

//! RXnBF pin configuration
typedef enum
{
  MCP251X_TxPIN_AS_INT_TX  = MCP251X_TXnRTS_INTERRUPT_MODE, //!< Pin is used to request message transmission of TXnRTS buffer (on falling edge)
  MCP251X_TxPIN_AS_GPIO_IN = MCP251X_TXnRTS_INPUT_MODE,     //!< TXnRTS pin as GPIO input
} eMCP251X_TxPinMode;

#define MCP251X_TX0RTS_MODE_Pos         0
#define MCP251X_TX0RTS_MODE_Mask        (0x1u << MCP251X_TX0RTS_MODE_Pos)
#define MCP251X_TX0RTS_MODE_SET(value)  (((uint8_t)(value) << MCP251X_TX0RTS_MODE_Pos) & MCP251X_TX0RTS_MODE_Mask) //!< Set TX0RTS pin mode
#define MCP251X_TX0RTS_MODE_GET(value)  (eMCP251X_TxPinMode)(((uint8_t)(value) & MCP251X_TX0RTS_MODE_Mask) >> MCP251X_TX0RTS_MODE_Pos) //!< Get TX0RTS pin mode
#define MCP251X_TX1RTS_MODE_Pos         1
#define MCP251X_TX1RTS_MODE_Mask        (0x1u << MCP251X_TX1RTS_MODE_Pos)
#define MCP251X_TX1RTS_MODE_SET(value)  (((uint8_t)(value) << MCP251X_TX1RTS_MODE_Pos) & MCP251X_TX1RTS_MODE_Mask) //!< Set TX1RTS pin mode
#define MCP251X_TX1RTS_MODE_GET(value)  (eMCP251X_TxPinMode)(((uint8_t)(value) & MCP251X_TX1RTS_MODE_Mask) >> MCP251X_TX1RTS_MODE_Pos) //!< Get TX1RTS pin mode
#define MCP251X_TX2RTS_MODE_Pos         2
#define MCP251X_TX2RTS_MODE_Mask        (0x1u << MCP251X_TX2RTS_MODE_Pos)
#define MCP251X_TX2RTS_MODE_SET(value)  (((uint8_t)(value) << MCP251X_TX2RTS_MODE_Pos) & MCP251X_TX2RTS_MODE_Mask) //!< Set TX2RTS pin mode
#define MCP251X_TX2RTS_MODE_GET(value)  (eMCP251X_TxPinMode)(((uint8_t)(value) & MCP251X_TX2RTS_MODE_Mask) >> MCP251X_TX2RTS_MODE_Pos) //!< Get TX2RTS pin mode

//-----------------------------------------------------------------------------

//! @brief TEC: Transmit Error Counter Register (Read/Write, Address: 0x1C, Initial value: 0b00000000)
typedef uint8_t MCP251X_TEC_Register;

//-----------------------------------------------------------------------------

//! @brief TEC: Receive Error Counter Register (Read/Write, Address: 0x1D, Initial value: 0b00000000)
typedef uint8_t MCP251X_REC_Register;

//-----------------------------------------------------------------------------

//! @brief CNF3: CAN Timings Configuration Register 3 (Read/Write, Address: 0x28, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CNF3_Register
{
  uint8_t CNF3;
  struct
  {
    uint8_t PHSEG2: 3; //!< 0-2 - PS2 Length. (PHSEG2 + 1) * Tq. Minimum valid setting for PS2 is 2 Tq
    uint8_t       : 3; //!< 3-5
    uint8_t WAKFIL: 1; //!< 6   - Wake-up Filter: '1' = Wake-up filter is enabled ; '0' = Wake-up filter is disabled
    uint8_t SOF   : 1; //!< 7   - [MSP2515 only] Start-of-Frame Signal. If CLKEN = '1': '1' = CLKOUT pin is enabled for SOF signal ; '0' = CLKOUT pin is enabled for clock out function
  } Bits;
} MCP251X_BTR1_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_BTR1_Register, 1);

#define MCP251X_CNF3_PHSEG2_Pos          0
#define MCP251X_CNF3_PHSEG2_Mask         (0x7u << MCP251X_CNF3_PHSEG2_Pos)
#define MCP251X_CNF3_PHSEG2_SET(value)   (((uint8_t)(value) << MCP251X_CNF3_PHSEG2_Pos) & MCP251X_CNF3_PHSEG2_Mask) //!< Set Phase Segment 2
#define MCP251X_CNF3_WAKEUP_FILTER_EN    (0x1u << 6) //!< Enable the wake-up filter
#define MCP251X_CNF3_WAKEUP_FILTER_DIS   (0x0u << 6) //!< Disable the wake-up filter
#define MCP251X_CNF3_WAKEUP_FILTER_Mask  (0x1u << 6) //!< Mask the wake-up filter
#define MCP251X_CNF3_CLKOUT_IS_SOF       (0x1u << 7) //!< CLKOUT pin is enabled for SOF signal
#define MCP251X_CNF3_CLKOUT_IS_CLK       (0x0u << 7) //!< CLKOUT pin is enabled for clock out function

//-----------------------------------------------------------------------------

//! @brief CNF3: CAN Timings Configuration Register 2 (Read/Write, Address: 0x29, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CNF2_Register
{
  uint8_t CNF2;
  struct
  {
    uint8_t PRSEG  : 3; //!< 0-2 - Propagation Segment Length. (PRSEG + 1) * Tq
    uint8_t PHSEG1 : 3; //!< 3-5 - PS1 Length. (PHSEG1 + 1) * Tq
    uint8_t SAMPL  : 1; //!< 6   - Sample Point Configuration: '1' = Bus line is sampled three times at the sample point ; '0' = Bus line is sampled once at the sample point
    uint8_t BTLMODE: 1; //!< 7   - PS2 Bit Time Length mode: '1' = Length of PS2 is determined by the PHSEG2 bits of CNF3 ; '0' = Length of PS2 is the greater of PS1 and IPT (2 Tq)
  } Bits;
} MCP251X_CNF2_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CNF2_Register, 1);

#define MCP251X_CNF2_PRSEG_Pos           0
#define MCP251X_CNF2_PRSEG_Mask          (0x7u << MCP251X_CNF2_PRSEG_Pos)
#define MCP251X_CNF2_PRSEG_SET(value)    (((uint8_t)(value) << MCP251X_CNF2_PRSEG_Pos) & MCP251X_CNF2_PRSEG_Mask) //!< Set Propagation Segment Length
#define MCP251X_CNF2_PHSEG1_Pos          3
#define MCP251X_CNF2_PHSEG1_Mask         (0x7u << MCP251X_CNF2_PHSEG1_Pos)
#define MCP251X_CNF2_PHSEG1_SET(value)   (((uint8_t)(value) << MCP251X_CNF2_PHSEG1_Pos) & MCP251X_CNF2_PHSEG1_Mask) //!< Set Phase Segment 1
#define MCP251X_CNF2_3_SAMPLES           (0x1u << 6) //!< The bus is sampled three times
#define MCP251X_CNF2_1_SAMPLE            (0x0u << 6) //!< The bus is sampled once
#define MCP251X_CNF2_BTLMODE_IS_PHSEG2   (0x1u << 7) //!< Length of PS2 is determined by the PHSEG2 bits of CNF3
#define MCP251X_CNF2_BTLMODE_IS_PS1_IPT  (0x0u << 7) //!< Length of PS2 is the greater of PS1 and IPT (2 Tq)

//-----------------------------------------------------------------------------

//! @brief CNF3: CAN Timings Configuration Register 1 (Read/Write, Address: 0x2A, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CNF1_Register
{
  uint8_t CNF1;
  struct
  {
    uint8_t BRP: 6; //!< 0-5 - Baud Rate Prescaler
    uint8_t SJW: 2; //!< 6-7 - Synchronization Jump Width Length
  } Bits;
} MCP251X_CNF1_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CNF1_Register, 1);

#define MCP251X_CNF1_BRP_Pos         0
#define MCP251X_CNF1_BRP_Mask        (0x3Fu << MCP251X_CNF1_BRP_Pos)
#define MCP251X_CNF1_BRP_SET(value)  (((uint8_t)(value) << MCP251X_CNF1_BRP_Pos) & MCP251X_CNF1_BRP_Mask) //!< Set Baud Rate Prescaler
#define MCP251X_CNF1_SJW_Pos         6
#define MCP251X_CNF1_SJW_Mask        (0x3u << MCP251X_CNF1_SJW_Pos)
#define MCP251X_CNF1_SJW_SET(value)  (((uint8_t)(value) << MCP251X_CNF1_SJW_Pos) & MCP251X_CNF1_SJW_Mask) //!< Set Synchronization Jump Width Length

//-----------------------------------------------------------------------------

//! @brief CANINTE: CAN Interrupt Enable Register (Read/Write, Address: 0x2B, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CANINTE_Register
{
  uint8_t IE;
  struct
  {
    uint8_t RX0IE: 1; //!< 0 - Receive Buffer 0 Full Interrupt Enable: '1' = Interrupt when message was received in RXB0 ; '0' = Disabled
    uint8_t RX1IE: 1; //!< 1 - Receive Buffer 1 Full Interrupt Enable: '1' = Interrupt when message was received in RXB1 ; '0' = Disabled
    uint8_t TX0IE: 1; //!< 2 - Transmit Buffer 0 Empty Interrupt Enable: '1' = Interrupt on TXB0 becoming empty ; '0' = Disabled
    uint8_t TX1IE: 1; //!< 3 - Transmit Buffer 1 Empty Interrupt Enable: '1' = Interrupt on TXB1 becoming empty ; '0' = Disabled
    uint8_t TX2IE: 1; //!< 4 - Transmit Buffer 2 Empty Interrupt Enable: '1' = Interrupt on TXB2 becoming empty ; '0' = Disabled
    uint8_t ERRIE: 1; //!< 5 - Error Interrupt Enable (multiple sources in EFLG register): '1' = Interrupt on EFLG error condition change ; '0' = Disabled
    uint8_t WAKIE: 1; //!< 6 - Wake-up Interrupt Enable: '1' = Interrupt on CAN bus activity ; '0' = Disabled
    uint8_t MERRE: 1; //!< 7 - Message Error Interrupt Enable: '1' = Interrupt on error during message reception or transmission ; '0' = Disabled
  } Bits;
} MCP251X_CANINTE_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CANINTE_Register, 1);

#define MCP251X_RX0IE_EVENT  (0x1u << 0) //!< Receive Buffer 0 Full Interrupt Event
#define MCP251X_RX1IE_EVENT  (0x1u << 1) //!< Receive Buffer 1 Full Interrupt Event
#define MCP251X_TX0IE_EVENT  (0x1u << 2) //!< Transmit Buffer 0 Empty Interrupt Event
#define MCP251X_TX1IE_EVENT  (0x1u << 3) //!< Transmit Buffer 1 Empty Interrupt Event
#define MCP251X_TX2IE_EVENT  (0x1u << 4) //!< Transmit Buffer 2 Empty Interrupt Event
#define MCP251X_ERRIE_EVENT  (0x1u << 5) //!< Error Interrupt Event
#define MCP251X_WAKIE_EVENT  (0x1u << 6) //!< Wake-up Interrupt Event
#define MCP251X_MERRE_EVENT  (0x1u << 7) //!< Message Error Interrupt Event

#define MCP251X_WAKIE_Mask   (0x1u << 6) //!< Wake-up Interrupt Mask

#define MCP251X_INT_ALL_EVENTS  ( MCP251X_RX0IE_EVENT | MCP251X_RX1IE_EVENT | MCP251X_TX0IE_EVENT | MCP251X_TX1IE_EVENT \
                                | MCP251X_TX2IE_EVENT | MCP251X_ERRIE_EVENT | MCP251X_WAKIE_EVENT | MCP251X_MERRE_EVENT )

//! Interrupt Events, can be OR'ed.
typedef enum
{
  MCP251X_NO_INTERRUPT_EVENT     = 0x00,                   //!< No interrupt events
  MCP251X_RX_BUFFER0_FULL_EVENT  = MCP251X_RX0IE_EVENT,    //!< Receive Buffer 0 Full event
  MCP251X_RX_BUFFER1_FULL_EVENT  = MCP251X_RX1IE_EVENT,    //!< Receive Buffer 1 Full event
  MCP251X_TX_BUFFER0_EMPTY_EVENT = MCP251X_TX0IE_EVENT,    //!< Transmit Buffer 0 Empty event
  MCP251X_TX_BUFFER1_EMPTY_EVENT = MCP251X_TX1IE_EVENT,    //!< Transmit Buffer 1 Empty event
  MCP251X_TX_BUFFER2_EMPTY_EVENT = MCP251X_TX2IE_EVENT,    //!< Transmit Buffer 2 Empty event
  MCP251X_ERROR_EVENT            = MCP251X_ERRIE_EVENT,    //!< Error event
  MCP251X_WAKEUP_EVENT           = MCP251X_WAKIE_EVENT,    //!< Wake-up event
  MCP251X_MESSAGE_ERROR_EVENT    = MCP251X_MERRE_EVENT,    //!< Message Error event

  MCP251X_ENABLE_ALL_INTERRUPTS  = MCP251X_INT_ALL_EVENTS, //!< Enable all interrupts
  MCP251X_INT_EVENTS_FLAGS_MASK  = MCP251X_INT_ALL_EVENTS, //!< Interrupt events flags mask
} eMCP251X_InterruptEvents;

typedef eMCP251X_InterruptEvents setMCP251X_InterruptEvents; //!< Set of Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

//! @brief CANINTE: CAN Interrupt Flag Register (Read/Write, Address: 0x2C, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CANINTF_Register
{
  uint8_t IF;
  struct
  {
    uint8_t RX0IF: 1; //!< 0 - Receive Buffer 0 Full Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t RX1IF: 1; //!< 1 - Receive Buffer 1 Full Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t TX0IF: 1; //!< 2 - Transmit Buffer 0 Empty Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t TX1IF: 1; //!< 3 - Transmit Buffer 1 Empty Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t TX2IF: 1; //!< 4 - Transmit Buffer 2 Empty Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t ERRIF: 1; //!< 5 - Error Interrupt Flag (multiple sources in EFLG register): '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t WAKIF: 1; //!< 6 - Wake-up Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
    uint8_t MERRF: 1; //!< 7 - Message Error Interrupt Flag: '1' = Interrupt is pending (must be cleared by MCU to reset the interrupt condition) ; '0' = No interrupt is pending
  } Bits;
} MCP251X_CANINTF_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CANINTF_Register, 1);

//-----------------------------------------------------------------------------

//! @brief EFLG: Error Flag Register (Read/Write, Address: 0x2D, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_EFLG_Register
{
  uint8_t EFLG;
  struct
  {
    uint8_t EWARN : 1; //!< 0 - Error Warning Flag. Sets when TEC or REC is equal to or greater than 96 (TXWAR or RXWAR = 1). Resets when both REC and TEC are less than 96
    uint8_t RXWAR : 1; //!< 1 - Receive Error Warning Flag. Sets when REC is equal to or greater than 96. Resets when REC is less than 96
    uint8_t TXWAR : 1; //!< 2 - Transmit Error Warning Flag. Sets when TEC is equal to or greater than 96. Resets when TEC is less than 96
    uint8_t RXEP  : 1; //!< 3 - Receive Error Passive Flag. Sets when REC is equal to or greater than 128. Resets when REC is less than 128
    uint8_t TXEP  : 1; //!< 4 - Transmit Error Passive Flag. Sets when TEC is equal to or greater than 128. Resets when TEC is less than 128
    uint8_t TXBO  : 1; //!< 5 - Bus-Off Error Flag. Sets when TEC reaches 255. Resets after a successful bus recovery sequence
    uint8_t RX0OVR: 1; //!< 6 - Receive Buffer 0 Overflow Flag. Sets when a valid message is received for RXB0 and RX0IF (CANINTF[0]) = 1. Must be reset by MCU
    uint8_t RX1OVR: 1; //!< 7 - Receive Buffer 1 Overflow Flag. Sets when a valid message is received for RXB1 and RX1IF (CANINTF[1]) = 1. Must be reset by MCU
  } Bits;
} MCP251X_EFLG_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_EFLG_Register, 1);

#define MCP251X_EWARN_EVENT   (0x1u << 0) //!< Error Warning Event
#define MCP251X_RXWAR_EVENT   (0x1u << 1) //!< Receive Error Warning Event
#define MCP251X_TXWAR_EVENT   (0x1u << 2) //!< Transmit Error Warning Event
#define MCP251X_RXEP_EVENT    (0x1u << 3) //!< Receive Error Passive Event
#define MCP251X_TXEP_EVENT    (0x1u << 4) //!< Transmit Error Passive Event
#define MCP251X_TXBO_EVENT    (0x1u << 5) //!< Bus-Off Error Event
#define MCP251X_RX0OVR_EVENT  (0x1u << 6) //!< Receive Buffer 0 Overflow Event
#define MCP251X_RX1OVR_EVENT  (0x1u << 7) //!< Receive Buffer 1 Overflow Event

#define MCP251X_ERR_CLEAR_EVENTS  ( MCP251X_RX0OVR_EVENT | MCP251X_RX1OVR_EVENT )

//! Error Events, can be OR'ed.
typedef enum
{
  MCP251X_NO_ERROR_EVENT            = 0x00,                 //!< No error event
  MCP251X_TEC_REC_SUP_96_EVENT      = MCP251X_EWARN_EVENT,  //!< TEC or REC is equal to or greater than 96 event
  MCP251X_REC_SUP_96_EVENT          = MCP251X_RXWAR_EVENT,  //!< REC is equal to or greater than 96 event
  MCP251X_TEC_SUP_96_EVENT          = MCP251X_TXWAR_EVENT,  //!< TEC is equal to or greater than 96 event
  MCP251X_REC_SUP_128_EVENT         = MCP251X_RXEP_EVENT,   //!< REC is equal to or greater than 128 event
  MCP251X_TEC_SUP_128_EVENT         = MCP251X_TXEP_EVENT,   //!< TEC is equal to or greater than 128 event
  MCP251X_TEC_AT_255_EVENT          = MCP251X_TXBO_EVENT,   //!< TEC reaches 255 event
  MCP251X_RX_BUFFER0_OVERFLOW_EVENT = MCP251X_RX0OVR_EVENT, //!< Receive buffer 0 overflow event
  MCP251X_RX_BUFFER1_OVERFLOW_EVENT = MCP251X_RX1OVR_EVENT, //!< Receive buffer 1 overflow event

  MCP251X_CLEARABLE_ERRORS_MASK = MCP251X_ERR_CLEAR_EVENTS, //!< Clearable error flags mask
} eMCP251X_ErrorEvents;

typedef eMCP251X_ErrorEvents setMCP251X_ErrorEvents; //!< Set of Error Events (can be OR'ed)

//-----------------------------------------------------------------------------

//! @brief TXBnCTRL: Transmit Buffer n Control Register (Read/Write, Address: 0x30, 0x40, 0x50, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_TXBnCTRL_Register
{
  uint8_t TXBnCTRL;
  struct
  {
    uint8_t TXP  : 2; //!< 0-1 - Transmit Buffer Priority
    uint8_t      : 1; //!< 2
    uint8_t TXREQ: 1; //!< 3   - Message Transmit Request: '1' = Buffer is currently pending transmission (MCU sets this bit to request message be transmitted – bit is automatically cleared when the message is sent) ; '0' = Buffer is not currently pending transmission (MCU can clear this bit to request a message abort)
    uint8_t TXERR: 1; //!< 4   - Transmission Error Detected: '1' = A bus error occurred while the message was being transmitted ; '0' = No bus error occurred while the message was being transmitted
    uint8_t MLOA : 1; //!< 5   - Message Lost Arbitration: '1' = Message lost arbitration while being sent ; '0' = Message did not lose arbitration while being sent
    uint8_t ABTF : 1; //!< 6   - Message Aborted Flag: '1' = Message was aborted ; '0' = Message completed transmission successfully
    uint8_t      : 1; //!< 7
  } Bits;
} MCP251X_TXBnCTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_TXBnCTRL_Register, 1);

//! Message Transmit Priority
typedef enum
{
  MCP251X_MESSAGE_TX_PRIORITY1 = 0b00, //!< Lowest message priority
  MCP251X_MESSAGE_TX_PRIORITY2 = 0b01, //!< Low intermediate message priority
  MCP251X_MESSAGE_TX_PRIORITY3 = 0b10, //!< High intermediate message priority
  MCP251X_MESSAGE_TX_PRIORITY4 = 0b11, //!< Highest message priority
} eMCP251X_Priority;

#define MCP251X_TX_MESSAGE_PRIORITY_Pos         0
#define MCP251X_TX_MESSAGE_PRIORITY_Mask        (0x3u << MCP251X_TX_MESSAGE_PRIORITY_Pos)
#define MCP251X_TX_MESSAGE_PRIORITY_SET(value)  (((uint8_t)(value) << MCP251X_TX_MESSAGE_PRIORITY_Pos) & MCP251X_TX_MESSAGE_PRIORITY_Mask) //!< Set Message Transmit Priority
#define MCP251X_MESSAGE_TX_REQUEST              (0x1u << 3) //!< Message Transmit Request
#define MCP251X_PENDING_TRANSMISSION            (0x1u << 3) //!< Buffer is currently pending transmission
#define MCP251X_NO_PENDING_TRANSMISSION         (0x0u << 3) //!< Buffer is not currently pending transmission
#define MCP251X_TX_BUS_ERROR                    (0x1u << 4) //!< A bus error occurred while the message was being transmitted
#define MCP251X_ARBITRATION_LOST                (0x1u << 5) //!< Message Lost Arbitration
#define MCP251X_MESSAGE_WAS_ABORTED             (0x1u << 6) //!< Message was aborted
#define MCP251X_MESSAGE_TX_SUCCESS              (0x0u << 6) //!< Message completed transmission successfully

#define MCP251X_TX_BUFFER_ALL_ERR_STATUS  ( MCP251X_PENDING_TRANSMISSION | MCP251X_RX1OVR_EVENT )

//! Tx Buffer error status, can be OR'ed.
typedef enum
{
  MCP251X_NO_TX_BUFFER_ERROR_STATUS      = 0x00,                         //!< No Tx Buffer error status
  MCP251X_TX_BUFFER_PENDING_TRANSMISSION = MCP251X_PENDING_TRANSMISSION, //!< Buffer is currently pending transmission
  MCP251X_TX_BUFFER_BUS_ERROR            = MCP251X_TX_BUS_ERROR,         //!< A bus error occurred while the message was being transmitted
  MCP251X_TX_BUFFER_ARBITRATION_LOST     = MCP251X_ARBITRATION_LOST,     //!< Message Lost Arbitration
  MCP251X_TX_BUFFER_MESSAGE_WAS_ABORTED  = MCP251X_MESSAGE_WAS_ABORTED,  //!< Message was aborted

  MCP251X_TX_BUFFER_ERR_STATUS_MASK      = MCP251X_TX_BUFFER_ALL_ERR_STATUS, //!< Tx Buffer error status mask
} eMCP251X_TxBufferErrorStatus;

typedef eMCP251X_TxBufferErrorStatus setMCP251X_TxBufferErrorStatus; //!< Set of Tx Buffer error status (can be OR'ed)

//-----------------------------------------------------------------------------

//! @brief RXB0CTRL: Receive Buffer 0 Control Register (Read/Write, Address: 0x60, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_RXB0CTRL_Register
{
  uint8_t RXB0CTRL;
  struct
  {
    uint8_t FILHIT: 1; //!< 0   - Filter Hit (indicates which acceptance filter enabled reception of message). If a rollover from RXB0 to RXB1 occurs, the FILHIT0 bit will reflect the filter that accepted the message that rolled over
    uint8_t BUKT1 : 1; //!< 1   - Read-Only Copy of BUKT bit (used internally by the MCP2515)
    uint8_t BUKT  : 1; //!< 2   - Rollover Enable: '1' = RXB0 message will roll over and be written to RXB1 if RXB0 is full ; '0' = Rollover is disabled
    uint8_t RXRTR : 1; //!< 3   - Received Remote Transfer Request: '1' = Remote Transfer Request received ; '0' = No Remote Transfer Request received
    uint8_t       : 1; //!< 4
    uint8_t RXM   : 2; //!< 5-6 - Receive Buffer Operating mode
    uint8_t       : 1; //!< 7
  } Bits;
} MCP251X_RXB0CTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_RXB0CTRL_Register, 1);

//! Acceptance filter enumerator
typedef enum
{
  MCP251X_ACCEPTANCE_FILTER0 = 0b000, //!< Acceptance Filter 0 (RXF0) (only if the BUKT bit is set in RXB0CTRL)
  MCP251X_ACCEPTANCE_FILTER1 = 0b001, //!< Acceptance Filter 1 (RXF1) (only if the BUKT bit is set in RXB0CTRL)
  MCP251X_ACCEPTANCE_FILTER2 = 0b010, //!< Acceptance Filter 2 (RXF2)
  MCP251X_ACCEPTANCE_FILTER3 = 0b011, //!< Acceptance Filter 3 (RXF3)
  MCP251X_ACCEPTANCE_FILTER4 = 0b100, //!< Acceptance Filter 4 (RXF4)
  MCP251X_ACCEPTANCE_FILTER5 = 0b101, //!< Acceptance Filter 5 (RXF5)
} eMCP251X_AcceptanceFilter;

#define MCP251X_RX0_ACCEPTANCE_FILTER_Pos         0
#define MCP251X_RX0_ACCEPTANCE_FILTER_Mask        (0x1u << MCP251X_RX0_ACCEPTANCE_FILTER_Pos)
#define MCP251X_RX0_ACCEPTANCE_FILTER_GET(value)  (((uint8_t)(value) & MCP251X_RX0_ACCEPTANCE_FILTER_Mask) >> MCP251X_RX0_ACCEPTANCE_FILTER_Pos) //!< Get Acceptance filter for RX buffer 0
#define MCP251X_ROLLOVER_MESSAGE_EN               (0x1u << 2) //!< Enable Rollover
#define MCP251X_ROLLOVER_MESSAGE_DIS              (0x0u << 2) //!< Disable Rollover
#define MCP251X_ROLLOVER_MESSAGE_Mask             (0x1u << 2) //!< Rollover mask
#define MCP251X_RTR_FRAME_RECEIVED                (0x1u << 3) //!< Remote Transfer Request received

//! Receive Buffer Operating mode enumerator
typedef enum
{
  MCP251X_USE_ACCEPTANCE_FILTERS = 0b00, //!< Receives all valid messages using either Standard or Extended Identifiers that meet filter criteria. For Rx buffer 0 only: Extended ID Filter registers, RXFnEID8:RXFnEID0, are applied to the first two bytes of data in the messages with standard IDs
  MCP251X_RESERVED1              = 0b01, //!< Reserved
  MCP251X_RESERVED2              = 0b10, //!< Reserved
  MCP251X_ACCEPT_ALL_MESSAGES    = 0b11, //!< Turns mask/filters off; receives any message
} eMCP251X_FilterMode;

#define MCP251X_FILTER_MODE_Pos         5
#define MCP251X_FILTER_MODE_Mask        (0x3u << MCP251X_FILTER_MODE_Pos)
#define MCP251X_FILTER_MODE_SET(value)  (((uint8_t)(value) << MCP251X_FILTER_MODE_Pos) & MCP251X_FILTER_MODE_Mask) //!< Set eceive Buffer Operating mode

//-----------------------------------------------------------------------------

//! @brief RXB1CTRL: Receive Buffer 1 Control Register (Read/Write, Address: 0x70, Initial value: 0b00000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_RXB1CTRL_Register
{
  uint8_t RXB1CTRL;
  struct
  {
    uint8_t FILHIT: 3; //!< 0-2 - Filter Hit (indicates which acceptance filter enabled reception of message)
    uint8_t RXRTR : 1; //!< 3   - Received Remote Transfer Request: '1' = Remote Transfer Request received ; '0' = No Remote Transfer Request received
    uint8_t       : 1; //!< 4
    uint8_t RXM   : 2; //!< 5-6 - Receive Buffer Operating mode
    uint8_t       : 1; //!< 7
  } Bits;
} MCP251X_RXB1CTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_RXB1CTRL_Register, 1);

#define MCP251X_RX1_ACCEPTANCE_FILTER_Pos         0
#define MCP251X_RX1_ACCEPTANCE_FILTER_Mask        (0x7u << MCP251X_RX1_ACCEPTANCE_FILTER_Pos)
#define MCP251X_RX1_ACCEPTANCE_FILTER_GET(value)  (((uint8_t)(value) & MCP251X_RX1_ACCEPTANCE_FILTER_Mask) >> MCP251X_RX1_ACCEPTANCE_FILTER_Pos) //!< Get Acceptance filter for RX buffer 1

//! Receive Buffer configuration, can be OR'ed.
typedef enum
{
  MCP251X_NO_RX_BUFFER_CONFIGURATION = 0x00,                                                    //!< No Rx buffer configuration
  MCP251X_RX_USE_ACCEPTANCE_FILTERS  = MCP251X_FILTER_MODE_SET(MCP251X_USE_ACCEPTANCE_FILTERS), //!< Receives all valid messages using either Standard or Extended Identifiers that meet filter criteria. For Rx buffer 0 only: Extended ID Filter registers, RXFnEID8:RXFnEID0, are applied to the first two bytes of data in the messages with standard IDs
  MCP251X_RX_ACCEPT_ALL_MESSAGES     = MCP251X_FILTER_MODE_SET(MCP251X_ACCEPT_ALL_MESSAGES),    //!< Turns mask/filters off; receives any message
  MCP251X_RX_USE_ROLL_OVER_MESSAGES  = MCP251X_ROLLOVER_MESSAGE_EN,                             //!< Enable Rollover. RXB0 message will roll over and be written to RXB1 if RXB0 is full
} eMCP251X_RxBufferConfig;

typedef eMCP251X_RxBufferConfig setMCP251X_RxBufferConfig; //!< Set of Receive Buffer configuration (can be OR'ed)

//-----------------------------------------------------------------------------

//! @brief CANSTAT: CAN Status Register (Read/Write, Address: 0xXE, Initial value: 0b10000000)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CANSTAT_Register
{
  uint8_t CANSTAT;
  struct
  {
    uint8_t      : 1; //!< 0
    uint8_t ICOD : 3; //!< 1-3 - Interrupt Flag Code
    uint8_t      : 1; //!< 4
    uint8_t OPMOD: 3; //!< 5-7 - Operation Mode
  } Bits;
} MCP251X_CANSTAT_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CANSTAT_Register, 1);

//! Interrupt flag code enumerator
typedef enum
{
  MCP251X_NO_INT_FLAG_CODE     = 0b000, //!< No interrupt
  MCP251X_ERROR_INT_FLAG_CODE  = 0b001, //!< Error interrupt
  MCP251X_WAKEUP_INT_FLAG_CODE = 0b010, //!< Wake-up interrupt
  MCP251X_TXB0_INT_FLAG_CODE   = 0b011, //!< TXB0 interrupt
  MCP251X_TXB1_INT_FLAG_CODE   = 0b100, //!< TXB1 interrupt
  MCP251X_TXB2_INT_FLAG_CODE   = 0b101, //!< TXB2 interrupt
  MCP251X_RXB0_INT_FLAG_CODE   = 0b110, //!< RXB0 interrupt
  MCP251X_RXB1_INT_FLAG_CODE   = 0b111, //!< RXB1 interrupt
} eMCP251X_IntFlagCode;

#define MCP251X_CANSTAT_INT_FLAG_CODE_Pos         1
#define MCP251X_CANSTAT_INT_FLAG_CODE_Mask        (0x7u << MCP251X_CANSTAT_INT_FLAG_CODE_Pos)
#define MCP251X_CANSTAT_INT_FLAG_CODE_GET(value)  (((uint8_t)(value) & MCP251X_CANSTAT_INT_FLAG_CODE_Mask) >> MCP251X_CANSTAT_INT_FLAG_CODE_Pos) //!< Get interrupt flag code

//! CAN Controller Operation Modes enumerator. All other values for the REQOPn bits are invalid and should not be used
typedef enum
{
  MCP251X_NORMAL_MODE            = 0b000, //!< Set Normal CAN mode
  MCP251X_SLEEP_MODE             = 0b001, //!< Set Sleep mode
  MCP251X_INTERNAL_LOOPBACK_MODE = 0b010, //!< Set Internal Loopback mode
  MCP251X_LISTEN_ONLY_MODE       = 0b011, //!< Set Listen Only mode
  MCP251X_CONFIGURATION_MODE     = 0b100, //!< Set Configuration mode
} eMCP251X_OperationMode;

#define MCP251X_CANSTAT_OPMOD_Pos         5
#define MCP251X_CANSTAT_OPMOD_Mask        (0x7u << MCP251X_CANSTAT_OPMOD_Pos)
#define MCP251X_CANSTAT_OPMOD_GET(value)  (((uint8_t)(value) & MCP251X_CANSTAT_OPMOD_Mask) >> MCP251X_CANSTAT_OPMOD_Pos) //!< Get Operation Mode

//-----------------------------------------------------------------------------

//! @brief CANCTRL: CAN Control Register (Read/Write, Address: 0xXF, Initial value: 0b10000111)
MCP251X_PACKITEM
typedef union __MCP251X_PACKED__ MCP251X_CANCTRL_Register
{
  uint8_t CANCTRL;
  struct
  {
    uint8_t CLKPRE: 2; //!< 0-1 - CLKOUT Pin Prescaler
    uint8_t CLKEN : 1; //!< 2   - CLKOUT Pin Enable: '1' = CLKOUT pin is enabled ; '0' = CLKOUT pin is disabled (pin is in high-impedance state)
    uint8_t OSM   : 1; //!< 3   - [MSP2515 only] One-Shot Mode: '1' = Enabled; messages will only attempt to transmit one time ; '0' = Disabled; messages will reattempt transmission if required
    uint8_t ABAT  : 1; //!< 4   - Abort All Pending Transmissions: '1' = Requests abort of all pending transmit buffers ; '0' = Terminates request to abort all transmissions
    uint8_t REQOP : 3; //!< 5-7 - Request Operation Mode
  } Bits;
} MCP251X_CANCTRL_Register;
MCP251X_UNPACKITEM;
MCP251X_CONTROL_ITEM_SIZE(MCP251X_CANCTRL_Register, 1);

//! CAN Controller Operation Modes enumerator. All other values for the REQOPn bits are invalid and should not be used
typedef enum
{
  MCP251X_CLKOUT_DISABLE      = MCP251X_CNF3_CLKOUT_IS_CLK | 0b011, //!< CLKOUT pin is disabled (pin is in high-impedance state)
  MCP251X_CLKOUT_SYSCLK_DIV_1 = MCP251X_CNF3_CLKOUT_IS_CLK | 0b100, //!< CLKOUT Pin Prescaler: fCLKOUT = System Clock/1
  MCP251X_CLKOUT_SYSCLK_DIV_2 = MCP251X_CNF3_CLKOUT_IS_CLK | 0b101, //!< CLKOUT Pin Prescaler: fCLKOUT = System Clock/2
  MCP251X_CLKOUT_SYSCLK_DIV_4 = MCP251X_CNF3_CLKOUT_IS_CLK | 0b110, //!< CLKOUT Pin Prescaler: fCLKOUT = System Clock/4
  MCP251X_CLKOUT_SYSCLK_DIV_8 = MCP251X_CNF3_CLKOUT_IS_CLK | 0b111, //!< CLKOUT Pin Prescaler: fCLKOUT = System Clock/8
  MCP251X_CLKOUT_IS_SOF       = MCP251X_CNF3_CLKOUT_IS_SOF | 0b111, //!< [MSP2515 only] CLKOUT Pin is Start Of Frame signal
} eMCP251X_CLKOUTpinMode;

#define MCP251X_CANCTRL_CLKOUT_Pos         0
#define MCP251X_CANCTRL_CLKOUT_Mask        (0x7u << MCP251X_CANCTRL_CLKOUT_Pos)
#define MCP251X_CANCTRL_CLKOUT_SET(value)  (((uint8_t)(value) << MCP251X_CANCTRL_CLKOUT_Pos) & MCP251X_CANCTRL_CLKOUT_Mask) //!< Set CLKOUT configuration
#define MCP251X_CANCTRL_OSM                (0x1u << 3) //!< Set One-Shot Mode
#define MCP251X_CANCTRL_OSM_Mask           (0x1u << 3) //!< Mask One-Shot Mode
#define MCP251X_CANCTRL_ABAT               (0x1u << 4) //!< Set Abort All Pending Transmissions
#define MCP251X_CANCTRL_REQOP_Pos          5
#define MCP251X_CANCTRL_REQOP_Mask         (0x7u << MCP251X_CANCTRL_REQOP_Pos)
#define MCP251X_CANCTRL_REQOP_SET(value)   (((uint8_t)(value) << MCP251X_CANCTRL_REQOP_Pos) & MCP251X_CANCTRL_REQOP_Mask) //!< Set Request Operation Mode

//-----------------------------------------------------------------------------





//********************************************************************************************************************

//! List of supported devices
typedef enum
{
  MCP2510 = 0x0, //!< MCP2510 supported
  MCP2515 = 0x1, //!< MCP2515 supported
  eMPC251X_DEVICE_COUNT, // Device count of this enum, keep last
} eMCP251X_Devices;

#define MCP251X_DEV_ID_Pos         2
#define MCP251X_DEV_ID_Mask        (0x1u << MCP251X_DEV_ID_Pos)
#define MCP251X_DEV_ID_SET(value)  (((uint8_t)(value) << MCP251X_DEV_ID_Pos) & MCP251X_DEV_ID_Mask) //!< Set Device ID
#define MCP251X_DEV_ID_GET(value)  (((uint8_t)(value) & MCP251X_DEV_ID_Mask) >> MCP251X_DEV_ID_Pos) //!< Get Device ID

static const char* const MCP251X_DevicesNames[eMPC251X_DEVICE_COUNT] =
{
  "MCP2510",
  "MCP2515",
};

//-----------------------------------------------------------------------------

//! Available FIFO/Buffer list
typedef enum
{
  MCP251X_RX_BUFFER0,   //!< RX buffer 0
  MCP251X_RX_BUFFER1,   //!< RX buffer 1
  MCP251X_TX_BUFFER0,   //!< TX buffer 0
  MCP251X_TX_BUFFER1,   //!< TX buffer 1
  MCP251X_TX_BUFFER2,   //!< TX buffer 2
  MCP251X_NO_FIFO_BUFF, //!< No specific FIFO/Buffer
} eMCP251X_FIFObuffer;

//-----------------------------------------------------------------------------

//! Data Field Size enumerator
typedef enum
{
  MCP251X_0_BYTES, //!< 0-byte data field
  MCP251X_1_BYTES, //!< 1-byte data field
  MCP251X_2_BYTES, //!< 2-byte data field
  MCP251X_3_BYTES, //!< 3-byte data field
  MCP251X_4_BYTES, //!< 4-byte data field
  MCP251X_5_BYTES, //!< 5-byte data field
  MCP251X_6_BYTES, //!< 6-byte data field
  MCP251X_7_BYTES, //!< 7-byte data field
  MCP251X_8_BYTES, //!< 8-byte data field
  MCP251X_PAYLOAD_COUNT, // Keep last
} eMCP251X_PayloadSize;

static const uint8_t MCP251X_PAYLOAD_TO_VALUE[MCP251X_PAYLOAD_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X FIFO/Buffers
//********************************************************************************************************************

// FIFO/Buffers definitions
#define MCP251X_TEF_MAX        ( 0 ) //!< 0 TEF maximum
#define MCP251X_TXQ_MAX        ( 0 ) //!< 0 TXQ maximum
#define MCP251X_TX_BUFFER_MAX  ( 3 ) //!< 3 Tx buffer maximum
#define MCP251X_RX_BUFFER_MAX  ( 2 ) //!< 3 Rx buffer maximum
#define MCP251X_FIFO_CONF_MAX  ( 0 ) //!< Maximum 0 buffers configurable
#define MCP251X_TX_FIFO_MAX    ( MCP251X_TXQ_MAX + MCP251X_TX_BUFFER_MAX ) //!< Maximum 3 transmit buffer (TXQ + Tx buffer)
#define MCP251X_RX_FIFO_MAX    ( MCP251X_TEF_MAX + MCP251X_RX_BUFFER_MAX ) //!< Maximum 2 receive buffer (TEF + Rx buffer)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Filter
//********************************************************************************************************************

// Filters definitions
#define MCP251X_SID_FILTERS_MAX  ( 5 ) //!< 5 Standard filters elements maximum
#define MCP251X_EID_FILTERS_MAX  ( 5 ) //!< 5 Extended filters elements maximum
#define MCP251X_FILTERS_MAX      ( MCP251X_SID_FILTERS_MAX ) //!< 5 SID or 5 EID filters elements maximum

//-----------------------------------------------------------------------------

#define MCP251X_ACCEPT_ALL_MESSAGES  ( 0x00000000u ) //!< Indicate that the filter will accept all messages

//-----------------------------------------------------------------------------

//! Filter match type
typedef enum
{
  MCP251X_MATCH_ONLY_SID = 0x0, //!< Match only messages with standard identifier (+SID11 in FD mode if configured)
  MCP251X_MATCH_SID_EID  = 0x2, //!< Match both standard and extended message frames
} eMCP251X_FilterMatch;

//-----------------------------------------------------------------------------

//! MCP251X Filter 0 configuration structure
typedef struct MCP251X_Filter0
{
  //--- Configuration ---
  eMCP251X_FilterMatch MatchID0; //!< Filter match type of the frame for ID0 (SID and/or EID)
  eMCP251X_FilterMatch MatchID1; //!< Filter match type of the frame for ID1 (SID and/or EID)

  //--- Message Filter ---
  uint32_t AcceptanceID0;        //!< Message Filter Acceptance 0 SID+EID for ID0. Extended ID Filter registers, EID8:EID0, are applied to the first two bytes of data in the messages with standard IDs
  uint32_t AcceptanceID1;        //!< Message Filter Acceptance 1 SID+EID for ID1. Extended ID Filter registers, EID8:EID0, are applied to the first two bytes of data in the messages with standard IDs
  uint32_t AcceptanceMask0;      //!< Message Filter Mask 0 SID+EID (corresponding bits to AcceptanceID: '1': bit to filter ; '0' bit that do not care)
} MCP251X_Filter0;

//-----------------------------------------------------------------------------

//! MCP251X Filter 1 configuration structure
typedef struct MCP251X_Filter1
{
  //--- Configuration ---
  eMCP251X_FilterMatch MatchID2; //!< Filter match type of the frame for ID2 (SID and/or EID)
  eMCP251X_FilterMatch MatchID3; //!< Filter match type of the frame for ID3 (SID and/or EID)
  eMCP251X_FilterMatch MatchID4; //!< Filter match type of the frame for ID4 (SID and/or EID)
  eMCP251X_FilterMatch MatchID5; //!< Filter match type of the frame for ID5 (SID and/or EID)

  //--- Message Filter ---
  uint32_t AcceptanceID2;        //!< Message Filter Acceptance 2 SID+EID for ID2
  uint32_t AcceptanceID3;        //!< Message Filter Acceptance 3 SID+EID for ID3
  uint32_t AcceptanceID4;        //!< Message Filter Acceptance 4 SID+EID for ID4
  uint32_t AcceptanceID5;        //!< Message Filter Acceptance 5 SID+EID for ID5
  uint32_t AcceptanceMask1;      //!< Message Filter Mask 1 SID+EID (corresponding bits to AcceptanceID: '1': bit to filter ; '0' bit that do not care)
} MCP251X_Filter1;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP251X Driver API
//********************************************************************************************************************

typedef struct MCP251X MCP251X;         //! Typedef of MCP251X device object structure
typedef uint8_t TMCP251XDriverInternal; //! Alias for Driver Internal data flags

//-----------------------------------------------------------------------------

//! Device power states
typedef enum
{
  MCP251X_DEVICE_SLEEP_NOT_CONFIGURED = 0x0, //!< Device sleep mode is not configured so the device is in normal power state
  MCP251X_DEVICE_NORMAL_POWER_STATE   = 0x1, //!< Device is in normal power state
  MCP251X_DEVICE_SLEEP_STATE          = 0x2, //!< Device is in sleep power state
} eMCP251X_PowerStates;

#define MCP251X_DEV_PS_Pos         0
#define MCP251X_DEV_PS_Mask        (0x3u << MCP251X_DEV_PS_Pos)
#define MCP251X_DEV_PS_SET(value)  (((uint32_t)(value) << MCP251X_DEV_PS_Pos) & MCP251X_DEV_PS_Mask)                       //!< Set Device Power State
#define MCP251X_DEV_PS_GET(value)  (eMCP251X_PowerStates)(((uint32_t)(value) & MCP251X_DEV_PS_Mask) >> MCP251X_DEV_PS_Pos) //!< Get Device Power State

//-----------------------------------------------------------------------------

/*! @brief Function that gives the current millisecond of the system to the driver
 *
 * This function will be called when the driver needs to get current millisecond
 * @return Returns the current millisecond of the system
 */
typedef uint32_t (*GetCurrentms_Func)(void);

//-----------------------------------------------------------------------------

//! MCP251X device object structure
struct MCP251X
{
  void *UserDriverData;                  //!< Optional, can be used to store driver data or NULL

  //--- Device configuration ---
  TMCP251XDriverInternal InternalConfig; //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION

  //--- Interface driver call functions ---
  uint8_t SPIchipSelect;                 //!< This is the Chip Select index that will be set at the call of a transfer
#ifdef USE_DYNAMIC_INTERFACE
  SPI_Interface* SPI;                    //!< This is the SPI_Interface descriptor pointer that will be used to communicate with the device
#else
  SPI_Interface SPI;                     //!< This is the SPI_Interface descriptor that will be used to communicate with the device
#endif
  uint32_t SPIclockSpeed;                //!< Clock frequency of the SPI interface in Hertz

  //--- Time call function ---
  GetCurrentms_Func fnGetCurrentms;      //!< This function will be called when the driver need to get current millisecond
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define MCP251X_UNIQUE_ID  ( (((uint32_t)'M' << 0) ^ ((uint32_t)'C' << 4) ^ ((uint32_t)'P' << 8) ^ ((uint32_t)'2' << 14) ^ ((uint32_t)'5' << 18) ^ ((uint32_t)'1' << 22) ^ ((uint32_t)'X' << 26)) + __LINE__ + (sizeof(struct MCP251X) << 19) )

//-----------------------------------------------------------------------------

//! MCP251X Configuration structure
typedef struct MCP251X_Config
{
  //--- Controller clocks ---
  uint32_t XtalFreq;                   //!< Component CLKIN Xtal/Resonator frequency (max 24MHz)

  //--- CAN configuration ---
#if defined(MCP251X_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_CAN20busConfig BusConfig;        //!< CAN Bus configuration
  CAN_BitTimeStats *BitTimeStats;      //!< Point to a Bit Time stat structure (set to NULL if no statistics are necessary)
#else
  CAN_BitTimeConfig BitTimeConfig;     //!< BitTime configuration
#endif

  //--- CAN configuration ---
  bool UseOneShotMode;                 //!< One-Shot Mode

  //--- Configure buffers ---
  eMCP251X_Priority tx0Priority;       //!< Tx buffer 0 priority
  eMCP251X_Priority tx1Priority;       //!< Tx buffer 1 priority
  eMCP251X_Priority tx2Priority;       //!< Tx buffer 2 priority
  setMCP251X_RxBufferConfig rx0Conf;   //!< Rx buffer 0 configuration
  setMCP251X_RxBufferConfig rx1Conf;   //!< Rx buffer 1 configuration

  //--- Pins configuration ---
  eMCP251X_TxPinMode tx0PinMode;       //!< Set the TX0 output pin mode
  eMCP251X_TxPinMode tx1PinMode;       //!< Set the TX1 output pin mode
  eMCP251X_TxPinMode tx2PinMode;       //!< Set the TX2 output pin mode
  eMCP251X_RxPinMode rx0PinMode;       //!< Set the RX0 output pin mode
  eMCP251X_RxPinMode rx1PinMode;       //!< Set the RX1 output pin mode
  eMCP251X_CLKOUTpinMode clkoutMode;   //!< Set the CLKOUT output pin mode

  //--- Interrupts ---
  eMCP251X_InterruptEvents Interrupts; //!< Is the set of events where interrupts will be enabled. Flags can be OR'ed
} MCP251X_Config;

//********************************************************************************************************************


/*! @brief MCP251X initialization
 *
 * This function initializes the MCP251X driver and call the initialization of the interface driver (I2C).
 * Next it checks parameters and configures the MCP251X
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @param[in] *pConf Is the pointed structure of the device configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_MCP251X(MCP251X *pComp, const MCP251X_Config* pConf);

//-----------------------------------------------------------------------------


/*! @brief Send a SPI instruction to the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] instruction Is the SPI instruction to send to the device
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_SendInstruction(MCP251X *pComp, uint8_t instruction);

/*! @brief Read data from register of the MCP251X device
 *
 * This function reads data from a register of a MCP251X device
 * @param[in] *pComp Is the pointed structure of the device to read
 * @param[in] reg Is the register where data will be read (address will be incremented automatically)
 * @param[out] *data Is where the data will be stored
 * @param[in] size Is the size of the data array to read
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ReadRegister(MCP251X *pComp, eMCP251X_Registers reg, uint8_t* data, size_t size);

/*! @brief Write data to register of the MCP251X device
 *
 * This function writes data to a register of a MCP251X device
 * @param[in] *pComp Is the pointed structure of the device to modify
 * @param[in] reg Is the register where data will be written (address will be incremented automatically)
 * @param[in] *data Is the data array to write
 * @param[in] size Is the size of the data array to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_WriteRegister(MCP251X *pComp, eMCP251X_Registers reg, const uint8_t* data, size_t size);

/*! @brief Modify a register of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] reg Is the register address where data will be written
 * @param[in] data Is the data to write
 * @param[in] mask If the bit is set to '1', then the corresponding register's bit have to be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ModifyRegister(MCP251X *pComp, eMCP251X_Registers reg, uint8_t data, uint8_t mask);

//********************************************************************************************************************


/*! @brief Transmit a message object (with data) to the Buffer of the MCP251X device
 *
 * Transmit the message to the Buffer. This function uses the specific format of the component (SIDH, SIDL, EID8, EID0, DLC, DATA).
 * This function sends the message object, and asks for transmission if ask
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageObjectToSend Is the message object to send with all its data
 * @param[in] objectSize Is the size of the message object (with its data)
 * @param[in] buffIdx Is the buffer index where to put the frame
 * @param[in] andFlush Indicate if the FIFO will be flush to the CAN bus right after this message
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_TransmitMessageObject(MCP251X *pComp, const uint8_t* messageObjectToSend, uint8_t objectSize, uint8_t buffIdx, bool andFlush);

/*! @brief Transmit a message to the Buffer of the MCP251X device
 *
 * Transmit the message to the Buffer
 * This function sends the message object, and asks for transmission if ask
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageToSend Is the message to send with all the data attached with
 * @param[in] buffIdx Is the buffer index where to put the frame
 * @param[in] andFlush Indicate if the FIFO will be flush to the CAN bus right after this message
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_TransmitMessage(MCP251X *pComp, CAN_CANMessage* const messageToSend, uint8_t buffIdx, bool andFlush);


/*! @brief Receive a message object (with data) from the Buffer of the MCP251X device
 *
 * Receive the message from the Buffer. This function uses the specific format of the component (SIDH, SIDL, EID8, EID0, DLC, DATA).
 * This function gets the message object, and free the buffer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageObjectGet Is the message object retrieve with all its data
 * @param[in] objectSize Is the size of the message object (with its data)
 * @param[in] buffIdx Is the buffer index where to get the frame
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ReceiveMessageObject(MCP251X *pComp, uint8_t* messageObjectGet, uint8_t objectSize, uint8_t buffIdx);

/*! @brief Receive a message from the Buffer of the MCP251X device
 *
 * Receive a message from the Buffer
 * This function gets the message object from, and free the buffer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageGet Is the message retrieve with all the data attached with
 * @param[in] payloadSize Indicate the payload of the FIFO (0..8 bytes)
 * @param[in] buffIdx Is the buffer index where to get the frame
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ReceiveMessage(MCP251X *pComp, CAN_CANMessage* const messageGet, eMCP251X_PayloadSize payloadSize, uint8_t buffIdx);

//********************************************************************************************************************


/*! @brief Configure pins of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be configured
 * @param[in] tx0PinMode Set the TX0 pin mode
 * @param[in] tx1PinMode Set the TX1 pin mode
 * @param[in] tx2PinMode Set the TX2 pin mode
 * @param[in] rx0PinMode Set the RX0 pin mode
 * @param[in] rx0PinMode Set the RX1 pin mode
 * @param[in] clkoutMode Set the CLKOUT output pin mode
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ConfigurePins(MCP251X *pComp, eMCP251X_TxPinMode tx0PinMode, eMCP251X_TxPinMode tx1PinMode, eMCP251X_TxPinMode tx2PinMode, eMCP251X_RxPinMode rx0PinMode, eMCP251X_RxPinMode rx1PinMode, eMCP251X_CLKOUTpinMode clkoutMode);

//********************************************************************************************************************


#if defined(MCP251X_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
/*! @brief Calculate Bit Time for CAN2.0 Configuration for the MCP251X
 *
 * Calculate the best Bit Time configuration following desired bitrates for CAN-2.0
 * This function call automatically the MCP251X_CalculateBitrateStatistics() function
 * @param[in] periphClk Is the clock of the device
 * @param[in] busConf Is the bus configuration of the CAN-bus
 * @param[out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CAN20busConfig busConf, struct CAN_BitTimeConfig* const pConf);

/*! @brief Calculate Bitrate Statistics of a Bit Time configuration
 *
 * Calculate bus length, sample points, bitrates and oscillator tolerance following BitTime Configuration
 * @param[in] periphClk Is the SYSCLK of the MCP251X
 * @param[in,out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_CalculateBitrateStatistics(const uint32_t periphClk, const CAN_BitTimeConfig* const pConf);
#endif

/*! @brief Set Bit Time Configuration to the MCP251X
 *
 * Set the Nominal and Data Bit Time to registers
 * @param[in] *pComp Is the pointed structure of the MCP251X to be used
 * @param[in] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_SetBitTimeConfiguration(MCP251X *pComp, const CAN_BitTimeConfig* const pConf);

//********************************************************************************************************************


/*! @brief Get actual operation mode of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *actualMode Is where the result of the actual mode will be saved
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_GetActualOperationMode(MCP251X *pComp, eMCP251X_OperationMode* const actualMode);

/*! @brief Request operation mode change of the MCP251X device
 *
 * @note Configuration write protection will be set but not for MCP251X_INITIALISATION_MODE mode
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] newMode Is the new operational mode to set
 * @param[in] waitOperationChange Set to 'true' if the function must wait for the actual operation mode change (wait up to 7ms)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_RequestOperationMode(MCP251X *pComp, eMCP251X_OperationMode newMode, bool waitOperationChange);

/*! @brief Wait for operation mode change of the MCP251X device
 *
 * The function can wait up to 7ms. After this time, if the device doesn't change its operation mode, the function returns an #ERR__DEVICETIMEOUT error
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] askedMode Is the mode asked after a call of MCP251X_RequestOperationMode()
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_WaitOperationModeChange(MCP251X *pComp, eMCP251X_OperationMode askedMode);

/*! @brief Start the MCP251X device in CAN2.0 mode
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_StartCAN20(MCP251X *pComp)
{
  return MCP251X_RequestOperationMode(pComp, MCP251X_NORMAL_MODE, false);
}

/*! @brief Start the MCP251X device in CAN Listen-Only mode
 *
 * This function asks for a mode change to CAN Listen-Only but do not wait for its actual change because normally the device is in configuration mode and the change to CAN Listen-Only mode will be instantaneous
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_StartCANListenOnly(MCP251X *pComp)
{
  return MCP251X_RequestOperationMode(pComp, MCP251X_LISTEN_ONLY_MODE, false);
}

//********************************************************************************************************************


/*! @brief Configure the filters of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *confFilter0 Is the configuration structure of the Filter 0
 * @param[in] *confFilter1 Is the configuration structure of the Filter 1
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ConfigureFilters(MCP251X *pComp, const MCP251X_Filter0* const confFilter0, const MCP251X_Filter1* const confFilter1);

//********************************************************************************************************************


/*! @brief Sleep mode configuration of the MCP251X device
 *
 * @warning Using this function will activate the WakeUp interrupt in order to get the #MCP251X_WakeUp() function to work
 * @param[in] *pComp Is the pointed structure of the device where the sleep will be configured
 * @param[in] wakeUpFilter Set to 'true' to filter CAN bus, else set 'false'. This feature can be used to protect the module from wake-up due to short glitches on the RXCAN pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ConfigureSleepMode(MCP251X *pComp, bool wakeUpFilter);

/*! @brief Enter the MCP251X device in sleep mode
 *
 * This function puts the device in sleep mode
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_EnterSleepMode(MCP251X *pComp)
{
  pComp->InternalConfig &= ~MCP251X_DEV_PS_Mask;
  pComp->InternalConfig |= MCP251X_DEV_PS_SET(MCP251X_DEVICE_SLEEP_STATE);
  return MCP251X_RequestOperationMode(pComp, MCP251X_SLEEP_MODE, false);
}

/*! @brief Verify if the MCP251X device is in sleep mode
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *isInSleepMode Indicate if the device is in sleep mode
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_IsDeviceInSleepMode(MCP251X *pComp, bool* const isInSleepMode);

/*! @brief Manually wake up the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_WakeUp(MCP251X *pComp);

//********************************************************************************************************************


/*! @brief Flush a buffer of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] bufferIdx Is the buffer index to flush
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_FlushBuffer(MCP251X *pComp, uint8_t bufferIdx)
{
  return MCP251X_SendInstruction(pComp, MCP251X_SPI_RTSx(bufferIdx));
}

/*! @brief Flush all Tx buffers of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_FlushAllbuffers(MCP251X *pComp)
{
  return MCP251X_SendInstruction(pComp, MCP251X_SPI_RTS_ALL_TXB);
}

//********************************************************************************************************************


/*! @brief Configure interrupt of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be enabled. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ConfigureInterrupt(MCP251X *pComp, setMCP251X_InterruptEvents interruptsFlags);

/*! @brief Get interrupt events of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *interruptsFlags Is the return value of interrupt events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_GetInterruptEvents(MCP251X *pComp, setMCP251X_InterruptEvents* const interruptsFlags);

/*! @brief Get interrupt code of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *interruptsCode Is the return value of interrupt code
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_GetInterruptCode(MCP251X *pComp, eMCP251X_IntFlagCode* interruptsCode);

/*! @brief Clear interrupt events of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be cleared. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ClearInterruptEvents(MCP251X *pComp, setMCP251X_InterruptEvents interruptsFlags);

/*! @brief Get status events of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *statusFlags Is the return value of status events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_GetStatusEvents(MCP251X *pComp, setMCP251X_StatusEvents* const statusFlags);

//********************************************************************************************************************


/*! @brief Configure all Buffers of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] tx0Priority Is the priority of Tx buffer 0
 * @param[in] tx1Priority Is the priority of Tx buffer 1
 * @param[in] tx2Priority Is the priority of Tx buffer 2
 * @param[in] rx0Conf Is the configuration of Rx buffer 0
 * @param[in] rx1Conf Is the configuration of Rx buffer 1
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_ConfigureBuffers(MCP251X *pComp, eMCP251X_Priority tx0Priority, eMCP251X_Priority tx1Priority, eMCP251X_Priority tx2Priority, setMCP251X_RxBufferConfig rx0Conf, setMCP251X_RxBufferConfig rx1Conf);

//********************************************************************************************************************


/*! @brief Abort all pending transmissions of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_AbortAllTransmissions(MCP251X *pComp)
{
  return MCP251X_ModifyRegister(pComp, RegMCP251X_CANCTRL1, MCP251X_CANCTRL_ABAT, MCP251X_CANCTRL_ABAT);
}

//********************************************************************************************************************


/*! @brief Get Error Events of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorEvents Is the return value of the error events
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_GetErrorStatus(MCP251X *pComp, setMCP251X_ErrorEvents* const errorEvents)
{
#ifdef CHECK_NULL_PARAM
  if (errorEvents == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Data;
  Error = MCP251X_ReadRegister(pComp, RegMCP251X_EFLG, &Data, sizeof(Data));
  *errorEvents = (setMCP251X_ErrorEvents)Data;
  return Error;
}

/*! @brief Get Rx Error Counter of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorCounter Is the return value of the error counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_GetRxErrorCounter(MCP251X *pComp, uint8_t* const errorCounter)
{
#ifdef CHECK_NULL_PARAM
  if (errorCounter == NULL) return ERR__PARAMETER_ERROR;
#endif
  return MCP251X_ReadRegister(pComp, RegMCP251X_REC, errorCounter, sizeof(uint8_t));
}

/*! @brief Get Tx Error Counter of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorCounter Is the return value of the error counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCP251X_GetTxErrorCounter(MCP251X *pComp, uint8_t* const errorCounter)
{
#ifdef CHECK_NULL_PARAM
  if (errorCounter == NULL) return ERR__PARAMETER_ERROR;
#endif
  return MCP251X_ReadRegister(pComp, RegMCP251X_TEC, errorCounter, sizeof(uint8_t));
}

//********************************************************************************************************************


/*! @brief Get error state of a Tx Buffer of the MCP251X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] bufferIdx Is the index of the Tx Buffer
 * @param[out] *statusFlags Is the return value of error status flags
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP251X_GetTxBufferErrorStatus(MCP251X *pComp, uint8_t bufferIdx, setMCP251X_TxBufferErrorStatus *statusFlags);

//********************************************************************************************************************


/*! @brief Message ID to Object Message Identifier for Filter
 *
 * @param[out] *buffer Is where the conversion will be stored
 * @param[in] messageID Is the message ID to convert
 * @param[in] extended Indicate if the messageID is an extended ID or not
 * @param[in] isMask Indicate if the messageID is for a filter mask or not
 * @return Returns the Message ID
 */
void MCP251X_MessageIDtoFilterObjectMessageIdentifier(uint8_t* buffer, uint32_t messageID, bool extended, bool isMask);

/*! @brief Message ID to Object Message Identifier for Tx Messages
 *
 * @param[out] *txMessage Is where the conversion will be stored
 * @param[in] messageID Is the message ID to convert
 * @param[in] extended Indicate if the messageID is an extended ID or not
 * @return Returns the Message ID
 */
void MCP251X_MessageIDtoTxObjectMessageIdentifier(MCP251X_CAN_TxMessage* txMessage, uint32_t messageID, bool extended);

/*! @brief Object Message Identifier for Rx Messages to Message ID
 *
 * @param[in] rxMessage Is the Rx message object to convert
 * @return Returns the Message ID
 */
uint32_t MCP251X_RxObjectMessageIdentifierToMessageID(MCP251X_CAN_RxMessage* rxMessage);

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCP251X_H_INC */