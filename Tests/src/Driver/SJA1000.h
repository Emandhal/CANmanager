/*!*****************************************************************************
 * @file    SJA1000.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    14/07/2023
 * @brief   SJA1000 driver
 * @details Stand-alone CAN controller
 * Follow datasheet SJA1000 Rev 3.0 (January 2004)
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
#ifndef SJA1000_H_INC
#define SJA1000_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include "CAN_common.h"
#include "ErrorsDef.h"
#include "GPIO_Interface.h"
//-----------------------------------------------------------------------------

#if !defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_USE_PELICAN)
#  define SJA1000_USE_PELICAN // Use PeliCAN only by default
#endif
#if defined(SJA1000_USE_BASICCAN) && defined(SJA1000_USE_PELICAN)
#  define SJA1000_BOTH_CAN_DEFINED
#endif

//-----------------------------------------------------------------------------

#if !defined(SJA1000_USE_INTELMODE) && !defined(SJA1000_USE_MOTOROLAMODE)
#  define SJA1000_USE_INTELMODE // Use Intel mode only by default
#endif
#if defined(SJA1000_USE_INTELMODE) && defined(SJA1000_USE_MOTOROLAMODE)
#  define SJA1000_BOTH_MODE_DEFINED
#endif

//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#  define __SJA1000_PACKED__
#  define SJA1000_PACKITEM             __pragma(pack(push, 1))
#  define SJA1000_UNPACKITEM           __pragma(pack(pop))
#  define SJA1000_PACKENUM(name,type)  typedef enum name : type
#  define SJA1000_UNPACKENUM(name)     name
#else
#  define __SJA1000_PACKED__           __attribute__((packed))
#  define SJA1000_PACKITEM
#  define SJA1000_UNPACKITEM
#  define SJA1000_PACKENUM(name,type)  typedef enum __SJA1000_PACKED__
#  define SJA1000_UNPACKENUM(name)     name
#endif

//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define SJA1000_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// SJA1000 limits definitions
//********************************************************************************************************************

// Frequencies and bitrate limits for SJA1000
#define SJA1000_XTALFREQ_MAX        ( 24000000u ) //!< Max Xtal frequency
#define SJA1000_PERIPHERAL_CLK_MAX  SJA1000_XTALFREQ_MAX //!< Max Peripheral frequency

#define SJA1000_NOMBITRATE_MIN      (        0u ) //!< Min Nominal bitrate
#define SJA1000_NOMBITRATE_MAX      (  1000000u ) //!< Max Nominal bitrate

#define SJA1000_SYSCLOCK_DIV        ( 1 ) //!< No specific divider of system clock inside the SJA1000

//-----------------------------------------------------------------------------

// Limits Bit Rate configuration range for SJA1000
#define SJA1000_tTXDtRXD_MAX  ( 255 ) //!< tTXD-RXD is the propagation delay of the transceiver, a maximum 255ns according to ISO 11898-1:2015
#define SJA1000_tBUS_CONV     (   5 ) //!< TBUS is the delay on the CAN bus, which is approximately 5ns/m

#define SJA1000_NBRP_MIN      (  1 ) //!< Min NBRP
#define SJA1000_NBRP_MAX      ( 64 ) //!< Max NBRP
#define SJA1000_NSYNC         (  1 ) //!< NSYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define SJA1000_NPRSEG_MIN    (  0 ) //!< Min NPRSEG
#define SJA1000_NPRSEG_MAX    (  8 ) //!< Max NPRSEG
#define SJA1000_NTSEG1_MIN    (  1 ) //!< Min NTSEG1
#define SJA1000_NTSEG1_MAX    (  8 ) //!< Max NTSEG1
#define SJA1000_NTSEG2_MIN    (  1 ) //!< Min NTSEG2
#define SJA1000_NTSEG2_MAX    (  8 ) //!< Max NTSEG2
#define SJA1000_NSJW_MIN      (  1 ) //!< Min NSJW
#define SJA1000_NSJW_MAX      (  4 ) //!< Max NSJW
#define SJA1000_NTQBIT_MIN    ( SJA1000_NSYNC + SJA1000_NPRSEG_MIN + SJA1000_NTSEG1_MIN + SJA1000_NTSEG2_MIN ) //!< Min NTQ per Bit (1-bit SYNC + 0-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define SJA1000_NTQBIT_MAX    ( SJA1000_NSYNC + SJA1000_NPRSEG_MAX + SJA1000_NTSEG1_MAX + SJA1000_NTSEG2_MAX ) //!< Max NTQ per Bit (25-bits)

#define SJA1000_MAX_OSC_TOLERANCE  ( 1.58f ) //!< The CAN specification indicates that the worst case oscillator tolerance is 1.58% and is only suitable for low bit rates (125kb/s or less)
#define SJA1000_UINT_MAX_OSC_TOL   ( (int32_t)(SJA1000_MAX_OSC_TOLERANCE * 100.0f) )

//-----------------------------------------------------------------------------

// Memory mapping definitions for SJA1000
#define SJA1000_RAM_ADDR  ( 32u ) //!< RAM Memory base address
#define SJA1000_RAM_SIZE  ( 80u ) //!< RAM (FIFO) size

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Tx, Rx Messages Objects
//********************************************************************************************************************

//! Data Length Size for the CAN message
typedef enum
{
  SJA1000_DLC_0BYTE   = 0b0000, //!< The DLC is 0 data byte
  SJA1000_DLC_1BYTE   = 0b0001, //!< The DLC is 1 data byte
  SJA1000_DLC_2BYTE   = 0b0010, //!< The DLC is 2 data bytes
  SJA1000_DLC_3BYTE   = 0b0011, //!< The DLC is 3 data bytes
  SJA1000_DLC_4BYTE   = 0b0100, //!< The DLC is 4 data bytes
  SJA1000_DLC_5BYTE   = 0b0101, //!< The DLC is 5 data bytes
  SJA1000_DLC_6BYTE   = 0b0110, //!< The DLC is 6 data bytes
  SJA1000_DLC_7BYTE   = 0b0111, //!< The DLC is 7 data bytes
  SJA1000_DLC_8BYTE   = 0b1000, //!< The DLC is 8 data bytes
  SJA1000_DLC_COUNT,            // Keep last
  SJA1000_PAYLOAD_MIN = 8,
  SJA1000_PAYLOAD_MAX = 8,
} eSJA1000_DataLength;

static const uint8_t SJA1000_DLC_TO_VALUE[CAN_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8};

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Descriptor (DES) for BasicCAN
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CAN_MessageDescriptor
{
  uint16_t DES;
  uint8_t Bytes[sizeof(uint16_t)];
  struct
  {
    uint16_t DLC:  4; //!< 0- 3 - Data Length Code
    uint16_t RTR:  1; //!< 4    - Remote transmission request
    uint16_t SID: 11; //!< 5-15 - Standard Message Identifier
  };
} SJA1000_CAN_MessageDescriptor;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CAN_MessageDescriptor, 2);

#define SJA1000_BCAN_DES_DLC_Pos         0
#define SJA1000_BCAN_DES_DLC_Mask        (0xFu << SJA1000_BCAN_DES_DLC_Pos)
#define SJA1000_BCAN_DES_DLC_SET(value)  (((uint8_t)(value) << SJA1000_BCAN_DES_DLC_Pos) & SJA1000_BCAN_DES_DLC_Mask) //!< Set Data Length Code
#define SJA1000_BCAN_DES_DLC_GET(value)  (((uint8_t)(value) & SJA1000_BCAN_DES_DLC_Mask) >> SJA1000_BCAN_DES_DLC_Pos) //!< Get Data Length Code
#define SJA1000_BCAN_DES_REMOTE_FRAME    (0x1u << 4) //!< Transmit remote frame
#define SJA1000_BCAN_DES_DATA_FRAME      (0x0u << 4) //!< Transmit data frame
#define SJA1000_BCAN_DES_SID_Pos         5
#define SJA1000_BCAN_DES_SID_Mask        (0x7FFu << SJA1000_BCAN_DES_SID_Pos)
#define SJA1000_BCAN_DES_SID_SET(value)  (((uint32_t)(value) << SJA1000_BCAN_DES_SID_Pos) & SJA1000_BCAN_DES_SID_Mask) //!< Set Standard Identifier filter
#define SJA1000_BCAN_DES_SID_GET(value)  (((uint32_t)(value) & SJA1000_BCAN_DES_SID_Mask) >> SJA1000_BCAN_DES_SID_Pos) //!< Get Standard Identifier filter

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Information (INF) for PeliCAN
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CAN_MessageInformation
{
  uint8_t INF;
  struct
  {
    uint8_t DLC: 4; //!< 0-3 - Data Length Code
    uint8_t    : 2; //!< 4-5
    uint8_t RTR: 1; //!< 6   - Remote transmission request
    uint8_t FF : 1; //!< 7   - Frame format: '1' = 29-bit extended identifier ; '0' = 11-bit standard identifier
  };
} SJA1000_CAN_MessageInformation;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CAN_MessageInformation, 1);

#define SJA1000_PCAN_INF_DLC_Pos         0
#define SJA1000_PCAN_INF_DLC_Mask        (0xFu << SJA1000_PCAN_INF_DLC_Pos)
#define SJA1000_PCAN_INF_DLC_SET(value)  (((uint8_t)(value) << SJA1000_PCAN_INF_DLC_Pos) & SJA1000_PCAN_INF_DLC_Mask) //!< Set Data Length Code
#define SJA1000_PCAN_INF_DLC_GET(value)  (((uint8_t)(value) & SJA1000_PCAN_INF_DLC_Mask) >> SJA1000_PCAN_INF_DLC_Pos) //!< Get Data Length Code
#define SJA1000_PCAN_INF_REMOTE_FRAME    (0x1u << 6) //!< Transmit remote frame
#define SJA1000_PCAN_INF_DATA_FRAME      (0x0u << 6) //!< Transmit data frame
#define SJA1000_PCAN_INF_EXTENDED_ID     (0x1u << 7) //!< 29-bit extended identifier
#define SJA1000_PCAN_INF_STANDARD_ID     (0x0u << 7) //!< 11-bit standard identifier

//-----------------------------------------------------------------------------

//! CAN Transmit/Receive Message Identifier (ID) for PeliCAN
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CAN_MessageIdentifier
{
  uint32_t ID;
  uint8_t Bytes[sizeof(uint32_t)];
  union
  {
    struct
    {
      uint32_t SID: 11; //!<  0-10 - Standard Message Identifier
      uint32_t    : 21; //!< 11-31
    } Standard;
    struct
    {
      uint32_t SIDEID: 29; //!<  0-28 - Extended Message Identifier
      uint32_t       :  3; //!< 29-31
    } Extended;
  };
} SJA1000_CAN_MessageIdentifier;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CAN_MessageIdentifier, 4);

#define SJA1000_PCAN_ID_SIDEID_Pos         0
#define SJA1000_PCAN_ID_SIDEID_Mask        (0x1FFFFFFFu << SJA1000_CAN_ID_SIDEID_Pos)
#define SJA1000_PCAN_ID_SIDEID_SET(value)  (((uint32_t)(value) << SJA1000_CAN_ID_SIDEID_Pos) & SJA1000_CAN_ID_SIDEID_Mask) //!< Set Extended Identifier filter
#define SJA1000_PCAN_ID_SIDEID_GET(value)  (((uint32_t)(value) & SJA1000_CAN_ID_SIDEID_Mask) >> SJA1000_CAN_ID_SIDEID_Pos) //!< Get Extended Identifier filter
#define SJA1000_PCAN_ID_SID_Pos            0
#define SJA1000_PCAN_ID_SID_Mask           (0x7FFu << SJA1000_CAN_MSGT0_SID_Pos)
#define SJA1000_PCAN_ID_SID_SET(value)     (((uint32_t)(value) << SJA1000_CAN_MSGT0_SID_Pos) & SJA1000_CAN_MSGT0_SID_Mask) //!< Set Standard Identifier filter
#define SJA1000_PCAN_ID_SID_GET(value)     (((uint32_t)(value) & SJA1000_CAN_MSGT0_SID_Mask) >> SJA1000_CAN_MSGT0_SID_Pos) //!< Get Standard Identifier filter

//-----------------------------------------------------------------------------

#define SJA1000_CAN_TXMSG_DES1  0
#define SJA1000_CAN_TXMSG_DES2  1
#define SJA1000_CAN_TXMSG_INF   0
#define SJA1000_CAN_TXMSG_ID1   1
#define SJA1000_CAN_TXMSG_ID2   2
#define SJA1000_CAN_TXMSG_ID3   3
#define SJA1000_CAN_TXMSG_ID4   4

//! Transmit Message Object Register (TX Buffer)
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CAN_TxMessage
{
  uint8_t Bytes[5];
  struct
  {
    SJA1000_CAN_MessageInformation INF; //!< PeliCAN Transmit Message Information (INF)
    SJA1000_CAN_MessageIdentifier  ID;  //!< PeliCAN Transmit Message Identifier (ID)
  };
  SJA1000_CAN_MessageDescriptor DES;    //!< BasicCAN Transmit Message Descriptor (DES)
} SJA1000_CAN_TxMessage;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CAN_TxMessage, 5);

//-----------------------------------------------------------------------------

#define SJA1000_CAN_TX_MESSAGE_HEADER_SIZE  ( sizeof(SJA1000_CAN_TxMessage) )
#define SJA1000_CAN_TX_MESSAGE_SIZE_MAX     ( sizeof(SJA1000_CAN_TxMessage) + SJA1000_PAYLOAD_MAX )

//-----------------------------------------------------------------------------

#define SJA1000_CAN_RXMSG_DES1  0
#define SJA1000_CAN_RXMSG_DES2  1
#define SJA1000_CAN_RXMSG_INF   0
#define SJA1000_CAN_RXMSG_ID1   1
#define SJA1000_CAN_RXMSG_ID2   2
#define SJA1000_CAN_RXMSG_ID3   3
#define SJA1000_CAN_RXMSG_ID4   4

//! Receive Message Object Register (RX FIFO)
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CAN_RxMessage
{
  uint8_t Bytes[5];
  struct
  {
    SJA1000_CAN_MessageInformation INF; //!< PeliCAN Receive Message Information (INF)
    SJA1000_CAN_MessageIdentifier  ID;  //!< PeliCAN Receive Message Identifier (ID)
  };
  SJA1000_CAN_MessageDescriptor DES;    //!< BasicCAN Receive Message Descriptor (DES)
} SJA1000_CAN_RxMessage;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CAN_RxMessage, 5);

//-----------------------------------------------------------------------------

#define SJA1000_CAN_RX_MESSAGE_HEADER_SIZE  ( sizeof(SJA1000_CAN_RxMessage) )
#define SJA1000_CAN_RX_MESSAGE_SIZE_MAX     ( sizeof(SJA1000_CAN_RxMessage) + SJA1000_PAYLOAD_MAX )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Register list
//********************************************************************************************************************

//! SJA1000 registers list
typedef enum
{
  // BasicCAN registers
  RegSJA1000_BCAN_CONTROL         =  0, //!< (Offset: 00) Control register
  RegSJA1000_BCAN_COMMAND         =  1, //!< (Offset: 01) Command register
  RegSJA1000_BCAN_STATUS          =  2, //!< (Offset: 02) Status register
  RegSJA1000_BCAN_INTERRUPT       =  3, //!< (Offset: 03) Interrupt register
  RegSJA1000_BCAN_ACCEPTANCE_CODE =  4, //!< (Offset: 04) Acceptance code register
  RegSJA1000_BCAN_ACCEPTANCE_MASK =  5, //!< (Offset: 05) Acceptance mask register
  RegSJA1000_BCAN_BUS_TIMING_0    =  6, //!< (Offset: 06) Bus timing 0 register
  RegSJA1000_BCAN_BUS_TIMING_1    =  7, //!< (Offset: 07) Bus timing 1 register
  RegSJA1000_BCAN_OUTPUT_CONTROL  =  8, //!< (Offset: 08) Output control register
  RegSJA1000_BCAN_TEST            =  9, //!< (Offset: 09) Test register
  RegSJA1000_BCAN_TX_BUFFER_ID0   = 10, //!< (Offset: 10) Transmit buffer: identifier (10 to 3) register
  RegSJA1000_BCAN_TX_BUFFER_ID1   = 11, //!< (Offset: 11) Transmit buffer: identifier (2 to 0), RTR and DLC register
  RegSJA1000_BCAN_TX_BUFFER_BYTE1 = 12, //!< (Offset: 12) Transmit buffer: data byte 1 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE2 = 13, //!< (Offset: 13) Transmit buffer: data byte 2 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE3 = 14, //!< (Offset: 14) Transmit buffer: data byte 3 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE4 = 15, //!< (Offset: 15) Transmit buffer: data byte 4 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE5 = 16, //!< (Offset: 16) Transmit buffer: data byte 5 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE6 = 17, //!< (Offset: 17) Transmit buffer: data byte 6 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE7 = 18, //!< (Offset: 18) Transmit buffer: data byte 7 register
  RegSJA1000_BCAN_TX_BUFFER_BYTE8 = 19, //!< (Offset: 19) Transmit buffer: data byte 8 register
  RegSJA1000_BCAN_RX_BUFFER_ID0   = 20, //!< (Offset: 20) Receive buffer: identifier (10 to 3) register
  RegSJA1000_BCAN_RX_BUFFER_ID1   = 21, //!< (Offset: 21) Receive buffer: identifier (2 to 0), RTR and DLC register
  RegSJA1000_BCAN_RX_BUFFER_BYTE1 = 22, //!< (Offset: 22) Receive buffer: data byte 1 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE2 = 23, //!< (Offset: 23) Receive buffer: data byte 2 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE3 = 24, //!< (Offset: 24) Receive buffer: data byte 3 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE4 = 25, //!< (Offset: 25) Receive buffer: data byte 4 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE5 = 26, //!< (Offset: 26) Receive buffer: data byte 5 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE6 = 27, //!< (Offset: 27) Receive buffer: data byte 6 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE7 = 28, //!< (Offset: 28) Receive buffer: data byte 7 register
  RegSJA1000_BCAN_RX_BUFFER_BYTE8 = 29, //!< (Offset: 29) Receive buffer: data byte 8 register
                                        //   (Offset: 30) Reserved
  RegSJA1000_BCAN_CLOCK_DIVIDER   = 31, //!< (Offset: 31) Clock divider register

  // PeliCAN registers
  RegSJA1000_PCAN_MODE              =   0, //!< (Offset:  00) Mode register
  RegSJA1000_PCAN_COMMAND           =   1, //!< (Offset:  01) Command register
  RegSJA1000_PCAN_STATUS            =   2, //!< (Offset:  02) Status register
  RegSJA1000_PCAN_INTERRUPT_STATUS  =   3, //!< (Offset:  03) Interrupt status register
  RegSJA1000_PCAN_INTERRUPT_ENABLE  =   4, //!< (Offset:  04) Interrupt enable register
                                           //   (Offset:  05) Reserved
  RegSJA1000_PCAN_BUS_TIMING_0      =   6, //!< (Offset:  06) Bus timing 0 register
  RegSJA1000_PCAN_BUS_TIMING_1      =   7, //!< (Offset:  07) Bus timing 1 register
  RegSJA1000_PCAN_OUTPUT_CONTROL    =   8, //!< (Offset:  08) Output control register
  RegSJA1000_PCAN_TEST              =   9, //!< (Offset:  09) Test register
                                           //   (Offset:  10) Reserved
  RegSJA1000_PCAN_ARBITRATION_LOST  =  11, //!< (Offset:  11) Arbitration lost capture register
  RegSJA1000_PCAN_ERROR_CODE        =  12, //!< (Offset:  12) Error code capture register
  RegSJA1000_PCAN_ERROR_WARNING     =  13, //!< (Offset:  13) Error warning limit register
  RegSJA1000_PCAN_RX_ERROR_COUNTER  =  14, //!< (Offset:  14) Rx error counter register
  RegSJA1000_PCAN_TX_ERROR_COUNTER  =  15, //!< (Offset:  15) Tx error counter register
  RegSJA1000_PCAN_RX_BUFFER_START   =  16, //!< (Offset:  16) Rx buffer start register
  RegSJA1000_PCAN_TX_BUFFER_START   =  16, //!< (Offset:  16) Tx buffer start register
  RegSJA1000_PCAN_ACCEPTANCE_CODE0  =  16, //!< (Offset:  16) Reset mode: Acceptance code 0 register
  RegSJA1000_PCAN_ACCEPTANCE_CODE1  =  17, //!< (Offset:  17) Reset mode: Acceptance code 1 register
  RegSJA1000_PCAN_ACCEPTANCE_CODE2  =  18, //!< (Offset:  18) Reset mode: Acceptance code 2 register
  RegSJA1000_PCAN_ACCEPTANCE_CODE3  =  19, //!< (Offset:  19) Reset mode: Acceptance code 3 register
  RegSJA1000_PCAN_ACCEPTANCE_MASK0  =  20, //!< (Offset:  20) Reset mode: Acceptance mask 0 register
  RegSJA1000_PCAN_ACCEPTANCE_MASK1  =  21, //!< (Offset:  21) Reset mode: Acceptance mask 1 register
  RegSJA1000_PCAN_ACCEPTANCE_MASK2  =  22, //!< (Offset:  22) Reset mode: Acceptance mask 2 register
  RegSJA1000_PCAN_ACCEPTANCE_MASK3  =  23, //!< (Offset:  23) Reset mode: Acceptance mask 3 register
  RegSJA1000_PCAN_RX_MSG_COUNTER    =  29, //!< (Offset:  29) Rx message counter register
  RegSJA1000_PCAN_RX_START_ADDR     =  30, //!< (Offset:  30) Rx buffer start address register
  RegSJA1000_PCAN_CLOCK_DIVIDER     =  31, //!< (Offset:  31) Clock divider register
  RegSJA1000_PCAN_RAM_RX_FIFO_START =  32, //!< (Offset:  32) Internal RAM address (Rx FIFO)
  RegSJA1000_PCAN_RAM_TX_BUF_START  =  96, //!< (Offset:  96) Internal RAM address (Tx Buffer)
  RegSJA1000_PCAN_RAM_FIFO_FREE0    = 109, //!< (Offset: 109) Internal RAM address free 0
  RegSJA1000_PCAN_RAM_FIFO_FREE1    = 110, //!< (Offset: 110) Internal RAM address free 1
  RegSJA1000_PCAN_RAM_FIFO_FREE2    = 111, //!< (Offset: 111) Internal RAM address free 2

  // Commons registers
  RegSJA1000_CAN_CONTROL        =  0, //!< (Offset: 00) Control register
  RegSJA1000_CAN_COMMAND        =  1, //!< (Offset: 01) Command register
  RegSJA1000_CAN_STATUS         =  2, //!< (Offset: 02) Status register
  RegSJA1000_CAN_BUS_TIMING_0   =  6, //!< (Offset: 06) Bus timing 0 register
  RegSJA1000_CAN_BUS_TIMING_1   =  7, //!< (Offset: 07) Bus timing 1 register
  RegSJA1000_CAN_OUTPUT_CONTROL =  8, //!< (Offset: 08) Output control register
  RegSJA1000_CAN_CLOCK_DIVIDER  = 31, //!< (Offset: 31) Clock divider register
} eSJA1000_Registers;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Specific BasicCAN Controller Registers
//********************************************************************************************************************

/*! @brief SJA1000-BasicCAN Control Register (Read/Write, Address: 0, Initial value: 0b0X1XXXX1)
 * @details The contents of the control register are used to change the behaviour of the CAN controller. Bits may be set or reset by the attached microcontroller which uses the control register as a read/write memory
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BCAN_CTRL_Register
{
  uint8_t Control;
  struct
  {
    uint8_t RR : 1; /*!< 0 - Reset Request: '1' = detection of a reset request results in aborting the current transmission/reception of a message and entering the reset mode ; '1-to-0' = transition of the reset request bit, the SJA1000 returns to the operating mode
                     *       During a hardware reset or when the bus status bit is set to logic 1 (bus-off), the reset request bit is set to logic 1 (present). If this bit is accessed by software, a value change will become visible and takes effect first with the next positive edge of the internal clock which operates with 1/2 of the external oscillator frequency.
                     *       During an external reset the microcontroller cannot set the reset request bit to logic 0 (absent). Therefore, after having set the reset request bit to logic 0, the microcontroller must check this bit to ensure that the external reset pin is not being held LOW.
                     *       Changes of the reset request bit are synchronized with the internal divided clock. Reading the reset request bit reflects the synchronized status. After the reset request bit is set to logic 0 the SJA1000 will wait for:
                     *        - One occurrence of bus-free signal (11 recessive bits), if the preceding reset request has been caused by a hardware reset or a CPU-initiated reset
                     *        - 128 occurrences of bus-free, if the preceding reset request has been caused by a CAN controller initiated bus-off, before re-entering the bus-on mode; it should be noted that several registers are modified if the reset request bit was set
                     */
    uint8_t RIE: 1; //!< 1 - Receive Interrupt Enable: '1' = enabled; when a message has been received without errors, the SJA1000 transmits a receive interrupt signal to the microcontroller ; '0' = disabled; the microcontroller receives no transmit interrupt signal from the SJA1000
    uint8_t TIE: 1; //!< 2 - Transmit Interrupt Enable: '1' = enabled; when a message has been successfully transmitted or the transmit buffer is accessible again, (e.g. after an abort transmission command) the SJA1000 transmits a transmit interrupt signal to the microcontroller ; '0' = disabled; the microcontroller receives no transmit interrupt signal from the SJA1000
    uint8_t EIE: 1; //!< 3 - Error Interrupt Enable: '1' = enabled; if the error or bus status change, the microcontroller receives an error interrupt signal (see also status register) ; '0' = disabled; the microcontroller receives no error interrupt signal from the SJA1000
    uint8_t OIE: 1; //!< 4 - Overrun Interrupt Enable: '1' = enabled; if the data overrun bit is set, the microcontroller receives an overrun interrupt signal (see also status register) ; '0' = disabled; the microcontroller receives no overrun interrupt signal from the SJA1000
    uint8_t    : 1; //!< 5 - Reading this bit will always reflect a logic 1
    uint8_t    : 1; //!< 6 - In the PCA82C200 this bit was used to select the synchronization mode. Because this mode is not longer implemented, setting this bit has no influence on the microcontroller. Due to software compatibility setting this bit is allowed. This bit will not change after hardware or software reset. In addition the value written by users software is reflected
    uint8_t    : 1; //!< 7 - Any write access to the control register has to set this bit to logic 0 (reset value is logic 0)
  } Bits;
} SJA1000_BCAN_CTRL_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BCAN_CTRL_Register, 1);

#define SJA1000_BCAN_CTRL_RESET_MODE   (0x1u << 0) //!< Reset Request
#define SJA1000_BCAN_CTRL_RIE_ENABLE   (0x1u << 1) //!< Enable the Receive Interrupt
#define SJA1000_BCAN_CTRL_RIE_DISABLE  (0x0u << 1) //!< Disable the Receive Interrupt
#define SJA1000_BCAN_CTRL_TIE_ENABLE   (0x1u << 2) //!< Enable the Transmit Interrupt
#define SJA1000_BCAN_CTRL_TIE_DISABLE  (0x0u << 2) //!< Disable the Transmit Interrupt
#define SJA1000_BCAN_CTRL_EIE_ENABLE   (0x1u << 3) //!< Enable the Error Interrupt
#define SJA1000_BCAN_CTRL_EIE_DISABLE  (0x0u << 3) //!< Disable the Error Interrupt
#define SJA1000_BCAN_CTRL_OIE_ENABLE   (0x1u << 4) //!< Enable the Overrun Interrupt
#define SJA1000_BCAN_CTRL_OIE_DISABLE  (0x0u << 4) //!< Disable the Overrun Interrupt
#define SJA1000_BCAN_CTRL_BIT5         (0x1u << 5) //!< Bit 5 always '1'

#define SJA1000_BCAN_IER_ALL_INT       ( SJA1000_BCAN_CTRL_RIE_ENABLE | SJA1000_BCAN_CTRL_TIE_ENABLE | SJA1000_BCAN_CTRL_EIE_ENABLE | SJA1000_BCAN_CTRL_OIE_ENABLE )

//-----------------------------------------------------------------------------

/*! @brief SJA1000-BasicCAN Command Register (Write, Address: 1, Initial value: 0b11111111)
 * @details A command bit initiates an action within the transfer layer of the SJA1000. The command register appears to the microcontroller as a write only memory.
 * If a read access is performed to this address the byte '11111111' is returned. Between two commands at least one internal clock cycle is needed to process.
 * The internal clock is divided by two from the external oscillator frequency
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BCAN_CMD_Register
{
  uint8_t Command;
  struct
  {
    uint8_t TR : 1; /*!< 0 - Transmission Request: '1' = present; a message will be transmitted ; '0' = absent; no action
                     *       The SJA1000 will enter sleep mode if the sleep bit is set to logic 1 (sleep); there is no bus activity and no interrupt is pending.
                     *       Setting of GTS with at least one of the previously mentioned exceptions valid will result in a wake-up interrupt.
                     *       After sleep mode is set, the CLKOUT signal continues until at least 15 bit times have passed, to allow a host microcontroller clocked via this signal to enter its own standby mode before the CLKOUT goes LOW.
                     *       The SJA1000 will wake up when one of the three previously mentioned conditions is negated: after 'Go To Sleep' is set LOW (wake-up), there is bus activity or INT is driven LOW (active).
                     *       On wake-up, the oscillator is started and a wake-up interrupt is generated. A sleeping SJA1000 which wakes up due to bus activity will not be able to receive this message until it detects 11 consecutive recessive bits (bus-free sequence).
                     *       It should be noted that setting of GTS is not possible in reset mode. After clearing of reset request, setting of GTS is possible first, when bus-free is detected again
                     */
    uint8_t AT : 1; /*!< 1 - Abort Transmission: '1' = present; if not already in progress, a pending transmission request is cancelled ; '0' = absent; no action
                     *       This command bit is used to clear the data overrun condition indicated by the data overrun status bit. As long as the data overrun status bit is set no further data overrun interrupt is generated.
                     *       It is allowed to give the clear data overrun command at the same time as a release receive buffer command
                     */
    uint8_t RRB: 1; /*!< 2 - Release Receive Buffer: '1' = released; the receive buffer, representing the message memory space in the RXFIFO is released ; '0' = no action
                     *       After reading the contents of the receive buffer, the microcontroller can release this memory space of the RXFIFO by setting the release receive buffer bit to logic 1.
                     *       This may result in another message becoming immediately available within the receive buffer. This event will force another receive interrupt, if enabled.
                     *       If there is no other message available no further receive interrupt is generated and the receive buffer status bit is cleared
                     */
    uint8_t CDO: 1; /*!< 3 - Clear Data Overrun: '1' = clear; data overrun status bit is cleared ; '0' = no action
                     *       The abort transmission bit is used when the CPU requires the suspension of the previously requested transmission, e.g. to transmit a more urgent message before.
                     *       A transmission already in progress is not stopped. In order to see if the original message had been either transmitted successfully or aborted, the transmission complete status bit should be checked.
                     *       This should be done after the transmit buffer status bit has been set to logic 1 (released) or a transmit interrupt has been generated
                     */
    uint8_t GTS: 1; /*!< 4 - Go To Sleep: '1' = sleep; the SJA1000 enters sleep mode if no CAN interrupt is pending and there is no bus activity ; '0' = wake up; SJA1000 operates normal
                     *       If the transmission request was set to logic 1 in a previous command, it cannot be cancelled by setting the transmission request bit to logic 0.
                     *       The requested transmission may be cancelled by setting the abort transmission bit to logic 1
                     */
    uint8_t    : 3; //!< 5-7
  } Bits;
} SJA1000_BCAN_CMD_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BCAN_CMD_Register, 1);

#define SJA1000_COMMAND_TX_REQUEST          (0x1u << 0) //!< Transmission Request
#define SJA1000_COMMAND_ABORT_TX            (0x1u << 1) //!< Abort Transmission
#define SJA1000_COMMAND_RELEASE_RX          (0x1u << 2) //!< Release Receive Buffer
#define SJA1000_COMMAND_CLEAR_DATA_OVERRUN  (0x1u << 3) //!< Clear Data Overrun
#define SJA1000_COMMAND_GO_TO_SLEEP         (0x1u << 4) //!< Go To Sleep
#define SJA1000_COMMAND_WAKE_UP             (0x0u << 4) //!< Wake up

//-----------------------------------------------------------------------------

/*! @brief SJA1000-BasicCAN Status Register (Read/Write, Address: 2, Initial value: 0b00001100)
 * @details The content of the status register reflects the status of the SJA1000. The status register appears to the microcontroller as a read only memory
 * If both the receive status and the transmit status bits are logic 0 (idle) the CAN-bus is idle
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BCAN_STATUS_Register
{
  uint8_t Status;
  struct
  {
    uint8_t RBS: 1; /*!< 0 - Receive Buffer Status: '1' = full; one or more messages are available in the RXFIFO ; '0' = empty; no message is available
                     *       After reading a message stored in the RXFIFO and releasing this memory space with the command release receive buffer, this bit is cleared.
                     *       If there is another message available within the FIFO this bit is set again with the next bit quantum (tscl)
                     */
    uint8_t DOS: 1; /*!< 1 - Data Overrun Status: '1' = overrun; a message was lost because there was not enough space for that message in the RXFIFO ; '0' = absent; no data overrun has occurred since the last clear data overrun command was given
                     *       When a message that shall be received has passed the acceptance filter successfully (i.e. earliest after arbitration field), the CAN controller needs space in the RXFIFO to store the message descriptor.
                     *       Accordingly there must be enough space for each data byte which has been received. If there is not enough space to store the message, that message will be dropped and the data overrun condition will be indicated to the CPU only, if this received message has no errors until the last but one bit of end of frame (message becomes valid)
                     */
    uint8_t TBS: 1; /*!< 2 - Transmit Buffer Status: '1' = released; the CPU may write a message into the transmit buffer ; '0' = locked; the CPU cannot access the transmit buffer; a message is waiting for transmission or is already in process
                     *       If the CPU tries to write to the transmit buffer when the transmit buffer status bit is at logic 0 (locked), the written byte will not be accepted and will be lost without being indicated
                     */
    uint8_t TCS: 1; /*!< 3 - Transmission Complete Status: '1' = complete; the last requested transmission has been successfully completed ; '0' = incomplete; the previously requested transmission is not yet completed
                     *       The transmission complete status bit is set to logic 0 (incomplete) whenever the transmission request bit is set to logic 1. The transmission complete status bit will remain at logic 0 (incomplete) until a message is transmitted successfully
                     */
    uint8_t RS : 1; //!< 4 - Receive Status: '1' = receive; the SJA1000 is receiving a message ; '0' = idle; no receive message is in progress
    uint8_t TS : 1; //!< 5 - Transmit Status: '1' = transmit; the SJA1000 is transmitting a message ; '0' = idle; no transmit message is in progress
    uint8_t ES : 1; /*!< 6 - Error Status: '1' = error; at least one of the error counters has reached or exceeded the CPU warning limit ; '0' = ok; both error counters are below the warning limit
                     *       Errors detected during reception or transmission will affect the error counters according to the CAN 2.0B protocol specification. The error status bit is set when at least one of the error counters has reached or exceeded the CPU warning limit of 96. An error interrupt is generated, if enabled
                     */
    uint8_t BS : 1; /*!< 7 - Bus Status: '1' = bus-off; the SJA1000 is not involved in bus activities ; '0' = bus-on; the SJA1000 is involved in bus activities
                     *       When the transmit error counter exceeds the limit of 255 [the bus status bit is set to logic 1 (bus-off)] the CAN controller will set the reset request bit to logic 1 (present) and an error interrupt is generated, if enabled.
                     *       It will stay in this mode until the CPU clears the reset request bit. Once this is completed the CAN controller will wait the minimum protocol-defined time (128 occurrences of the bus-free signal).
                     *       After that the bus status bit is cleared (bus-on), the error status bit is set to logic 0 (ok), the error counters are reset and an error interrupt is generated, if enabled
                     */
  } Bits;
} SJA1000_BCAN_STATUS_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BCAN_STATUS_Register, 1);

#define SJA1000_STATUS_RX_BUFFER_FULL      (0x1u << 0) //!< Receive Buffer Full
#define SJA1000_STATUS_RX_BUFFER_EMPTY     (0x0u << 0) //!< Receive Buffer Empty
#define SJA1000_STATUS_DATA_OVERRUN        (0x1u << 1) //!< Receive Buffer Data Overrun
#define SJA1000_STATUS_TX_BUFFER_RELEASED  (0x1u << 2) //!< Transmit Buffer Released
#define SJA1000_STATUS_TX_BUFFER_LOCKED    (0x0u << 2) //!< Transmit Buffer Locked
#define SJA1000_STATUS_TX_COMPLETE         (0x1u << 3) //!< Transmission Complete
#define SJA1000_STATUS_TX_INCOMPLETE       (0x0u << 3) //!< Transmission Incomplete
#define SJA1000_STATUS_TRANSMIT_MESSAGE    (0x1u << 4) //!< Transmit Message
#define SJA1000_STATUS_RECEIVE_MESSAGE     (0x1u << 5) //!< Receive Message
#define SJA1000_STATUS_ERROR_EXCEED_LIMIT  (0x1u << 6) //!< Error Exceed Limit
#define SJA1000_STATUS_ERROR_BELOW_LIMIT   (0x0u << 6) //!< Error Below Limit
#define SJA1000_STATUS_BUS_OFF             (0x1u << 7) //!< Bus-Off Status
#define SJA1000_STATUS_BUS_ON              (0x0u << 7) //!< Bus-On Status

//-----------------------------------------------------------------------------

/*! @brief SJA1000-BasicCAN Interrupt Register (Read/Write, Address: 3, Initial value: 0b11100000)
 * @details The interrupt register allows the identification of an interrupt source. When one or more bits of this register are set, the INT pin is activated (LOW).
 * After this register is read by the microcontroller, all bits are reset what results in a floating level at INT. The interrupt register appears to the microcontroller as a read only memory
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BCAN_INT_Register
{
  uint8_t Control;
  struct
  {
    uint8_t RI : 1; /*!< 0 - Receive Interrupt: '1' = set; this bit is set while the receive FIFO is not empty and the receive interrupt enable bit is set to logic 1 (enabled) ; '0' = reset; this bit is cleared by any read access of the microcontroller
                     *       The receive interrupt bit (if enabled) and the receive buffer status bit are set at the same time. It should be noted that the receive interrupt bit is cleared upon a read access, even if there is another message available within the FIFO.
                     *       The moment the release receive buffer command is given and there is another message valid within the receive buffer, the receive interrupt is set again (if enabled) with the next tscl
                     */
    uint8_t TI : 1; //!< 1 - Transmit Interrupt: '1' = set; this bit is set whenever the transmit buffer status changes from logic 0 to logic 1 (released) and transmit interrupt enable is set to logic 1 (enabled) ; '0' = reset; this bit is cleared by any read access of the microcontroller
    uint8_t EI : 1; //!< 2 - Error Interrupt: '1' = set; this bit is set on a change of either the error status or bus status bits if the error interrupt enable is set to logic 1 (enabled) ; '0' = reset; this bit is cleared by any read access of the microcontroller
    uint8_t DOI: 1; /*!< 3 - Data Overrun Interrupt: '1' = set; this bit is set on a '0-to-1' transition of the data overrun status bit, when the data overrun interrupt enable is set to logic 1 (enabled) ; '0' = reset; this bit is cleared by any read access of the microcontroller
                     *       The overrun interrupt bit (if enabled) and the data overrun status bit are set at the same time
                     */
    uint8_t WUI: 1; /*!< 4 - Wake-Up Interrupt: '1' = set; this bit is set when the sleep mode is left ; '0' = reset; this bit is cleared by any read access of the microcontroller
                     *       A wake-up interrupt is also generated if the CPU tries to set go to sleep while the CAN controller is involved in bus activities or a CAN interrupt is pending
                     */
    uint8_t    : 1; //!< 5 - Reading this bit will always reflect a logic 1
    uint8_t    : 1; //!< 6 - Reading this bit will always reflect a logic 1
    uint8_t    : 1; //!< 7 - Reading this bit will always reflect a logic 1
  } Bits;
} SJA1000_BCAN_INT_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BCAN_INT_Register, 1);

#define SJA1000_BCAN_INT_RI_RX_EVENT             (0x1u << 0) //!< Receive Interrupt Event
#define SJA1000_BCAN_INT_TI_TX_EVENT             (0x1u << 1) //!< Transmit Interrupt Event
#define SJA1000_BCAN_INT_EI_ERROR_EVENT          (0x1u << 2) //!< Error Interrupt Event
#define SJA1000_BCAN_INT_DOI_DATA_OVERRUN_EVENT  (0x1u << 3) //!< Data Overrun Interrupt Event
#define SJA1000_BCAN_INT_WUI_WAKEUP_EVENT        (0x1u << 4) //!< Wake-Up Interrupt Event
#define SJA1000_BCAN_INT_BIT5                    (0x1u << 5) //!< Bit 5 always '1'
#define SJA1000_BCAN_INT_BIT6                    (0x1u << 6) //!< Bit 6 always '1'
#define SJA1000_BCAN_INT_BIT7                    (0x1u << 7) //!< Bit 7 always '1'

#define SJA1000_BCAN_INT_ALL_EVENTS  ( SJA1000_BCAN_INT_RI_RX_EVENT | SJA1000_BCAN_INT_TI_TX_EVENT | SJA1000_BCAN_INT_EI_ERROR_EVENT | SJA1000_BCAN_INT_DOI_DATA_OVERRUN_EVENT | SJA1000_BCAN_INT_WUI_WAKEUP_EVENT )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Specific PeliCAN Controller Registers
//********************************************************************************************************************

/*! @brief SJA1000-PeliCAN Mode Register (Read/Write, Address: 0, Initial value: 0b00000001)
 * @details The contents of the mode register are used to change the behaviour of the CAN controller. Bits may be set or reset by the CPU which uses the control register as a read/write memory. Reserved bits are read as logic 0
 * A write access to the bits MOD.LOM, MOD.STM and MOD.AFM is only possible, if the reset mode is entered previously
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_MOD_Register
{
  uint8_t Mode;
  struct
  {
    uint8_t RM : 1; /*!< 0 - Reset Request: '1' = reset, detection of a set reset mode bit results in aborting the current transmission/reception of a message and entering the reset mode ; '1-to-0' = normal, transition of the reset mode bit, the CAN controller returns to the operating mode
                     *       During a hardware reset or when the bus status bit is set to logic 1 (bus-off), the reset mode bit is also set to logic 1 (present).
                     *       If this bit is accessed by software, a value change will become visible and takes effect first with the next positive edge of the internal clock which operates at half of the external oscillator frequency.
                     *       During an external reset the microcontroller cannot set the reset mode bit to logic 0 (absent). Therefore, after having set the reset mode bit to logic 1, the microcontroller must check this bit to ensure that the external reset pin is not being held HIGH.
                     *       Changes of the reset request bit are synchronized with the internal divided clock. Reading the reset request bit reflects the synchronized status. After the reset mode bit is set to logic 0 the CAN controller will wait for:
                     *        - One occurrence of bus-free signal (11 recessive bits), if the preceding reset has been caused by a hardware reset or a CPU-initiated reset
                     *        - 128 occurrences of bus-free, if the preceding reset has been caused by a CAN controller initiated bus-off, before re-entering the bus-on mode
                     */
    uint8_t LOM: 1; //!< 1 - Listen Only Mode: '1' = listen only, in this mode the CAN controller would give no acknowledge to the CAN-bus, even if a message is received successfully; the error counters are stopped at the current value ; '0' = normal
    uint8_t STM: 1; //!< 2 - Self Test Mode: '1' = self test; in this mode a full node test is possible without any other active node on the bus using the self reception request command; the CAN controller will perform a successful transmission, even if there is no acknowledge received ; '0' = normal, an acknowledge is required for successful transmission
    uint8_t AFM: 1; /*!< 3 - Acceptance Filter Mode: '1' = single, the single acceptance filter option is enabled (one filter with the length of 32 bit is active) ; '0' = dual, the dual acceptance filter option is enabled (two filters, each with the length of 16 bit are active)
                     *       This mode of operation forces the CAN controller to be error passive. Message transmission is not possible. The listen only mode can be used e.g. for software driven bit rate detection and 'hot plugging'. All other functions can be used like in normal mode
                     */
    uint8_t SM : 1; /*!< 4 - Sleep Mode: '1' = sleep, the CAN controller enters sleep mode if no CAN interrupt is pending and if there is no bus activity ; '0' = wake-up, the CAN controller wakes up if sleeping
                     *       The SJA1000 will enter sleep mode if the sleep mode bit is set to logic 1 (sleep); then there is no bus activity and no interrupt is pending.
                     *       Setting of SM with at least one of the previously mentioned exceptions valid will result in a wake-up interrupt.
                     *       After sleep mode is set, the CLKOUT signal continues until at least 15 bit times have passed, to allow a host microcontroller clocked via this signal to enter its own standby mode before the CLKOUT goes LOW.
                     *       The SJA1000 will wake up when one of the three previously mentioned conditions is negated: after SM is set LOW (wake-up), there is bus activity or INT is driven LOW (active).
                     *       On wake-up, the oscillator is started and a wake-up interrupt is generated. A sleeping SJA1000 which wakes up due to bus activity will not be able to receive this message until it detects 11 consecutive recessive bits (bus-free sequence).
                     *       It should be noted that setting of SM is not possible in reset mode. After clearing of reset mode, setting of SM is possible first, when bus-free is detected again
                     */
    uint8_t    : 3; //!< 5-7
  } Bits;
} SJA1000_PCAN_MOD_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_MOD_Register, 1);

#define SJA1000_SINGLE_ACCEPTANCE_FILTER  (0x1u << 3) //!< Single acceptance filter option is enabled (one filter with the length of 32 bit is active)
#define SJA1000_DUAL_ACCEPTANCE_FILTER    (0x0u << 3) //!< Dual acceptance filter option is enabled (two filters, each with the length of 16 bit are active)
#define SJA1000_ACCEPTANCE_FILTER_Mask    (SJA1000_SINGLE_ACCEPTANCE_FILTER) //!< Acceptance filter option mask

//! Mode selection enumerator
typedef enum
{
  SJA1000_MODE_NORMAL  = 0x00, //!< Normal mode operation
  SJA1000_MODE_RESET   = 0x01, //!< Reset mode operation
  SJA1000_MODE_STANDBY = 0x02, //!< Standby mode operation
  SJA1000_MODE_TEST    = 0x04, //!< Test mode operation
  SJA1000_MODE_SLEEP   = 0x10, //!< Sleep mode operation
} eSJA1000_OperationMode;

#define SJA1000_MODE_OPERATION_Pos         0
#define SJA1000_MODE_OPERATION_Mask        (0x17u << SJA1000_MODE_OPERATION_Pos)
#define SJA1000_MODE_OPERATION_SET(value)  (((uint32_t)(value) << SJA1000_MODE_OPERATION_Pos) & SJA1000_MODE_OPERATION_Mask) //!< Set mode operation
#define SJA1000_MODE_OPERATION_GET(value)  (eSJA1000_OperationMode)(((uint32_t)(value) & SJA1000_MODE_OPERATION_Mask) >> SJA1000_MODE_OPERATION_Pos) //!< Get mode operation

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Command Register (Read/Write, Address: 1, Initial value: 0b00000000)
 * @details A command bit initiates an action within the transfer layer of the CAN controller. This register is write only, all bits will return a logic 0 when being read. Between two commands at least one internal clock cycle is needed in order to proceed. The internal clock is half of the external oscillator frequency
 * Setting the command bits CMR.TR and CMR.AT simultaneously results in sending the transmit message once. No re-transmission will be performed in the event of an error or arbitration lost (single-shot transmission).
 * Setting the command bits CMR.SRR and CMR.AT simultaneously results in sending the transmit message once using the self reception feature. No re-transmission will be performed in the event of an error or arbitration lost. Setting the command bits CMR.TR, CMR.AT and CMR.SRR simultaneously results in sending the transmit message once as described for CMR.TR and CMR.AT.
 * The moment the transmit status bit is set within the status register, the internal transmission request bit is cleared automatically.
 * Setting CMR.TR and CMR.SRR simultaneously will ignore the set CMR.SRR bit
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_CMR_Register
{
  uint8_t Command;
  struct
  {
    uint8_t TR : 1; /*!< 0 - Transmission Request: '1' = a message shall be transmitted ; '0' = (absent)
                     *       If the transmission request was set to logic 1 in a previous command, it cannot be cancelled by setting the transmission request bit to logic 0. The requested transmission may be cancelled by setting the abort transmission bit to logic 1
                     */
    uint8_t AT : 1; /*!< 1 - Abort Transmission: '1' = if not already in progress, a pending transmission request is cancelled ; '0' = (absent)
                     *       The abort transmission bit is used when the CPU requires the suspension of the previously requested transmission, e.g. to transmit a more urgent message before.
                     *       A transmission already in progress is not stopped. In order to see if the original message has been either transmitted successfully or aborted, the transmission complete status bit should be checked.
                     *       This should be done after the transmit buffer status bit has been set to logic 1 or a transmit interrupt has been generated.
                     *       It should be noted that a transmit interrupt is generated even if the message was aborted because the transmit buffer status bit changes to 'released'
                     */
    uint8_t RRB: 1; /*!< 2 - Release Receive Buffer: '1' = the receive buffer, representing the message memory space in the RXFIFO is released ; '0' = (no action)
                     *       After reading the contents of the receive buffer, the CPU can release this memory space in the RXFIFO by setting the release receive buffer bit to logic 1.
                     *       This may result in another message becoming immediately available within the receive buffer. If there is no other message available, the receive interrupt bit is reset
                     */
    uint8_t CDO: 1; /*!< 3 - Clear Data Overrun: '1' = the data overrun status bit is cleared ; '0' = (no action)
                     *       This command bit is used to clear the data overrun condition indicated by the data overrun status bit. As long as the data overrun status bit is set no further data overrun interrupt is generated
                     */
    uint8_t SRR: 1; /*!< 4 - Self Reception Request: '1' = a message shall be transmitted and received simultaneously ; '0' = (absent)
                     *       Upon self reception request a message is transmitted and simultaneously received if the acceptance filter is set to the corresponding identifier. A receive and a transmit interrupt will indicate correct self reception (see also self test mode in mode register)
                     */
    uint8_t    : 3; //!< 5-7
  } Bits;
} SJA1000_PCAN_CMR_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_CMR_Register, 1);

#define SJA1000_CMR_TRANSMISSION_REQUEST    (0x1u << 0) //!< Set transmission request
#define SJA1000_CMR_ABORT_TRANSMISSION      (0x1u << 1) //!< Set abort transmission
#define SJA1000_CMR_RELEASE_RECEIVE_BUFFER  (0x1u << 2) //!< Set release receive buffer
#define SJA1000_CMR_CLEAR_DATA_OVERRUN      (0x1u << 3) //!< Set clear data overrun
#define SJA1000_CMR_SELF_RECEPTION_REQUEST  (0x1u << 4) //!< Set self Reception Request

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Status Register (Read/Write, Address: 2, Initial value: 0b00111100)
 * @details The content of the status register reflects the status of the CAN controller. The status register appears to the CPU as a read only memory
 * If both the receive status and the transmit status bits are logic 0 (idle) the CAN-bus is idle. If both bits are set the controller is waiting to become idle again.
 * After a hardware reset 11 consecutive recessive bits have to be detected until the idle status is reached. After bus-off this will take 128 of 11 consecutive recessive bits
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_SR_Register
{
  uint8_t Status;
  struct
  {
    uint8_t RBS: 1; /*!< 0 - Receive Buffer Status: '1' = full; one or more messages are available in the RXFIFO ; '0' = empty; no message is available
                     *       After reading all messages within the RXFIFO and releasing their memory space with the command release receive buffer this bit is cleared
                     */
    uint8_t DOS: 1; /*!< 1 - Data Overrun Status: '1' = overrun; a message was lost because there was not enough space for that message in the RXFIFO ; '0' = absent; no data overrun has occurred since the last clear data overrun command was given
                     *       When a message that is to be received has passed the acceptance filter successfully, the CAN controller needs space in the RXFIFO to store the message descriptor and for each data byte which has been received.
                     *       If there is not enough space to store the message, that message is dropped and the data overrun condition is indicated to the CPU at the moment this message becomes valid. If this message is not completed successfully (e.g. due to an error), no overrun condition is indicated
                     */
    uint8_t TBS: 1; /*!< 2 - Transmit Buffer Status: '1' = released; the CPU may write a message into the transmit buffer ; '0' = locked; the CPU cannot access the transmit buffer; a message is waiting for transmission or is already in process
                     *       If the CPU tries to write to the transmit buffer when the transmit buffer status bit is logic 0 (locked), the written byte will not be accepted and will be lost without this being indicated
                     */
    uint8_t TCS: 1; /*!< 3 - Transmission Complete Status: '1' = complete; the last requested transmission has been successfully completed ; '0' = incomplete; the previously requested transmission is not yet completed
                     *       The transmission complete status bit is set to logic 0 (incomplete) whenever the transmission request bit or the self reception request bit is set to logic 1. The transmission complete status bit will remain at logic 0 until a message is transmitted successfully
                     */
    uint8_t RS : 1; //!< 4 - Receive Status: '1' = receive; the SJA1000 is receiving a message ; '0' = idle; no receive message is in progress
    uint8_t TS : 1; //!< 5 - Transmit Status: '1' = transmit; the SJA1000 is transmitting a message ; '0' = idle; no transmit message is in progress
    uint8_t ES : 1; /*!< 6 - Error Status: '1' = error; at least one of the error counters has reached or exceeded the CPU warning limit ; '0' = ok; both error counters are below the warning limit
                     *       Errors detected during reception or transmission will effect the error counters according to the CAN 2.0B protocol specification.
                     *       The error status bit is set when at least one of the error counters has reached or exceeded the CPU warning limit (EWLR). An error warning interrupt is generated, if enabled. The default value of EWLR after hardware reset is 96
                     */
    uint8_t BS : 1; /*!< 7 - Bus Status: '1' = bus-off; the SJA1000 is not involved in bus activities ; '0' = bus-on; the SJA1000 is involved in bus activities
                     *       When the transmit error counter exceeds the limit of 255, the bus status bit is set to logic 1 (bus-off), the CAN controller will set the reset mode bit to logic 1 (present) and an error warning interrupt is generated, if enabled.
                     *       The transmit error counter is set to 127 and the receive error counter is cleared. It will stay in this mode until the CPU clears the reset mode bit.
                     *       Once this is completed the CAN controller will wait the minimum protocol-defined time (128 occurrences of the bus-free signal) counting down the transmit error counter. After that the bus status bit is cleared (bus-on), the error status bit is set to logic 0 (ok), the error counters are reset and an error warning interrupt is generated, if enabled.
                     *       Reading the TX error counter during this time gives information about the status of the bus-off recovery
                     */
  } Bits;
} SJA1000_PCAN_SR_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_SR_Register, 1);

#define SJA1000_STATUS_RX_BUFFER_FULL      (0x1u << 0) //!< Receive Buffer Full
#define SJA1000_STATUS_RX_BUFFER_EMPTY     (0x0u << 0) //!< Receive Buffer Empty
#define SJA1000_STATUS_DATA_OVERRUN        (0x1u << 1) //!< Receive Buffer Data Overrun
#define SJA1000_STATUS_TX_BUFFER_RELEASED  (0x1u << 2) //!< Transmit Buffer Released
#define SJA1000_STATUS_TX_BUFFER_LOCKED    (0x0u << 2) //!< Transmit Buffer Locked
#define SJA1000_STATUS_TX_COMPLETE         (0x1u << 3) //!< Transmission Complete
#define SJA1000_STATUS_TX_INCOMPLETE       (0x0u << 3) //!< Transmission Incomplete
#define SJA1000_STATUS_TRANSMITED_MESSAGE  (0x1u << 4) //!< Transmited Message
#define SJA1000_STATUS_RECEIVED_MESSAGE    (0x1u << 5) //!< Received Message
#define SJA1000_STATUS_ERROR_EXCEED_LIMIT  (0x1u << 6) //!< Error Exceed Limit
#define SJA1000_STATUS_ERROR_BELOW_LIMIT   (0x0u << 6) //!< Error Below Limit
#define SJA1000_STATUS_BUS_OFF             (0x1u << 7) //!< Bus-Off Status
#define SJA1000_STATUS_BUS_ON              (0x0u << 7) //!< Bus-On Status

//! Status Events of the device, can be OR'ed.
SJA1000_PACKENUM(eSJA1000_StatusEvents, uint8_t)
{
  SJA1000_NO_STATUS          = 0x00,                              //!< No interrupt
  SJA1000_RX_BUFFER_FULL     = SJA1000_STATUS_RX_BUFFER_FULL,     //!< Receive Buffer Full event
  SJA1000_DATA_OVERRUN       = SJA1000_STATUS_DATA_OVERRUN,       //!< Receive Buffer Data Overrun event
  SJA1000_TX_BUFFER_RELEASED = SJA1000_STATUS_TX_BUFFER_RELEASED, //!< Transmit Buffer Released event
  SJA1000_TX_COMPLETE        = SJA1000_STATUS_TX_COMPLETE,        //!< Transmission Complete event
  SJA1000_TRANSMITED_MESSAGE = SJA1000_STATUS_TRANSMITED_MESSAGE, //!< Transmited Message event
  SJA1000_RECEIVED_MESSAGE   = SJA1000_STATUS_RECEIVED_MESSAGE,   //!< Received Message event
  SJA1000_ERROR_EXCEED_LIMIT = SJA1000_STATUS_ERROR_EXCEED_LIMIT, //!< Error Exceed Limit event
  SJA1000_BUS_OFF            = SJA1000_STATUS_BUS_OFF,            //!< Bus-Off Status event
} SJA1000_UNPACKENUM(eSJA1000_StatusEvents);
SJA1000_CONTROL_ITEM_SIZE(eSJA1000_StatusEvents, 1);

typedef eSJA1000_StatusEvents setSJA1000_StatusEvents; //!< Set of Status Events (can be OR'ed)

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Interrupt Register (Read/Write, Address: 3, Initial value: 0b11100000)
 * @details The interrupt register allows the identification of an interrupt source. When one or more bits of this register are set, the INT pin is activated (LOW).
 * After this register is read by the microcontroller, all bits are reset what results in a floating level at INT. The interrupt register appears to the microcontroller as a read only memory
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_IR_Register
{
  uint8_t IR;
  struct
  {
    uint8_t RI : 1; /*!< 0 - Receive Interrupt: '1' = set; this bit is set while the receive FIFO is not empty and the RIE bit is set within the interrupt enable register ; '0' = reset; no more message is available within the RXFIFO
                     *       The behaviour of this bit is equivalent to that of the receive buffer status bit with the exception, that RI depends on the corresponding interrupt enable bit (RIE).
                     *       So the receive interrupt bit is not cleared upon a read access to the interrupt register. Giving the command 'release receive buffer' will clear RI temporarily. If there is another message available within the FIFO after the release command, RI is set again. Otherwise RI remains cleared
                     */
    uint8_t TI : 1; //!< 1 - Transmit Interrupt: '1' = set; this bit is set whenever the transmit buffer status changes from '0-to-1' (released) and the TIE bit is set within the interrupt enable register ; '0' = reset
    uint8_t EI : 1; //!< 2 - Error Interrupt: '1' = set; this bit is set on every change (set and clear) of either the error status or bus status bits and the EIE bit is set within the interrupt enable register ; '0' = reset
    uint8_t DOI: 1; //!< 3 - Data Overrun Interrupt: '1' = set; this bit is set on a '0-to-1' transition of the data overrun status bit and the DOIE bit is set within the interrupt enable register ; '0' = reset
    uint8_t WUI: 1; /*!< 4 - Wake-Up Interrupt: '1' = set; this bit is set when the CAN controller is sleeping and bus activity is detected and the WUIE bit is set within the interrupt enable register ; '0' = reset
                     *       A wake-up interrupt is also generated, if the CPU tries to set the sleep bit while the CAN controller is involved in bus activities or a CAN interrupt is pending
                     */
    uint8_t EPI: 1; //!< 5 - Error Passive Interrupt: '1' = set; this bit is set whenever the CAN controller has reached the error passive status (at least one error counter exceeds the protocol-defined level of 127) or if the CAN controller is in the error passive status and enters the error active status again and the EPIE bit is set within the interrupt enable register ; '0' = reset
    uint8_t ALI: 1; //!< 6 - Arbitration Lost Interrupt: '1' = set; this bit is set when the CAN controller lost the arbitration and becomes a receiver and the ALIE bit is set within the interrupt enable register ; '0' = reset
    uint8_t BEI: 1; //!< 7 - Bus Error Interrupt: '1' = set; this bit is set when the CAN controller detects an error on the CAN-bus and the BEIE bit is set within the interrupt enable register ; '0' = reset
  } Bits;
} SJA1000_PCAN_IR_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_IR_Register, 1);

#define SJA1000_PCAN_INT_RI_RX_EVENT                 (0x1u << 0) //!< Receive Interrupt Event
#define SJA1000_PCAN_INT_TI_TX_EVENT                 (0x1u << 1) //!< Transmit Interrupt Event
#define SJA1000_PCAN_INT_EI_ERROR_EVENT              (0x1u << 2) //!< Error Interrupt Event
#define SJA1000_PCAN_INT_DOI_DATA_OVERRUN_EVENT      (0x1u << 3) //!< Data Overrun Interrupt Event
#define SJA1000_PCAN_INT_WUI_WAKEUP_EVENT            (0x1u << 4) //!< Wake-Up Interrupt Event
#define SJA1000_PCAN_INT_EPI_ERROR_PASSIVE_EVENT     (0x1u << 5) //!< Error Passive Event
#define SJA1000_PCAN_INT_ALI_ARBITRATION_LOST_EVENT  (0x1u << 6) //!< Arbitration Lost Event
#define SJA1000_PCAN_INT_BEI_BUS_ERROR_EVENT         (0x1u << 7) //!< Bus Error Event

#define SJA1000_PCAN_INT_ALL_EVENTS  ( SJA1000_PCAN_INT_RI_RX_EVENT      | SJA1000_PCAN_INT_TI_TX_EVENT             | SJA1000_PCAN_INT_EI_ERROR_EVENT             | SJA1000_PCAN_INT_DOI_DATA_OVERRUN_EVENT \
                                     | SJA1000_PCAN_INT_WUI_WAKEUP_EVENT | SJA1000_PCAN_INT_EPI_ERROR_PASSIVE_EVENT | SJA1000_PCAN_INT_ALI_ARBITRATION_LOST_EVENT | SJA1000_PCAN_INT_BEI_BUS_ERROR_EVENT )

//! Interrupt Events, can be OR'ed.
typedef enum
{
  SJA1000_NO_INTERRUPT_EVENT     = 0x00,                                        //!< No interrupt events
  SJA1000_RX_BUFFER_EVENT        = SJA1000_PCAN_INT_RI_RX_EVENT,                //!< Receive event
  SJA1000_TX_BUFFER_EVENT        = SJA1000_PCAN_INT_TI_TX_EVENT,                //!< Transmit event
  SJA1000_ERROR_EVENT            = SJA1000_PCAN_INT_EI_ERROR_EVENT,             //!< Error event
  SJA1000_OVERRUN_EVENT          = SJA1000_PCAN_INT_DOI_DATA_OVERRUN_EVENT,     //!< Overrun event
  SJA1000_WAKEUP_EVENT           = SJA1000_PCAN_INT_WUI_WAKEUP_EVENT,           //!< Wake-Up event
  SJA1000_ERROR_PASSIVE_EVENT    = SJA1000_PCAN_INT_EPI_ERROR_PASSIVE_EVENT,    //!< Error passive event
  SJA1000_ARBITRATION_LOST_EVENT = SJA1000_PCAN_INT_ALI_ARBITRATION_LOST_EVENT, //!< Arbitration lost event
  SJA1000_BUS_ERROR_EVENT        = SJA1000_PCAN_INT_BEI_BUS_ERROR_EVENT,        //!< Bus error event

  SJA1000_BCAN__ENABLE_ALL_EVENTS    = SJA1000_BCAN_INT_ALL_EVENTS, //!< Enable all events
  SJA1000_PCAN__ENABLE_ALL_EVENTS    = SJA1000_PCAN_INT_ALL_EVENTS, //!< Enable all events
  SJA1000_BCAN_INT_EVENTS_FLAGS_MASK = SJA1000_BCAN_INT_ALL_EVENTS, //!< BasicCAN Interrupt events flags mask
  SJA1000_PCAN_INT_EVENTS_FLAGS_MASK = SJA1000_PCAN_INT_ALL_EVENTS, //!< PeliCAN Interrupt events flags mask
} eSJA1000_InterruptEvents;

typedef eSJA1000_InterruptEvents setSJA1000_InterruptEvents; //!< Set of Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Interrupt Enable Register (Read/Write, Address: 4, Initial value: 0b00000000)
 * @details The register allows to enable different types of interrupt sources which are indicated to the CPU.
 * The interrupt enable register appears to the CPU as a read/write memory
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_IER_Register
{
  uint8_t IER;
  struct
  {
    uint8_t RIE : 1; /*!< 0 - Receive Interrupt Enable: '1' = enabled; when a message has been received without errors, the SJA1000 transmits a receive interrupt signal to the microcontroller ; '0' = disabled
                      *       The receive interrupt enable bit has direct influence to the receive interrupt bit and the external interrupt output INT. If RIE is cleared, the external INT pin will become HIGH immediately, if there is no other interrupt pending
                      */
    uint8_t TIE : 1; //!< 1 - Transmit Interrupt Enable: '1' = enabled; when a message has been successfully transmitted or the transmit buffer is accessible again, (e.g. after an abort transmission command) the SJA1000 transmits a transmit interrupt signal to the microcontroller ; '0' = disabled
    uint8_t EIE : 1; //!< 2 - Error Interrupt Enable: '1' = enabled; if the error or bus status change, the microcontroller receives an error interrupt signal (see also status register) ; '0' = disabled
    uint8_t DOIE: 1; //!< 3 - Data Overrun Interrupt Enable: '1' = enabled; if the data overrun bit is set, the microcontroller receives an overrun interrupt signal (see also status register) ; '0' = disabled
    uint8_t WUIE: 1; //!< 4 - Wake-Up Interrupt Enable: '1' = enabled; if the sleeping CAN controller wakes up, the respective interrupt is requested ; '0' = disabled
    uint8_t EPIE: 1; //!< 5 - Error Passive Interrupt Enable: '1' = enabled; if the error status of the CAN controller changes from error active to error passive or vice versa, the respective interrupt is requested ; '0' = disabled
    uint8_t ALIE: 1; //!< 6 - Arbitration Lost Interrupt Enable: '1' = enabled; if the CAN controller has lost arbitration, the respective interrupt is requested ; '0' = disabled
    uint8_t BEIE: 1; //!< 7 - Bus Error Interrupt Enable: '1' = enabled; if an bus error has been detected, the CAN controller requests the respective interrupt ; '0' = disabled
  } Bits;
} SJA1000_PCAN_IER_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_IER_Register, 1);

#define SJA1000_PCAN_IER_RIE_ENABLE    (0x1u << 0) //!< Enable the Receive Interrupt
#define SJA1000_PCAN_IER_RIE_DISABLE   (0x0u << 0) //!< Disable the Receive Interrupt
#define SJA1000_PCAN_IER_TIE_ENABLE    (0x1u << 1) //!< Enable the Transmit Interrupt
#define SJA1000_PCAN_IER_TIE_DISABLE   (0x0u << 1) //!< Disable the Transmit Interrupt
#define SJA1000_PCAN_IER_EIE_ENABLE    (0x1u << 2) //!< Enable the Error Interrupt
#define SJA1000_PCAN_IER_EIE_DISABLE   (0x0u << 2) //!< Disable the Error Interrupt
#define SJA1000_PCAN_IER_OIE_ENABLE    (0x1u << 3) //!< Enable the Data Overrun Interrupt
#define SJA1000_PCAN_IER_OIE_DISABLE   (0x0u << 3) //!< Disable the Data Overrun Interrupt
#define SJA1000_PCAN_IER_WUIE_ENABLE   (0x1u << 4) //!< Enable the Wake-Up Interrupt
#define SJA1000_PCAN_IER_WUIE_DISABLE  (0x0u << 4) //!< Disable the Wake-Up Interrupt
#define SJA1000_PCAN_IER_EPIE_ENABLE   (0x1u << 5) //!< Enable the Error Passive Interrupt
#define SJA1000_PCAN_IER_EPIE_DISABLE  (0x0u << 5) //!< Disable the Error Passive Interrupt
#define SJA1000_PCAN_IER_ALIE_ENABLE   (0x1u << 6) //!< Enable the Arbitration Lost Interrupt
#define SJA1000_PCAN_IER_ALIE_DISABLE  (0x0u << 6) //!< Disable the Arbitration Lost Interrupt
#define SJA1000_PCAN_IER_BEIE_ENABLE   (0x1u << 7) //!< Enable the Bus Error Interrupt
#define SJA1000_PCAN_IER_BEIE_DISABLE  (0x0u << 7) //!< Disable the Bus Error Interrupt

#define SJA1000_PCAN_IER_ALL_INT  ( SJA1000_PCAN_IER_RIE_ENABLE  | SJA1000_PCAN_IER_TIE_ENABLE  | SJA1000_PCAN_IER_EIE_ENABLE  | SJA1000_PCAN_IER_OIE_ENABLE  \
                                  | SJA1000_PCAN_IER_WUIE_ENABLE | SJA1000_PCAN_IER_EPIE_ENABLE | SJA1000_PCAN_IER_ALIE_ENABLE | SJA1000_PCAN_IER_BEIE_ENABLE )

//! Interrupt Enables, can be OR'ed.
typedef enum
{
  SJA1000_NO_INTERRUPT               = 0x00,                         //!< No interrupt
  SJA1000_RX_BUFFER_INTERRUPT        = SJA1000_PCAN_IER_RIE_ENABLE,  //!< Receive interrupt
  SJA1000_TX_BUFFER_INTERRUPT        = SJA1000_PCAN_IER_TIE_ENABLE,  //!< Transmit interrupt
  SJA1000_ERROR_INTERRUPT            = SJA1000_PCAN_IER_EIE_ENABLE,  //!< Error interrupt
  SJA1000_DATA_OVERRUN_INTERRUPT     = SJA1000_PCAN_IER_OIE_ENABLE,  //!< Data Overrun interrupt
  SJA1000_WAKEUP_INTERRUPT           = SJA1000_PCAN_IER_WUIE_ENABLE, //!< Wake-Up interrupt
  SJA1000_ERROR_PASSIVE_INTERRUPT    = SJA1000_PCAN_IER_EPIE_ENABLE, //!< Error Passive interrupt
  SJA1000_ARBITRATION_LOST_INTERRUPT = SJA1000_PCAN_IER_ALIE_ENABLE, //!< Arbitration Lost interrupt
  SJA1000_BUS_ERROR_INTERRUPT        = SJA1000_PCAN_IER_BEIE_ENABLE, //!< Bus Error interrupt

  SJA1000_ENABLE_ALL_BCAN_INTERRUPTS = SJA1000_BCAN_IER_ALL_INT,     //!< Enable all BasicCAN interrupts
  SJA1000_ENABLE_ALL_PCAN_INTERRUPTS = SJA1000_PCAN_IER_ALL_INT,     //!< Enable all PeliCAN interrupts
  SJA1000_BCAN_INTERRUPTS_FLAGS_MASK = SJA1000_BCAN_IER_ALL_INT,     //!< BasicCAN Interrupts flags mask
  SJA1000_PCAN_INTERRUPTS_FLAGS_MASK = SJA1000_PCAN_IER_ALL_INT,     //!< PeliCAN Interrupts flags mask
} eSJA1000_Interrupts;

typedef eSJA1000_Interrupts setSJA1000_Interrupts; //!< Set of Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Arbitration Lost Capture Register (Read/Write, Address: 11, Initial value: 0b00000000)
 * @details This register contains information about the bit position of losing arbitration. The arbitration lost capture register appears to the CPU as a read only memory. Reserved bits are read as logic 0.
 * On arbitration lost, the corresponding arbitration lost interrupt is forced, if enabled. At the same time, the current bit position of the bit stream processor is captured into the arbitration lost capture register. The content within this register is fixed until the users software has read out its contents once. The capture mechanism is then activated again.
 * The corresponding interrupt flag located in the interrupt register is cleared during the read access to the interrupt register. A new arbitration lost interrupt is not possible until the arbitration lost capture register is read out once.
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_ALC_Register
{
  uint8_t ALC;
  struct
  {
    uint8_t BITNO: 5; //!< 0-4 - Arbitration lost position
    uint8_t      : 3; //!< 5-7
  } Bits;
} SJA1000_PCAN_ALC_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_ALC_Register, 1);

/*! Arbitration lost position enumerator.
 * Binary coded frame bit number where arbitration was lost
 */
typedef enum
{
  SJA1000_ARBITRATION_LOST_BIT1  = 0b00000, //!< Arbitration lost in bit 1
  SJA1000_ARBITRATION_LOST_BIT2  = 0b00001, //!< Arbitration lost in bit 2
  SJA1000_ARBITRATION_LOST_BIT3  = 0b00010, //!< Arbitration lost in bit 3
  SJA1000_ARBITRATION_LOST_BIT4  = 0b00011, //!< Arbitration lost in bit 4
  SJA1000_ARBITRATION_LOST_BIT5  = 0b00100, //!< Arbitration lost in bit 5
  SJA1000_ARBITRATION_LOST_BIT6  = 0b00101, //!< Arbitration lost in bit 6
  SJA1000_ARBITRATION_LOST_BIT7  = 0b00110, //!< Arbitration lost in bit 7
  SJA1000_ARBITRATION_LOST_BIT8  = 0b00111, //!< Arbitration lost in bit 8
  SJA1000_ARBITRATION_LOST_BIT9  = 0b01000, //!< Arbitration lost in bit 9
  SJA1000_ARBITRATION_LOST_BIT10 = 0b01001, //!< Arbitration lost in bit 10
  SJA1000_ARBITRATION_LOST_BIT11 = 0b01010, //!< Arbitration lost in bit 11
  SJA1000_ARBITRATION_LOST_SRTR  = 0b01011, //!< Arbitration lost in bit RTR for standard frame messages
  SJA1000_ARBITRATION_LOST_IDE   = 0b01100, //!< Arbitration lost in bit IDE
  SJA1000_ARBITRATION_LOST_BIT12 = 0b01101, //!< Arbitration lost in bit 12
  SJA1000_ARBITRATION_LOST_BIT13 = 0b01110, //!< Arbitration lost in bit 13
  SJA1000_ARBITRATION_LOST_BIT14 = 0b01111, //!< Arbitration lost in bit 14
  SJA1000_ARBITRATION_LOST_BIT15 = 0b10000, //!< Arbitration lost in bit 15
  SJA1000_ARBITRATION_LOST_BIT16 = 0b10001, //!< Arbitration lost in bit 16
  SJA1000_ARBITRATION_LOST_BIT17 = 0b10010, //!< Arbitration lost in bit 17
  SJA1000_ARBITRATION_LOST_BIT18 = 0b10011, //!< Arbitration lost in bit 18
  SJA1000_ARBITRATION_LOST_BIT19 = 0b10100, //!< Arbitration lost in bit 19
  SJA1000_ARBITRATION_LOST_BIT20 = 0b10101, //!< Arbitration lost in bit 20
  SJA1000_ARBITRATION_LOST_BIT21 = 0b10110, //!< Arbitration lost in bit 21
  SJA1000_ARBITRATION_LOST_BIT22 = 0b10111, //!< Arbitration lost in bit 22
  SJA1000_ARBITRATION_LOST_BIT23 = 0b11000, //!< Arbitration lost in bit 23
  SJA1000_ARBITRATION_LOST_BIT24 = 0b11001, //!< Arbitration lost in bit 24
  SJA1000_ARBITRATION_LOST_BIT25 = 0b11010, //!< Arbitration lost in bit 25
  SJA1000_ARBITRATION_LOST_BIT26 = 0b11011, //!< Arbitration lost in bit 26
  SJA1000_ARBITRATION_LOST_BIT27 = 0b11100, //!< Arbitration lost in bit 27
  SJA1000_ARBITRATION_LOST_BIT28 = 0b11101, //!< Arbitration lost in bit 28
  SJA1000_ARBITRATION_LOST_BIT29 = 0b11110, //!< Arbitration lost in bit 29
  SJA1000_ARBITRATION_LOST_ERTR  = 0b11111, //!< Arbitration lost in bit RTR for extended frame messages
} eSJA1000_ArbitrationLost;

#define SJA1000_ARBITRATION_LOST_Pos         0
#define SJA1000_ARBITRATION_LOST_Mask        (0x1Fu << SJA1000_ARBITRATION_LOST_Pos)
#define SJA1000_ARBITRATION_LOST_GET(value)  (((uint8_t)(value) & SJA1000_ARBITRATION_LOST_Mask) >> SJA1000_ARBITRATION_LOST_Pos) //!< Get arbitration lost position

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Error Capture Register (Read/Write, Address: 12, Initial value: 0b00000000)
 * @details This register contains information about the type and location of errors on the bus. The error code capture register appears to the CPU as a read only memory.
 * If a bus error occurs, the corresponding bus error interrupt is always forced, if enabled. At the same time, the current position of the bit stream processor is captured into the error code capture register.
 * The content within this register is fixed until the users software has read out its content once. The capture mechanism is then activated again.
 * The corresponding interrupt flag located in the interrupt register is cleared during the read access to the interrupt register. A new bus error interrupt is not possible until the capture register is read out once
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_ECC_Register
{
  uint8_t ECC;
  struct
  {
    uint8_t SEG : 5; //!< 0-4 - Segment
    uint8_t DIR : 1; //!< 5   - Direction
    uint8_t ERRC: 2; //!< 6-7 - Error Code
  } Bits;
} SJA1000_PCAN_ECC_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_ECC_Register, 1);

/*! Current frame segment error enumerator.
 * Reflect the current frame segment to distinguish between different error events
 */
typedef enum
{
  SJA1000_ERROR_SEGMENT_SOF         = 0b00011, //!< Current frame segment is start of frame
  SJA1000_ERROR_SEGMENT_ID28_21     = 0b00010, //!< Current frame segment is ID.28 to ID.21
  SJA1000_ERROR_SEGMENT_ID20_18     = 0b00110, //!< Current frame segment is ID.20 to ID.18
  SJA1000_ERROR_SEGMENT_SRTR        = 0b00100, //!< Current frame segment is bit SRTR
  SJA1000_ERROR_SEGMENT_IDE         = 0b00101, //!< Current frame segment is bit IDE
  SJA1000_ERROR_SEGMENT_ID17_13     = 0b00111, //!< Current frame segment is ID.17 to ID.13
  SJA1000_ERROR_SEGMENT_ID12_5      = 0b01111, //!< Current frame segment is ID.12 to ID.5
  SJA1000_ERROR_SEGMENT_ID4_0       = 0b01110, //!< Current frame segment is ID.4 to ID.0
  SJA1000_ERROR_SEGMENT_RTR         = 0b01100, //!< Current frame segment is bit RTR
  SJA1000_ERROR_SEGMENT_RES1        = 0b01101, //!< Current frame segment is reserved bit 1
  SJA1000_ERROR_SEGMENT_RES0        = 0b01001, //!< Current frame segment is reserved bit 0
  SJA1000_ERROR_SEGMENT_DLC         = 0b01011, //!< Current frame segment is data length code
  SJA1000_ERROR_SEGMENT_DATA        = 0b01010, //!< Current frame segment is data field
  SJA1000_ERROR_SEGMENT_CRC_SEQ     = 0b01000, //!< Current frame segment is CRC sequence
  SJA1000_ERROR_SEGMENT_CRC_DELIM   = 0b11000, //!< Current frame segment is CRC delimiter
  SJA1000_ERROR_SEGMENT_ACK_SLOT    = 0b11001, //!< Current frame segment is acknowledge slot
  SJA1000_ERROR_SEGMENT_ACK_DELIM   = 0b11011, //!< Current frame segment is acknowledge delimiter
  SJA1000_ERROR_SEGMENT_EOF         = 0b11010, //!< Current frame segment is end of frame
  SJA1000_ERROR_SEGMENT_INTERM      = 0b10010, //!< Current frame segment is intermission
  SJA1000_ERROR_SEGMENT_ACTIVE      = 0b10001, //!< Current frame segment is active error flag
  SJA1000_ERROR_SEGMENT_PASSIVE     = 0b10110, //!< Current frame segment is passive error flag
  SJA1000_ERROR_SEGMENT_DOMINANT    = 0b10011, //!< Current frame segment is tolerate dominant bits
  SJA1000_ERROR_SEGMENT_ERROR_DELIM = 0b10111, //!< Current frame segment is error delimiter
  SJA1000_ERROR_SEGMENT_OVERLOAD    = 0b11100, //!< Current frame segment is overload flag
} eSJA1000_ErrorSegment;

#define SJA1000_ERROR_SEGMENT_Pos         0
#define SJA1000_ERROR_SEGMENT_Mask        (0x1Fu << SJA1000_ERROR_SEGMENT_Pos)
#define SJA1000_ERROR_SEGMENT_GET(value)  (((uint8_t)(value) & SJA1000_ERROR_SEGMENT_Mask) >> SJA1000_ERROR_SEGMENT_Pos) //!< Get error segment

//! Error direction enumerator
typedef enum
{
  SJA1000_ERROR_DIRECTION_TX = 0b00, //!< Error direction Tx
  SJA1000_ERROR_DIRECTION_RX = 0b01, //!< Error direction Rx
} eSJA1000_ErrorDirection;

#define SJA1000_ERROR_DIRECTION_Pos         5
#define SJA1000_ERROR_DIRECTION_Mask        (0x1u << SJA1000_ERROR_DIRECTION_Pos)
#define SJA1000_ERROR_DIRECTION_GET(value)  (((uint8_t)(value) & SJA1000_ERROR_DIRECTION_Mask) >> SJA1000_ERROR_DIRECTION_Pos) //!< Get error direction

//! Error code enumerator
typedef enum
{
  SJA1000_BIT_ERROR   = 0b00, //!< Bit error code
  SJA1000_FORM_ERROR  = 0b01, //!< Form error code
  SJA1000_STUFF_ERROR = 0b10, //!< Stuff error code
  SJA1000_OTHER_ERROR = 0b11, //!< Other type of error code
} eSJA1000_ErrorCode;

#define SJA1000_ERROR_CODE_Pos         6
#define SJA1000_ERROR_CODE_Mask        (0x3u << SJA1000_ERROR_CODE_Pos)
#define SJA1000_ERROR_CODE_GET(value)  (((uint8_t)(value) & SJA1000_ERROR_CODE_Mask) >> SJA1000_ERROR_CODE_Pos) //!< Get error code

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Error Warning Limit Register (Read/Write, Address: 13, Initial value: 0b01100000)
 * @details The error warning limit can be defined within this register. The default value (after hardware reset) is 96. In reset mode this register appears to the CPU as a read/write memory. In operating mode it is read only.
 * @note That a content change of the EWLR is only possible, if the reset mode was entered previously. An error status change (see status register; Table 14) and an error warning interrupt forced by the new register content will not occur until the reset mode is cancelled again
 */
typedef uint8_t SJA1000_PCAN_EWLR_Register;

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN RX error counter Register (Read/Write, Address: 14, Initial value: 0b00000000)
 * @details The RX error counter register reflects the current value of the receive error counter. After a hardware reset this register is initialized to logic 0. In operating mode this register appears to the CPU as a read only memory. A write access to this register is possible only in reset mode.
 * If a bus-off event occurs, the RX error counter is initialized to logic 0. The time bus-off is valid, writing to this register has no effect.
 * @note That a CPU-forced content change of the RX error counter is only possible, if the reset mode was entered previously. An error status change (see status register; Table 14), an error warning or an error passive interrupt forced by the new register content will not occur, until the reset mode is cancelled again
 */
typedef uint8_t SJA1000_PCAN_RXERR_Register;

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN TX error counter Register (Read/Write, Address: 15, Initial value: 0b00000000)
 * @details The TX error counter register reflects the current value of the transmit error counter.
 * In operating mode this register appears to the CPU as a read only memory. A write access to this register is possible only in reset mode. After a hardware reset this register is initialized to logic 0.
 * If a bus-off event occurs, the TX error counter is initialized to 127 to count the minimum protocol-defined time (128 occurrences of the bus-free signal). Reading the TX error counter during this time gives information about the status of the bus-off recovery.
 * If bus-off is active, a write access to TXERR in the range from 0 to 254 clears the bus-off flag and the controller will wait for one occurrence of 11 consecutive recessive bits (bus-free) after the reset mode has been cleared.
 * Writing 255 to TXERR allows to initiate a CPU-driven bus-off event. It should be noted that a CPU-forced content change of the TX error counter is only possible, if the reset mode was entered previously. An error or bus status change (see status register; Table 14), an error warning or an error passive interrupt forced by the new register content will not occur until the reset mode is cancelled again.
 * After leaving the reset mode, the new TX counter content is interpreted and the bus-off event is performed in the same way, as if it was forced by a bus error event. That means, that the reset mode is entered again, the TX error counter is initialized to 127, the RX counter is cleared and all concerned status and interrupt register bits are set.
 * Clearing of reset mode now will perform the protocol-defined bus-off recovery sequence (waiting for 128 occurrences of the bus-free signal).
 * If the reset mode is entered again before the end of bus-off recovery (TXERR > 0), bus-off keeps active and TXERR is frozen.
 */
typedef uint8_t SJA1000_PCAN_TXERR_Register;

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Rx Message counter Register (Read/Write, Address: 29, Initial value: 0b00000000)
 * @details The RMC register reflects the number of messages available within the RXFIFO. The value is incremented with each receive event and decremented by the release receive buffer command. After any reset event, this register is cleared
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_RMC_Register
{
  uint8_t RMC;
  struct
  {
    uint8_t COUNTER: 5; //!< 0-4 - Rx message counter
    uint8_t        : 3; //!< 5-7
  } Bits;
} SJA1000_PCAN_RMC_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_RMC_Register, 1);

#define SJA1000_RX_MSG_COUNTER_Pos         0
#define SJA1000_RX_MSG_COUNTER_Mask        (0x1Fu << SJA1000_RX_MSG_COUNTER_Pos)
#define SJA1000_RX_MSG_COUNTER_GET(value)  (((uint8_t)(value) & SJA1000_RX_MSG_COUNTER_Mask) >> SJA1000_RX_MSG_COUNTER_Pos) //!< Get Rx message counter

//-----------------------------------------------------------------------------

/*! @brief SJA1000-PeliCAN Rx buffer start address register (Read/Write, Address: 30, Initial value: 0b00000000)
 * @details The RBSA register (CAN address 30) reflects the currently valid internal RAM address, where the first byte of the received message, which is mapped to the receive buffer window, is stored.
 * With the help of this information it is possible to interpret the internal RAM contents. The internal RAM address area begins at CAN address 32 and may be accessed by the CPU for reading and writing (writing in reset mode only)
 * Example: if RBSA is set to 24 (decimal), the current message visible in the receive buffer window (CAN address 16 to 28) is stored within the internal RAM beginning at RAM address 24. Because the RAM is also mapped directly to the CAN address space beginning at CAN address 32 (equal to RAM address 0) this message may also be accessed using CAN address 56 and the following bytes (CAN address = RBSA + 32 > 24 + 32 = 56).
 * If a message exceeds RAM address 63, it continues at RAM address 0.
 * The release receive buffer command is always given while there is at least one more message available within the FIFO. RBSA is updated to the beginning of the next message.
 * On hardware reset, this pointer is initialized to '00H'. Upon a software reset (setting of reset mode) this pointer keeps its old value, but the FIFO is cleared; this means that the RAM contents are not changed, but the next received (or transmitted) message will override the currently visible message within the receive buffer window.
 * The RX buffer start address register appears to the CPU as a read only memory in operating mode and as read/write memory in reset mode. It should be noted that a write access to RBSA takes effect first after the next positive edge of the internal clock frequency, which is half of the external oscillator frequency
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_PCAN_RBSA_Register
{
  uint8_t RBSA;
  struct
  {
    uint8_t ADDR: 6; //!< 0-5 - Rx buffer start address
    uint8_t     : 2; //!< 6-7
  } Bits;
} SJA1000_PCAN_RBSA_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_PCAN_RBSA_Register, 1);

#define SJA1000_RX_START_ADDR_Pos         0
#define SJA1000_RX_START_ADDR_Mask        (0x3Fu << SJA1000_RX_START_ADDR_Pos)
#define SJA1000_RX_START_ADDR_GET(value)  (((uint8_t)(value) & SJA1000_RX_START_ADDR_Mask) >> SJA1000_RX_START_ADDR_Pos) //!< Get Rx buffer start address

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Common Controller Registers
//********************************************************************************************************************

/*! @brief SJA1000 Acceptance Code Register (Read/Write, Address: 4, Initial value: 0bXXXXXXXX)
 * @details This register can be accessed (read/write), if the reset request bit is set HIGH (present). When a message is received which passes the acceptance test and there is receive buffer space left, then the respective descriptor and data field are sequentially stored in the RXFIFO.
 * When the complete message has been correctly received the following occurs:
 *  - The receive status bit is set HIGH (full)
 *  - If the receive interrupt enable bit is set HIGH (enabled), the receive interrupt is set HIGH (set)
 * The acceptance code bits (AC.7 to AC.0) and the eight most significant bits of the message's identifier (ID.10 to ID.3) must be equal to those bit positions which are marked relevant by the acceptance mask bits (AM.7 to AM.0).
 * If the conditions as described in the following equation are fulfilled, acceptance is given:
 *   (ID.10 to ID.3) = (AC.7 to AC.0) | (AM.7 to AM.0) = 11111111
 */
typedef uint8_t SJA1000_AC_Register;

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Acceptance Mask Register (Read/Write, Address: 5, Initial value: 0bXXXXXXXX)
 * @details This register can be accessed (read/write), if the reset request bit is set HIGH (present). The acceptance mask register qualifies which of the corresponding bits of the acceptance code are 'relevant' (AM.X = 0) or 'don't care' (AM.X = 1) for acceptance filtering
 */
typedef uint8_t SJA1000_ACM_Register;

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Bus Timing Register 0 (Read/Write, Address: 6, Initial value: 0bXXXXXXXX)
 * @details The contents of the bus timing register 0 defines the values of the Baud Rate Prescaler (BRP) and the Synchronization Jump Width (SJW).
 * This register can be accessed (read/write) if the reset mode is active. In operating mode this register is read only, if the PeliCAN mode is selected. In BasicCAN mode a 'FFH' is reflected
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BTR0_Register
{
  uint8_t BTR0;
  struct
  {
    uint8_t BRP: 6; /*!< 0-5 - Baud Rate Prescaler. The period of the CAN system clock tscl is programmable and determines the individual bit timing
                     *         tscl = 2 * tCLK * (BRP + 1) where tCLK = time period of the XTAL frequency = 1 / fXTAL
                     */
    uint8_t SJW: 2; /*!< 6-7 - Synchronization Jump Width. To compensate for phase shifts between clock oscillators of different bus controllers, any bus controller must re-synchronize on any relevant signal edge of the current transmission.
                     *         The synchronization jump width defines the maximum number of clock cycles a bit period may be shortened or lengthened by one re-synchronization: tSJW = tscl * (SJW + 1)
                     */
  } Bits;
} SJA1000_BTR0_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BTR0_Register, 1);

#define SJA1000_BUS_TIMING0_BRP_Pos         0
#define SJA1000_BUS_TIMING0_BRP_Mask        (0x3Fu << SJA1000_BUS_TIMING0_BRP_Pos)
#define SJA1000_BUS_TIMING0_BRP_SET(value)  (((uint8_t)(value) << SJA1000_BUS_TIMING0_BRP_Pos) & SJA1000_BUS_TIMING0_BRP_Mask) //!< Set Baud Rate Prescaler
#define SJA1000_BUS_TIMING0_SJW_Pos         6
#define SJA1000_BUS_TIMING0_SJW_Mask        (0x3u << SJA1000_BUS_TIMING0_SJW_Pos)
#define SJA1000_BUS_TIMING0_SJW_SET(value)  (((uint8_t)(value) << SJA1000_BUS_TIMING0_SJW_Pos) & SJA1000_BUS_TIMING0_SJW_Mask) //!< Set Synchronization Jump Width

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Bus Timing Register 1 (Read/Write, Address: 7, Initial value: 0bXXXXXXXX)
 * @details The contents of bus timing register 1 defines the length of the bit period, the location of the sample point and the number of samples to be taken at each sample point. This register can be accessed (read/write) if the reset mode is active.
 * In operating mode, this register is read only, if the PeliCAN mode is selected. In BasicCAN mode a 'FFH' is reflected
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_BTR1_Register
{
  uint8_t BTR1;
  struct
  {
    uint8_t TSEG1: 4; //!< 0-3 - Time Segment 1. tTSEG1 = tscl * (8 * TSEG1 + 1)
    uint8_t TSEG2: 3; //!< 4-6 - Time Segment 2. tTSEG2 = tscl * (4 * TSEG2 + 1)
    uint8_t SAMPL: 1; //!< 7   - Sampling: '1' = triple; the bus is sampled three times; recommended for low/medium speed buses (class A and B) where filtering spikes on the bus line is beneficial ; '0' = single; the bus is sampled once; recommended for high speed buses (SAE class C)
  } Bits;
} SJA1000_BTR1_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_BTR1_Register, 1);

#define SJA1000_BUS_TIMING1_TSEG1_Pos         0
#define SJA1000_BUS_TIMING1_TSEG1_Mask        (0xFu << SJA1000_BUS_TIMING1_TSEG1_Pos)
#define SJA1000_BUS_TIMING1_TSEG1_SET(value)  (((uint8_t)(value) << SJA1000_BUS_TIMING1_TSEG1_Pos) & SJA1000_BUS_TIMING1_TSEG1_Mask) //!< Set Time Segment 1
#define SJA1000_BUS_TIMING1_TSEG2_Pos         4
#define SJA1000_BUS_TIMING1_TSEG2_Mask        (0x7u << SJA1000_BUS_TIMING1_TSEG2_Pos)
#define SJA1000_BUS_TIMING1_TSEG2_SET(value)  (((uint8_t)(value) << SJA1000_BUS_TIMING1_TSEG2_Pos) & SJA1000_BUS_TIMING1_TSEG2_Mask) //!< Set Time Segment 2
#define SJA1000_BUS_TIMING1_3_SAMPLES         (0x1u << 7) //!< The bus is sampled three times
#define SJA1000_BUS_TIMING1_1_SAMPLE          (0x0u << 7) //!< The bus is sampled once

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Output Control Register (Read/Write, Address: 8, Initial value: 0bXXXXXXXX)
 * @details The output control register allows the set-up of different output driver configurations under software control.
 * This register may be accessed (read/write) if the reset mode is active. In operating mode, this register is read only, if the PeliCAN mode is selected. In BasicCAN mode a '0xFF' is reflected
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_OCR_Register
{
  uint8_t OCR;
  struct
  {
    uint8_t OCMODE: 2; //!< 0-1 - Output mode
    uint8_t OCT0  : 3; //!< 2-4 - Output pin 0 configuration
    uint8_t OCT1  : 3; //!< 5-7 - Output pin 1 configuration
  } Bits;
} SJA1000_OCR_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_OCR_Register, 1);

//! OC modes enumerator
typedef enum
{
  SJA1000_BIPHASE_OUT_MODE = 0b00, /*!< Bi-phase output mode.
                                    *   In contrast to the normal output mode the bit representation is time variant and toggled.
                                    *   If the bus controllers are galvanically decoupled from the bus line by a transformer, the bit stream is not allowed to contain a DC component.
                                    *   This is achieved by the following scheme. During recessive bits all outputs are deactivated (floating).
                                    *   Dominant bits are sent with alternating levels on TX0 and TX1, i.e. the first dominant bit is sent on TX0, the second is sent on TX1, and the third one is sent on TX0 again, and so on
                                    */
  SJA1000_TEST_OUT_MODE    = 0b01, /*!< Test output mode.
                                    *   In test output mode TXn will reflect the bit, detected on RX pins, with the next positive edge of the system clock. TN1, TN0, TP1 and TP0 are configured in accordance with the setting of OCR.
                                    *   In test output mode the level connected to RX is reflected at TXn with the next positive edge of the system clock fosc/2  corresponding to the programmed polarity in the output control register
                                    */
  SJA1000_NORMAL_OUT_MODE  = 0b10, /*!< Normal output mode.
                                    *   In normal output mode the bit sequence (TXD) is sent via TX0 and TX1.
                                    *   The voltage levels on the output driver pins TX0 and TX1 depend on both the driver characteristic programmed by OCTPx, OCTNx (float, pull-up, pull-down, push-pull) and the output polarity programmed by OCPOLx
                                    */
  SJA1000_CLOCK_OUT_MODE   = 0b11, /*!< Clock output mode.
                                    *   For the TX0 pin this is the same as in normal output mode. However, the data stream to TX1 is replaced by the transmit clock (TXCLK).
                                    *   The rising edge of the transmit clock (non-inverted) marks the beginning of a bit period. The clock pulse width is 1 * tscl
                                    */
} eSJA1000_OutMode;

#define SJA1000_OC_MODE_Pos         0
#define SJA1000_OC_MODE_Mask        (0x3u << SJA1000_OC_MODE_Pos)
#define SJA1000_OC_MODE_SET(value)  (((uint8_t)(value) << SJA1000_OC_MODE_Pos) & SJA1000_OC_MODE_Mask) //!< Set output mode

#define SJA1000_TPX_ON   (0x4) //!< TPX is the on-chip output transistor X, connected to Vdd
#define SJA1000_TNX_ON   (0x2) //!< TNX is the on-chip output transistor X, connected to Vss
#define SJA1000_INV_POL  (0x1) //!< Invert polarity of TXX

//! TX pin mode enumerator
typedef enum
{
  SJA1000_TX_FLOATING      = 0x00,                                              //!< Floating TX pin
  SJA1000_TX_PULL_DOWN     = SJA1000_TNX_ON,                                    //!< TX pin pull-down. Low when TXD = 0 and float when TXD = 1
  SJA1000_TX_PULL_DOWN_INV = SJA1000_TNX_ON | SJA1000_INV_POL,                  //!< TX pin pull-down. Low when TXD = 1 and float when TXD = 0
  SJA1000_TX_PULL_UP       = SJA1000_TPX_ON,                                    //!< TX pin pull-up. High when TXD = 1 and float when TXD = 0
  SJA1000_TX_PULL_UP_INV   = SJA1000_TPX_ON | SJA1000_INV_POL,                  //!< TX pin pull-up. High when TXD = 0 and float when TXD = 1
  SJA1000_TX_PUSH_PULL     = SJA1000_TPX_ON | SJA1000_TNX_ON,                   //!< TX pin push_pull. Low when TXD = 0 and High when TXD = 1
  SJA1000_TX_PUSH_PULL_INV = SJA1000_TPX_ON | SJA1000_TNX_ON | SJA1000_INV_POL, //!< TX pin push_pull. Low when TXD = 1 and High when TXD = 0
} eSJA1000_TXpinMode;

#define SJA1000_TX0_PIN_MODE_Pos         2
#define SJA1000_TX0_PIN_MODE_Mask        (0x7u << SJA1000_TX0_PIN_MODE_Pos)
#define SJA1000_TX0_PIN_MODE_SET(value)  (((uint8_t)(value) << SJA1000_TX0_PIN_MODE_Pos) & SJA1000_TX0_PIN_MODE_Mask) //!< Set TX0 pin mode
#define SJA1000_TX1_PIN_MODE_Pos         5
#define SJA1000_TX1_PIN_MODE_Mask        (0x7u << SJA1000_TX1_PIN_MODE_Pos)
#define SJA1000_TX1_PIN_MODE_SET(value)  (((uint8_t)(value) << SJA1000_TX1_PIN_MODE_Pos) & SJA1000_TX1_PIN_MODE_Mask) //!< Set TX1 pin mode

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Clock divider Register (Read/Write, Address: 31, Initial value: 0b00000000 (Intel), 0b00000101 (Motorola))
 * @details The clock divider register controls the CLKOUT frequency for the microcontroller and allows to deactivate the CLKOUT pin.
 * Additionally a dedicated receive interrupt pulse on TX1, a receive comparator bypass and the selection between BasicCAN mode and PeliCAN mode is made here.
 * The default state of the register after hardware reset is divide-by-12 for Motorola mode (00000101) and divide-by-2 for Intel mode (00000000).
 * On software reset (reset request/reset mode) this register is not influenced. The reserved bit (CDR.4) will always reflect a logic 0.
 * The application software should always write a logic 0 to this bit in order to be compatible with future features, which may be 1-active using this bit
 */
SJA1000_PACKITEM
typedef union __SJA1000_PACKED__ SJA1000_CDR_Register
{
  uint8_t CDR;
  struct
  {
    uint8_t CD      : 3; //!< 0-2 - These bits are used to define the frequency at the external CLKOUT pin
    uint8_t CLK_OFF : 1; /*!< 3   - Setting this bit allows the external CLKOUT pin of the SJA1000 to be disabled.
                          *         A write access is possible only in reset mode. If this bit is set, CLKOUT is LOW during sleep mode, otherwise it is HIGH
                          */
    uint8_t         : 1; //!< 4   - This bit cannot be written. During read-out of this register always a zero is
    uint8_t RXINTEN : 1; /*!< 5   - This bit allows the TX1 output to be used as a dedicated receive interrupt output.
                          *         When a received message has passed the acceptance filter successfully, a receive interrupt pulse with the length of one bit time is always output at the TX1 pin (during the last bit of end of frame).
                          *         The transmit output stage should operate in normal output mode. The polarity and output drive are programmable via the output control register. A write access is only possible in reset mode
                          */
    uint8_t CBP     : 1; /*!< 6   - Setting of this bit allows to bypass the CAN input comparator and is only possible in reset mode.
                          *         This is useful in the event that the SJA1000 is connected to an external transceiver circuit.
                          *         The internal delay of the SJA1000 is reduced, which will result in a longer maximum possible bus length. If CBP is set, only RX0 is active.
                          *         The unused RX1 input should be connected to a defined level (e.g. VSS)
                          */
    uint8_t CAN_MODE: 1; /*!< 7   - This bit defines the CAN mode. If at logic 0 the CAN controller operates in BasicCAN mode.
                          *         If set to logic 1 the CAN controller operates in PeliCAN mode. Write access is only possible in reset mode
                          */
  } Bits;
} SJA1000_CDR_Register;
SJA1000_UNPACKITEM;
SJA1000_CONTROL_ITEM_SIZE(SJA1000_CDR_Register, 1);

#define SJA1000_CDR_CLOCK_OFF                  (0x1u << 3) //!< Clock off
#define SJA1000_CDR_CLOCK_ON                   (0x0u << 3) //!< Clock on

//! CLKOUT frequency selection enumerator
typedef enum
{
  SJA1000_CLKOUT_OFF         = SJA1000_CDR_CLOCK_OFF,        //!< CLKOUT is OFF
  SJA1000_CLKOUT_FOSC_DIV_2  = SJA1000_CDR_CLOCK_ON | 0b000, //!< CLKOUT is fosc div by 2
  SJA1000_CLKOUT_FOSC_DIV_4  = SJA1000_CDR_CLOCK_ON | 0b001, //!< CLKOUT is fosc div by 4
  SJA1000_CLKOUT_FOSC_DIV_6  = SJA1000_CDR_CLOCK_ON | 0b010, //!< CLKOUT is fosc div by 6
  SJA1000_CLKOUT_FOSC_DIV_8  = SJA1000_CDR_CLOCK_ON | 0b011, //!< CLKOUT is fosc div by 8
  SJA1000_CLKOUT_FOSC_DIV_10 = SJA1000_CDR_CLOCK_ON | 0b100, //!< CLKOUT is fosc div by 10
  SJA1000_CLKOUT_FOSC_DIV_12 = SJA1000_CDR_CLOCK_ON | 0b101, //!< CLKOUT is fosc div by 12
  SJA1000_CLKOUT_FOSC_DIV_14 = SJA1000_CDR_CLOCK_ON | 0b110, //!< CLKOUT is fosc div by 14
  SJA1000_CLKOUT_FOSC        = SJA1000_CDR_CLOCK_ON | 0b111, //!< fosc is the frequency of the external oscillator (XTAL)
} eSJA1000_CLKOUTpinMode;

#define SJA1000_CDR_CLKOUT_Pos                 0
#define SJA1000_CDR_CLKOUT_Mask                (0x1Fu << SJA1000_CDR_CLKOUT_Pos)
#define SJA1000_CDR_CLKOUT_SET(value)          (((uint8_t)(value) << SJA1000_CDR_CLKOUT_Pos) & SJA1000_CDR_CLKOUT_Mask) //!< Set CLKOUT frequency
#define SJA1000_CDR_TX1_AS_RX_INT              (0x1u << 5) //!< TX1 output to be used as a dedicated receive interrupt output
#define SJA1000_CDR_TX1_AS_TX                  (0x0u << 5) //!< TX1 pin as TX
#define SJA1000_CDR_BYPASS_CAN_INPUT_COMP_EN   (0x1u << 6) //!< Enable bypass CAN input comparator
#define SJA1000_CDR_BYPASS_CAN_INPUT_COMP_DIS  (0x0u << 6) //!< Disable bypass CAN input comparator
#define SJA1000_CDR_PELI_CAN_MODE              (0x1u << 7) //!< CAN controller operates in PeliCAN mode
#define SJA1000_CDR_BASIC_CAN_MODE             (0x0u << 7) //!< CAN controller operates in BasicCAN mode

//-----------------------------------------------------------------------------

#ifdef SJA1000_BOTH_CAN_DEFINED
//! CAN types enumerator
typedef enum
{
  SJA1000_BasicCAN, //!< Use the BasicCAN
  SJA1000_PeliCAN,  //!< Use the PeliCAN
} eSJA1000_CANtype;
#endif

//-----------------------------------------------------------------------------

#ifdef SJA1000_BOTH_MODE_DEFINED
//! Communication modes enumerator
typedef enum
{
  SJA1000_INTEL_MODE,    //!< Use the intel mode
  SJA1000_MOTOROLA_MODE, //!< Use the motorola mode
} eSJA1000_CommMode;
#endif

//-----------------------------------------------------------------------------

static const char* const SJA1000_DeviceName = "SJA1000";

//-----------------------------------------------------------------------------

//! Available FIFO/Buffer list
typedef enum
{
  SJA1000_TEF       = -1, //!< TEF - Transmit Event FIFO
  SJA1000_RX_FIFO   =  0, //!< RX FIFO
  SJA1000_TX_BUFFER =  1, //!< TX buffer
  SJA1000_NO_FIFO_BUFF,   //!< No specific FIFO/Buffer
} eSJA1000_FIFObuffer;

//-----------------------------------------------------------------------------

//! Data Field Size enumerator
typedef enum
{
  SJA1000_0_BYTES, //!< 0-byte data field
  SJA1000_1_BYTES, //!< 1-byte data field
  SJA1000_2_BYTES, //!< 2-byte data field
  SJA1000_3_BYTES, //!< 3-byte data field
  SJA1000_4_BYTES, //!< 4-byte data field
  SJA1000_5_BYTES, //!< 5-byte data field
  SJA1000_6_BYTES, //!< 6-byte data field
  SJA1000_7_BYTES, //!< 7-byte data field
  SJA1000_8_BYTES, //!< 8-byte data field
  SJA1000_PAYLOAD_COUNT,  // Keep last
} eSJA1000_PayloadSize;

static const uint8_t SJA1000_PAYLOAD_TO_VALUE[SJA1000_PAYLOAD_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 FIFO/Buffers
//********************************************************************************************************************

// FIFO/Buffers definitions
#define SJA1000_TEF_MAX        ( 0 ) //!< 0 TEF maximum
#define SJA1000_TXQ_MAX        ( 0 ) //!< 0 TXQ maximum
#define SJA1000_TX_BUFFER_MAX  ( 1 ) //!< 1 Tx buffer maximum
#define SJA1000_RX_FIFO_MAX    ( 1 ) //!< 1 Rx FIFO maximum
#define SJA1000_FIFO_CONF_MAX  ( SJA1000_TEF_MAX + SJA1000_TXQ_MAX + SJA1000_TX_BUFFER_MAX + SJA1000_RX_FIFO_MAX ) //!< Maximum 2 FIFO configurable (TEF + TXQ + Tx buffer + 1 Rx FIFO)
#define SJA1000_TX_MAX         ( SJA1000_TXQ_MAX + SJA1000_TX_BUFFER_MAX ) //!< Maximum 1 transmit (TXQ + Tx buffer)
#define SJA1000_RX_MAX         ( SJA1000_TEF_MAX + SJA1000_RX_FIFO_MAX )   //!< Maximum 1 receive (TEF + 1 Rx FIFO)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Filter
//********************************************************************************************************************

// Filters definitions
#define SJA1000_SID_FILTERS_MAX  ( 2 ) //!< 2 Standard filters elements maximum
#define SJA1000_EID_FILTERS_MAX  ( 2 ) //!< 1 Extended or 2 partial filters elements maximum
#define SJA1000_FILTERS_MAX      ( SJA1000_SID_FILTERS_MAX ) //!< 2 SID or 2 partial EID filters elements maximum

//-----------------------------------------------------------------------------

#define SJA1000_ACCEPT_ALL_MESSAGES  ( 0x00000000u ) //!< Indicate that the filter will accept all messages
#define SJA1000_ACCEPT_ALL_BITS      ( 0x00000000u ) //!< Indicate that the filter will accept all ID bits in #AcceptanceIDmask, and will accept all Data bits in #AcceptanceDataMask

//-----------------------------------------------------------------------------

/*! @brief SJA1000 Filter configuration structure
 * @details Filters act differently following if there are 1 or 2 filters, and the CAN mode used.
 * For BasicCAN:
 *  - There is only one filter and it is a Standard filter (SID) only
 *  - Filter only the 8 MSBits of the SID
 *  - No data acceptance will be used
 *  - No RTR filter will be used
 * For PeliCAN:
 *  - Single filter mode (filter configuration one long filter (4-bytes)):
 *    - For Standard frames: SID+RTR and the first 2 bytes of the payload. If no payload, the frame will be accepted
 *    - For Extended frames: SID+EID+RTR. #AcceptanceData will not be used
 *  - Dual filter mode (filter configuration two short filters (2 x 2-bytes)):
 *    - For Standard frames: SID+RTR and the first byte of the payload of the first filter only. If no payload, the frame will be accepted
 *    - For Extended frames: SID+EID but only filter the 16 MSBits. #AcceptanceData will not be used
 *  - One filter configuration will filter both Standard and Extended messages:
 *    - Single filter mode (filter configuration one long filter (4-bytes)) the correspondance bits will be:
 * @verbatim
 *      IIIIIIIIIIIIIIIIIIIIIIIIIIIIIR..
 *      22222222211111111110000000000T..
 *      87654321098765432109876543210R..
 *      ||||||||||||||||||||||||||||||||
 *      IIIIIIIIIIIR....DDDDDDDDDDDDDDDD
 *      11000000000T....1111111122222222
 *      09876543210R....7654321076543210
 * @endverbatim
 *    - Dual filter mode (filter configuration two short filters (2 x 2-bytes)) the correspondance bits will be:
 * @verbatim
 *      Remember that SID is 11 bits, EID is 18 bits and SIDEID is 29 bits (where SID is the highest bits of SIDEID). So SID11..0 are SIDEID28..13
 *      IIIIIIIIIIIIIIII  IIIIIIIIIIIIIIII
 *      2222222221111111  2222222221111111
 *      8765432109876543  8765432109876543
 *      ||||||||||||||||  ||||||||||||||||
 *      IIIIIIIIIIIRDDDD  IIIIIIIIIIIRDDDD
 *      11000000000T1111  11000000000T1111 <-- Data byte of the first filter
 *      09876543210R7654  09876543210R3210
 * @endverbatim
 */
typedef struct SJA1000_Filter
{
  //--- Message Filter ---
  uint32_t AcceptanceID;       //!< Message Filter Acceptance SID+EID
  uint32_t AcceptanceIDmask;   //!< Message Filter Mask SID+EID (corresponding bits to #AcceptanceID: '1': bit to filter ; '0' bit that do not care)
#if defined(SJA1000_USE_PELICAN) || defined(SJA1000_BOTH_CAN_DEFINED)
  uint16_t AcceptanceData;     //!< Message filter Acceptance Data bits in big-endian for single filter with standard frames. For dual filter with standard frames, the byte to filter is at LSB
  uint16_t AcceptanceDataMask; //!< Message filter Acceptance Data bits Mask (corresponding bits to #AcceptanceData: '1': bit to filter ; '0' bit that do not care)
  bool ExtendedID;             //!< The message filter is an extended ID
  uint8_t RTRbitValue;         //!< Filter the RTR bit of the frame (bit value): '0' = RTR bit shall be low level ; '1'..'255' = RTR bit shall be high level
  bool FilterRTR;              //!< Filter the RTR bit of the frame (mask): 'true' = filter RTR bit ; 'false' = do not filter RTR bit)
#endif
} SJA1000_Filter;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 Driver API
//********************************************************************************************************************

typedef struct SJA1000 SJA1000; //! Typedef of SJA1000 device object structure

//-----------------------------------------------------------------------------

//! SJA1000 device object structure
struct SJA1000
{
  void *UserDriverData;       //!< Optional, can be used to store driver data or NULL

  //--- Device configuration ---
#ifdef SJA1000_BOTH_CAN_DEFINED
  eSJA1000_CANtype CANtype;   //!< Is the CAN type of the SJA1000 controller
#endif
#ifdef SJA1000_BOTH_MODE_DEFINED
  eSJA1000_CommMode CommMode; //!< Is the communication mode that will be used for with the device (#SJA1000_INTEL_MODE or #SJA1000_MOTOROLA_MODE)
#endif

  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  GPIO_Interface* CS;         //!< This is the GPIO_Interface descriptor pointer that will be used to communicate with the CS pin of the device
  GPIO_Interface* ALE_AS;     //!< This is the GPIO_Interface descriptor pointer that will be used to communicate with the ALE/AS pin of the device
  GPIO_Interface* RD_E;       //!< This is the GPIO_Interface descriptor pointer that will be used to communicate with the RD/E pin of the device
  GPIO_Interface* WR;         //!< This is the GPIO_Interface descriptor pointer that will be used to communicate with the WR pin of the device
  PORT_Interface* DATA;       //!< This is the PORT_Interface descriptor pointer that will be used to communicate with the DATA port of the device
#else
  GPIO_Interface CS;          //!< This is the GPIO_Interface descriptor that will be used to communicate with the CS pin of the device
  GPIO_Interface ALE_AS;      //!< This is the GPIO_Interface descriptor that will be used to communicate with the ALE/AS pin of the device
  GPIO_Interface RD_E;        //!< This is the GPIO_Interface descriptor that will be used to communicate with the RD/E pin of the device
  GPIO_Interface WR;          //!< This is the GPIO_Interface descriptor that will be used to communicate with the WR pin of the device
  PORT_Interface DATA;        //!< This is the GPIO_Interface descriptor that will be used to communicate with the DATA port of the device
#endif
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define SJA1000_UNIQUE_ID  ( (((uint32_t)'S' << 0) ^ ((uint32_t)'J' << 4) ^ ((uint32_t)'A' << 8) ^ ((uint32_t)'1' << 14) ^ ((uint32_t)'0' << 18) ^ ((uint32_t)'0' << 22) ^ ((uint32_t)'0' << 26)) + (sizeof(struct SJA1000) << 19) )

//-----------------------------------------------------------------------------

//! SJA1000 Configuration structure
typedef struct SJA1000_Config
{
  //--- Controller clocks ---
  uint32_t XtalFreq;                 //!< Component CLKIN Xtal/Resonator frequency (max 24MHz)

  //--- CAN configuration ---
#if defined(SJA1000_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_CAN20busConfig BusConfig;      //!< CAN Bus configuration
  CAN_BitTimeStats *BitTimeStats;    //!< Point to a Bit Time stat structure (set to NULL if no statistics are necessary)
#else
  CAN_BitTimeConfig BitTimeConfig;   //!< BitTime configuration
#endif

  //--- Pins configuration ---
  eSJA1000_OutMode outMode;          //!< Set the OC mode of the TX and RX pins
  eSJA1000_TXpinMode tx0PinMode;     //!< Set the TX0 output pin mode
  eSJA1000_TXpinMode tx1PinMode;     //!< Set the TX1 output pin mode
  eSJA1000_CLKOUTpinMode clkoutMode; //!< Set the CLKOUT output pin mode
  bool tx1AsRxInt;                   //!< If 'true', then TX1 pin as Rx interrupt else TX1 as set with #tx1PinMode
  bool bypassRxComp;                 //!< If 'true', then the bypass of the input comparator is enabled and RX1 is disabled else the bypass is disabled and RX1 act as normal

  //--- Interrupts ---
  setSJA1000_Interrupts Interrupts;  //!< Is the set of events where interrupts will be enabled. Flags can be OR'ed
} SJA1000_Config;

//-----------------------------------------------------------------------------


/*! @brief SJA1000 initialization
 *
 * This function initializes the SJA1000 driver and call the initialization of the interface driver (I2C).
 * Next it checks parameters and configures the SJA1000
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @param[in] *pConf Is the pointed structure of the device configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_SJA1000(SJA1000 *pComp, const SJA1000_Config* pConf);

//-----------------------------------------------------------------------------


/*! @brief Read data from register of the SJA1000 device
 *
 * This function reads data from a register of a SJA1000 device
 * @param[in] *pComp Is the pointed structure of the device to read
 * @param[in] reg Is the register to read
 * @param[in] *data Is where the data byte will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ReadRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t* const data);

/*! @brief Write data to register of the SJA1000 device
 *
 * This function writes data to a register of a SJA1000 device
 * @param[in] *pComp Is the pointed structure of the device to modify
 * @param[in] reg Is the register where data will be written
 * @param[in] data Is the data byte to store
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_WriteRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t data);

/*! @brief Modify a register of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] reg Is the register address where data will be written
 * @param[in] data Is the data to write
 * @param[in] mask If the bit is set to '1', then the corresponding register's bit have to be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ModifyRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t data, uint8_t mask);

//********************************************************************************************************************


/*! @brief Transmit a message object (with data) to the Buffer of the SJA1000 device
 *
 * Transmit the message to the Buffer. This function uses the specific format of the component (INF, ID1, ID2[, ID3, ID4], DATA).
 * This function sends the message object, and update asks for transmission
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageObjectToSend Is the message object to send with all its data
 * @param[in] objectSize Is the size of the message object (with its data)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_TransmitMessageObject(SJA1000 *pComp, const uint8_t* messageObjectToSend, uint8_t objectSize);

/*! @brief Transmit a message to the Buffer of the SJA1000 device
 *
 * Transmit the message to the Buffer
 * This function sends the message object, and update asks for transmission
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageToSend Is the message to send with all the data attached with
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_TransmitMessage(SJA1000 *pComp, CAN_CANMessage* const messageToSend);


/*! @brief Receive a message object (with data) from the FIFO of the SJA1000 device
 *
 * Receive the message from the FIFO. This function uses the specific format of the component (INF, ID1, ID2[, ID3, ID4], DATA).
 * This function gets the message object, and update the tail pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageObjectGet Is the message object retrieve with all its data
 * @param[in] objectSize Is the size of the message object (with its data)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ReceiveMessageObject(SJA1000 *pComp, uint8_t* messageObjectGet, uint8_t objectSize);

/*! @brief Receive a message from the FIFO of the SJA1000 device
 *
 * Receive a message from the FIFO
 * This function gets the message object from, and update the tail pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageGet Is the message retrieve with all the data attached with
 * @param[in] payloadSize Indicate the payload of the FIFO (0..8 bytes)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ReceiveMessage(SJA1000 *pComp, CAN_CANMessage* const messageGet, eSJA1000_PayloadSize payloadSize);

//********************************************************************************************************************


/*! @brief Configure pins of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be configured
 * @param[in] outMode Set the OC mode of the TX and RX pins
 * @param[in] tx0PinMode Set the TX0 output pin mode
 * @param[in] tx1PinMode Set the TX1 output pin mode
 * @param[in] clkoutMode Set the CLKOUT output pin mode
 * @param[in] tx1AsRxInt If 'true', then TX1 pin as Rx interrupt else TX1 as set with #tx1PinMode
 * @param[in] bypassRxComp If 'true', then the bypass of the input comparator is enabled and RX1 is disabled else the bypass is disabled and RX1 act as normal
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ConfigurePins(SJA1000 *pComp, eSJA1000_OutMode outMode, eSJA1000_TXpinMode tx0PinMode, eSJA1000_TXpinMode tx1PinMode, eSJA1000_CLKOUTpinMode clkoutMode, bool tx1AsRxInt, bool bypassRxComp);

//********************************************************************************************************************


#if defined(SJA1000_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
/*! @brief Calculate Bit Time for CAN2.0 Configuration for the SJA1000
 *
 * Calculate the best Bit Time configuration following desired bitrates for CAN-2.0
 * This function call automatically the SJA1000_CalculateBitrateStatistics() function
 * @param[in] periphClk Is the clock of the device
 * @param[in] pBusConf Is the bus configuration of the CAN-bus
 * @param[out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CAN20busConfig pBusConf, struct CAN_BitTimeConfig* const pConf);

/*! @brief Calculate Bitrate Statistics of a Bit Time configuration
 *
 * Calculate bus length, sample points, bitrates and oscillator tolerance following BitTime Configuration
 * @param[in] periphClk Is the SYSCLK of the SJA1000
 * @param[in,out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_CalculateBitrateStatistics(const uint32_t periphClk, CAN_BitTimeConfig* const pConf);
#endif

/*! @brief Set Bit Time Configuration to the SJA1000
 *
 * Set the Nominal and Data Bit Time to registers
 * @param[in] *pComp Is the pointed structure of the SJA1000 to be used
 * @param[in] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_SetBitTimeConfiguration(SJA1000 *pComp, const CAN_BitTimeConfig* const pConf);

//********************************************************************************************************************


/*! @brief Get actual operation mode of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *actualMode Is where the result of the actual mode will be saved
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_GetActualOperationMode(SJA1000 *pComp, eSJA1000_OperationMode* const actualMode);

/*! @brief Request operation mode change of the SJA1000 device
 *
 * @note Configuration write protection will be set but not for SJA1000_INITIALISATION_MODE mode
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] newMode Is the new operational mode to set
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_RequestOperationMode(SJA1000 *pComp, eSJA1000_OperationMode newMode);

/*! @brief Start the SJA1000 device in CAN2.0 mode
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_StartCAN20(SJA1000 *pComp)
{
  return SJA1000_RequestOperationMode(pComp, SJA1000_MODE_NORMAL);
}

//********************************************************************************************************************


/*! @brief Configure the filters of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *confFilter1 Is the configuration structure of the Filter 1
 * @param[in] *confFilter2 Is the configuration structure of the Filter 2. Set to NULL if only use 1 filter (mandatory for the use of BasicCAN)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ConfigureFilters(SJA1000 *pComp, SJA1000_Filter* const confFilter1, SJA1000_Filter* const confFilter2);

//********************************************************************************************************************


/*! @brief Enter the SJA1000 device in sleep mode
 *
 * This function puts the device in sleep mode
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_EnterSleepMode(SJA1000 *pComp);

/*! @brief Verify if the SJA1000 device is in sleep mode
 *
 * This function verifies if the device is in sleep mode by checking the CMR.GTS
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *isInSleepMode Indicate if the device is in sleep mode
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_IsDeviceInSleepMode(SJA1000 *pComp, bool* const isInSleepMode);

/*! @brief Manually wake up the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_WakeUp(SJA1000 *pComp);

//********************************************************************************************************************


/*! @brief Configure interrupt of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be enabled. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ConfigureInterrupt(SJA1000 *pComp, setSJA1000_Interrupts interruptsFlags);

/*! @brief Get interrupt events of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *interruptsFlags Is the return value of interrupt events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_GetInterruptEvents(SJA1000 *pComp, setSJA1000_InterruptEvents* const interruptsFlags);

/*! @brief Clear interrupt events of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be cleared. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_ClearInterruptEvents(SJA1000 *pComp, setSJA1000_InterruptEvents interruptsFlags);

/*! @brief Get status events of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *statusFlags Is the return value of status events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetStatusEvents(SJA1000 *pComp, setSJA1000_StatusEvents* const statusFlags)
{
  return SJA1000_ReadRegister(pComp, RegSJA1000_CAN_STATUS, (uint8_t*)&statusFlags);
}

//-----------------------------------------------------------------------------


/*! @brief Clear FIFO overrun event of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_ClearFIFOoverrunEvent(SJA1000 *pComp)
{
  return SJA1000_WriteRegister(pComp, RegSJA1000_CAN_COMMAND, SJA1000_COMMAND_CLEAR_DATA_OVERRUN);
}

/*! @brief Abort transmission of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_AbortTransmission(SJA1000 *pComp)
{
  return SJA1000_WriteRegister(pComp, RegSJA1000_CAN_COMMAND, SJA1000_COMMAND_ABORT_TX);
}

//********************************************************************************************************************


/*! @brief Get the last arbitration lost bit position of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *arbitrationLost Is the return value of the last arbitration lost
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetLastArbitrationLostBitPos(SJA1000 *pComp, eSJA1000_ArbitrationLost* const arbitrationLost)
{
  eERRORRESULT Error;
  uint8_t Config;
  Error = SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_ARBITRATION_LOST, &Config);
  *arbitrationLost = SJA1000_ARBITRATION_LOST_GET(Config);
  return Error;
}

/*! @brief Get the last error capture of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *errorSegment Is the return value of the last error capture segment
 * @param[out] *errorDirection Is the return value of the last error capture direction
 * @param[out] *errorCode Is the return value of the last error capture code
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT SJA1000_GetLastErrorCapture(SJA1000 *pComp, eSJA1000_ErrorSegment* const errorSegment, eSJA1000_ErrorDirection* const errorDirection, eSJA1000_ErrorCode* const errorCode);

/*! @brief Get Bus diagnostic of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorWarningLimit Is the return value of the warning limit
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetBusErrorWarningLimit(SJA1000 *pComp, uint8_t* const errorWarningLimit)
{
#ifdef CHECK_NULL_PARAM
  if (errorWarningLimit == NULL) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  return SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_ERROR_WARNING, errorWarningLimit);
}

/*! @brief Get Rx Error Counter of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorCounter Is the return value of the error counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetRxErrorCounter(SJA1000 *pComp, uint8_t* const errorCounter)
{
#ifdef CHECK_NULL_PARAM
  if (errorCounter == NULL) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  return SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_RX_ERROR_COUNTER, errorCounter);
}

/*! @brief Get Tx Error Counter of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *errorCounter Is the return value of the error counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetTxErrorCounter(SJA1000 *pComp, uint8_t* const errorCounter)
{
#ifdef CHECK_NULL_PARAM
  if (errorCounter == NULL) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  return SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_TX_ERROR_COUNTER, errorCounter);
}

//-----------------------------------------------------------------------------


/*! @brief Get Rx Message Counter of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *messageCounter Is the return value of the message counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetRxMessageCounter(SJA1000 *pComp, uint8_t* const messageCounter)
{
#ifdef CHECK_NULL_PARAM
  if (messageCounter == NULL) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  return SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_RX_MSG_COUNTER, messageCounter);
}


/*! @brief Get Rx Start Address of the SJA1000 device
 *
 * @param[in] *pComp Is the pointed structure of the device use
 * @param[out] *rxStartAddr Is the return value of the Rx start address
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT SJA1000_GetRxStartAddress(SJA1000 *pComp, uint8_t* const rxStartAddr)
{
#ifdef CHECK_NULL_PARAM
  if (rxStartAddr == NULL) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  eERRORRESULT Error;
  Error = SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_RX_START_ADDR, rxStartAddr);
  *rxStartAddr = SJA1000_RX_START_ADDR_GET(*rxStartAddr) + SJA1000_RAM_ADDR;
  return Error;
}

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* SJA1000_H_INC */