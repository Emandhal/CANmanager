/*!*****************************************************************************
 * @file    MCAN_core.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/05/2021
 * @brief   Bosch MCAN core registers
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
#ifndef MCAN_CORE_H_INC
#define MCAN_CORE_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#ifdef __cplusplus

#  define MCAN_MEMBER(name)
#  if defined(_MSC_VER)
#    define MCAN_WEAK
#    define MCAN_EXTERN             extern
#  else
#    define MCAN_WEAK               __attribute__((weak))
#    define MCAN_EXTERN
#  endif
#  define __MCAN_PACKED__
#  define MCAN_PACKITEM             __pragma(pack(push, 1))
#  define MCAN_UNPACKITEM           __pragma(pack(pop))
#  define MCAN_PACKENUM(name,type)  typedef enum name : type
#  define MCAN_UNPACKENUM(name)     name
extern "C" {
  
#else

#  define MCAN_MEMBER(name)         .name =
#  define MCAN_WEAK                 __attribute__((weak))
#  define MCAN_EXTERN
#  define __MCAN_PACKED__           __attribute__((packed))
#  define MCAN_PACKITEM
#  define MCAN_UNPACKITEM
#  define MCAN_PACKENUM(name,type)  typedef enum __MCAN_PACKED__
#  define MCAN_UNPACKENUM(name)     name

#endif
//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define MCAN_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------

//! 8-bit DCB to decimal without error checking
#define MCAN_DCB8_TO_DECIMAL(dcb)  ( (uint8_t)(dcb) - (6u * ((uint8_t)(dcb) >> 4u)) )

//! Endian swap
#ifdef __GNUC__
    // Optimized 32-bit bytes swap
#  define MCAN_SWAP32(x)  ({ uint32_t z = (x); \
                          (uint32_t)(((z & 0xff) << 24) | ((z & 0xff00) << 8) | ((z >> 8) & 0xff00) | ((z >> 24) & 0xff)); })
#else
#  define MCAN_SWAP32(x)  ( (uint32_t)((((int32_t)(x) & 0xff) << 24) | (((int32_t)(x) & 0xff00) << 8) | (((int32_t)(x) >> 8) & 0xff00) | (((int32_t)(x) >> 24) & 0xff)) )
#endif

//-----------------------------------------------------------------------------

#if !defined(MCAN_MAX_POSSIBLE_RAM)
#  error MCAN_MAX_POSSIBLE_RAM shall be defined
#endif

#ifdef MCAN_INTERNAL_CAN_CONTROLLER
#  define MCAN_RAM_START_ADDRESS  ( (uint32_t)pComp->RAMallocation )
#  define MCAN_MAX_RAM            ( MCAN_MAX_POSSIBLE_RAM )
#  define MCAN_RAM_END_ADDRESS    ( MCAN_RAM_START_ADDRESS + MCAN_MAX_RAM )
#else // MCAN_EXTERNAL_CAN_CONTROLLER
#  if !defined(MCAN_DEVICE_RAM_START_ADDRESS) && !defined(MCAN_INTERNAL_CAN_CONTROLLER)
#    error MCAN_DEVICE_RAM_START_ADDRESS shall be defined
#  endif
#  define MCAN_RAM_START_ADDRESS  ( MCAN_DEVICE_RAM_START_ADDRESS )
#  define MCAN_MAX_RAM            ( MCAN_MAX_POSSIBLE_RAM )
#  define MCAN_RAM_END_ADDRESS    ( MCAN_RAM_START_ADDRESS + MCAN_MAX_RAM )
#endif

#define MCAN_BASE_ADDRESS_MASK     ( 0xFFFF0000 ) //!< Base address mask for RAM address reconstitution
#define MCAN_ADDRESS32_ALIGN_MASK  ( 0xFFFFFFFC ) //!< Base 32-bits addresswith alignment mask for RAM address

//-----------------------------------------------------------------------------

//! Sysclock divider. On some MCU the sysclock is divided before entering the MCAN peripheral
#ifdef MCAN_SYSTEM_CLOCK_DIVIDER
#  define MCAN_PERIPHERAL_DIV  ( MCAN_SYSTEM_CLOCK_DIVIDER )
#else // !defined(MCAN_SYSCLOCK_DIV)
#  define MCAN_PERIPHERAL_DIV  ( 1 )
#endif

//! Minimum peripheral clock of the MCAN peripheral
#ifdef MCAN_CLOCK_MIN
#  define MCAN_PERIPHERAL_CLK_MIN  ( MCAN_CLOCK_MIN )
#else // !defined(MCAN_CLOCK_MIN)
#  define MCAN_PERIPHERAL_CLK_MIN  ( 1000000 )
#endif

//! Maximum peripheral clock of the MCAN peripheral
#ifdef MCAN_CLOCK_MAX
#  define MCAN_PERIPHERAL_CLK_MAX  ( MCAN_CLOCK_MAX )
#else // !defined(MCAN_CLOCK_MAX)
#  define MCAN_PERIPHERAL_CLK_MAX  ( 80000000 )
#endif

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN Filters Objects
//********************************************************************************************************************

#define MCAN_CAN_FILTER_SID_MASK  ( 0x7FF )


//! Filter for Rx Buffers or for debug messages enumerator
typedef enum
{
  MCAN_STORE_IN_RX_BUFFER       = 0b00, //!< Store message in a Rx buffer
  MCAN_TREAT_AS_DEBUG_MESSAGE_A = 0b01, //!< Treated as debug message A
  MCAN_TREAT_AS_DEBUG_MESSAGE_B = 0b10, //!< Treated as debug message B
  MCAN_TREAT_AS_DEBUG_MESSAGE_C = 0b11, //!< Treated as debug message C
} eMCAN_FilterDebugMessage;


//! Standard and Extended Filter Element Configuration enumerator
typedef enum
{
  MCAN_DISABLE_FILTER                  = 0b000, //!< Disable filter element
  MCAN_STORE_TO_RX_FIFO_0              = 0b001, //!< Store in Rx FIFO 0 if filter matches
  MCAN_STORE_TO_RX_FIFO_1              = 0b010, //!< Store in Rx FIFO 1 if filter matches
  MCAN_REJECT_ID                       = 0b011, //!< Reject ID if filter matches
  MCAN_SET_PRIORITY                    = 0b100, //!< Set priority if filter matches
  MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_0 = 0b101, //!< Set priority and store in FIFO 0 if filter matches
  MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_1 = 0b110, //!< Set priority and store in FIFO 1 if filter matches
  MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG = 0b111, //!< Store into Rx Buffer or as debug message, configuration of SFT[1:0] ignored
} eMCAN_FilterElementConfig;


//! Standard and Extended Filter Type enumerator
typedef enum
{
  MCAN_RANGE_FROM_FID1_TO_FDI2         = 0b00, //!< Range filter from F1ID to F2ID (F2ID ≥ F1ID)
  MCAN_DUAL_ID_FILTER_FID1_OR_FDI2     = 0b01, //!< Dual ID filter for F1ID or F2ID
  MCAN_FID1_IS_FILTER_FID2_IS_MASK     = 0b10, //!< Classic filter: F1ID = filter, F2ID = mask
  MCAN_RANGE_FROM_FID1_TO_FDI2_NO_MASK = 0b11, //!< [Extended Filter Only] Range filter from F1ID to F2ID (F2ID ≥ F1ID), MCAN_XIDAM mask not applied
} eMCAN_FilterType;

//-----------------------------------------------------------------------------

//! Standard Message ID Filter Element (S0)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_StandardFilterObject
{
  uint32_t S0;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    union
    {
      struct                //!< SFEC = '001'..'110' – Second ID of standard ID filter element
      {
        uint16_t SFID2: 11; //!<  0-10 - Standard Filter ID 2. This field has a different meaning depending on the configuration of SFEC
        uint16_t      :  5; //!< 11-15
      };
      struct                //!< SFEC = '111' – Filter for Rx Buffers or for debug messages
      {
        uint16_t RXID :  6; //!<  0- 5 - Index of the dedicated Rx Buffer element to which a matching message is stored
        uint16_t      :  3; //!<  6- 8
        uint16_t DBMSG:  2; //!<  9-10 - Decides whether the received message is stored into an Rx Buffer or treated as message A, B, or C of the debug message sequence
        uint16_t      :  5; //!< 11-15
      };
    };
    uint16_t SFID1: 11; //!< 16-26 - Standard Filter ID 1. First ID of standard ID filter element. When filtering for Rx Buffers or for debug messages this field defines the ID of a standard message to be stored. The received identifiers must match exactly, no masking mechanism is used
    uint16_t SFEC :  3; //!< 27-29 - Standard Filter Element Configuration. All enabled filter elements are used for acceptance filtering of standard frames. Acceptance filtering stops at the first matching enabled filter element or when the end of the filter list is reached. If SFEC = '0b100', '0b101', or '0b110' a match sets interrupt flag MCAN_IR.HPM and, if enabled, an interrupt is generated. In this case register HPMS is updated with the status of the priority match
    uint16_t SFT  :  2; //!< 30-31 - Standard Filter Type
  };
} MCAN_StandardFilterObject;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_StandardFilterObject, 4);

#define MCAN_CAN_FILTS0_SFID2_Pos         0
#define MCAN_CAN_FILTS0_SFID2_Mask        (MCAN_CAN_FILTER_SID_MASK << MCAN_CAN_FILTS0_SFID2_Pos)
#define MCAN_CAN_FILTS0_SFID2_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTS0_SFID2_Pos) & MCAN_CAN_FILTS0_SFID2_Mask) //!< Set Standard Filter ID 2
#define MCAN_CAN_FILTS0_SFID2_GET(value)  (((uint32_t)(value) & MCAN_CAN_FILTS0_SFID2_Mask) >> MCAN_CAN_FILTS0_SFID2_Pos) //!< Get Standard Filter ID 2
#define MCAN_CAN_FILTS0_RXID_Pos          0
#define MCAN_CAN_FILTS0_RXID_Mask         (0x3Fu << MCAN_CAN_FILTS0_RXID_Pos)
#define MCAN_CAN_FILTS0_RXID_SET(value)   (((uint32_t)(value) << MCAN_CAN_FILTS0_RXID_Pos) & MCAN_CAN_FILTS0_RXID_Mask) //!< Set index of the dedicated Rx Buffer element to which a matching message is stored
#define MCAN_CAN_FILTS0_RXID_GET(value)   (((uint32_t)(value) & MCAN_CAN_FILTS0_RXID_Mask) >> MCAN_CAN_FILTS0_RXID_Pos) //!< Get index of the dedicated Rx Buffer element to which a matching message is stored
#define MCAN_CAN_FILTS0_DBMSG_Pos         9
#define MCAN_CAN_FILTS0_DBMSG_Mask        (0x3u << MCAN_CAN_FILTS0_DBMSG_Pos)
#define MCAN_CAN_FILTS0_DBMSG_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTS0_DBMSG_Pos) & MCAN_CAN_FILTS0_DBMSG_Mask) //!< Set Filter Debug Message
#define MCAN_CAN_FILTS0_DBMSG_GET(value)  (eMCAN_FilterDebugMessage)(((uint32_t)(value) & MCAN_CAN_FILTS0_DBMSG_Mask) >> MCAN_CAN_FILTS0_DBMSG_Pos) //!< Get Filter Debug Message
#define MCAN_CAN_FILTS0_SFID1_Pos         16
#define MCAN_CAN_FILTS0_SFID1_Mask        (MCAN_CAN_FILTER_SID_MASK << MCAN_CAN_FILTS0_SFID1_Pos)
#define MCAN_CAN_FILTS0_SFID1_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTS0_SFID1_Pos) & MCAN_CAN_FILTS0_SFID1_Mask) //!< Set Standard Filter ID 1
#define MCAN_CAN_FILTS0_SFID1_GET(value)  (((uint32_t)(value) & MCAN_CAN_FILTS0_SFID1_Mask) >> MCAN_CAN_FILTS0_SFID1_Pos) //!< Get Standard Filter ID 1
#define MCAN_CAN_FILTS0_SFEC_Pos          27
#define MCAN_CAN_FILTS0_SFEC_Mask         (0x7u << MCAN_CAN_FILTS0_SFEC_Pos)
#define MCAN_CAN_FILTS0_SFEC_SET(value)   (((uint32_t)(value) << MCAN_CAN_FILTS0_SFEC_Pos) & MCAN_CAN_FILTS0_SFEC_Mask) //!< Set Standard Filter Element Configuration
#define MCAN_CAN_FILTS0_SFEC_GET(value)   (eMCAN_FilterElementConfig)(((uint32_t)(value) & MCAN_CAN_FILTS0_SFEC_Mask) >> MCAN_CAN_FILTS0_SFEC_Pos) //!< Get Standard Filter Element Configuration
#define MCAN_CAN_FILTS0_SFT_Pos           30
#define MCAN_CAN_FILTS0_SFT_Mask          (0x3u << MCAN_CAN_FILTS0_SFT_Pos)
#define MCAN_CAN_FILTS0_SFT_SET(value)    (((uint32_t)(value) << MCAN_CAN_FILTS0_SFT_Pos) & MCAN_CAN_FILTS0_SFT_Mask) //!< Set Standard Filter Type
#define MCAN_CAN_FILTS0_SFT_GET(value)    (eMCAN_FilterType)(((uint32_t)(value) & MCAN_CAN_FILTS0_SFT_Mask) >> MCAN_CAN_FILTS0_SFT_Pos) //!< Get Standard Filter Type

//-----------------------------------------------------------------------------

#define MCAN_CAN_STANDARD_FILTER_SIZE      ( sizeof(MCAN_StandardFilterObject) )

//-----------------------------------------------------------------------------

#define MCAN_CAN_FILTER_EID_AND_SID_MASK  ( 0x1FFFFFFFu )


//! Extended Message ID Filter Element Identifier 1 (F0)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ExtendedFilterIdentifier1
{
  uint32_t F0;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t EFID1: 29; //!<  0-28 - Extended Filter ID 1. First ID of extended ID filter element. When filtering for Rx Buffers or for debug messages this field defines the ID of an extended message to be stored. The received identifiers must match exactly, only MCAN_XIDAM masking mechanism (see Extended Message ID Filtering) is used
    uint32_t EFEC :  3; //!< 29-31 - Extended Filter Element Configuration. All enabled filter elements are used for acceptance filtering of extended frames. Acceptance filtering stops at the first matching enabled filter element or when the end of the filter list is reached. If EFEC = '100', '101', or '110', a match sets the interrupt flag MCAN_IR.HPM and, if enabled, an interrupt is generated. In this case, register MCAN_HPMS is updated with the status of the priority match
  };
} MCAN_ExtendedFilterIdentifier1;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ExtendedFilterIdentifier1, 4);

#define MCAN_CAN_FILTF0_EFID1_Pos         0
#define MCAN_CAN_FILTF0_EFID1_Mask        (MCAN_CAN_FILTER_EID_AND_SID_MASK << MCAN_CAN_FILTF0_EFID1_Pos)
#define MCAN_CAN_FILTF0_EFID1_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTF0_EFID1_Pos) & MCAN_CAN_FILTS0_SFID1_Mask) //!< Set Extended Filter ID 1
#define MCAN_CAN_FILTF0_EFID1_GET(value)  (((uint32_t)(value) & MCAN_CAN_FILTS0_SFID1_Mask) >> MCAN_CAN_FILTF0_EFID1_Pos) //!< Get Extended Filter ID 1
#define MCAN_CAN_FILTF0_EFEC_Pos          29
#define MCAN_CAN_FILTF0_EFEC_Mask         (0x7u << MCAN_CAN_FILTF0_EFEC_Pos)
#define MCAN_CAN_FILTF0_EFEC_SET(value)   (((uint32_t)(value) << MCAN_CAN_FILTF0_EFEC_Pos) & MCAN_CAN_FILTF0_EFEC_Mask) //!< Set Extended Filter Element Configuration
#define MCAN_CAN_FILTF0_EFEC_GET(value)   (eMCAN_FilterElementConfig)(((uint32_t)(value) & MCAN_CAN_FILTF0_EFEC_Mask) >> MCAN_CAN_FILTF0_EFEC_Pos) //!< Get Extended Filter Element Configuration

//-----------------------------------------------------------------------------

//! Extended Message ID Filter Element Identifier 2 (F1)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ExtendedFilterIdentifier2
{
  uint32_t F1;
  uint8_t Bytes[sizeof(uint32_t)];
  struct                //!< F0.EFEC = '001'..'110' – Second ID of extended ID filter element
  {
    uint32_t EFID2: 29; //!<  0-28 - Extended Filter ID 2. This field has a different meaning depending on the configuration of F0.EFEC
    uint32_t      :  1; //!< 29
    uint32_t EFT  :  2; //!< 30-31 - Extended Filter Type
  };
  struct                //!< F0.EFEC = '111' – Filter for Rx Buffers or for debug messages
  {
    uint32_t RXID :  6; //!<  0- 5 - Index of the dedicated Rx Buffer element to which a matching message is stored
    uint32_t      :  3; //!<  6- 8
    uint32_t DBMSG:  2; //!<  9-10 - Decides whether the received message is stored into an Rx Buffer or treated as message A, B, or C of the debug message sequence
    uint32_t      : 18; //!< 11-28
    uint32_t      :  1; //!< 29
    uint32_t EFT_ :  2; //!< 30-31 - Extended Filter Type
  };
} MCAN_ExtendedFilterIdentifier2;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ExtendedFilterIdentifier2, 4);

#define MCAN_CAN_FILTF1_EFID2_Pos         0
#define MCAN_CAN_FILTF1_EFID2_Mask        (MCAN_CAN_FILTER_EID_AND_SID_MASK << MCAN_CAN_FILTF1_EFID2_Pos)
#define MCAN_CAN_FILTF1_EFID2_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTF1_EFID2_Pos) & MCAN_CAN_FILTF1_EFID2_Mask) //!< Set Extended Filter ID 2
#define MCAN_CAN_FILTF1_EFID2_GET(value)  (((uint32_t)(value) & MCAN_CAN_FILTF1_EFID2_Mask) >> MCAN_CAN_FILTF1_EFID2_Pos) //!< Get Extended Filter ID 2
#define MCAN_CAN_FILTF1_RXID_Pos          0
#define MCAN_CAN_FILTF1_RXID_Mask         (0x3Fu << MCAN_CAN_FILTF1_RXID_Pos)
#define MCAN_CAN_FILTF1_RXID_SET(value)   (((uint32_t)(value) << MCAN_CAN_FILTF1_RXID_Pos) & MCAN_CAN_FILTF1_RXID_Mask) //!< Set index of the dedicated Rx Buffer element to which a matching message is stored
#define MCAN_CAN_FILTF1_RXID_GET(value)   (((uint32_t)(value) & MCAN_CAN_FILTF1_RXID_Mask) >> MCAN_CAN_FILTF1_RXID_Pos) //!< Get index of the dedicated Rx Buffer element to which a matching message is stored
#define MCAN_CAN_FILTF1_DBMSG_Pos         9
#define MCAN_CAN_FILTF1_DBMSG_Mask        (0x3u << MCAN_CAN_FILTF1_DBMSG_Pos)
#define MCAN_CAN_FILTF1_DBMSG_SET(value)  (((uint32_t)(value) << MCAN_CAN_FILTF1_DBMSG_Pos) & MCAN_CAN_FILTF1_DBMSG_Mask) //!< Set Filter Debug Message
#define MCAN_CAN_FILTF1_DBMSG_GET(value)  (eMCAN_FilterDebugMessage)(((uint32_t)(value) & MCAN_CAN_FILTF1_DBMSG_Mask) >> MCAN_CAN_FILTF1_DBMSG_Pos) //!< Get Filter Debug Message
#define MCAN_CAN_FILTF1_EFT_Pos           30
#define MCAN_CAN_FILTF1_EFT_Mask          (0x3u << MCAN_CAN_FILTF1_EFT_Pos)
#define MCAN_CAN_FILTF1_EFT_SET(value)    (((uint32_t)(value) << MCAN_CAN_FILTF1_EFT_Pos) & MCAN_CAN_FILTF1_EFT_Mask) //!< Set Extended Filter Type
#define MCAN_CAN_FILTF1_EFT_GET(value)    (eMCAN_FilterType)(((uint32_t)(value) & MCAN_CAN_FILTF1_EFT_Mask) >> MCAN_CAN_FILTF1_EFT_Pos) //!< Get Extended Filter Type

//-----------------------------------------------------------------------------

#define MCAN_CAN_FILT_F0  0
#define MCAN_CAN_FILT_F1  1

//! Extended Message ID Filter Element
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ExtendedFilterObject
{
  uint32_t Word[2];
  uint8_t Bytes[8];
  struct
  {
    MCAN_ExtendedFilterIdentifier1 F0; //!< Extended Filter Identifier 1 (F0)
    MCAN_ExtendedFilterIdentifier2 F1; //!< Extended Filter Identifier 2 (F1)
  };
} MCAN_ExtendedFilterObject;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ExtendedFilterObject, 8);

//-----------------------------------------------------------------------------

#define MCAN_CAN_EXTENDED_FILTER_SIZE  ( sizeof(MCAN_ExtendedFilterObject) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN TEF, Tx, Rx Messages Objects
//********************************************************************************************************************

//! Data Length Size for the CAN message
typedef enum
{
  MCAN_DLC_0BYTE   = 0b0000, //!< The DLC is  0 data byte
  MCAN_DLC_1BYTE   = 0b0001, //!< The DLC is  1 data byte
  MCAN_DLC_2BYTE   = 0b0010, //!< The DLC is  2 data bytes
  MCAN_DLC_3BYTE   = 0b0011, //!< The DLC is  3 data bytes
  MCAN_DLC_4BYTE   = 0b0100, //!< The DLC is  4 data bytes
  MCAN_DLC_5BYTE   = 0b0101, //!< The DLC is  5 data bytes
  MCAN_DLC_6BYTE   = 0b0110, //!< The DLC is  6 data bytes
  MCAN_DLC_7BYTE   = 0b0111, //!< The DLC is  7 data bytes
  MCAN_DLC_8BYTE   = 0b1000, //!< The DLC is  8 data bytes
  MCAN_DLC_12BYTE  = 0b1001, //!< The DLC is 12 data bytes
  MCAN_DLC_16BYTE  = 0b1010, //!< The DLC is 16 data bytes
  MCAN_DLC_20BYTE  = 0b1011, //!< The DLC is 20 data bytes
  MCAN_DLC_24BYTE  = 0b1100, //!< The DLC is 24 data bytes
  MCAN_DLC_32BYTE  = 0b1101, //!< The DLC is 32 data bytes
  MCAN_DLC_48BYTE  = 0b1110, //!< The DLC is 48 data bytes
  MCAN_DLC_64BYTE  = 0b1111, //!< The DLC is 64 data bytes
  MCAN_DLC_COUNT,            // Keep last
  MCAN_PAYLOAD_MIN =  8,
  MCAN_PAYLOAD_MAX = 64,
} eMCAN_DataLength;

static const uint8_t MCAN20_DLC_TO_VALUE[MCAN_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8,  8,  8,  8,  8,  8,  8,  8};
static const uint8_t MCANFD_DLC_TO_VALUE[MCAN_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
#define MCAN_DLCToByte(dlc, isCANFD)  ( ((isCANFD) ? CANFD_DLC_TO_VALUE[(size_t)(dlc) & 0xF] : CAN20_DLC_TO_VALUE[(size_t)(dlc) & 0xF] )

//-----------------------------------------------------------------------------

//! CAN Transmit Message Identifier (T0)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxMessageIdentifier
{
  uint32_t T0;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ID : 29; //!<  0-28 - Message Identifier. Standard or extended identifier depending on bit XTD. A standard identifier has to be written to ID[28:18]
    uint32_t RTR:  1; //!< 29    - Remote Transmission Request. When RTR = 1, the MCAN transmits a remote frame according to ISO11898-1, even if MCAN_CCCR.FDOE enables the transmission in CAN FD format
    uint32_t XTD:  1; //!< 30    - Extended Identifier: '1' = 29-bit extended identifier ; '0' = 11-bit standard identifier
    uint32_t ESI:  1; //!< 31    - Error State Indicator. The ESI bit of the transmit buffer is or'ed with the error passive flag to decide the value of the ESI bit in the transmitted FD frame. As required by the CAN FD protocol specification, an error active node may optionally transmit the ESI bit recessive, but an error passive node will always transmit the ESI bit recessive. This feature can be used in gateway applications when a message from an error passive node is routed to another CAN network: '1' = ESI bit in CAN FD format transmitted recessive ; '0' = ESI bit in CAN FD format depends only on error passive flag
  };
} MCAN_CAN_TxMessageIdentifier;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxMessageIdentifier, 4);

#define MCAN_CAN_MSGT0_ID_Pos            0
#define MCAN_CAN_MSGT0_ID_Mask           (0x1FFFFFFFu << MCAN_CAN_MSGT0_ID_Pos)
#define MCAN_CAN_MSGT0_ID_SET(value)     (((uint32_t)(value) << MCAN_CAN_MSGT0_ID_Pos) & MCAN_CAN_MSGT0_ID_Mask) //!< Message Identifier filter

#define MCAN_CAN_MSGT0_EID_Pos           0
#define MCAN_CAN_MSGT0_EID_Mask          (0x3FFFFu << MCAN_CAN_MSGT0_EID_Pos)
#define MCAN_CAN_MSGT0_EID_SET(value)    (((uint32_t)(value) << MCAN_CAN_MSGT0_EID_Pos) & MCAN_CAN_MSGT0_EID_Mask) //!< Extended Identifier filter
#define MCAN_CAN_MSGT0_SID_Pos           18
#define MCAN_CAN_MSGT0_SID_Mask          (0x7FFu << MCAN_CAN_MSGT0_SID_Pos)
#define MCAN_CAN_MSGT0_SID_SET(value)    (((uint32_t)(value) << MCAN_CAN_MSGT0_SID_Pos) & MCAN_CAN_MSGT0_SID_Mask) //!< Standard Identifier filter
#define MCAN_CAN_MSGT0_REMOTE_FRAME      (0x1u << 29) //!< Transmit remote frame
#define MCAN_CAN_MSGT0_DATA_FRAME        (0x0u << 29) //!< Transmit data frame
#define MCAN_CAN_MSGT0_EXTENDED_ID       (0x1u << 30) //!< 29-bit extended identifier
#define MCAN_CAN_MSGT0_STANDARD_ID       (0x0u << 30) //!< 11-bit standard identifier
#define MCAN_CAN_MSGT0_ERROR_STATUS_IND  (0x1u << 31) //!< Error Status Indicator

//-----------------------------------------------------------------------------

//! CAN Transmit Message Control Field (T1)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxMessageControl
{
  uint32_t T1;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     : 8; //!<  0- 7
    uint32_t MMh : 8; //!<  8-15 - Message Marker. High byte of Wide Message Marker, written by CPU during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status. Available only when CCCR.WMM = '1' or when CCCR.UTSU = '1'
    uint32_t DLC : 4; //!< 16-19 - Data Length Code
    uint32_t BRS : 1; //!< 20    - Bit Rate Switching: '1' = CAN FD frames transmitted with bit rate switching ; '0' = CAN FD frames transmitted without bit rate switching
    uint32_t FDF : 1; //!< 21    - FD Format: '1' = Frame transmitted in CAN FD format ; '0' = Frame transmitted in Classic CAN format
    uint32_t TSCE: 1; //!< 22    - Time Stamp Capture Enable for TSU: '1' = Time Stamp Capture enabled ; '0' = Time Stamp Capture disabled
    uint32_t EFC_: 1; //!< 23    - Event FIFO Control: '1 = Store Tx events ; '0' = Do not store Tx events
    uint32_t MMl : 8; //!< 24-31 - Message Marker. Written by processor during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
  };
} MCAN_CAN_TxMessageControl;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxMessageControl, 4);

#define MCAN_CAN_MSGT1_MMh_Pos                8
#define MCAN_CAN_MSGT1_MMh_Mask               (0xFFu << MCAN_CAN_MSGT1_MMh_Pos)
#define MCAN_CAN_MSGT1_MMh_SET(value)         (((uint32_t)(value) << MCAN_CAN_MSGT1_MMh_Pos) & MCAN_CAN_MSGT1_MMh_Mask) //!< Message Marker. High byte of Wide Message Marker, written by CPU during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status. Available only when CCCR.WMM = '1' or when CCCR.UTSU = '1'
#define MCAN_CAN_MSGT1_DLC_Pos                16
#define MCAN_CAN_MSGT1_DLC_Mask               (0xFu << MCAN_CAN_MSGT1_DLC_Pos)
#define MCAN_CAN_MSGT1_DLC_SET(value)         (((uint32_t)(value) << MCAN_CAN_MSGT1_DLC_Pos) & MCAN_CAN_MSGT1_DLC_Mask) //!< Data Length Code
#define MCAN_CAN_MSGT1_BITRATE_SWITCH         (0x1u << 20) //!< CAN FD frames transmitted with bit rate switching
#define MCAN_CAN_MSGT1_NO_BITRATE_SWITCH      (0x0u << 20) //!< CAN FD frames transmitted without bit rate switching
#define MCAN_CAN_MSGT1_FD_FORMAT              (0x1u << 21) //!< Frame transmitted in CAN FD format
#define MCAN_CAN_MSGT1_CLASSIC_CAN_FORMAT     (0x0u << 21) //!< Frame transmitted in Classic CAN format
#define MCAN_CAN_MSGT1_USE_TIMESTAMP_CAPTURE  (0x1u << 22) //!< Timestamp capture enable for TSU
#define MCAN_CAN_MSGT1_NO_TIMESTAMP_CAPTURE   (0x0u << 22) //!< Timestamp capture disable for TSU
#define MCAN_CAN_MSGT1_STORE_TX_EVENT         (0x1u << 23) //!< Store Tx events
#define MCAN_CAN_MSGT1_NO_STORE_TX_EVENT      (0x0u << 23) //!< Do not store Tx events
#define MCAN_CAN_MSGT1_MMl_Pos                24
#define MCAN_CAN_MSGT1_MMl_Mask               (0xFFu << MCAN_CAN_MSGT1_MMl_Pos)
#define MCAN_CAN_MSGT1_MMl_SET(value)         (((uint32_t)(value) << MCAN_CAN_MSGT1_MMl_Pos) & MCAN_CAN_MSGT1_MMl_Mask) //!< Message Marker. Copied into Tx Event FIFO element for identification of Tx message status
#define MCAN_CAN_MSGT1_MM_SET(value)          ( MCAN_CAN_MSGT1_MMh_SET((value) >> 8) | MCAN_CAN_MSGT1_MMl_SET(value) ) //!< Complete Message Marker

//-----------------------------------------------------------------------------

#define MCAN_CAN_MSG_T0  0
#define MCAN_CAN_MSG_T1  1

//! Transmit Message Object Register (TX Buffer, TX Queue, and TX FIFO)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxMessage
{
  uint32_t Word[2];
  uint8_t Bytes[8];
  struct
  {
    MCAN_CAN_TxMessageIdentifier T0; //!< CAN Transmit Message Identifier (T0)
    MCAN_CAN_TxMessageControl    T1; //!< CAN Transmit Message Control Field (T1)
  };
} MCAN_CAN_TxMessage;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxMessage, 8);

//-----------------------------------------------------------------------------

#define MCAN_CAN_TX_MESSAGE_HEADER_SIZE  ( sizeof(MCAN_CAN_TxMessage) )
#define MCAN_CAN_TX_MESSAGE_SIZE_MAX     ( sizeof(MCAN_CAN_TxMessage) + MCAN_PAYLOAD_MAX )

//-----------------------------------------------------------------------------

//! CAN Transmit Event Identifier (E0)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxEventIdentifier
{
  uint32_t E0;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ID : 29; //!<  0-28 - Message Identifier. Standard or extended identifier depending on bit XTD. A standard identifier has to be written to ID[28:18]
    uint32_t RTR:  1; //!< 29    - Remote Transmission Request. When RTR = 1, the MCAN transmits a remote frame according to ISO11898-1, even if MCAN_CCCR.FDOE enables the transmission in CAN FD format
    uint32_t XTD:  1; //!< 30    - Extended Identifier: '1' = 29-bit extended identifier ; '0' = 11-bit standard identifier
    uint32_t ESI:  1; //!< 31    - Error State Indicator. The ESI bit of the transmit buffer is or'ed with the error passive flag to decide the value of the ESI bit in the transmitted FD frame. As required by the CAN FD protocol specification, an error active node may optionally transmit the ESI bit recessive, but an error passive node will always transmit the ESI bit recessive. This feature can be used in gateway applications when a message from an error passive node is routed to another CAN network: '1' = ESI bit in CAN FD format transmitted recessive ; '0' = ESI bit in CAN FD format depends only on error passive flag
  };
} MCAN_CAN_TxEventIdentifier;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxEventIdentifier, 4);

#define MCAN_CAN_MSGE0_ID_Pos            0
#define MCAN_CAN_MSGE0_ID_Mask           (0x1FFFFFFFu << MCAN_CAN_MSGE0_ID_Pos)
#define MCAN_CAN_MSGE0_ID_GET(value)     (((uint32_t)(value) & MCAN_CAN_MSGE0_ID_Mask) >> MCAN_CAN_MSGE0_ID_Pos) //!< Message Identifier filter

#define MCAN_CAN_MSGE0_SID_Pos           18
#define MCAN_CAN_MSGE0_SID_Mask          (0x7FFu << MCAN_CAN_MSGE0_SID_Pos)
#define MCAN_CAN_MSGE0_SID_GET(value)    (((uint32_t)(value) & MCAN_CAN_MSGE0_SID_Mask) >> MCAN_CAN_MSGE0_SID_Pos) //!< Standard Identifier filter
#define MCAN_CAN_MSGE0_EID_Pos           0
#define MCAN_CAN_MSGE0_EID_Mask          (0x3FFFFu << MCAN_CAN_MSGE0_EID_Pos)
#define MCAN_CAN_MSGE0_EID_GET(value)    (((uint32_t)(value) & MCAN_CAN_MSGE0_EID_Mask) >> MCAN_CAN_MSGE0_EID_Pos) //!< Extended Identifier filter
#define MCAN_CAN_MSGE0_REMOTE_FRAME      (0x1u << 29) //!< Transmit remote frame
#define MCAN_CAN_MSGE0_DATA_FRAME        (0x0u << 29) //!< Transmit data frame
#define MCAN_CAN_MSGE0_EXTENDED_ID       (0x1u << 30) //!< 29-bit extended identifier
#define MCAN_CAN_MSGE0_STANDARD_ID       (0x0u << 30) //!< 11-bit standard identifier
#define MCAN_CAN_MSGE0_ERROR_STATUS_IND  (0x1u << 31) //!< Error Status Indicator

//-----------------------------------------------------------------------------

/*! CAN Transmit Event Control Field A (E1 or E1A)
 * When CCCR.WMM = '0' and no TSU is used (CCCR.UTSU = '0'), E1A.TXTS[15:0] holds the 16-bit timestamp generated by the M_CAN's internal timestamping logic
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxEventControlA
{
  uint32_t E1;
  uint32_t E1A;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TXTS: 16; //!<  0-15 - Tx Timestamp. Timestamp Counter value captured on start of frame transmission. Resolution depending on configuration of the Timestamp Counter Prescaler MCAN_TSCC.TCP
    uint32_t DLC :  4; //!< 16-19 - Data Length Code
    uint32_t BRS :  1; //!< 20    - Bit Rate Switching: '1' = CAN FD frames transmitted with bit rate switching ; '0' = CAN FD frames transmitted without bit rate switching
    uint32_t FDF :  1; //!< 21    - FD Format: '1' = Frame transmitted in CAN FD format ; '0' = Frame transmitted in Classic CAN format
    uint32_t ET  :  2; //!< 22-23 - Event Type
    uint32_t MM  :  8; //!< 24-31 - Message Marker. Written by processor during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
  };
} MCAN_CAN_TxEventControlA;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxEventControlA, 4);

#define MCAN_CAN_MSGE1A_TXTS_Pos            0
#define MCAN_CAN_MSGE1A_TXTS_Mask           (0xFFFFu << MCAN_CAN_MSGE1A_TXTS_Pos)
#define MCAN_CAN_MSGE1A_TXTS_GET(value)     (((uint32_t)(value) & MCAN_CAN_MSGE1A_TXTS_Mask) >> MCAN_CAN_MSGE1A_TXTS_Pos) //!< Tx Timestamp
#define MCAN_CAN_MSGE1A_DLC_Pos             16
#define MCAN_CAN_MSGE1A_DLC_Mask            (0xFu << MCAN_CAN_MSGE1A_DLC_Pos)
#define MCAN_CAN_MSGE1A_DLC_GET(value)      (((uint32_t)(value) & MCAN_CAN_MSGE1A_DLC_Mask) >> MCAN_CAN_MSGE1A_DLC_Pos) //!< Data Length Code
#define MCAN_CAN_MSGE1A_BITRATE_SWITCH      (0x1u << 20) //!< CAN FD frames transmitted with bit rate switching
#define MCAN_CAN_MSGE1A_NO_BITRATE_SWITCH   (0x0u << 20) //!< CAN FD frames transmitted without bit rate switching
#define MCAN_CAN_MSGE1A_FD_FORMAT           (0x1u << 21) //!< Frame transmitted in CAN FD format
#define MCAN_CAN_MSGE1A_CLASSIC_CAN_FORMAT  (0x0u << 21) //!< Frame transmitted in Classic CAN format

//! Event Type enumerator
typedef enum
{
  MCAN_EVENT_TYPE_RESERVED0  = 0b00, //!< Event Type Reserved
  MCAN_TX_EVENT              = 0b01, //!< TX Event
  MCAN_TX_SPITE_CANCELLATION = 0b10, //!< Transmission in spite of cancellation (always set for transmissions in DAR mode)
  MCAN_EVENT_TYPE_RESERVED3  = 0b11, //!< Event Type Reserved
} eMCAN_EventType;

#define MCAN_CAN_MSGE1A_ET_Pos              22
#define MCAN_CAN_MSGE1A_ET_Mask             (0x3u << MCAN_CAN_MSGE1A_ET_Pos)
#define MCAN_CAN_MSGE1A_ET_GET(value)       (eMCAN_EventType)(((uint32_t)(value) & MCAN_CAN_MSGE1A_ET_Mask) >> MCAN_CAN_MSGE1A_ET_Pos) //!< Event Type
#define MCAN_CAN_MSGE1A_MM_Pos              24
#define MCAN_CAN_MSGE1A_MM_Mask             (0xFFu << MCAN_CAN_MSGE1A_MM_Pos)
#define MCAN_CAN_MSGE1A_MM_GET(value)       (((uint32_t)(value) & MCAN_CAN_MSGE1A_MM_Mask) >> MCAN_CAN_MSGE1A_MM_Pos) //!< Message Marker. Written by processor during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status

//-----------------------------------------------------------------------------

/*! CAN Transmit Event Control Field B (E1B)
 * When 16-bit Message Markers are enabled (CCCR.WMM = '1') or when CCCR.UTSU = '1', E1B.MM[15:8] holds the upper 8 bit of the Wide Message Marker.
 * When a TSU is used (CCCR.UTSU = '1') and when bit TSCE of the related Tx Buffer element is set, E1B.TSC = '1' and E1B.TXTSP[3:0]
 * holds the number of the TSU's Timestamp register which holds the 32-bit timestamp captured by the TSU. Else E1B.TSC = '0' and E1B.TXTSP[3:0] is not valid
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxEventControlB
{
  uint32_t E1B;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TXTSP: 4; //!<  0- 3 - Tx Timestamp Pointer. Number of TSU Time Stamp register (TS0..15) where the related timestamp is stored
    uint32_t TSC  : 1; //!<  4    - Timestamp Captured: '1' = Timestamp captured and stored in TSU Timestamp register referenced by E1B.TXTSP ; '0' = No timestamp captured
    uint32_t      : 3; //!<  5-7
    uint32_t MMh  : 8; //!<  8-15 - Message Marker. High byte of Wide Message Marker, written by CPU during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
    uint32_t DLC  : 4; //!< 16-19 - Data Length Code
    uint32_t BRS  : 1; //!< 20    - Bit Rate Switching: '1' = CAN FD frames transmitted with bit rate switching ; '0' = CAN FD frames transmitted without bit rate switching
    uint32_t FDF  : 1; //!< 21    - FD Format: '1' = Frame transmitted in CAN FD format ; '0' = Frame transmitted in Classic CAN format
    uint32_t ET   : 2; //!< 22-23 - Event Type
    uint32_t MMl  : 8; //!< 24-31 - Message Marker. Written by processor during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
  };
} MCAN_CAN_TxEventControlB;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxEventControlB, 4);

#define MCAN_CAN_MSGE1B_TXTSP_Pos              0
#define MCAN_CAN_MSGE1B_TXTSP_Mask             (0xFu << MCAN_CAN_MSGE1B_TXTSP_Pos)
#define MCAN_CAN_MSGE1B_TXTSP_GET(value)       (((uint32_t)(value) & MCAN_CAN_MSGE1B_TXTSP_Mask) >> MCAN_CAN_MSGE1B_TXTSP_Pos) //!< Tx Timestamp Pointer
#define MCAN_CAN_MSGE1B_TIMESTAMP_CAPTURED     (0x1u << 4) //!< Timestamp captured and stored in TSU Timestamp register referenced by E1B.TXTSP
#define MCAN_CAN_MSGE1B_NO_TIMESTAMP_CAPTURED  (0x0u << 4) //!< No timestamp captured
#define MCAN_CAN_MSGE1B_MMh_Pos                8
#define MCAN_CAN_MSGE1B_MMh_Mask               (0xFFu << MCAN_CAN_MSGE1B_MMh_Pos)
#define MCAN_CAN_MSGE1B_MMh_GET(value)         (((uint32_t)(value) & MCAN_CAN_MSGE1B_MMh_Mask) >> MCAN_CAN_MSGE1B_MMh_Pos) //!< Message Marker. High byte of Wide Message Marker, written by CPU during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
#define MCAN_CAN_MSGE1B_DLC_Pos                16
#define MCAN_CAN_MSGE1B_DLC_Mask               (0xFu << MCAN_CAN_MSGE1B_DLC_Pos)
#define MCAN_CAN_MSGE1B_DLC_GET(value)         (((uint32_t)(value) & MCAN_CAN_MSGE1B_DLC_Mask) >> MCAN_CAN_MSGE1B_DLC_Pos) //!< Data Length Code
#define MCAN_CAN_MSGE1B_BITRATE_SWITCH         (0x1u << 20) //!< CAN FD frames transmitted with bit rate switching
#define MCAN_CAN_MSGE1B_NO_BITRATE_SWITCH      (0x0u << 20) //!< CAN FD frames transmitted without bit rate switching
#define MCAN_CAN_MSGE1B_FD_FORMAT              (0x1u << 21) //!< Frame transmitted in CAN FD format
#define MCAN_CAN_MSGE1B_CLASSIC_CAN_FORMAT     (0x0u << 21) //!< Frame transmitted in Classic CAN format
#define MCAN_CAN_MSGE1B_ET_Pos                 22
#define MCAN_CAN_MSGE1B_ET_Mask                (0x3u << MCAN_CAN_MSGE1B_ET_Pos)
#define MCAN_CAN_MSGE1B_ET_GET(value)          (eMCAN_EventType)(((uint32_t)(value) & MCAN_CAN_MSGE1B_ET_Mask) >> MCAN_CAN_MSGE1B_ET_Pos) //!< Event Type
#define MCAN_CAN_MSGE1B_MMl_Pos                24
#define MCAN_CAN_MSGE1B_MMl_Mask               (0xFFu << MCAN_CAN_MSGE1B_MMl_Pos)
#define MCAN_CAN_MSGE1B_MMl_GET(value)         (((uint32_t)(value) & MCAN_CAN_MSGE1B_MMl_Mask) >> MCAN_CAN_MSGE1B_MMl_Pos) //!< Message Marker. Written by processor during Tx Buffer configuration. Copied into Tx Event FIFO element for identification of Tx message status
#define MCAN_CAN_MSGE1B_MM_GET(value)          ( (MCAN_CAN_MSGE1B_MMh_GET(value) << 8) | MCAN_CAN_MSGE1B_MMl_GET(value) ) //!< Complete Message Marker. Written by processor during Tx Buffer configuration

//-----------------------------------------------------------------------------

#define MCAN_CAN_MSG_E0  0
#define MCAN_CAN_MSG_E1  1

//! Transmit Event Object Register (TX Event)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_TxEventObject
{
  uint32_t Word[2];
  uint8_t Bytes[8];
  struct
  {
    MCAN_CAN_TxEventIdentifier E0;  //!< CAN Transmit Event Object Identifier (E0)
    union
    {
      MCAN_CAN_TxEventControlA E1A; //!< CAN Transmit Event Object Control Field A (E1 or E1A)
      MCAN_CAN_TxEventControlB E1B; //!< CAN Transmit Event Object Control Field B (E1B)
    };
  };
} MCAN_CAN_TxEventObject;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_TxEventObject, 8);

//-----------------------------------------------------------------------------

#define MCAN_CAN_TX_EVENTOBJECT_SIZE  ( sizeof(MCAN_CAN_TxEventObject) )

//-----------------------------------------------------------------------------

//! CAN Receive Message Identifier (R0)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_RxMessageIdentifier
{
  uint32_t R0;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ID : 29; //!<  0-28 - Message Identifier. Standard or extended identifier depending on bit XTD. A standard identifier has to be written to ID[28:18]
    uint32_t RTR:  1; //!< 29    - Remote Transmission Request. When RTR = 1, the MCAN transmits a remote frame according to ISO11898-1, even if MCAN_CCCR.FDOE enables the transmission in CAN FD format
    uint32_t XTD:  1; //!< 30    - Extended Identifier: '1' = 29-bit extended identifier ; '0' = 11-bit standard identifier
    uint32_t ESI:  1; //!< 31    - Error State Indicator. The ESI bit of the transmit buffer is or'ed with the error passive flag to decide the value of the ESI bit in the transmitted FD frame. As required by the CAN FD protocol specification, an error active node may optionally transmit the ESI bit recessive, but an error passive node will always transmit the ESI bit recessive. This feature can be used in gateway applications when a message from an error passive node is routed to another CAN network: '1' = ESI bit in CAN FD format transmitted recessive ; '0' = ESI bit in CAN FD format depends only on error passive flag
  };
} MCAN_CAN_RxMessageIdentifier;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_RxMessageIdentifier, 4);

#define MCAN_CAN_MSGR0_ID_Pos            0
#define MCAN_CAN_MSGR0_ID_Mask           (0x1FFFFFFFu << MCAN_CAN_MSGR0_ID_Pos)
#define MCAN_CAN_MSGR0_ID_GET(value)     (((uint32_t)(value) & MCAN_CAN_MSGR0_ID_Mask) >> MCAN_CAN_MSGR0_ID_Pos) //!< Message Identifier filter

#define MCAN_CAN_MSGR0_SID_Pos           18
#define MCAN_CAN_MSGR0_SID_Mask          (0x7FFu << MCAN_CAN_MSGR0_SID_Pos)
#define MCAN_CAN_MSGR0_SID_GET(value)    (((uint32_t)(value) & MCAN_CAN_MSGR0_SID_Mask) >> MCAN_CAN_MSGR0_SID_Pos) //!< Standard Identifier filter
#define MCAN_CAN_MSGR0_EID_Pos           0
#define MCAN_CAN_MSGR0_EID_Mask          (0x3FFFFu << MCAN_CAN_MSGR0_EID_Pos)
#define MCAN_CAN_MSGR0_EID_GET(value)    (((uint32_t)(value) & MCAN_CAN_MSGR0_EID_Mask) >> MCAN_CAN_MSGR0_EID_Pos) //!< Extended Identifier filter
#define MCAN_CAN_MSGR0_REMOTE_FRAME      (0x1u << 29) //!< Transmit remote frame
#define MCAN_CAN_MSGR0_DATA_FRAME        (0x0u << 29) //!< Transmit data frame
#define MCAN_CAN_MSGR0_EXTENDED_ID       (0x1u << 30) //!< 29-bit extended identifier
#define MCAN_CAN_MSGR0_STANDARD_ID       (0x0u << 30) //!< 11-bit standard identifier
#define MCAN_CAN_MSGR0_ERROR_STATUS_IND  (0x1u << 31) //!< Error Status Indicator

//-----------------------------------------------------------------------------

/*! CAN Receive Message Control Field A (R1 or R1A)
 * When no TSU is used (CCCR.UTSU = '0'), R1A.RXTS[15:0] holds the 16-bit timestamp generated by the M_CAN's internal timestamping logic
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_RxMessageControlA
{
  uint32_t R1;
  uint32_t R1A;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RXTS: 16; //!<  0-15 - Rx Timestamp. Timestamp Counter value captured on start of frame transmission. Resolution depending on configuration of the Timestamp Counter Prescaler MCAN_TSCC.TCP
    uint32_t DLC :  4; //!< 16-19 - Data Length Code
    uint32_t BRS :  1; //!< 20    - Bit Rate Switching: '1' = CAN FD frames transmitted with bit rate switching ; '0' = CAN FD frames transmitted without bit rate switching
    uint32_t FDF :  1; //!< 21    - FD Format: '1' = Frame transmitted in CAN FD format ; '0' = Frame transmitted in Classic CAN format
    uint32_t     :  2; //!< 22-23
    uint32_t FIDX:  7; //!< 24-30 - Filter Index. Range is 0 to MCAN_SIDFC.LSS-1 resp. MCAN_XIDFC.LSE-1: '0-127' = Index of matching Rx acceptance filter element (invalid if ANMF = '1')
    uint32_t ANMF:  1; //!< 31    - Accepted Non-matching Frame. Acceptance of non-matching frames may be enabled via MCAN_GFC.ANFS and MCAN_GFC.ANFE: '1' = Received frame did not match any Rx filter element ; '0' = Received frame matching filter index FIDX
  };
} MCAN_CAN_RxMessageControlA;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_RxMessageControlA, 4);

#define MCAN_CAN_MSGR1A_RXTS_Pos            0
#define MCAN_CAN_MSGR1A_RXTS_Mask           (0xFFFFu << MCAN_CAN_MSGR1A_RXTS_Pos)
#define MCAN_CAN_MSGR1A_RXTS_GET(value)     (((uint32_t)(value) & MCAN_CAN_MSGR1A_RXTS_Mask) >> MCAN_CAN_MSGR1A_RXTS_Pos) //!< Rx Timestamp
#define MCAN_CAN_MSGR1A_DLC_Pos             16
#define MCAN_CAN_MSGR1A_DLC_Mask            (0xFu << MCAN_CAN_MSGR1A_DLC_Pos)
#define MCAN_CAN_MSGR1A_DLC_GET(value)      (((uint32_t)(value) & MCAN_CAN_MSGR1A_DLC_Mask) >> MCAN_CAN_MSGR1A_DLC_Pos) //!< Data Length Code
#define MCAN_CAN_MSGR1A_BITRATE_SWITCH      (0x1u << 20) //!< CAN FD frames transmitted with bit rate switching
#define MCAN_CAN_MSGR1A_NO_BITRATE_SWITCH   (0x0u << 20) //!< CAN FD frames transmitted without bit rate switching
#define MCAN_CAN_MSGR1A_FD_FORMAT           (0x1u << 21) //!< Frame transmitted in CAN FD format
#define MCAN_CAN_MSGR1A_CLASSIC_CAN_FORMAT  (0x0u << 21) //!< Frame transmitted in Classic CAN format
#define MCAN_CAN_MSGR1A_FIDX_Pos            24
#define MCAN_CAN_MSGR1A_FIDX_Mask           (0x7Fu << MCAN_CAN_MSGR1A_FIDX_Pos)
#define MCAN_CAN_MSGR1A_FIDX_GET(value)     (((uint32_t)(value) & MCAN_CAN_MSGR1A_FIDX_Mask) >> MCAN_CAN_MSGR1A_FIDX_Pos) //!< Filter Index
#define MCAN_CAN_MSGR1A_NON_MATCHING_FRAME  (0x1u << 31) //!< Received frame did not match any Rx filter element

//-----------------------------------------------------------------------------

/*! CAN Receive Message Control Field B (R1B)
 * When a TSU is used (CCCR.UTSU = '1') and when bit SSYNC/ESYNC of the matching filter element is set, R1B.TSC = '1' and R1B.RXTSP[3:0] holds the number of the TSU's Timestamp
 * register which holds the 32-bit timestamp captured by the TSU. Else R1B.TSC = '0' and R1B.RXTSP[3:0] is not valid
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_RxMessageControlB
{
  uint32_t R1B;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RXTSP:  4; //!<  0- 3 - Rx Timestamp Pointer. Number of TSU Time Stamp register (TS0..15) where the related timestamp is stored
    uint32_t TSC  :  1; //!<  4    - Timestamp Captured: '1' = Timestamp captured and stored in TSU Timestamp register referenced by R1B.RXTSP ; '0' = No timestamp captured
    uint32_t      : 11; //!<  5-15
    uint32_t DLC  :  4; //!< 16-19 - Data Length Code
    uint32_t BRS  :  1; //!< 20    - Bit Rate Switching: '1' = CAN FD frames transmitted with bit rate switching ; '0' = CAN FD frames transmitted without bit rate switching
    uint32_t FDF  :  1; //!< 21    - FD Format: '1' = Frame transmitted in CAN FD format ; '0' = Frame transmitted in Classic CAN format
    uint32_t      :  2; //!< 22-23
    uint32_t FIDX :  7; //!< 24-30 - Filter Index. Range is 0 to MCAN_SIDFC.LSS-1 resp. MCAN_XIDFC.LSE-1: '0-127' = Index of matching Rx acceptance filter element (invalid if ANMF = '1')
    uint32_t ANMF :  1; //!< 31    - Accepted Non-matching Frame. Acceptance of non-matching frames may be enabled via MCAN_GFC.ANFS and MCAN_GFC.ANFE: '1' = Received frame did not match any Rx filter element ; '0' = Received frame matching filter index FIDX
  };
} MCAN_CAN_RxMessageControlB;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_RxMessageControlB, 4);

#define MCAN_CAN_MSGR1B_RXTSP_Pos              0
#define MCAN_CAN_MSGR1B_RXTSP_Mask             (0xFu << MCAN_CAN_MSGR1B_RXTSP_Pos)
#define MCAN_CAN_MSGR1B_RXTSP_GET(value)       (((uint32_t)(value) & MCAN_CAN_MSGR1B_RXTSP_Mask) >> MCAN_CAN_MSGR1B_RXTSP_Pos) //!< Rx Timestamp Pointer
#define MCAN_CAN_MSGR1B_TIMESTAMP_CAPTURED     (0x1u << 4) //!< Timestamp captured and stored in TSU Timestamp register referenced by R1B.RXTSP
#define MCAN_CAN_MSGR1B_NO_TIMESTAMP_CAPTURED  (0x0u << 4) //!< No timestamp captured
#define MCAN_CAN_MSGR1B_DLC_Pos                16
#define MCAN_CAN_MSGR1B_DLC_Mask               (0xFu << MCAN_CAN_MSGR1B_DLC_Pos)
#define MCAN_CAN_MSGR1B_DLC_GET(value)         (((uint32_t)(value) & MCAN_CAN_MSGR1B_DLC_Mask) >> MCAN_CAN_MSGR1B_DLC_Pos) //!< Data Length Code
#define MCAN_CAN_MSGR1B_BITRATE_SWITCH         (0x1u << 20) //!< CAN FD frames transmitted with bit rate switching
#define MCAN_CAN_MSGR1B_NO_BITRATE_SWITCH      (0x0u << 20) //!< CAN FD frames transmitted without bit rate switching
#define MCAN_CAN_MSGR1B_FD_FORMAT              (0x1u << 21) //!< Frame transmitted in CAN FD format
#define MCAN_CAN_MSGR1B_CLASSIC_CAN_FORMAT     (0x0u << 21) //!< Frame transmitted in Classic CAN format
#define MCAN_CAN_MSGR1B_FIDX_Pos               24
#define MCAN_CAN_MSGR1B_FIDX_Mask              (0x7Fu << MCAN_CAN_MSGR1B_FIDX_Pos)
#define MCAN_CAN_MSGR1B_FIDX_GET(value)        (((uint32_t)(value) & MCAN_CAN_MSGR1B_FIDX_Mask) >> MCAN_CAN_MSGR1B_FIDX_Pos) //!< Filter Index
#define MCAN_CAN_MSGR1B_NON_MATCHING_FRAME     (0x1u << 21) //!< Received frame did not match any Rx filter element

//-----------------------------------------------------------------------------

#define MCAN_CAN_MSG_R0  0
#define MCAN_CAN_MSG_R1  1

//! Receive Message Object Register
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CAN_RxMessage
{
  uint32_t Word[2];
  uint8_t Bytes[8];
  struct
  {
    MCAN_CAN_RxMessageIdentifier R0;  //!< CAN Receive Message Identifier (R0)
    union
    {
      MCAN_CAN_RxMessageControlA R1A; //!< CAN Receive Message Control Field A (R1 or R1A)
      MCAN_CAN_RxMessageControlB R1B; //!< CAN Receive Message Control Field B (R1B)
    };
  };
} MCAN_CAN_RxMessage;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CAN_RxMessage, 8);

//-----------------------------------------------------------------------------

#define MCAN_CAN_RX_MESSAGE_HEADER_SIZE  ( sizeof(MCAN_CAN_RxMessage) )
#define MCAN_CAN_RX_MESSAGE_SIZE_MAX     ( sizeof(MCAN_CAN_RxMessage) + MCAN_PAYLOAD_MAX )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN Core Register list
//********************************************************************************************************************

#define RegMCAN_COUNT  ( 64 )
#define RegMCAN_SIZE   ( RegMCAN_COUNT * sizeof(uint32_t) )

#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
//! MCAN Core registers list
typedef enum
{
  RegMCAN_CREL   = 0x00u, //!< (Offset: 0x00) Core Release Register
  RegMCAN_ENDN   = 0x04u, //!< (Offset: 0x04) Endian Register
  RegMCAN_CUST   = 0x08u, //!< (Offset: 0x08) Customer Register
  RegMCAN_DBTP   = 0x0Cu, //!< (Offset: 0x0C) Data Bit Timing and Prescaler Register
  RegMCAN_TEST   = 0x10u, //!< (Offset: 0x10) Test Register
  RegMCAN_RWD    = 0x14u, //!< (Offset: 0x14) RAM Watchdog Register
  RegMCAN_CCCR   = 0x18u, //!< (Offset: 0x18) CC Control Register
  RegMCAN_NBTP   = 0x1Cu, //!< (Offset: 0x1C) Nominal Bit Timing and Prescaler Register
  RegMCAN_TSCC   = 0x20u, //!< (Offset: 0x20) Timestamp Counter Configuration Register
  RegMCAN_TSCV   = 0x24u, //!< (Offset: 0x24) Timestamp Counter Value Register
  RegMCAN_TOCC   = 0x28u, //!< (Offset: 0x28) Timeout Counter Configuration Register
  RegMCAN_TOCV   = 0x2Cu, //!< (Offset: 0x2C) Timeout Counter Value Register
                          //   (Offset: 0x30..0x3C) Reserved
  RegMCAN_ECR    = 0x40u, //!< (Offset: 0x40) Error Counter Register
  RegMCAN_PSR    = 0x44u, //!< (Offset: 0x44) Protocol Status Register
  RegMCAN_TDCR   = 0x48u, //!< (Offset: 0x48) Transmit Delay Compensation Register
                          //   (Offset: 0x4C) Reserved
  RegMCAN_IR     = 0x50u, //!< (Offset: 0x50) Interrupt Register
  RegMCAN_IE     = 0x54u, //!< (Offset: 0x54) Interrupt Enable Register
  RegMCAN_ILS    = 0x58u, //!< (Offset: 0x58) Interrupt Line Select Register
  RegMCAN_ILE    = 0x5Cu, //!< (Offset: 0x5C) Interrupt Line Enable Register
                          //   (Offset: 0x60..0x7C) Reserved
  RegMCAN_GFC    = 0x80u, //!< (Offset: 0x80) Global Filter Configuration Register
  RegMCAN_SIDFC  = 0x84u, //!< (Offset: 0x84) Standard ID Filter Configuration Register
  RegMCAN_XIDFC  = 0x88u, //!< (Offset: 0x88) Extended ID Filter Configuration Register
                          //   (Offset: 0x8C) Reserved
  RegMCAN_XIDAM  = 0x90u, //!< (Offset: 0x90) Extended ID AND Mask Register
  RegMCAN_HPMS   = 0x94u, //!< (Offset: 0x94) High Priority Message Status Register
  RegMCAN_NDAT1  = 0x98u, //!< (Offset: 0x98) New Data 1 Register
  RegMCAN_NDAT2  = 0x9Cu, //!< (Offset: 0x9C) New Data 2 Register
  RegMCAN_RXF0C  = 0xA0u, //!< (Offset: 0xA0) Receive FIFO 0 Configuration Register
  RegMCAN_RXF0S  = 0xA4u, //!< (Offset: 0xA4) Receive FIFO 0 Status Register
  RegMCAN_RXF0A  = 0xA8u, //!< (Offset: 0xA8) Receive FIFO 0 Acknowledge Register
  RegMCAN_RXBC   = 0xACu, //!< (Offset: 0xAC) Receive Rx Buffer Configuration Register
  RegMCAN_RXF1C  = 0xB0u, //!< (Offset: 0xB0) Receive FIFO 1 Configuration Register
  RegMCAN_RXF1S  = 0xB4u, //!< (Offset: 0xB4) Receive FIFO 1 Status Register
  RegMCAN_RXF1A  = 0xB8u, //!< (Offset: 0xB8) Receive FIFO 1 Acknowledge Register
  RegMCAN_RXESC  = 0xBCu, //!< (Offset: 0xBC) Receive Buffer / FIFO Element Size Configuration Register
  RegMCAN_TXBC   = 0xC0u, //!< (Offset: 0xC0) Transmit Buffer Configuration Register
  RegMCAN_TXFQS  = 0xC4u, //!< (Offset: 0xC4) Transmit FIFO/Queue Status Register
  RegMCAN_TXESC  = 0xC8u, //!< (Offset: 0xC8) Transmit Buffer Element Size Configuration Register
  RegMCAN_TXBRP  = 0xCCu, //!< (Offset: 0xCC) Transmit Buffer Request Pending Register
  RegMCAN_TXBAR  = 0xD0u, //!< (Offset: 0xD0) Transmit Buffer Add Request Register
  RegMCAN_TXBCR  = 0xD4u, //!< (Offset: 0xD4) Transmit Buffer Cancellation Request Register
  RegMCAN_TXBTO  = 0xD8u, //!< (Offset: 0xD8) Transmit Buffer Transmission Occurred Register
  RegMCAN_TXBCF  = 0xDCu, //!< (Offset: 0xDC) Transmit Buffer Cancellation Finished Register
  RegMCAN_TXBTIE = 0xE0u, //!< (Offset: 0xE0) Transmit Buffer Transmission Interrupt Enable Register
  RegMCAN_TXBCIE = 0xE4u, //!< (Offset: 0xE4) Transmit Buffer Cancellation Finished Interrupt Enable Register
                          //   (Offset: 0xE8..0xEC) Reserved
  RegMCAN_TXEFC  = 0xF0u, //!< (Offset: 0xF0) Transmit Event FIFO Configuration Register
  RegMCAN_TXEFS  = 0xF4u, //!< (Offset: 0xF4) Transmit Event FIFO Status Register
  RegMCAN_TXEFA  = 0xF8u, //!< (Offset: 0xF8) Transmit Event FIFO Acknowledge Register
                          //   (Offset: 0xFC) Reserved
} eMCAN_Registers;
#endif // !defined(MCAN_INTERNAL_CAN_CONTROLLER)




//********************************************************************************************************************
// MCAN Controller Registers
//********************************************************************************************************************

//! MCAN Core Release Register (Read-only, Offset: 0x00)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CREL_Register
{
  uint32_t CREL;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t DAY    : 8; //!<  0- 7 - Timestamp Day. Two digits, BCD-coded. This field is set by generic parameter on MCAN synthesis
    uint32_t MON    : 8; //!<  8-15 - Timestamp Month. Two digits, BCD-coded. This field is set by generic parameter on MCAN synthesis
    uint32_t YEAR   : 4; //!< 16-19 - Timestamp Year. One digit, BCD-coded. This field is set by generic parameter on MCAN synthesis
    uint32_t SUBSTEP: 4; //!< 20-23 - Sub-step of Core Release. One digit, BCD-coded
    uint32_t STEP   : 4; //!< 24-27 - Step of Core Release. One digit, BCD-coded
    uint32_t REL    : 4; //!< 28-31 - Core Release. One digit, BCD-coded
  } Bits;
} MCAN_CREL_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CREL_Register, 4);

#define MCAN_CREL_DAY_Pos             0
#define MCAN_CREL_DAY_Mask            (0xFFu << MCAN_CREL_DAY_Pos)
#define MCAN_CREL_DAY_GET(value)      MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_DAY_Mask) >> MCAN_CREL_DAY_Pos) //!< Get Timestamp Day
#define MCAN_CREL_MON_Pos             8
#define MCAN_CREL_MON_Mask            (0xFFu << MCAN_CREL_MON_Pos)
#define MCAN_CREL_MON_GET(value)      MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_MON_Mask) >> MCAN_CREL_MON_Pos) //!< Get Timestamp Month
#define MCAN_CREL_YEAR_Pos            16
#define MCAN_CREL_YEAR_Mask           (0xFu << MCAN_CREL_YEAR_Pos)
#define MCAN_CREL_YEAR_GET(value)     MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_YEAR_Mask) >> MCAN_CREL_YEAR_Pos) //!< Get Timestamp Year
#define MCAN_CREL_SUBSTEP_Pos         20
#define MCAN_CREL_SUBSTEP_Mask        (0xFu << MCAN_CREL_SUBSTEP_Pos)
#define MCAN_CREL_SUBSTEP_GET(value)  MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_SUBSTEP_Mask) >> MCAN_CREL_SUBSTEP_Pos) //!< Get Sub-step of Core Release
#define MCAN_CREL_STEP_Pos            24
#define MCAN_CREL_STEP_Mask           (0xFu << MCAN_CREL_STEP_Pos)
#define MCAN_CREL_STEP_GET(value)     MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_STEP_Mask) >> MCAN_CREL_STEP_Pos) //!< Get Step of Core Release
#define MCAN_CREL_REL_Pos             28
#define MCAN_CREL_REL_Mask            (0xFu << MCAN_CREL_REL_Pos)
#define MCAN_CREL_REL_GET(value)      MCAN_DCB8_TO_DECIMAL(((uint32_t)(value) & MCAN_CREL_REL_Mask) >> MCAN_CREL_REL_Pos) //!< Get Core Release

//-----------------------------------------------------------------------------

//! MCAN Endian Register (Read-only, Offset: 0x04)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ENDN_Register
{
  uint32_t ENDN;
  uint8_t Bytes[sizeof(uint32_t)];
} MCAN_ENDN_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ENDN_Register, 4);

#define MCAN_ENDN_ENDIANNESS_TEST_VALUE         ( 0x87654321ul ) //!< The endianness test value is 0x87654321
#define MCAN_ENDN_IS_CORRECT_ENDIANNESS(value)  ( (value) == MCAN_ENDN_ENDIANNESS_TEST_VALUE ) //!< Is the MCAN endianness correct (useful in case of an external device that communicates in big-endian for example)

//-----------------------------------------------------------------------------

//! MCAN Customer Register (Read/Write, Offset: 0x08)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CUST_Register
{
  uint32_t CUST;                   //!< Customer-specific Value
  uint8_t Bytes[sizeof(uint32_t)];
} MCAN_CUST_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CUST_Register, 4);

//-----------------------------------------------------------------------------

/*! MCAN Data Bit Timing and Prescaler Register (Read/Write, Offset: 0x0C)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * The CAN bit time may be programmed in the range of 4 to 25 time quanta. The CAN time quantum may be programmed in the range of 1 to 32 CAN core clock periods. tq = (DBRP + 1) CAN core clock periods.
 * DTSEG1 is the sum of Prop_Seg and Phase_Seg1. DTSEG2 is Phase_Seg2.
 * Therefore the length of the bit time is (programmed values) [DTSEG1 + DTSEG2 + 3] tq or (functional values) [Sync_Seg + Prop_Seg + Phase_Seg1 + Phase_Seg2] tq.
 * The Information Processing Time (IPT) is zero, meaning the data for the next bit is available at the first clock edge after the sample point
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_DBTP_Register
{
  uint32_t DBTP;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t DSJW  : 3; //!<  0- 2 - Data (Re) Synchronization Jump Width. The duration of a synchronization jump is tq x (DSJW + 1)
    uint32_t       : 1; //!<  3
    uint32_t DTSEG2: 4; //!<  4- 7 - Data Time Segment After Sample Point. The duration of time segment is tq x (DTSEG2 + 1)
    uint32_t DTSEG1: 5; //!<  8-12 - Data Time Segment Before Sample Point. '0' = Forbidden ; '1' to '31' = The duration of time segment is tq x (DTSEG1 + 1)
    uint32_t       : 3; //!< 13-15
    uint32_t DBRP  : 5; //!< 16-20 - Data Bit Rate Prescaler. The value by which the peripheral clock is divided for generating the bit time quanta. The bit time is built up from a multiple of this quanta. Valid values for the Bit Rate Prescaler are 0 to 31. The actual interpretation by the hardware of this value is such that one more than the value programmed here is use
    uint32_t       : 2; //!< 21-22
    uint32_t TDC   : 1; //!< 23    - Transmitter Delay Compensation. '0' = Transmitter Delay Compensation disabled ; '1' = Transmitter Delay Compensation enabled
    uint32_t       : 8; //!< 24-31
  } Bits;
} MCAN_DBTP_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_DBTP_Register, 4);

#define MCAN_DBTP_DSJW_Pos           0
#define MCAN_DBTP_DSJW_Mask          (0x7u << MCAN_DBTP_DSJW_Pos)
#define MCAN_DBTP_DSJW_SET(value)    (((uint32_t)(value) << MCAN_DBTP_DSJW_Pos) & MCAN_DBTP_DSJW_Mask) //!< Set Data (Re) Synchronization Jump Width
#define MCAN_DBTP_DSJW_GET(value)    (((uint32_t)(value) & MCAN_DBTP_DSJW_Mask) >> MCAN_DBTP_DSJW_Pos) //!< Get Data (Re) Synchronization Jump Width
#define MCAN_DBTP_DTSEG2_Pos         4
#define MCAN_DBTP_DTSEG2_Mask        (0xFu << MCAN_DBTP_DTSEG2_Pos)
#define MCAN_DBTP_DTSEG2_SET(value)  (((uint32_t)(value) << MCAN_DBTP_DTSEG2_Pos) & MCAN_DBTP_DTSEG2_Mask) //!< Set Data Time Segment After Sample Point
#define MCAN_DBTP_DTSEG2_GET(value)  (((uint32_t)(value) & MCAN_DBTP_DTSEG2_Mask) >> MCAN_DBTP_DTSEG2_Pos) //!< Get Data Time Segment After Sample Point
#define MCAN_DBTP_DTSEG1_Pos         8
#define MCAN_DBTP_DTSEG1_Mask        (0x1Fu << MCAN_DBTP_DTSEG1_Pos)
#define MCAN_DBTP_DTSEG1_SET(value)  (((uint32_t)(value) << MCAN_DBTP_DTSEG1_Pos) & MCAN_DBTP_DTSEG1_Mask) //!< Set Data Time Segment Before Sample Point
#define MCAN_DBTP_DTSEG1_GET(value)  (((uint32_t)(value) & MCAN_DBTP_DTSEG1_Mask) >> MCAN_DBTP_DTSEG1_Pos) //!< Get Data Time Segment Before Sample Point
#define MCAN_DBTP_DBRP_Pos           16
#define MCAN_DBTP_DBRP_Mask          (0x1Fu << MCAN_DBTP_DBRP_Pos)
#define MCAN_DBTP_DBRP_SET(value)    (((uint32_t)(value) << MCAN_DBTP_DBRP_Pos) & MCAN_DBTP_DBRP_Mask) //!< Set Data Bit Rate Prescaler
#define MCAN_DBTP_DBRP_GET(value)    (((uint32_t)(value) & MCAN_DBTP_DBRP_Mask) >> MCAN_DBTP_DBRP_Pos) //!< Get Data Bit Rate Prescaler
#define MCAN_DBTP_TDC_EN             (0x1u << 23) //!< Transmitter Delay Compensation Enabled
#define MCAN_DBTP_TDC_DIS            (0x0u << 23) //!< Transmitter Delay Compensation Disabled

//-----------------------------------------------------------------------------

/*! MCAN Test Register (Read/Write, Offset: 0x10)
 * @note Write access to the Test Register has to be enabled by setting bit MCAN_CCCR.TEST to '1'
 * All MCAN Test Register functions are set to their reset values when bit MCAN_CCCR.TEST is cleared.
 * Loop Back mode and software control of pin CANTX are hardware test modes. Programming of TX ≠ 0 disturbs the message transfer on the CAN bus.
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TEST_Register
{
  uint32_t TEST;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t      :  4; //!<  0- 3
    uint32_t LBCK :  1; //!<  4    - LBCK: Loop Back Mode: '1' = Loop Back mode is enabled ; '0' = Reset value. Loop Back mode is disabled
    uint32_t TX   :  2; //!<  5- 6 - Control of Transmit Pin
    uint32_t RX   :  1; //!<  7    - Receive Pin: '1' = The CAN bus is recessive (CANRX = '1') ; '0' = The CAN bus is dominant (CANRX = '0')
    uint32_t TXBNP:  5; //!<  8-12 - Tx Buffer Number Prepared. Tx Buffer number of message that is ready for transmission. Valid when PVAL is set. Valid values are 0 to 31
    uint32_t PVAL :  1; //!< 13    - Prepared Valid: '1' = Value of TXBNP valid ; '0' = Value of TXBNP not valid
    uint32_t      :  2; //!< 14-15
    uint32_t TXBNS:  5; //!< 16-20 - Tx Buffer Number Started. Tx Buffer number of message whose transmission was started last. Valid when SVAL is set. Valid values are 0 to 31
    uint32_t SVAL :  1; //!< 21    - Started Valid: '1' = Value of TXBNS valid ; '0' = Value of TXBNS not valid
    uint32_t      : 10; //!< 22-31
  } Bits;
} MCAN_TEST_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TEST_Register, 4);

#define MCAN_TEST_LOOPBACK_MODE_EN   (0x1u << 4) //!< Loop Back mode is Enabled
#define MCAN_TEST_LOOPBACK_MODE_DIS  (0x0u << 4) //!< Loop Back mode is Disabled

//! Control of Transmit Pin enumerator
typedef enum
{
  MCAN_TX_RESET                   = 0b00, //!< Reset value, CANTX controlled by the CAN Core, updated at the end of the CAN bit time
  MCAN_TX_SAMPLE_POINT_MONITORING = 0b01, //!< Sample Point can be monitored at pin CANTX
  MCAN_TX_DOMINANT                = 0b10, //!< Dominant ('0') level at pin CANTX
  MCAN_TX_RECESSIVE               = 0b11, //!< Recessive ('1') level at pin CANTX
} eMCAN_TestTxPin;

#define MCAN_TEST_TX_Pos             5
#define MCAN_TEST_TX_Mask            (0x3u << MCAN_TEST_TX_Pos)
#define MCAN_TEST_TX_SET(value)      (((uint32_t)(value) << MCAN_TEST_TX_Pos) & MCAN_TEST_TX_Mask) //!< Set Control of Transmit Pin
#define MCAN_TEST_TX_GET(value)      (((uint32_t)(value) & MCAN_TEST_TX_Mask) >> MCAN_TEST_TX_Pos) //!< Get Control of Transmit Pin

//! Receive Pin status enumerator
typedef enum
{
  MCAN_RX_DOMINANT  = 0b0, //!< Dominant ('0') level at pin CANRX
  MCAN_RX_RECESSIVE = 0b1, //!< Recessive ('1') level at pin CANRX
} eMCAN_TestRxPin;

#define MCAN_TEST_RX_Pos             7
#define MCAN_TEST_RX_Mask            (0x1u << MCAN_TEST_RX_Pos)
#define MCAN_TEST_RX_GET(value)      (((uint32_t)(value) & MCAN_TEST_RX_Mask) >> MCAN_TEST_RX_Pos) //!< Get Receive Pin status
#define MCAN_TEST_RX_RECESSIVE       (0x1u << 7) //!< The CAN bus is recessive (CANRX = '1')
#define MCAN_TEST_RX_DOMINANT        (0x0u << 7) //!< The CAN bus is dominant (CANRX = '0')
#define MCAN_TEST_TXBNP_Pos          8
#define MCAN_TEST_TXBNP_Mask         (0x1Fu << MCAN_TEST_TXBNP_Pos)
#define MCAN_TEST_TXBNP_GET(value)   (((uint32_t)(value) & MCAN_TEST_TXBNP_Mask) >> MCAN_TEST_TXBNP_Pos) //!< Get Tx Buffer Number Prepared
#define MCAN_TEST_PREPARE_VALID      (0x1u << 13) //!< Value of TXBNP valid
#define MCAN_TEST_PREPARE_NOT_VALID  (0x0u << 13) //!< Value of TXBNP not valid
#define MCAN_TEST_TXBNS_Pos          16
#define MCAN_TEST_TXBNS_Mask         (0x1Fu << MCAN_TEST_TXBNS_Pos)
#define MCAN_TEST_TXBNS_GET(value)   (((uint32_t)(value) & MCAN_TEST_TXBNS_Mask) >> MCAN_TEST_TXBNS_Pos) //!< Get Tx Buffer Number Started
#define MCAN_TEST_STARTED_VALID      (0x1u << 21) //!< Value of TXBNS valid
#define MCAN_TEST_STARTED_NOT_VALID  (0x0u << 21) //!< Value of TXBNS not valid

//-----------------------------------------------------------------------------

/*! MCAN RAM Watchdog Register (Read/Write, Offset: 0x14)
 * The RAM Watchdog monitors the Message RAM response time. A Message RAM access via the MCAN's Generic Master Interface starts the Message RAM Watchdog Counter with the value configured by MCAN_RWD.WDC.
 * The counter is reloaded with MCAN_RWD.WDC when the Message RAM signals successful completion by activating its READY output.
 * In case there is no response from the Message RAM until the counter has counted down to zero, the counter stops and interrupt flag MCAN_IR.WDI is set. The RAM Watchdog Counter is clocked by the system bus clock (peripheral clock).
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RWD_Register
{
  uint32_t RWD;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t WDC:  8; //!<  0- 7 - Watchdog Configuration. Start value of the Message RAM Watchdog Counter. The counter is disabled when WDC is cleared
    uint32_t WDV:  8; //!<  8-15 - Watchdog Value. Watchdog Counter Value for the current message located in RAM
    uint32_t    : 16; //!< 16-31
  } Bits;
} MCAN_RWD_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RWD_Register, 4);

#define MCAN_RWD_WATCHDOG_CONFIG_Pos         0
#define MCAN_RWD_WATCHDOG_CONFIG_Mask        (0xFFu << MCAN_RWD_WATCHDOG_CONFIG_Pos)
#define MCAN_RWD_WATCHDOG_CONFIG_SET(value)  (((uint32_t)(value) << MCAN_RWD_WATCHDOG_CONFIG_Pos) & MCAN_RWD_WATCHDOG_CONFIG_Mask) //!< Set Watchdog Configuration
#define MCAN_RWD_WATCHDOG_CONFIG_GET(value)  (((uint32_t)(value) & MCAN_RWD_WATCHDOG_CONFIG_Mask) >> MCAN_RWD_WATCHDOG_CONFIG_Pos) //!< Get Watchdog Configuration
#define MCAN_RWD_WATCHDOG_VALUE_Pos          8
#define MCAN_RWD_WATCHDOG_VALUE_Mask         (0xFFu << MCAN_RWD_WATCHDOG_VALUE_Pos)
#define MCAN_RWD_WATCHDOG_VALUE_GET(value)   (((uint32_t)(value) & MCAN_RWD_WATCHDOG_VALUE_Mask) >> MCAN_RWD_WATCHDOG_VALUE_Pos) //!< Get Watchdog Value

//-----------------------------------------------------------------------------

//! MCAN CC Control Register (Read/Write, Offset: 0x18)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_CCCR_Register
{
  uint32_t CCCR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t INIT:  1; //!<  0    - Initialization: '1' = Initialization is started ; '0' = Normal operation
    uint32_t CCE :  1; //!<  1    - Configuration Change Enable: '1' = The processor has write access to the protected configuration registers (while MCAN_CCCR.INIT = '1') ; '0' = The processor has no write access to the protected configuration registers
    uint32_t ASM :  1; //!<  2    - Restricted Operation Mode: '1' = Restricted Operation mode active ; '0' = Normal CAN operation
    uint32_t CSA :  1; //!<  3    - Clock Stop Acknowledge: '1' = MCAN may be set in power down by stopping the peripheral clock and the CAN core clock ; '0' = No clock stop acknowledged
    uint32_t CSR :  1; //!<  4    - Clock Stop Request: '1' = Clock stop requested. When clock stop is requested, first INIT and then CSA will be set after all pending transfer requests have been completed and the CAN bus reached idle ; '0' = No clock stop is requested
    uint32_t MON :  1; //!<  5    - Bus Monitoring Mode: '1' = Bus Monitoring mode is enabled ; '0' = Bus Monitoring mode is disabled
    uint32_t DAR :  1; //!<  6    - Disable Automatic Retransmission: '1' = Automatic retransmission disabled ; '0' = Automatic retransmission of messages not transmitted successfully enabled
    uint32_t TEST:  1; //!<  7    - Test Mode Enable: '1' = Test mode, write access to MCAN_TEST register enabled ; '0' = Normal operation, MCAN_TEST register holds reset values
    uint32_t FDOE:  1; //!<  8    - CAN FD Operation Enable: '1' = FD operation enabled ; '0' = FD operation disabled
    uint32_t BRSE:  1; //!<  9    - Bit Rate Switching Enable: '1' = Bit rate switching for transmissions enabled ; '0' = Bit rate switching for transmissions disabled
    uint32_t UTSU:  1; //!< 10    - When UTSU is set, 16-bit Wide Message Markers are also enabled regardless of the value of WMM: '1' = External time stamping by TSU ; '0' = Internal time stamping
    uint32_t WMM :  1; //!< 11    - Enables the use of 16-bit Wide Message Markers. When 16-bit Wide Message Markers are used (WMM = '1'), 16-bit inter nal timestamping is disabled for the Tx Event FIFO: '1' = 16-bit Message Marker used, replacing 16-bit timestamps in Tx Event FIFO ; '0' = 8-bit Message Marker used
    uint32_t PXHD:  1; //!< 12    - Protocol Exception Event Handling: '1' = Protocol exception handling disabled ; '0' = Protocol exception handling enabled
    uint32_t EFBI:  1; //!< 13    - Edge Filtering during Bus Integration: '1' = Edge filtering is enabled. Two consecutive dominant tq required to detect an edge for hard synchronization ; '0' = Edge filtering is disabled
    uint32_t TXP :  1; //!< 14    - Transmit Pause: '1' = Transmit pause enabled ; '0' = Transmit pause disabled
    uint32_t NISO:  1; //!< 15    - Non-ISO Operation: '1' = CAN FD frame format according to Bosch CAN FD Specification V1.0 ; '0' = CAN FD frame format according to ISO11898-1 (default)
    uint32_t     : 16; //!<  16-31
  } Bits;
} MCAN_CCCR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_CCCR_Register, 4);

#define MCAN_CCCR_INIT_STARTED                  (0x1u <<  0) //!< Initialization is started
#define MCAN_CCCR_NORMAL_OPERATION              (0x0u <<  0) //!< Normal operation
#define MCAN_CCCR_CONF_WRITE_PROTECT_DIS        (0x1u <<  1) //!< The processor has write access to the protected configuration registers (while MCAN_CCCR.INIT = '1')
#define MCAN_CCCR_CONF_WRITE_PROTECT_EN         (0x0u <<  1) //!< The processor has no write access to the protected configuration registers
#define MCAN_CCCR_RESTRICTED_OPERATION          (0x1u <<  2) //!< Restricted Operation mode active
#define MCAN_CCCR_NORMAL_CAN_OPERATION          (0x0u <<  2) //!< Normal CAN operation
#define MCAN_CCCR_CLOCK_STOP_ACK                (0x1u <<  3) //!< Clock Stop Acknowledge
#define MCAN_CCCR_NO_CLOCK_STOP_ACK             (0x0u <<  3) //!< No clock stop acknowledged
#define MCAN_CCCR_CLOCK_STOP_REQ                (0x1u <<  4) //!< Clock Stop Request
#define MCAN_CCCR_NO_CLOCK_STOP_REQ             (0x0u <<  4) //!< No clock stop is requested
#define MCAN_CCCR_BUS_MONITOR_EN                (0x1u <<  5) //!< Bus Monitoring mode is enabled
#define MCAN_CCCR_BUS_MONITOR_DIS               (0x0u <<  5) //!< Bus Monitoring mode is disabled
#define MCAN_CCCR_AUTOMATIC_RETRANSMISSION_DIS  (0x1u <<  6) //!< Automatic retransmission disabled
#define MCAN_CCCR_AUTOMATIC_RETRANSMISSION_EN   (0x0u <<  6) //!< Automatic retransmission of messages not transmitted successfully enabled
#define MCAN_CCCR_TEST_MODE_EN                  (0x1u <<  7) //!< Test mode, write access to MCAN_TEST register enabled
#define MCAN_CCCR_TEST_MODE_DIS                 (0x0u <<  7) //!< Normal operation, MCAN_TEST register holds reset values
#define MCAN_CCCR_CAN_FD_MODE_EN                (0x1u <<  8) //!< CAN FD operation enabled
#define MCAN_CCCR_CAN_FD_MODE_DIS               (0x0u <<  8) //!< CAN FD operation disabled
#define MCAN_CCCR_BITRATE_SWITCHING_EN          (0x1u <<  9) //!< Bit rate switching for transmissions enabled
#define MCAN_CCCR_BITRATE_SWITCHING_DIS         (0x0u <<  9) //!< Bit rate switching for transmissions disabled
#define MCAN_CCCR_EXTERNAL_TIMESTAMP_TSU        (0x1u << 10) //!< External time stamping by TSU
#define MCAN_CCCR_INTERNAL_TIMESTAMP            (0x0u << 10) //!< Internal time stamping
#define MCAN_CCCR_16BIT_MESSAGE_MARKER_USED     (0x1u << 11) //!< 16-bit Message Marker used, replacing 16-bit timestamps in Tx Event FIFO
#define MCAN_CCCR_8BIT_MESSAGE_MARKER_USED      (0x0u << 11) //!< 8-bit Message Marker used
#define MCAN_CCCR_PROTOCOL_EXCEPTION_DIS        (0x1u << 12) //!< Protocol exception handling disabled
#define MCAN_CCCR_PROTOCOL_EXCEPTION_EN         (0x0u << 12) //!< Protocol exception handling enabled
#define MCAN_CCCR_EDGE_FILTERING_EN             (0x1u << 13) //!< Edge filtering is enabled. Two consecutive dominant tq required to detect an edge for hard synchronization
#define MCAN_CCCR_EDGE_FILTERING_DIS            (0x0u << 13) //!< Edge filtering is disabled
#define MCAN_CCCR_TRANSMIT_PAUSE_EN             (0x1u << 14) //!< Transmit pause enabled
#define MCAN_CCCR_TRANSMIT_PAUSE_DIS            (0x0u << 14) //!< Transmit pause disabled
#define MCAN_CCCR_NONISO_OPERATION_EN           (0x1u << 15) //!< CAN FD frame format according to Bosch CAN FD Specification V1.0
#define MCAN_CCCR_NONISO_OPERATION_DIS          (0x0u << 15) //!< CAN FD frame format according to ISO11898-1 (default)
#define MCAN_CCCR_Mask                          (0x0000FFFFu) //!< CCCR register mask

//! CAN Controller Operation Modes
typedef enum
{
  MCAN_NORMAL_CAN20_MODE         = MCAN_CCCR_NORMAL_CAN_OPERATION,                                             //!< Set Normal CAN 2.0 mode; possible error frames on CAN FD frames
  MCAN_SLEEP_MODE                = MCAN_CCCR_CLOCK_STOP_REQ,                                                   //!< Set Sleep mode
  MCAN_TEST_MODE                 = MCAN_CCCR_INIT_STARTED | MCAN_CCCR_TEST_MODE_EN,                            //!< Set Test mode
  MCAN_INTERNAL_LOOPBACK_MODE    = MCAN_CCCR_INIT_STARTED | MCAN_CCCR_TEST_MODE_EN | MCAN_CCCR_BUS_MONITOR_EN, //!< Set Internal Loopback mode
  MCAN_LISTEN_ONLY_MODE          = MCAN_CCCR_BUS_MONITOR_EN,                                                   //!< Set Listen Only mode, Bus Monitoring Mode
  MCAN_INITIALIZATION_MODE       = MCAN_CCCR_INIT_STARTED,                                                     //!< Set Initialization mode
  MCAN_EXTERNAL_LOOPBACK_MODE    = MCAN_CCCR_INIT_STARTED | MCAN_CCCR_TEST_MODE_EN | 0x80000000,               //!< Set External Loopback mode
  MCAN_NORMAL_CANFD_MODE         = MCAN_CCCR_CAN_FD_MODE_EN,                                                   //!< Set Normal CAN FD mode; supports mixing of CAN FD and Classic CAN 2.0 frames
  MCAN_RESTRICTED_OPERATION_MODE = MCAN_CCCR_RESTRICTED_OPERATION,                                             //!< Set Restricted Operation mode
} eMCAN_OperationMode;

#define MCAN_MODE_CLEAR_MASK          ( ~(MCAN_CCCR_CLOCK_STOP_REQ | MCAN_CCCR_TEST_MODE_EN | MCAN_CCCR_BUS_MONITOR_EN | MCAN_CCCR_CAN_FD_MODE_EN | MCAN_CCCR_RESTRICTED_OPERATION) ) //!< Mask for clearing operation mode
#define MCAN_IN_INIT_AND_UNPROTECTED  ( MCAN_CCCR_INIT_STARTED | MCAN_CCCR_CONF_WRITE_PROTECT_DIS ) //!< Initialization and write protection disabled flags mask

//-----------------------------------------------------------------------------

/*! MCAN Nominal Bit Timing and Prescaler Register (Read/Write, Offset: 0x1C)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN_CCCR
 * The CAN bit time may be programmed in the range of 4 to 385 time quanta. The CAN time quantum may be programmed in the range of 1 to 512 CAN core clock periods. tq = tcore clock x (NBRP + 1).
 * NTSEG1 is the sum of Prop_Seg and Phase_Seg1. NTSEG2 is Phase_Seg2.
 * Therefore the length of the bit time is (programmed values) [NTSEG1 + NTSEG2 + 3] tq or (functional values) [Sync_Seg + Prop_Seg + Phase_Seg1 + Phase_Seg2] tq.
 * The Information Processing Time (IPT) is zero, meaning the data for the next bit is available at the first clock edge after the sample point
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_NBTP_Register
{
  uint32_t NBTP;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t NTSEG2: 7; //!<  0- 6 - Nominal Time Segment After Sample Point: '0' to '127': The duration of time segment is tq x (NTSEG2 + 1)
    uint32_t       : 1; //!<  7
    uint32_t NTSEG1: 8; //!<  8-15 - Nominal Time Segment Before Sample Point: '0' = Forbidden ; '1' to '255' = The duration of time segment is tq x (NTSEG1 + 1)
    uint32_t NBRP  : 9; //!< 16-24 - Nominal Bit Rate Prescaler: '0' to '511': The value by which the oscillator frequency is divided for generating the CAN time quanta. The CAN time is built up from a multiple of this quanta. CAN time quantum (tq) = tcore clock x (NBRP + 1)
    uint32_t NSJW  : 7; //!< 25-31 - Nominal (Re) Synchronization Jump Width: '0' to '127': The duration of a synchronization jump is tq x (NSJW + 1)
  } Bits;
} MCAN_NBTP_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_NBTP_Register, 4);

#define MCAN_NBTP_NTSEG2_Pos         0
#define MCAN_NBTP_NTSEG2_Mask        (0x7Fu << MCAN_NBTP_NTSEG2_Pos)
#define MCAN_NBTP_NTSEG2_SET(value)  (((uint32_t)(value) << MCAN_NBTP_NTSEG2_Pos) & MCAN_NBTP_NTSEG2_Mask) //!< Set Nominal Time Segment After Sample Point
#define MCAN_NBTP_NTSEG2_GET(value)  (((uint32_t)(value) & MCAN_NBTP_NTSEG2_Mask) >> MCAN_NBTP_NTSEG2_Pos) //!< Get Nominal Time Segment After Sample Point
#define MCAN_NBTP_NTSEG1_Pos         8
#define MCAN_NBTP_NTSEG1_Mask        (0xFFu << MCAN_NBTP_NTSEG1_Pos)
#define MCAN_NBTP_NTSEG1_SET(value)  (((uint32_t)(value) << MCAN_NBTP_NTSEG1_Pos) & MCAN_NBTP_NTSEG1_Mask) //!< Set Nominal Time Segment Before Sample Point
#define MCAN_NBTP_NTSEG1_GET(value)  (((uint32_t)(value) & MCAN_NBTP_NTSEG1_Mask) >> MCAN_NBTP_NTSEG1_Pos) //!< Get Nominal Time Segment Before Sample Point
#define MCAN_NBTP_NBRP_Pos           16
#define MCAN_NBTP_NBRP_Mask          (0x1FFu << MCAN_NBTP_NBRP_Pos)
#define MCAN_NBTP_NBRP_SET(value)    (((uint32_t)(value) << MCAN_NBTP_NBRP_Pos) & MCAN_NBTP_NBRP_Mask) //!< Set Nominal Bit Rate Prescaler
#define MCAN_NBTP_NBRP_GET(value)    (((uint32_t)(value) & MCAN_NBTP_NBRP_Mask) >> MCAN_NBTP_NBRP_Pos) //!< Get Nominal Bit Rate Prescaler
#define MCAN_NBTP_NSJW_Pos           25
#define MCAN_NBTP_NSJW_Mask          (0x7Fu << MCAN_NBTP_NSJW_Pos)
#define MCAN_NBTP_NSJW_SET(value)    (((uint32_t)(value) << MCAN_NBTP_NSJW_Pos) & MCAN_NBTP_NSJW_Mask) //!< Set Nominal (Re) Synchronization Jump Width
#define MCAN_NBTP_NSJW_GET(value)    (((uint32_t)(value) & MCAN_NBTP_NSJW_Mask) >> MCAN_NBTP_NSJW_Pos) //!< Get Nominal (Re) Synchronization Jump Width

//-----------------------------------------------------------------------------

//! MCAN Timestamp Counter Configuration Register (Read/Write, Offset: 0x20)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TSCC_Register
{
  uint32_t TSCC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TSS:  2; //!<  0- 1 - Timestamp Select
    uint32_t    : 14; //!<  2-15
    uint32_t TCP:  4; //!< 16-19 - Configures the timestamp and timeout counters time unit in multiples of CAN bit times [1..16]. The actual interpretation by the hardware of this value is such that one more than the value programmed here is used
    uint32_t    : 12; //!< 20-31
  } Bits;
} MCAN_TSCC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TSCC_Register, 4);

//! TimeStamp Select enumerator
typedef enum
{
  MCAN_TIMESTAMP_ALWAYS_0         = 0b00, //!< Timestamp counter value always 0x0000
  MCAN_TIMESTAMP_TCP_INCREMENT    = 0b01, //!< Timestamp counter value incremented according to TCP
  MCAN_TIMESTAMP_EXTERN_TIMESTAMP = 0b10, //!< External timestamp counter value used
  MCAN_TIMESTAMP_ALWAYS_0_        = 0b11, //!< Timestamp counter value always 0x0000
} eMCAN_TimeStampSelect;

#define MCAN_TSCC_TIMESTAMP_SELECT_Pos         0
#define MCAN_TSCC_TIMESTAMP_SELECT_Mask        (0x3u << MCAN_TSCC_TIMESTAMP_SELECT_Pos)
#define MCAN_TSCC_TIMESTAMP_SELECT_SET(value)  (((uint32_t)(value) << MCAN_TSCC_TIMESTAMP_SELECT_Pos) & MCAN_TSCC_TIMESTAMP_SELECT_Mask) //!< Set Timestamp Select
#define MCAN_TSCC_TIMESTAMP_SELECT_GET(value)  (((uint32_t)(value) & MCAN_TSCC_TIMESTAMP_SELECT_Mask) >> MCAN_TSCC_TIMESTAMP_SELECT_Pos) //!< Get Timestamp Select

#define MCAN_TSCC_TCP_MINVALUE    ( 0u )
#define MCAN_TSCC_TCP_Pos         16
#define MCAN_TSCC_TCP_Bits        4
#define MCAN_TSCC_TCP_MAXVALUE    ((1u << MCAN_TSCC_TCP_Bits) - 1u)
#define MCAN_TSCC_TCP_Mask        (MCAN_TSCC_TCP_MAXVALUE << MCAN_TSCC_TCP_Pos)
#define MCAN_TSCC_TCP_SET(value)  (((uint32_t)(value) << MCAN_TSCC_TCP_Pos) & MCAN_TSCC_TCP_Mask) //!< Set Configures the timestamp and timeout counters time unit in multiples of CAN bit times
#define MCAN_TSCC_TCP_GET(value)  (((uint32_t)(value) & MCAN_TSCC_TCP_Mask) >> MCAN_TSCC_TCP_Pos) //!< Get Configures the timestamp and timeout counters time unit in multiples of CAN bit times

//-----------------------------------------------------------------------------

/*! MCAN Timestamp Counter Value Register (Read/Write, Offset: 0x24)
 * The internal/external Timestamp Counter value is captured on start of frame (both Receive and Transmit).
 * When MCAN_TSCC.TSS = 1, the Timestamp Counter is incremented in multiples of CAN bit times [1..16] depending on the configuration of MCAN_TSCC.TCP. A wrap around sets interrupt flag MCAN_IR.TSW. Write access resets the counter to zero.
 * When MCAN_TSCC.TSS = 2, TSC reflects the external Timestamp Counter value. Thus a write access has no impact
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TSCV_Register
{
  uint32_t TSCV;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TSC: 16; //!<  0-15 - Timestamp Counter: The internal/external Timestamp Counter value is captured on start of frame (both Receive and Transmit)
    uint32_t    : 16; //!< 16-31
  } Bits;
} MCAN_TSCV_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TSCV_Register, 4);

#define MCAN_TSCV_TIMESTAMP_Pos         0
#define MCAN_TSCV_TIMESTAMP_Mask        (0xFFFFu << MCAN_TSCV_TIMESTAMP_Pos)
#define MCAN_TSCV_TIMESTAMP_SET(value)  (((uint32_t)(value) << MCAN_TSCV_TIMESTAMP_Pos) & MCAN_TSCV_TIMESTAMP_Mask) //!< Set Timestamp Counter
#define MCAN_TSCV_TIMESTAMP_GET(value)  (((uint32_t)(value) & MCAN_TSCV_TIMESTAMP_Mask) >> MCAN_TSCV_TIMESTAMP_Pos) //!< Get Timestamp Counter

//-----------------------------------------------------------------------------

/*! MCAN Timeout Counter Configuration Register (Read/Write, Offset: 0x28)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * When MCAN_TSCC.TSS = 1, the Timestamp Counter is incremented in multiples of CAN bit times [1..16] depending on the configuration of MCAN_TSCC.TCP. A wrap around sets interrupt flag MCAN_IR.TSW. Write access resets the counter to zero.
 * When MCAN_TSCC.TSS = 2, TSC reflects the external Timestamp Counter value. Thus a write access has no impact.
 * When operating in Continuous mode, a write to MCAN_TOCV presets the counter to the value configured by MCAN_TOCC.TOP and continues down-counting.
 * When the Timeout Counter is controlled by one of the FIFOs, an empty FIFO presets the counter to the value configured by MCAN_TOCC.TOP. Down-counting is started when the first FIFO element is stored
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TOCC_Register
{
  uint32_t TOCC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ETOC:  1; //!<  0    - Enable Timeout Counter: '1' = Timeout Counter enabled ; '0' = Timeout Counter disabled
    uint32_t TOS :  2; //!<  1- 2 - Timeout Select
    uint32_t     : 13; //!<  3-15
    uint32_t TOP : 16; //!< 16-31 - Timeout Period. Start value of the Timeout Counter (down-counter). Configures the Timeout Period
  } Bits;
} MCAN_TOCC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TOCC_Register, 4);

#define MCAN_TOCC_TIMEOUT_COUNTER_EN   (0x1u << 0) //!< Timeout Counter enabled
#define MCAN_TOCC_TIMEOUT_COUNTER_DIS  (0x0u << 0) //!< Timeout Counter disabled

//! Timeout Select enumerator
typedef enum
{
  MCAN_TIMEOUT_CONTINUOUS        = 0b00, //!< Continuous operation
  MCAN_TIMEOUT_TX_EVENT_TIMEOUT  = 0b01, //!< Timeout controlled by Tx Event FIFO
  MCAN_TIMEOUT_RX0_EVENT_TIMEOUT = 0b10, //!< Timeout controlled by Receive FIFO 0
  MCAN_TIMEOUT_RX1_EVENT_TIMEOUT = 0b11, //!< Timeout controlled by Receive FIFO 1
} eMCAN_TimeoutSelect;

#define MCAN_TOCC_TIMEOUT_SELECT_Pos         1
#define MCAN_TOCC_TIMEOUT_SELECT_Mask        (0x3u << MCAN_TOCC_TIMEOUT_SELECT_Pos)
#define MCAN_TOCC_TIMEOUT_SELECT_SET(value)  (((uint32_t)(value) << MCAN_TOCC_TIMEOUT_SELECT_Pos) & MCAN_TOCC_TIMEOUT_SELECT_Mask) //!< Set Timeout Select
#define MCAN_TOCC_TIMEOUT_SELECT_GET(value)  (((uint32_t)(value) & MCAN_TOCC_TIMEOUT_SELECT_Mask) >> MCAN_TOCC_TIMEOUT_SELECT_Pos) //!< Get Timeout Select
#define MCAN_TOCC_TIMEOUT_PERIOD_Pos         16
#define MCAN_TOCC_TIMEOUT_PERIOD_Mask        (0xFFFFu << MCAN_TOCC_TIMEOUT_PERIOD_Pos)
#define MCAN_TOCC_TIMEOUT_PERIOD_SET(value)  (((uint32_t)(value) << MCAN_TOCC_TIMEOUT_PERIOD_Pos) & MCAN_TOCC_TIMEOUT_PERIOD_Mask) //!< Set Timeout Period
#define MCAN_TOCC_TIMEOUT_PERIOD_GET(value)  (((uint32_t)(value) & MCAN_TOCC_TIMEOUT_PERIOD_Mask) >> MCAN_TOCC_TIMEOUT_PERIOD_Pos) //!< Get Timeout Period

//-----------------------------------------------------------------------------

/*! MCAN Timeout Counter Value Register (Read/Write, Offset: 0x2C)
 * The Timeout Counter is decremented in multiples of CAN bit times [1…16] depending on the configuration of MCAN_TSCC.TCP.
 * When decremented to zero, interrupt flag MCAN_IR.TOO is set and the Timeout Counter is stopped. Start and reset/restart conditions are configured via MCAN_TOCC.TOS
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TOCV_Register
{
  uint32_t TOCV;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TOC: 16; //!<  0-15 - Timeout Counter
    uint32_t    : 16; //!< 16-31
  } Bits;
} MCAN_TOCV_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TOCV_Register, 4);

#define MCAN_TOCV_TIMEOUT_COUNTER_Pos         0
#define MCAN_TOCV_TIMEOUT_COUNTER_Mask        (0xFFFFu << MCAN_TOCV_TIMEOUT_COUNTER_Pos)
#define MCAN_TOCV_TIMEOUT_COUNTER_SET(value)  (((uint32_t)(value) << MCAN_TOCV_TIMEOUT_COUNTER_Pos) & MCAN_TOCV_TIMEOUT_COUNTER_Mask) //!< Set Timeout Counter
#define MCAN_TOCV_TIMEOUT_COUNTER_GET(value)  (((uint32_t)(value) & MCAN_TOCV_TIMEOUT_COUNTER_Mask) >> MCAN_TOCV_TIMEOUT_COUNTER_Pos) //!< Get Timeout Counter

//-----------------------------------------------------------------------------

//! MCAN Error Counter Register (Read-only, Offset: 0x40)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ECR_Register
{
  uint32_t ECR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TEC: 8; //!<  0- 7 - Transmit Error Counter. Actual state of the Transmit Error Counter
    uint32_t REC: 7; //!<  8-14 - Receive Error Counter. Actual state of the Receive Error Counter
    uint32_t RP : 1; //!< 15    - Receive Error Passive: '1' = The Receive Error Counter has reached the error passive level of 128 ; '0' = The Receive Error Counter is below the error passive level of 128
    uint32_t CEL: 8; //!< 16-23 - CAN Error Logging. The counter is incremented each time when a CAN protocol error causes the Transmit Error Counter or the Receive Error Counter to be incremented. It is reset by read access to CEL. The counter stops at 0xFF; the next increment of TEC or REC sets interrupt flag IR.ELO
    uint32_t    : 8; //!< 24-31
  } Bits;
} MCAN_ECR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ECR_Register, 4);

#define MCAN_ECR_TRANSMIT_ERROR_COUNTER_Pos         0
#define MCAN_ECR_TRANSMIT_ERROR_COUNTER_Mask        (0xFFu << MCAN_ECR_TRANSMIT_ERROR_COUNTER_Pos)
#define MCAN_ECR_TRANSMIT_ERROR_COUNTER_GET(value)  (((uint32_t)(value) & MCAN_ECR_TRANSMIT_ERROR_COUNTER_Mask) >> MCAN_ECR_TRANSMIT_ERROR_COUNTER_Pos) //!< Get Transmit Error Counter
#define MCAN_ECR_RECEIVE_ERROR_COUNTER_Pos          8
#define MCAN_ECR_RECEIVE_ERROR_COUNTER_Mask         (0x7Fu << MCAN_ECR_RECEIVE_ERROR_COUNTER_Pos)
#define MCAN_ECR_RECEIVE_ERROR_COUNTER_GET(value)   (((uint32_t)(value) & MCAN_ECR_RECEIVE_ERROR_COUNTER_Mask) >> MCAN_ECR_RECEIVE_ERROR_COUNTER_Pos) //!< Get Receive Error Counter
#define MCAN_ECR_RECEIVE_ERROR_REACH_PASSIVE        (0x1u << 15) //!< The Receive Error Counter has reached the error passive level of 128
#define MCAN_ECR_RECEIVE_ERROR_BELOW_PASSIVE        (0x0u << 15) //!< The Receive Error Counter is below the error passive level of 128
#define MCAN_ECR_CAN_ERROR_LOGGING_Pos              16
#define MCAN_ECR_CAN_ERROR_LOGGING_Mask             (0xFFu << MCAN_ECR_CAN_ERROR_LOGGING_Pos)
#define MCAN_ECR_CAN_ERROR_LOGGING_GET(value)       (((uint32_t)(value) & MCAN_ECR_CAN_ERROR_LOGGING_Mask) >> MCAN_ECR_CAN_ERROR_LOGGING_Pos) //!< Get CAN Error Logging

//-----------------------------------------------------------------------------

//! MCAN Protocol Status Register (Read-only, Offset: 0x44)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_PSR_Register
{
  uint32_t PSR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t LEC : 3; //!<  0- 2 - Last Error Code. The LEC indicates the type of the last error to occur on the CAN bus. This field is cleared when a message has been transferred (reception or transmission) without error
    uint32_t ACT : 2; //!<  3- 4 - Activity. Monitors the CAN communication state of the CAN module
    uint32_t EP  : 1; //!<  5    - Error Passive: '1' = The MCAN is in the Error_Passive state ; '0' = The MCAN is in the Error_Active state. It normally takes part in bus communication and sends an active error flag when an error has been detected
    uint32_t EW  : 1; //!<  6    - Warning Status: '1' = At least one of error counter has reached the Error_Warning limit of 96 ; '0' = Both error counters are below the Error_Warning limit of 96
    uint32_t BO  : 1; //!<  7    - Bus_Off Status: '1' = The MCAN is in Bus_Off state ; '0' = The MCAN is not Bus_Off
    uint32_t DLEC: 3; //!<  8-10 - Data Phase Last Error Code. Type of last error that occurred in the data phase of a CAN FD format frame with its BRS flag set. Coding is the same as for LEC. This field will be cleared to zero when a CAN FD format frame with its BRS flag set has been transferred (reception or transmission) without error.
    uint32_t RESI: 1; //!< 11    - ESI Flag of Last Received CAN FD Message. This bit is set together with RFDF, independently from acceptance filtering: '1' = Last received CAN FD message had its ESI flag set ; '0' = Last received CAN FD message did not have its ESI flag set
    uint32_t RBRS: 1; //!< 12    - BRS Flag of Last Received CAN FD Message. This bit is set together with RFDF, independently from acceptance filtering: '1' = Last received CAN FD message had its BRS flag set ; '0' = Last received CAN FD message did not have its BRS flag set
    uint32_t RFDF: 1; //!< 13    - Received a CAN FD Message. This bit is set independently from acceptance filtering: '1' = Message in CAN FD format with FDF flag set has been received ; '0' = Since this bit was reset by the CPU, no CAN FD message has been received
    uint32_t PXE : 1; //!< 14    - Protocol Exception Event: '1' = Protocol exception event occurred ; '0' = No protocol exception event occurred since last read access
    uint32_t     : 1; //!< 15
    uint32_t TDCV: 7; //!< 16-22 - Transmitter Delay Compensation Value: '0' to '127': Position of the secondary sample point, in CAN core clock periods, defined by the sum of the measured delay from CANTX to CANRX and MCAN_TDCR.TDCO
    uint32_t     : 9; //!< 23-31
  } Bits;
} MCAN_PSR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_PSR_Register, 4);

//! Last Error Code enumerator for MCAN_PSR.LEC and MCAN_PSR.DLEC
typedef enum
{
  MCAN_NO_LAST_ERROR        = 0b000, //!< No error occurred since LEC has been reset by successful reception or transmission
  MCAN_LAST_STUFF_ERROR     = 0b001, //!< More than 5 equal bits in a sequence have occurred in a part of a received message where this is not allowed
  MCAN_LAST_FORM_ERROR      = 0b010, //!< A fixed format part of a received frame has the wrong format
  MCAN_LAST_ACK_ERROR       = 0b011, //!< The message transmitted by the MCAN was not acknowledged by another node
  MCAN_LAST_BIT1_ERROR      = 0b100, //!< During transmission of a message (with the exception of the arbitration field), the device tried to send a recessive level (bit of logical value '1'), but the monitored bus value was dominant
  MCAN_LAST_BIT0_ERROR      = 0b101, //!< During transmission of a message (or acknowledge bit, or active error flag, or overload flag), the device tried to send a dominant level (data or identifier bit logical value '0'), but the monitored bus value was recessive. During Bus_Off recovery, this status is set each time a sequence of 11 recessive bits has been monitored. This enables the processor to monitor the proceeding of the Bus_Off recovery sequence (indicating the bus is not stuck at dominant or continuously disturbed)
  MCAN_LAST_CRC_ERROR       = 0b110, //!< The CRC check sum of a received message was incorrect. The CRC of an incoming message does not match the CRC calculated from the received data
  MCAN_LAST_NO_CHANGE_ERROR = 0b111, //!< Any read access to the Protocol Status Register re-initializes the LEC to '7'. When the LEC shows value '7', no CAN bus event was detected since the last processor read access to the Protocol Status Register
} eMCAN_LastErrorCode;

#define MCAN_PSR_LAST_ERROR_CODE_Pos         0
#define MCAN_PSR_LAST_ERROR_CODE_Mask        (0x7u << MCAN_PSR_LAST_ERROR_CODE_Pos)
#define MCAN_PSR_LAST_ERROR_CODE_GET(value)  (((uint32_t)(value) & MCAN_PSR_LAST_ERROR_CODE_Mask) >> MCAN_PSR_LAST_ERROR_CODE_Pos) //!< Get Last Error Code

//! Activity enumerator
typedef enum
{
  MCAN_ACTIVITY_SYNCHRONIZING = 0b00, //!< Node is synchronizing on CAN communication
  MCAN_ACTIVITY_IDLE          = 0b01, //!< Node is neither receiver nor transmitter
  MCAN_ACTIVITY_RECEIVER      = 0b10, //!< Node is operating as receiver
  MCAN_ACTIVITY_TRANSMITTER   = 0b11, //!< Node is operating as transmitter
} eMCAN_Activity;

#define MCAN_PSR_ACTIVITY_Pos                           3
#define MCAN_PSR_ACTIVITY_Mask                          (0x3u << MCAN_PSR_ACTIVITY_Pos)
#define MCAN_PSR_ACTIVITY_GET(value)                    (((uint32_t)(value) & MCAN_PSR_ACTIVITY_Mask) >> MCAN_PSR_ACTIVITY_Pos) //!< Get Activity
#define MCAN_PSR_IS_IN_ERROR_PASSIVE_STATE              (0x1u << 5) //!< The MCAN is in the Error_Passive state
#define MCAN_PSR_IS_IN_ERROR_ACTIVE_STATE               (0x0u << 5) //!< The MCAN is in the Error_Active state
#define MCAN_PSR_REACH_ERROR_WARNING_LIMIT              (0x1u << 6) //!< At least one of error counter has reached the Error_Warning limit of 96
#define MCAN_PSR_BELOW_ERROR_WARNING_LIMIT              (0x0u << 6) //!< Both error counters are below the Error_Warning limit of 96
#define MCAN_PSR_BUS_OFF_STATE                          (0x1u << 7) //!< The MCAN is in Bus_Off state
#define MCAN_PSR_NOT_IN_BUS_OFF_STATE                   (0x0u << 7) //!< The MCAN is not Bus_Off
#define MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_Pos         8
#define MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_Mask        (0x7u << MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_Pos)
#define MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_GET(value)  (((uint32_t)(value) & MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_Mask) >> MCAN_PSR_DATA_PHASE_LAST_ERROR_CODE_Pos) //!< Get Data Phase Last Error Code
#define MCAN_PSR_LAST_MESSAGE_HAD_ESI_FLAG              (0x1u << 11) //!< Last received CAN FD message had its ESI flag set
#define MCAN_PSR_LAST_MESSAGE_DID_NOT_HAD_ESI_FLAG      (0x0u << 11) //!< Last received CAN FD message did not have its ESI flag set
#define MCAN_PSR_LAST_MESSAGE_HAD_BRS_FLAG              (0x1u << 12) //!< Last received CAN FD message had its BRS flag set
#define MCAN_PSR_LAST_MESSAGE_DID_NOT_HAD_BRS_FLAG      (0x0u << 12) //!< Last received CAN FD message did not have its BRS flag set
#define MCAN_PSR_RECEIVE_CANFD_MESSAGE                  (0x1u << 13) //!< Message in CAN FD format with FDF flag set has been received
#define MCAN_PSR_DID_NOT_RECEIVE_CANFD_MESSAGE          (0x0u << 13) //!< Since this bit was reset by the CPU, no CAN FD message has been received
#define MCAN_PSR_PROTOCOL_EXCEPTION_EVENT_OCCURED       (0x1u << 14) //!< Protocol exception event occurred
#define MCAN_PSR_NO_PROTOCOL_EXCEPTION_EVENT            (0x0u << 14) //!< No protocol exception event occurred since last read access
#define MCAN_PSR_TDCV_Pos                               16
#define MCAN_PSR_TDCV_Mask                              (0x7Fu << MCAN_PSR_TDCV_Pos)
#define MCAN_PSR_TDCV_GET(value)                        (((uint32_t)(value) & MCAN_PSR_TDCV_Mask) >> MCAN_PSR_TDCV_Pos) //!< Get Transmitter Delay Compensation Value

//-----------------------------------------------------------------------------

//! MCAN Transmitter Delay Compensation Register (Read/Write, Offset: 0x48)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TDCR_Register
{
  uint32_t TDCR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TDCF:  7; //!<  0- 6 - Transmitter Delay Compensation Filter
    uint32_t     :  1; //!<  7
    uint32_t TDCO:  7; //!<  8-14 - Transmitter Delay Compensation Offset
    uint32_t     : 17; //!< 15-31
  } Bits;
} MCAN_TDCR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TDCR_Register, 4);

#define MCAN_TDCF_Pos         0
#define MCAN_TDCF_Mask        (0x7Fu << MCAN_TDCF_Pos)
#define MCAN_TDCF_SET(value)  (((uint32_t)(value) << MCAN_TDCF_Pos) & MCAN_TDCF_Mask) //!< Set Transmitter Delay Compensation Filter
#define MCAN_TDCF_GET(value)  (((uint32_t)(value) & MCAN_TDCF_Mask) >> MCAN_TDCF_Pos) //!< Get Transmitter Delay Compensation Filter
#define MCAN_TDCO_Pos         8
#define MCAN_TDCO_Mask        (0x7Fu << MCAN_TDCO_Pos)
#define MCAN_TDCO_SET(value)  (((uint32_t)(value) << MCAN_TDCO_Pos) & MCAN_TDCO_Mask) //!< Set Transmitter Delay Compensation Offset
#define MCAN_TDCO_GET(value)  (((uint32_t)(value) & MCAN_TDCO_Mask) >> MCAN_TDCO_Pos) //!< Get Transmitter Delay Compensation Offset

//-----------------------------------------------------------------------------

/*! MCAN Interrupt Register (Read/Write, Offset: 0x50)
 * The flags are set when one of the listed conditions is detected (edge-sensitive). The flags remain set until the processor clears them.
 * A flag is cleared by writing a '1' to the corresponding bit position. Writing a '0' has no effect. A hard reset will clear the register.
 * The configuration of IE controls whether an interrupt is generated. The configuration of ILS controls on which interrupt line an interrupt is signalled
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_IR_Register
{
  uint32_t IR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RF0N: 1; //!<  0    - Receive FIFO 0 New Message: '1' = New message written to Receive FIFO 0 ; '0' = No new message written to Receive FIFO 0
    uint32_t RF0W: 1; //!<  1    - Receive FIFO 0 Watermark Reached: '1' = Receive FIFO 0 fill level reached watermark ; '0' = Receive FIFO 0 fill level below watermark
    uint32_t RF0F: 1; //!<  2    - Receive FIFO 0 Full: '1' = Receive FIFO 0 full ; '0' = Receive FIFO 0 not full
    uint32_t RF0L: 1; //!<  3    - Receive FIFO 0 Message Lost: '1' = Receive FIFO 0 message lost, also set after write attempt to Receive FIFO 0 of size zero ; '0' = No Receive FIFO 0 message lost
    uint32_t RF1N: 1; //!<  4    - Receive FIFO 1 New Message: '1' = New message written to Receive FIFO 1 ; '0' = No new message written to Receive FIFO 1
    uint32_t RF1W: 1; //!<  5    - Receive FIFO 1 Watermark Reached: '1' = Receive FIFO 1 fill level reached watermark ; '0' = Receive FIFO 1 fill level below watermark
    uint32_t RF1F: 1; //!<  6    - Receive FIFO 1 Full: '1' = Receive FIFO 1 full ; '0' = Receive FIFO 1 not full
    uint32_t RF1L: 1; //!<  7    - Receive FIFO 1 Message Lost: '1' = Receive FIFO 1 message lost, also set after write attempt to Receive FIFO 1 of size zero ; '0' = No Receive FIFO 1 message lost
    uint32_t HPM : 1; //!<  8    - High Priority Message: '1' = High priority message received ; '0' = No high priority message received
    uint32_t TC  : 1; //!<  9    - Transmission Completed: '1' = Transmission completed ; '0' = No transmission completed
    uint32_t TCF : 1; //!< 10    - Transmission Cancellation Finished: '1' = Transmission cancellation finished ; '0' = No transmission cancellation finished
    uint32_t TFE : 1; //!< 11    - Tx FIFO Empty: '1' = Tx FIFO empty ; '0' = Tx FIFO non-empty
    uint32_t TEFN: 1; //!< 12    - Tx Event FIFO New Entry: '1' = Tx Handler wrote Tx Event FIFO element ; '0' = Tx Event FIFO unchanged
    uint32_t TEFW: 1; //!< 13    - Tx Event FIFO Watermark Reached: '1' = Tx Event FIFO fill level reached watermark ; '0' = Tx Event FIFO fill level below watermark
    uint32_t TEFF: 1; //!< 14    - Tx Event FIFO Full: '1' = Tx Event FIFO full ; '0' = Tx Event FIFO not full
    uint32_t TEFL: 1; //!< 15    - Tx Event FIFO Element Lost: '1' = Tx Event FIFO element lost, also set after write attempt to Tx Event FIFO of size zero ; '0' = No Tx Event FIFO element lost
    uint32_t TSW : 1; //!< 16    - Timestamp Wraparound: '1' = Timestamp counter wrapped around ; '0' = No timestamp counter wrap-around
    uint32_t MRAF: 1; //!< 17    - Message RAM Access Failure: '1' = Message RAM access failure occurred ; '0' = No Message RAM access failure occurred
    uint32_t TOO : 1; //!< 18    - Timeout Occurred: '1' = Timeout reached ; '0' = No timeout
    uint32_t DRX : 1; //!< 19    - Message stored to Dedicated Receive Buffer: '1' = At least one received message stored into a Receive Buffer ; '0' = No Receive Buffer updated
    uint32_t BEC : 1; //!< 20    - Bit Error Corrected: '1' = Bit error detected and corrected (e.g. ECC) ; '0' = No bit error detected when reading from Message RAM
    uint32_t BEU : 1; //!< 21    - Bit Error Uncorrected: '1' = Bit error detected, uncorrected (e.g. parity logic) ; '0' = No bit error detected when reading from Message RAM
    uint32_t ELO : 1; //!< 22    - Error Logging Overflow: '1' = Overflow of CAN Error Logging Counter occurred ; '0' = CAN Error Logging Counter did not overflow
    uint32_t EP  : 1; //!< 23    - Error Passive: '1' = Error_Passive status changed ; '0' = Error_Passive status unchanged
    uint32_t EW  : 1; //!< 24    - Warning Status: '1' = Error_Warning status changed ; '0' = Error_Warning status unchanged
    uint32_t BO  : 1; //!< 25    - Bus_Off Status: '1' = Bus_Off status changed ; '0' = Bus_Off status unchanged
    uint32_t WDI : 1; //!< 26    - Watchdog Interrupt: '1' = Message RAM Watchdog event due to missing READY ; '0' = No Message RAM Watchdog event occurred
    uint32_t PEA : 1; //!< 27    - Protocol Error in Arbitration Phase: '1' = Protocol error in arbitration phase detected (MCAN_PSR.LEC differs from 0 or 7) ; '0' = No protocol error in arbitration phase
    uint32_t PED : 1; //!< 28    - Protocol Error in Data Phase: '1' = Protocol error in data phase detected (MCAN_PSR.DLEC differs from 0 or 7) ; '0' = No protocol error in data phase
    uint32_t ARA : 1; //!< 29    - Access to Reserved Address: '1' = Access to reserved address occurred ; '0' = No access to reserved address occurred
    uint32_t     : 2; //!< 30-31
  } Bits;
} MCAN_IR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_IR_Register, 4);

#define MCAN_IR_RF0N  (0x1u <<  0) //!< Receive FIFO 0 New Message
#define MCAN_IR_RF0W  (0x1u <<  1) //!< Receive FIFO 0 Watermark Reached
#define MCAN_IR_RF0F  (0x1u <<  2) //!< Receive FIFO 0 Full
#define MCAN_IR_RF0L  (0x1u <<  3) //!< Receive FIFO 0 Message Lost
#define MCAN_IR_RF1N  (0x1u <<  4) //!< Receive FIFO 1 New Message
#define MCAN_IR_RF1W  (0x1u <<  5) //!< Receive FIFO 1 Watermark Reached
#define MCAN_IR_RF1F  (0x1u <<  6) //!< Receive FIFO 1 Full
#define MCAN_IR_RF1L  (0x1u <<  7) //!< Receive FIFO 1 Message Lost
#define MCAN_IR_HPM   (0x1u <<  8) //!< High Priority Message
#define MCAN_IR_TC    (0x1u <<  9) //!< Transmission Completed
#define MCAN_IR_TCF   (0x1u << 10) //!< Transmission Cancellation Finished
#define MCAN_IR_TFE   (0x1u << 11) //!< Tx FIFO Empty
#define MCAN_IR_TEFN  (0x1u << 12) //!< Tx Event FIFO New Entry
#define MCAN_IR_TEFW  (0x1u << 13) //!< Tx Event FIFO Watermark Reached
#define MCAN_IR_TEFF  (0x1u << 14) //!< Tx Event FIFO Full
#define MCAN_IR_TEFL  (0x1u << 15) //!< Tx Event FIFO Element Lost
#define MCAN_IR_TSW   (0x1u << 16) //!< Timestamp Wraparound
#define MCAN_IR_MRAF  (0x1u << 17) //!< Message RAM Access Failure
#define MCAN_IR_TOO   (0x1u << 18) //!< Timeout Occurred
#define MCAN_IR_DRX   (0x1u << 19) //!< Message stored to Dedicated Receive Buffer
#define MCAN_IR_BEC   (0x1u << 20) //!< Bit Error Corrected
#define MCAN_IR_BEU   (0x1u << 21) //!< Bit Error Uncorrected
#define MCAN_IR_ELO   (0x1u << 22) //!< Error Logging Overflow
#define MCAN_IR_EP    (0x1u << 23) //!< Error Passive
#define MCAN_IR_EW    (0x1u << 24) //!< Warning Status
#define MCAN_IR_BO    (0x1u << 25) //!< Bus_Off Status
#define MCAN_IR_WDI   (0x1u << 26) //!< Watchdog Interrupt
#define MCAN_IR_PEA   (0x1u << 27) //!< Protocol Error in Arbitration Phase
#define MCAN_IR_PED   (0x1u << 28) //!< Protocol Error in Data Phase
#define MCAN_IR_ARA   (0x1u << 29) //!< Access to Reserved Address

#define MCAN_IR_EVENTS_STATUS_FLAGS  ( MCAN_IR_RF0N | MCAN_IR_RF0W | MCAN_IR_RF0F | MCAN_IR_RF0L | \
                                       MCAN_IR_RF1N | MCAN_IR_RF1W | MCAN_IR_RF1F | MCAN_IR_RF1L | \
                                       MCAN_IR_HPM  | MCAN_IR_TC   | MCAN_IR_TCF  | MCAN_IR_TFE  | \
                                       MCAN_IR_TEFN | MCAN_IR_TEFW | MCAN_IR_TEFF | MCAN_IR_TEFL | \
                                       MCAN_IR_TSW  | MCAN_IR_MRAF | MCAN_IR_TOO  | MCAN_IR_DRX  | \
                                       MCAN_IR_BEC  | MCAN_IR_BEU  | MCAN_IR_ELO  | MCAN_IR_EP   | \
                                       MCAN_IR_EW   | MCAN_IR_BO   | MCAN_IR_WDI  | MCAN_IR_PEA  | \
                                       MCAN_IR_PED  | MCAN_IR_ARA )                                  //!< All events status flags
#define MCAN_IR_FIFO_BUFF_TEF_FLAGS  ( MCAN_IR_RF0N | MCAN_IR_RF0W | MCAN_IR_RF0F | MCAN_IR_RF0L | \
                                       MCAN_IR_RF1N | MCAN_IR_RF1W | MCAN_IR_RF1F | MCAN_IR_RF1L | \
                                       MCAN_IR_TFE  |                                              \
                                       MCAN_IR_TEFN | MCAN_IR_TEFW | MCAN_IR_TEFF | MCAN_IR_TEFL )   //!< All FIFO/Buffer/TEF related flags

//! Interrupt Events, can be OR'ed.
typedef enum
{
  MCAN_INT_NO_EVENT                      = 0x00000000,   //!< No interrupt events
  // Receive FIFO interrupts
  MCAN_INT_RX0_FIFO_NEW_MESSAGE_EVENT    = MCAN_IR_RF0N, //!< Receive FIFO 0 New Message event
  MCAN_INT_RX0_WATERMARK_REACHED_EVENT   = MCAN_IR_RF0W, //!< Receive FIFO 0 Watermark Reached event
  MCAN_INT_RX0_FULL_EVENT                = MCAN_IR_RF0F, //!< Receive FIFO 0 Full event
  MCAN_INT_RX0_MESSAGE_LOST_EVENT        = MCAN_IR_RF0L, //!< Receive FIFO 0 Message Lost event
  MCAN_INT_RX1_FIFO_NEW_MESSAGE_EVENT    = MCAN_IR_RF1N, //!< Receive FIFO 1 New Message event
  MCAN_INT_RX1_WATERMARK_REACHED_EVENT   = MCAN_IR_RF1W, //!< Receive FIFO 1 Watermark Reached event
  MCAN_INT_RX1_FULL_EVENT                = MCAN_IR_RF1F, //!< Receive FIFO 1 Full event
  MCAN_INT_RX1_MESSAGE_LOST_EVENT        = MCAN_IR_RF1L, //!< Receive FIFO 1 Message Lost event
  MCAN_INT_MSG_TO_DEDICATED_BUFFER_EVENT = MCAN_IR_DRX,  //!< Message stored to Dedicated Receive Buffer event
  // Transmit interrupts
  MCAN_INT_HIGH_PRIORITY_MESSAGE_EVENT   = MCAN_IR_HPM,  //!< High Priority Message event
  MCAN_INT_TX_COMPLETE_EVENT             = MCAN_IR_TC,   //!< Transmission Completed event
  MCAN_INT_TX_CANCEL_COMPLETE_EVENT      = MCAN_IR_TCF,  //!< Transmission Cancellation Finished event
  MCAN_INT_TX_FIFO_EMPTY_EVENT           = MCAN_IR_TFE,  //!< Tx FIFO Empty event
  MCAN_INT_TX_FIFO_NEW_ENTRY_EVENT       = MCAN_IR_TEFN, //!< Tx Event FIFO New Entry event
  MCAN_INT_TX_FIFO_WATERMARK_REACH_EVENT = MCAN_IR_TEFW, //!< Tx Event FIFO Watermark Reached event
  MCAN_INT_TX_FIFO_FULL_EVENT            = MCAN_IR_TEFF, //!< Tx Event FIFO Full event
  MCAN_INT_TX_FIFO_ELEMENT_LOST_EVENT    = MCAN_IR_TEFL, //!< Tx Event FIFO Element Lost event
  // System interrupts
  MCAN_INT_TIMESTAMP_WRAPAROUND_EVENT    = MCAN_IR_TSW,  //!< Timestamp Wraparound event
  MCAN_INT_MESSAGE_RAM_ACCESS_EVENT      = MCAN_IR_MRAF, //!< Message RAM Access Failure event
  MCAN_INT_TIMEOUT_OCCURED_EVENT         = MCAN_IR_TOO,  //!< Timeout Occurred event
  MCAN_INT_ERROR_LOGGING_OVERFLOW_EVENT  = MCAN_IR_ELO,  //!< Error Logging Overflow event
  MCAN_INT_ERROR_PASSIVE_EVENT           = MCAN_IR_EP,   //!< Error Passive event
  MCAN_INT_WARNING_STATUS_EVENT          = MCAN_IR_EW,   //!< Warning Status event
  MCAN_INT_BUS_OFF_STATUS_EVENT          = MCAN_IR_BO,   //!< Bus_Off Status event
  MCAN_INT_WATCHDOG_EVENT                = MCAN_IR_WDI,  //!< Watchdog Interrupt event
  MCAN_INT_ARBITRATION_ERROR_EVENT       = MCAN_IR_PEA,  //!< Protocol Error in Arbitration Phase event
  MCAN_INT_DATA_PHASE_ERROR_EVENT        = MCAN_IR_PED,  //!< Protocol Error in Data Phase event
  MCAN_INT_ACCESS_RESERVED_ADDRESS_EVENT = MCAN_IR_ARA,  //!< Access to Reserved Address event

  MCAN_INT_ENABLE_ALL_EVENTS             = MCAN_IR_EVENTS_STATUS_FLAGS, //!< Enable all events
  MCAN_INT_EVENTS_STATUS_FLAGS_MASK      = MCAN_IR_EVENTS_STATUS_FLAGS, //!< Events flags mask
  MCAN_INT_FIFO_BUFFER_TEF_FLAGS_MASK    = MCAN_IR_FIFO_BUFF_TEF_FLAGS, //!< FIFO/Buffer/TEF flag mask
} eMCAN_InterruptEvents;

typedef eMCAN_InterruptEvents setMCAN_InterruptEvents; //! Set of Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

//! MCAN Interrupt Enable Register (Read/Write, Offset: 0x54)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_IE_Register
{
  uint32_t IE;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RF0NE: 1; //!<  0    - Receive FIFO 0 New Message Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF0WE: 1; //!<  1    - Receive FIFO 0 Watermark Reached Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF0FE: 1; //!<  2    - Receive FIFO 0 Full Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF0LE: 1; //!<  3    - Receive FIFO 0 Message Lost Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF1NE: 1; //!<  4    - Receive FIFO 1 New Message Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF1WE: 1; //!<  5    - Receive FIFO 1 Watermark Reached Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF1FE: 1; //!<  6    - Receive FIFO 1 Full Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t RF1LE: 1; //!<  7    - Receive FIFO 1 Message Lost Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t HPME : 1; //!<  8    - High Priority Message Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TCE  : 1; //!<  9    - Transmission Completed Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TCFE : 1; //!< 10    - Transmission Cancellation Finished Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TFEE : 1; //!< 11    - Tx FIFO Empty Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TEFNE: 1; //!< 12    - Tx Event FIFO New Entry Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TEFWE: 1; //!< 13    - Tx Event FIFO Watermark Reached Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TEFFE: 1; //!< 14    - Tx Event FIFO Full Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TEFLE: 1; //!< 15    - Tx Event FIFO Element Lost Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TSWE : 1; //!< 16    - Timestamp Wraparound Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t MRAFE: 1; //!< 17    - Message RAM Access Failure Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t TOOE : 1; //!< 18    - Timeout Occurred Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t DRXE : 1; //!< 19    - Message stored to Dedicated Receive Buffer Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t BECE : 1; //!< 20    - Bit Error Corrected Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t BEUE : 1; //!< 21    - Bit Error Uncorrected Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t ELOE : 1; //!< 22    - Error Logging Overflow Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t EPE  : 1; //!< 23    - Error Passive Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t EWE  : 1; //!< 24    - Warning Status Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t BOE  : 1; //!< 25    - Bus_Off Status Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t WDIE : 1; //!< 26    - Watchdog Interrupt Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t PEAE : 1; //!< 27    - Protocol Error in Arbitration Phase Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t PEDE : 1; //!< 28    - Protocol Error in Data Phase Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t ARAE : 1; //!< 29    - Access to Reserved Address Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t      : 2; //!< 30-31
  } Bits;
} MCAN_IE_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_IE_Register, 4);

#define MCAN_IE_RF0N_EN  (0x1u <<  0) //!< Receive FIFO 0 New Message Interrupt Enable
#define MCAN_IE_RF0W_EN  (0x1u <<  1) //!< Receive FIFO 0 Watermark Reached Interrupt Enable
#define MCAN_IE_RF0F_EN  (0x1u <<  2) //!< Receive FIFO 0 Full Interrupt Enable
#define MCAN_IE_RF0L_EN  (0x1u <<  3) //!< Receive FIFO 0 Message Lost Interrupt Enable
#define MCAN_IE_RF1N_EN  (0x1u <<  4) //!< Receive FIFO 1 New Message Interrupt Enable
#define MCAN_IE_RF1W_EN  (0x1u <<  5) //!< Receive FIFO 1 Watermark Reached Interrupt Enable
#define MCAN_IE_RF1F_EN  (0x1u <<  6) //!< Receive FIFO 1 Full Interrupt Enable
#define MCAN_IE_RF1L_EN  (0x1u <<  7) //!< Receive FIFO 1 Message Lost Interrupt Enable
#define MCAN_IE_HPM_EN   (0x1u <<  8) //!< High Priority Message Interrupt Enable
#define MCAN_IE_TC_EN    (0x1u <<  9) //!< Transmission Completed Interrupt Enable
#define MCAN_IE_TCF_EN   (0x1u << 10) //!< Transmission Cancellation Finished Interrupt Enable
#define MCAN_IE_TFE_EN   (0x1u << 11) //!< Tx FIFO Empty Interrupt Enable
#define MCAN_IE_TEFN_EN  (0x1u << 12) //!< Tx Event FIFO New Entry Interrupt Enable
#define MCAN_IE_TEFW_EN  (0x1u << 13) //!< Tx Event FIFO Watermark Reached Interrupt Enable
#define MCAN_IE_TEFF_EN  (0x1u << 14) //!< Tx Event FIFO Full Interrupt Enable
#define MCAN_IE_TEFL_EN  (0x1u << 15) //!< Tx Event FIFO Element Lost Interrupt Enable
#define MCAN_IE_TSW_EN   (0x1u << 16) //!< Timestamp Wraparound Interrupt Enable
#define MCAN_IE_MRAF_EN  (0x1u << 17) //!< Message RAM Access Failure Interrupt Enable
#define MCAN_IE_TOO_EN   (0x1u << 18) //!< Timeout Occurred Interrupt Enable
#define MCAN_IE_DRX_EN   (0x1u << 19) //!< Message stored to Dedicated Receive Buffer Interrupt Enable
#define MCAN_IE_BEC_EN   (0x1u << 20) //!< Bit Error Corrected Interrupt Enable
#define MCAN_IE_BEU_EN   (0x1u << 21) //!< Bit Error Uncorrected Interrupt Enable
#define MCAN_IE_ELO_EN   (0x1u << 22) //!< Error Logging Overflow Interrupt Enable
#define MCAN_IE_EP_EN    (0x1u << 23) //!< Error Passive Interrupt Enable
#define MCAN_IE_EW_EN    (0x1u << 24) //!< Warning Status Interrupt Enable
#define MCAN_IE_BO_EN    (0x1u << 25) //!< Bus_Off Status Interrupt Enable
#define MCAN_IE_WDI_EN   (0x1u << 26) //!< Watchdog Interrupt Interrupt Enable
#define MCAN_IE_PEA_EN   (0x1u << 27) //!< Protocol Error in Arbitration Phase Interrupt Enable
#define MCAN_IE_PED_EN   (0x1u << 28) //!< Protocol Error in Data Phase Interrupt Enable
#define MCAN_IE_ARA_EN   (0x1u << 29) //!< Access to Reserved Address Interrupt Enable

//-----------------------------------------------------------------------------

/*! MCAN Interrupt Line Select Register (Read/Write, Offset: 0x58)
 * The Interrupt Line Select register assigns an interrupt generated by a specific interrupt flag from the Interrupt Register to one of the two module interrupt lines
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ILS_Register
{
  uint32_t ILS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RF0NL: 1; //!<  0    - Receive FIFO 0 New Message Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF0WL: 1; //!<  1    - Receive FIFO 0 Watermark Reached Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF0FL: 1; //!<  2    - Receive FIFO 0 Full Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF0LL: 1; //!<  3    - Receive FIFO 0 Message Lost Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF1NL: 1; //!<  4    - Receive FIFO 1 New Message Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF1WL: 1; //!<  5    - Receive FIFO 1 Watermark Reached Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF1FL: 1; //!<  6    - Receive FIFO 1 Full Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t RF1LL: 1; //!<  7    - Receive FIFO 1 Message Lost Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t HPML : 1; //!<  8    - High Priority Message Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TCL  : 1; //!<  9    - Transmission Completed Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TCFL : 1; //!< 10    - Transmission Cancellation Finished Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TFEL : 1; //!< 11    - Tx FIFO Empty Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TEFNL: 1; //!< 12    - Tx Event FIFO New Entry Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TEFWL: 1; //!< 13    - Tx Event FIFO Watermark Reached Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TEFFL: 1; //!< 14    - Tx Event FIFO Full Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TEFLL: 1; //!< 15    - Tx Event FIFO Element Lost Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TSWL : 1; //!< 16    - Timestamp Wraparound Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t MRAFL: 1; //!< 17    - Message RAM Access Failure Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t TOOL : 1; //!< 18    - Timeout Occurred Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t DRXL : 1; //!< 19    - Message stored to Dedicated Receive Buffer Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t BECL : 1; //!< 20    - Bit Error Corrected Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t BEUL : 1; //!< 21    - Bit Error Uncorrected Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t ELOL : 1; //!< 22    - Error Logging Overflow Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t EPL  : 1; //!< 23    - Error Passive Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t EWL  : 1; //!< 24    - Warning Status Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t BOL  : 1; //!< 25    - Bus_Off Status Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t WDIL : 1; //!< 26    - Watchdog Interrupt Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t PEAL : 1; //!< 27    - Protocol Error in Arbitration Phase Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t PEDL : 1; //!< 28    - Protocol Error in Data Phase Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t ARAL : 1; //!< 29    - Access to Reserved Address Interrupt Line: '1' = Interrupt assigned to interrupt line MCAN_INT1 ; '0' = Interrupt assigned to interrupt line MCAN_INT0
    uint32_t      : 2; //!< 30-31
  } Bits;
} MCAN_ILS_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ILS_Register, 4);

#define MCAN_ILS_RF0N_INT0  (0x0u <<  0) //! Receive FIFO 0 New Message Interrupt Line on INT0
#define MCAN_ILS_RF0N_INT1  (0x1u <<  0) //! Receive FIFO 0 New Message Interrupt Line on INT1
#define MCAN_ILS_RF0W_INT0  (0x0u <<  1) //! Receive FIFO 0 Watermark Reached Interrupt Line on INT0
#define MCAN_ILS_RF0W_INT1  (0x1u <<  1) //! Receive FIFO 0 Watermark Reached Interrupt Line on INT1
#define MCAN_ILS_RF0F_INT0  (0x0u <<  2) //! Receive FIFO 0 Full Interrupt Line on INT0
#define MCAN_ILS_RF0F_INT1  (0x1u <<  2) //! Receive FIFO 0 Full Interrupt Line on INT1
#define MCAN_ILS_RF0L_INT0  (0x0u <<  3) //! Receive FIFO 0 Message Lost Interrupt Line on INT0
#define MCAN_ILS_RF0L_INT1  (0x1u <<  3) //! Receive FIFO 0 Message Lost Interrupt Line on INT1
#define MCAN_ILS_RF1N_INT0  (0x0u <<  4) //! Receive FIFO 1 New Message Interrupt Line on INT0
#define MCAN_ILS_RF1N_INT1  (0x1u <<  4) //! Receive FIFO 1 New Message Interrupt Line on INT1
#define MCAN_ILS_RF1W_INT0  (0x0u <<  5) //! Receive FIFO 1 Watermark Reached Interrupt Line on INT0
#define MCAN_ILS_RF1W_INT1  (0x1u <<  5) //! Receive FIFO 1 Watermark Reached Interrupt Line on INT1
#define MCAN_ILS_RF1F_INT0  (0x0u <<  6) //! Receive FIFO 1 Full Interrupt Line on INT0
#define MCAN_ILS_RF1F_INT1  (0x1u <<  6) //! Receive FIFO 1 Full Interrupt Line on INT1
#define MCAN_ILS_RF1L_INT0  (0x0u <<  7) //! Receive FIFO 1 Message Lost Interrupt Line on INT0
#define MCAN_ILS_RF1L_INT1  (0x1u <<  7) //! Receive FIFO 1 Message Lost Interrupt Line on INT1
#define MCAN_ILS_HPM_INT0   (0x0u <<  8) //! High Priority Message Interrupt Line on INT0
#define MCAN_ILS_HPM_INT1   (0x1u <<  8) //! High Priority Message Interrupt Line on INT1
#define MCAN_ILS_TC_INT0    (0x0u <<  9) //! Transmission Completed Interrupt Line on INT0
#define MCAN_ILS_TC_INT1    (0x1u <<  9) //! Transmission Completed Interrupt Line on INT1
#define MCAN_ILS_TCF_INT0   (0x0u << 10) //! Transmission Cancellation Finished Interrupt Line on INT0
#define MCAN_ILS_TCF_INT1   (0x1u << 10) //! Transmission Cancellation Finished Interrupt Line on INT1
#define MCAN_ILS_TFE_INT0   (0x0u << 11) //! Tx FIFO Empty Interrupt Line on INT0
#define MCAN_ILS_TFE_INT1   (0x1u << 11) //! Tx FIFO Empty Interrupt Line on INT1
#define MCAN_ILS_TEFN_INT0  (0x0u << 12) //! Tx Event FIFO New Entry Interrupt Line on INT0
#define MCAN_ILS_TEFN_INT1  (0x1u << 12) //! Tx Event FIFO New Entry Interrupt Line on INT1
#define MCAN_ILS_TEFW_INT0  (0x0u << 13) //! Tx Event FIFO Watermark Reached Interrupt Line on INT0
#define MCAN_ILS_TEFW_INT1  (0x1u << 13) //! Tx Event FIFO Watermark Reached Interrupt Line on INT1
#define MCAN_ILS_TEFF_INT0  (0x0u << 14) //! Tx Event FIFO Full Interrupt Line on INT0
#define MCAN_ILS_TEFF_INT1  (0x1u << 14) //! Tx Event FIFO Full Interrupt Line on INT1
#define MCAN_ILS_TEFL_INT0  (0x0u << 15) //! Tx Event FIFO Element Lost Interrupt Line on INT0
#define MCAN_ILS_TEFL_INT1  (0x1u << 15) //! Tx Event FIFO Element Lost Interrupt Line on INT1
#define MCAN_ILS_TSW_INT0   (0x0u << 16) //! Timestamp Wraparound Interrupt Line on INT0
#define MCAN_ILS_TSW_INT1   (0x1u << 16) //! Timestamp Wraparound Interrupt Line on INT1
#define MCAN_ILS_MRAF_INT0  (0x0u << 17) //! Message RAM Access Failure Interrupt Line on INT0
#define MCAN_ILS_MRAF_INT1  (0x1u << 17) //! Message RAM Access Failure Interrupt Line on INT1
#define MCAN_ILS_TOO_INT0   (0x0u << 18) //! Timeout Occurred Interrupt Line on INT0
#define MCAN_ILS_TOO_INT1   (0x1u << 18) //! Timeout Occurred Interrupt Line on INT1
#define MCAN_ILS_DRX_INT0   (0x0u << 19) //! Message stored to Dedicated Receive Buffer Interrupt Line on INT0
#define MCAN_ILS_DRX_INT1   (0x1u << 19) //! Message stored to Dedicated Receive Buffer Interrupt Line on INT1
#define MCAN_ILS_BEC_INT0   (0x0u << 20) //! Bit Error Corrected Interrupt Line on INT0
#define MCAN_ILS_BEC_INT1   (0x1u << 20) //! Bit Error Corrected Interrupt Line on INT1
#define MCAN_ILS_BEU_INT0   (0x0u << 21) //! Bit Error Uncorrected Interrupt Line on INT0
#define MCAN_ILS_BEU_INT1   (0x1u << 21) //! Bit Error Uncorrected Interrupt Line on INT1
#define MCAN_ILS_ELO_INT0   (0x0u << 22) //! Error Logging Overflow Interrupt Line on INT0
#define MCAN_ILS_ELO_INT1   (0x1u << 22) //! Error Logging Overflow Interrupt Line on INT1
#define MCAN_ILS_EP_INT0    (0x0u << 23) //! Error Passive Interrupt Line on INT0
#define MCAN_ILS_EP_INT1    (0x1u << 23) //! Error Passive Interrupt Line on INT1
#define MCAN_ILS_EW_INT0    (0x0u << 24) //! Warning Status Interrupt Line on INT0
#define MCAN_ILS_EW_INT1    (0x1u << 24) //! Warning Status Interrupt Line on INT1
#define MCAN_ILS_BO_INT0    (0x0u << 25) //! Bus_Off Status Interrupt Line on INT0
#define MCAN_ILS_BO_INT1    (0x1u << 25) //! Bus_Off Status Interrupt Line on INT1
#define MCAN_ILS_WDI_INT0   (0x0u << 26) //! Watchdog Interrupt Interrupt Line on INT0
#define MCAN_ILS_WDI_INT1   (0x1u << 26) //! Watchdog Interrupt Interrupt Line on INT1
#define MCAN_ILS_PEA_INT0   (0x0u << 27) //! Protocol Error in Arbitration Phase Interrupt Line on INT0
#define MCAN_ILS_PEA_INT1   (0x1u << 27) //! Protocol Error in Arbitration Phase Interrupt Line on INT1
#define MCAN_ILS_PED_INT0   (0x0u << 28) //! Protocol Error in Data Phase Interrupt Line on INT0
#define MCAN_ILS_PED_INT1   (0x1u << 28) //! Protocol Error in Data Phase Interrupt Line on INT1
#define MCAN_ILS_ARA_INT0   (0x0u << 29) //! Access to Reserved Address Interrupt Line on INT0
#define MCAN_ILS_ARA_INT1   (0x1u << 29) //! Access to Reserved Address Interrupt Line on INT1

//! Interrupt Line Select, can be OR'ed.
typedef enum
{
  MCAN_INT_ALL_ON_INT0                         = 0x00000000,         //!< Set all interrupts on INT0
  // Receive FIFO interrupts
  MCAN_INT_RX0_FIFO_NEW_MESSAGE_ON_INT0        = MCAN_ILS_RF0N_INT0, //!< Receive FIFO 0 New Message event on line INT0
  MCAN_INT_RX0_FIFO_NEW_MESSAGE_ON_INT1        = MCAN_ILS_RF0N_INT1, //!< Receive FIFO 0 New Message event on line INT1
  MCAN_INT_RX0_WATERMARK_REACHED_ON_INT0       = MCAN_ILS_RF0W_INT0, //!< Receive FIFO 0 Watermark Reached event on line INT0
  MCAN_INT_RX0_WATERMARK_REACHED_ON_INT1       = MCAN_ILS_RF0W_INT1, //!< Receive FIFO 0 Watermark Reached event on line INT1
  MCAN_INT_RX0_FULL_ON_INT0                    = MCAN_ILS_RF0F_INT0, //!< Receive FIFO 0 Full event on line INT0
  MCAN_INT_RX0_FULL_ON_INT1                    = MCAN_ILS_RF0F_INT1, //!< Receive FIFO 0 Full event on line INT1
  MCAN_INT_RX0_MESSAGE_LOST_ON_INT0            = MCAN_ILS_RF0L_INT0, //!< Receive FIFO 0 Message Lost event on line INT0
  MCAN_INT_RX0_MESSAGE_LOST_ON_INT1            = MCAN_ILS_RF0L_INT1, //!< Receive FIFO 0 Message Lost event on line INT1
  MCAN_INT_RX1_FIFO_NEW_MESSAGE_ON_INT0        = MCAN_ILS_RF1N_INT0, //!< Receive FIFO 1 New Message event on line INT0
  MCAN_INT_RX1_FIFO_NEW_MESSAGE_ON_INT1        = MCAN_ILS_RF1N_INT1, //!< Receive FIFO 1 New Message event on line INT1
  MCAN_INT_RX1_WATERMARK_REACHED_ON_INT0       = MCAN_ILS_RF1W_INT0, //!< Receive FIFO 1 Watermark Reached event on line INT0
  MCAN_INT_RX1_WATERMARK_REACHED_ON_INT1       = MCAN_ILS_RF1W_INT1, //!< Receive FIFO 1 Watermark Reached event on line INT1
  MCAN_INT_RX1_FULL_ON_INT0                    = MCAN_ILS_RF1F_INT0, //!< Receive FIFO 1 Full event on line INT0
  MCAN_INT_RX1_FULL_ON_INT1                    = MCAN_ILS_RF1F_INT1, //!< Receive FIFO 1 Full event on line INT1
  MCAN_INT_RX1_MESSAGE_LOST_ON_INT0            = MCAN_ILS_RF1L_INT0, //!< Receive FIFO 1 Message Lost event on line INT0
  MCAN_INT_RX1_MESSAGE_LOST_ON_INT1            = MCAN_ILS_RF1L_INT1, //!< Receive FIFO 1 Message Lost event on line INT1
  MCAN_INT_MSG_TO_DEDICATED_BUFFER_ON_INT0     = MCAN_ILS_DRX_INT0,  //!< Message stored to Dedicated Receive Buffer event on line INT0
  MCAN_INT_MSG_TO_DEDICATED_BUFFER_ON_INT1     = MCAN_ILS_DRX_INT1,  //!< Message stored to Dedicated Receive Buffer event on line INT1
  // Transmit interrupts
  MCAN_INT_HIGH_PRIORITY_MESSAGE_ON_INT0       = MCAN_ILS_HPM_INT0,  //!< High Priority Message event on line INT0
  MCAN_INT_HIGH_PRIORITY_MESSAGE_ON_INT1       = MCAN_ILS_HPM_INT1,  //!< High Priority Message event on line INT1
  MCAN_INT_TX_COMPLETE_ON_INT0                 = MCAN_ILS_TC_INT0,   //!< Transmission Completed event on line INT0
  MCAN_INT_TX_COMPLETE_ON_INT1                 = MCAN_ILS_TC_INT1,   //!< Transmission Completed event on line INT1
  MCAN_INT_TX_CANCEL_COMPLETE_ON_INT0          = MCAN_ILS_TCF_INT0,  //!< Transmission Cancellation Finished event on line INT0
  MCAN_INT_TX_CANCEL_COMPLETE_ON_INT1          = MCAN_ILS_TCF_INT1,  //!< Transmission Cancellation Finished event on line INT1
  MCAN_INT_TX_FIFO_EMPTY_ON_INT0               = MCAN_ILS_TFE_INT0,  //!< Tx FIFO Empty event on line INT0
  MCAN_INT_TX_FIFO_EMPTY_ON_INT1               = MCAN_ILS_TFE_INT1,  //!< Tx FIFO Empty event on line INT1
  MCAN_INT_TX_FIFO_NEW_ENTRY_ON_INT0           = MCAN_ILS_TEFN_INT0, //!< Tx Event FIFO New Entry event on line INT0
  MCAN_INT_TX_FIFO_NEW_ENTRY_ON_INT1           = MCAN_ILS_TEFN_INT1, //!< Tx Event FIFO New Entry event on line INT1
  MCAN_INT_TX_FIFO_WATERMARK_REACH_ON_INT0     = MCAN_ILS_TEFW_INT0, //!< Tx Event FIFO Watermark Reached event on line INT0
  MCAN_INT_TX_FIFO_WATERMARK_REACH_ON_INT1     = MCAN_ILS_TEFW_INT1, //!< Tx Event FIFO Watermark Reached event on line INT1
  MCAN_INT_TX_FIFO_FULL_ON_INT0                = MCAN_ILS_TEFF_INT0, //!< Tx Event FIFO Full event on line INT0
  MCAN_INT_TX_FIFO_FULL_ON_INT1                = MCAN_ILS_TEFF_INT1, //!< Tx Event FIFO Full event on line INT1
  MCAN_INT_TX_FIFO_ELEMENT_LOST_ON_INT0        = MCAN_ILS_TEFL_INT0, //!< Tx Event FIFO Element Lost event on line INT0
  MCAN_INT_TX_FIFO_ELEMENT_LOST_ON_INT1        = MCAN_ILS_TEFL_INT1, //!< Tx Event FIFO Element Lost event on line INT1
  // System interrupts
  MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT0        = MCAN_ILS_TSW_INT0,  //!< Timestamp Wraparound event on line INT0
  MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT1        = MCAN_ILS_TSW_INT1,  //!< Timestamp Wraparound event on line INT1
  MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT0          = MCAN_ILS_MRAF_INT0, //!< Message RAM Access Failure event on line INT0
  MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT1          = MCAN_ILS_MRAF_INT1, //!< Message RAM Access Failure event on line INT1
  MCAN_INT_TIMEOUT_OCCURED_ON_INT0             = MCAN_ILS_TOO_INT0,  //!< Timeout Occurred event on line INT0
  MCAN_INT_TIMEOUT_OCCURED_ON_INT1             = MCAN_ILS_TOO_INT1,  //!< Timeout Occurred event on line INT1
  MCAN_INT_ERROR_CORRECTED_ON_INT0             = MCAN_ILS_BEC_INT0,  //!< Bit Error Corrected Interrupt Line on INT0
  MCAN_INT_ERROR_CORRECTED_ON_INT1             = MCAN_ILS_BEC_INT1,  //!< Bit Error Corrected Interrupt Line on INT1
  MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT0 = MCAN_ILS_BEU_INT0,  //!< Bit Error Uncorrected Interrupt Line on INT0
  MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT1 = MCAN_ILS_BEU_INT1,  //!< Bit Error Uncorrected Interrupt Line on INT1
  MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT0      = MCAN_ILS_ELO_INT0,  //!< Error Logging Overflow event on line INT0
  MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT1      = MCAN_ILS_ELO_INT1,  //!< Error Logging Overflow event on line INT1
  MCAN_INT_ERROR_PASSIVE_ON_INT0               = MCAN_ILS_EP_INT0,   //!< Error Passive event on line INT0
  MCAN_INT_ERROR_PASSIVE_ON_INT1               = MCAN_ILS_EP_INT1,   //!< Error Passive event on line INT1
  MCAN_INT_WARNING_STATUS_ON_INT0              = MCAN_ILS_EW_INT0,   //!< Warning Status event on line INT0
  MCAN_INT_WARNING_STATUS_ON_INT1              = MCAN_ILS_EW_INT1,   //!< Warning Status event on line INT1
  MCAN_INT_BUS_OFF_STATUS_ON_INT0              = MCAN_ILS_BO_INT0,   //!< Bus_Off Status event on line INT0
  MCAN_INT_BUS_OFF_STATUS_ON_INT1              = MCAN_ILS_BO_INT1,   //!< Bus_Off Status event on line INT1
  MCAN_INT_WATCHDOG_ON_INT0                    = MCAN_ILS_WDI_INT0,  //!< Watchdog Interrupt event on line INT0
  MCAN_INT_WATCHDOG_ON_INT1                    = MCAN_ILS_WDI_INT1,  //!< Watchdog Interrupt event on line INT1
  MCAN_INT_ARBITRATION_ERROR_ON_INT0           = MCAN_ILS_PEA_INT0,  //!< Protocol Error in Arbitration Phase event on line INT0
  MCAN_INT_ARBITRATION_ERROR_ON_INT1           = MCAN_ILS_PEA_INT1,  //!< Protocol Error in Arbitration Phase event on line INT1
  MCAN_INT_DATA_PHASE_ERROR_ON_INT0            = MCAN_ILS_PED_INT0,  //!< Protocol Error in Data Phase event on line INT0
  MCAN_INT_DATA_PHASE_ERROR_ON_INT1            = MCAN_ILS_PED_INT1,  //!< Protocol Error in Data Phase event on line INT1
  MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT0     = MCAN_ILS_ARA_INT0,  //!< Access to Reserved Address event on line INT0
  MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT1     = MCAN_ILS_ARA_INT1,  //!< Access to Reserved Address event on line INT1
} eMCAN_IntLineSelect;

typedef eMCAN_IntLineSelect setMCAN_IntLineSelect; //! Set of Interrupt Line Select (can be OR'ed)

//-----------------------------------------------------------------------------

/*! MCAN Interrupt Line Enable (Read/Write, Offset: 0x5C)
 * Each of the two interrupt lines to the processor can be enabled / disabled separately by programming bits EINT0 and EINT1
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_ILE_Register
{
  uint32_t ILE;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t EINT0:  1; //!< 0 - Enable Interrupt Line 0: '1' = Interrupt line MCAN_INT0 enabled ; '0' = Interrupt line MCAN_INT0 disabled
    uint32_t EINT1:  1; //!< 1 - Enable Interrupt Line 1: '1' = Interrupt line MCAN_INT1 enabled ; '0' = Interrupt line MCAN_INT1 disabled
    uint32_t      : 30; //!< 2-31
  } Bits;
} MCAN_ILE_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_ILE_Register, 4);

#define MCAN_ILE_EINT0_EN   (0x1u << 0) //! Interrupt line MCAN_INT0 enabled
#define MCAN_ILE_EINT0_DIS  (0x0u << 0) //! Interrupt line MCAN_INT0 disabled
#define MCAN_ILE_EINT1_EN   (0x1u << 1) //! Interrupt line MCAN_INT1 enabled
#define MCAN_ILE_EINT1_DIS  (0x0u << 1) //! Interrupt line MCAN_INT1 disabled

//-----------------------------------------------------------------------------

/*! MCAN Global Filter Configuration (Read/Write, Offset: 0x80)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * Global settings for Message ID filtering. The Global Filter Configuration controls the filter path for standard and extended messages
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_GFC_Register
{
  uint32_t GFC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t RRFE:  1; //!< 0    - Reject Remote Frames Extended: '1' = Reject all remote frames with 29-bit extended IDs ; '0' = Filter remote frames with 29-bit extended IDs
    uint32_t RRFS:  1; //!< 1    - Reject Remote Frames Standard: '1' = Reject all remote frames with 11-bit standard IDs ; '0' = Filter remote frames with 11-bit standard IDs
    uint32_t ANFE:  2; //!< 2- 3 - Accept Non-matching Frames Extended. Defines how received messages with 29-bit IDs that do not match any element of the filter list are treated
    uint32_t ANFS:  2; //!< 4- 5 - Accept Non-matching Frames Standard. Defines how received messages with 11-bit IDs that do not match any element of the filter list are treated
    uint32_t     : 26; //!< 6-31
  } Bits;
} MCAN_GFC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_GFC_Register, 4);

#define MCAN_GFC_REJECT_REMOTE_FRAMES_EXTENDED_ID  (0x1u << 0) //! Reject all remote frames with 29-bit extended IDs
#define MCAN_GFC_FILTER_REMOTE_FRAMES_EXTENDED_ID  (0x0u << 0) //! Filter remote frames with 29-bit extended IDs
#define MCAN_GFC_REJECT_REMOTE_FRAMES_STANDARD_ID  (0x1u << 1) //! Reject all remote frames with 11-bit standard IDs
#define MCAN_GFC_FILTER_REMOTE_FRAMES_STANDARD_ID  (0x0u << 1) //! Filter remote frames with 11-bit standard IDs

//! Accept Non-matching Frames enumerator
typedef enum
{
  MCAN_ACCEPT_IN_RX_FIFO_0  = 0b00, //!< Accept Non-matching Frames Standard/Extended in Rx FIFO 0
  MCAN_ACCEPT_IN_RX_FIFO_1  = 0b01, //!< Accept Non-matching Frames Standard/Extended in Rx FIFO 1
  MCAN_REJECT_NON_MATCHING  = 0b10, //!< Reject Non-matching Frames Standard/Extended
  MCAN_REJECT_NON_MATCHING_ = 0b11, //!< Reject Non-matching Frames Standard/Extended
} eMCAN_AcceptNonMatching;

#define MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Pos         2
#define MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Mask        (0x3u << MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Pos)
#define MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_SET(value)  (((uint32_t)(value) << MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Pos) & MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Mask) //!< Set Accept Non-matching Frames Extended
#define MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_GET(value)  (((uint32_t)(value) & MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Mask) >> MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_Pos) //!< Get Accept Non-matching Frames Extended
#define MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Pos         4
#define MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Mask        (0x3u << MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Pos)
#define MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_SET(value)  (((uint32_t)(value) << MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Pos) & MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Mask) //!< Set Accept Non-matching Frames Standard
#define MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_GET(value)  (((uint32_t)(value) & MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Mask) >> MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_Pos) //!< Get Accept Non-matching Frames Standard

//-----------------------------------------------------------------------------

/*! MCAN Standard ID Filter Configuration (Read/Write, Offset: 0x84)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * Settings for 11-bit standard Message ID filtering. The Standard ID Filter Configuration controls the filter path for standard messages
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_SIDFC_Register
{
  uint32_t SIDFC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t      :  2; //!<  0- 1
    uint32_t FLSSA: 14; //!<  2-15 - Filter List Standard Start Address. Start address of standard Message ID filter list. Write FLSSA with the bits [15:2] of the 32-bit address
    uint32_t LSS  :  8; //!< 16-23 - List Size Standard: '0' = No standard Message ID filter ; '1-128' = Number of standard Message ID filter elements ; '>128' = Values greater than 128 are interpreted as 128
    uint32_t      :  8; //!< 24-31
  } Bits;
} MCAN_SIDFC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_SIDFC_Register, 4);

#define MCAN_SIDFC_FILTER_LIST_SA_Pos         0
#define MCAN_SIDFC_FILTER_LIST_SA_Mask        (0xFFFCu << MCAN_SIDFC_FILTER_LIST_SA_Pos)
#define MCAN_SIDFC_FILTER_LIST_SA_SET(value)  (((uint32_t)(value) << MCAN_SIDFC_FILTER_LIST_SA_Pos) & MCAN_SIDFC_FILTER_LIST_SA_Mask) //!< Set Filter List Standard Start Address
#define MCAN_SIDFC_FILTER_LIST_SA_GET(value)  (((uint32_t)(value) & MCAN_SIDFC_FILTER_LIST_SA_Mask) >> MCAN_SIDFC_FILTER_LIST_SA_Pos) //!< Get Filter List Standard Start Address
#define MCAN_SIDFC_LIST_SIZE_Pos              16
#define MCAN_SIDFC_LIST_SIZE_Mask             (0xFFu << MCAN_SIDFC_LIST_SIZE_Pos)
#define MCAN_SIDFC_LIST_SIZE_SET(value)       (((uint32_t)(value) << MCAN_SIDFC_LIST_SIZE_Pos) & MCAN_SIDFC_LIST_SIZE_Mask) //!< Set List Size Standard
#define MCAN_SIDFC_LIST_SIZE_GET(value)       (((uint32_t)(value) & MCAN_SIDFC_LIST_SIZE_Mask) >> MCAN_SIDFC_LIST_SIZE_Pos) //!< Get List Size Standard

//-----------------------------------------------------------------------------

/*! MCAN Extended ID Filter Configuration (Read/Write, Offset: 0x88)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * Settings for 29-bit extended Message ID filtering. The Extended ID Filter Configuration controls the filter path for standard messages
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_XIDFC_Register
{
  uint32_t XIDFC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t      :  2; //!<  0- 1
    uint32_t FLESA: 14; //!<  2-15 - Filter List Extended Start Address. Start address of extended Message ID filter list. Write FLESA with the bits [15:2] of the 32-bit address
    uint32_t LSE  :  8; //!< 16-23 - List Size Extended: '0' = No extended Message ID filter ; '1-64' = Number of extended Message ID filter elements ; '>64' = Values greater than 64 are interpreted as 64
    uint32_t      :  8; //!< 24-31
  } Bits;
} MCAN_XIDFC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_XIDFC_Register, 4);

#define MCAN_XIDFC_FILTER_LIST_SA_Pos         0
#define MCAN_XIDFC_FILTER_LIST_SA_Mask        (0xFFFCu << MCAN_XIDFC_FILTER_LIST_SA_Pos)
#define MCAN_XIDFC_FILTER_LIST_SA_SET(value)  ((uint32_t)(value) & MCAN_XIDFC_FILTER_LIST_SA_Mask) //!< Set Filter List Extended Start Address
#define MCAN_XIDFC_FILTER_LIST_SA_GET(value)  ((uint32_t)(value) & MCAN_XIDFC_FILTER_LIST_SA_Mask) //!< Get Filter List Extended Start Address
#define MCAN_XIDFC_LIST_SIZE_Pos              16
#define MCAN_XIDFC_LIST_SIZE_Mask             (0xFFu << MCAN_XIDFC_LIST_SIZE_Pos)
#define MCAN_XIDFC_LIST_SIZE_SET(value)       (((uint32_t)(value) << MCAN_XIDFC_LIST_SIZE_Pos) & MCAN_XIDFC_LIST_SIZE_Mask) //!< Set List Size Extended
#define MCAN_XIDFC_LIST_SIZE_GET(value)       (((uint32_t)(value) & MCAN_XIDFC_LIST_SIZE_Mask) >> MCAN_XIDFC_LIST_SIZE_Pos) //!< Get List Size Extended

//-----------------------------------------------------------------------------

/*! MCAN Extended ID AND Mask (Read/Write, Offset: 0x90)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_XIDAM_Register
{
  uint32_t XIDAM;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t EIDM: 29; //!<  0-28 - Extended ID Mask. For acceptance filtering of extended frames the Extended ID AND Mask is ANDed with the Message ID of a received frame. Intended for masking of 29-bit IDs in SAE J1939. With the reset value of all bits set to one the mask is not active
    uint32_t     :  3; //!< 29-31
  } Bits;
} MCAN_XIDAM_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_XIDAM_Register, 4);

#define MCAN_XIDAM_EXTENDED_ID_AND_MASK_Pos         0
#define MCAN_XIDAM_EXTENDED_ID_AND_MASK_Mask        (0x1FFFFFFFu << MCAN_XIDAM_EXTENDED_ID_AND_MASK_Pos)
#define MCAN_XIDAM_EXTENDED_ID_AND_MASK_SET(value)  (((uint32_t)(value) << MCAN_XIDAM_EXTENDED_ID_AND_MASK_Pos) & MCAN_XIDAM_EXTENDED_ID_AND_MASK_Mask) //!< Set Extended ID Mask
#define MCAN_XIDAM_EXTENDED_ID_AND_MASK_GET(value)  (((uint32_t)(value) & MCAN_XIDAM_EXTENDED_ID_AND_MASK_Mask) >> MCAN_XIDAM_EXTENDED_ID_AND_MASK_Pos) //!< Get Extended ID Mask

//-----------------------------------------------------------------------------

/*! MCAN High Priority Message Status (Read-only, Offset: 0x94)
 * @note This register is updated every time a Message ID filter element configured to generate a priority event matches. This can be used to monitor the status of incoming high priority messages and to enable fast access to these messages
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_HPMS_Register
{
  uint32_t HPMS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t BIDX: 6; //!<  0- 5 - Buffer Index. Index of Receive FIFO element to which the message was stored. Only valid when MSI[1] = '1'
    uint32_t MSI : 2; //!<  6- 7 - Message Storage Indicator
    uint32_t FIDX: 7; //!<  8-14 - Filter Index. Index of matching filter element. Range is 0 to MCAN_SIDFC.LSS-1 resp. MCAN_XIDFC.LSE-1
    uint32_t FLST: 1; //!< 15    - Filter List. Indicates the filter list of the matching filter element: '1' = Extended filter list ; '0' = Standard filter list
    uint32_t     : 3; //!< 16-31
  } Bits;
} MCAN_HPMS_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_HPMS_Register, 4);

#define MCAN_HPMS_BUFFER_INDEX_Pos         0
#define MCAN_HPMS_BUFFER_INDEX_Mask        (0x3Fu << MCAN_HPMS_BUFFER_INDEX_Pos)
#define MCAN_HPMS_BUFFER_INDEX_GET(value)  (((uint32_t)(value) & MCAN_HPMS_BUFFER_INDEX_Mask) >> MCAN_HPMS_BUFFER_INDEX_Pos) //!< Get Extended ID Mask

//! Message Storage Indicator enumerator
typedef enum
{
  MCAN_NO_FIFO_SELECTED   = 0b00, //!< No FIFO selected
  MCAN_FIFO_MESSAGE_LOST  = 0b01, //!< FIFO message lost
  MCAN_STORED_IN_FIFO_0   = 0b10, //!< Message stored in FIFO 0
  MCAN_STORED_IN_FIFO_1   = 0b11, //!< Message stored in FIFO 1
} eMCAN_MessageStorageIndicator;

#define MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_Pos         6
#define MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_Mask        (0x3u << MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_Pos)
#define MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_GET(value)  (((uint32_t)(value) & MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_Mask) >> MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_Pos) //!< Get Message Storage Indicator
#define MCAN_HPMS_FILTER_INDEX_Pos                      8
#define MCAN_HPMS_FILTER_INDEX_Mask                     (0x7Fu << MCAN_HPMS_FILTER_INDEX_Pos)
#define MCAN_HPMS_FILTER_INDEX_GET(value)               (((uint32_t)(value) & MCAN_HPMS_FILTER_INDEX_Mask) >> MCAN_HPMS_FILTER_INDEX_Pos) //!< Get Filter Index
#define MCAN_HPMS_EXTENDED_FILTER_LIST                  (0x1u << 15) //! Extended filter list
#define MCAN_HPMS_STANDARD_FILTER_LIST                  (0x0u << 15) //! Standard filter list

//-----------------------------------------------------------------------------

/*! MCAN New Data 1 (Read/Write, Offset: 0x98)
 * The register holds the New Data flags of Receive Buffers 0 to 31. The flags are set when the respective Receive Buffer has been updated from a received frame.
 * The flags remain set until the processor clears them. A flag is cleared by writing a '1' to the corresponding bit position. Writing a '0' has no effect. A hard reset will clear the register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_NDAT1_Register
{
  uint32_t NDAT1;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ND0 : 1; //!<  0 - New Data in Receive Buffer 0
    uint32_t ND1 : 1; //!<  1 - New Data in Receive Buffer 1
    uint32_t ND2 : 1; //!<  2 - New Data in Receive Buffer 2
    uint32_t ND3 : 1; //!<  3 - New Data in Receive Buffer 3
    uint32_t ND4 : 1; //!<  4 - New Data in Receive Buffer 4
    uint32_t ND5 : 1; //!<  5 - New Data in Receive Buffer 5
    uint32_t ND6 : 1; //!<  6 - New Data in Receive Buffer 6
    uint32_t ND7 : 1; //!<  7 - New Data in Receive Buffer 7
    uint32_t ND8 : 1; //!<  8 - New Data in Receive Buffer 8
    uint32_t ND9 : 1; //!<  9 - New Data in Receive Buffer 9
    uint32_t ND10: 1; //!< 10 - New Data in Receive Buffer 10
    uint32_t ND11: 1; //!< 11 - New Data in Receive Buffer 11
    uint32_t ND12: 1; //!< 12 - New Data in Receive Buffer 12
    uint32_t ND13: 1; //!< 13 - New Data in Receive Buffer 13
    uint32_t ND14: 1; //!< 14 - New Data in Receive Buffer 14
    uint32_t ND15: 1; //!< 15 - New Data in Receive Buffer 15
    uint32_t ND16: 1; //!< 16 - New Data in Receive Buffer 16
    uint32_t ND17: 1; //!< 17 - New Data in Receive Buffer 17
    uint32_t ND18: 1; //!< 18 - New Data in Receive Buffer 18
    uint32_t ND19: 1; //!< 19 - New Data in Receive Buffer 19
    uint32_t ND20: 1; //!< 20 - New Data in Receive Buffer 20
    uint32_t ND21: 1; //!< 21 - New Data in Receive Buffer 21
    uint32_t ND22: 1; //!< 22 - New Data in Receive Buffer 22
    uint32_t ND23: 1; //!< 23 - New Data in Receive Buffer 23
    uint32_t ND24: 1; //!< 24 - New Data in Receive Buffer 24
    uint32_t ND25: 1; //!< 25 - New Data in Receive Buffer 25
    uint32_t ND26: 1; //!< 26 - New Data in Receive Buffer 26
    uint32_t ND27: 1; //!< 27 - New Data in Receive Buffer 27
    uint32_t ND28: 1; //!< 28 - New Data in Receive Buffer 28
    uint32_t ND29: 1; //!< 29 - New Data in Receive Buffer 29
    uint32_t ND30: 1; //!< 30 - New Data in Receive Buffer 30
    uint32_t ND31: 1; //!< 31 - New Data in Receive Buffer 31
  } Bits;
} MCAN_NDAT1_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_NDAT1_Register, 4);

#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_0   (0x1u <<  0) //!< New Data in Receive Buffer 0 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_1   (0x1u <<  1) //!< New Data in Receive Buffer 1 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_2   (0x1u <<  2) //!< New Data in Receive Buffer 2 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_3   (0x1u <<  3) //!< New Data in Receive Buffer 3 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_4   (0x1u <<  4) //!< New Data in Receive Buffer 4 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_5   (0x1u <<  5) //!< New Data in Receive Buffer 5 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_6   (0x1u <<  6) //!< New Data in Receive Buffer 6 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_7   (0x1u <<  7) //!< New Data in Receive Buffer 7 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_8   (0x1u <<  8) //!< New Data in Receive Buffer 8 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_9   (0x1u <<  9) //!< New Data in Receive Buffer 9 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_10  (0x1u << 10) //!< New Data in Receive Buffer 10 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_11  (0x1u << 11) //!< New Data in Receive Buffer 11 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_12  (0x1u << 12) //!< New Data in Receive Buffer 12 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_13  (0x1u << 13) //!< New Data in Receive Buffer 13 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_14  (0x1u << 14) //!< New Data in Receive Buffer 14 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_15  (0x1u << 15) //!< New Data in Receive Buffer 15 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_16  (0x1u << 16) //!< New Data in Receive Buffer 16 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_17  (0x1u << 17) //!< New Data in Receive Buffer 17 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_18  (0x1u << 18) //!< New Data in Receive Buffer 18 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_19  (0x1u << 19) //!< New Data in Receive Buffer 19 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_20  (0x1u << 20) //!< New Data in Receive Buffer 20 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_21  (0x1u << 21) //!< New Data in Receive Buffer 21 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_22  (0x1u << 22) //!< New Data in Receive Buffer 22 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_23  (0x1u << 23) //!< New Data in Receive Buffer 23 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_24  (0x1u << 24) //!< New Data in Receive Buffer 24 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_25  (0x1u << 25) //!< New Data in Receive Buffer 25 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_26  (0x1u << 26) //!< New Data in Receive Buffer 26 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_27  (0x1u << 27) //!< New Data in Receive Buffer 27 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_28  (0x1u << 28) //!< New Data in Receive Buffer 28 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_29  (0x1u << 29) //!< New Data in Receive Buffer 29 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_30  (0x1u << 30) //!< New Data in Receive Buffer 30 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_31  (0x1u << 31) //!< New Data in Receive Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN New Data 2 (Read/Write, Offset: 0x9C)
 * The register holds the New Data flags of Receive Buffers 32 to 63. The flags are set when the respective Receive Buffer has been updated from a received frame.
 * The flags remain set until the processor clears them. A flag is cleared by writing a '1' to the corresponding bit position. Writing a '0' has no effect. A hard reset will clear the register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_NDAT2_Register
{
  uint32_t NDAT2;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t ND32: 1; //!<  0 - New Data in Receive Buffer 32
    uint32_t ND33: 1; //!<  1 - New Data in Receive Buffer 33
    uint32_t ND34: 1; //!<  2 - New Data in Receive Buffer 34
    uint32_t ND35: 1; //!<  3 - New Data in Receive Buffer 35
    uint32_t ND36: 1; //!<  4 - New Data in Receive Buffer 36
    uint32_t ND37: 1; //!<  5 - New Data in Receive Buffer 37
    uint32_t ND38: 1; //!<  6 - New Data in Receive Buffer 38
    uint32_t ND39: 1; //!<  7 - New Data in Receive Buffer 39
    uint32_t ND40: 1; //!<  8 - New Data in Receive Buffer 40
    uint32_t ND41: 1; //!<  9 - New Data in Receive Buffer 41
    uint32_t ND42: 1; //!< 10 - New Data in Receive Buffer 42
    uint32_t ND43: 1; //!< 11 - New Data in Receive Buffer 43
    uint32_t ND44: 1; //!< 12 - New Data in Receive Buffer 44
    uint32_t ND45: 1; //!< 13 - New Data in Receive Buffer 45
    uint32_t ND46: 1; //!< 14 - New Data in Receive Buffer 46
    uint32_t ND47: 1; //!< 15 - New Data in Receive Buffer 47
    uint32_t ND48: 1; //!< 16 - New Data in Receive Buffer 48
    uint32_t ND49: 1; //!< 17 - New Data in Receive Buffer 49
    uint32_t ND50: 1; //!< 18 - New Data in Receive Buffer 50
    uint32_t ND51: 1; //!< 19 - New Data in Receive Buffer 51
    uint32_t ND52: 1; //!< 20 - New Data in Receive Buffer 52
    uint32_t ND53: 1; //!< 21 - New Data in Receive Buffer 53
    uint32_t ND54: 1; //!< 22 - New Data in Receive Buffer 54
    uint32_t ND55: 1; //!< 23 - New Data in Receive Buffer 55
    uint32_t ND56: 1; //!< 24 - New Data in Receive Buffer 56
    uint32_t ND57: 1; //!< 25 - New Data in Receive Buffer 57
    uint32_t ND58: 1; //!< 26 - New Data in Receive Buffer 58
    uint32_t ND59: 1; //!< 27 - New Data in Receive Buffer 59
    uint32_t ND60: 1; //!< 28 - New Data in Receive Buffer 60
    uint32_t ND61: 1; //!< 29 - New Data in Receive Buffer 61
    uint32_t ND62: 1; //!< 30 - New Data in Receive Buffer 62
    uint32_t ND63: 1; //!< 31 - New Data in Receive Buffer 63
  } Bits;
} MCAN_NDAT2_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_NDAT2_Register, 4);

#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_32  (0x1u <<  0) //!< New Data in Receive Buffer 32 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_33  (0x1u <<  1) //!< New Data in Receive Buffer 33 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_34  (0x1u <<  2) //!< New Data in Receive Buffer 34 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_35  (0x1u <<  3) //!< New Data in Receive Buffer 35 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_36  (0x1u <<  4) //!< New Data in Receive Buffer 36 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_37  (0x1u <<  5) //!< New Data in Receive Buffer 37 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_38  (0x1u <<  6) //!< New Data in Receive Buffer 38 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_39  (0x1u <<  7) //!< New Data in Receive Buffer 39 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_40  (0x1u <<  8) //!< New Data in Receive Buffer 40 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_41  (0x1u <<  9) //!< New Data in Receive Buffer 41 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_42  (0x1u << 10) //!< New Data in Receive Buffer 42 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_43  (0x1u << 11) //!< New Data in Receive Buffer 43 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_44  (0x1u << 12) //!< New Data in Receive Buffer 44 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_45  (0x1u << 13) //!< New Data in Receive Buffer 45 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_46  (0x1u << 14) //!< New Data in Receive Buffer 46 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_47  (0x1u << 15) //!< New Data in Receive Buffer 47 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_48  (0x1u << 16) //!< New Data in Receive Buffer 48 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_49  (0x1u << 17) //!< New Data in Receive Buffer 49 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_50  (0x1u << 18) //!< New Data in Receive Buffer 50 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_51  (0x1u << 19) //!< New Data in Receive Buffer 51 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_52  (0x1u << 20) //!< New Data in Receive Buffer 52 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_53  (0x1u << 21) //!< New Data in Receive Buffer 53 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_54  (0x1u << 22) //!< New Data in Receive Buffer 54 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_55  (0x1u << 23) //!< New Data in Receive Buffer 55 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_56  (0x1u << 24) //!< New Data in Receive Buffer 56 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_57  (0x1u << 25) //!< New Data in Receive Buffer 57 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_58  (0x1u << 26) //!< New Data in Receive Buffer 58 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_59  (0x1u << 27) //!< New Data in Receive Buffer 59 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_60  (0x1u << 28) //!< New Data in Receive Buffer 60 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_61  (0x1u << 29) //!< New Data in Receive Buffer 61 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_62  (0x1u << 30) //!< New Data in Receive Buffer 62 flag
#define MCAN_NEW_DATA_IN_RECEIVE_BUFFER_63  (0x1u << 31) //!< New Data in Receive Buffer 63 flag

//-----------------------------------------------------------------------------

/*! MCAN Receive FIFO 0 Configuration (Read/Write, Offset: 0xA0)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF0C_Register
{
  uint32_t RXF0C;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     :  2; //!<  0- 1
    uint32_t F0SA: 14; //!<  2-15 - Receive FIFO 0 Start Address. Start address of Receive FIFO 0 in Message RAM. Write F0SA with the bits [15:2] of the 32-bit address
    uint32_t F0S :  7; //!< 16-22 - Receive FIFO 0 Size. The Receive FIFO 0 elements are indexed from 0 to F0S-1: '0' = No Receive FIFO 0 ; '1-64' = Number of Receive FIFO 0 elements ; '>64' = Values greater than 64 are interpreted as 64
    uint32_t     :  1; //!< 23
    uint32_t F0WM:  7; //!< 24-30 - Receive FIFO 0 Watermark. '0' = Watermark interrupt disabled ; '1-64' = Level for Receive FIFO 0 watermark interrupt (MCAN_IR.RF0W) ; '>64' = Watermark interrupt disabled
    uint32_t F0OM:  1; //!< 31    - FIFO 0 Operation Mode. FIFO 0 can be operated in Blocking or in Overwrite mode: '1' = FIFO 0 Overwrite mode ; '0' = FIFO 0 Blocking mode
  } Bits;
} MCAN_RXF0C_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF0C_Register, 4);

#define MCAN_RXF0C_RX_FIFO0_SA_Pos                0
#define MCAN_RXF0C_RX_FIFO0_SA_Mask               (0xFFFCu << MCAN_RXF0C_RX_FIFO0_SA_Pos)
#define MCAN_RXF0C_RX_FIFO0_SA_GET(value)         ((uint32_t)(value) & MCAN_RXF0C_RX_FIFO0_SA_Mask) //!< Get Receive FIFO 0 Start Address
#define MCAN_RXF0C_RX_FIFO0_SA_SET(value)         ((uint32_t)(value) & MCAN_RXF0C_RX_FIFO0_SA_Mask) //!< Set Receive FIFO 0 Start Address
#define MCAN_RXF0C_RX_FIFO0_SIZE_Pos              16
#define MCAN_RXF0C_RX_FIFO0_SIZE_Mask             (0x7Fu << MCAN_RXF0C_RX_FIFO0_SIZE_Pos)
#define MCAN_RXF0C_RX_FIFO0_SIZE_GET(value)       (((uint32_t)(value) & MCAN_RXF0C_RX_FIFO0_SIZE_Mask) >> MCAN_RXF0C_RX_FIFO0_SIZE_Pos) //!< Get Receive FIFO 0 Size
#define MCAN_RXF0C_RX_FIFO0_SIZE_SET(value)       (((uint32_t)(value) << MCAN_RXF0C_RX_FIFO0_SIZE_Pos) & MCAN_RXF0C_RX_FIFO0_SIZE_Mask) //!< Set Receive FIFO 0 Size
#define MCAN_RXF0C_RX_FIFO0_WATERMARK_Pos         24
#define MCAN_RXF0C_RX_FIFO0_WATERMARK_Mask        (0x7Fu << MCAN_RXF0C_RX_FIFO0_WATERMARK_Pos)
#define MCAN_RXF0C_RX_FIFO0_WATERMARK_GET(value)  (((uint32_t)(value) & MCAN_RXF0C_RX_FIFO0_WATERMARK_Mask) >> MCAN_RXF0C_RX_FIFO0_WATERMARK_Pos) //!< Get Receive FIFO 0 Watermark
#define MCAN_RXF0C_RX_FIFO0_WATERMARK_SET(value)  (((uint32_t)(value) << MCAN_RXF0C_RX_FIFO0_WATERMARK_Pos) & MCAN_RXF0C_RX_FIFO0_WATERMARK_Mask) //!< Set Receive FIFO 0 Watermark
#define MCAN_RXF0C_RX_FIFO0_OVERWRITE_MODE        (0x1u << 31) //!< FIFO 0 in Overwrite mode
#define MCAN_RXF0C_RX_FIFO0_BLOCKING_MODE         (0x0u << 31) //!< FIFO 0 in Blocking mode

//-----------------------------------------------------------------------------

//! MCAN Receive FIFO 0 Status (Read-only, Offset: 0xA4)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF0S_Register
{
  uint32_t RXF0S;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t F0FL: 7; //!<  0- 6 - Receive FIFO 0 Fill Level. Number of elements stored in Receive FIFO 0, range 0 to 64
    uint32_t     : 1; //!<  7
    uint32_t F0GI: 6; //!<  8-13 - Receive FIFO 0 Get Index. Receive FIFO 0 read index pointer, range 0 to 63
    uint32_t     : 2; //!< 14-15
    uint32_t F0PI: 6; //!< 16-21 - Receive FIFO 0 Put Index. Receive FIFO 0 write index pointer, range 0 to 63
    uint32_t     : 2; //!< 22-23
    uint32_t F0F : 1; //!< 24    - Receive FIFO 0 Full: '1' = Receive FIFO 0 full ; '0' = Receive FIFO 0 not full
    uint32_t RF0L: 1; //!< 25    - Receive FIFO 0 Message Lost. This bit is a copy of interrupt flag MCAN_IR.RF0L. When MCAN_IR.RF0L is reset, this bit is also reset: '1' = Receive FIFO 0 message lost, also set after write attempt to Receive FIFO 0 of size zero ; '0' = No Receive FIFO 0 message lost
    uint32_t     : 6; //!< 26-31
  } Bits;
} MCAN_RXF0S_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF0S_Register, 4);

#define MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_Pos         0
#define MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_Mask        (0x7Fu << MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_Pos)
#define MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_GET(value)  (((uint32_t)(value) & MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_Mask) >> MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_Pos) //!< Get Receive FIFO 0 Fill Level
#define MCAN_RXF0S_RX_FIFO0_GET_INDEX_Pos          8
#define MCAN_RXF0S_RX_FIFO0_GET_INDEX_Mask         (0x3Fu << MCAN_RXF0S_RX_FIFO0_GET_INDEX_Pos)
#define MCAN_RXF0S_RX_FIFO0_GET_INDEX_GET(value)   (((uint32_t)(value) & MCAN_RXF0S_RX_FIFO0_GET_INDEX_Mask) >> MCAN_RXF0S_RX_FIFO0_GET_INDEX_Pos) //!< Get Receive FIFO 0 Get Index
#define MCAN_RXF0S_RX_FIFO0_PUT_INDEX_Pos          16
#define MCAN_RXF0S_RX_FIFO0_PUT_INDEX_Mask         (0x3Fu << MCAN_RXF0S_RX_FIFO0_PUT_INDEX_Pos)
#define MCAN_RXF0S_RX_FIFO0_PUT_INDEX_GET(value)   (((uint32_t)(value) & MCAN_RXF0S_RX_FIFO0_PUT_INDEX_Mask) >> MCAN_RXF0S_RX_FIFO0_PUT_INDEX_Pos) //!< Get Receive FIFO 0 Put Index
#define MCAN_RXF0S_RX_FIFO0_FULL                   (0x1u << 24) //!< Receive FIFO 0 full
#define MCAN_RXF0S_RX_FIFO0_NOT_FULL               (0x0u << 24) //!< Receive FIFO 0 not full
#define MCAN_RXF0S_RX_FIFO0_MESSAGE_LOST           (0x1u << 25) //!< Receive FIFO 0 message lost, also set after write attempt to Receive FIFO 0 of size zero
#define MCAN_RXF0S_RX_FIFO0_NO_MESSAGE_LOST        (0x0u << 25) //!< No Receive FIFO 0 message lost

//-----------------------------------------------------------------------------

/*! MCAN Receive FIFO 0 Acknowledge (Read/Write, Offset: 0xA8)
 * After the processor has read a message or a sequence of messages from Receive FIFO 0 it has to write the buffer index of the last element read from Receive FIFO 0 to F0AI.
 * This will set the Receive FIFO 0 Get Index MCAN_RXF0S.F0GI to F0AI+1 and update the FIFO 0 Fill Level MCAN_RXF0S.F0FL
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF0A_Register
{
  uint32_t RXF0A;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t F0AI:  6; //!< 0- 5 - Receive FIFO 0 Acknowledge Index
    uint32_t     : 26; //!< 6-31
  } Bits;
} MCAN_RXF0A_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF0A_Register, 4);

#define MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Pos         0
#define MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Mask        (0x3Fu << MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Pos)
#define MCAN_RXF0A_RX_FIFO0_ACK_INDEX_GET(value)  (((uint32_t)(value) & MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Mask) >> MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Pos) //!< Get Receive FIFO 0 Acknowledge Index
#define MCAN_RXF0A_RX_FIFO0_ACK_INDEX_SET(value)  (((uint32_t)(value) << MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Pos) & MCAN_RXF0A_RX_FIFO0_ACK_INDEX_Mask) //!< Set Receive FIFO 0 Acknowledge Index

//-----------------------------------------------------------------------------

/*! MCAN Receive Buffer Configuration (Read/Write, Offset: 0xAC)
 * Configures the start address of the Receive Buffers section in the Message RAM (32-bit word address, see Figure 49-12). Also used to reference debug messages A,B,C. Write RBSA with the bits [15:2] of the 32-bit address
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXBC_Register
{
  uint32_t RXBC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     :  2; //!<  0- 1
    uint32_t RBSA: 14; //!<  2-15 - Receive Buffer Start Address. Write RBSA with the bits [15:2] of the 32-bit address
    uint32_t     : 16; //!< 16-31
  } Bits;
} MCAN_RXBC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXBC_Register, 4);

#define MCAN_RXBC_RX_BUFFER_SA_Pos         0
#define MCAN_RXBC_RX_BUFFER_SA_Mask        (0xFFFCu << MCAN_RXBC_RX_BUFFER_SA_Pos)
#define MCAN_RXBC_RX_BUFFER_SA_GET(value)  ((uint32_t)(value) & MCAN_RXBC_RX_BUFFER_SA_Mask) //!< Get Receive Buffer Start Address
#define MCAN_RXBC_RX_BUFFER_SA_SET(value)  ((uint32_t)(value) & MCAN_RXBC_RX_BUFFER_SA_Mask) //!< Set Receive Buffer Start Address

//-----------------------------------------------------------------------------

/*! MCAN Receive FIFO 1 Configuration (Read/Write, Offset: 0xB0)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF1C_Register
{
  uint32_t RXF1C;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     :  2; //!<  0- 1
    uint32_t F1SA: 14; //!<  2-15 - Receive FIFO 1 Start Address. Start address of Receive FIFO 1 in Message RAM. Write F0SA with the bits [15:2] of the 32-bit address
    uint32_t F1S :  7; //!< 16-22 - Receive FIFO 1 Size. The Receive FIFO 0 elements are indexed from 1 to F0S-1: '0' = No Receive FIFO 0 ; '1-64' = Number of Receive FIFO 1 elements ; '>64' = Values greater than 64 are interpreted as 64
    uint32_t     :  1; //!< 23
    uint32_t F1WM:  7; //!< 24-30 - Receive FIFO 1 Watermark. '0' = Watermark interrupt disabled ; '1-64' = Level for Receive FIFO 1 watermark interrupt (MCAN_IR.RF0W) ; '>64' = Watermark interrupt disabled
    uint32_t F1OM:  1; //!< 31    - FIFO 1 Operation Mode. FIFO 1 can be operated in Blocking or in Overwrite mode: '1' = FIFO 1 Overwrite mode ; '0' = FIFO 1 Blocking mode
  } Bits;
} MCAN_RXF1C_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF1C_Register, 4);

#define MCAN_RXF1C_RX_FIFO1_SA_Pos                0
#define MCAN_RXF1C_RX_FIFO1_SA_Mask               (0xFFFCu << MCAN_RXF1C_RX_FIFO1_SA_Pos)
#define MCAN_RXF1C_RX_FIFO1_SA_GET(value)         ((uint32_t)(value) & MCAN_RXF1C_RX_FIFO1_SA_Mask) //!< Get Receive FIFO 1 Start Address
#define MCAN_RXF1C_RX_FIFO1_SA_SET(value)         ((uint32_t)(value) & MCAN_RXF1C_RX_FIFO1_SA_Mask) //!< Set Receive FIFO 1 Start Address
#define MCAN_RXF1C_RX_FIFO1_SIZE_Pos              16
#define MCAN_RXF1C_RX_FIFO1_SIZE_Mask             (0x7Fu << MCAN_RXF1C_RX_FIFO1_SIZE_Pos)
#define MCAN_RXF1C_RX_FIFO1_SIZE_GET(value)       (((uint32_t)(value) & MCAN_RXF1C_RX_FIFO1_SIZE_Mask) >> MCAN_RXF1C_RX_FIFO1_SIZE_Pos) //!< Get Receive FIFO 1 Size
#define MCAN_RXF1C_RX_FIFO1_SIZE_SET(value)       (((uint32_t)(value) << MCAN_RXF1C_RX_FIFO1_SIZE_Pos) & MCAN_RXF1C_RX_FIFO1_SIZE_Mask) //!< Set Receive FIFO 1 Size
#define MCAN_RXF1C_RX_FIFO1_WATERMARK_Pos         24
#define MCAN_RXF1C_RX_FIFO1_WATERMARK_Mask        (0x7Fu << MCAN_RXF1C_RX_FIFO1_WATERMARK_Pos)
#define MCAN_RXF1C_RX_FIFO1_WATERMARK_GET(value)  (((uint32_t)(value) & MCAN_RXF1C_RX_FIFO1_WATERMARK_Mask) >> MCAN_RXF1C_RX_FIFO1_WATERMARK_Pos) //!< Get Receive FIFO 1 Watermark
#define MCAN_RXF1C_RX_FIFO1_WATERMARK_SET(value)  (((uint32_t)(value) << MCAN_RXF1C_RX_FIFO1_WATERMARK_Pos) & MCAN_RXF1C_RX_FIFO1_WATERMARK_Mask) //!< Set Receive FIFO 1 Watermark
#define MCAN_RXF1C_RX_FIFO1_OVERWRITE_MODE        (0x1u << 31) //!< FIFO 1 in Overwrite mode
#define MCAN_RXF1C_RX_FIFO1_BLOCKING_MODE         (0x0u << 31) //!< FIFO 1 in Blocking mode

//-----------------------------------------------------------------------------

//! MCAN Receive FIFO 1 Status (Read-only, Offset: 0xB4)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF1S_Register
{
  uint32_t RXF1S;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t F1FL: 7; //!<  0- 6 - Receive FIFO 1 Fill Level. Number of elements stored in Receive FIFO 1, range 0 to 64
    uint32_t     : 1; //!<  7
    uint32_t F1GI: 6; //!<  8-13 - Receive FIFO 1 Get Index. Receive FIFO 1 read index pointer, range 0 to 63
    uint32_t     : 2; //!< 14-15
    uint32_t F1PI: 6; //!< 16-21 - Receive FIFO 1 Put Index. Receive FIFO 1 write index pointer, range 0 to 63
    uint32_t     : 2; //!< 22-23
    uint32_t F1F : 1; //!< 24    - Receive FIFO 1 Full: '1' = Receive FIFO 1 full ; '0' = Receive FIFO 1 not full
    uint32_t RF1L: 1; //!< 25    - Receive FIFO 1 Message Lost. This bit is a copy of interrupt flag MCAN_IR.RF0L. When MCAN_IR.RF0L is reset, this bit is also reset: '1' = Receive FIFO 1 message lost, also set after write attempt to Receive FIFO 1 of size zero ; '0' = No Receive FIFO 1 message lost
    uint32_t     : 4; //!< 26-29
    uint32_t DMS : 2; //!< 30-31 - Debug Message Status
  } Bits;
} MCAN_RXF1S_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF1S_Register, 4);

#define MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_Pos         0
#define MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_Mask        (0x7Fu << MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_Pos)
#define MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_GET(value)  (((uint32_t)(value) & MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_Mask) >> MCAN_RXF1S_RX_FIFO1_FILL_LEVEL_Pos) //!< Get Receive FIFO 1 Fill Level
#define MCAN_RXF1S_RX_FIFO1_GET_INDEX_Pos          8
#define MCAN_RXF1S_RX_FIFO1_GET_INDEX_Mask         (0x3Fu << MCAN_RXF1S_RX_FIFO1_GET_INDEX_Pos)
#define MCAN_RXF1S_RX_FIFO1_GET_INDEX_GET(value)   (((uint32_t)(value) & MCAN_RXF1S_RX_FIFO1_GET_INDEX_Mask) >> MCAN_RXF1S_RX_FIFO1_GET_INDEX_Pos) //!< Get Receive FIFO 1 Get Index
#define MCAN_RXF1S_RX_FIFO1_PUT_INDEX_Pos          16
#define MCAN_RXF1S_RX_FIFO1_PUT_INDEX_Mask         (0x3Fu << MCAN_RXF1S_RX_FIFO1_PUT_INDEX_Pos)
#define MCAN_RXF1S_RX_FIFO1_PUT_INDEX_GET(value)   (((uint32_t)(value) & MCAN_RXF1S_RX_FIFO1_PUT_INDEX_Mask) >> MCAN_RXF1S_RX_FIFO1_PUT_INDEX_Pos) //!< Get Receive FIFO 1 Put Index
#define MCAN_RXF1S_RX_FIFO1_FULL                   (0x1u << 24) //!< Receive FIFO 1 full
#define MCAN_RXF1S_RX_FIFO1_NOT_FULL               (0x0u << 24) //!< Receive FIFO 1 not full
#define MCAN_RXF1S_RX_FIFO1_MESSAGE_LOST           (0x1u << 25) //!< Receive FIFO 1 message lost, also set after write attempt to Receive FIFO 1 of size zero
#define MCAN_RXF1S_RX_FIFO1_NO_MESSAGE_LOST        (0x0u << 25) //!< No Receive FIFO 1 message lost

//! Debug Message Status enumerator
typedef enum
{
  MCAN_IDLE_STATE                 = 0b00, //!< Idle state, wait for reception of debug messages, DMA request is cleared
  MCAN_DEBUG_MESSAGE_A_RECEIVED   = 0b01, //!< Debug message A received
  MCAN_DEBUG_MESSAGE_AB_RECEIVED  = 0b10, //!< Debug messages A, B received
  MCAN_DEBUG_MESSAGE_ABC_RECEIVED = 0b11, //!< Debug messages A, B, C received, DMA request is set
} eMCAN_DebugMessageStatus;

#define MCAN_RXF1S_DEBUG_MESSAGE_STATUS_Pos         30
#define MCAN_RXF1S_DEBUG_MESSAGE_STATUS_Mask        (0x3u << MCAN_RXF1S_DEBUG_MESSAGE_STATUS_Pos)
#define MCAN_RXF1S_DEBUG_MESSAGE_STATUS_GET(value)  (eMCAN_DebugMessageStatus)(((uint32_t)(value) & MCAN_RXF1S_DEBUG_MESSAGE_STATUS_Mask) >> MCAN_RXF1S_DEBUG_MESSAGE_STATUS_Pos) //!< Get Debug Message Status

//-----------------------------------------------------------------------------

/*! MCAN Receive FIFO 1 Acknowledge (Read/Write, Offset: 0xB8)
 * After the processor has read a message or a sequence of messages from Receive FIFO 1 it has to write the buffer index of the last element read from Receive FIFO 1 to F1AI.
 * This will set the Receive FIFO 1 Get Index MCAN_RXF1S.F1GI to F0AI+1 and update the FIFO 1 Fill Level MCAN_RXF1S.F1FL
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXF1A_Register
{
  uint32_t RXF1A;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t F0AI:  6; //!< 0- 5 - Receive FIFO 1 Acknowledge Index
    uint32_t     : 26; //!< 6-31
  } Bits;
} MCAN_RXF1A_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXF1A_Register, 4);

#define MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Pos         0
#define MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Mask        (0x3Fu << MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Pos)
#define MCAN_RXF1A_RX_FIFO1_ACK_INDEX_GET(value)  (((uint32_t)(value) & MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Mask) >> MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Pos) //!< Get Receive FIFO 1 Acknowledge Index
#define MCAN_RXF1A_RX_FIFO1_ACK_INDEX_SET(value)  (((uint32_t)(value) << MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Pos) & MCAN_RXF1A_RX_FIFO1_ACK_INDEX_Mask) //!< Set Receive FIFO 1 Acknowledge Index

//-----------------------------------------------------------------------------

/*! MCAN Receive Buffer / FIFO Element Size Configuration (Read/Write, Offset: 0xBC)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * Configures the number of data bytes belonging to a Receive Buffer / Receive FIFO element. Data field sizes >8 bytes are intended for CAN FD operation only
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_RXESC_Register
{
  uint32_t RXESC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t F0DS:  3; //!<  0- 2 - Receive FIFO 0 Data Field Size
    uint32_t     :  1; //!<  3
    uint32_t F1DS:  3; //!<  4- 6 - Receive FIFO 1 Data Field Size
    uint32_t     :  1; //!<  7
    uint32_t RBDS:  3; //!<  8-10 - Receive Buffer Data Field Size
    uint32_t     : 21; //!< 11-31
  } Bits;
} MCAN_RXESC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_RXESC_Register, 4);

//! Data Field Size enumerator for MCAN_RXESC.F0DS, MCAN_RXESC.F1DS, MCAN_RXESC.RBDS, and MCAN_TXESC.TBDS
typedef enum
{
  MCAN_8_BYTES  = 0x0, //!< 8-byte data field
  MCAN_12_BYTES = 0x1, //!< 12-byte data field
  MCAN_16_BYTES = 0x2, //!< 16-byte data field
  MCAN_20_BYTES = 0x3, //!< 20-byte data field
  MCAN_24_BYTES = 0x4, //!< 24-byte data field
  MCAN_32_BYTES = 0x5, //!< 32-byte data field
  MCAN_48_BYTES = 0x6, //!< 48-byte data field
  MCAN_64_BYTES = 0x7, //!< 64-byte data field
  MCAN_PAYLOAD_COUNT,  // Keep last
} eMCAN_PayloadSize;

#define MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Pos          0
#define MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Mask         (0x7u << MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Pos)
#define MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_GET(value)   (((uint32_t)(value) & MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Mask) >> MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Pos) //!< Get Receive FIFO 0 Data Field Size
#define MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_SET(value)   (((uint32_t)(value) << MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Pos) & MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_Mask) //!< Set Receive FIFO 0 Data Field Size
#define MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Pos          4
#define MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Mask         (0x7u << MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Pos)
#define MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_GET(value)   (((uint32_t)(value) & MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Mask) >> MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Pos) //!< Get Receive FIFO 1 Data Field Size
#define MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_SET(value)   (((uint32_t)(value) << MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Pos) & MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_Mask) //!< Set Receive FIFO 1 Data Field Size
#define MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Pos         8
#define MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Mask        (0x7u << MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Pos)
#define MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_GET(value)  (((uint32_t)(value) & MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Mask) >> MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Pos) //!< Get Receive Buffer Data Field Size
#define MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_SET(value)  (((uint32_t)(value) << MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Pos) & MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_Mask) //!< Set Receive Buffer Data Field Size

//-----------------------------------------------------------------------------

static const uint8_t MCAN_PAYLOAD_TO_VALUE[MCAN_PAYLOAD_COUNT] = {8, 12, 16, 20, 24, 32, 48, 64};
#define MCAN_PayploadToByte(payload)  ( ((payload >= MCAN_PAYLOAD_COUNT) ? CANFD_DLC_TO_VALUE[(size_t)(dlc) & 0xF] : CAN20_DLC_TO_VALUE[(size_t)(dlc) & 0xF] )

//-----------------------------------------------------------------------------

/*! MCAN Tx Buffer Configuration (Read/Write, Offset: 0xC0)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * The sum of TFQS and NDTB may be not greater than 32. There is no check for erroneous configurations. The Tx Buffers section in the Message RAM starts with the dedicated Tx Buffers
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBC_Register
{
  uint32_t TXBC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     :  2; //!<  0- 1
    uint32_t TBSA: 14; //!<  2-15 - Tx Buffers Start Address. Start address of Tx Buffers section in Message RAM. Write TBSA with the bits [15:2] of the 32-bit address
    uint32_t NDTB:  6; //!< 16-21 - Number of Dedicated Transmit Buffers: '0' = No dedicated Tx Buffers ; '1-32' = Number of dedicated Tx Buffers ; '>32' = Values greater than 32 are interpreted as 32
    uint32_t     :  2; //!< 22-23
    uint32_t TFQS:  6; //!< 24-29 - Transmit FIFO/Queue Size: '0' = No Tx FIFO/Queue ; '1-32' = Number of Tx Buffers used for Tx FIFO/Queue ; '>32' = Values greater than 32 are interpreted as 32
    uint32_t TFQM:  1; //!< 30    - Tx FIFO/Queue Mode: '1' = Tx Queue operation ; '0' = Tx FIFO operation
    uint32_t     :  1; //!< 31
  } Bits;
} MCAN_TXBC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBC_Register, 4);

#define MCAN_TXBC_TX_BUFFERS_SA_Pos              0
#define MCAN_TXBC_TX_BUFFERS_SA_Mask             (0xFFFCu << MCAN_TXBC_TX_BUFFERS_SA_Pos)
#define MCAN_TXBC_TX_BUFFERS_SA_GET(value)       ((uint32_t)(value) & MCAN_TXBC_TX_BUFFERS_SA_Mask) //!< Get Tx Buffers Start Address
#define MCAN_TXBC_TX_BUFFERS_SA_SET(value)       ((uint32_t)(value) & MCAN_TXBC_TX_BUFFERS_SA_Mask) //!< Set Tx Buffers Start Address
#define MCAN_TXBC_TX_BUFFER_SIZE_Pos             16
#define MCAN_TXBC_TX_BUFFER_SIZE_Mask            (0x3Fu << MCAN_TXBC_TX_BUFFER_SIZE_Pos)
#define MCAN_TXBC_TX_BUFFER_SIZE_GET(value)      (((uint32_t)(value) & MCAN_TXBC_TX_BUFFER_SIZE_Mask) >> MCAN_TXBC_TX_BUFFER_SIZE_Pos) //!< Get Number of Dedicated Transmit Buffers
#define MCAN_TXBC_TX_BUFFER_SIZE_SET(value)      (((uint32_t)(value) << MCAN_TXBC_TX_BUFFER_SIZE_Pos) & MCAN_TXBC_TX_BUFFER_SIZE_Mask) //!< Set Number of Dedicated Transmit Buffers
#define MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Pos         24
#define MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Mask        (0x3Fu << MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Pos)
#define MCAN_TXBC_TX_FIFO_QUEUE_SIZE_GET(value)  (((uint32_t)(value) & MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Mask) >> MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Pos) //!< Get Transmit FIFO/Queue Size
#define MCAN_TXBC_TX_FIFO_QUEUE_SIZE_SET(value)  (((uint32_t)(value) << MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Pos) & MCAN_TXBC_TX_FIFO_QUEUE_SIZE_Mask) //!< Set Transmit FIFO/Queue Size
#define MCAN_TXBC_TX_QUEUE_OPERATION             (0x1u << 30) //!< Tx Queue operation
#define MCAN_TXBC_TX_FIFO_OPERATION              (0x0u << 30) //!< Tx FIFO operation
#define MCAN_TXBC_IS_TX_FIFO_OPERATION(value)    ( ( (value) & MCAN_TXBC_TX_QUEUE_OPERATION) == MCAN_TXBC_TX_FIFO_OPERATION) //!< Is Tx FIFO operation?

//-----------------------------------------------------------------------------

/*! MCAN Tx FIFO/Queue Status (Read-only, Offset: 0xC4)
 * The Tx FIFO/Queue status is related to the pending Tx requests listed in register MCAN_TXBRP. Therefore the effect of Add/Cancellation requests may be delayed due to a running Tx scan (MCAN_TXBRP not yet updated)
 * In case of mixed configurations where dedicated Tx Buffers are combined with a Tx FIFO or a Tx Queue, the Put and Get Indices indicate the number of the Tx Buffer starting with the first dedicated Tx Buffers.
 * Example: For a configuration of 12 dedicated Tx Buffers and a Tx FIFO of 20 Buffers a Put Index of 15 points to the fourth buffer of the Tx FIFO
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXFQS_Register
{
  uint32_t TXFQS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TFFL :  6; //!<  0- 5 - Tx FIFO Free Level. Number of consecutive free Tx FIFO elements starting from TFGI, range 0 to 32. Read as zero when Tx Queue operation is configured (MCAN_TXBC.TFQM = '1')
    uint32_t      :  2; //!<  6- 7
    uint32_t TFGI :  5; //!<  8-12 - Tx FIFO Get Index. Tx FIFO read index pointer, range 0 to 31. Read as zero when Tx Queue operation is configured (MCAN_TXBC.TFQM = '1')
    uint32_t      :  3; //!< 13-15
    uint32_t TFQPI:  5; //!< 16-20 - Tx FIFO/Queue Put Index. Tx FIFO/Queue write index pointer, range 0 to 31
    uint32_t TFQF :  1; //!< 21    - Tx FIFO/Queue Full: '1' = Tx FIFO/Queue full ; '0' = Tx FIFO/Queue not full
    uint32_t      : 10; //!< 22-31
  } Bits;
} MCAN_TXFQS_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXFQS_Register, 4);

#define MCAN_TXFQS_TX_FIFO_FREE_LEVEL_Pos         0
#define MCAN_TXFQS_TX_FIFO_FREE_LEVEL_Mask        (0x3Fu << MCAN_TXFQS_TX_FIFO_FREE_LEVEL_Pos)
#define MCAN_TXFQS_TX_FIFO_FREE_LEVEL_GET(value)  (((uint32_t)(value) & MCAN_TXFQS_TX_FIFO_FREE_LEVEL_Mask) >> MCAN_TXFQS_TX_FIFO_FREE_LEVEL_Pos) //!< Get Tx FIFO Free Level
#define MCAN_TXFQS_TX_FIFO_GET_INDEX_Pos          8
#define MCAN_TXFQS_TX_FIFO_GET_INDEX_Mask         (0x1Fu << MCAN_TXFQS_TX_FIFO_GET_INDEX_Pos)
#define MCAN_TXFQS_TX_FIFO_GET_INDEX(value)       (((uint32_t)(value) & MCAN_TXFQS_TX_FIFO_GET_INDEX_Mask) >> MCAN_TXFQS_TX_FIFO_GET_INDEX_Pos) //!< Get Tx FIFO Get Index
#define MCAN_TXFQS_TX_FIFO_PUT_INDEX_Pos          16
#define MCAN_TXFQS_TX_FIFO_PUT_INDEX_Mask         (0x1Fu << MCAN_TXFQS_TX_FIFO_PUT_INDEX_Pos)
#define MCAN_TXFQS_TX_FIFO_PUT_INDEX_GET(value)   (((uint32_t)(value) & MCAN_TXFQS_TX_FIFO_PUT_INDEX_Mask) >> MCAN_TXFQS_TX_FIFO_PUT_INDEX_Pos) //!< Get Receive FIFO 1 Put Index
#define MCAN_TXFQS_TX_FIFO_QUEUE_FULL             (0x1u << 21) //!< Tx FIFO/Queue full
#define MCAN_TXFQS_TX_FIFO_QUEUE_NOT_FULL         (0x0u << 21) //!< Tx FIFO/Queue not full

//-----------------------------------------------------------------------------

/*! MCAN Tx Buffer Element Size Configuration (Read/Write, Offset: 0xC8)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * Configures the number of data bytes belonging to a Tx Buffer element. Data field sizes > 8 bytes are intended for CAN FD operation only.
 * In case the data length code DLC of a Tx Buffer element is configured to a value higher than the Tx Buffer data field size MCAN_TXESC.TBDS, the bytes not defined by the Tx Buffer are transmitted as '0xCC' (padding bytes).
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXESC_Register
{
  uint32_t TXESC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TBDS:  3; //!< 0- 2 - Tx Buffer Data Field Size
    uint32_t     : 29; //!< 3-31
  } Bits;
} MCAN_TXESC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXESC_Register, 4);

#define MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Pos         0
#define MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Mask        (0x7u << MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Pos)
#define MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_GET(value)  (((uint32_t)(value) & MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Mask) >> MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Pos) //!< Get Tx Buffer Data Field Size
#define MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_SET(value)  (((uint32_t)(value) << MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Pos) & MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_Mask) //!< Set Tx Buffer Data Field Size

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Request Pending (Read-only, Offset: 0xCC)
 * Each Tx Buffer has its own Transmission Request Pending bit. The bits are set via register MCAN_TXBAR. The bits are reset after a requested transmission has completed or has been cancelled via register MCAN_TXBCR
 * TXBRP bits are set only for those Tx Buffers configured via MCAN_TXBC. After a MCAN_TXBRP bit has been set, a Tx scan (see Section 49.5.5) is started to check for the pending Tx request with the highest priority (Tx Buffer with lowest Message ID).
 * A cancellation request resets the corresponding transmission request pending bit of register MCAN_TXBRP. In case a transmission has already been started when a cancellation is requested, this is done at the end of the transmission, regardless whether the transmission was successful or not. The cancellation request bits are reset directly after the corresponding TXBRP bit has been reset.
 * After a cancellation has been requested, a finished cancellation is signalled via MCAN_TXBCF.
 * - after successful transmission together with the corresponding MCAN_TXBTO bit
 * - when the transmission has not yet been started at the point of cancellation
 * - when the transmission has been aborted due to lost arbitration
 * - when an error occurred during frame transmission
 * In DAR mode, all transmissions are automatically cancelled if they are not successful. The corresponding MCAN_TXBCF bit is set for all unsuccessful transmissions
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBRP_Register
{
  uint32_t TXBRP;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TRP0 : 1; //!<  0 - Transmission Request Pending for Buffer 0
    uint32_t TRP1 : 1; //!<  1 - Transmission Request Pending for Buffer 1
    uint32_t TRP2 : 1; //!<  2 - Transmission Request Pending for Buffer 2
    uint32_t TRP3 : 1; //!<  3 - Transmission Request Pending for Buffer 3
    uint32_t TRP4 : 1; //!<  4 - Transmission Request Pending for Buffer 4
    uint32_t TRP5 : 1; //!<  5 - Transmission Request Pending for Buffer 5
    uint32_t TRP6 : 1; //!<  6 - Transmission Request Pending for Buffer 6
    uint32_t TRP7 : 1; //!<  7 - Transmission Request Pending for Buffer 7
    uint32_t TRP8 : 1; //!<  8 - Transmission Request Pending for Buffer 8
    uint32_t TRP9 : 1; //!<  9 - Transmission Request Pending for Buffer 9
    uint32_t TRP10: 1; //!< 10 - Transmission Request Pending for Buffer 10
    uint32_t TRP11: 1; //!< 11 - Transmission Request Pending for Buffer 11
    uint32_t TRP12: 1; //!< 12 - Transmission Request Pending for Buffer 12
    uint32_t TRP13: 1; //!< 13 - Transmission Request Pending for Buffer 13
    uint32_t TRP14: 1; //!< 14 - Transmission Request Pending for Buffer 14
    uint32_t TRP15: 1; //!< 15 - Transmission Request Pending for Buffer 15
    uint32_t TRP16: 1; //!< 16 - Transmission Request Pending for Buffer 16
    uint32_t TRP17: 1; //!< 17 - Transmission Request Pending for Buffer 17
    uint32_t TRP18: 1; //!< 18 - Transmission Request Pending for Buffer 18
    uint32_t TRP19: 1; //!< 19 - Transmission Request Pending for Buffer 19
    uint32_t TRP20: 1; //!< 20 - Transmission Request Pending for Buffer 20
    uint32_t TRP21: 1; //!< 21 - Transmission Request Pending for Buffer 21
    uint32_t TRP22: 1; //!< 22 - Transmission Request Pending for Buffer 22
    uint32_t TRP23: 1; //!< 23 - Transmission Request Pending for Buffer 23
    uint32_t TRP24: 1; //!< 24 - Transmission Request Pending for Buffer 24
    uint32_t TRP25: 1; //!< 25 - Transmission Request Pending for Buffer 25
    uint32_t TRP26: 1; //!< 26 - Transmission Request Pending for Buffer 26
    uint32_t TRP27: 1; //!< 27 - Transmission Request Pending for Buffer 27
    uint32_t TRP28: 1; //!< 28 - Transmission Request Pending for Buffer 28
    uint32_t TRP29: 1; //!< 29 - Transmission Request Pending for Buffer 29
    uint32_t TRP30: 1; //!< 30 - Transmission Request Pending for Buffer 30
    uint32_t TRP31: 1; //!< 31 - Transmission Request Pending for Buffer 31
  } Bits;
} MCAN_TXBRP_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBRP_Register, 4);

#define MCAN_TX_REQ_PENDING_BUFFER_0   (0x1u <<  0) //!< Transmission Request Pending for Buffer 0 flag
#define MCAN_TX_REQ_PENDING_BUFFER_1   (0x1u <<  1) //!< Transmission Request Pending for Buffer 1 flag
#define MCAN_TX_REQ_PENDING_BUFFER_2   (0x1u <<  2) //!< Transmission Request Pending for Buffer 2 flag
#define MCAN_TX_REQ_PENDING_BUFFER_3   (0x1u <<  3) //!< Transmission Request Pending for Buffer 3 flag
#define MCAN_TX_REQ_PENDING_BUFFER_4   (0x1u <<  4) //!< Transmission Request Pending for Buffer 4 flag
#define MCAN_TX_REQ_PENDING_BUFFER_5   (0x1u <<  5) //!< Transmission Request Pending for Buffer 5 flag
#define MCAN_TX_REQ_PENDING_BUFFER_6   (0x1u <<  6) //!< Transmission Request Pending for Buffer 6 flag
#define MCAN_TX_REQ_PENDING_BUFFER_7   (0x1u <<  7) //!< Transmission Request Pending for Buffer 7 flag
#define MCAN_TX_REQ_PENDING_BUFFER_8   (0x1u <<  8) //!< Transmission Request Pending for Buffer 8 flag
#define MCAN_TX_REQ_PENDING_BUFFER_9   (0x1u <<  9) //!< Transmission Request Pending for Buffer 9 flag
#define MCAN_TX_REQ_PENDING_BUFFER_10  (0x1u << 10) //!< Transmission Request Pending for Buffer 10 flag
#define MCAN_TX_REQ_PENDING_BUFFER_11  (0x1u << 11) //!< Transmission Request Pending for Buffer 11 flag
#define MCAN_TX_REQ_PENDING_BUFFER_12  (0x1u << 12) //!< Transmission Request Pending for Buffer 12 flag
#define MCAN_TX_REQ_PENDING_BUFFER_13  (0x1u << 13) //!< Transmission Request Pending for Buffer 13 flag
#define MCAN_TX_REQ_PENDING_BUFFER_14  (0x1u << 14) //!< Transmission Request Pending for Buffer 14 flag
#define MCAN_TX_REQ_PENDING_BUFFER_15  (0x1u << 15) //!< Transmission Request Pending for Buffer 15 flag
#define MCAN_TX_REQ_PENDING_BUFFER_16  (0x1u << 16) //!< Transmission Request Pending for Buffer 16 flag
#define MCAN_TX_REQ_PENDING_BUFFER_17  (0x1u << 17) //!< Transmission Request Pending for Buffer 17 flag
#define MCAN_TX_REQ_PENDING_BUFFER_18  (0x1u << 18) //!< Transmission Request Pending for Buffer 18 flag
#define MCAN_TX_REQ_PENDING_BUFFER_19  (0x1u << 19) //!< Transmission Request Pending for Buffer 19 flag
#define MCAN_TX_REQ_PENDING_BUFFER_20  (0x1u << 20) //!< Transmission Request Pending for Buffer 20 flag
#define MCAN_TX_REQ_PENDING_BUFFER_21  (0x1u << 21) //!< Transmission Request Pending for Buffer 21 flag
#define MCAN_TX_REQ_PENDING_BUFFER_22  (0x1u << 22) //!< Transmission Request Pending for Buffer 22 flag
#define MCAN_TX_REQ_PENDING_BUFFER_23  (0x1u << 23) //!< Transmission Request Pending for Buffer 23 flag
#define MCAN_TX_REQ_PENDING_BUFFER_24  (0x1u << 24) //!< Transmission Request Pending for Buffer 24 flag
#define MCAN_TX_REQ_PENDING_BUFFER_25  (0x1u << 25) //!< Transmission Request Pending for Buffer 25 flag
#define MCAN_TX_REQ_PENDING_BUFFER_26  (0x1u << 26) //!< Transmission Request Pending for Buffer 26 flag
#define MCAN_TX_REQ_PENDING_BUFFER_27  (0x1u << 27) //!< Transmission Request Pending for Buffer 27 flag
#define MCAN_TX_REQ_PENDING_BUFFER_28  (0x1u << 28) //!< Transmission Request Pending for Buffer 28 flag
#define MCAN_TX_REQ_PENDING_BUFFER_29  (0x1u << 29) //!< Transmission Request Pending for Buffer 29 flag
#define MCAN_TX_REQ_PENDING_BUFFER_30  (0x1u << 30) //!< Transmission Request Pending for Buffer 30 flag
#define MCAN_TX_REQ_PENDING_BUFFER_31  (0x1u << 31) //!< Transmission Request Pending for Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Add Request (Read/Write, Offset: 0xD0)
 * Each Transmit Buffer has its own Add Request bit. Writing a '1' will set the corresponding Add Request bit; writing a '0' has no impact.
 * This enables the processor to set transmission requests for multiple Transmit Buffers with one write to MCAN_TXBAR.
 * MCAN_TXBAR bits are set only for those Transmit Buffers configured via TXBC.
 * When no Transmit scan is running, the bits are reset immediately, else the bits remain set until the Transmit scan process has completed
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBAR_Register
{
  uint32_t TXBAR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t AR0 : 1; //!<  0 - Add Request Added for Transmit Buffer 0
    uint32_t AR1 : 1; //!<  1 - Add Request Added for Transmit Buffer 1
    uint32_t AR2 : 1; //!<  2 - Add Request Added for Transmit Buffer 2
    uint32_t AR3 : 1; //!<  3 - Add Request Added for Transmit Buffer 3
    uint32_t AR4 : 1; //!<  4 - Add Request Added for Transmit Buffer 4
    uint32_t AR5 : 1; //!<  5 - Add Request Added for Transmit Buffer 5
    uint32_t AR6 : 1; //!<  6 - Add Request Added for Transmit Buffer 6
    uint32_t AR7 : 1; //!<  7 - Add Request Added for Transmit Buffer 7
    uint32_t AR8 : 1; //!<  8 - Add Request Added for Transmit Buffer 8
    uint32_t AR9 : 1; //!<  9 - Add Request Added for Transmit Buffer 9
    uint32_t AR10: 1; //!< 10 - Add Request Added for Transmit Buffer 10
    uint32_t AR11: 1; //!< 11 - Add Request Added for Transmit Buffer 11
    uint32_t AR12: 1; //!< 12 - Add Request Added for Transmit Buffer 12
    uint32_t AR13: 1; //!< 13 - Add Request Added for Transmit Buffer 13
    uint32_t AR14: 1; //!< 14 - Add Request Added for Transmit Buffer 14
    uint32_t AR15: 1; //!< 15 - Add Request Added for Transmit Buffer 15
    uint32_t AR16: 1; //!< 16 - Add Request Added for Transmit Buffer 16
    uint32_t AR17: 1; //!< 17 - Add Request Added for Transmit Buffer 17
    uint32_t AR18: 1; //!< 18 - Add Request Added for Transmit Buffer 18
    uint32_t AR19: 1; //!< 19 - Add Request Added for Transmit Buffer 19
    uint32_t AR20: 1; //!< 20 - Add Request Added for Transmit Buffer 20
    uint32_t AR21: 1; //!< 21 - Add Request Added for Transmit Buffer 21
    uint32_t AR22: 1; //!< 22 - Add Request Added for Transmit Buffer 22
    uint32_t AR23: 1; //!< 23 - Add Request Added for Transmit Buffer 23
    uint32_t AR24: 1; //!< 24 - Add Request Added for Transmit Buffer 24
    uint32_t AR25: 1; //!< 25 - Add Request Added for Transmit Buffer 25
    uint32_t AR26: 1; //!< 26 - Add Request Added for Transmit Buffer 26
    uint32_t AR27: 1; //!< 27 - Add Request Added for Transmit Buffer 27
    uint32_t AR28: 1; //!< 28 - Add Request Added for Transmit Buffer 28
    uint32_t AR29: 1; //!< 29 - Add Request Added for Transmit Buffer 29
    uint32_t AR30: 1; //!< 30 - Add Request Added for Transmit Buffer 30
    uint32_t AR31: 1; //!< 31 - Add Request Added for Transmit Buffer 31
  } Bits;
} MCAN_TXBAR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBAR_Register, 4);

#define MCAN_TX_REQ_ADDED_BUFFER_0   (0x1u <<  0) //!< Transmission Request Added for Buffer 0 flag
#define MCAN_TX_REQ_ADDED_BUFFER_1   (0x1u <<  1) //!< Transmission Request Added for Buffer 1 flag
#define MCAN_TX_REQ_ADDED_BUFFER_2   (0x1u <<  2) //!< Transmission Request Added for Buffer 2 flag
#define MCAN_TX_REQ_ADDED_BUFFER_3   (0x1u <<  3) //!< Transmission Request Added for Buffer 3 flag
#define MCAN_TX_REQ_ADDED_BUFFER_4   (0x1u <<  4) //!< Transmission Request Added for Buffer 4 flag
#define MCAN_TX_REQ_ADDED_BUFFER_5   (0x1u <<  5) //!< Transmission Request Added for Buffer 5 flag
#define MCAN_TX_REQ_ADDED_BUFFER_6   (0x1u <<  6) //!< Transmission Request Added for Buffer 6 flag
#define MCAN_TX_REQ_ADDED_BUFFER_7   (0x1u <<  7) //!< Transmission Request Added for Buffer 7 flag
#define MCAN_TX_REQ_ADDED_BUFFER_8   (0x1u <<  8) //!< Transmission Request Added for Buffer 8 flag
#define MCAN_TX_REQ_ADDED_BUFFER_9   (0x1u <<  9) //!< Transmission Request Added for Buffer 9 flag
#define MCAN_TX_REQ_ADDED_BUFFER_10  (0x1u << 10) //!< Transmission Request Added for Buffer 10 flag
#define MCAN_TX_REQ_ADDED_BUFFER_11  (0x1u << 11) //!< Transmission Request Added for Buffer 11 flag
#define MCAN_TX_REQ_ADDED_BUFFER_12  (0x1u << 12) //!< Transmission Request Added for Buffer 12 flag
#define MCAN_TX_REQ_ADDED_BUFFER_13  (0x1u << 13) //!< Transmission Request Added for Buffer 13 flag
#define MCAN_TX_REQ_ADDED_BUFFER_14  (0x1u << 14) //!< Transmission Request Added for Buffer 14 flag
#define MCAN_TX_REQ_ADDED_BUFFER_15  (0x1u << 15) //!< Transmission Request Added for Buffer 15 flag
#define MCAN_TX_REQ_ADDED_BUFFER_16  (0x1u << 16) //!< Transmission Request Added for Buffer 16 flag
#define MCAN_TX_REQ_ADDED_BUFFER_17  (0x1u << 17) //!< Transmission Request Added for Buffer 17 flag
#define MCAN_TX_REQ_ADDED_BUFFER_18  (0x1u << 18) //!< Transmission Request Added for Buffer 18 flag
#define MCAN_TX_REQ_ADDED_BUFFER_19  (0x1u << 19) //!< Transmission Request Added for Buffer 19 flag
#define MCAN_TX_REQ_ADDED_BUFFER_20  (0x1u << 20) //!< Transmission Request Added for Buffer 20 flag
#define MCAN_TX_REQ_ADDED_BUFFER_21  (0x1u << 21) //!< Transmission Request Added for Buffer 21 flag
#define MCAN_TX_REQ_ADDED_BUFFER_22  (0x1u << 22) //!< Transmission Request Added for Buffer 22 flag
#define MCAN_TX_REQ_ADDED_BUFFER_23  (0x1u << 23) //!< Transmission Request Added for Buffer 23 flag
#define MCAN_TX_REQ_ADDED_BUFFER_24  (0x1u << 24) //!< Transmission Request Added for Buffer 24 flag
#define MCAN_TX_REQ_ADDED_BUFFER_25  (0x1u << 25) //!< Transmission Request Added for Buffer 25 flag
#define MCAN_TX_REQ_ADDED_BUFFER_26  (0x1u << 26) //!< Transmission Request Added for Buffer 26 flag
#define MCAN_TX_REQ_ADDED_BUFFER_27  (0x1u << 27) //!< Transmission Request Added for Buffer 27 flag
#define MCAN_TX_REQ_ADDED_BUFFER_28  (0x1u << 28) //!< Transmission Request Added for Buffer 28 flag
#define MCAN_TX_REQ_ADDED_BUFFER_29  (0x1u << 29) //!< Transmission Request Added for Buffer 29 flag
#define MCAN_TX_REQ_ADDED_BUFFER_30  (0x1u << 30) //!< Transmission Request Added for Buffer 30 flag
#define MCAN_TX_REQ_ADDED_BUFFER_31  (0x1u << 31) //!< Transmission Request Added for Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Cancellation Request (Read/Write, Offset: 0xD4)
 * Each Transmit Buffer has its own Cancellation Request bit. Writing a '1' will set the corresponding Cancellation Request bit; writing a '0' has no impact.
 * This enables the processor to set cancellation requests for multiple Transmit Buffers with one write to MCAN_TXBCR.
 * MCAN_TXBCR bits are set only for those Transmit Buffers configured via TXBC. The bits remain set until the corresponding bit of MCAN_TXBRP is reset
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBCR_Register
{
  uint32_t TXBCR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t CR0 : 1; //!<  0 - Cancellation Request for Transmit Buffer 0
    uint32_t CR1 : 1; //!<  1 - Cancellation Request for Transmit Buffer 1
    uint32_t CR2 : 1; //!<  2 - Cancellation Request for Transmit Buffer 2
    uint32_t CR3 : 1; //!<  3 - Cancellation Request for Transmit Buffer 3
    uint32_t CR4 : 1; //!<  4 - Cancellation Request for Transmit Buffer 4
    uint32_t CR5 : 1; //!<  5 - Cancellation Request for Transmit Buffer 5
    uint32_t CR6 : 1; //!<  6 - Cancellation Request for Transmit Buffer 6
    uint32_t CR7 : 1; //!<  7 - Cancellation Request for Transmit Buffer 7
    uint32_t CR8 : 1; //!<  8 - Cancellation Request for Transmit Buffer 8
    uint32_t CR9 : 1; //!<  9 - Cancellation Request for Transmit Buffer 9
    uint32_t CR10: 1; //!< 10 - Cancellation Request for Transmit Buffer 10
    uint32_t CR11: 1; //!< 11 - Cancellation Request for Transmit Buffer 11
    uint32_t CR12: 1; //!< 12 - Cancellation Request for Transmit Buffer 12
    uint32_t CR13: 1; //!< 13 - Cancellation Request for Transmit Buffer 13
    uint32_t CR14: 1; //!< 14 - Cancellation Request for Transmit Buffer 14
    uint32_t CR15: 1; //!< 15 - Cancellation Request for Transmit Buffer 15
    uint32_t CR16: 1; //!< 16 - Cancellation Request for Transmit Buffer 16
    uint32_t CR17: 1; //!< 17 - Cancellation Request for Transmit Buffer 17
    uint32_t CR18: 1; //!< 18 - Cancellation Request for Transmit Buffer 18
    uint32_t CR19: 1; //!< 19 - Cancellation Request for Transmit Buffer 19
    uint32_t CR20: 1; //!< 20 - Cancellation Request for Transmit Buffer 20
    uint32_t CR21: 1; //!< 21 - Cancellation Request for Transmit Buffer 21
    uint32_t CR22: 1; //!< 22 - Cancellation Request for Transmit Buffer 22
    uint32_t CR23: 1; //!< 23 - Cancellation Request for Transmit Buffer 23
    uint32_t CR24: 1; //!< 24 - Cancellation Request for Transmit Buffer 24
    uint32_t CR25: 1; //!< 25 - Cancellation Request for Transmit Buffer 25
    uint32_t CR26: 1; //!< 26 - Cancellation Request for Transmit Buffer 26
    uint32_t CR27: 1; //!< 27 - Cancellation Request for Transmit Buffer 27
    uint32_t CR28: 1; //!< 28 - Cancellation Request for Transmit Buffer 28
    uint32_t CR29: 1; //!< 29 - Cancellation Request for Transmit Buffer 29
    uint32_t CR30: 1; //!< 30 - Cancellation Request for Transmit Buffer 30
    uint32_t CR31: 1; //!< 31 - Cancellation Request for Transmit Buffer 31
  } Bits;
} MCAN_TXBCR_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBCR_Register, 4);

#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_0   (0x1u <<  0) //!< Cancellation Request for Transmit Buffer 0 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_1   (0x1u <<  1) //!< Cancellation Request for Transmit Buffer 1 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_2   (0x1u <<  2) //!< Cancellation Request for Transmit Buffer 2 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_3   (0x1u <<  3) //!< Cancellation Request for Transmit Buffer 3 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_4   (0x1u <<  4) //!< Cancellation Request for Transmit Buffer 4 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_5   (0x1u <<  5) //!< Cancellation Request for Transmit Buffer 5 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_6   (0x1u <<  6) //!< Cancellation Request for Transmit Buffer 6 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_7   (0x1u <<  7) //!< Cancellation Request for Transmit Buffer 7 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_8   (0x1u <<  8) //!< Cancellation Request for Transmit Buffer 8 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_9   (0x1u <<  9) //!< Cancellation Request for Transmit Buffer 9 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_10  (0x1u << 10) //!< Cancellation Request for Transmit Buffer 10 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_11  (0x1u << 11) //!< Cancellation Request for Transmit Buffer 11 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_12  (0x1u << 12) //!< Cancellation Request for Transmit Buffer 12 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_13  (0x1u << 13) //!< Cancellation Request for Transmit Buffer 13 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_14  (0x1u << 14) //!< Cancellation Request for Transmit Buffer 14 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_15  (0x1u << 15) //!< Cancellation Request for Transmit Buffer 15 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_16  (0x1u << 16) //!< Cancellation Request for Transmit Buffer 16 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_17  (0x1u << 17) //!< Cancellation Request for Transmit Buffer 17 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_18  (0x1u << 18) //!< Cancellation Request for Transmit Buffer 18 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_19  (0x1u << 19) //!< Cancellation Request for Transmit Buffer 19 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_20  (0x1u << 20) //!< Cancellation Request for Transmit Buffer 20 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_21  (0x1u << 21) //!< Cancellation Request for Transmit Buffer 21 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_22  (0x1u << 22) //!< Cancellation Request for Transmit Buffer 22 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_23  (0x1u << 23) //!< Cancellation Request for Transmit Buffer 23 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_24  (0x1u << 24) //!< Cancellation Request for Transmit Buffer 24 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_25  (0x1u << 25) //!< Cancellation Request for Transmit Buffer 25 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_26  (0x1u << 26) //!< Cancellation Request for Transmit Buffer 26 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_27  (0x1u << 27) //!< Cancellation Request for Transmit Buffer 27 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_28  (0x1u << 28) //!< Cancellation Request for Transmit Buffer 28 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_29  (0x1u << 29) //!< Cancellation Request for Transmit Buffer 29 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_30  (0x1u << 30) //!< Cancellation Request for Transmit Buffer 30 flag
#define MCAN_CANCEL_REQ_FOR_TX_BUFFER_31  (0x1u << 31) //!< Cancellation Request for Transmit Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Transmission Occurred (Read-only, Offset: 0xD8)
 * Each Transmit Buffer has its own Transmission Occurred bit. The bits are set when the corresponding MCAN_TXBRP bit is cleared after a successful transmission.
 * The bits are reset when a new transmission is requested by writing a '1' to the corresponding bit of register MCAN_TXBAR
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBTO_Register
{
  uint32_t TXBTO;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TO0 : 1; //!<  0 - Transmission Occurred for Buffer 0
    uint32_t TO1 : 1; //!<  1 - Transmission Occurred for Buffer 1
    uint32_t TO2 : 1; //!<  2 - Transmission Occurred for Buffer 2
    uint32_t TO3 : 1; //!<  3 - Transmission Occurred for Buffer 3
    uint32_t TO4 : 1; //!<  4 - Transmission Occurred for Buffer 4
    uint32_t TO5 : 1; //!<  5 - Transmission Occurred for Buffer 5
    uint32_t TO6 : 1; //!<  6 - Transmission Occurred for Buffer 6
    uint32_t TO7 : 1; //!<  7 - Transmission Occurred for Buffer 7
    uint32_t TO8 : 1; //!<  8 - Transmission Occurred for Buffer 8
    uint32_t TO9 : 1; //!<  9 - Transmission Occurred for Buffer 9
    uint32_t TO10: 1; //!< 10 - Transmission Occurred for Buffer 10
    uint32_t TO11: 1; //!< 11 - Transmission Occurred for Buffer 11
    uint32_t TO12: 1; //!< 12 - Transmission Occurred for Buffer 12
    uint32_t TO13: 1; //!< 13 - Transmission Occurred for Buffer 13
    uint32_t TO14: 1; //!< 14 - Transmission Occurred for Buffer 14
    uint32_t TO15: 1; //!< 15 - Transmission Occurred for Buffer 15
    uint32_t TO16: 1; //!< 16 - Transmission Occurred for Buffer 16
    uint32_t TO17: 1; //!< 17 - Transmission Occurred for Buffer 17
    uint32_t TO18: 1; //!< 18 - Transmission Occurred for Buffer 18
    uint32_t TO19: 1; //!< 19 - Transmission Occurred for Buffer 19
    uint32_t TO20: 1; //!< 20 - Transmission Occurred for Buffer 20
    uint32_t TO21: 1; //!< 21 - Transmission Occurred for Buffer 21
    uint32_t TO22: 1; //!< 22 - Transmission Occurred for Buffer 22
    uint32_t TO23: 1; //!< 23 - Transmission Occurred for Buffer 23
    uint32_t TO24: 1; //!< 24 - Transmission Occurred for Buffer 24
    uint32_t TO25: 1; //!< 25 - Transmission Occurred for Buffer 25
    uint32_t TO26: 1; //!< 26 - Transmission Occurred for Buffer 26
    uint32_t TO27: 1; //!< 27 - Transmission Occurred for Buffer 27
    uint32_t TO28: 1; //!< 28 - Transmission Occurred for Buffer 28
    uint32_t TO29: 1; //!< 29 - Transmission Occurred for Buffer 29
    uint32_t TO30: 1; //!< 30 - Transmission Occurred for Buffer 30
    uint32_t TO31: 1; //!< 31 - Transmission Occurred for Buffer 31
  } Bits;
} MCAN_TXBTO_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBTO_Register, 4);

#define MCAN_TX_OCCURED_BUFFER_0   (0x1u <<  0) //!< Transmission Occurred for Buffer 0 flag
#define MCAN_TX_OCCURED_BUFFER_1   (0x1u <<  1) //!< Transmission Occurred for Buffer 1 flag
#define MCAN_TX_OCCURED_BUFFER_2   (0x1u <<  2) //!< Transmission Occurred for Buffer 2 flag
#define MCAN_TX_OCCURED_BUFFER_3   (0x1u <<  3) //!< Transmission Occurred for Buffer 3 flag
#define MCAN_TX_OCCURED_BUFFER_4   (0x1u <<  4) //!< Transmission Occurred for Buffer 4 flag
#define MCAN_TX_OCCURED_BUFFER_5   (0x1u <<  5) //!< Transmission Occurred for Buffer 5 flag
#define MCAN_TX_OCCURED_BUFFER_6   (0x1u <<  6) //!< Transmission Occurred for Buffer 6 flag
#define MCAN_TX_OCCURED_BUFFER_7   (0x1u <<  7) //!< Transmission Occurred for Buffer 7 flag
#define MCAN_TX_OCCURED_BUFFER_8   (0x1u <<  8) //!< Transmission Occurred for Buffer 8 flag
#define MCAN_TX_OCCURED_BUFFER_9   (0x1u <<  9) //!< Transmission Occurred for Buffer 9 flag
#define MCAN_TX_OCCURED_BUFFER_10  (0x1u << 10) //!< Transmission Occurred for Buffer 10 flag
#define MCAN_TX_OCCURED_BUFFER_11  (0x1u << 11) //!< Transmission Occurred for Buffer 11 flag
#define MCAN_TX_OCCURED_BUFFER_12  (0x1u << 12) //!< Transmission Occurred for Buffer 12 flag
#define MCAN_TX_OCCURED_BUFFER_13  (0x1u << 13) //!< Transmission Occurred for Buffer 13 flag
#define MCAN_TX_OCCURED_BUFFER_14  (0x1u << 14) //!< Transmission Occurred for Buffer 14 flag
#define MCAN_TX_OCCURED_BUFFER_15  (0x1u << 15) //!< Transmission Occurred for Buffer 15 flag
#define MCAN_TX_OCCURED_BUFFER_16  (0x1u << 16) //!< Transmission Occurred for Buffer 16 flag
#define MCAN_TX_OCCURED_BUFFER_17  (0x1u << 17) //!< Transmission Occurred for Buffer 17 flag
#define MCAN_TX_OCCURED_BUFFER_18  (0x1u << 18) //!< Transmission Occurred for Buffer 18 flag
#define MCAN_TX_OCCURED_BUFFER_19  (0x1u << 19) //!< Transmission Occurred for Buffer 19 flag
#define MCAN_TX_OCCURED_BUFFER_20  (0x1u << 20) //!< Transmission Occurred for Buffer 20 flag
#define MCAN_TX_OCCURED_BUFFER_21  (0x1u << 21) //!< Transmission Occurred for Buffer 21 flag
#define MCAN_TX_OCCURED_BUFFER_22  (0x1u << 22) //!< Transmission Occurred for Buffer 22 flag
#define MCAN_TX_OCCURED_BUFFER_23  (0x1u << 23) //!< Transmission Occurred for Buffer 23 flag
#define MCAN_TX_OCCURED_BUFFER_24  (0x1u << 24) //!< Transmission Occurred for Buffer 24 flag
#define MCAN_TX_OCCURED_BUFFER_25  (0x1u << 25) //!< Transmission Occurred for Buffer 25 flag
#define MCAN_TX_OCCURED_BUFFER_26  (0x1u << 26) //!< Transmission Occurred for Buffer 26 flag
#define MCAN_TX_OCCURED_BUFFER_27  (0x1u << 27) //!< Transmission Occurred for Buffer 27 flag
#define MCAN_TX_OCCURED_BUFFER_28  (0x1u << 28) //!< Transmission Occurred for Buffer 28 flag
#define MCAN_TX_OCCURED_BUFFER_29  (0x1u << 29) //!< Transmission Occurred for Buffer 29 flag
#define MCAN_TX_OCCURED_BUFFER_30  (0x1u << 30) //!< Transmission Occurred for Buffer 30 flag
#define MCAN_TX_OCCURED_BUFFER_31  (0x1u << 31) //!< Transmission Occurred for Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Cancellation Finished (Read-only, Offset: 0xDC)
 * Each Transmit Buffer has its own Cancellation Finished bit. The bits are set when the corresponding MCAN_TXBRP bit is cleared after a cancellation was requested via MCAN_TXBCR.
 * In case the corresponding MCAN_TXBRP bit was not set at the point of cancellation, CF is set immediately. The bits are reset when a new transmission is requested by writing a '1' to the corresponding bit of register MCAN_TXBAR
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBCF_Register
{
  uint32_t TXBCF;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t CF0 : 1; //!<  0 - Cancellation Finished for Transmit Buffer 0
    uint32_t CF1 : 1; //!<  1 - Cancellation Finished for Transmit Buffer 1
    uint32_t CF2 : 1; //!<  2 - Cancellation Finished for Transmit Buffer 2
    uint32_t CF3 : 1; //!<  3 - Cancellation Finished for Transmit Buffer 3
    uint32_t CF4 : 1; //!<  4 - Cancellation Finished for Transmit Buffer 4
    uint32_t CF5 : 1; //!<  5 - Cancellation Finished for Transmit Buffer 5
    uint32_t CF6 : 1; //!<  6 - Cancellation Finished for Transmit Buffer 6
    uint32_t CF7 : 1; //!<  7 - Cancellation Finished for Transmit Buffer 7
    uint32_t CF8 : 1; //!<  8 - Cancellation Finished for Transmit Buffer 8
    uint32_t CF9 : 1; //!<  9 - Cancellation Finished for Transmit Buffer 9
    uint32_t CF10: 1; //!< 10 - Cancellation Finished for Transmit Buffer 10
    uint32_t CF11: 1; //!< 11 - Cancellation Finished for Transmit Buffer 11
    uint32_t CF12: 1; //!< 12 - Cancellation Finished for Transmit Buffer 12
    uint32_t CF13: 1; //!< 13 - Cancellation Finished for Transmit Buffer 13
    uint32_t CF14: 1; //!< 14 - Cancellation Finished for Transmit Buffer 14
    uint32_t CF15: 1; //!< 15 - Cancellation Finished for Transmit Buffer 15
    uint32_t CF16: 1; //!< 16 - Cancellation Finished for Transmit Buffer 16
    uint32_t CF17: 1; //!< 17 - Cancellation Finished for Transmit Buffer 17
    uint32_t CF18: 1; //!< 18 - Cancellation Finished for Transmit Buffer 18
    uint32_t CF19: 1; //!< 19 - Cancellation Finished for Transmit Buffer 19
    uint32_t CF20: 1; //!< 20 - Cancellation Finished for Transmit Buffer 20
    uint32_t CF21: 1; //!< 21 - Cancellation Finished for Transmit Buffer 21
    uint32_t CF22: 1; //!< 22 - Cancellation Finished for Transmit Buffer 22
    uint32_t CF23: 1; //!< 23 - Cancellation Finished for Transmit Buffer 23
    uint32_t CF24: 1; //!< 24 - Cancellation Finished for Transmit Buffer 24
    uint32_t CF25: 1; //!< 25 - Cancellation Finished for Transmit Buffer 25
    uint32_t CF26: 1; //!< 26 - Cancellation Finished for Transmit Buffer 26
    uint32_t CF27: 1; //!< 27 - Cancellation Finished for Transmit Buffer 27
    uint32_t CF28: 1; //!< 28 - Cancellation Finished for Transmit Buffer 28
    uint32_t CF29: 1; //!< 29 - Cancellation Finished for Transmit Buffer 29
    uint32_t CF30: 1; //!< 30 - Cancellation Finished for Transmit Buffer 30
    uint32_t CF31: 1; //!< 31 - Cancellation Finished for Transmit Buffer 31
  } Bits;
} MCAN_TXBCF_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBCF_Register, 4);

#define MCAN_CANCEL_FINISH_TX_BUFFER_0   (0x1u <<  0) //!< Cancellation Finished for Transmit Buffer 0 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_1   (0x1u <<  1) //!< Cancellation Finished for Transmit Buffer 1 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_2   (0x1u <<  2) //!< Cancellation Finished for Transmit Buffer 2 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_3   (0x1u <<  3) //!< Cancellation Finished for Transmit Buffer 3 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_4   (0x1u <<  4) //!< Cancellation Finished for Transmit Buffer 4 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_5   (0x1u <<  5) //!< Cancellation Finished for Transmit Buffer 5 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_6   (0x1u <<  6) //!< Cancellation Finished for Transmit Buffer 6 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_7   (0x1u <<  7) //!< Cancellation Finished for Transmit Buffer 7 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_8   (0x1u <<  8) //!< Cancellation Finished for Transmit Buffer 8 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_9   (0x1u <<  9) //!< Cancellation Finished for Transmit Buffer 9 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_10  (0x1u << 10) //!< Cancellation Finished for Transmit Buffer 10 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_11  (0x1u << 11) //!< Cancellation Finished for Transmit Buffer 11 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_12  (0x1u << 12) //!< Cancellation Finished for Transmit Buffer 12 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_13  (0x1u << 13) //!< Cancellation Finished for Transmit Buffer 13 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_14  (0x1u << 14) //!< Cancellation Finished for Transmit Buffer 14 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_15  (0x1u << 15) //!< Cancellation Finished for Transmit Buffer 15 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_16  (0x1u << 16) //!< Cancellation Finished for Transmit Buffer 16 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_17  (0x1u << 17) //!< Cancellation Finished for Transmit Buffer 17 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_18  (0x1u << 18) //!< Cancellation Finished for Transmit Buffer 18 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_19  (0x1u << 19) //!< Cancellation Finished for Transmit Buffer 19 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_20  (0x1u << 20) //!< Cancellation Finished for Transmit Buffer 20 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_21  (0x1u << 21) //!< Cancellation Finished for Transmit Buffer 21 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_22  (0x1u << 22) //!< Cancellation Finished for Transmit Buffer 22 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_23  (0x1u << 23) //!< Cancellation Finished for Transmit Buffer 23 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_24  (0x1u << 24) //!< Cancellation Finished for Transmit Buffer 24 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_25  (0x1u << 25) //!< Cancellation Finished for Transmit Buffer 25 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_26  (0x1u << 26) //!< Cancellation Finished for Transmit Buffer 26 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_27  (0x1u << 27) //!< Cancellation Finished for Transmit Buffer 27 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_28  (0x1u << 28) //!< Cancellation Finished for Transmit Buffer 28 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_29  (0x1u << 29) //!< Cancellation Finished for Transmit Buffer 29 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_30  (0x1u << 30) //!< Cancellation Finished for Transmit Buffer 30 flag
#define MCAN_CANCEL_FINISH_TX_BUFFER_31  (0x1u << 31) //!< Cancellation Finished for Transmit Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Transmission Interrupt Enable (Read/Write, Offset: 0xE0)
 * Each Transmit Buffer has its own Transmission Interrupt Enable bit
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBTIE_Register
{
  uint32_t TXBTIE;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TIE0 : 1; //!<  0 - Transmission Interrupt Enable for Buffer 0
    uint32_t TIE1 : 1; //!<  1 - Transmission Interrupt Enable for Buffer 1
    uint32_t TIE2 : 1; //!<  2 - Transmission Interrupt Enable for Buffer 2
    uint32_t TIE3 : 1; //!<  3 - Transmission Interrupt Enable for Buffer 3
    uint32_t TIE4 : 1; //!<  4 - Transmission Interrupt Enable for Buffer 4
    uint32_t TIE5 : 1; //!<  5 - Transmission Interrupt Enable for Buffer 5
    uint32_t TIE6 : 1; //!<  6 - Transmission Interrupt Enable for Buffer 6
    uint32_t TIE7 : 1; //!<  7 - Transmission Interrupt Enable for Buffer 7
    uint32_t TIE8 : 1; //!<  8 - Transmission Interrupt Enable for Buffer 8
    uint32_t TIE9 : 1; //!<  9 - Transmission Interrupt Enable for Buffer 9
    uint32_t TIE10: 1; //!< 10 - Transmission Interrupt Enable for Buffer 10
    uint32_t TIE11: 1; //!< 11 - Transmission Interrupt Enable for Buffer 11
    uint32_t TIE12: 1; //!< 12 - Transmission Interrupt Enable for Buffer 12
    uint32_t TIE13: 1; //!< 13 - Transmission Interrupt Enable for Buffer 13
    uint32_t TIE14: 1; //!< 14 - Transmission Interrupt Enable for Buffer 14
    uint32_t TIE15: 1; //!< 15 - Transmission Interrupt Enable for Buffer 15
    uint32_t TIE16: 1; //!< 16 - Transmission Interrupt Enable for Buffer 16
    uint32_t TIE17: 1; //!< 17 - Transmission Interrupt Enable for Buffer 17
    uint32_t TIE18: 1; //!< 18 - Transmission Interrupt Enable for Buffer 18
    uint32_t TIE19: 1; //!< 19 - Transmission Interrupt Enable for Buffer 19
    uint32_t TIE20: 1; //!< 20 - Transmission Interrupt Enable for Buffer 20
    uint32_t TIE21: 1; //!< 21 - Transmission Interrupt Enable for Buffer 21
    uint32_t TIE22: 1; //!< 22 - Transmission Interrupt Enable for Buffer 22
    uint32_t TIE23: 1; //!< 23 - Transmission Interrupt Enable for Buffer 23
    uint32_t TIE24: 1; //!< 24 - Transmission Interrupt Enable for Buffer 24
    uint32_t TIE25: 1; //!< 25 - Transmission Interrupt Enable for Buffer 25
    uint32_t TIE26: 1; //!< 26 - Transmission Interrupt Enable for Buffer 26
    uint32_t TIE27: 1; //!< 27 - Transmission Interrupt Enable for Buffer 27
    uint32_t TIE28: 1; //!< 28 - Transmission Interrupt Enable for Buffer 28
    uint32_t TIE29: 1; //!< 29 - Transmission Interrupt Enable for Buffer 29
    uint32_t TIE30: 1; //!< 30 - Transmission Interrupt Enable for Buffer 30
    uint32_t TIE31: 1; //!< 31 - Transmission Interrupt Enable for Buffer 31
  } Bits;
} MCAN_TXBTIE_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBTIE_Register, 4);

#define MCAN_TX_INT_ENABLE_BUFFER_0   (0x1u <<  0) //!< Transmission Interrupt Enable for Buffer 0 flag
#define MCAN_TX_INT_ENABLE_BUFFER_1   (0x1u <<  1) //!< Transmission Interrupt Enable for Buffer 1 flag
#define MCAN_TX_INT_ENABLE_BUFFER_2   (0x1u <<  2) //!< Transmission Interrupt Enable for Buffer 2 flag
#define MCAN_TX_INT_ENABLE_BUFFER_3   (0x1u <<  3) //!< Transmission Interrupt Enable for Buffer 3 flag
#define MCAN_TX_INT_ENABLE_BUFFER_4   (0x1u <<  4) //!< Transmission Interrupt Enable for Buffer 4 flag
#define MCAN_TX_INT_ENABLE_BUFFER_5   (0x1u <<  5) //!< Transmission Interrupt Enable for Buffer 5 flag
#define MCAN_TX_INT_ENABLE_BUFFER_6   (0x1u <<  6) //!< Transmission Interrupt Enable for Buffer 6 flag
#define MCAN_TX_INT_ENABLE_BUFFER_7   (0x1u <<  7) //!< Transmission Interrupt Enable for Buffer 7 flag
#define MCAN_TX_INT_ENABLE_BUFFER_8   (0x1u <<  8) //!< Transmission Interrupt Enable for Buffer 8 flag
#define MCAN_TX_INT_ENABLE_BUFFER_9   (0x1u <<  9) //!< Transmission Interrupt Enable for Buffer 9 flag
#define MCAN_TX_INT_ENABLE_BUFFER_10  (0x1u << 10) //!< Transmission Interrupt Enable for Buffer 10 flag
#define MCAN_TX_INT_ENABLE_BUFFER_11  (0x1u << 11) //!< Transmission Interrupt Enable for Buffer 11 flag
#define MCAN_TX_INT_ENABLE_BUFFER_12  (0x1u << 12) //!< Transmission Interrupt Enable for Buffer 12 flag
#define MCAN_TX_INT_ENABLE_BUFFER_13  (0x1u << 13) //!< Transmission Interrupt Enable for Buffer 13 flag
#define MCAN_TX_INT_ENABLE_BUFFER_14  (0x1u << 14) //!< Transmission Interrupt Enable for Buffer 14 flag
#define MCAN_TX_INT_ENABLE_BUFFER_15  (0x1u << 15) //!< Transmission Interrupt Enable for Buffer 15 flag
#define MCAN_TX_INT_ENABLE_BUFFER_16  (0x1u << 16) //!< Transmission Interrupt Enable for Buffer 16 flag
#define MCAN_TX_INT_ENABLE_BUFFER_17  (0x1u << 17) //!< Transmission Interrupt Enable for Buffer 17 flag
#define MCAN_TX_INT_ENABLE_BUFFER_18  (0x1u << 18) //!< Transmission Interrupt Enable for Buffer 18 flag
#define MCAN_TX_INT_ENABLE_BUFFER_19  (0x1u << 19) //!< Transmission Interrupt Enable for Buffer 19 flag
#define MCAN_TX_INT_ENABLE_BUFFER_20  (0x1u << 20) //!< Transmission Interrupt Enable for Buffer 20 flag
#define MCAN_TX_INT_ENABLE_BUFFER_21  (0x1u << 21) //!< Transmission Interrupt Enable for Buffer 21 flag
#define MCAN_TX_INT_ENABLE_BUFFER_22  (0x1u << 22) //!< Transmission Interrupt Enable for Buffer 22 flag
#define MCAN_TX_INT_ENABLE_BUFFER_23  (0x1u << 23) //!< Transmission Interrupt Enable for Buffer 23 flag
#define MCAN_TX_INT_ENABLE_BUFFER_24  (0x1u << 24) //!< Transmission Interrupt Enable for Buffer 24 flag
#define MCAN_TX_INT_ENABLE_BUFFER_25  (0x1u << 25) //!< Transmission Interrupt Enable for Buffer 25 flag
#define MCAN_TX_INT_ENABLE_BUFFER_26  (0x1u << 26) //!< Transmission Interrupt Enable for Buffer 26 flag
#define MCAN_TX_INT_ENABLE_BUFFER_27  (0x1u << 27) //!< Transmission Interrupt Enable for Buffer 27 flag
#define MCAN_TX_INT_ENABLE_BUFFER_28  (0x1u << 28) //!< Transmission Interrupt Enable for Buffer 28 flag
#define MCAN_TX_INT_ENABLE_BUFFER_29  (0x1u << 29) //!< Transmission Interrupt Enable for Buffer 29 flag
#define MCAN_TX_INT_ENABLE_BUFFER_30  (0x1u << 30) //!< Transmission Interrupt Enable for Buffer 30 flag
#define MCAN_TX_INT_ENABLE_BUFFER_31  (0x1u << 31) //!< Transmission Interrupt Enable for Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Buffer Cancellation Finished Interrupt Enable (Read/Write, Offset: 0xE4)
 * Each Transmit Buffer has its own Cancellation Finished Interrupt Enable bit
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXBCIE_Register
{
  uint32_t TXBCIE;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t CFIE0 : 1; //!<  0 - Cancellation Finished Interrupt Enable for Transmit Buffer 0
    uint32_t CFIE1 : 1; //!<  1 - Cancellation Finished Interrupt Enable for Transmit Buffer 1
    uint32_t CFIE2 : 1; //!<  2 - Cancellation Finished Interrupt Enable for Transmit Buffer 2
    uint32_t CFIE3 : 1; //!<  3 - Cancellation Finished Interrupt Enable for Transmit Buffer 3
    uint32_t CFIE4 : 1; //!<  4 - Cancellation Finished Interrupt Enable for Transmit Buffer 4
    uint32_t CFIE5 : 1; //!<  5 - Cancellation Finished Interrupt Enable for Transmit Buffer 5
    uint32_t CFIE6 : 1; //!<  6 - Cancellation Finished Interrupt Enable for Transmit Buffer 6
    uint32_t CFIE7 : 1; //!<  7 - Cancellation Finished Interrupt Enable for Transmit Buffer 7
    uint32_t CFIE8 : 1; //!<  8 - Cancellation Finished Interrupt Enable for Transmit Buffer 8
    uint32_t CFIE9 : 1; //!<  9 - Cancellation Finished Interrupt Enable for Transmit Buffer 9
    uint32_t CFIE10: 1; //!< 10 - Cancellation Finished Interrupt Enable for Transmit Buffer 10
    uint32_t CFIE11: 1; //!< 11 - Cancellation Finished Interrupt Enable for Transmit Buffer 11
    uint32_t CFIE12: 1; //!< 12 - Cancellation Finished Interrupt Enable for Transmit Buffer 12
    uint32_t CFIE13: 1; //!< 13 - Cancellation Finished Interrupt Enable for Transmit Buffer 13
    uint32_t CFIE14: 1; //!< 14 - Cancellation Finished Interrupt Enable for Transmit Buffer 14
    uint32_t CFIE15: 1; //!< 15 - Cancellation Finished Interrupt Enable for Transmit Buffer 15
    uint32_t CFIE16: 1; //!< 16 - Cancellation Finished Interrupt Enable for Transmit Buffer 16
    uint32_t CFIE17: 1; //!< 17 - Cancellation Finished Interrupt Enable for Transmit Buffer 17
    uint32_t CFIE18: 1; //!< 18 - Cancellation Finished Interrupt Enable for Transmit Buffer 18
    uint32_t CFIE19: 1; //!< 19 - Cancellation Finished Interrupt Enable for Transmit Buffer 19
    uint32_t CFIE20: 1; //!< 20 - Cancellation Finished Interrupt Enable for Transmit Buffer 20
    uint32_t CFIE21: 1; //!< 21 - Cancellation Finished Interrupt Enable for Transmit Buffer 21
    uint32_t CFIE22: 1; //!< 22 - Cancellation Finished Interrupt Enable for Transmit Buffer 22
    uint32_t CFIE23: 1; //!< 23 - Cancellation Finished Interrupt Enable for Transmit Buffer 23
    uint32_t CFIE24: 1; //!< 24 - Cancellation Finished Interrupt Enable for Transmit Buffer 24
    uint32_t CFIE25: 1; //!< 25 - Cancellation Finished Interrupt Enable for Transmit Buffer 25
    uint32_t CFIE26: 1; //!< 26 - Cancellation Finished Interrupt Enable for Transmit Buffer 26
    uint32_t CFIE27: 1; //!< 27 - Cancellation Finished Interrupt Enable for Transmit Buffer 27
    uint32_t CFIE28: 1; //!< 28 - Cancellation Finished Interrupt Enable for Transmit Buffer 28
    uint32_t CFIE29: 1; //!< 29 - Cancellation Finished Interrupt Enable for Transmit Buffer 29
    uint32_t CFIE30: 1; //!< 30 - Cancellation Finished Interrupt Enable for Transmit Buffer 30
    uint32_t CFIE31: 1; //!< 31 - Cancellation Finished Interrupt Enable for Transmit Buffer 31
  } Bits;
} MCAN_TXBCIE_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXBCIE_Register, 4);

#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_0   (0x1u <<  0) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 0 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_1   (0x1u <<  1) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 1 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_2   (0x1u <<  2) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 2 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_3   (0x1u <<  3) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 3 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_4   (0x1u <<  4) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 4 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_5   (0x1u <<  5) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 5 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_6   (0x1u <<  6) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 6 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_7   (0x1u <<  7) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 7 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_8   (0x1u <<  8) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 8 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_9   (0x1u <<  9) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 9 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_10  (0x1u << 10) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 10 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_11  (0x1u << 11) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 11 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_12  (0x1u << 12) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 12 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_13  (0x1u << 13) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 13 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_14  (0x1u << 14) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 14 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_15  (0x1u << 15) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 15 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_16  (0x1u << 16) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 16 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_17  (0x1u << 17) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 17 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_18  (0x1u << 18) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 18 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_19  (0x1u << 19) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 19 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_20  (0x1u << 20) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 20 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_21  (0x1u << 21) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 21 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_22  (0x1u << 22) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 22 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_23  (0x1u << 23) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 23 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_24  (0x1u << 24) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 24 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_25  (0x1u << 25) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 25 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_26  (0x1u << 26) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 26 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_27  (0x1u << 27) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 27 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_28  (0x1u << 28) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 28 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_29  (0x1u << 29) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 29 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_30  (0x1u << 30) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 30 flag
#define MCAN_CANCEL_FINISH_TX_INT_BUFFER_31  (0x1u << 31) //!< Cancellation Finished Interrupt Enable for Transmit Buffer 31 flag

//-----------------------------------------------------------------------------

/*! MCAN Transmit Event FIFO Configuration (Read/Write, Offset: 0xF0)
 * @note This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXEFC_Register
{
  uint32_t TXEFC;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t     :  2; //!<  0- 1
    uint32_t EFSA: 14; //!<  2-15 - Event FIFO Start Address. Start address of Tx Event FIFO in Message RAM. Write EFSA with the bits [15:2] of the 32-bit address
    uint32_t EFS :  6; //!< 16-21 - Event FIFO Size. The Tx Event FIFO elements are indexed from 0 to EFS-1: '0' = Tx Event FIFO disabled ; '1-32' = Number of Tx Event FIFO elements ; '>32' = Values greater than 32 are interpreted as 32
    uint32_t     :  2; //!< 22-23
    uint32_t EFWM:  6; //!< 24-29 - Event FIFO Watermark: '0' = Watermark interrupt disabled ; '1-32' = Level for Tx Event FIFO watermark interrupt (MCAN_IR.TEFW) ; '>32' = Watermark interrupt disabled
    uint32_t     :  2; //!< 30-31
  } Bits;
} MCAN_TXEFC_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXEFC_Register, 4);

#define MCAN_TXEFC_EVENT_FIFO_SA_Pos                0
#define MCAN_TXEFC_EVENT_FIFO_SA_Mask               (0xFFFCu << MCAN_TXEFC_EVENT_FIFO_SA_Pos)
#define MCAN_TXEFC_EVENT_FIFO_SA_GET(value)         ((uint32_t)(value) & MCAN_TXEFC_EVENT_FIFO_SA_Mask) //!< Get Event FIFO Start Address
#define MCAN_TXEFC_EVENT_FIFO_SA_SET(value)         ((uint32_t)(value) & MCAN_TXEFC_EVENT_FIFO_SA_Mask) //!< Set Event FIFO Start Address
#define MCAN_TXEFC_EVENT_FIFO_SIZE_Pos              16
#define MCAN_TXEFC_EVENT_FIFO_SIZE_Mask             (0x3Fu << MCAN_TXEFC_EVENT_FIFO_SIZE_Pos)
#define MCAN_TXEFC_EVENT_FIFO_SIZE_GET(value)       (((uint32_t)(value) & MCAN_TXEFC_EVENT_FIFO_SIZE_Mask) >> MCAN_TXEFC_EVENT_FIFO_SIZE_Pos) //!< Get Event FIFO Size
#define MCAN_TXEFC_EVENT_FIFO_SIZE_SET(value)       (((uint32_t)(value) << MCAN_TXEFC_EVENT_FIFO_SIZE_Pos) & MCAN_TXEFC_EVENT_FIFO_SIZE_Mask) //!< Set Event FIFO Size
#define MCAN_TXEFC_EVENT_FIFO_WATERMARK_Pos         24
#define MCAN_TXEFC_EVENT_FIFO_WATERMARK_Mask        (0x3Fu << MCAN_TXEFC_EVENT_FIFO_WATERMARK_Pos)
#define MCAN_TXEFC_EVENT_FIFO_WATERMARK_GET(value)  (((uint32_t)(value) & MCAN_TXEFC_EVENT_FIFO_WATERMARK_Mask) >> MCAN_TXEFC_EVENT_FIFO_WATERMARK_Pos) //!< Get Event FIFO Watermark
#define MCAN_TXEFC_EVENT_FIFO_WATERMARK_SET(value)  (((uint32_t)(value) << MCAN_TXEFC_EVENT_FIFO_WATERMARK_Pos) & MCAN_TXEFC_EVENT_FIFO_WATERMARK_Mask) //!< Set Event FIFO Watermark

//-----------------------------------------------------------------------------

//! MCAN Tx Event FIFO Status (Read-only, Offset: 0xF4)
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXEFS_Register
{
  uint32_t TXEFS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t EFFL: 6; //!<  0- 5 - Event FIFO Fill Level. Number of elements stored in Tx Event FIFO, range 0 to 32
    uint32_t     : 2; //!<  6- 7
    uint32_t EFGI: 5; //!<  8-12 - Event FIFO Get Index. Tx Event FIFO read index pointer, range 0 to 31
    uint32_t     : 3; //!< 13-15
    uint32_t EFPI: 5; //!< 16-20 - Event FIFO Put Index. Tx Event FIFO write index pointer, range 0 to 31
    uint32_t     : 3; //!< 21-23
    uint32_t EFF : 1; //!< 24    - Event FIFO Full: '1' = Tx Event FIFO full ; '0' = Tx Event FIFO not full
    uint32_t TEFL: 1; //!< 25    - Tx Event FIFO Element Lost: '1' = Tx Event FIFO element lost, also set after write attempt to Tx Event FIFO of size zero ; '0' = No Tx Event FIFO element lost
    uint32_t     : 6; //!< 26-31
  } Bits;
} MCAN_TXEFS_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXEFS_Register, 4);

#define MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_Pos         0
#define MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_Mask        (0x3Fu << MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_Pos)
#define MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_GET(value)  (((uint32_t)(value) & MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_Mask) >> MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_Pos) //!< Get Event FIFO Fill Level
#define MCAN_TXFQS_EVENT_FIFO_GET_INDEX_Pos          8
#define MCAN_TXFQS_EVENT_FIFO_GET_INDEX_Mask         (0x1Fu << MCAN_TXFQS_EVENT_FIFO_GET_INDEX_Pos)
#define MCAN_TXFQS_EVENT_FIFO_GET_INDEX_GET(value)   (((uint32_t)(value) & MCAN_TXFQS_EVENT_FIFO_GET_INDEX_Mask) >> MCAN_TXFQS_EVENT_FIFO_GET_INDEX_Pos) //!< Get Event FIFO Get Index
#define MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_Pos          16
#define MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_Mask         (0x1Fu << MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_Pos)
#define MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_GET(value)   (((uint32_t)(value) & MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_Mask) >> MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_Pos) //!< Get Event FIFO Put Index
#define MCAN_TXFQS_EVENT_FIFO_FULL                   (0x1u << 24) //!< Tx Event FIFO full
#define MCAN_TXFQS_EVENT_FIFO_NOT_FULL               (0x0u << 24) //!< Tx Event FIFO not full
#define MCAN_TXFQS_EVENT_FIFO_ELEMENT_LOST           (0x1u << 25) //!< Tx Event FIFO element lost
#define MCAN_TXFQS_NO_EVENT_FIFO_ELEMENT_LOST        (0x0u << 25) //!< No Tx Event FIFO element lost

//-----------------------------------------------------------------------------

/*! MCAN Tx Event FIFO Acknowledge (Read/Write, Offset: 0xF8)
 * After the processor has read an element or a sequence of elements from the Tx Event FIFO, it has to write the index of the last element read from Tx Event FIFO to EFAI.
 * This will set the Tx Event FIFO Get Index MCAN_TXEFS.EFGI to EFAI+1 and update the FIFO 0 Fill Level MCAN_TXEFS.EFFL
 */
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_TXEFA_Register
{
  uint32_t TXEFA;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t EFAI:  5; //!< 0- 4 - Event FIFO Acknowledge Index
    uint32_t     : 27; //!< 5-31
  } Bits;
} MCAN_TXEFA_Register;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_TXEFA_Register, 4);

#define MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Pos         0
#define MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Mask        (0x1Fu << MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Pos)
#define MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_GET(value)  (((uint32_t)(value) & MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Mask) >> MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Pos) //!< Get Event FIFO Acknowledge Index
#define MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_SET(value)  (((uint32_t)(value) << MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Pos) & MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_Mask) //!< Set Event FIFO Acknowledge Index

//-----------------------------------------------------------------------------

//! Available FIFO/Buffer list
typedef enum
{
  MCAN_TEF       = -1, //!< TEF - Transmit Event FIFO
  MCAN_RX_FIFO0  =  0, //!< RX FIFO 0
  MCAN_RX_FIFO1  =  1, //!< RX FIFO 1
  MCAN_RX_BUFFER =  2, //!< RX buffer
  MCAN_TX_BUFFER =  3, //!< TX buffer
  MCAN_TXQ_FIFO  =  4, //!< TXQ/FIFO - Transmit Queue or Tx FIFO
  MCAN_NO_FIFO_BUFF,   //!< No specific FIFO/Buffer
} eMCAN_FIFObuffer;

//-----------------------------------------------------------------------------

//! Transmit and Receive FIFO/Buffer status
typedef enum
{
  // Transmit FIFO/Buffer status
  MCAN_TX_FIFOBUFF_FULL         = 0x00, //!< Transmit FIFO/Buffer full
  MCAN_TX_FIFOBUFF_NOT_FULL     = 0x01, //!< Transmit FIFO/Buffer not full
  MCAN_TX_FIFOBUFF_HALF_EMPTY   = 0x02, //!< Transmit FIFO/Buffer half empty
  MCAN_TX_FIFOBUFF_EMPTY        = 0x04, //!< Transmit FIFO/Buffer empty

  // Receive FIFO/Buffer status
  MCAN_RX_FIFOBUFF_EMPTY        = 0x00, //!< Receive FIFO/Buffer empty
  MCAN_RX_FIFOBUFF_NOT_EMPTY    = 0x01, //!< Receive FIFO/Buffer not empty
  MCAN_RX_FIFOBUFF_HALF_FULL    = 0x02, //!< Receive FIFO/Buffer half full
  MCAN_RX_FIFOBUFF_FULL         = 0x04, //!< Receive FIFO/Buffer full
  MCAN_RX_FIFOBUFF_MESSAGE_LOST = 0x08, //!< Receive FIFO/Buffer message lost (overflow)

  // Debug message status
  MCAN_RX_DEBUG_MESSAGE_IDLE    = 0x10, //!< Debug Message: idle state, wait for reception of debug messages, DMA request is cleared
  MCAN_RX_DEBUG_MESSAGE_MSG_A   = 0x20, //!< Debug Message: debug message A received
  MCAN_RX_DEBUG_MESSAGE_MSG_AB  = 0x40, //!< Debug Message: debug messages A, B received
  MCAN_RX_DEBUG_MESSAGE_MSG_ABC = 0x80, //!< Debug Message: debug messages A, B, C received, DMA request is set
} eMCAN_FIFObufferstatus;

typedef eMCAN_FIFObufferstatus setMCAN_FIFObufferstatus; //! Set of Transmit and Receive FIFO/Buffer status (can be OR'ed)


//! Transmit Event FIFO status
typedef enum
{
  MCAN_TEF_FIFO_EMPTY        = 0x00, //!< TEF FIFO empty
  MCAN_TEF_FIFO_NOT_EMPTY    = 0x01, //!< TEF FIFO not empty
  MCAN_TEF_FIFO_HALF_FULL    = 0x02, //!< TEF FIFO half full
  MCAN_TEF_FIFO_FULL         = 0x04, //!< TEF FIFO full
  MCAN_TEF_FIFO_MESSAGE_LOST = 0x08, //!< TEF message lost (overflow)
} eMCAN_TEFstatus;

typedef eMCAN_TEFstatus setMCAN_TEFstatus; //! Set of Transmit Event FIFO status (can be OR'ed)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN Core Hardware Register Access
//********************************************************************************************************************
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
//! MCAN Hardware Registers
MCAN_PACKITEM
typedef union __MCAN_PACKED__ MCAN_HardReg
{
  volatile uint32_t Regs[RegMCAN_COUNT];
  volatile uint8_t  Bytes[RegMCAN_SIZE];
  struct
  {
    volatile uint32_t RegMCAN_CREL;   //!< (Offset: 0x00) Core Release Register
    volatile uint32_t RegMCAN_ENDN;   //!< (Offset: 0x04) Endian Register
    volatile uint32_t RegMCAN_CUST;   //!< (Offset: 0x08) Customer Register
    volatile uint32_t RegMCAN_DBTP;   //!< (Offset: 0x0C) Data Bit Timing and Prescaler Register
    volatile uint32_t RegMCAN_TEST;   //!< (Offset: 0x10) Test Register
    volatile uint32_t RegMCAN_RWD;    //!< (Offset: 0x14) RAM Watchdog Register
    volatile uint32_t RegMCAN_CCCR;   //!< (Offset: 0x18) CC Control Register
    volatile uint32_t RegMCAN_NBTP;   //!< (Offset: 0x1C) Nominal Bit Timing and Prescaler Register
    volatile uint32_t RegMCAN_TSCC;   //!< (Offset: 0x20) Timestamp Counter Configuration Register
    volatile uint32_t RegMCAN_TSCV;   //!< (Offset: 0x24) Timestamp Counter Value Register
    volatile uint32_t RegMCAN_TOCC;   //!< (Offset: 0x28) Timeout Counter Configuration Register
    volatile uint32_t RegMCAN_TOCV;   //!< (Offset: 0x2C) Timeout Counter Value Register
    volatile uint32_t Reserved1[4];   //!< (Offset: 0x30..0x3C)
    volatile uint32_t RegMCAN_ECR;    //!< (Offset: 0x40) Error Counter Register
    volatile uint32_t RegMCAN_PSR;    //!< (Offset: 0x44) Protocol Status Register
    volatile uint32_t RegMCAN_TDCR;   //!< (Offset: 0x48) Transmit Delay Compensation Register
    volatile uint32_t Reserved2[1];   //!< (Offset: 0x4C)
    volatile uint32_t RegMCAN_IR;     //!< (Offset: 0x50) Interrupt Register
    volatile uint32_t RegMCAN_IE;     //!< (Offset: 0x54) Interrupt Enable Register
    volatile uint32_t RegMCAN_ILS;    //!< (Offset: 0x58) Interrupt Line Select Register
    volatile uint32_t RegMCAN_ILE;    //!< (Offset: 0x5C) Interrupt Line Enable Register
    volatile uint32_t Reserved3[8];   //!< (Offset: 0x60..0x7C)
    volatile uint32_t RegMCAN_GFC;    //!< (Offset: 0x80) Global Filter Configuration Register
    volatile uint32_t RegMCAN_SIDFC;  //!< (Offset: 0x84) Standard ID Filter Configuration Register
    volatile uint32_t RegMCAN_XIDFC;  //!< (Offset: 0x88) Extended ID Filter Configuration Register
    volatile uint32_t Reserved4[1];   //!< (Offset: 0x8C)
    volatile uint32_t RegMCAN_XIDAM;  //!< (Offset: 0x90) Extended ID AND Mask Register
    volatile uint32_t RegMCAN_HPMS;   //!< (Offset: 0x94) High Priority Message Status Register
    volatile uint32_t RegMCAN_NDAT1;  //!< (Offset: 0x98) New Data 1 Register
    volatile uint32_t RegMCAN_NDAT2;  //!< (Offset: 0x9C) New Data 2 Register
    volatile uint32_t RegMCAN_RXF0C;  //!< (Offset: 0xA0) Receive FIFO 0 Configuration Register
    volatile uint32_t RegMCAN_RXF0S;  //!< (Offset: 0xA4) Receive FIFO 0 Status Register
    volatile uint32_t RegMCAN_RXF0A;  //!< (Offset: 0xA8) Receive FIFO 0 Acknowledge Register
    volatile uint32_t RegMCAN_RXBC;   //!< (Offset: 0xAC) Receive Rx Buffer Configuration Register
    volatile uint32_t RegMCAN_RXF1C;  //!< (Offset: 0xB0) Receive FIFO 1 Configuration Register
    volatile uint32_t RegMCAN_RXF1S;  //!< (Offset: 0xB4) Receive FIFO 1 Status Register
    volatile uint32_t RegMCAN_RXF1A;  //!< (Offset: 0xB8) Receive FIFO 1 Acknowledge Register
    volatile uint32_t RegMCAN_RXESC;  //!< (Offset: 0xBC) Receive Buffer / FIFO Element Size Configuration Register
    volatile uint32_t RegMCAN_TXBC;   //!< (Offset: 0xC0) Transmit Buffer Configuration Register
    volatile uint32_t RegMCAN_TXFQS;  //!< (Offset: 0xC4) Transmit FIFO/Queue Status Register
    volatile uint32_t RegMCAN_TXESC;  //!< (Offset: 0xC8) Transmit Buffer Element Size Configuration Register
    volatile uint32_t RegMCAN_TXBRP;  //!< (Offset: 0xCC) Transmit Buffer Request Pending Register
    volatile uint32_t RegMCAN_TXBAR;  //!< (Offset: 0xD0) Transmit Buffer Add Request Register
    volatile uint32_t RegMCAN_TXBCR;  //!< (Offset: 0xD4) Transmit Buffer Cancellation Request Register
    volatile uint32_t RegMCAN_TXBTO;  //!< (Offset: 0xD8) Transmit Buffer Transmission Occurred Register
    volatile uint32_t RegMCAN_TXBCF;  //!< (Offset: 0xDC) Transmit Buffer Cancellation Finished Register
    volatile uint32_t RegMCAN_TXBTIE; //!< (Offset: 0xE0) Transmit Buffer Transmission Interrupt Enable Register
    volatile uint32_t RegMCAN_TXBCIE; //!< (Offset: 0xE4) Transmit Buffer Cancellation Finished Interrupt Enable Register
    volatile uint32_t Reserved5[2];   //!< (Offset: 0xE8..0xEC)
    volatile uint32_t RegMCAN_TXEFC;  //!< (Offset: 0xF0) Transmit Event FIFO Configuration Register
    volatile uint32_t RegMCAN_TXEFS;  //!< (Offset: 0xF4) Transmit Event FIFO Status Register
    volatile uint32_t RegMCAN_TXEFA;  //!< (Offset: 0xF8) Transmit Event FIFO Acknowledge Register
    volatile uint32_t Reserved6[1];   //!< (Offset: 0xFC)
  };
} MCAN_HardReg;
MCAN_UNPACKITEM;
MCAN_CONTROL_ITEM_SIZE(MCAN_HardReg, 0x100);

#else // MCAN_EXTERNAL_CAN_CONTROLLER

//! MCAN external controller cache table indexes
typedef enum
{
  MCAN_CACHE_SIDFC, //!< Cache index for MCAN_SIDFC register
  MCAN_CACHE_XIDFC, //!< Cache index for MCAN_XIDFC register
  MCAN_CACHE_RXF0C, //!< Cache index for MCAN_RXF0C register
  MCAN_CACHE_RXBC,  //!< Cache index for MCAN_RXBC register
  MCAN_CACHE_RXF1C, //!< Cache index for MCAN_RXF1C register
  MCAN_CACHE_RXESC, //!< Cache index for MCAN_RXESC register
  MCAN_CACHE_TXBC,  //!< Cache index for MCAN_TXBC register
  MCAN_CACHE_TXESC, //!< Cache index for MCAN_TXESC register
  MCAN_CACHE_TXEFC, //!< Cache index for MCAN_TXEFC register
  MCAN_CACHE_COUNT, // KEEP LAST
} eMCAN_CacheIndex;

#endif

//-----------------------------------------------------------------------------

//! Device power states
typedef enum
{
  MCAN_DEVICE_SLEEP_NOT_CONFIGURED = 0x0, //!< Device sleep mode is not configured so the device is in normal power state
  MCAN_DEVICE_NORMAL_POWER_STATE   = 0x1, //!< Device is in normal power state
  MCAN_DEVICE_SLEEP_STATE          = 0x2, //!< Device is in sleep power state
} eMCAN_PowerStates;

//! CAN control configuration flags
typedef enum
{
  MCAN_CAN_EDGE_FILTERING_ENABLE            = 0x00, //!< Edge filtering is enabled. Two consecutive dominant tq required to detect an edge for hard synchronization
  MCAN_CAN_EDGE_FILTERING_DISABLE           = 0x01, //!< Edge filtering is disabled
  MCAN_CAN_TRANSMIT_PAUSE_ENABLE            = 0x00, //!< Transmit pause enabled. the MCAN pauses for two CAN bit times before starting the next transmission after itself has successfully transmitted a frame
  MCAN_CAN_TRANSMIT_PAUSE_DISABLE           = 0x02, //!< Transmit pause disabled
  MCAN_CAN_AUTOMATIC_RETRANSMISSION_ENABLE  = 0x00, //!< Unlimited number of retransmission attempts
  MCAN_CAN_AUTOMATIC_RETRANSMISSION_DISABLE = 0x04, //!< No retransmission attempts
  MCAN_CANFD_BITRATE_SWITCHING_ENABLE       = 0x00, //!< Bit Rate Switching is Enabled, Bit Rate Switching depends on BRS in the Transmit Message Object
  MCAN_CANFD_BITRATE_SWITCHING_DISABLE      = 0x08, //!< Bit Rate Switching is Disabled, regardless of BRS in the Transmit Message Object
  MCAN_CAN_PROTOCOL_EXCEPT_HANDLING_ENABLE  = 0x00, //!< If a Protocol Exception is detected, the CAN FD Controller Module will enter Bus Integrating state. A recessive "res bit" following a recessive FDF bit is called a Protocol Exception
  MCAN_CAN_PROTOCOL_EXCEPT_HANDLING_DISABLE = 0x10, //!< Protocol Exception is treated as a Form Error. A recessive "res bit" following a recessive FDF bit is called a Protocol Exception
  MCAN_CANFD_USE_NONISO_CRC                 = 0x00, //!< Do NOT include Stuff Bit Count in CRC Field and use CRC Initialization Vector with all zeros
  MCAN_CANFD_USE_ISO_CRC                    = 0x20, //!< Include Stuff Bit Count in CRC Field and use Non-Zero CRC Initialization Vector according to ISO 11898-1:2015
  MCAN_CAN_WIDE_MESSAGE_MARKER_8BIT         = 0x00, //!< 8-bit Message Marker used
  MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT        = 0x40, //!< 16-bit Message Marker used, replacing 16-bit timestamps in Tx Event FIFO
  MCAN_CAN_INTERNAL_TIMESTAMPING            = 0x00, //!< Internal time stamping
  MCAN_CAN_EXTERNAL_TIMESTAMPING_BY_TSU     = 0x80, //!< External time stamping by TSU, the 16-bit Wide Message Markers are also enabled regardless of MCAN_CAN_WIDE_MESSAGE_MARKER_8BIT
} eMCAN_CANCtrlFlags;

typedef eMCAN_CANCtrlFlags setMCAN_CANCtrlFlags; //! Set of CAN control configuration flags (can be OR'ed)

#define MCAN_WIDE_MESSAGE_MARKER  ( MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT | MCAN_CAN_EXTERNAL_TIMESTAMPING_BY_TSU ) //!< Wide message marker flags mask

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// BitTime Commons
//********************************************************************************************************************

#define MCAN_NOMBITRATE_MIN   (   125000u ) //!< Min Nominal bitrate
#define MCAN_NOMBITRATE_MAX   (  1000000u ) //!< Max Nominal bitrate
#define MCAN_DATABITRATE_MIN  (   500000u ) //!< Min Data bitrate
#define MCAN_DATABITRATE_MAX  ( 10000000u ) //!< Max Data bitrate

//-----------------------------------------------------------------------------

// Limits Bit Rate configuration range for MCAN
#define MCAN_tTXDtRXD_MAX  ( 255 ) //!< tTXD-RXD is the propagation delay of the transceiver, a maximum 255ns according to ISO 11898-1:2015
#define MCAN_tBUS_CONV     (   5 ) //!< TBUS is the delay on the CAN bus, which is approximately 5ns/m

#define MCAN_NBRP_MIN      (   1 ) //!< Nominal: Min BRP
#define MCAN_NBRP_MAX      ( 512 ) //!< Nominal: Max BRP
#define MCAN_NSYNC         (   1 ) //!< Nominal: SYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define MCAN_NTSEG1_MIN    (   2 ) //!< Nominal: Min TSEG1
#define MCAN_NTSEG1_MAX    ( 256 ) //!< Nominal: Max TSEG1
#define MCAN_NTSEG2_MIN    (   1 ) //!< Nominal: Min TSEG2
#define MCAN_NTSEG2_MAX    ( 128 ) //!< Nominal: Max TSEG2
#define MCAN_NSJW_MIN      (   1 ) //!< Nominal: Min SJW
#define MCAN_NSJW_MAX      ( 128 ) //!< Nominal: Max SJW
#define MCAN_NTQBIT_MIN    ( MCAN_NSYNC + MCAN_NTSEG1_MIN + MCAN_NTSEG2_MIN ) //!< Nominal: Min TQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define MCAN_NTQBIT_MAX    ( MCAN_NTSEG1_MAX + MCAN_NTSEG2_MAX + 1 )          //!< Nominal: Max TQ per Bit (385-bits)

#define MCAN_DBRP_MIN      (   1 ) //!< Data: Min BRP
#define MCAN_DBRP_MAX      (  32 ) //!< Data: Max BRP
#define MCAN_DSYNC         (   1 ) //!< Data: SYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define MCAN_DTSEG1_MIN    (   2 ) //!< Data: Min TSEG1
#define MCAN_DTSEG1_MAX    (  32 ) //!< Data: Max TSEG1
#define MCAN_DTSEG2_MIN    (   1 ) //!< Data: Min TSEG2
#define MCAN_DTSEG2_MAX    (  16 ) //!< Data: Max TSEG2
#define MCAN_DSJW_MIN      (   1 ) //!< Data: Min SJW
#define MCAN_DSJW_MAX      (   8 ) //!< Data: Max SJW
#define MCAN_DTQBIT_MIN    ( MCAN_NSYNC + MCAN_NTSEG1_MIN + MCAN_NTSEG2_MIN ) //!< Data: Min TQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define MCAN_DTQBIT_MAX    ( MCAN_NTSEG1_MAX + MCAN_NTSEG2_MAX + 1 )          //!< Data: Max TQ per Bit (49-bits)

#define MCAN_TDCO_MIN      (   0 ) //!< Min TDCO
#define MCAN_TDCO_MAX      ( 127 ) //!< Max TDCO

#define MCAN_NFDBRP_MIN    ( MCAN_NBRP_MIN ) //!< Nominal Only: Min BRP (Identical to NBRP min)
#define MCAN_NFDBRP_MAX    ( MCAN_NBRP_MAX ) //!< Nominal Only: Max BRP (Identical to NBRP max)
#define MCAN_NFDSYNC       (   1 ) //!< Nominal Only: SYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define MCAN_NFDPRSEG_MIN  (   1 ) //!< Nominal Only: Min PRSEG
#define MCAN_NFDPRSEG_MAX  (   8 ) //!< Nominal Only: Max PRSEG
#define MCAN_NFDTSEG1_MIN  (   1 ) //!< Nominal Only: Min TSEG1
#define MCAN_NFDTSEG1_MAX  (   8 ) //!< Nominal Only: Max TSEG1
#define MCAN_NFDTSEG2_MIN  (   1 ) //!< Nominal Only: Min TSEG2
#define MCAN_NFDTSEG2_MAX  (   8 ) //!< Nominal Only: Max TSEG2
#define MCAN_NFDSJW_MIN    (   1 ) //!< Nominal Only: Min SJW
#define MCAN_NFDSJW_MAX    (   4 ) //!< Nominal Only: Max SJW
#define MCAN_NFDTQBIT_MIN  ( MCAN_NFDSYNC + MCAN_NFDTSEG1_MIN + MCAN_NFDTSEG2_MIN ) //!< Nominal: Min TQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define MCAN_NFDTQBIT_MAX  ( MCAN_NFDTSEG1_MAX + MCAN_NFDTSEG2_MAX + 1 )            //!< Nominal: Max TQ per Bit (29-bits)

#define MCAN_MAX_OSC_TOLERANCE  ( 1.58f ) //!< The CAN specification indicates that the worst case oscillator tolerance is 1.58% and is only suitable for low bit rates (125kb/s or less)
#define MCAN_UINT_MAX_OSC_TOL   ( (uint32_t)(MCAN_MAX_OSC_TOLERANCE * 100.0f) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// CAN FIFO/Buffers Commons
//********************************************************************************************************************

// FIFO/Buffers definitions
#define MCAN_TEF_MAX        ( 1 ) //!< 1 TEF maximum
#define MCAN_TXQ_FIFO_MAX   ( 1 ) //!< 1 TXQ/FIFO maximum
#define MCAN_TX_BUFFER_MAX  ( 1 ) //!< 1 Tx buffer maximum
#define MCAN_FIFO_MAX       ( 2 ) //!< 2 Rx FIFOs maximum
#define MCAN_RX_BUFFER_MAX  ( 1 ) //!< 1 Rx buffer maximum
#define MCAN_FIFO_CONF_MAX  ( MCAN_TEF_MAX + MCAN_TXQ_FIFO_MAX + MCAN_TX_BUFFER_MAX + MCAN_FIFO_MAX + MCAN_RX_BUFFER_MAX ) //!< Maximum 6 FIFO configurable (TEF + TXQ/FIFO + Tx buffer + 2 Rx FIFO + Rx buffer)
#define MCAN_TX_FIFO_MAX    ( MCAN_TXQ_FIFO_MAX + MCAN_TX_BUFFER_MAX )            //!< Maximum 2 transmit FIFO (TXQ/FIFO + Tx buffer)
#define MCAN_RX_FIFO_MAX    ( MCAN_TEF_MAX + MCAN_FIFO_MAX + MCAN_RX_BUFFER_MAX ) //!< Maximum 4 receive FIFO (TEF + 2 FIFO + Rx buffer)

#define MCAN_TX_EVENT_FIFO_SIZE_MAX  ( 32 ) //!< 32 elements max for the Tx Event FIFO
#define MCAN_TX_BUFFER_SIZE_MAX      ( 32 ) //!< 32 elements max for the Tx Buffer
#define MCAN_TX_FIFO_TXQ_SIZE_MAX    ( 32 ) //!< 32 elements max for the Tx Queue/FIFO
#define MCAN_TX_ELEMENTS_SIZE_MAX    ( 32 ) //!< 32 elements max for the Tx Buffer + Tx Queue/FIFO
#define MCAN_RX_FIFO0_SIZE_MAX       ( 64 ) //!< 64 elements max for the Rx FIFO0
#define MCAN_RX_FIFO1_SIZE_MAX       ( 64 ) //!< 64 elements max for the Rx FIFO1
#define MCAN_RX_BUFFER_SIZE_MAX      ( 64 ) //!< 64 elements max for the Rx Buffer

//-----------------------------------------------------------------------------

//! FIFO/Buffers configuration flags
typedef enum
{
  //--- Tx FIFO/Buffer ---
  MCAN_TX_FIFO_MODE           = 0x00, //!< Tx FIFO/Queue in Tx FIFO mode
  MCAN_TXQ_MODE               = 0x01, //!< Tx FIFO/Queue in Tx Queue mode
  //--- Rx FINO/Buffer ---
  MCAN_RX_FIFO_BLOCKING_MODE  = 0x00, //!< Rx FIFO/Buffer Blocking mode
  MCAN_RX_FIFO_OVERWRITE_MODE = 0x01, //!< Rx FIFO/Buffer Overwrite mode
} eMCAN_FIFOCtrlFlags;

//! FIFO/Buffers interruption flags
typedef enum
{
  MCAN_FIFO_NO_INTERRUPT_FLAGS            = 0x00, //!< Set no interrupt flags
  MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT       = 0x01, //!< Receive FIFO New Message Interrupt Enable
  MCAN_FIFO_RECEIVE_WATERMARK_REACHED_INT = 0x02, //!< Receive FIFO Watermark Reached Interrupt Enable
  MCAN_FIFO_RECEIVE_FULL_INT              = 0x04, //!< Receive FIFO Full Interrupt Enable
  MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT      = 0x08, //!< Receive FIFO Message Lost Interrupt Enable
  MCAN_FIFO_TRANSMIT_FIFO_EMPTY_INT       = 0x04, //!< Tx FIFO Empty Interrupt Enable
  MCAN_FIFO_EVENT_NEW_MESSAGE_INT         = 0x01, //!< Tx Event FIFO New Entry Interrupt Enable
  MCAN_FIFO_EVENT_WATERMARK_REACHED_INT   = 0x02, //!< Tx Event FIFO Watermark Reached Interrupt Enable
  MCAN_FIFO_EVENT_FULL_INT                = 0x04, //!< Tx Event FIFO Full Interrupt Enable
  MCAN_FIFO_EVENT_LOST_MESSAGE_INT        = 0x08, //!< Tx Event FIFO Element Lost Interrupt Enable
} eMCAN_FIFOIntFlags;

//-----------------------------------------------------------------------------

//! FIFO/Buffers configuration structure
typedef struct MCAN_FIFObuff
{
  eMCAN_FIFObuffer Name;             //!< FIFO/buffer name (TXQ, FIFO, Buffer, or TEF)

  //--- FIFO Size ---
  uint8_t Size;                      //!< FIFO/buffer Message size deep (1 to 32/64 depending on FIFO/Buffer). Set to 0 to disable
  eMCAN_PayloadSize Payload;         //!< Message Payload Size (8, 12, 16, 20, 24, 32, 48 or 64 bytes)

  //--- Configuration ---
  eMCAN_FIFOCtrlFlags ControlFlags;  //!< FIFO/buffer control flags to configure the FIFO
  eMCAN_FIFOIntFlags InterruptFlags; //!< FIFO/buffer interrupt flags to configure interrupts of the FIFO
  uint8_t WatermarkLevel;            //!< Receive/TxEvent FIFO watermark level. Set to 0 or superior to FIFO/TxEvent size to disable
} MCAN_FIFObuff;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN Filter Commons
//********************************************************************************************************************

//! Filter match type
typedef enum
{
  MCAN_FILTER_MATCH_ONLY_SID = 0x0, //!< Match only messages with standard identifier (+SID11 in FD mode if configured and available in the driver)
  MCAN_FILTER_MATCH_ONLY_EID = 0x1, //!< Match only messages with extended identifier
  MCAN_FILTER_MATCH_SID_EID  = 0x2, //!< Match both standard and extended message frames
} eMCAN_FilterMatch;

//! Filter descriptor type
typedef enum
{
  MCAN_FILTER_MATCH_ID_RANGE      = 0x0, //!< ID range filter (from ID1 to ID2 included)
  MCAN_FILTER_MATCH_DUAL_ID       = 0x1, //!< Dual ID filter (ID + ID)
  MCAN_FILTER_MATCH_ID_MASK       = 0x2, //!< Classic filter (ID + Mask)
  MCAN_FILTER_MATCH_ID_RANGE_MASK = 0x3, //!< ID range filter (from ID1 to ID2 included) with an external mask
} eMCAN_FilterDesc;

//! Filter configuration type
typedef enum
{
  MCAN_FILTER_NO_CONFIG        = 0x0, //!< No specific configuration
  MCAN_FILTER_REJECT_ID        = 0x1, //!< Reject ID if filter matches
  MCAN_FILTER_SET_PRIORITY     = 0x2, //!< Set priority if filter matches
  MCAN_FILTER_AS_DEBUG_MESSAGE = 0x3, //!< Store as debug message
} eMCAN_FilterConfig;

//-----------------------------------------------------------------------------

// Filters definitions
#define MCAN_SID_FILTERS_MAX  ( 128 ) //!< 128 Standard filters elements maximum
#define MCAN_EID_FILTERS_MAX  (  64 ) //!< 64 Extended filters elements maximum
#define MCAN_FILTERS_MAX      ( MCAN_SID_FILTERS_MAX + MCAN_EID_FILTERS_MAX ) //!< 128+64 filters elements maximum

//-----------------------------------------------------------------------------

#define MCAN_FILTER_ACCEPT_ALL_MESSAGES  ( 0x00000000u ) //!< Indicate that the filter will accept all messages

//-----------------------------------------------------------------------------

//! MCAN filter with ID to Rx buffer structure
typedef struct MCAN_FilterID
{
  uint32_t AcceptanceID;   //!< Message Filter Acceptance SID+(SID11 in FD mode if supported and activated)+EID
  uint32_t BufferPosition; //!< Position in the Rx Buffer
} MCAN_FilterID;

//! MCAN filter with ID for debug to Rx buffer structure
typedef struct MCAN_FilterDebugID
{
  uint32_t AcceptanceID;   //!< Message Filter Acceptance SID+(SID11 in FD mode if supported and activated)+EID
  uint16_t DebugMessage;   //!< Debug message to store in the Rx Buffer
  uint16_t BufferPosition; //!< Position in the Rx Buffer
} MCAN_FilterDebugID;

//! MCAN classic filter with ID and Mask structure
typedef struct MCAN_FilterIDmask
{
  uint32_t AcceptanceID;   //!< Message Filter Acceptance SID+(SID11 in FD mode if supported and activated)+EID
  uint32_t AcceptanceMask; //!< Message Filter Mask SID+(SID11 in FD mode if supported and activated)+EID (corresponding bits to AcceptanceID: '1': bit to filter ; '0' bit that do not care)
} MCAN_FilterIDmask;

//! MCAN filter with dual ID structure
typedef struct MCAN_FilterDualID
{
  uint32_t AcceptanceID1; //!< Message Filter Acceptance 1 SID+(SID11 in FD mode if supported and activated)+EID
  uint32_t AcceptanceID2; //!< Message Filter Acceptance 2 SID+(SID11 in FD mode if supported and activated)+EID
} MCAN_FilterDualID;

//! MCAN filter with a range ID structure
typedef struct MCAN_FilterRangeID
{
  uint32_t MinID; //!< Message Filter Minimum Acceptance SID+(SID11 in FD mode if supported and activated)+EID
  uint32_t MaxID; //!< Message Filter Maximum Acceptance SID+(SID11 in FD mode if supported and activated)+EID
} MCAN_FilterRangeID;

//-----------------------------------------------------------------------------

//! MCAN common Filter configuration structure
typedef struct MCAN_Filter
{
  //--- Configuration ---
  uint16_t Filter;               //!< Filter to configure
  bool EnableFilter;             //!< Enable the filter
  eMCAN_FilterMatch Match;       //!< Filter match type of the frame (SID or EID + SID and EID if supported)
  eMCAN_FilterDesc Type;         //!< Filter descriptor type (Classic, dual ID, or ID range)
  eMCAN_FilterConfig Config;     //!< Filter configuration
  eMCAN_FIFObuffer PointTo;      //!< Message matching filter is stored in pointed FIFO/Buffer name (RxBuffer, FIFO0, FIFO1, etc.)

  //--- Message Filter ---
  bool ExtendedID;               //!< The message filter is an extended ID
  union
  {
    MCAN_FilterID IDbuffer;      //!< Fill only if MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_ID_MASK and MCAN_FilterConfig.PointTo a Rx Buffer
    MCAN_FilterDebugID DebugID;  //!< Fill only if MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_ID_MASK, MCAN_FilterConfig.Config = MCAN_FILTER_AS_DEBUG_MESSAGE, and MCAN_FilterConfig.PointTo a Rx Buffer
    MCAN_FilterIDmask IDandMask; //!< Fill only if MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_ID_MASK
    MCAN_FilterDualID DualID;    //!< Fill only if MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_DUAL_ID
    MCAN_FilterRangeID RangeID;  //!< Fill only if MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_ID_RANGE or MCAN_FilterConfig.Type == MCAN_FILTER_MATCH_ID_RANGE_MASK
  };
} MCAN_Filter;

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCAN_CORE_H_INC */