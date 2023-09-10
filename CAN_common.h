/*!*****************************************************************************
 * @file    CAN_common.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    22/05/2021
 * @brief   CAN bus common configuration and structs
 *
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
#ifndef CAN_COMMON_H_INC
#define CAN_COMMON_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
//#define CAN_AUTOMATIC_BITRATE_CALCULUS //!< If uncomment, all CAN controller's drivers will user their automatic bitrate calculus system, else only the bit timings will be used
//-----------------------------------------------------------------------------
#include "ErrorsDef.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#  define CAN_MEMBER(name)
#  define __CAN_PACKED__
#  define CAN_PACKITEM      __pragma(pack(push, 1))
#  define CAN_UNPACKITEM    __pragma(pack(pop))
#else
#  define CAN_MEMBER(name)  .name =
#  define __CAN_PACKED__    __attribute__((packed))
#  define CAN_PACKITEM
#  define CAN_UNPACKITEM
#endif
//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define CAN_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// CAN BitTime configuration Commons
//********************************************************************************************************************

#define CAN_NO_CANFD  ( 0u ) //!< This value specify that the driver will not calculate CAN-FD bitrate

//-----------------------------------------------------------------------------

//! CAN2.0A/CAN2.0B bus configuration
typedef struct CAN_CAN20busConfig
{
  uint32_t DesiredBitrate;   //!< Desired CAN2.0A/CAN2.0B bitrate in bit/s
  uint16_t BusMeters;        //!< Bus length in meter
  uint16_t TransceiverDelay; //!< Worst transceiver delay at worst temperature of the entire bus (or transceiver of the board) in ns
} CAN_CAN20busConfig;

//-----------------------------------------------------------------------------

//! CANFD bus configuration
typedef struct CAN_CANFDbusConfig
{
  uint32_t DesiredNominalBitrate; //!< Desired nominal CANFD bitrate in bit/s
  uint32_t DesiredDataBitrate;    //!< Desired data CANFD bitrate in bit/s (if CAN2.0 only mode, set to value CAN_NO_CANFD)
  uint16_t BusMeters;             //!< Bus length in meter
  uint16_t TransceiverDelay;      //!< Worst transceiver delay at worst temperature of the entire bus (or transceiver of the board) in ns
  uint8_t  NominalSamplePoint;    //!< Nominal sample point in percent
  uint8_t  DataSamplePoint;       //!< Data sample point in percent
} CAN_CANFDbusConfig;

//-----------------------------------------------------------------------------

//! Nominal Phase Seg2 BitTime Length mode
typedef enum
{
  CAN_PS2_BLT_PS1_IPT, //!< Length of PS2 is the greater of PS1 and IPT
  CAN_PS2_BLT_PHSEG2,  //!< Length of PS2 is determined by the PHSEG2
} eCAN_PS2mode;

//-----------------------------------------------------------------------------

//! Transmitter Delay Compensation Mode; Secondary Sample Point (SSP) for the TDCMOD
typedef enum
{
  CAN_TDC_DISABLED    = 0b000, //!< TDC Disabled
  CAN_TDC_MANUAL_MODE = 0b001, //!< Manual; Don’t measure, use TDCV + TDCO from register
  CAN_TDC_AUTO_MODE   = 0b010, //!< Auto; measure delay and add TDCO
} eCAN_TDCmode;

//-----------------------------------------------------------------------------

#define CAN_SAE_CLASS_A_SPEED_MAX    10000 //!< SAE class A for low bus speed (Convenience features: truk release, electric mirror adjustement)
#define CAN_SAE_CLASS_B_SPEED_MAX   125000 //!< SAE class B for medium bus speed (General information transfer: instruments, power windows)
#define CAN_SAE_CLASS_C_SPEED_MAX  1000000 //!< SAE class C for high bus speed (Real time control: Power train, vehicle dynamics)
#define CAN_SAE_CLASS_D_SPEED_MIN  1000000 //!< SAE class D for very high bus speed (Multimedia application, hard real time critical functions: Internet, X-by-Wire applications)

//-----------------------------------------------------------------------------

//! Bit Time statistics structure for CAN speed
typedef struct CAN_BitTimeStats
{
  uint32_t NominalBitrate; //!< This is the actual nominal bitrate in bit/s
  uint32_t DataBitrate;    //!< This is the actual data bitrate in bit/s
  uint32_t MaxBusLength;   //!< This is the maximum bus length in meters according to parameters
  uint32_t NSamplePoint;   //!< Nominal Sample Point in percent
  uint32_t DSamplePoint;   //!< Data Sample Point in percent
  uint32_t OscTolC1;       //!< Condition 1 for the maximum tolerance of the oscillator in 100th of percent
  uint32_t OscTolC2;       //!< Condition 2 for the maximum tolerance of the oscillator in 100th of percent
  uint32_t OscTolC3;       //!< Condition 3 for the maximum tolerance of the oscillator in 100th of percent
  uint32_t OscTolC4;       //!< Condition 4 for the maximum tolerance of the oscillator in 100th of percent
  uint32_t OscTolC5;       //!< Condition 5 for the maximum tolerance of the oscillator in 100th of percent
  uint32_t OscTolerance;   //!< Oscillator Tolerance, minimum of conditions 1-5 in 100th of percent
} CAN_BitTimeStats;

//-----------------------------------------------------------------------------

//! Bit Time Configuration structure for CAN speed
typedef struct CAN_BitTimeConfig
{
  //--- Nominal Bit Times ---
  uint16_t NBRP;           //!< Nominal Baudrate Prescaler bits; TQ = NBRP/Fperipheral
  uint16_t NPRSEG;         //!< Nominal Propagation Segment bits
  uint16_t NTSEG1;         //!< Nominal Time Segment 1 bits
  uint16_t NTSEG2;         //!< Nominal Time Segment 2 bits
  uint16_t NSJW;           //!< Nominal Synchronization Jump Width bits
  //--- Data Bit Times ---
  uint16_t DBRP;           //!< Data Baudrate Prescaler bits; TQ = DBRP/Fperipheral
  uint16_t DPRSEG;         //!< Data Propagation Segment bits
  uint16_t DTSEG1;         //!< Data Time Segment 1 bits
  uint16_t DTSEG2;         //!< Data Time Segment 2 bits
  uint16_t DSJW;           //!< Data Synchronization Jump Width bits
  //--- Transmitter Delay Compensation ---
  int16_t  TDCO;           //!< Transmitter Delay Compensation Offset
  uint16_t TDCV;           //!< Transmitter Delay Compensation Value
  uint8_t SAMPL;           //!< Sampling count
  eCAN_TDCmode TDCmode;    //!< Transmitter Delay Compensation Mode
  eCAN_PS2mode PS2mode;    //!< Nominal Phase Seg2 BitTime Length Mode
  //--- Result Statistics ---
  CAN_BitTimeStats* Stats; //!< Point to a stat structure (set to NULL if no statistics are necessary)
  bool EdgeFilter;         //!< Defines Edge Filtering enable
  bool CAN20only;          //!< The bit time config is only for CAN2.0A and CAN2.0B
  bool Valid;              //!< Indicate the validness of the bit time configuration
} CAN_BitTimeConfig;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// CAN RAM configuration Commons
//********************************************************************************************************************

//! CAN RAM element type enumerator
typedef enum
{
  CAN_RAM_OBJECTS_ARE_FILTERS_SID,   //!< RAM Objects type are 11-bit filter
  CAN_RAM_OBJECTS_ARE_FILTERS_EID,   //!< RAM Objects type are 29-bit filter
  CAN_RAM_OBJECTS_ARE_RX_FIFO_0,     //!< RAM Objects type are Rx FIFO 0
  CAN_RAM_OBJECTS_ARE_RX_FIFO_1,     //!< RAM Objects type are Rx FIFO 1
  CAN_RAM_OBJECTS_ARE_RX_BUFFER,     //!< RAM Objects type are Rx buffer
  CAN_RAM_OBJECTS_ARE_TX_EVENT_FIFO, //!< RAM Objects type are Tx event FIFO
  CAN_RAM_OBJECTS_ARE_TX_QUEUE,      //!< RAM Objects type are Tx queue
  CAN_RAM_OBJECTS_ARE_TX_BUFFER,     //!< RAM Objects type are Tx buffer
} eCAN_RAMelementType;

//-----------------------------------------------------------------------------

#define CAN_RAM_OBJECTS_COUNT_FOR_MCAN  ( 8 )

//-----------------------------------------------------------------------------

//! CAN common RAM configuration structure
typedef struct CAN_RAMconfig
{
  uint16_t RAMStartAddress;        //!< RAM Start Address of the RAM range
  uint16_t ByteSize;               //!< Total number of bytes that RAM range takes in RAM
  uint8_t BytePerObject;           //!< How many bytes in an object of the RAM range
  eCAN_RAMelementType ObjectsType; //!< Type of objects in this RAM range
} CAN_RAMconfig;

#define CAN_RAM_CONFIG_OBJECT_SET(obj,type,sa,count,bpo)  do { (obj).RAMStartAddress = (sa); (obj).ByteSize = (count) * (bpo); (obj).BytePerObject = (bpo); (obj).ObjectsType = (type); } while(0)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// CAN Message Commons
//********************************************************************************************************************

#define CAN_SID_Size  11
#define CAN_SID_Mask  ((1 << CAN_SID_Size) - 1)
#define CAN_EID_Size  18
#define CAN_EID_Mask  ((1 << CAN_EID_Size) - 1)

//-----------------------------------------------------------------------------

//! Control flags of CAN message
typedef enum
{
  CAN_NO_MESSAGE_CTRL_FLAGS       = 0x000, //!< No Message Control Flags
  CAN_CAN20_FRAME                 = 0x000, //!< Indicate that the frame is a CAN2.0A/B
  CAN_CANFD_FRAME                 = 0x001, //!< Indicate that the frame is a CAN-FD
  CAN_NO_SWITCH_BITRATE           = 0x000, //!< The data bitrate is not switched (only CAN-FD frame)
  CAN_SWITCH_BITRATE              = 0x002, //!< The data bitrate is switched (only CAN-FD frame)
  CAN_DATA_FRAME                  = 0x000, //!< The frame is a data frame
  CAN_REMOTE_TRANSMISSION_REQUEST = 0x004, //!< The frame is a Remote Transmission Request; not used in CAN FD
  CAN_STANDARD_MESSAGE_ID         = 0x000, //!< Clear the Identifier Extension Flag that set the standard ID format
  CAN_EXTENDED_MESSAGE_ID         = 0x008, //!< Set the Identifier Extension Flag that set the extended ID format
  CAN_TRANSMIT_ERROR_PASSIVE      = 0x010, //!< Error Status Indicator: In CAN to CAN gateway mode, the transmitted ESI flag is a "logical OR" of ESI and error passive state of the CAN controller; In normal mode ESI indicates the error status
  CAN_STORE_TX_EVENT              = 0x020, //!< Store Tx events in TEF
  CAN_USE_EXTERNAL_TIMESTAMP      = 0x040, //!< Use external timestamp unit capture
  CAN_RX_NON_MATCHING_FRAME       = 0x080, //!< Received frame did not match any Rx filter element
  CAN_TEF_CANCELLED_FRAME         = 0x100, //!< TEF report a transmission in spite of cancellation (always set for transmissions in DAR mode)
  CAN_R0_RECESSIVE                = 0x200, //!< R0 bit is set or received at '1'
  CAN_R1_RECESSIVE                = 0x400, //!< R1 bit is set or received at '1'
} eCAN_MessageCtrlFlags;

typedef eCAN_MessageCtrlFlags setCAN_MessageCtrlFlags; //! Set of Control flags of CAN message (can be OR'ed)

#define eCAN_SET_CONTROL_FLAG(var,flag)  var = (eCAN_MessageCtrlFlags)(var | (flag))

//-----------------------------------------------------------------------------

//! Data Length Size for the CAN message
typedef enum
{
  CAN_DLC_0BYTE   = 0b0000, //!< The DLC is  0 data byte
  CAN_DLC_1BYTE   = 0b0001, //!< The DLC is  1 data byte
  CAN_DLC_2BYTE   = 0b0010, //!< The DLC is  2 data bytes
  CAN_DLC_3BYTE   = 0b0011, //!< The DLC is  3 data bytes
  CAN_DLC_4BYTE   = 0b0100, //!< The DLC is  4 data bytes
  CAN_DLC_5BYTE   = 0b0101, //!< The DLC is  5 data bytes
  CAN_DLC_6BYTE   = 0b0110, //!< The DLC is  6 data bytes
  CAN_DLC_7BYTE   = 0b0111, //!< The DLC is  7 data bytes
  CAN_DLC_8BYTE   = 0b1000, //!< The DLC is  8 data bytes
  CAN_DLC_12BYTE  = 0b1001, //!< The DLC is 12 data bytes
  CAN_DLC_16BYTE  = 0b1010, //!< The DLC is 16 data bytes
  CAN_DLC_20BYTE  = 0b1011, //!< The DLC is 20 data bytes
  CAN_DLC_24BYTE  = 0b1100, //!< The DLC is 24 data bytes
  CAN_DLC_32BYTE  = 0b1101, //!< The DLC is 32 data bytes
  CAN_DLC_48BYTE  = 0b1110, //!< The DLC is 48 data bytes
  CAN_DLC_64BYTE  = 0b1111, //!< The DLC is 64 data bytes
  CAN_DLC_COUNT,            // Keep last
  CAN_PAYLOAD_MIN =  8,
  CAN_PAYLOAD_MAX = 64,
} eCAN_DataLength;

//-----------------------------------------------------------------------------

//! CAN CAN message configuration structure
typedef struct CAN_CANMessage
{
  uint32_t MessageID;                   //!< Contain the message ID to send
  uint32_t MessageMarker;               //!< This is the context of the CAN message. This sequence will be copied in the TEF to trace the message sent
  setCAN_MessageCtrlFlags ControlFlags; //!< Contain the CAN controls flags
  eCAN_DataLength DLC;                  //!< Indicate how many bytes in the payload data will be sent or how many bytes in the payload data is received
  uint8_t* PayloadData;                 //!< Pointer to the payload data that will be sent. PayloadData array should be at least the same size as indicate by the DLC
} CAN_CANMessage;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// CAN Data Structure Commons
//********************************************************************************************************************

/*! @brief Data structure specification
 * @details When sending/receiving data to/from CAN controller, the data specification that send/receive messages will be defined
 * It is coded like this .....dtt_nnnnnnnn, where:
 * - 'n' is the index of the data type structure
 * - 't' is the type of data structure #eCAN_DataStructs
 * - 'd' is the direction of the data structure, #CAN_DATA_STRUCT_TRANSMIT for transmission and #CAN_DATA_STRUCT_RECEIVE for reception
 * @ex For a TXQ, the value will be 0b00000010_00000000
 * @ex For a receive FIFO22, the value will be 0b00000100_00010110
 * @ex For a receive Buffer at index 62, the value will be 0b00000101_00111110
 */

//-----------------------------------------------------------------------------

//! Data structures directions enumerator
typedef enum
{
  CAN_DATA_STRUCT_TRANSMIT = 0, //!< CAN data structures direction is transmit
  CAN_DATA_STRUCT_RECEIVE  = 1, //!< CAN data structures direction is receive
} eCAN_DataStructsDirection;

#define CAN_DATA_STRUCT_DIR_Pos         10
#define CAN_DATA_STRUCT_DIR_Mask        (0x1u << CAN_DATA_STRUCT_DIR_Pos)
#define CAN_DATA_STRUCT_DIR_GET(value)  (eCAN_DataStructsDirection)(((uint32_t)(value) & CAN_DATA_STRUCT_DIR_Mask) >> CAN_DATA_STRUCT_DIR_Pos) //!< Get data structure direction
#define CAN_DATA_STRUCT_DIR_SET(value)  (((uint32_t)(value) << CAN_DATA_STRUCT_DIR_Pos) & CAN_DATA_STRUCT_DIR_Mask) //!< Set data structure direction

//-----------------------------------------------------------------------------

//! Data structures types enumerator
typedef enum
{
  CAN_FIFO_TYPE   = 0, //!< CAN data structures is FIFO type
  CAN_BUFFER_TYPE = 1, //!< CAN data structures is Buffer type
  CAN_QUEUE_TYPE  = 2, //!< CAN data structures is Queue type
  CAN_EVENT_TYPE  = 3, //!< CAN data structures is Event FIFO type
} eCAN_DataStructsType;

#define CAN_DATA_STRUCT_TYPE_Pos         8
#define CAN_DATA_STRUCT_TYPE_Mask        (0x3u << CAN_DATA_STRUCT_TYPE_Pos)
#define CAN_DATA_STRUCT_TYPE_GET(value)  (eCAN_DataStructsType)(((uint32_t)(value) & CAN_DATA_STRUCT_TYPE_Mask) >> CAN_DATA_STRUCT_TYPE_Pos) //!< Get data structure type
#define CAN_DATA_STRUCT_TYPE_SET(value)  (((uint32_t)(value) << CAN_DATA_STRUCT_TYPE_Pos) & CAN_DATA_STRUCT_TYPE_Mask) //!< Set data structure type

//-----------------------------------------------------------------------------

#define CAN_DATA_STRUCT_INDEX_Pos         0
#define CAN_DATA_STRUCT_INDEX_Mask        (0xFFu << CAN_DATA_STRUCT_INDEX_Pos)
#define CAN_DATA_STRUCT_INDEX_GET(value)  (eCAN_DataStructsType)(((uint32_t)(value) & CAN_DATA_STRUCT_INDEX_Mask) >> CAN_DATA_STRUCT_INDEX_Pos) //!< Get data structure index
#define CAN_DATA_STRUCT_INDEX_SET(value)  (((uint32_t)(value) << CAN_DATA_STRUCT_INDEX_Pos) & CAN_DATA_STRUCT_INDEX_Mask) //!< Set data structure index

//-----------------------------------------------------------------------------

#define CAN_TEF            ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_TRANSMIT) | CAN_DATA_STRUCT_TYPE_SET(CAN_EVENT_TYPE ) ) //!< TEF data structure
#define CAN_TXQ            ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_TRANSMIT) | CAN_DATA_STRUCT_TYPE_SET(CAN_QUEUE_TYPE ) ) //!< TXQ data structure
#define CAN_TxBUFFER(idx)  ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_TRANSMIT) | CAN_DATA_STRUCT_TYPE_SET(CAN_BUFFER_TYPE) | CAN_DATA_STRUCT_INDEX_SET(idx) ) //!< Tx Buffer data structure
#define CAN_RxBUFFER(idx)  ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_RECEIVE ) | CAN_DATA_STRUCT_TYPE_SET(CAN_BUFFER_TYPE) | CAN_DATA_STRUCT_INDEX_SET(idx) ) //!< Rx Buffer data structure
#define CAN_TxFIFO(idx)    ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_TRANSMIT) | CAN_DATA_STRUCT_TYPE_SET(CAN_FIFO_TYPE  ) | CAN_DATA_STRUCT_INDEX_SET(idx) ) //!< Tx FIFO data structure
#define CAN_RxFIFO(idx)    ( CAN_DATA_STRUCT_DIR_SET(CAN_DATA_STRUCT_RECEIVE ) | CAN_DATA_STRUCT_TYPE_SET(CAN_FIFO_TYPE  ) | CAN_DATA_STRUCT_INDEX_SET(idx) ) //!< Rx FIFO data structure

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CAN_COMMON_H_INC */