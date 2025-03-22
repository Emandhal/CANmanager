/*!*****************************************************************************
 * @file    TCAN455X.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.1
 * @date    22/03/2025
 * @brief   TCAN4550 and TCAN4551 driver
 * @details
 * The TCAN4550/TCAN4551 component is a CAN-bus controller supporting CAN2.0A, CAN2.0B
 * and CAN-FD with SPI interface
 * Follow datasheet TCAN4550    Rev.A (Jan  2020)
 *                  TCAN4550-Q1 Rev.D (June 2022)
 *                  TCAN4551-Q1 Rev.A (Nov  2019)
 ******************************************************************************/
/* @page License
 *
 * Copyright (c) 2020-2025 Fabien MAILLY
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
 * 1.0.1    Put 'ul' suffix instead of 'u' suffix on Masks values to remove warnings on 16-bits systems
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef TCAN455X_H_INC
#define TCAN455X_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#include "CAN_common.h"
#include "MCAN_core.h"
#include "SPI_Interface.h"
#include "ErrorsDef.h"
#ifdef USE_GENERICS_DEFINED
#  include "GPIO_Interface.h"
#endif
//-----------------------------------------------------------------------------
#ifdef __cplusplus

# define USE_TCAN455X_TOOLS
  extern "C" {
#  define __TCAN455X_PACKED__
#  define TCAN455X_PACKITEM             __pragma(pack(push, 1))
#  define TCAN455X_UNPACKITEM           __pragma(pack(pop))
#  define TCAN455X_PACKENUM(name,type)  typedef enum name : type
#  define TCAN455X_UNPACKENUM(name)     name

#else

#  define __TCAN455X_PACKED__           __attribute__((packed))
#  define TCAN455X_PACKITEM
#  define TCAN455X_UNPACKITEM
#  define TCAN455X_PACKENUM(name,type)  typedef enum __TCAN455X_PACKED__
#  define TCAN455X_UNPACKENUM(name)     name

#endif
//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define TCAN455X_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------

//! int32_t to 2-uint16_t to 4-uint8_t conversion
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__
{
  uint32_t Uint32;
  uint16_t Uint16[sizeof(uint32_t) / sizeof(uint16_t)];
  uint8_t  Bytes[sizeof(uint32_t) / sizeof(uint8_t)];
} TCAN455X_uint32t_Conv;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_uint32t_Conv, 4);

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN455X limits definitions
//********************************************************************************************************************

// Frequencies and bitrate limits for TCAN455X
#define TCAN455X_XTALFREQ_MIN     ( 20000000u ) //!< Min Xtal/Resonator frequency
#define TCAN455X_XTALFREQ_MAX     ( 40000000u ) //!< Max Xtal/Resonator frequency
#define TCAN455X_NOMBITRATE_MIN   (   125000u ) //!< Min Nominal bitrate
#define TCAN455X_NOMBITRATE_MAX   (  1000000u ) //!< Max Nominal bitrate
#define TCAN455X_DATABITRATE_MIN  (   500000u ) //!< Min Data bitrate
#define TCAN455X_DATABITRATE_MAX  (  5000000u ) //!< Max Data bitrate
#define TCAN455X_SPICLOCK_MAX     ( 18000000u ) //!< Max SPI clock frequency for TCAN455X

//-----------------------------------------------------------------------------

// Limits Bit Rate configuration range for TCAN455X
#define TCAN455X_tTXDtRXD_MAX  ( MCAN_tTXDtRXD_MAX ) //!< tTXD-RXD is the propagation delay of the transceiver, a maximum 255ns according to ISO 11898-1:2015
#define TCAN455X_tBUS_CONV     ( MCAN_tBUS_CONV    ) //!< TBUS is the delay on the CAN bus, which is approximately 5ns/m

#define TCAN455X_NBRP_MIN      ( MCAN_NBRP_MIN   ) //!< Min NBRP
#define TCAN455X_NBRP_MAX      ( MCAN_NBRP_MAX   ) //!< Max NBRP
#define TCAN455X_NSYNC         ( MCAN_NSYNC      ) //!< NSYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define TCAN455X_NTSEG1_MIN    ( MCAN_NTSEG1_MIN ) //!< Min NTSEG1
#define TCAN455X_NTSEG1_MAX    ( MCAN_NTSEG1_MAX ) //!< Max NTSEG1
#define TCAN455X_NTSEG2_MIN    ( MCAN_NTSEG2_MIN ) //!< Min NTSEG2
#define TCAN455X_NTSEG2_MAX    ( MCAN_NTSEG2_MAX ) //!< Max NTSEG2
#define TCAN455X_NSJW_MIN      ( MCAN_NSJW_MIN   ) //!< Min NSJW
#define TCAN455X_NSJW_MAX      ( MCAN_NSJW_MAX   ) //!< Max NSJW
#define TCAN455X_NTQBIT_MIN    ( MCAN_NTQBIT_MIN ) //!< Min NTQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define TCAN455X_NTQBIT_MAX    ( MCAN_NTQBIT_MAX ) //!< Max NTQ per Bit (385-bits)

#define TCAN455X_DBRP_MIN      ( MCAN_DBRP_MIN   ) //!< Min DBRP
#define TCAN455X_DBRP_MAX      ( MCAN_DBRP_MAX   ) //!< Max DBRP
#define TCAN455X_DSYNC         ( MCAN_DSYNC      ) //!< DSYNC is 1 NTQ (Defined in ISO 11898-1:2015)
#define TCAN455X_DTSEG1_MIN    ( MCAN_DTSEG1_MIN ) //!< Min DTSEG1
#define TCAN455X_DTSEG1_MAX    ( MCAN_DTSEG1_MAX ) //!< Max DTSEG1
#define TCAN455X_DTSEG2_MIN    ( MCAN_DTSEG2_MIN ) //!< Min DTSEG2
#define TCAN455X_DTSEG2_MAX    ( MCAN_DTSEG2_MAX ) //!< Max DTSEG2
#define TCAN455X_DSJW_MIN      ( MCAN_DSJW_MIN   ) //!< Min DSJW
#define TCAN455X_DSJW_MAX      ( MCAN_DSJW_MAX   ) //!< Max DSJW
#define TCAN455X_DTQBIT_MIN    ( MCAN_DTQBIT_MIN ) //!< Min DTQ per Bit (1-bit SYNC + 1-bit PRSEG + 1-bit PHSEG1 + 1-bit PHSEG2)
#define TCAN455X_DTQBIT_MAX    ( MCAN_DTQBIT_MAX ) //!< Max DTQ per Bit (49-bits)

#define TCAN455X_TDCO_MIN      ( MCAN_TDCO_MIN   ) //!< Min TDCO
#define TCAN455X_TDCO_MAX      ( MCAN_TDCO_MAX   ) //!< Max TDCO

//-----------------------------------------------------------------------------

// FIFO/Buffers definitions
#define TCAN455X_TEF_MAX        ( MCAN_TEF_MAX       ) //!< 1 TEF maximum
#define TCAN455X_TXQ_FIFO_MAX   ( MCAN_TXQ_FIFO_MAX  ) //!< 1 TXQ/FIFO maximum
#define TCAN455X_TX_BUFFER_MAX  ( MCAN_TX_BUFFER_MAX ) //!< 1 Tx buffer maximum
#define TCAN455X_FIFO_MAX       ( MCAN_FIFO_MAX      ) //!< 2 Rx FIFOs maximum
#define TCAN455X_RX_BUFFER_MAX  ( MCAN_RX_BUFFER_MAX ) //!< 1 Rx buffer maximum
#define TCAN455X_FIFO_CONF_MAX  ( MCAN_FIFO_CONF_MAX ) //!< Maximum 6 FIFO configurable (TEF + TXQ/FIFO + Tx buffer + 2 Rx FIFO + Rx buffer)
#define TCAN455X_TX_FIFO_MAX    ( MCAN_TX_FIFO_MAX   ) //!< Maximum 2 transmit FIFO (TXQ/FIFO + Tx buffer)
#define TCAN455X_RX_FIFO_MAX    ( MCAN_RX_FIFO_MAX   ) //!< Maximum 4 receive FIFO (TEF + 2 FIFO + Rx buffer)

#define TCAN455X_TX_EVENT_FIFO_SIZE_MAX  ( MCAN_TX_EVENT_FIFO_SIZE_MAX ) //!< 32 elements max for the Tx Event FIFO
#define TCAN455X_TX_BUFFER_SIZE_MAX      ( MCAN_TX_BUFFER_SIZE_MAX     ) //!< 32 elements max for the Tx Buffer
#define TCAN455X_TX_FIFO_TXQ_SIZE_MAX    ( MCAN_TX_FIFO_TXQ_SIZE_MAX   ) //!< 32 elements max for the Tx Queue/FIFO
#define TCAN455X_TX_ELEMENTS_SIZE_MAX    ( MCAN_TX_ELEMENTS_SIZE_MAX   ) //!< 32 elements max for the Tx Buffer + Tx Queue/FIFO
#define TCAN455X_RX_FIFO0_SIZE_MAX       ( MCAN_RX_FIFO0_SIZE_MAX      ) //!< 64 elements max for the Rx FIFO0
#define TCAN455X_RX_FIFO1_SIZE_MAX       ( MCAN_RX_FIFO1_SIZE_MAX      ) //!< 64 elements max for the Rx FIFO1
#define TCAN455X_RX_BUFFER_SIZE_MAX      ( MCAN_RX_BUFFER_SIZE_MAX     ) //!< 64 elements max for the Rx Buffer

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN455X's driver definitions
//********************************************************************************************************************

//! Driver configuration enum
typedef enum
{
  TCAN455X_DRIVER_NORMAL_USE               = 0x00, //!< Use the driver with no special verifications, just settings verifications (usually the fastest mode)
  TCAN455X_DRIVER_ENABLE_ECC               = 0x02, //!< Enable the ECC just before the RAM initialization and activate ECCCON_SECIE and ECCCON_DEDIE interrupt flags
  TCAN455X_DRIVER_INIT_CHECK_RAM           = 0x04, //!< Check RAM at initialization by writing some data and checking them on all the RAM range (slower at initialization, take a long time)
  TCAN455X_DRIVER_INIT_SET_RAM_AT_0        = 0x08, //!< Set all bytes of the RAM to 0x00 (slower at initialization)
} eTCAN455X_DriverConfig;

typedef eTCAN455X_DriverConfig setTCAN455X_DriverConfig; //!< Set of Driver configuration (can be OR'ed)

//-----------------------------------------------------------------------------

/*! @defgroup TCAN455X_SPIcomm SPI commands instructions
 * @brief SPI commands instructions for TCAN455X communications
 * @details The start address must be word aligned (32-bit). Any time the registers are accessed, bits [1:0] of the address are ignored as the addresses are always word (32-bit/4-byte) aligned.
 * As an example for accessing the M_CAN registers, for the register 0x1004, give the SPI address 1004, 1005, 1006 or 1007, and access register 1004. The registers are 32 bit and only 1004 is valid in this example
 * @note
 * - The two low order address bits is ignored
 * - A length of 8’h00 indicates 256 words to be transferred
 */
//! @addtogroup TCAN455X_SPIcomm
//! @{
#define TCAN455X_READ_B_FL   ( 0x41 ) //!< Read instruction (< READ_B_FL > <2 address bytes> <1 length bytes> <length words of read data>)
#define TCAN455X_WRITE_B_FL  ( 0x61 ) //!< Write instruction (< WRITE_B_FL > <2 address bytes> <1 length bytes> <length words of write data>)
//! @}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN455X Register list
//********************************************************************************************************************

//! TCAN455X registers list
typedef enum
{
  //--- TCAN455X Registers ---
  // Device ID and Interrupt/Diagnostic Flag registers (Base Address: 0x0000, Address Range: 0x002F)
  RegTCAN455X_DEVICE_ID1              = 0x0000u, //!< (Offset: 0x0000) Device ID1 Register
  RegTCAN455X_DEVICE_ID2              = 0x0004u, //!< (Offset: 0x0004) Device ID2 Register
  RegTCAN455X_REVISION                = 0x0008u, //!< (Offset: 0x0008) Revision Register
  RegTCAN455X_STATUS                  = 0x000Cu, //!< (Offset: 0x000C) Status Register
  RegTCAN455X_SPI_MASK                = 0x0010u, //!< (Offset: 0x0010) [TCAN4550-Q1 only] SPI Error status mask Register
  //   (Offset: 0x0014..0x07FC) Reserved

  // Device Configuration registers (Base Address: 0x0800, Address Range: 0x08FF)
  RegTCAN455X_DEV_MODES_AND_PINS      = 0x0800u, //!< (Offset: 0x0800) Modes of Operation and Pin Configurations
  RegTCAN455X_DEV_TIMESTAMP_PRESCALER = 0x0804u, //!< (Offset: 0x0804) Timestamp Prescalar
  RegTCAN455X_DEV_TEST_SCRATCH_PAD    = 0x0808u, //!< (Offset: 0x0808) Test Register and Scratch Pad
  RegTCAN455X_DEV_ECC_TEST            = 0x080Cu, //!< (Offset: 0x080C) ECC Test Register
  //   (Offset: 0x0810..0x081C) Reserved

  // Interrupt/Diagnostic Flag and Enable Flag registers (Base Address: 0x0820, Address Range: 0x0830)
  RegTCAN455X_DEV_IR                  = 0x0820u, //!< (Offset: 0x0820) Interrupt Flags
  RegTCAN455X_DEV_IR_MCAN             = 0x0824u, //!< (Offset: 0x0824) MCAN Interrupt Flags
  //   (Offset: 0x0828..0x082F) Reserved
  RegTCAN455X_DEV_IE                  = 0x0830u, //!< (Offset: 0x0830) Interrupt Enable
  //   (Offset: 0x0834..0x083F) Reserved

  //--- MCAN CAN-FD register set (Base Address: 0x1000, Address Range: 0x10FF) ---
  RegTCAN455X_MCAN                    = 0x1000u, //!< (Offset: 0x1000) MCAN CAN-FD register set base address

  //--- MRAM (Base Address: 0x8000, Address Range: 0x87FF) ---
  RegTCAN455X_MRAM                    = 0x8000u, //!< (Offset: 0x8000) MRAM base address
  RegTCAN455X_MRAM_START              = 0x8000u, //!< (Offset: 0x8000) MRAM start address
  RegTCAN455X_MRAM_END                = 0x87FFu, //!< (Offset: 0x87FF) MRAM end address
} eTCAN455X_Registers;

#define TCAN455X_CONVERT_TO_MCAN_REG_ADDR(addr)  ( (uint32_t)RegTCAN455X_MCAN + MCAN_EXTRACT_REG_ADDR(addr) ) //!< Convert address into MCAN register address range

//-----------------------------------------------------------------------------

#define TCAN455X_SYSTEM_CLOCK_DIVIDER  ( 1 ) //!< Sysclock divider before entering the MCAN peripheral
#define TCAN455X_CLOCK_MAX             ( TCAN455X_XTALFREQ_MAX ) //!< Max TCAN455X peripheral clock is 40MHz

#define TCAN455X_RAM_START_ADDRESS     ( RegTCAN455X_MRAM_START )
#define TCAN455X_MAX_RAM               ( RegTCAN455X_MRAM_END + 1 - RegTCAN455X_MRAM ) //!< Max possible RAM use on TCAN455X is 512 words of 32 bits
#define TCAN455X_MAX_RAM_ADDRESS       ( TCAN455X_RAM_START_ADDRESS + TCAN455X_MAX_RAM )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN455X Controller Registers
//********************************************************************************************************************

//! TCAN455X Device ID Register (Read-Only, Offset: 0x0000, Reset: 0x4E414354_30353534 (for TCAN4550) or 0x4E414354_31353534 (for TCAN4551))
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_DEVID_Register
{
  uint64_t DEVID;
  uint8_t Bytes[sizeof(uint64_t)];
  struct
  {
    uint32_t DEVID1; //!< (Offset: 0x0000) Device ID1 Register
    uint32_t DEVID2; //!< (Offset: 0x0004) Device ID2 Register
  };
} TCAN455X_DEVID_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_DEVID_Register, 8);

//! List of supported devices
typedef enum
{
  TCAN4550,               //!< TCAN4550/TCAN4550-Q1 supported
  TCAN4551,               //!< TCAN4551-Q1 supported
  eTCAN455X_DEVICE_COUNT, //!< Device count, KEEP LAST
} eTCAN455X_Devices;

#define TCAN4550_DEVID1_VALUE       0x4E414354ul //!< This value reflect the string: "TCAN"
#define TCAN4550_DEVID2_VALUE       0x30353534ul //!< This value reflect the string: "4550"
#define TCAN4551_DEVID1_VALUE       0x4E414354ul //!< This value reflect the string: "TCAN"
#define TCAN4551_DEVID2_VALUE       0x31353534ul //!< This value reflect the string: "4551"

#define TCAN455X_DEVID1_Pos         0
#define TCAN455X_DEVID1_Mask        (0xFFFFFFFFul << TCAN455X_DEVID1_Pos)
#define TCAN455X_DEVID1_GET(value)  (((uint32_t)(value) & TCAN455X_DEVID1_Mask) >> TCAN455X_DEVID1_Pos) //!< Get Device ID1
#define TCAN455X_DEVID2_Pos         0
#define TCAN455X_DEVID2_Mask        (0xFFFFFFFFul << TCAN455X_DEVID2_Pos)
#define TCAN455X_DEVID2_GET(value)  (((uint32_t)(value) & TCAN455X_DEVID2_Mask) >> TCAN455X_DEVID2_Pos) //!< Get Device ID2

static const char* const TCAN455X_DevicesNames[eTCAN455X_DEVICE_COUNT] =
{
  "TCAN4550",
  "TCAN4551",
};

//-----------------------------------------------------------------------------

//! TCAN455X Revision Register (Read-Only, Offset: 0x0008, Reset: 0x00110201)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_REVISION_Register
{
  uint32_t REVISION;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t REV_ID_MINOR  : 8; //!<  0- 7 - Device REV_ID Minor
    uint32_t REV_ID_MAJOR  : 8; //!<  8-15 - Device REV_ID Major
    uint32_t               : 8; //!< 16-23
    uint32_t SPI_2_REVISION: 8; //!< 24-31 - Revision version of the SPI module
  } Bits;
} TCAN455X_REVISION_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_REVISION_Register, 4);

#define TCAN455X_REVISION_REV_ID_MINOR_Pos           0
#define TCAN455X_REVISION_REV_ID_MINOR_Mask          (0xFFul << TCAN455X_REVISION_REV_ID_MINOR_Pos)
#define TCAN455X_REVISION_REV_ID_MINOR_GET(value)    (((uint32_t)(value) & TCAN455X_REVISION_REV_ID_MINOR_Mask) >> TCAN455X_REVISION_REV_ID_MINOR_Pos) //!< Get Device REV_ID Minor
#define TCAN455X_REVISION_REV_ID_MAJOR_Pos           8
#define TCAN455X_REVISION_REV_ID_MAJOR_Mask          (0xFFul << TCAN455X_REVISION_REV_ID_MAJOR_Pos)
#define TCAN455X_REVISION_REV_ID_MAJOR_GET(value)    (((uint32_t)(value) & TCAN455X_REVISION_REV_ID_MAJOR_Mask) >> TCAN455X_REVISION_REV_ID_MAJOR_Pos) //!< Get Device REV_ID Major
#define TCAN455X_REVISION_SPI_2_REVISION_Pos         24
#define TCAN455X_REVISION_SPI_2_REVISION_Mask        (0xFFul << TCAN455X_REVISION_SPI_2_REVISION_Pos)
#define TCAN455X_REVISION_SPI_2_REVISION_GET(value)  (((uint32_t)(value) & TCAN455X_REVISION_SPI_2_REVISION_Mask) >> TCAN455X_REVISION_SPI_2_REVISION_Pos) //!< Get Revision version of the SPI module

//-----------------------------------------------------------------------------

//! TCAN455X Status Register (Read/Write, Offset: 0x000C, Reset: 0x0000000U)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_STATUS_Register
{
  uint32_t STATUS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t INTERRUPT               :  1; //!<  0    - [Read-Only] Value of interrupt input level (active high)
    uint32_t SPI_ERROR_INTERRUPT     :  1; //!<  1    - [Read-Only] Unmasked SPI error set
    uint32_t INTERNAL_ERROR_INTERRUPT:  1; //!<  2    - [Read-Only] Unmasked Internal error set
    uint32_t INTERNAL_ACCESS_ACTIVE  :  1; //!<  3    - [Read-Only] Internal Multiple transfer mode access in progress
    uint32_t READ_FIFO_AVAILABLE     :  1; //!<  4    - [Read-Only] Read fifo entries is greater than or equal to the read_fifo_threshold
    uint32_t WRITE_FIFO_AVAILABLE    :  1; //!<  5    - [Read-Only] Write fifo empty entries is greater than or equal to the write_fifo_threshold
    uint32_t                         : 10; //!<  6-15
    uint32_t READ_UNDERFLOW          :  1; //!< 16    - SPI read sequence ended with less data transferred then requested
    uint32_t READ_OVERFLOW           :  1; //!< 17    - SPI read sequence had continue requests after the data transfer was completed
    uint32_t WRITE_UNDERFLOW         :  1; //!< 18    - SPI write sequence ended with less data transferred then requested
    uint32_t WRITE_OVERFLOW          :  1; //!< 19    - SPI write sequence had continue requests after the data transfer was completed
    uint32_t INVALID_COMMAND         :  1; //!< 20    - Invalid SPI command received
    uint32_t SPI_END_ERROR           :  1; //!< 21    - SPI transfer did not end on a byte boundary
    uint32_t                         :  2; //!< 22-23
    uint32_t WRITE_FIFO_OVERFLOW     :  1; //!< 24    - Write/command FIFO overflow
    uint32_t READ_FIFO_EMPTY         :  1; //!< 25    - Read FIFO empty for first read data word to return
    uint32_t READ_FIFO_UNDERFLOW     :  1; //!< 26    - Read FIFO underflow after 1 or more read data words returned
    uint32_t INTERNAL_ERROR_LOG_WRITE:  1; //!< 27    - Entry written to the Internal error log
    uint32_t INTERNAL_WRITE_ERROR    :  1; //!< 28    - Internal write received an error response
    uint32_t INTERNAL_READ_ERROR     :  1; //!< 29    - Internal read received an error response
    uint32_t                         :  2; //!< 30-31
  } Bits;
} TCAN455X_STATUS_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_STATUS_Register, 4);

#define TCAN455X_STATUS_INTERRUPT                 (0x1ul <<  0) //!< Value of interrupt input level (active high)
#define TCAN455X_STATUS_SPI_ERROR_INTERRUPT       (0x1ul <<  1) //!< Unmasked SPI error set
#define TCAN455X_STATUS_INTERNAL_ERROR_INTERRUPT  (0x1ul <<  2) //!< Unmasked Internal error set
#define TCAN455X_STATUS_INTERNAL_ACCESS_ACTIVE    (0x1ul <<  3) //!< Internal Multiple transfer mode access in progress
#define TCAN455X_STATUS_READ_FIFO_AVAILABLE       (0x1ul <<  4) //!< Read fifo entries is greater than or equal to the read_fifo_threshold
#define TCAN455X_STATUS_WRITE_FIFO_AVAILABLE      (0x1ul <<  5) //!< Write fifo empty entries is greater than or equal to the write_fifo_threshold

#define TCAN455X_STATUS_READ_UNDERFLOW            (0x1ul << 16) //!< SPI read sequence ended with less data transferred then requested
#define TCAN455X_STATUS_READ_OVERFLOW             (0x1ul << 17) //!< SPI read sequence had continue requests after the data transfer was completed
#define TCAN455X_STATUS_WRITE_UNDERFLOW           (0x1ul << 18) //!< SPI write sequence ended with less data transferred then requested
#define TCAN455X_STATUS_WRITE_OVERFLOW            (0x1ul << 19) //!< SPI write sequence had continue requests after the data transfer was completed
#define TCAN455X_STATUS_INVALID_COMMAND           (0x1ul << 20) //!< Invalid SPI command received
#define TCAN455X_STATUS_SPI_END_ERROR             (0x1ul << 21) //!< SPI transfer did not end on a byte boundary

#define TCAN455X_STATUS_WRITE_FIFO_OVERFLOW       (0x1ul << 24) //!< Write/command FIFO overflow
#define TCAN455X_STATUS_READ_FIFO_EMPTY           (0x1ul << 25) //!< Read FIFO empty for first read data word to return
#define TCAN455X_STATUS_READ_FIFO_UNDERFLOW       (0x1ul << 26) //!< Read FIFO underflow after 1 or more read data words returned
#define TCAN455X_STATUS_INTERNAL_ERROR_LOG_WRITE  (0x1ul << 27) //!< Entry written to the Internal error log
#define TCAN455X_STATUS_INTERNAL_WRITE_ERROR      (0x1ul << 28) //!< Internal write received an error response
#define TCAN455X_STATUS_INTERNAL_READ_ERROR       (0x1ul << 29) //!< Internal read received an error response

#define TCAN455X_DEVICE_STATUS_EVENT_CLEARABLE_FLAGS  ( TCAN455X_STATUS_READ_UNDERFLOW           | TCAN455X_STATUS_READ_OVERFLOW            | \
                                                        TCAN455X_STATUS_WRITE_UNDERFLOW          | TCAN455X_STATUS_WRITE_OVERFLOW           | \
                                                        TCAN455X_STATUS_INVALID_COMMAND          | TCAN455X_STATUS_SPI_END_ERROR            | \
                                                        TCAN455X_STATUS_WRITE_FIFO_OVERFLOW      | TCAN455X_STATUS_READ_FIFO_EMPTY          | \
                                                        TCAN455X_STATUS_READ_FIFO_UNDERFLOW      | TCAN455X_STATUS_INTERNAL_ERROR_LOG_WRITE | \
                                                        TCAN455X_STATUS_INTERNAL_WRITE_ERROR     | TCAN455X_STATUS_INTERNAL_READ_ERROR      ) //!< All TCAN455X clearable events status flags

#define TCAN455X_DEVICE_STATUS_EVENT_FLAGS  ( TCAN455X_STATUS_INTERRUPT                | TCAN455X_STATUS_SPI_ERROR_INTERRUPT      | \
                                              TCAN455X_STATUS_INTERNAL_ERROR_INTERRUPT | TCAN455X_STATUS_INTERNAL_ACCESS_ACTIVE   | \
                                              TCAN455X_STATUS_READ_FIFO_AVAILABLE      | TCAN455X_STATUS_WRITE_FIFO_AVAILABLE     | \
                                              TCAN455X_STATUS_READ_UNDERFLOW           | TCAN455X_STATUS_READ_OVERFLOW            | \
                                              TCAN455X_STATUS_WRITE_UNDERFLOW          | TCAN455X_STATUS_WRITE_OVERFLOW           | \
                                              TCAN455X_STATUS_INVALID_COMMAND          | TCAN455X_STATUS_SPI_END_ERROR            | \
                                              TCAN455X_STATUS_WRITE_FIFO_OVERFLOW      | TCAN455X_STATUS_READ_FIFO_EMPTY          | \
                                              TCAN455X_STATUS_READ_FIFO_UNDERFLOW      | TCAN455X_STATUS_INTERNAL_ERROR_LOG_WRITE | \
                                              TCAN455X_STATUS_INTERNAL_WRITE_ERROR     | TCAN455X_STATUS_INTERNAL_READ_ERROR      ) //!< All TCAN455X events status flags

//! Device Status Events, can be OR'ed.
typedef enum
{
  TCAN455X_NO_EVENT                           = 0x00,                                         //!< No interrupt events
  TCAN455X_INT_INPUT_LEVEL                    = TCAN455X_STATUS_INTERRUPT,                    //!< Value of interrupt input level (active high)
  TCAN455X_SPI_ERROR_EVENT                    = TCAN455X_STATUS_SPI_ERROR_INTERRUPT,          //!< Unmasked SPI error set
  TCAN455X_INTERNAL_ERROR_EVENT               = TCAN455X_STATUS_INTERNAL_ERROR_INTERRUPT,     //!< Unmasked Internal error set
  TCAN455X_INTERNAL_ACCESS_EVENT              = TCAN455X_STATUS_INTERNAL_ACCESS_ACTIVE,       //!< Internal Multiple transfer mode access in progress
  TCAN455X_READ_FIFO_AVAILABLE_EVENT          = TCAN455X_STATUS_READ_FIFO_AVAILABLE,          //!< Read fifo entries is greater than or equal to the read_fifo_threshold
  TCAN455X_WRITE_FIFO_AVAILABLE_EVENT         = TCAN455X_STATUS_WRITE_FIFO_AVAILABLE,         //!< Write fifo empty entries is greater than or equal to the write_fifo_threshold

  TCAN455X_SPI_LESS_READ_DATA_EVENT           = TCAN455X_STATUS_READ_UNDERFLOW,               //!< SPI read sequence ended with less data transferred then requested
  TCAN455X_SPI_MORE_READ_DATA_EVENT           = TCAN455X_STATUS_READ_OVERFLOW,                //!< SPI read sequence had continue requests after the data transfer was completed
  TCAN455X_SPI_LESS_WRITE_DATA_EVENT          = TCAN455X_STATUS_WRITE_UNDERFLOW,              //!< SPI write sequence ended with less data transferred then requested
  TCAN455X_SPI_MORE_WRITE_DATA_EVENT          = TCAN455X_STATUS_WRITE_OVERFLOW,               //!< SPI write sequence had continue requests after the data transfer was completed
  TCAN455X_INVALID_SPI_COMMAND_EVENT          = TCAN455X_STATUS_INVALID_COMMAND,              //!< Invalid SPI command received
  TCAN455X_SPI_NO_END_BOUNDARY_EVENT          = TCAN455X_STATUS_SPI_END_ERROR,                //!< SPI transfer did not end on a byte boundary

  TCAN455X_WRITE_CMD_FIFO_OVERFLOW_EVENT      = TCAN455X_STATUS_WRITE_FIFO_OVERFLOW,          //!< Write/command FIFO overflow
  TCAN455X_READ_FIFO_EMPTY_EVENT              = TCAN455X_STATUS_READ_FIFO_EMPTY,              //!< Read FIFO empty for first read data word to return
  TCAN455X_READ_FIFO_UNDERFLOW_EVENT          = TCAN455X_STATUS_READ_FIFO_UNDERFLOW,          //!< Read FIFO underflow after 1 or more read data words returned
  TCAN455X_INTERNAL_ERROR_LOG_WRITE_EVENT     = TCAN455X_STATUS_INTERNAL_ERROR_LOG_WRITE,     //!< Entry written to the Internal error log
  TCAN455X_INTERNAL_WRITE_ERROR_EVENT         = TCAN455X_STATUS_INTERNAL_WRITE_ERROR,         //!< Internal write received an error response
  TCAN455X_INTERNAL_READ_ERROR_EVENT          = TCAN455X_STATUS_INTERNAL_READ_ERROR,          //!< Internal read received an error response

  TCAN455X_CLEARABLE_DEVICE_STATUS_EVENT_MASK = TCAN455X_DEVICE_STATUS_EVENT_CLEARABLE_FLAGS, //!< Device Status Events flags mask
  TCAN455X_DEVICE_STATUS_EVENT_MASK           = TCAN455X_DEVICE_STATUS_EVENT_FLAGS,           //!< Device Status Events flags mask
} eTCAN455X_DeviceStatusEvents;

typedef eTCAN455X_DeviceStatusEvents setTCAN455X_DeviceStatusEvents; //!< Set of TCAN455X Device Status Events (can be OR'ed)

//-----------------------------------------------------------------------------

//! TCAN4550-Q1 SPI error status mask Register (Read/Write, Offset: 0x0010, Reset: 0x00000000)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_SPI_MASK_Register
{
  uint32_t SPI_MASK;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t                              : 16; //!<  0-15
    uint32_t MASK_READ_UNDERFLOW          :  1; //!< 16    - When set the corresponding error bit will be masked
    uint32_t MASK_READ_OVERFLOW           :  1; //!< 17    - When set the corresponding error bit will be masked
    uint32_t MASK_WRITE_UNDERFLOW         :  1; //!< 18    - When set the corresponding error bit will be masked
    uint32_t MASK_WRITE_OVERFLOW          :  1; //!< 19    - When set the corresponding error bit will be masked
    uint32_t MASK_INVALID_COMMAND         :  1; //!< 20    - When set the corresponding error bit will be masked
    uint32_t MASK_SPI_END_ERROR           :  1; //!< 21    - When set the corresponding error bit will be masked
    uint32_t                              :  2; //!< 22-23
    uint32_t MASK_WRITE_FIFO_OVERFLOW     :  1; //!< 24    - When set the corresponding error bit will be masked
    uint32_t MASK_READ_FIFO_EMPTY         :  1; //!< 25    - When set the corresponding error bit will be masked
    uint32_t MASK_READ_FIFO_UNDERFLOW     :  1; //!< 26    - When set the corresponding error bit will be masked
    uint32_t MASK_INTERNAL_ERROR_LOG_WRITE:  1; //!< 27    - When set the corresponding error bit will be masked
    uint32_t MASK_INTERNAL_WRITE_ERROR    :  1; //!< 28    - When set the corresponding error bit will be masked
    uint32_t MASK_INTERNAL_READ_ERROR     :  1; //!< 29    - When set the corresponding error bit will be masked
    uint32_t                              :  2; //!< 30-31
  } Bits;
} TCAN455X_SPI_MASK_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_SPI_MASK_Register, 4);

#define TCAN455X_MASK_READ_UNDERFLOW            (0x1ul << 16) //!< Set the mask read underflow error bit will be masked
#define TCAN455X_MASK_READ_OVERFLOW             (0x1ul << 17) //!< Set the mask read overflow error bit will be masked
#define TCAN455X_MASK_WRITE_UNDERFLOW           (0x1ul << 18) //!< Set the mask write underflow error bit will be masked
#define TCAN455X_MASK_WRITE_OVERFLOW            (0x1ul << 19) //!< Set the mask write overflow error bit will be masked
#define TCAN455X_MASK_INVALID_COMMAND           (0x1ul << 20) //!< Set the mask invalid command error bit will be masked
#define TCAN455X_MASK_SPI_END_ERROR             (0x1ul << 21) //!< Set the mask SPI end error bit will be masked

#define TCAN455X_MASK_WRITE_FIFO_OVERFLOW       (0x1ul << 24) //!< Set the mask write FIFO overflow error bit will be masked
#define TCAN455X_MASK_READ_FIFO_EMPTY           (0x1ul << 25) //!< Set the mask read FIFO empty error bit will be masked
#define TCAN455X_MASK_READ_FIFO_UNDERFLOW       (0x1ul << 26) //!< Set the mask read FIFO underflow error bit will be masked
#define TCAN455X_MASK_INTERNAL_ERROR_LOG_WRITE  (0x1ul << 27) //!< Set the mask internal error log write bit will be masked
#define TCAN455X_MASK_INTERNAL_WRITE_ERROR      (0x1ul << 28) //!< Set the mask internal write error bit will be masked
#define TCAN455X_MASK_INTERNAL_READ_ERROR       (0x1ul << 29) //!< Set the mask internal read error bit will be masked

//-----------------------------------------------------------------------------

//! TCAN455X Modes of Operation and Pin Configurations Register (Read/Write, Offset: 0x0800, Reset: 0xC8000468)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_MODE_PINS_Register
{
  uint32_t MODE_PINS;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t TEST_MODE_CONFIG: 1; //!<  0    - [Saved when entering sleep mode] Test Mode Configuration: '1' = CAN Controller test with TXD/RXD_INT_CAN mapped to external pins ; '0' = Phy Test with TXD/RXD_INT_PHY and EN_INT are mapped to external pins
    uint32_t SWE_DIS         : 1; //!<  1    - [Saved when entering sleep mode] Sleep Wake Error Disable. This disables the device from starting the four minute timer when coming out of sleep mode on a wake event. If this is enabled a SPI read or write must take place within this four minute window or the device will go back to sleep. This does not disable the function for initial power on or in case of a power on reset: '1' = Disabled ; '0' = Enabled
    uint32_t DEVICE_RESET    : 1; //!<  2    - Device Reset. Same function as RST pin: '1' = Device resets to default ; '0' = Current configuration
    uint32_t WD_EN           : 1; //!<  3    - [TCAN4550/TCAN4550-Q1 only] Watchdog Enable: '1' = Enabled ; '0' = Disabled
    uint32_t                 : 1; //!<  4
    uint32_t                 : 1; //!<  5    - When writing to this register, this bit must always be a 1
    uint32_t MODE_SEL        : 2; //!<  6- 7 - Mode of operation select. The Mode of Operation changes the mode but will read back the mode the device is currently in. When the device is changing the device to normal mode a write of 0 to CCCR.INIT is automatically issued and when changing from normal mode to standby or sleep modes a write of 1 to CCCR.INIT is automatically issued
    uint32_t nWKRQ_CONFIG    : 1; //!<  8    - [Saved when entering sleep mode] nWKRQ Pin Function. nWKRQ pin defaults to a push-pull active low configuration based off an internal voltage rail. When configuring this to work off of VIO the pin becomes and open drain output and a external pull up resistor to the VIO rail is required: '1' = Wake request interrupt ; '0' = Mirrors INH function
    uint32_t INH_DIS         : 1; //!<  9    - [Saved when entering sleep mode] INH Pin Disable: '1' = = Pin disabled ; '0' = Pin enabled
    uint32_t GPO1_GPO_CONFIG : 2; //!< 10-11 - [Saved when entering sleep mode] GPIO1 pin GPO1 function select
    uint32_t                 : 1; //!< 12
    uint32_t FAIL_SAFE_EN    : 1; //!< 13    - [Saved when entering sleep mode] Fail safe mode enable. Excludes power up fail safe: '1' = Enabled ; '0' = Disabled
    uint32_t GPIO1_CONFIG    : 2; //!< 14-15 - [TCAN4550/TCAN4550-Q1 only] [Saved when entering sleep mode] GPIO1 Pin Function Select
    uint32_t WD_ACTION       : 2; //!< 16-17 - [TCAN4550/TCAN4550-Q1 only] Selected action when WD_TIMER times out
    uint32_t WD_BIT_SET      : 1; //!< 18    - [TCAN4550/TCAN4550-Q1 only] Write a 1 to reset timer: if times out this bit will set and then the selected action from 0800[17:16] will take place. (TCAN4x50 Only otherwise reserved) This is a selfclearing bit. Writing a 1 resets the timer and then the bit clears
    uint32_t nWKRQ_VOLTAGE   : 1; //!< 19    - [Saved when entering sleep mode] nWKRQ Pin GPO buffer voltage rail configuration: '1' =  Vio voltage rail ; '0' = Internal voltage rail
    uint32_t                 : 1; //!< 20
    uint32_t TEST_MODE_EN    : 1; //!< 21    - [Saved when entering sleep mode] Test mode enable. When set device is in test mode: '1' = Enabled ; '0' = Disabled
    uint32_t GPO2_CONFIG     : 2; //!< 22-23 - [Saved when entering sleep mode] GPO2 Pin GPO Configuration
    uint32_t                 : 3; //!< 24-26
    uint32_t CLK_REF         : 1; //!< 27    - CLKIN/Crystal Frequency Reference: '1' = 40MHz ; '0' = 20MHz
    uint32_t WD_TIMER        : 2; //!< 28-29 - [TCAN4550/TCAN4550-Q1 only] Watchdog timer
    uint32_t WAKE_CONFIG     : 2; //!< 30-31 - [Saved when entering sleep mode] Wake pin configuration
  } Bits;
} TCAN455X_MODE_PINS_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_MODE_PINS_Register, 4);

//! Test configuration enumerator
typedef enum
{
  TCAN455X_TEST_PHY_TEST_MAPPED_EXT_PIN       = 0, //!< Phy Test with TXD/RXD_INT_PHY and EN_INT are mapped to external pins
  TCAN455X_TEST_CAN_CONTROLLER_MAPPED_EXT_PIN = 1, //!< CAN Controller test with TXD/RXD_INT_CAN mapped to external pins
} eTCAN455X_TestConfig;

#define TCAN455X_MODE_TEST_CONFIG_Pos          0
#define TCAN455X_MODE_TEST_CONFIG_Mask         (0x1ul << TCAN455X_MODE_TEST_CONFIG_Pos)
#define TCAN455X_MODE_TEST_CONFIG_SET(value)   (((uint32_t)(value) << TCAN455X_MODE_TEST_CONFIG_Pos) & TCAN455X_MODE_TEST_CONFIG_Mask) //!< Set test configuration
#define TCAN455X_MODE_TEST_CONFIG_GET(value)   (((uint32_t)(value) & TCAN455X_MODE_TEST_CONFIG_Mask) >> TCAN455X_MODE_TEST_CONFIG_Pos) //!< Get test configuration

#define TCAN455X_MODE_SLEEP_WAKE_ERROR_DISABLE        (0x1ul << 1) //!< Sleep Wake Error Disable
#define TCAN455X_MODE_SLEEP_WAKE_ERROR_ENABLE         (0x0ul << 1) //!< Sleep Wake Error Enable
#define TCAN455X_MODE_SLEEP_WAKE_ERROR_Mask           TCAN455X_MODE_SLEEP_WAKE_ERROR_DISABLE //!< Sleep Wake Error mask
#define TCAN455X_MODE_DEVICE_RESETS_TO_DEFAULT        (0x1ul << 2) //!< Device resets to default
#define TCAN455X_MODE_NO_RESET_CURRENT_CONFIGURATION  (0x0ul << 2) //!< No Reset to current configuration
#define TCAN455X_MODE_WATCHDOG_ENABLE                 (0x1ul << 3) //!< Watchdog Enable
#define TCAN455X_MODE_WATCHDOG_DISABLE                (0x0ul << 3) //!< Watchdog Disable
#define TCAN455X_MODE_WATCHDOG_Mask                   TCAN455X_MODE_WATCHDOG_ENABLE //!< Watchdog Enable mask
#define TCAN455X_MODE_RESERVED_BIT5                   (0x1ul << 5) //!< When writing to this register, this bit must always be a 1

/*! Mode selection enumerator
 * The Mode of Operation changes the mode but will read back the mode the device is currently in.
 * When the device is changing the device to normal mode a write of 0 to CCCR.INIT is automatically issued
 * and when changing from normal mode to standby or sleep modes a write of 1 to CCCR.INIT is automatically issued
 */
typedef enum
{
  TCAN455X_MODE_SLEEP   = 0x0000 | 0b00, //!< Sleep mode operation
  TCAN455X_MODE_STANDBY = 0x0000 | 0b01, //!< Standby mode operation
  TCAN455X_MODE_NORMAL  = 0x0000 | 0b10, //!< Normal mode operation
  TCAN455X_MODE_TEST    = 0x8000 | 0b01, //!< Test mode operation
} eTCAN455X_Mode;

#define TCAN455X_MODE_OPERATION_Pos         6
#define TCAN455X_MODE_OPERATION_Mask        (0x8003ul << TCAN455X_MODE_OPERATION_Pos)
#define TCAN455X_MODE_OPERATION_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_OPERATION_Pos) & TCAN455X_MODE_OPERATION_Mask) //!< Set mode operation
#define TCAN455X_MODE_OPERATION_GET(value)  (eTCAN455X_Mode)(((uint32_t)(value) & TCAN455X_MODE_OPERATION_Mask) >> TCAN455X_MODE_OPERATION_Pos) //!< Get mode operation

//! nWKRQ configuration enumerator
typedef enum
{
  TCAN455X_nWKRQ_MIRRORS_INH_FUNCTION   = 0, //!< nWKRQ Pin Mirrors INH function
  TCAN455X_nWKRQ_WAKE_REQUEST_INTERRUPT = 1, //!< nWKRQ Pin Wake request interrupt
} eTCAN455X_nWKRQconfig;

#define TCAN455X_MODE_nWKRQ_CONFIG_Pos          8
#define TCAN455X_MODE_nWKRQ_CONFIG_Mask         (0x1ul << TCAN455X_MODE_nWKRQ_CONFIG_Pos)
#define TCAN455X_MODE_nWKRQ_CONFIG_SET(value)   (((uint32_t)(value) << TCAN455X_MODE_nWKRQ_CONFIG_Pos) & TCAN455X_MODE_nWKRQ_CONFIG_Mask) //!< Set nWKRQ configuration
#define TCAN455X_MODE_nWKRQ_CONFIG_GET(value)   (((uint32_t)(value) & TCAN455X_MODE_nWKRQ_CONFIG_Mask) >> TCAN455X_MODE_nWKRQ_CONFIG_Pos) //!< Get nWKRQ configuration

#define TCAN455X_MODE_INH_PIN_DISABLE         (0x1ul << 9) //!< INH Pin Disable
#define TCAN455X_MODE_INH_PIN_ENABLE          (0x0ul << 9) //!< INH Pin Enable
#define TCAN455X_MODE_INH_PIN_Mask            TCAN455X_MODE_INH_PIN_DISABLE //!< INH Pin mask

//! GPIO1 GPO configuration enumerator
typedef enum
{
  TCAN455X_PIN_GPO1_SPI_INTERRUPT     = 0b00, //!< SPI fault Interrupt (Active low). Matches SPIERR if not masked
  TCAN455X_PIN_GPO1_MCAN_INT1         = 0b01, //!< MCAN_INT1 (Active low): m_can_int1
  TCAN455X_PIN_GPO1_UNDERVOLT_THERMAL = 0b10, //!< Under Voltage or Thermal Event Interrupt (Active low): Logical OR of UVccout, UVsup, UVvio. TSD faults that are not masked
  TCAN455X_PIN_GPO1_RESERVED          = 0b11, //!< Reserved
} eTCAN455X_GPO1;

#define TCAN455X_MODE_GPO1_Pos           10
#define TCAN455X_MODE_GPO1_Mask          (0x3ul << TCAN455X_MODE_GPO1_Pos)
#define TCAN455X_MODE_GPO1_SET(value)    (((uint32_t)(value) << TCAN455X_MODE_GPO1_Pos) & TCAN455X_MODE_GPO1_Mask) //!< Set GPO1 mode
#define TCAN455X_MODE_GPO1_GET(value)    (eTCAN455X_GPO1)(((uint32_t)(value) & TCAN455X_MODE_GPO1_Mask) >> TCAN455X_MODE_GPO1_Pos) //!< Get GPO1 mode
#define TCAN455X_MODE_FAIL_SAFE_ENABLE   (0x1ul << 13) //!< Fail safe mode enable
#define TCAN455X_MODE_FAIL_SAFE_DISABLE  (0x0ul << 13) //!< Fail safe mode disable
#define TCAN455X_MODE_FAIL_SAFE_Mask     TCAN455X_MODE_FAIL_SAFE_ENABLE //!< Fail safe mode mask

//! GPIO1 configuration enumerator
typedef enum
{
  TCAN455X_GPIO1_GPO       = 0b00, //!< GPIO1 as GPO
  TCAN455X_GPIO1_RESERVED  = 0b01, //!< Reserved
  TCAN455X_GPIO1_GPI       = 0b10, //!< [TCAN4550/TCAN4550-Q1 only] GPIO1 as GPI – Automatically becomes a WD input trigger pin
  TCAN455X_GPIO1_RESERVED_ = 0b11, //!< Reserved
} eTCAN455X_GPIO1;

#define TCAN455X_MODE_GPIO1_Pos         14
#define TCAN455X_MODE_GPIO1_Mask        (0x3ul << TCAN455X_MODE_GPIO1_Pos)
#define TCAN455X_MODE_GPIO1_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_GPIO1_Pos) & TCAN455X_MODE_GPIO1_Mask) //!< Set GPIO1 mode
#define TCAN455X_MODE_GPIO1_GET(value)  (eTCAN455X_GPIO1)(((uint32_t)(value) & TCAN455X_MODE_GPIO1_Mask) >> TCAN455X_MODE_GPIO1_Pos) //!< Get GPIO1 mode

//! Watchdog action enumerator
typedef enum
{
  TCAN455X_WD_ACTION_SET_INTERRUPT_FLAG = 0b00, //!< Set interrupt flag and if a pin is configure to reflect WD output as an interrupt the pin will show a low
  TCAN455X_WD_ACTION_PULSE_INH_PIN      = 0b01, //!< Pulse INH pin and place device into standby mode – high to low to high ~300ms
  TCAN455X_WD_ACTION_PULSE_WD_OUT_PIN   = 0b10, //!< Pulse watchdog output pin if enabled – high to low to high ~300ms
  TCAN455X_WD_ACTION_RESERVED           = 0b11, //!< Reserved
} eTCAN455X_WDaction;

#define TCAN455X_MODE_WD_ACTION_Pos         16
#define TCAN455X_MODE_WD_ACTION_Mask        (0x3ul << TCAN455X_MODE_WD_ACTION_Pos)
#define TCAN455X_MODE_WD_ACTION_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_WD_ACTION_Pos) & TCAN455X_MODE_WD_ACTION_Mask) //!< Set Watchdog action
#define TCAN455X_MODE_WD_ACTION_GET(value)  (eTCAN455X_WDaction)(((uint32_t)(value) & TCAN455X_MODE_WD_ACTION_Mask) >> TCAN455X_MODE_WD_ACTION_Pos) //!< Get Watchdog action
#define TCAN455X_MODE_RESET_WATCHDOG_TIMER  (0x1ul << 18) //!< Reset watchdog timer

//! nWKRQ voltage reference enumerator
typedef enum
{
  TCAN455X_nWKRQ_Vio_VOLTAGE_RAIL      = 0, //!< nWKRQ Pin GPO buffer Vio voltage rail
  TCAN455X_nWKRQ_INTERNAL_VOLTAGE_RAIL = 1, //!< nWKRQ Pin GPO buffer internal voltage rail
} eTCAN455X_nWKRQvoltRef;

#define TCAN455X_MODE_nWKRQ_VOLTAGE_Pos         19
#define TCAN455X_MODE_nWKRQ_VOLTAGE_Mask        (0x1ul << TCAN455X_MODE_nWKRQ_VOLTAGE_Pos)
#define TCAN455X_MODE_nWKRQ_VOLTAGE_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_nWKRQ_VOLTAGE_Pos) & TCAN455X_MODE_nWKRQ_VOLTAGE_Mask) //!< Set nWKRQ voltage reference
#define TCAN455X_MODE_nWKRQ_VOLTAGE_GET(value)  (((uint32_t)(value) & TCAN455X_MODE_nWKRQ_VOLTAGE_Mask) >> TCAN455X_MODE_nWKRQ_VOLTAGE_Pos) //!< Get nWKRQ voltage reference

#define TCAN455X_MODE_TEST_MODE_ENABLE          (0x1ul << 21) //!< Test mode enable
#define TCAN455X_MODE_TEST_MODE_DISABLE         (0x0ul << 21) //!< Test mode disable

//! GPO2 configuration enumerator
typedef enum
{
  TCAN455X_GPO2_NO_ACTION       = 0b00, //!< GPO2 no action
  TCAN455X_GPO2_MCAN_INT0       = 0b01, //!< GPO2 as MCAN_INT 0 interrupt (Active low): m_can_int0
  TCAN455X_GPO2_WATCHDOG_OUTPUT = 0b10, //!< GPO2 as Watchdog output
  TCAN455X_GPO2_MIRROR_nINT_PIN = 0b11, //!< GPO2 as Mirrors nINT pin (Active low)
} eTCAN455X_GPO2;

#define TCAN455X_MODE_GPO2_Pos          22
#define TCAN455X_MODE_GPO2_Mask         (0x3ul << TCAN455X_MODE_GPO2_Pos)
#define TCAN455X_MODE_GPO2_SET(value)   (((uint32_t)(value) << TCAN455X_MODE_GPO2_Pos) & TCAN455X_MODE_GPO2_Mask) //!< Set GPO2 mode
#define TCAN455X_MODE_GPO2_GET(value)   (eTCAN455X_GPO2)(((uint32_t)(value) & TCAN455X_MODE_GPO2_Mask) >> TCAN455X_MODE_GPO2_Pos) //!< Get GPO2 mode

//! CLKIN frequencies enumerator
typedef enum
{
  TCAN455X_CLKIN_FREQ_20MHz = 0, //!< CLKIN/Crystal Frequency Reference is 20MHz
  TCAN455X_CLKIN_FREQ_40MHz = 1, //!< CLKIN/Crystal Frequency Reference is 40MHz
} eTCAN455X_CLKinFreq;

#define TCAN455X_MODE_CLKIN_FREQ_Pos         27
#define TCAN455X_MODE_CLKIN_FREQ_Mask        (0x1ul << TCAN455X_MODE_CLKIN_FREQ_Pos)
#define TCAN455X_MODE_CLKIN_FREQ_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_CLKIN_FREQ_Pos) & TCAN455X_MODE_CLKIN_FREQ_Mask) //!< Set CLKIN/Crystal Frequency Reference
#define TCAN455X_MODE_CLKIN_FREQ_GET(value)  (((uint32_t)(value) & TCAN455X_MODE_CLKIN_FREQ_Mask) >> TCAN455X_MODE_CLKIN_FREQ_Pos) //!< Get CLKIN/Crystal Frequency Reference

//! Watchdog timer configuration enumerator
typedef enum
{
  TCAN455X_WD_TIMER_60ms  = 0b00, //!< Watchdog timer 60ms
  TCAN455X_WD_TIMER_600ms = 0b01, //!< 600ms
  TCAN455X_WD_TIMER_3s    = 0b10, //!< 3s
  TCAN455X_WD_TIMER_6s    = 0b11, //!< 6s
} eTCAN455X_WDtimer;

#define TCAN455X_MODE_WD_TIMER_Pos         28
#define TCAN455X_MODE_WD_TIMER_Mask        (0x3ul << TCAN455X_MODE_WD_TIMER_Pos)
#define TCAN455X_MODE_WD_TIMER_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_WD_TIMER_Pos) & TCAN455X_MODE_WD_TIMER_Mask) //!< Set Watchdog timer
#define TCAN455X_MODE_WD_TIMER_GET(value)  (eTCAN455X_WDtimer)(((uint32_t)(value) & TCAN455X_MODE_WD_TIMER_Mask) >> TCAN455X_MODE_WD_TIMER_Pos) //!< Get Watchdog timer

//! Wake pin configuration enumerator
typedef enum
{
  TCAN455X_WAKE_PIN_DISABLED     = 0b00, //!< Wake pin Disabled
  TCAN455X_WAKE_PIN_RISING_EDGE  = 0b01, //!< Wake pin Rising edge
  TCAN455X_WAKE_PIN_FALLING_EDGE = 0b10, //!< Wake pin Falling edge
  TCAN455X_WAKE_PIN_BOTH_EDGE    = 0b11, //!< Wake pin Bi-Directional – either edge
} eTCAN455X_WAKEpinConf;

#define TCAN455X_MODE_WAKE_PIN_Pos         30
#define TCAN455X_MODE_WAKE_PIN_Mask        (0x3ul << TCAN455X_MODE_WAKE_PIN_Pos)
#define TCAN455X_MODE_WAKE_PIN_SET(value)  (((uint32_t)(value) << TCAN455X_MODE_WAKE_PIN_Pos) & TCAN455X_MODE_WAKE_PIN_Mask) //!< Set Wake pin configuration
#define TCAN455X_MODE_WAKE_PIN_GET(value)  (eTCAN455X_WAKEpinConf)(((uint32_t)(value) & TCAN455X_MODE_WAKE_PIN_Mask) >> TCAN455X_MODE_WAKE_PIN_Pos) //!< Get Wake pin configuration



//! GPIO1 mode configuration enumerator
typedef enum
{
  TCAN455X_GPI1_WD_TRIGGER_INPUT  = TCAN455X_MODE_GPIO1_SET(TCAN455X_GPIO1_GPI),                                                               //!< [TCAN4550/TCAN4550-Q1 only] GPIO1 as GPI – Automatically becomes a WD input trigger pin
  TCAN455X_GPO1_SPI_INTERRUPT     = TCAN455X_MODE_GPIO1_SET(TCAN455X_GPIO1_GPO) | TCAN455X_MODE_GPO1_SET(TCAN455X_PIN_GPO1_SPI_INTERRUPT),     //!< GPIO1 pin as GPO SPI fault Interrupt (Active low). Matches SPIERR if not masked
  TCAN455X_GPO1_MCAN_INT1         = TCAN455X_MODE_GPIO1_SET(TCAN455X_GPIO1_GPO) | TCAN455X_MODE_GPO1_SET(TCAN455X_PIN_GPO1_MCAN_INT1),         //!< GPIO1 pin as GPO MCAN_INT1 (Active low): m_can_int1
  TCAN455X_GPO1_UNDERVOLT_THERMAL = TCAN455X_MODE_GPIO1_SET(TCAN455X_GPIO1_GPO) | TCAN455X_MODE_GPO1_SET(TCAN455X_PIN_GPO1_UNDERVOLT_THERMAL), //!< GPIO1 pin as GPO Under Voltage or Thermal Event Interrupt (Active low): Logical OR of UVccout, UVsup, UVvio. TSD faults that are not masked
} eTCAN455X_GPIO1Mode;

typedef eTCAN455X_GPO2 eTCAN455X_GPIO2Mode; //!< GPO2 pin mode is the same as GPO2 configuration

//-----------------------------------------------------------------------------

//! TCAN455X Timestamp Prescalar Register (Read/Write, Offset: 0x0804, Reset: 0x00000002)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_TS_PRESCALER_Register
{
  uint32_t TIMESTAMP_PRESCALER;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t PRESCALER:  8; //!< 0- 7 - Writing to this register resets the internal timestamp counter to 0 and will set the internal CAN clock divider used for MCAN Timestamp generation to (Timestamp Prescalar x 8)
    uint32_t          : 24; //!< 8-31
  } Bits;
} TCAN455X_TS_PRESCALER_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_TS_PRESCALER_Register, 4);

#define TCAN455X_TIMESTAMP_PRESCALER_Pos         0
#define TCAN455X_TIMESTAMP_PRESCALER_Mask        (0xFFul << TCAN455X_TIMESTAMP_PRESCALER_Pos)
#define TCAN455X_TIMESTAMP_PRESCALER_SET(value)  (((uint32_t)(value) << TCAN455X_TIMESTAMP_PRESCALER_Pos) & TCAN455X_TIMESTAMP_PRESCALER_Mask) //!< Set Timestamp Prescaler
#define TCAN455X_TIMESTAMP_PRESCALER_GET(value)  (((uint32_t)(value) & TCAN455X_TIMESTAMP_PRESCALER_Mask) >> TCAN455X_TIMESTAMP_PRESCALER_Pos) //!< Get Timestamp Prescaler

//-----------------------------------------------------------------------------

//! TCAN455X Test and Scratch Pad Register (Read/Write, Offset: 0x0808, Reset: 0x00000000)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_TEST_SCRATCH_PAD_Register
{
  uint32_t TEST_SCRATCH_PAD;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t SCRATCH_PAD: 16; //!<  0-15 - Scratch Pad, saved when device is configured for sleep mode
    uint32_t TEST       : 16; //!< 16-31 - Test register, NOT saved when device is configured for sleep mode
  } Bits;
} TCAN455X_TEST_SCRATCH_PAD_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_TEST_SCRATCH_PAD_Register, 4);

#define TCAN455X_TEST_Pos                0
#define TCAN455X_TEST_Mask               (0xFFFFul << TCAN455X_TEST_Pos)
#define TCAN455X_TEST_SET(value)         (((uint32_t)(value) << TCAN455X_TEST_Pos) & TCAN455X_TEST_Mask) //!< Set Scratch Pad
#define TCAN455X_TEST_GET(value)         (((uint32_t)(value) & TCAN455X_TEST_Mask) >> TCAN455X_TEST_Pos) //!< Get Scratch Pad
#define TCAN455X_SCRATCH_PAD_Pos         16
#define TCAN455X_SCRATCH_PAD_Mask        (0xFFFFul << TCAN455X_SCRATCH_PAD_Pos)
#define TCAN455X_SCRATCH_PAD_SET(value)  (((uint32_t)(value) << TCAN455X_SCRATCH_PAD_Pos) & TCAN455X_SCRATCH_PAD_Mask) //!< Set Test register
#define TCAN455X_SCRATCH_PAD_GET(value)  (((uint32_t)(value) & TCAN455X_SCRATCH_PAD_Mask) >> TCAN455X_SCRATCH_PAD_Pos) //!< Get Test register

//-----------------------------------------------------------------------------

//! TCAN455X ECC test Register (Read/Write, Offset: 0x080C, Reset: 0x00000000)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_ECC_TEST_Register
{
  uint32_t ECC_TEST;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t                      : 11; //!<  0-10
    uint32_t ECC_ERR_CHECK        :  1; //!< 11    - ECC error check: '1' = Single Bit ECC error detected ; '0' = = No Single Bit ECC error detected
    uint32_t ECC_ERR_FORCE        :  1; //!< 12    - ECC error force: '1' = Force a single bit ECC error ; '0' = No Force
    uint32_t                      :  3; //!< 13-15
    uint32_t ECC_ERR_FORCE_BIT_SEL:  6; //!< 16-21 - ECC error force bit selection: 0b000000 = Bit 0 ; 0b000001 = Bit 1 ; ... ; 0b100110 = Bit 38 ; All other bit combinations are Reserved
    uint32_t                      : 10; //!< 22-31
  } Bits;
} TCAN455X_ECC_TEST_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_ECC_TEST_Register, 4);

#define TCAN455X_ECC_TEST_SINGLE_BIT_ERROR_DETECTED  (0x1ul << 11) //!< Single Bit ECC error detected
#define TCAN455X_ECC_TEST_FORCE_SINGLE_BIT_ERROR     (0x1ul << 12) //!< Force a single bit ECC error
#define TCAN455X_ECC_ERR_FORCE_BIT_SEL_MAX_VALUE     38
#define TCAN455X_ECC_ERR_FORCE_BIT_SEL_Pos           16
#define TCAN455X_ECC_ERR_FORCE_BIT_SEL_Mask          (0x3Ful << TCAN455X_ECC_ERR_FORCE_BIT_SEL_Pos)
#define TCAN455X_ECC_ERR_FORCE_BIT_SEL_SET(value)    (((uint32_t)(value) << TCAN455X_ECC_ERR_FORCE_BIT_SEL_Pos) & TCAN455X_ECC_ERR_FORCE_BIT_SEL_Mask) //!< Set ECC error force bit selection
#define TCAN455X_ECC_ERR_FORCE_BIT_SEL_GET(value)    (((uint32_t)(value) & TCAN455X_ECC_ERR_FORCE_BIT_SEL_Mask) >> TCAN455X_ECC_ERR_FORCE_BIT_SEL_Pos) //!< Get ECC error force bit selection

//-----------------------------------------------------------------------------

/*! TCAN455X Interrupt Register (Read-Only, Offset: 0x0820, Reset: 0x00100000)
 * @details
 * GLOBALERR: Logical OR of all faults in registers 0x0820-0824.
 * WKRQ: Logical OR of CANINT, LWU and WKERR.
 * CANBUSNOM is not an interrupt but a flag. In normal mode after the first dominant-recessive transition it will set.
 *   It will reset to 0 when entering Standby or Sleep modes or when a bus fault condition takes place in normal mode.
 * CANERR: Logical OR of CANSLNT and CANDOM faults.
 * SPIERR: Will be set if any of the SPI status register 16'h000C[30:16] is set.
 *  - In the event of a SPI underflow, the error is not detected/alerted until the start of the next SPI transaction.
 *  - 16'h0010[30:16] are the mask for these errors
 * VTWD: Logical or of UVCCOUT, UVSUP, UVVIO, TSD, WDTO (Watchdog time out) and ECCERR.
 * CANINT: Indicates a WUP has occurred; Once a CANINT flag is set, LWU events will be ignored. Flag can be cleared by changing to Normal or Sleep modes.
 * LWU: Indicates a local wake event, from toggling the WAKE pin, has occurred. Once a LWU flag is set, CANINT events will be ignored. Flag can be cleared by changing to Normal or Sleep modes.
 * WKERR: If the device receives a wake up request WUP and does not transition to Normal mode or clear the PWRON or Wake flag before tINACTIVE, the device will transition to Sleep Mode.
 *   After the wake event, a Wake Error (WKERR) will be reported and the SMS flag will be set to 1
 */
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_IR_Register
{
  uint32_t IR;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t VTWD     : 1; //!<  0    - Global Voltage, Temp or WDTO Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t M_CAN_INT: 1; //!<  1    - M_CAN global INT Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 1; //!<  2
    uint32_t SPIERR   : 1; //!<  3    - SPI Error Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 1; //!<  4
    uint32_t CANERR   : 1; //!<  5    - CAN Error Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t WKRQ     : 1; //!<  6    - Wake Request Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t GLOBALERR: 1; //!<  7    - Global Error (Any Fault) Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t CANDOM   : 1; //!<  8    - CAN Stuck Dominant Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 1; //!<  9
    uint32_t CANSLNT  : 1; //!< 10    - CAN Silent Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 2; //!< 11-12
    uint32_t WKERR    : 1; //!< 13    - Wake Error Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t LWU      : 1; //!< 14    - Local Wake Up Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t CANINT   : 1; //!< 15    - Can Bus Wake Up Interrupt Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t ECCERR   : 1; //!< 16    - Uncorrectable ECC error detected Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 1; //!< 17
    uint32_t WDTO     : 1; //!< 18    - [TCAN4550/TCAN4550-Q1 only] [Saved when entering sleep mode] Watchdog Time Out Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t TSD      : 1; //!< 19    - [Saved when entering sleep mode] Thermal Shutdown Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t PWRON    : 1; //!< 20    - Power ON Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t UVIO     : 1; //!< 21    - [TCAN4550/TCAN4550-Q1 only] [Saved when entering sleep mode] Under Voltage VIO Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t UVSUP    : 1; //!< 22    - Under Voltage VSUP and UVCC Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t SMS      : 1; //!< 23    - [TCAN4550/TCAN4550-Q1 only] Sleep Mode Status (Flag & Not an interrupt) Only sets when sleep mode is entered by a WKERR, UVIO timeout, or UVIO+TSD fault Interrupt Enable: '1' = Event occured ; '0' = No event occured
    uint32_t          : 7; //!< 24-30
    uint32_t CANBUSNOM: 1; //!< 31    - CAN Bus normal (Flag and Not Interrupt) Will change to 1 when in normal mode after first Dom to Rec transition Interrupt Enable: '1' = Event occured ; '0' = No event occured
  } Bits;
} TCAN455X_IR_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_IR_Register, 4);

#define TCAN455X_IR_VTWD_EN       (0x1ul <<  0) //!< Global Voltage, Temp or WDTO Interrupt Enable
#define TCAN455X_IR_MCAN_INT_EN   (0x1ul <<  1) //!< M_CAN global INT Interrupt Enable
#define TCAN455X_IR_SPIERR_EN     (0x1ul <<  3) //!< SPI Error Interrupt Enable
#define TCAN455X_IR_CANERR_EN     (0x1ul <<  5) //!< CAN Error Interrupt Enable
#define TCAN455X_IR_WKRQ_EN       (0x1ul <<  6) //!< Wake Request Interrupt Enable
#define TCAN455X_IR_GLOBALERR_EN  (0x1ul <<  7) //!< Global Error (Any Fault) Interrupt Enable
#define TCAN455X_IR_CANDOM_EN     (0x1ul <<  8) //!< CAN Stuck Dominant Interrupt Enable
#define TCAN455X_IR_CANSLNT_EN    (0x1ul << 10) //!< CAN Silent Interrupt Enable
#define TCAN455X_IR_WKERR_EN      (0x1ul << 13) //!< Wake Error Interrupt Enable
#define TCAN455X_IR_LWU_EN        (0x1ul << 14) //!< Local Wake Up Interrupt Enable
#define TCAN455X_IR_CANINT_EN     (0x1ul << 15) //!< Can Bus Wake Up Interrupt Interrupt Enable
#define TCAN455X_IR_ECCERR_EN     (0x1ul << 16) //!< Uncorrectable ECC error detected Interrupt Enable
#define TCAN455X_IR_WDTO_EN       (0x1ul << 18) //!< Watchdog Time Out Interrupt Enable
#define TCAN455X_IR_TSD_EN        (0x1ul << 19) //!< Thermal Shutdown Interrupt Enable
#define TCAN455X_IR_PWRON_EN      (0x1ul << 20) //!< Power ON Interrupt Enable
#define TCAN455X_IR_UVIO_EN       (0x1ul << 21) //!< Under Voltage VIO Interrupt Enable
#define TCAN455X_IR_UVSUP_EN      (0x1ul << 22) //!< Under Voltage VSUP and UVCC Interrupt Enable
#define TCAN455X_IR_SMS_EN        (0x1ul << 23) //!< Sleep Mode Status Interrupt Enable
#define TCAN455X_IR_CANBUSNOM_EN  (0x1ul << 31) //!< CAN Bus normal Interrupt Enable


#define TCAN455X_IR_CLEARABLE_EVENTS_FLAGS  ( TCAN455X_IR_CANDOM_EN | TCAN455X_IR_CANSLNT_EN | TCAN455X_IR_WKERR_EN | TCAN455X_IR_LWU_EN | \
                                              TCAN455X_IR_CANINT_EN | TCAN455X_IR_ECCERR_EN  | TCAN455X_IR_WDTO_EN  | TCAN455X_IR_TSD_EN | \
                                              TCAN455X_IR_PWRON_EN  | TCAN455X_IR_UVIO_EN    | TCAN455X_IR_UVSUP_EN )                        //!< All TCAN455X clearable events flags
#define TCAN455X_IR_EVENTS_STATUS_FLAGS  ( TCAN455X_IR_VTWD_EN   | TCAN455X_IR_MCAN_INT_EN  | TCAN455X_IR_SPIERR_EN  | TCAN455X_IR_CANERR_EN  | \
                                           TCAN455X_IR_WKRQ_EN   | TCAN455X_IR_GLOBALERR_EN | TCAN455X_IR_CANDOM_EN  | TCAN455X_IR_CANSLNT_EN | \
                                           TCAN455X_IR_WKERR_EN  | TCAN455X_IR_LWU_EN       | TCAN455X_IR_CANINT_EN  | TCAN455X_IR_ECCERR_EN  | \
                                           TCAN455X_IR_WDTO_EN   | TCAN455X_IR_TSD_EN       | TCAN455X_IR_PWRON_EN   | TCAN455X_IR_UVIO_EN    | \
                                           TCAN455X_IR_UVSUP_EN  | TCAN455X_IR_SMS_EN       | TCAN455X_IR_CANBUSNOM_EN )                     //!< All TCAN455X events status flags

//! Interrupt Events, can be OR'ed.
typedef enum
{
  TCAN455X_INT_NO_EVENT                       = 0x00000000,               //!< No interrupt events
  TCAN455X_INT_GLOBAL_VOLTAGE_TEMP_WDTO_EVENT = TCAN455X_IR_VTWD_EN,      //!< Global Voltage, Temp or WDTO event
  TCAN455X_INT_MCAN_GLOBAL_INT_EVENT          = TCAN455X_IR_MCAN_INT_EN,  //!< M_CAN global INT event
  TCAN455X_INT_SPI_ERROR_EVENT                = TCAN455X_IR_SPIERR_EN,    //!< SPI Error event
  TCAN455X_INT_CAN_ERROR_EVENT                = TCAN455X_IR_CANERR_EN,    //!< CAN Error event
  TCAN455X_INT_WAKE_REQUEST_EVENT             = TCAN455X_IR_WKRQ_EN,      //!< Wake Request event
  TCAN455X_INT_GLOBAL_ERROR_EVENT             = TCAN455X_IR_GLOBALERR_EN, //!< Global Error (Any Fault) event
  TCAN455X_INT_CAN_STUCK_DOMINANT_EVENT       = TCAN455X_IR_CANDOM_EN,    //!< CAN Stuck Dominant event
  TCAN455X_INT_CAN_SILENT_EVENT               = TCAN455X_IR_CANSLNT_EN,   //!< CAN Silent event
  TCAN455X_INT_WAKE_ERROR_EVENT               = TCAN455X_IR_WKERR_EN,     //!< Wake Error events
  TCAN455X_INT_LOCAL_WAKE_UP_EVENT            = TCAN455X_IR_LWU_EN,       //!< Local Wake Up events
  TCAN455X_INT_CAN_BUS_WAKE_UP_EVENT          = TCAN455X_IR_CANINT_EN,    //!< CAN Bus Wake Up events
  TCAN455X_INT_UNCORRECTABLE_ECC_ERROR_EVENT  = TCAN455X_IR_ECCERR_EN,    //!< Uncorrectable ECC error detected events
  TCAN455X_INT_WATCHDOG_TIMEOUT_EVENT         = TCAN455X_IR_WDTO_EN,      //!< [TCAN4550/TCAN4550-Q1 only] Watchdog Time Out events
  TCAN455X_INT_THERMAL_SHUTDOWN_EVENT         = TCAN455X_IR_TSD_EN,       //!< Thermal Shutdown events
  TCAN455X_INT_POWER_ON_EVENT                 = TCAN455X_IR_PWRON_EN,     //!< Power ON events
  TCAN455X_INT_UNDERVOLTAGE_VIO_EVENT         = TCAN455X_IR_UVIO_EN,      //!< [TCAN4550/TCAN4550-Q1 only] Under Voltage VIO events
  TCAN455X_INT_UNDERVOLTAGE_VSUP_EVENT        = TCAN455X_IR_UVSUP_EN,     //!< Under Voltage VSUP events
  TCAN455X_INT_SLEEP_MODE_STATUS_EVENT        = TCAN455X_IR_SMS_EN,       //!< [TCAN4550/TCAN4550-Q1 only] Sleep Mode Status events
  TCAN455X_INT_CAN_BUS_NORMAL_EVENT           = TCAN455X_IR_CANBUSNOM_EN, //!< CAN Bus normal events

  TCAN455X_INT_CLEARABLE_FLAG_EVENTS          = TCAN455X_IR_CLEARABLE_EVENTS_FLAGS, //!< Clearable flag events
  TCAN455X_INT_ENABLE_ALL_EVENTS              = TCAN455X_IR_EVENTS_STATUS_FLAGS,    //!< Enable all events
  TCAN455X_INT_EVENTS_STATUS_FLAGS_MASK       = TCAN455X_IR_EVENTS_STATUS_FLAGS,    //!< Events flags mask
} eTCAN455X_InterruptEvents;

typedef eTCAN455X_InterruptEvents setTCAN455X_InterruptEvents; //!< Set of TCAN455X Interrupt Events (can be OR'ed)

//-----------------------------------------------------------------------------

/*! TCAN455X MCAN Interrupt Register (Read-Only, Offset: 0x0824, Reset: 0x00000000)
 * These flags are a mirror of the MCAN Interrupt Register (Read/Write, Offset: 0x50, Absolute: 0x1050)
 */
typedef MCAN_IR_Register TCAN455X_IR_MCAN_Register;

//-----------------------------------------------------------------------------

//! TCAN455X Interrupt Enable Register (Read/Write, Offset: 0x0830, Reset: 0xFFFFFFFF)
TCAN455X_PACKITEM
typedef union __TCAN455X_PACKED__ TCAN455X_IE_Register
{
  uint32_t IE;
  uint8_t Bytes[sizeof(uint32_t)];
  struct
  {
    uint32_t        : 8; //!<  0- 7
    uint32_t CANDOM : 1; //!<  8    - CAN Stuck Dominant Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t        : 1; //!<  9
    uint32_t CANSLNT: 1; //!< 10    - CAN Silent Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t        : 3; //!< 11-13
    uint32_t LWU    : 1; //!< 14    - [Saved when entering sleep mode] Local Wake Up Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t CANINT : 1; //!< 15    - [Saved when entering sleep mode] CAN Bus Wake Up Interrupt Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t ECCERR : 1; //!< 16    - Uncorrectable ECC error detected Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t        : 2; //!< 17-18
    uint32_t TSD    : 1; //!< 19    - Thermal Shutdown Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t        : 1; //!< 20
    uint32_t UVIO   : 1; //!< 21    - [TCAN4550/TCAN4550-Q1 only] Under Voltage VIO Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t UVSUP  : 1; //!< 22    - Under Voltage VSUP and UVCC Interrupt Enable: '1' = Enables the corresponding interrupt ; '0' = Disables the corresponding interrupt
    uint32_t        : 9; //!< 23-31
  } Bits;
} TCAN455X_IE_Register;
TCAN455X_UNPACKITEM;
TCAN455X_CONTROL_ITEM_SIZE(TCAN455X_IE_Register, 4);

#define TCAN455X_IE_CANDOM_EN   (0x1ul <<  8) //!< CAN Stuck Dominant Interrupt Enable
#define TCAN455X_IE_CANSLNT_EN  (0x1ul << 10) //!< CAN Silent Interrupt Enable
#define TCAN455X_IE_LWU_EN      (0x1ul << 14) //!< Local Wake Up Interrupt Enable
#define TCAN455X_IE_CANINT_EN   (0x1ul << 15) //!< CAN Bus Wake Up Interrupt Interrupt Enable
#define TCAN455X_IE_ECCERR_EN   (0x1ul << 16) //!< Uncorrectable ECC error detected Interrupt Enable
#define TCAN455X_IE_TSD_EN      (0x1ul << 19) //!< Thermal Shutdown Interrupt Enable
#define TCAN455X_IE_UVIO_EN     (0x1ul << 21) //!< [TCAN4550/TCAN4550-Q1 only] Under Voltage VIO Interrupt Enable
#define TCAN455X_IE_UVSUP_EN    (0x1ul << 22) //!< Under Voltage VSUP and UVCC Interrupt Enable

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN455X Driver API
//********************************************************************************************************************

#define TCAN455X_GetSIDfilterElementCount(pComp)   ( MCAN_FILTER_SID_SIZE_GET((pComp)->_MCAN.InternalConfig) ) //!< Get the SID filter elements count configured for the pComp
#define TCAN455X_GetEIDfilterElementCount(pComp)   ( MCAN_FILTER_EID_SIZE_GET((pComp)->_MCAN.InternalConfig) ) //!< Get the EID filter elements count configured for the pComp

#define TCAN455X_GetSIDfilterTotalByteSize(pComp)  ( TCAN455X_GetSIDfilterElementCount(pComp) * MCAN_CAN_STANDARD_FILTER_SIZE ) //!< Get the SID filter total byte size
#define TCAN455X_GetEIDfilterTotalByteSize(pComp)  ( TCAN455X_GetEIDfilterElementCount(pComp) * MCAN_CAN_EXTENDED_FILTER_SIZE ) //!< Get the EID filter total byte size

#define TCAN455X_CRYSTAL_TO_FREQ(enumerator)  ( ((uint32_t)(enumerator) + 1u) * 20000000ul ) //!< Convert the #eTCAN455X_CLKinFreq enum value to frequency

//-----------------------------------------------------------------------------

//! Device power states
typedef enum
{
  TCAN455X_DEVICE_SLEEP_NOT_CONFIGURED = 0x0, //!< Device sleep mode is not configured so the device is in normal power state
  TCAN455X_DEVICE_NORMAL_POWER_STATE   = 0x1, //!< Device is in normal power state
  TCAN455X_DEVICE_SLEEP_STATE          = 0x2, //!< Device is in sleep power state
  TCAN455X_DEVICE_LOWPOWER_SLEEP_STATE = 0x3, //!< Device is in low-power sleep power state
} eTCAN455X_PowerStates;

#define TCAN455X_DEV_PS_Pos         0
#define TCAN455X_DEV_PS_Mask        (0x3ul << TCAN455X_DEV_PS_Pos)
#define TCAN455X_DEV_PS_SET(value)  (((uint32_t)(value) << TCAN455X_DEV_PS_Pos) & TCAN455X_DEV_PS_Mask)                        //!< Set Device Power State
#define TCAN455X_DEV_PS_GET(value)  (eTCAN455X_PowerStates)(((uint32_t)(value) & TCAN455X_DEV_PS_Mask) >> TCAN455X_DEV_PS_Pos) //!< Get Device Power State

#define TCAN455X_DEV_ID_Pos         2
#define TCAN455X_DEV_ID_Mask        (0x1ul << TCAN455X_DEV_ID_Pos)
#define TCAN455X_DEV_ID_SET(value)  (((uint8_t)(value) << TCAN455X_DEV_ID_Pos) & TCAN455X_DEV_ID_Mask) //!< Set Device ID
#define TCAN455X_DEV_ID_GET(value)  (eTCAN455X_Devices)(((uint8_t)(value) & TCAN455X_DEV_ID_Mask) >> TCAN455X_DEV_ID_Pos) //!< Get Device ID

//-----------------------------------------------------------------------------

typedef struct TCAN455X TCAN455X;        //!< Typedef of TCAN455X device object structure
typedef uint8_t TTCAN455XDriverInternal; //!< Alias for Driver Internal data flags

//-----------------------------------------------------------------------------

/*! @brief Function that gives the current millisecond of the system to the driver
 *
 * This function will be called when the driver needs to get current millisecond
 * @return Returns the current millisecond of the system
 */
typedef uint32_t (*GetCurrentms_Func)(void);

//-----------------------------------------------------------------------------

//! TCAN455X device object structure
struct TCAN455X
{
  void *UserDriverData;                   //!< Optional, can be used to store driver data or NULL

  //--- Driver configuration ---
  setTCAN455X_DriverConfig DriverConfig;  //!< Driver configuration, by default it is TCAN455X_DRIVER_NORMAL_USE. Configuration can be OR'ed
  TTCAN455XDriverInternal InternalConfig; //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION

  //--- MCAN interface ---
  MCAN_Interface _MCAN;                   //!< DO NOT USE OR CHANGE THIS VALUE, WILL BE FILLED AT INIT WITH PROPER VALUES. This is the MCAN_Interface descriptor that will be used to communicate with the MCAN core
  uint32_t __RegCache[MCAN_CACHE_COUNT];  //!< DO NOT USE OR CHANGE THESE VALUES. This is a cache for some register's configuration to avoid getting FIFOs/Buffers configuration each time the driver transmit/receive frames

  //--- Interface driver call functions ---
  uint8_t SPIchipSelect;                  //!< This is the Chip Select index that will be set at the call of a transfer
#ifdef USE_DYNAMIC_INTERFACE
  SPI_Interface* SPI;                     //!< This is the SPI_Interface descriptor pointer that will be used to communicate with the device
#else //#if !defined(USE_DYNAMIC_INTERFACE)
  SPI_Interface SPI;                      //!< This is the SPI_Interface descriptor that will be used to communicate with the device
#endif
  uint32_t SPIclockSpeed;                 //!< Clock frequency of the SPI interface in Hertz

  //--- Time call function ---
  GetCurrentms_Func fnGetCurrentms;       //!< This function will be called when the driver need to get current millisecond
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define TCAN455X_UNIQUE_ID  ( (((uint32_t)'T' << 0) ^ ((uint32_t)'C' << 4) ^ ((uint32_t)'A' << 8) ^ ((uint32_t)'N' << 12) ^ ((uint32_t)'4' << 16) ^ ((uint32_t)'5' << 20) ^ ((uint32_t)'5' << 24) ^ ((uint32_t)'X' << 28)) + (sizeof(struct TCAN455X) << 19) )

//-----------------------------------------------------------------------------

//! TCAN455X Controller and CAN configuration structure
typedef struct TCAN455X_Config
{
  //--- Controller clocks ---
  eTCAN455X_CLKinFreq XtalFreq;                  //!< Component CLKIN Xtal/Resonator frequency
  bool FailSafeEnable;                           //!< Fail safe enable: set to 'true' to enable, else 'false'
  bool SleepWakeDisable;                         //!< Sleep Wake Error Disable: set to 'true' to disable the Sleep Wake Error, else 'false'. See TCAN455X_MODE_PINS_Register.SWE_DIS for more informations

  //--- Watchdog Timer ---
  bool EnableWatchdog;                           //!< Set to 'true to enable the watchdog, else 'false'
  eTCAN455X_WDtimer WatchdogTimeout;             //!< Watchdog timeout value
  eTCAN455X_WDaction WDtimeoutAction;            //!< Watchdog action when timeout

  //--- CAN configuration ---
  MCAN_Config MCANconf;                          //!< MCAN core configuration structure

  //--- GPIOs and Interrupts pins ---
  eTCAN455X_GPIO1Mode GPIO1PinMode;              //!< Startup INT1/WDtrig/UDV pins mode
  eTCAN455X_GPIO2Mode GPIO2PinMode;              //!< Startup INT0/WDout/nINT pins mode
  eTCAN455X_WAKEpinConf WakePinConf;             //!< Define the Wake pin configuration
  bool INHpinEnable;                             //!< INH pin enable: set to 'true' to enable the use of the INH pin, else 'false'
  eTCAN455X_nWKRQconfig nWKRQconfig;             //!< Define the nWKRQ configuration
  eTCAN455X_nWKRQvoltRef nWKRQvoltageRef;        //!< Define the nWKRQ voltage reference

  //--- Interrupts ---
  setTCAN455X_InterruptEvents SysInterruptFlags; //!< Set of system interrupt flags to enable. Configuration can be OR'ed
} TCAN455X_Config;

//-----------------------------------------------------------------------------





//********************************************************************************************************************


/*! @brief MCAN on SAMV71 peripheral initialization
 *
 * This function initializes the MCAN driver and call the initialization of the interface driver (SPI). It checks parameters and perform a RESET
 * Next this function configures the MCAN Controller and the CAN controller
 * @param[in] *pComp Is the pointed structure of the peripheral to be initialized
 * @param[in] *pConf Is the pointed structure of the peripheral configuration
 * @param[in] *listFIFO Is the FIFO/Buffers list to configure in RAM
 * @param[in] *count Is the element count of the listFIFO array
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_TCAN455X(TCAN455X *pComp, const TCAN455X_Config* const pConf, const MCAN_FIFObuff* const listFIFO, size_t listFIFOcount);

//********************************************************************************************************************


/*! @brief Read from a 32-bits register of MCAN in TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address to be read
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ReadMCANREG32(void *pComp, const uint32_t address, uint32_t* data);

/*! @brief Write to a 32-bits register of MCAN in TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address where data will be written
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_WriteMCANREG32(void *pComp, const uint32_t address, const uint32_t data);


/*! @brief Read from a 32-bits register of TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address to be read
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ReadREG32(TCAN455X *pComp, const uint32_t address, uint32_t* data);

/*! @brief Write to a 32-bits register of the TCAN455X
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address where data will be written
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_WriteREG32(TCAN455X *pComp, const uint32_t address, const uint32_t data);

/*! @brief Modify a 32-bits register of the TCAN455X
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] registerAddr Is the register address where data will be written
 * @param[in] registerValue Is the data to write
 * @param[in] registerMask If the bit is set to '1', then the corresponding register's bit have to be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ModifyREG32(TCAN455X *pComp, const uint32_t registerAddr, uint32_t registerValue, uint32_t registerMask);


/*! @brief Read from memory data of TCAN455X device
 *
 * @param[in] *_pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address to be read
 * @param[out] *data Is where the data will be stored
 * @param[in] count Is the byte count to read (multiple of 4 bytes)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ReadRAM(void *_pComp, const uint32_t address, uint8_t* data, const uint16_t count);

/*! @brief Read a word data (4 bytes) from a RAM address of the TCAN455X
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the address where data will be read in the TCAN455X
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ReadRAM32(TCAN455X *pComp, const uint32_t address, uint32_t* const data)
{
  if (data == NULL) return ERR__PARAMETER_ERROR;
  TCAN455X_uint32t_Conv Tmp;
  eERRORRESULT Error = TCAN455X_ReadRAM(pComp, address, &Tmp.Bytes[0], sizeof(Tmp));
  *data = Tmp.Uint32;
  return Error;
}

/*! @brief Write to memory data of TCAN455X device
 *
 * @param[in] *_pComp Is the pointed structure of the device to be used
 * @param[in] address Is the register address where data will be written
 * @param[in] *data Is the data to write
 * @param[in] count Is the byte count to read (multiple of 4 bytes)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_WriteRAM(void *_pComp, const uint32_t address, const uint8_t* data, const uint16_t count);

/*! @brief Write a word data (4 bytes) to a RAM register of the TCAN455X
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] address Is the address where data will be written in the TCAN455X
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WriteRAM32(TCAN455X *pComp, const uint32_t address, const uint32_t data)
{
  TCAN455X_uint32t_Conv Tmp;
  Tmp.Uint32 = data;
  eERRORRESULT Error = TCAN455X_WriteRAM(pComp, address, &Tmp.Bytes[0], sizeof(Tmp));
  return Error;
}

//********************************************************************************************************************


/*! @brief Reset the TCAN455X device
 *
 * @warning The device will reset directly, this means the MCAN will not be put in a proper mode for this.
 * Call #MCANV71_RequestOperationMode() with the #MCAN_INITIALIZATION_MODE mode before calling this function
 * @param[in] *pComp Is the pointed structure of the device to reset
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ResetDevice(TCAN455X *pComp)
{
  return TCAN455X_WriteREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_DEVICE_RESETS_TO_DEFAULT);
}

//********************************************************************************************************************


/*! @brief Get actual device of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the peripheral
 * @param[out] *device Is the device connected to SPI
 * @param[out] *revIdMajor Is the device's revision Major ID
 * @param[out] *revIdMinor Is the device's revision Minor ID
 * @param[out] *revSPI Is the SPI's revision
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_GetDeviceID(TCAN455X *pComp, eTCAN455X_Devices* const device, uint8_t* const revIdMajor, uint8_t* const revIdMinor, uint8_t* const revSPI);

//********************************************************************************************************************


/*! @brief Get actual operation mode of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[out] *actualMode Is where the result of the actual mode will be saved
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_GetTCAN455XactualOperationMode(TCAN455X *pComp, eTCAN455X_Mode* const actualMode)
{
#ifdef CHECK_NULL_PARAM
  if (actualMode == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Value;
  eERRORRESULT Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, &Value); // Read the device mode register
  *actualMode = TCAN455X_MODE_OPERATION_GET(Value);                                       // Get device's actual mode
  return Error;
}

/*! @brief Request operation mode change of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] newMode Is the new operational mode to set
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_RequestTCAN455XoperationMode(TCAN455X *pComp, eTCAN455X_Mode newMode)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_OPERATION_SET(newMode);
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_OPERATION_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

/*! @brief Start the MCAN peripheral in Standby mode
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_SetStandby(TCAN455X *pComp)
{
  return TCAN455X_RequestTCAN455XoperationMode(pComp, TCAN455X_MODE_STANDBY);
}

/*! @brief Start the MCAN peripheral in Normal mode
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_Start(TCAN455X *pComp)
{
  return TCAN455X_RequestTCAN455XoperationMode(pComp, TCAN455X_MODE_NORMAL);
}

//********************************************************************************************************************


/*! @brief Configure pins of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] gpio1PinMode Is the configuration of the GPIO1 pin
 * @param[in] gpio2PinMode Is the configuration of the GPIO2 pin
 * @param[in] wakePinConf Is the configuration of the WAKE pin
 * @param[in] inhPinEnable Is the configuration of the INH pin
 * @param[in] nWKRQconfig Is the configuration of the nWKRQ pin
 * @param[in] nWKRQvoltageRef Is the voltage reference of the nWKRQ pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ConfigurePins(TCAN455X *pComp, eTCAN455X_GPIO1Mode gpio1PinMode, eTCAN455X_GPIO2Mode gpio2PinMode, eTCAN455X_WAKEpinConf wakePinConf, bool inhPinEnable, eTCAN455X_nWKRQconfig nWKRQconfig, eTCAN455X_nWKRQvoltRef nWKRQvoltageRef);

//********************************************************************************************************************


/*! @brief Configure Device SPI Status Events of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] events Is the set of events to set
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ConfigureDeviceSPIstatusEvents(TCAN455X *pComp, setTCAN455X_DeviceStatusEvents events)
{
  return TCAN455X_WriteREG32(pComp, RegTCAN455X_SPI_MASK, ((uint32_t)events & TCAN455X_CLEARABLE_DEVICE_STATUS_EVENT_MASK)); // Write the device SPI status register
}

/*! @brief Get Device Status Events of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] events Is where the set of events will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_GetDeviceStatusEvents(TCAN455X *pComp, setTCAN455X_DeviceStatusEvents* const events)
{
#ifdef CHECK_NULL_PARAM
  if (events == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Value;
  eERRORRESULT Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_STATUS, &Value);            // Read the device status register
  *events = (setTCAN455X_DeviceStatusEvents)(Value & TCAN455X_DEVICE_STATUS_EVENT_MASK); // Get device status flag status
  return Error;
}

/*! @brief Clear Device Status Events of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] events Is the set of events to clear
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ClearDeviceStatusEvents(TCAN455X *pComp, setTCAN455X_DeviceStatusEvents events)
{
  return TCAN455X_WriteREG32(pComp, RegTCAN455X_STATUS, ((uint32_t)events & TCAN455X_CLEARABLE_DEVICE_STATUS_EVENT_MASK)); // Write the device status register
}

/*! @brief Configure interrupt of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be enabled. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ConfigureTCAN455Xinterrupt(TCAN455X *pComp, setTCAN455X_InterruptEvents interruptsFlags)
{
  return TCAN455X_WriteREG32(pComp, RegTCAN455X_DEV_IE, (uint32_t)interruptsFlags); // Write the device interrupt register
}

/*! @brief Get interrupt events of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *data Is where the interrupts events will be stored. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_GetTCAN455XinterruptEvents(TCAN455X *pComp, setTCAN455X_InterruptEvents* const data)
{
#ifdef CHECK_NULL_PARAM
  if (data == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Value;
  eERRORRESULT Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_DEV_IR, &Value); // Read the device status register
  if (Error != ERR_NONE) return Error;                                        // If there is an error while calling TCAN455X_ReadREG32() then return the error
  *data = (setTCAN455X_InterruptEvents)(Value);                               // Get device status flag status
  return Error;
}

/*! @brief Clear interrupt events of the TCAN455X peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be cleared. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ClearTCAN455XinterruptEvents(TCAN455X *pComp, setMCAN_InterruptEvents interruptsFlags)
{
  return TCAN455X_WriteREG32(pComp, RegTCAN455X_DEV_IR, ((uint32_t)interruptsFlags & TCAN455X_IR_CLEARABLE_EVENTS_FLAGS)); // Write flags to clear to the device
}

//********************************************************************************************************************


/*! @brief Configure ECC tests of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] forceSingleBitError Set 'true' to force single bit error, else set 'false' to disable the test
 * @param[in] forceBitSel Is the ECC bit error value to force (Min = 0 ; Max = 38)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT TCAN455X_ConfigureTestECC(TCAN455X *pComp, bool forceSingleBitError, uint8_t forceBitSel);

/*! @brief Get ECC tests single bit error of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *detected Is 'true' if a single bit error is detected, else 'false'
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_TestECCsingleBitErrorDetected(TCAN455X *pComp, bool* detected)
{
#ifdef CHECK_NULL_PARAM
  if (detected == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Value;
  eERRORRESULT Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_DEV_TEST_SCRATCH_PAD, &Value); // Read the device status register
  *detected = ((Value & TCAN455X_ECC_TEST_SINGLE_BIT_ERROR_DETECTED) > 0);                  // Get single bit error detected status
  return Error;
}

//********************************************************************************************************************


/*! @brief Configure the test mode in the TCAN455X peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] testConfig Is the configuration of the test mode
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ConfigureTCAN455Xtest(TCAN455X *pComp, eTCAN455X_TestConfig testConfig)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_TEST_CONFIG_SET(testConfig);
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_TEST_CONFIG_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

//********************************************************************************************************************


/*! @brief Configure the watchdog timer of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] watchdogTimeout Is the watchdog timeout value
 * @param[in] timeoutAction Is the watchdog action when timeout
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WDT_Configuration(TCAN455X *pComp, eTCAN455X_WDtimer watchdogTimeout, eTCAN455X_WDaction timeoutAction)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WD_TIMER_SET(watchdogTimeout) | TCAN455X_MODE_WD_ACTION_SET(timeoutAction);
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WD_TIMER_Mask | TCAN455X_MODE_WD_ACTION_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

/*! @brief Enable the watchdog timer of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WDT_Enable(TCAN455X *pComp)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WATCHDOG_ENABLE;
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WATCHDOG_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

/*! @brief Disable the watchdog timer of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WDT_Disable(TCAN455X *pComp)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WATCHDOG_DISABLE;
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_WATCHDOG_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

/*! @brief Reset the watchdog timer of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WDT_Reset(TCAN455X *pComp)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_RESET_WATCHDOG_TIMER;
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_RESET_WATCHDOG_TIMER;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

//********************************************************************************************************************


/*! @brief Read the scratchpad register of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_ReadScratchPadRegister(TCAN455X *pComp, uint16_t* const data)
{
#ifdef CHECK_NULL_PARAM
  if (data == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Value;
  eERRORRESULT Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_DEV_TEST_SCRATCH_PAD, &Value); // Read the device status register
  *data = TCAN455X_SCRATCH_PAD_GET(Value);                                                  // Get device status flag status
  return Error;
}

/*! @brief Write the scratchpad register of the TCAN455X device
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT TCAN455X_WriteCustomerRegister(TCAN455X *pComp, const uint16_t data)
{
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_TEST_SCRATCH_PAD, TCAN455X_SCRATCH_PAD_SET(data), TCAN455X_SCRATCH_PAD_Mask); // Modify Scratch Pad register;
}



//********************************************************************************************************************
// MCAN links Driver API
//********************************************************************************************************************

inline eERRORRESULT TCAN455X_CheckEndianness(TCAN455X *pComp) //!< @see #MCAN_CheckEndianness
{ return MCAN_CheckEndianness(&pComp->_MCAN); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_GetMCANcoreID(TCAN455X *pComp, uint8_t* const mcanCoreId, uint8_t* const mcanStep, uint8_t* const mcanSubStep, uint8_t* const mcanYear, uint8_t* const mcanMonth, uint8_t* const mcanDay) //!< @see #MCAN_GetMCANcoreID
{ return MCAN_GetMCANcoreID(&pComp->_MCAN, mcanCoreId, mcanStep, mcanSubStep, mcanYear, mcanMonth, mcanDay); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ReadMCANcustomerRegister(TCAN455X *pComp, uint32_t* const data) //!< @see #MCAN_ReadCustomerRegister
{ return MCAN_ReadCustomerRegister(&pComp->_MCAN, data); };
inline eERRORRESULT TCAN455X_WriteMCANcustomerRegister(TCAN455X *pComp, const uint32_t data) //!< @see #MCAN_WriteCustomerRegister
{ return MCAN_WriteCustomerRegister(&pComp->_MCAN, data); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_TransmitMessageObject(TCAN455X *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize, eMCAN_FIFObuffer toFIFObuff, uint8_t index) //!< @see #MCAN_TransmitMessageObject
{ return MCAN_TransmitMessageObject(&pComp->_MCAN, messageObjectToSend, objectSize, toFIFObuff, index); };
inline eERRORRESULT TCAN455X_TransmitMessageObjectToTXQ(TCAN455X *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize) //!< @see #MCAN_TransmitMessageObjectToTXQ
{ return MCAN_TransmitMessageObjectToTXQ(&pComp->_MCAN, messageObjectToSend, objectSize); };
inline eERRORRESULT TCAN455X_TransmitMessage(TCAN455X *pComp, const CAN_CANMessage* const messageToSend, eMCAN_FIFObuffer toFIFObuff, uint8_t index) //!< @see #MCAN_TransmitMessage
{ return MCAN_TransmitMessage(&pComp->_MCAN, messageToSend, toFIFObuff, index); };
inline eERRORRESULT TCAN455X_TransmitMessageToTXQ(TCAN455X *pComp, const CAN_CANMessage* const messageToSend) //!< @see #MCAN_TransmitMessageToTXQ
{ return MCAN_TransmitMessageToTXQ(&pComp->_MCAN, messageToSend); };

inline eERRORRESULT TCAN455X_ReceiveMessageObject(TCAN455X *pComp, uint8_t* const messageObjectGet, uint8_t objectSize, eMCAN_FIFObuffer fromFIFObuff, uint8_t index) //!< @see #MCAN_ReceiveMessageObject
{ return MCAN_ReceiveMessageObject(&pComp->_MCAN, messageObjectGet, objectSize, fromFIFObuff, index); };
inline eERRORRESULT TCAN455X_ReceiveMessageObjectFromTEF(TCAN455X *pComp, uint8_t* const messageObjectGet, uint8_t objectSize) //!< @see #MCAN_ReceiveMessageObjectFromTEF
{ return MCAN_ReceiveMessageObjectFromTEF(&pComp->_MCAN, messageObjectGet, objectSize); };
inline eERRORRESULT TCAN455X_ReceiveMessage(TCAN455X *pComp, CAN_CANMessage* const messageGet, eMCAN_PayloadSize payloadSize, uint32_t* const timeStamp, uint8_t* const filterHitIdx, eMCAN_FIFObuffer fromFIFObuff, uint8_t index) //!< @see #MCAN_ReceiveMessage
{ return MCAN_ReceiveMessage(&pComp->_MCAN, messageGet, payloadSize, timeStamp, filterHitIdx, fromFIFObuff, index); };
inline eERRORRESULT TCAN455X_ReceiveMessageFromTEF(TCAN455X *pComp, CAN_CANMessage* const messageGet, uint32_t* const timeStamp, uint8_t* const filterHitIdx) //!< @see #MCAN_ReceiveMessageFromTEF
{ return MCAN_ReceiveMessageFromTEF(&pComp->_MCAN, messageGet, timeStamp, filterHitIdx); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureWatchdogRAM(TCAN455X *pComp, uint8_t messageRAMwatchdogConf) //!< @see #MCAN_ConfigureWatchdogRAM
{ return MCAN_ConfigureWatchdogRAM(&pComp->_MCAN, messageRAMwatchdogConf); };
inline eERRORRESULT TCAN455X_GetWatchdogRAMvalue(TCAN455X *pComp, uint8_t* const messageRAMwatchdogValue) //!< @see #MCAN_GetWatchdogRAMvalue
{ return MCAN_GetWatchdogRAMvalue(&pComp->_MCAN, messageRAMwatchdogValue); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureINTlines(TCAN455X *pComp, bool enableLineINT0, bool enableLineINT1) //!< @see #MCAN_ConfigureINTlines
{ return MCAN_ConfigureINTlines(&pComp->_MCAN, enableLineINT0, enableLineINT1); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_SetBitTimeConfiguration(TCAN455X *pComp, const CAN_BitTimeConfig* const pConf) //!< @see #MCAN_SetBitTimeConfiguration
{ return MCAN_SetBitTimeConfiguration(&pComp->_MCAN, pConf); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureWriteProtection(TCAN455X *pComp, bool enable, uint32_t* const lastRegRead) //!< @see #MCAN_ConfigureWriteProtection
{ return MCAN_ConfigureWriteProtection(&pComp->_MCAN, enable, lastRegRead); };
inline eERRORRESULT TCAN455X_SetWriteProtection(TCAN455X *pComp) //!< @see #MCAN_SetWriteProtection
{ return MCAN_SetWriteProtection(&pComp->_MCAN); };
inline eERRORRESULT TCAN455X_RemoveWriteProtection(TCAN455X *pComp) //!< @see #MCAN_RemoveWriteProtection
{ return MCAN_RemoveWriteProtection(&pComp->_MCAN); };

inline eERRORRESULT TCAN455X_GetActualOperationMode(TCAN455X *pComp, eMCAN_OperationMode* const actualMode) //!< @see #MCAN_GetActualOperationMode
{ return MCAN_GetActualOperationMode(&pComp->_MCAN, actualMode); };
inline eERRORRESULT TCAN455X_RequestOperationMode(TCAN455X *pComp, eMCAN_OperationMode newMode) //!< @see #MCAN_RequestOperationMode
{ return MCAN_RequestOperationMode(&pComp->_MCAN, newMode); };
inline eERRORRESULT TCAN455X_StartCAN20(TCAN455X *pComp) //!< @see #MCAN_StartCAN20
{ return MCAN_StartCAN20(&pComp->_MCAN); };
inline eERRORRESULT TCAN455X_StartCANFD(TCAN455X *pComp) //!< @see #MCAN_StartCANFD
{ return MCAN_StartCANFD(&pComp->_MCAN); };
inline eERRORRESULT TCAN455X_StartCANListenOnly(TCAN455X *pComp) //!< @see #MCAN_StartCANListenOnly
{ return MCAN_StartCANListenOnly(&pComp->_MCAN); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureTest(TCAN455X *pComp, bool enableLoopback, eMCAN_TestTxPin txPinControl) //!< @see #MCAN_ConfigureTest
{ return MCAN_ConfigureTest(&pComp->_MCAN, enableLoopback, txPinControl); };
inline eERRORRESULT TCAN455X_GetTest(TCAN455X *pComp, eMCAN_TestRxPin* const rxPin, uint8_t* const txNumPrepared, bool* const preparedValid, uint8_t* const txNumStarted, bool* const startedValid) //!< @see #MCAN_GetTest
{ return MCAN_GetTest(&pComp->_MCAN, rxPin, txNumPrepared, preparedValid, txNumStarted, startedValid); };
inline eERRORRESULT TCAN455X_ConfigureCANController(TCAN455X *pComp, setMCAN_CANCtrlFlags flags) //!< @see #MCAN_ConfigureCANController
{ return MCAN_ConfigureCANController(&pComp->_MCAN, flags); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_EnterSleepMode(TCAN455X *pComp) //!< @see #MCAN_EnterSleepMode
{ return MCAN_EnterSleepMode(&pComp->_MCAN); };
inline eERRORRESULT TCAN455X_IsDeviceInSleepMode(TCAN455X *pComp, bool* const isInSleepMode) //!< @see #MCAN_IsDeviceInSleepMode
{ return MCAN_IsDeviceInSleepMode(&pComp->_MCAN, isInSleepMode); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureTimeStamp(TCAN455X *pComp, eMCAN_TimeStampSelect timestampSource, uint8_t prescaler) //!< @see #MCAN_ConfigureTimeStamp
{ return MCAN_ConfigureTimeStamp(&pComp->_MCAN, timestampSource, prescaler); };
inline eERRORRESULT TCAN455X_SetTimeStamp(TCAN455X *pComp, uint32_t value) //!< @see #MCAN_SetTimeStamp
{ return MCAN_SetTimeStamp(&pComp->_MCAN, value); };
inline eERRORRESULT TCAN455X_GetTimeStamp(TCAN455X *pComp, uint32_t* value) //!< @see #MCAN_GetTimeStamp
{ return MCAN_GetTimeStamp(&pComp->_MCAN, value); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureTimeoutCounter(TCAN455X *pComp, bool enableTC, eMCAN_TimeoutSelect timeoutSelect, uint16_t period) //!< @see #MCAN_ConfigureTimeoutCounter
{ return MCAN_ConfigureTimeoutCounter(&pComp->_MCAN, enableTC, timeoutSelect, period); };
inline eERRORRESULT TCAN455X_SetTimeoutCounter(TCAN455X *pComp, uint32_t value) //!< @see #MCAN_SetTimeoutCounter
{ return MCAN_SetTimeoutCounter(&pComp->_MCAN, value); };
inline eERRORRESULT TCAN455X_GetTimeoutCounter(TCAN455X *pComp, uint32_t* value) //!< @see #MCAN_GetTimeoutCounter
{ return MCAN_GetTimeoutCounter(&pComp->_MCAN, value); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_GetFIFOStatus(TCAN455X *pComp, eMCAN_FIFObuffer name, setMCAN_FIFObufferstatus* const statusFlags) //!< @see #MCAN_GetFIFOStatus
{ return MCAN_GetFIFOStatus(&pComp->_MCAN, name, statusFlags); };
inline eERRORRESULT TCAN455X_GetNextMessageAddressFIFO(TCAN455X *pComp, eMCAN_FIFObuffer name, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex) //!< @see #MCAN_GetNextMessageAddressFIFO
{ return MCAN_GetNextMessageAddressFIFO(&pComp->_MCAN, name, level, getIndex, putIndex); };
inline eERRORRESULT TCAN455X_GetNextMessageAddressTEF(TCAN455X *pComp, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex) //!< @see #MCAN_GetNextMessageAddressTEF
{ return MCAN_GetNextMessageAddressTEF(&pComp->_MCAN, level, getIndex, putIndex); };
inline eERRORRESULT TCAN455X_AcknowledgeFIFO(TCAN455X *pComp, eMCAN_FIFObuffer name, uint32_t acknowledgeIndex) //!< @see #MCAN_AcknowledgeFIFO
{ return MCAN_AcknowledgeFIFO(&pComp->_MCAN, name, acknowledgeIndex); };
inline eERRORRESULT TCAN455X_AcknowledgeTEF(TCAN455X *pComp, uint32_t acknowledgeIndex) //!< @see #MCAN_AcknowledgeTEF
{ return MCAN_AcknowledgeTEF(&pComp->_MCAN, acknowledgeIndex); };
inline eERRORRESULT TCAN455X_SetTxBufferAddRequest(TCAN455X *pComp, uint32_t bufferIndex) //!< @see #MCAN_SetTxBufferAddRequest
{ return MCAN_SetTxBufferAddRequest(&pComp->_MCAN, bufferIndex); };
inline eERRORRESULT TCAN455X_GetAllTxBufferRequestPending(TCAN455X *pComp, uint32_t* const requestPending) //!< @see #MCAN_GetAllTxBufferRequestPending
{ return MCAN_GetAllTxBufferRequestPending(&pComp->_MCAN, requestPending); };
inline eERRORRESULT TCAN455X_SetMultipleTxBufferCancellationRequest(TCAN455X *pComp, uint32_t multipleRequest) //!< @see #MCAN_SetMultipleTxBufferCancellationRequest
{ return MCAN_SetMultipleTxBufferCancellationRequest(&pComp->_MCAN, multipleRequest); };
inline eERRORRESULT TCAN455X_SetTxBufferCancellationRequest(TCAN455X *pComp, uint32_t bufferIndex) //!< @see #MCAN_SetTxBufferCancellationRequest
{ return MCAN_SetTxBufferCancellationRequest(&pComp->_MCAN, bufferIndex); };
inline eERRORRESULT TCAN455X_GetAllTxBufferTransmitOccured(TCAN455X *pComp, uint32_t* const transmissionOccured) //!< @see #MCAN_GetAllTxBufferTransmitOccured
{ return MCAN_GetAllTxBufferTransmitOccured(&pComp->_MCAN, transmissionOccured); };
inline eERRORRESULT TCAN455X_GetAllTxBufferCancellationFinished(TCAN455X *pComp, uint32_t* const cancellationFinished) //!< @see #MCAN_GetAllTxBufferCancellationFinished
{ return MCAN_GetAllTxBufferCancellationFinished(&pComp->_MCAN, cancellationFinished); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureGlobalFilters(TCAN455X *pComp, bool rejectAllStandardIDs, bool rejectAllExtendedIDs, eMCAN_AcceptNonMatching nonMatchingStandardID, eMCAN_AcceptNonMatching nonMatchingExtendedID) //!< @see #MCAN_ConfigureGlobalFilters
{ return MCAN_ConfigureGlobalFilters(&pComp->_MCAN, rejectAllStandardIDs, rejectAllExtendedIDs, nonMatchingStandardID, nonMatchingExtendedID); };
inline eERRORRESULT TCAN455X_SetEIDrangeFilterMask(TCAN455X *pComp, uint32_t andMask) //!< @see #MCAN_SetEIDrangeFilterMask
{ return MCAN_SetEIDrangeFilterMask(&pComp->_MCAN, andMask); };
inline eERRORRESULT TCAN455X_ConfigureSIDfilter(TCAN455X *pComp, const MCAN_Filter* const confFilter) //!< @see #MCAN_ConfigureSIDfilter
{ return MCAN_ConfigureSIDfilter(&pComp->_MCAN, confFilter); };
inline eERRORRESULT TCAN455X_ConfigureEIDfilter(TCAN455X *pComp, const MCAN_Filter* const confFilter) //!< @see #MCAN_ConfigureEIDfilter
{ return MCAN_ConfigureEIDfilter(&pComp->_MCAN, confFilter); };
inline eERRORRESULT TCAN455X_ConfigureFilterList(TCAN455X *pComp, MCAN_Filter* const listFilter, size_t count) //!< @see #MCAN_ConfigureFilterList
{ return MCAN_ConfigureFilterList(&pComp->_MCAN, listFilter, count); };
inline eERRORRESULT TCAN455X_DisableFilter(TCAN455X *pComp, uint16_t name, bool extendedID) //!< @see #MCAN_DisableFilter
{ return MCAN_DisableFilter(&pComp->_MCAN, name, extendedID); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_ConfigureInterrupt(TCAN455X *pComp, setMCAN_InterruptEvents interruptsFlags, setMCAN_IntLineSelect intLineSelect) //!< @see #MCAN_ConfigureInterrupt
{ return MCAN_ConfigureInterrupt(&pComp->_MCAN, interruptsFlags, intLineSelect); };
inline eERRORRESULT TCAN455X_GetInterruptEvents(TCAN455X *pComp, setMCAN_InterruptEvents* interruptsFlags) //!< @see #MCAN_GetInterruptEvents
{ return MCAN_GetInterruptEvents(&pComp->_MCAN, interruptsFlags); };
inline eERRORRESULT TCAN455X_ClearInterruptEvents(TCAN455X *pComp, setMCAN_InterruptEvents interruptsFlags) //!< @see #MCAN_ClearInterruptEvents
{ return MCAN_ClearInterruptEvents(&pComp->_MCAN, interruptsFlags); };
inline eERRORRESULT TCAN455X_GetHighPriorityMessageStatus(TCAN455X *pComp, eMCAN_MessageStorageIndicator* const messageIndicator, bool* const isExtended, uint8_t* const bufferIndex, uint8_t* const filterIndex) //!< @see #MCAN_GetHighPriorityMessageStatus
{ return MCAN_GetHighPriorityMessageStatus(&pComp->_MCAN, messageIndicator, isExtended, bufferIndex, filterIndex); };
inline eERRORRESULT TCAN455X_GetRxBufferNewDataFlag(TCAN455X *pComp, uint32_t* const newDataIdx0_31, uint32_t* const newDataIdx32_63) //!< @see #MCAN_GetRxBufferNewDataFlag
{ return MCAN_GetRxBufferNewDataFlag(&pComp->_MCAN, newDataIdx0_31, newDataIdx32_63); };
inline eERRORRESULT TCAN455X_ClearRxBufferNewDataFlag(TCAN455X *pComp, uint8_t index) //!< @see #MCAN_ClearRxBufferNewDataFlag
{ return MCAN_ClearRxBufferNewDataFlag(&pComp->_MCAN, index); };
inline eERRORRESULT TCAN455X_ConfigureTxBufferInterrupts(TCAN455X *pComp, uint32_t transmitEnable, uint32_t cancelFinishEnable) //!< @see #MCAN_ConfigureTxBufferInterrupts
{ return MCAN_ConfigureTxBufferInterrupts(&pComp->_MCAN, transmitEnable, cancelFinishEnable); };

//********************************************************************************************************************

inline eERRORRESULT TCAN455X_GetTransmitReceiveErrorCountAndStatus(TCAN455X *pComp, uint8_t* transmitErrorCount, uint8_t* receiveErrorCount, bool* receiveErrorPassive, uint8_t* canErrorLogging) //!< @see #MCAN_GetTransmitReceiveErrorCountAndStatus
{ return MCAN_GetTransmitReceiveErrorCountAndStatus(&pComp->_MCAN, transmitErrorCount, receiveErrorCount, receiveErrorPassive, canErrorLogging); };
inline eERRORRESULT TCAN455X_GetBusDiagnostic(TCAN455X *pComp, MCAN_PSR_Register* const busDiagnostic) //!< @see #MCAN_GetBusDiagnostic
{ return MCAN_GetBusDiagnostic(&pComp->_MCAN, busDiagnostic); };

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* TCAN455X_H_INC */