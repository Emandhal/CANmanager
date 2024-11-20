/*!*****************************************************************************
 * @file    Ultra_V71Interfaces.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    01/07/2023
 * @brief   V71 Xplained Ultra board interfaces
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "Ultra_V71Interfaces.h"
#ifdef USE_EEPROM_GENERICNESS
#  include "TWIHS_V71.h"
#endif
#include "SPI_V71.h"
#include "Main.h"
//-----------------------------------------------------------------------------
#if !defined(__cplusplus)
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 MCAN0 peripheral
//********************************************************************************************************************
#ifdef USE_V71_MCAN

#define MCAN0_CAN_BUFFER_SIZE  ( sizeof(uint32_t) * 4352u )         //!< 32-bits word * 4352 words max for MCAN0 is 17408 bytes
COMPILER_ALIGNED(4) uint8_t MCAN0_RAMbuffer[MCAN0_CAN_BUFFER_SIZE]; //!< RAM buffer for the MCAN0

//! Peripheral structure of the MCAN1 on SAMV71
struct MCANV71 MCAN0V71 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCAN_DRIVER_NORMAL_USE,
  //--- MCAN peripheral ---
  .Instance       = MCAN0,
  .RAMallocation  = &MCAN0_RAMbuffer[0],
  .RAMsize        = MCAN0_CAN_BUFFER_SIZE,
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCAN0V71_BitTimeStats = { 0 }; //!< MCAN0V71 Bit Time stat
uint32_t MCAN0V71_SYSCLK = 0; //!< SYSCLK frequency will be stored here after using #Init_MCANV71()

//! Configuration structure of the MCAN0 on SAMV71
struct MCAN_Config MCAN0V71_Conf =
{
  //--- Controller clocks ---
  .MessageRAMwatchdogConf = 0,
  .SYSCLK_Result          = &MCAN0V71_SYSCLK,
  //--- RAM configuration ---
  .SIDelementsCount       = 10,
  .EIDelementsCount       = 11,
  //--- CAN configuration ---
  .ExtendedIDrangeMask    = MCAN_EID_AND_RANGE_MASK,
  .RejectAllStandardIDs   = false,
  .RejectAllExtendedIDs   = false,
  .NonMatchingStandardID  = MCAN_ACCEPT_IN_RX_FIFO_0,
  .NonMatchingExtendedID  = MCAN_ACCEPT_IN_RX_FIFO_1,
  .BusConfig =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,
    .TransceiverDelay      = 300,                  // The transceiver is a ATA6561-GBQW on the board. The worst delay is from Normal mode, Falling edge at pin TXD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats          = &MCAN0V71_BitTimeStats,
  .ControlFlags          = MCAN_CAN_AUTOMATIC_RETRANSMISSION_ENABLE
                         | MCAN_CANFD_BITRATE_SWITCHING_ENABLE
                         | MCAN_CANFD_USE_ISO_CRC
                         | MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT
                         | MCAN_CAN_INTERNAL_TIMESTAMPING,
  //--- GPIOs and Interrupts pins ---
  .EnableLineINT0        = true,
  .EnableLineINT1        = true,
  //--- Interrupts ---
  .SysInterruptFlags     = MCAN_INT_ENABLE_ALL_EVENTS,
  .SysIntLineSelect      = MCAN_INT_ALL_ON_INT0 // All but...
                         | MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT1
                         | MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT1
                         | MCAN_INT_TIMEOUT_OCCURED_ON_INT1
                         | MCAN_INT_ERROR_CORRECTED_ON_INT1
                         | MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT1
                         | MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT1
                         | MCAN_INT_ERROR_PASSIVE_ON_INT1
                         | MCAN_INT_WARNING_STATUS_ON_INT1
                         | MCAN_INT_BUS_OFF_STATUS_ON_INT1
                         | MCAN_INT_WATCHDOG_ON_INT1
                         | MCAN_INT_ARBITRATION_ERROR_ON_INT1
                         | MCAN_INT_DATA_PHASE_ERROR_ON_INT1
                         | MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT1,
  .BufferTransmitInt     = 0xFFFFFFFF,
  .BufferCancelFinishInt = 0xFFFFFFFF,
};
#endif

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 MCAN1 peripheral
//********************************************************************************************************************
#ifdef USE_V71_MCAN

#define MCAN1_CAN_BUFFER_SIZE  ( sizeof(uint32_t) * 4352u )         //!< 32-bits word * 4352 words max for MCAN1 is 17408 bytes
COMPILER_ALIGNED(4) uint8_t MCAN1_RAMbuffer[MCAN1_CAN_BUFFER_SIZE]; //!< RAM buffer for the MCAN1

//! Peripheral structure of the MCAN1 on SAMV71
struct MCANV71 MCAN1V71 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCAN_DRIVER_NORMAL_USE,
  //--- MCAN peripheral ---
  .Instance       = MCAN1,
  .RAMallocation  = &MCAN1_RAMbuffer[0],
  .RAMsize        = MCAN1_CAN_BUFFER_SIZE,
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCAN1V71_BitTimeStats = { 0 }; //!< MCAN1V71 Bit Time stat
uint32_t MCAN1V71_SYSCLK = 0; //!< SYSCLK frequency will be stored here after using #Init_MCANV71()

//! Configuration structure of the MCAN1 on SAMV71
struct MCAN_Config MCAN1V71_Conf =
{
  //--- Controller clocks ---
  .MessageRAMwatchdogConf = 0,
  .SYSCLK_Result          = &MCAN1V71_SYSCLK,
  //--- RAM configuration ---
  .SIDelementsCount       = 10,
  .EIDelementsCount       = 11,
  //--- CAN configuration ---
  .ExtendedIDrangeMask    = MCAN_EID_AND_RANGE_MASK,
  .RejectAllStandardIDs   = false,
  .RejectAllExtendedIDs   = false,
  .NonMatchingStandardID  = MCAN_ACCEPT_IN_RX_FIFO_0,
  .NonMatchingExtendedID  = MCAN_ACCEPT_IN_RX_FIFO_1,
  .BusConfig =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,
    .TransceiverDelay      = 300,                  // The transceiver is a ATA6561-GBQW on the board. The worst delay is from Normal mode, Falling edge at pin TXD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats          = &MCAN1V71_BitTimeStats,
  .ControlFlags          = MCAN_CAN_AUTOMATIC_RETRANSMISSION_ENABLE
                         | MCAN_CANFD_BITRATE_SWITCHING_ENABLE
                         | MCAN_CANFD_USE_ISO_CRC
                         | MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT
                         | MCAN_CAN_INTERNAL_TIMESTAMPING,
  //--- GPIOs and Interrupts pins ---
  .EnableLineINT0        = true,
  .EnableLineINT1        = true,
  //--- Interrupts ---
  .SysInterruptFlags     = MCAN_INT_ENABLE_ALL_EVENTS,
  .SysIntLineSelect      = MCAN_INT_ALL_ON_INT0 // All but...
                         | MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT1
                         | MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT1
                         | MCAN_INT_TIMEOUT_OCCURED_ON_INT1
                         | MCAN_INT_ERROR_CORRECTED_ON_INT1
                         | MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT1
                         | MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT1
                         | MCAN_INT_ERROR_PASSIVE_ON_INT1
                         | MCAN_INT_WARNING_STATUS_ON_INT1
                         | MCAN_INT_BUS_OFF_STATUS_ON_INT1
                         | MCAN_INT_WATCHDOG_ON_INT1
                         | MCAN_INT_ARBITRATION_ERROR_ON_INT1
                         | MCAN_INT_DATA_PHASE_ERROR_ON_INT1
                         | MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT1,
  .BufferTransmitInt     = 0xFFFFFFFF,
  .BufferCancelFinishInt = 0xFFFFFFFF,
};

CAN_RAMconfig MCAN1_FIFObuff_RAMInfos[MCAN1V71_OBJ_COUNT];

MCAN_FIFObuff MCAN1_FIFObuffList[MCAN1V71_OBJ_COUNT] =
{
  { .Name = MCAN_RX_FIFO0,  .Size = 64, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[0], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_RX_FIFO1,  .Size = 64, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[1], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_RX_BUFFER, .Size = 64, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[2], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_TEF,       .Size = 32, .Payload = MCAN_8_BYTES,  .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_EVENT_NEW_MESSAGE_INT   | MCAN_FIFO_EVENT_LOST_MESSAGE_INT,   .WatermarkLevel = 16, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[3], }, // 32 *  2 * UINT32
  { .Name = MCAN_TX_BUFFER, .Size = 16, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_TX_BUFFER_MODE,        .InterruptFlags = MCAN_FIFO_NO_INTERRUPT_FLAGS,                                           .WatermarkLevel =  8, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[4], }, // 16 * (2 * UINT32 + 64)
  { .Name = MCAN_TXQ_FIFO,  .Size = 16, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_TX_FIFO_MODE,          .InterruptFlags = MCAN_FIFO_TRANSMIT_FIFO_EMPTY_INT,                                      .WatermarkLevel =  8, .RAMInfos = &MCAN1_FIFObuff_RAMInfos[5], }, // 16 * (2 * UINT32 + 64)
};

MCAN_Filter MCAN1_FilterList[MCAN1V71_FILTER_COUNT] =
{
  //--- SID filters ---
  { .Filter = 0, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_REJECT_ID,        .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = false, .DualID    = { .AcceptanceID1 = 0x000, .AcceptanceID2  = 0x001,                                               }, }, // Reject 0x000 and 0x001
  { .Filter = 1, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_RANGE, .Config = MCAN_FILTER_SET_PRIORITY,     .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = false, .RangeID   = { .MinID         = 0x002, .MaxID          = 0x00F,                                               }, }, // High priority message for 0x002 to 0x00F, no store
  { .Filter = 2, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_SET_PRIORITY,     .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .IDandMask = { .AcceptanceID  = 0x010, .AcceptanceMask = 0x7F0,                                               }, }, // High priority message for 0x010 to 0x01F, store to FIFO 0
  { .Filter = 3, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_BUFFER,    .ExtendedID = false, .IDbuffer  = { .AcceptanceID  = 0x200, .BufferPosition = 20,                                                  }, }, // Store message 0x200 to buffer 20
  { .Filter = 4, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = false, .DebugID   = { .AcceptanceID  = 0x400, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_A, .BufferPosition = 58, }, }, // Store debug message A in buffer 58
  { .Filter = 5, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = false, .DebugID   = { .AcceptanceID  = 0x401, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_B, .BufferPosition = 59, }, }, // Store debug message B in buffer 59
  { .Filter = 6, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = false, .DebugID   = { .AcceptanceID  = 0x402, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_C, .BufferPosition = 60, }, }, // Store debug message C in buffer 60
  { .Filter = 7, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .IDandMask = { .AcceptanceID  = 0x600, .AcceptanceMask = 0x700,                                               }, }, // Range 0x600 to 0x6FF
  { .Filter = 8, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .DualID    = { .AcceptanceID1 = 0x700, .AcceptanceID2  = 0x701,                                               }, }, // IDs 0x700 and 0x701
  { .Filter = 9, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_RANGE, .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .RangeID   = { .MinID         = 0x702, .MaxID          = 0x7FF,                                               }, }, // Range 0x702 to 0x7FF

  //--- EID filters ---
  { .Filter =  0, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_REJECT_ID,        .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = true, .DualID    = { .AcceptanceID1 = 0x00000000, .AcceptanceID2  = 0x00000001,                                          }, }, // Reject 0x00000000 and 0x00000001
  { .Filter =  1, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE,      .Config = MCAN_FILTER_SET_PRIORITY,     .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = true, .RangeID   = { .MinID         = 0x00000002, .MaxID          = 0x0000000F,                                          }, }, // High priority message for 0x00000002 to 0x0000000F, no store
  { .Filter =  2, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_SET_PRIORITY,     .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .IDandMask = { .AcceptanceID  = 0x00000010, .AcceptanceMask = 0x1FFFFFF0,                                          }, }, // High priority message for 0x00000010 to 0x0000001F, store to FIFO 1
  { .Filter =  3, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_BUFFER,    .ExtendedID = true, .IDbuffer  = { .AcceptanceID  = 0x08000000, .BufferPosition = 25,                                                  }, }, // Store message 0x08000000 to buffer 25
  { .Filter =  4, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = true, .DebugID   = { .AcceptanceID  = 0x10000000, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_A, .BufferPosition = 61, }, }, // Store debug message A in buffer 61
  { .Filter =  5, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = true, .DebugID   = { .AcceptanceID  = 0x10000001, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_B, .BufferPosition = 62, }, }, // Store debug message B in buffer 62
  { .Filter =  6, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_AS_DEBUG_MESSAGE, .PointTo = MCAN_RX_BUFFER,    .ExtendedID = true, .DebugID   = { .AcceptanceID  = 0x10000002, .DebugMessage   = MCAN_TREAT_AS_DEBUG_MESSAGE_C, .BufferPosition = 63, }, }, // Store debug message C in buffer 63
  { .Filter =  7, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .IDandMask = { .AcceptanceID  = 0x10000000, .AcceptanceMask = 0x1F000000,                                          }, }, // Range 0x10000000 to 0x10FFFFFF
  { .Filter =  8, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .DualID    = { .AcceptanceID1 = 0x14000000, .AcceptanceID2  = 0x14000001,                                          }, }, // IDs 0x14000000 and 0x14000001
  { .Filter =  9, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE,      .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .RangeID   = { .MinID         = 0x18000000, .MaxID          = 0x1C000000,                                          }, }, // Range 0x18000000 to 0x1C000000
  { .Filter = 10, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE_MASK, .Config = MCAN_FILTER_NO_CONFIG,        .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .RangeID   = { .MinID         = 0x1C000000, .MaxID          = 0x1F000000,                                          }, }, // Range 0x1C000000 to 0x3FFFFFFF but with ExtendedIDrangeMask mask: 0x00000000 to 0x1FFFFFFF
};
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 I2C peripheral
//********************************************************************************************************************
#ifdef USE_EEPROM_GENERICNESS
//! Peripheral structure of the hard I2C0 on the V71
struct I2C_Interface I2C0_V71 =
{
  .InterfaceDevice = TWIHS0,
  .UniqueID        = TWIHS_UNIQUE_ID,
  .fnI2C_Init      = TWIHS_MasterInit_Gen,
  .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
};
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 SPI peripheral
//********************************************************************************************************************

//! Peripheral structure of the hard SPI0 on the V71
struct SPI_Interface SPI0_V71 =
{
  .InterfaceDevice = SPI0,
  .UniqueID        = SPI_UNIQUE_ID,
  .fnSPI_Init      = SPI_MasterInit_Gen,
  .fnSPI_Transfer  = SPI_PacketTransfer_Gen,
};

//! Delay before SPCK
#define SPI_DLYBS   ( 0x01 ) // Tspick/2 needed
//! Delay between consecutive transfers
#define SPI_DLYBCT  ( 0x01 ) // To conform last SCK rise to nCS rise time (1 Tspick)
//! Delay Between Chip Selects
#define SPI_DLYBCS  ( 0x01 ) // To conform 1 Tspick needed

//! Configuration of the SPI0 on the V71
SPI_Config SPI0_Config =
{
  .VariablePS      = true,
  .CSdecoder       = true,
  .ModeFaultDetect = false,
  .WaitRead        = true,
  .DLYBCS_ns       = SPI_DLYBCS,
  .CSR             =
  {
    { .DLYBCT_ns = SPI_DLYBCT, .DLYBS_ns = SPI_DLYBS, .BitsPerTransfer = 8, .CSbehavior = SPI_CS_KEEP_LOW, },
    { .DLYBCT_ns = SPI_DLYBCT, .DLYBS_ns = SPI_DLYBS, .BitsPerTransfer = 8, .CSbehavior = SPI_CS_KEEP_LOW, },
    { .DLYBCT_ns = SPI_DLYBCT, .DLYBS_ns = SPI_DLYBS, .BitsPerTransfer = 8, .CSbehavior = SPI_CS_KEEP_LOW, },
    { .DLYBCT_ns = SPI_DLYBCT, .DLYBS_ns = SPI_DLYBS, .BitsPerTransfer = 8, .CSbehavior = SPI_CS_KEEP_LOW, },
  },
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// AT24MAC402 Component
//********************************************************************************************************************
#ifdef USE_EEPROM_GENERICNESS
//! Component structure of the AT24MAC402 with hard I2C on the V71
struct AT24MAC402 AT24MAC402_V71 =
{
  .Eeprom =
  {
    .UserDriverData = NULL,
    //--- EEPROM configuration ---
    .Conf           = &AT24MAC402_Conf,
    //--- Interface driver call functions ---
    .I2C            = &I2C0_V71,
    .I2CclockSpeed  = 400000, // I2C speed at 400kHz
    //--- Time call function ---
    .fnGetCurrentms = GetCurrentms_V71,
    //--- Interface clocks ---
    .AddrA2A1A0     = AT24MAC402_ADDR(1, 1, 1),
  },
};
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAN V71 GPIO Ports
//********************************************************************************************************************

//! IOPORTA interface of the V71
PORT_Interface IOPORTA =
{
  .InterfaceDevice       = PIOA,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTB interface of the V71
PORT_Interface IOPORTB =
{
  .InterfaceDevice       = PIOB,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTC interface of the V71
PORT_Interface IOPORTC =
{
  .InterfaceDevice       = PIOC,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTD interface of the V71
PORT_Interface IOPORTD =
{
  .InterfaceDevice       = PIOD,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTE interface of the V71
PORT_Interface IOPORTE =
{
  .InterfaceDevice       = PIOE,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Get millisecond
//=============================================================================
uint32_t GetCurrentms_V71(void)
{
  return msCount;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// PORTs interfaces of V71
//********************************************************************************************************************

//=============================================================================
// PORT set pins direction for V71
//=============================================================================
eERRORRESULT PORTSetDirection_V71(PORT_Interface *pIntDev, const uint32_t pinsDirection)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Pio* pPIO = (Pio*)(pIntDev->InterfaceDevice);
  if (pPIO == NULL) return ERR__PARAMETER_ERROR;

  return ERR_NONE;
}


//=============================================================================
// PORT pins input level for V71
//=============================================================================
eERRORRESULT PORTGetInputLevel_V71(PORT_Interface *pIntDev, uint32_t *pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Pio* pPIO = (Pio*)(pIntDev->InterfaceDevice);
  if (pPIO == NULL) return ERR__PARAMETER_ERROR;

  return ERR_NONE;
}


//=============================================================================
// PORT pins output level for V71
//=============================================================================
eERRORRESULT PORTSetOutputLevel_V71(PORT_Interface *pIntDev, const uint32_t pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Pio* pPIO = (Pio*)(pIntDev->InterfaceDevice);
  if (pPIO == NULL) return ERR__PARAMETER_ERROR;

  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// GPIO Interfaces of V71
//********************************************************************************************************************

//=============================================================================
// GPIO set direction for V71
//=============================================================================
eERRORRESULT GPIOSetState_V71(GPIO_Interface *pIntDev, const eGPIO_State pinState)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Pio* pPIO = (Pio*)(pIntDev->InterfaceDevice);
  if (pPIO == NULL) return ERR__PARAMETER_ERROR;

  return ERR_NONE;
}


//=============================================================================
// GPIO pin input level for V71
//=============================================================================
eERRORRESULT GPIOGetInputLevel_V71(GPIO_Interface *pIntDev, eGPIO_State *pinLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  Pio* pPIO = (Pio*)(pIntDev->InterfaceDevice);
  if (pPIO == NULL) return ERR__PARAMETER_ERROR;

  return ERR_NONE;
}


//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------