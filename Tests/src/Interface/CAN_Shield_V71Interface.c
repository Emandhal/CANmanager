/*!*****************************************************************************
 * @file    CAN_Shield_V71Interface.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/07/2023
 * @brief   CAN Shield board interfaces on V71
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "CAN_Shield_V71Interface.h"
#include "TWIHS_V71.h"
#include "SPI_V71.h"
#include "Main.h"
#include "CRC16_CMS.h"
//-----------------------------------------------------------------------------
#if !defined(__cplusplus)
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------





//********************************************************************************************************************

//=============================================================================
// MCP251XFD compute CRC16-CMS
//=============================================================================
static uint16_t MCP251XFD_ComputeCRC16CMS(const uint8_t* data, size_t size)
{
  return ComputeCRC16CMS(data, size);
}
//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP230XX and MCP23SXX ports expanders
//********************************************************************************************************************

//! Component structure of the MCP230XX as U12 with hard I2C on the CAN_Shield board
struct MCP230XX MCP23008_U12 =
{
  .UserDriverData = NULL,
  //--- Device configuration ---
  .ControlFlags   = MCP230XX_READING_GPIO_CLEARS_INTERRUPT
                  | MCP230XX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH
                  | MCP230XX_INT_PIN_OUTPUT_PUSH_PULL
                  | MCP230XX_GPIO_SAMPLING_RATE_FAST,
  //--- Interface driver call functions ---
  .I2C            = &I2C0_V71,
  .I2CclockSpeed  = 400000, // I2C speed at 400kHz
  //--- Device address --
  .AddrA2A1A0     = MCP230XX_ADDR(0, 0, 1),
  .DeviceName     = MCP23008,
};

//! Configuration structure of the MCP230XX as U12
struct MCP230XX_Config MCP23008_U12_Conf =
{
  .GP =
  {
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP0: mikroBUS1_RST
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP1: mikroBUS2_RST
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP2: SJA1000_RST
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP3: SJA1000_MODE
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GP4: MCAN1_Trans_INH/LIMP
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP5: MCAN1_Trans_WAKE
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP6: mikroBUS3_AN
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GP7: mikroBUS3_RST
  },
};

//! PORT interface of the MCP230XX as U8 PORT GP on the CAN_Shield board
PORT_Interface PORTGP_U12 =
{
  .InterfaceDevice       = &MCP23008_U12,
  .UniqueID              = MCP230XX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP230XX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP230XX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP230XX_SetPORToutputLevel_Gen,
  .PORTindex             = MCP230XX_PORT_GP,
};

//-----------------------------------------------------------------------------

//! Component structure of the MCP230XX as U8 with hard I2C on the CAN_Shield board
struct MCP230XX MCP23017_U8 =
{
  .UserDriverData = NULL,
  //--- Device configuration ---
  .ControlFlags   = MCP230XX_READING_GPIO_CLEARS_INTERRUPT
                  | MCP230XX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH
                  | MCP230XX_INT_PIN_OUTPUT_PUSH_PULL
                  | MCP230XX_GPIO_SAMPLING_RATE_FAST
                  | MCP230XX_INT_PIN_MIRRORED,
  //--- Interface driver call functions ---
  .I2C            = &I2C0_V71,
  .I2CclockSpeed  = 400000, // I2C speed at 400kHz
  //--- Device address --
  .AddrA2A1A0     = MCP230XX_ADDR(0, 0, 0),
  .DeviceName     = MCP23017,
};

//! Configuration structure of the MCP230XX as U8
struct MCP230XX_Config MCP23017_U8_Conf =
{
  .GPA =
  {
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA0: SJA1000_GPIO_INT
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA1: GPIO_INT
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA2: SJA1000_INT
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA3: MCAN1_Trans_INT
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA4: MCP2515_SOF
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA5: MCP2515_INT
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA6: MCP2515_RX0BF#
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPA7: MCP2515_RX1BF#
  },
  .GPB =
  {
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPB0: MCP2515_TX0RTS#
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPB1: MCP2515_TX1RTS#
    MCP230XX_PIN_AS_INPUT  | MCP230XX_GPIO_STATE_SAME_LOGIC |                                  MCP230XX_GPIO_PULLUP_ENABLE  | MCP230XX_INTERRUPT_ON_CHANGE_ENABLE , // GPB2: MCP2515_TX2RTS#
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GPB3: MCP2515_RST
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GPB4: MCP2515_Trans_INH
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GPB5: MCP2515_Trans_WAKE
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GPB6: MCAN0_Trans_INH
    MCP230XX_PIN_AS_OUTPUT | MCP230XX_GPIO_STATE_SAME_LOGIC | MCP230XX_GPIO_OUTPUT_STATE_LOW | MCP230XX_GPIO_PULLUP_DISABLE | MCP230XX_INTERRUPT_ON_CHANGE_DISABLE, // GPB7: MCAN0_Trans_WAKE
  },
};

//! PORT interface of the MCP230XX as U12 PORT GPA on the CAN_Shield board
PORT_Interface PORTGPA_U8 =
{
  .InterfaceDevice       = &MCP23017_U8,
  .UniqueID              = MCP230XX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP230XX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP230XX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP230XX_SetPORToutputLevel_Gen,
  .PORTindex             = MCP230XX_PORT_GPA,
};

//! PORT interface of the MCP230XX as U12 PORT GPB on the CAN_Shield board
PORT_Interface PORTGPB_U8 =
{
  .InterfaceDevice       = &MCP23017_U8,
  .UniqueID              = MCP230XX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP230XX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP230XX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP230XX_SetPORToutputLevel_Gen,
  .PORTindex             = MCP230XX_PORT_GPB,
};

//-----------------------------------------------------------------------------

//! Component structure of the MCP23SXX as U9 with hard SPI on the CAN_Shield board
struct MCP23SXX MCP23S17_U9 =
{
  .UserDriverData = NULL,
  //--- Device configuration ---
  .ControlFlags   = MCP23SXX_READING_GPIO_CLEARS_INTERRUPT
                  | MCP23SXX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH
                  | MCP23SXX_INT_PIN_OUTPUT_PUSH_PULL
                  | MCP23SXX_ADDRESS_PIN_DISABLE
                  | MCP23SXX_INT_PIN_MIRRORED,
  //--- Interface driver call functions ---
  .SPIchipSelect  = 0x3 << 1, // Y3 output on U7
  .SPI            = &SPI0_V71,
  .SPIclockSpeed  = 10000000, // SPI speed at 10MHz
  //--- Device address --
  .AddrA2A1A0     = MCP23SXX_ADDR(0, 0, 0),
  .DeviceName     = MCP23S17,
};

//! Configuration structure of the MCP23SXX as U9
struct MCP23SXX_Config MCP23S17_U9_Conf =
{
  .GPA =
  {
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA0: SJA1000_ALE/AS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA1: SJA1000_nCS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA2: SJA1000_nRD/E
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA3: SJA1000_nWR
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA4: MCP_Trans_CS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA5: SJA_Trans_CS
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA6: SJA_Trans_INH
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA7: SJA_Trans_WAKE
  },
  .GPB =
  {
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB0: SJA1000_AD0
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB1: SJA1000_AD1
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB2: SJA1000_AD2
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB3: SJA1000_AD3
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB4: SJA1000_AD4
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB5: SJA1000_AD5
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB6: SJA1000_AD6
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB7: SJA1000_AD7
  },
};

//! PORT interface of the MCP23SXX as U9 PORT GPA on the CAN_Shield board
PORT_Interface PORTGPA_U9 =
{
  .InterfaceDevice       = &MCP23S17_U9,
  .UniqueID              = MCP23SXX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP23SXX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP23SXX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP23SXX_SetPORToutputLevel_Gen,
  .PORTindex             = MCP23SXX_PORT_GPA,
};

//! PORT interface of the MCP23SXX as U9 PORT GPB on the CAN_Shield board
PORT_Interface PORTGPB_U9 =
{
  .InterfaceDevice       = &MCP23S17_U9,
  .UniqueID              = MCP23SXX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP23SXX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP23SXX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP23SXX_SetPORToutputLevel_Gen,
  .PORTindex             = MCP23SXX_PORT_GPB,
};

//! GPIO interface of the MCP23SXX as SJA1000 CS pin on the CAN_Shield board
GPIO_Interface SJA1000_CS =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 1), // On GPA1
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//! GPIO interface of the MCP23SXX as SJA1000 RD#/E pin on the CAN_Shield board
GPIO_Interface SJA1000_RD_E =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 2), // On GPA2
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//! GPIO interface of the MCP23SXX as SJA1000 WR# pin on the CAN_Shield board
GPIO_Interface SJA1000_WR =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 3), // On GPA3
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//! GPIO interface of the MCP23SXX as SJA1000 ALE/AS pin on the CAN_Shield board
GPIO_Interface SJA1000_ALE_AS =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 0), // On GPA0
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//! GPIO interface of the MCP23SXX as MCP2515 Transceiver CS pin on the CAN_Shield board
GPIO_Interface MCP2515_Trans_CS =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 4), // On GPA4
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//! GPIO interface of the MCP23SXX as SJA1000 Transceiver CS pin on the CAN_Shield board
GPIO_Interface SJA1000_Trans_CS =
{
  .InterfaceDevice      = &MCP23S17_U9,
  .UniqueID             = MCP23SXX_UNIQUE_ID,
  .fnGPIO_SetState      = MCP23SXX_SetPinState_Gen,
  .fnGPIO_GetInputLevel = MCP23SXX_GetPinInputLevel_Gen,
  .PinBitMask           = (1 << 5), // On GPA5
  .PORTindex            = MCP23SXX_PORT_GPA,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP2515 External CAN controller
//********************************************************************************************************************

//! Component structure of the MCP2515 as U5
struct MCP251X MCP2515_U5 =
{
  .UserDriverData = NULL,
  //--- Device configuration ---
  .InternalConfig = 0,
  //--- Interface driver call functions ---
  .SPIchipSelect  = 0x2 << 1, // Y2 output on U7
  .SPI            = &SPI0_V71,
  .SPIclockSpeed  = 10000000, // 10MHz
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCP2515_BitTimeStats = { 0 }; //!< MCP2515 Bit Time stat

//! Configuration structure of the MCP2515 as U5
struct MCP251X_Config MCP2515_U5_Conf =
{
  //--- Controller clocks ---
  .XtalFreq       = 16000000,
  //--- CAN configuration ---
  .BusConfig      =
  {
    .DesiredBitrate   = CAN_SHIELD_BITRATE, // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .BusMeters        = 1,                  // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay = 120,                // The transceiver is a TJA1145T/FDJ (U4). The worst delay is from bus recessive to RXD
  },
  .BitTimeStats   = &MCP2515_BitTimeStats,
  //--- CAN configuration ---
  .UseOneShotMode = false,
  //--- Configure buffers ---
  .tx0Priority    = MCP251X_MESSAGE_TX_PRIORITY1,
  .tx1Priority    = MCP251X_MESSAGE_TX_PRIORITY1,
  .tx2Priority    = MCP251X_MESSAGE_TX_PRIORITY1,
  .rx0Conf        = MCP251X_RX_ACCEPT_ALL_MESSAGES,
  .rx1Conf        = MCP251X_RX_ACCEPT_ALL_MESSAGES,
  //--- Pins configuration ---
  .tx0PinMode     = MCP251X_TxPIN_AS_INT_TX,
  .tx1PinMode     = MCP251X_TxPIN_AS_INT_TX,
  .tx2PinMode     = MCP251X_TxPIN_AS_INT_TX,
  .rx0PinMode     = MCP251X_RxPIN_AS_INT_RX,
  .rx1PinMode     = MCP251X_RxPIN_AS_INT_RX,
  .clkoutMode     = MCP251X_CLKOUT_IS_SOF,
  //--- Interrupts ---
  .Interrupts     = MCP251X_ENABLE_ALL_INTERRUPTS,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SJA1000 External CAN controller
//********************************************************************************************************************

//! Component structure of the SJA1000 as U6 with link to GPIO expander on the CAN_Shield board
struct SJA1000 SJA1000_U6 =
{
  .UserDriverData = NULL,
  //--- Interface driver call functions ---
  .CS     = &SJA1000_CS,
  .ALE_AS = &SJA1000_ALE_AS,
  .RD_E   = &SJA1000_RD_E,
  .WR     = &SJA1000_WR,
  .DATA   = &PORTGPB_U9,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats SJA1000_BitTimeStats = { 0 }; //!< SJA1000 Bit Time stat

//! Configuration structure of the SJA1000 as U6
struct SJA1000_Config SJA1000_U6_Conf =
{
  //--- Controller clocks ---
  .XtalFreq      = 16000000,
  //--- CAN configuration --
  .BusConfig     =
  {
    .DesiredBitrate   = CAN_SHIELD_BITRATE, // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .BusMeters        = 1,                  // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay = 120,                // The transceiver is a TJA1145T/FDJ (U4). The worst delay is from bus recessive to RXD           
  },
  .BitTimeStats  = &SJA1000_BitTimeStats,
  //--- Pins configuration ---
  .outMode      = SJA1000_NORMAL_OUT_MODE,
  .tx0PinMode   = SJA1000_TX_PUSH_PULL,
  .tx1PinMode   = SJA1000_TX_PUSH_PULL,
  .clkoutMode   = SJA1000_CLKOUT_OFF,
  .tx1AsRxInt   = false,
  .bypassRxComp = false,
  //--- Interrupts ---
  .Interrupts   = SJA1000_ENABLE_ALL_PCAN_INTERRUPTS,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP2518FD External CAN controller on MIKROBUS1
//********************************************************************************************************************

//! Component structure of the MCP2518FD on MIKROBUS1 on the V71_XplainedUltra_CAN_Shield board
struct MCP251XFD MCP2518FD_MB1 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCP251XFD_DRIVER_NORMAL_USE
                  | MCP251XFD_DRIVER_SAFE_RESET
                  | MCP251XFD_DRIVER_USE_READ_WRITE_CRC
                  | MCP251XFD_DRIVER_INIT_SET_RAM_AT_0
                  | MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ,
  //--- IO configuration ---
  .GPIOsDirection = 0,
  .GPIOsOutLevel  = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,
  //--- Interface driver call functions ---
  .SPIchipSelect  = 0x0 << 1, // Y0 output on U7
  .SPI            = &SPI0_V71,
  .SPIclockSpeed  = 17000000, // 17MHz
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
  //--- CRC16-USB call function ---
  .fnComputeCRC16 = MCP251XFD_ComputeCRC16CMS,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCP2518FD_BitTimeStats = { 0 }; //!< MCP2518FD Bit Time stat
uint32_t MCP2518FD_SYSCLK; //!< SYSCLK frequency will be stored here after using #Init_MCP251XFD()

//! Configuration structure of the MCP2518FD on MIKROBUS1
struct MCP251XFD_Config MCP2518FD_Config =
{
  //--- Controller clocks ---
  .XtalFreq       = 0,                         // CLKIN is not a crystal
  .OscFreq        = 40000000,                  // CLKIN is an oscillator
  .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN,
  .ClkoPinConfig  = MCP251XFD_CLKO_SOF,
  .SYSCLK_Result  = &MCP2518FD_SYSCLK,
  //--- CAN configuration ---
  .BusConfig      =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,                    // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay      = 300,                  // The transceiver is a ATA6563-GAQW1 on the MCP2518FD click board. The worst delay is from Normal mode, Rising edge at pin TXD or Falling edge at pin TXD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats   = &MCP2518FD_BitTimeStats,
  .Bandwidth      = MCP251XFD_NO_DELAY,
  .ControlFlags   = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR      // Transition to Restricted Operation Mode on system error
                  | MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS     // ESI reflects error status of CAN controller
                  | MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS   // Restricted retransmission attempts, MCP251XFD_FIFO.Attempts (CiFIFOCONm.TXAT) is used
                  | MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE    // Bit Rate Switching is Enabled, Bit Rate Switching depends on BRS in the Transmit Message Object
                  | MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR // Protocol Exception is treated as a Form Error. A recessive "res bit" following a recessive FDF bit is called a Protocol Exception
                  | MCP251XFD_CANFD_USE_ISO_CRC                 // Include Stuff Bit Count in CRC Field and use Non-Zero CRC Initialization Vector according to ISO 11898-1:2015
                  | MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,  // Don�t use RRS; SID<10:0> according to ISO 11898-1:2015
  //--- GPIOs and Interrupts pins ---
  .GPIO0PinMode   = MCP251XFD_PIN_AS_GPIO0_OUT,
  .GPIO1PinMode   = MCP251XFD_PIN_AS_INT1_RX,
  .INTsOutMode    = MCP251XFD_PINS_PUSHPULL_OUT,
  .TXCANOutMode   = MCP251XFD_PINS_PUSHPULL_OUT,
  //--- Interrupts ---
  .SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS - MCP251XFD_INT_TX_EVENT,
};

//-----------------------------------------------------------------------------

MCP251XFD_RAMInfos MCP2518FD_RAMInfos; //!< RAM informations will be stored here after using #MCP251XFD_ConfigureFIFOList()

//! Configuration structure for FIFO of the MCP2518FD on MIKROBUS1
MCP251XFD_FIFO MCP2518FD_FIFOlist[MCP2518FD_FIFO_COUNT] =
{
  { .Name = MCP251XFD_FIFO1, .Size = MCP251XFD_FIFO_26_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_64BYTE, .Direction = MCP251XFD_RECEIVE_FIFO, .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX, .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT, .RAMInfos = &MCP2518FD_RAMInfos, },
};

//-----------------------------------------------------------------------------

//! Configuration structure for Filters of the MCP2518FD on MIKROBUS1
MCP251XFD_Filter MCP2518FD_FilterList[MCP2518FD_FILTER_COUNT] =
{
  { .Filter = MCP251XFD_FILTER0, .EnableFilter = true, .Match = MCP251XFD_MATCH_SID_EID, .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES, .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES, .PointTo = MCP251XFD_FIFO1, },
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// TCAN4550 External CAN controller on MIKROBUS2
//********************************************************************************************************************

//! Peripheral structure of the TCAN4550 on MIKROBUS2 on the V71_XplainedUltra_CAN_Shield board
TCAN455X TCAN4550_MB2 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = TCAN455X_DRIVER_NORMAL_USE,
  .InternalConfig = 0,
  //--- MCAN peripheral ---
  .SPIchipSelect  = 0x1 << 1, // Y1 output on U7
  .SPI            = &SPI0_V71,
  .SPIclockSpeed  = 1800000,
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats TCAN4550_BitTimeStats = { 0 }; //!< TCAN4550 Bit Time stat
uint32_t TCAN4550_SYSCLK = 0; //!< SYSCLK frequency will be stored here after using #Init_TCAN455X()

//! Configuration structure of the TCAN4550 on MIKROBUS2 on the V71_XplainedUltra_CAN_Shield board
TCAN455X_Config TCAN4550_Conf =
{
  //--- Controller clocks ---
  .XtalFreq         = TCAN455X_CLKIN_FREQ_40MHz,
  .FailSafeEnable   = false,
  .SleepWakeDisable = false,
    
  //--- Watchdog Timer ---
  .EnableWatchdog   = false,
  .WatchdogTimeout  = TCAN455X_WD_TIMER_6s,
  .WDtimeoutAction  = TCAN455X_WD_ACTION_SET_INTERRUPT_FLAG,

  //--- CAN configuration ---
  .MCANconf =
  {
    //--- Controller clocks ---
    .MessageRAMwatchdogConf = 0,
    .SYSCLK_Result          = &TCAN4550_SYSCLK,
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
      .TransceiverDelay      = 300,                  // The transceiver is inside the TCAN4550 on the board. The worst delay is Propagation delay time, high TXD_INT to Driver Recessive
      .NominalSamplePoint    = 75,                   // Nominal sample point in percent
      .DataSamplePoint       = 75,                   // Data sample point in percent
    },
    .BitTimeStats          = &TCAN4550_BitTimeStats,
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
  },

  //--- GPIOs and Interrupts pins ---
  .GPIO1PinMode      = TCAN455X_GPO1_MCAN_INT1,
  .GPIO2PinMode      = TCAN455X_GPO2_MCAN_INT0,
  .WakePinConf       = TCAN455X_WAKE_PIN_DISABLED,
  .INHpinEnable      = false,
  .nWKRQconfig       = TCAN455X_nWKRQ_MIRRORS_INH_FUNCTION,
  .nWKRQvoltageRef   = TCAN455X_nWKRQ_Vio_VOLTAGE_RAIL,

  //--- Interrupts ---
  .SysInterruptFlags = TCAN455X_INT_ENABLE_ALL_EVENTS,
};

//-----------------------------------------------------------------------------

CAN_RAMconfig TCAN4550_FIFObuff_RAMInfos[TCAN4550_OBJ_COUNT]; //!< RAM informations will be stored here after using #Init_TCAN455X()

//! Configuration structure for FIFO of the TCAN4550 on MIKROBUS2
MCAN_FIFObuff TCAN4550_FIFObuffList[TCAN4550_OBJ_COUNT] =
{
  { .Name = MCAN_RX_FIFO0,  .Size =  6, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[0], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_RX_FIFO1,  .Size =  6, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[1], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_RX_BUFFER, .Size =  6, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT | MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT, .WatermarkLevel = 32, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[2], }, // 64 * (2 * UINT32 + 64)
  { .Name = MCAN_TEF,       .Size = 10, .Payload = MCAN_8_BYTES,  .ControlFlags = MCAN_RX_FIFO_BLOCKING_MODE, .InterruptFlags = MCAN_FIFO_EVENT_NEW_MESSAGE_INT   | MCAN_FIFO_EVENT_LOST_MESSAGE_INT,   .WatermarkLevel = 16, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[3], }, // 32 *  2 * UINT32
  { .Name = MCAN_TX_BUFFER, .Size =  4, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_TX_BUFFER_MODE,        .InterruptFlags = MCAN_FIFO_NO_INTERRUPT_FLAGS,                                           .WatermarkLevel =  8, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[4], }, // 16 * (2 * UINT32 + 64)
  { .Name = MCAN_TXQ_FIFO,  .Size =  4, .Payload = MCAN_64_BYTES, .ControlFlags = MCAN_TX_FIFO_MODE,          .InterruptFlags = MCAN_FIFO_TRANSMIT_FIFO_EMPTY_INT,                                      .WatermarkLevel =  8, .RAMInfos = &TCAN4550_FIFObuff_RAMInfos[5], }, // 16 * (2 * UINT32 + 64)
};

//-----------------------------------------------------------------------------

//! Configuration structure for Filters of the TCAN4550 on MIKROBUS2
MCAN_Filter TCAN4550_FilterList[TCAN4550_FILTER_COUNT] =
{
  //--- SID filters ---
  { .Filter = 0, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_REJECT_ID,    .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = false, .DualID    = { .AcceptanceID1 = 0x000, .AcceptanceID2  = 0x001, }, }, // Reject 0x000 and 0x001
  { .Filter = 1, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_RANGE, .Config = MCAN_FILTER_SET_PRIORITY, .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = false, .RangeID   = { .MinID         = 0x002, .MaxID          = 0x00F, }, }, // High priority message for 0x002 to 0x00F, no store
  { .Filter = 2, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_SET_PRIORITY, .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .IDandMask = { .AcceptanceID  = 0x010, .AcceptanceMask = 0x7F0, }, }, // High priority message for 0x010 to 0x01F, store to FIFO 0
  { .Filter = 3, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_BUFFER,    .ExtendedID = false, .IDbuffer  = { .AcceptanceID  = 0x200, .BufferPosition = 2,     }, }, // Store message 0x200 to buffer 2
  { .Filter = 4, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_MASK,  .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .IDandMask = { .AcceptanceID  = 0x600, .AcceptanceMask = 0x700, }, }, // Range 0x600 to 0x6FF
  { .Filter = 5, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_DUAL_ID,  .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .DualID    = { .AcceptanceID1 = 0x700, .AcceptanceID2  = 0x701, }, }, // IDs 0x700 and 0x701
  { .Filter = 6, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_ONLY_SID, .Type = MCAN_FILTER_MATCH_ID_RANGE, .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO0,     .ExtendedID = false, .RangeID   = { .MinID         = 0x702, .MaxID          = 0x7FF, }, }, // Range 0x702 to 0x7FF

  //--- EID filters ---
  { .Filter = 0, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_REJECT_ID,    .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = true, .DualID    = { .AcceptanceID1 = 0x00000000, .AcceptanceID2  = 0x00000001, }, }, // Reject 0x00000000 and 0x00000001
  { .Filter = 1, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE,      .Config = MCAN_FILTER_SET_PRIORITY, .PointTo = MCAN_NO_FIFO_BUFF, .ExtendedID = true, .RangeID   = { .MinID         = 0x00000002, .MaxID          = 0x0000000F, }, }, // High priority message for 0x00000002 to 0x0000000F, no store
  { .Filter = 2, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_SET_PRIORITY, .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .IDandMask = { .AcceptanceID  = 0x00000010, .AcceptanceMask = 0x1FFFFFF0, }, }, // High priority message for 0x00000010 to 0x0000001F, store to FIFO 1
  { .Filter = 3, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_BUFFER,    .ExtendedID = true, .IDbuffer  = { .AcceptanceID  = 0x08000000, .BufferPosition = 3,          }, }, // Store message 0x08000000 to buffer 3
  { .Filter = 4, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK,       .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .IDandMask = { .AcceptanceID  = 0x10000000, .AcceptanceMask = 0x1F000000, }, }, // Range 0x10000000 to 0x10FFFFFF
  { .Filter = 5, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_DUAL_ID,       .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .DualID    = { .AcceptanceID1 = 0x14000000, .AcceptanceID2  = 0x14000001, }, }, // IDs 0x14000000 and 0x14000001
  { .Filter = 6, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE,      .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .RangeID   = { .MinID         = 0x18000000, .MaxID          = 0x1C000000, }, }, // Range 0x18000000 to 0x1C000000
  { .Filter = 7, .EnableFilter = true, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_RANGE_MASK, .Config = MCAN_FILTER_NO_CONFIG,    .PointTo = MCAN_RX_FIFO1,     .ExtendedID = true, .RangeID   = { .MinID         = 0x1C000000, .MaxID          = 0x1F000000, }, }, // Range 0x1C000000 to 0x3FFFFFFF but with ExtendedIDrangeMask mask: 0x00000000 to 0x1FFFFFFF
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP2517FD External CAN controller on MIKROBUS3
//********************************************************************************************************************

//! Component structure of the MCP2517FD on MIKROBUS3 on the V71_XplainedUltra_CAN_Shield board
MCP251XFD MCP2517FD_MB3 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCP251XFD_DRIVER_NORMAL_USE
                  | MCP251XFD_DRIVER_SAFE_RESET
                  | MCP251XFD_DRIVER_USE_READ_WRITE_CRC
                  | MCP251XFD_DRIVER_INIT_SET_RAM_AT_0
                  | MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ,
  //--- IO configuration ---
  .GPIOsDirection = 0,
  .GPIOsOutLevel  = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,
  //--- Interface driver call functions ---
  .SPIchipSelect  = 0x6 << 1, // Y6 output on U7
  .SPI            = &SPI0_V71,
  .SPIclockSpeed  = 17000000, // 17MHz
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
  //--- CRC16-USB call function ---
  .fnComputeCRC16 = MCP251XFD_ComputeCRC16CMS,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCP2517FD_BitTimeStats = { 0 }; //!< MCP2517FD Bit Time stat
uint32_t MCP2517FD_SYSCLK; //!< SYSCLK frequency will be stored here after using #Init_MCP251XFD()

//! Configuration structure of the MCP2517FD on MIKROBUS3
MCP251XFD_Config MCP2517FD_Config =
{
  //--- Controller clocks ---
  .XtalFreq       = 0,                         // CLKIN is not a crystal
  .OscFreq        = 40000000,                  // CLKIN is an oscillator
  .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN,
  .ClkoPinConfig  = MCP251XFD_CLKO_SOF,
  .SYSCLK_Result  = &MCP2517FD_SYSCLK,
  //--- CAN configuration ---
  .BusConfig      =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,                    // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay      = 300,                  // The transceiver is a ATA6563-GAQW1 on the MCP2517FD click board. The worst delay is from Normal mode, Rising edge at pin TXD or Falling edge at pin TXD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats   = &MCP2517FD_BitTimeStats,
  .Bandwidth      = MCP251XFD_NO_DELAY,
  .ControlFlags   = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR      // Transition to Restricted Operation Mode on system error
                  | MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS     // ESI reflects error status of CAN controller
                  | MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS   // Restricted retransmission attempts, MCP251XFD_FIFO.Attempts (CiFIFOCONm.TXAT) is used
                  | MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE    // Bit Rate Switching is Enabled, Bit Rate Switching depends on BRS in the Transmit Message Object
                  | MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR // Protocol Exception is treated as a Form Error. A recessive "res bit" following a recessive FDF bit is called a Protocol Exception
                  | MCP251XFD_CANFD_USE_ISO_CRC                 // Include Stuff Bit Count in CRC Field and use Non-Zero CRC Initialization Vector according to ISO 11898-1:2015
                  | MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,  // Don�t use RRS; SID<10:0> according to ISO 11898-1:2015
  //--- GPIOs and Interrupts pins ---
  .GPIO0PinMode   = MCP251XFD_PIN_AS_GPIO0_OUT,
  .GPIO1PinMode   = MCP251XFD_PIN_AS_INT1_RX,
  .INTsOutMode    = MCP251XFD_PINS_PUSHPULL_OUT,
  .TXCANOutMode   = MCP251XFD_PINS_PUSHPULL_OUT,
  //--- Interrupts ---
  .SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS - MCP251XFD_INT_TX_EVENT,
};

//-----------------------------------------------------------------------------

MCP251XFD_RAMInfos MCP2517FD_RAMInfos; //!< RAM informations will be stored here after using #MCP251XFD_ConfigureFIFOList()

//! Configuration structure for FIFO of the MCP2517FD on MIKROBUS3
MCP251XFD_FIFO MCP2517FD_FIFOlist[MCP2517FD_FIFO_COUNT] =
{
  { .Name = MCP251XFD_FIFO1, .Size = MCP251XFD_FIFO_26_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_64BYTE, .Direction = MCP251XFD_RECEIVE_FIFO, .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX, .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT, .RAMInfos = &MCP2518FD_RAMInfos, },
};

//-----------------------------------------------------------------------------

//! Configuration structure for Filters of the MCP2517FD on MIKROBUS3
MCP251XFD_Filter MCP2517FD_FilterList[MCP2517FD_FILTER_COUNT] =
{
  { .Filter = MCP251XFD_FILTER0, .EnableFilter = true, .Match = MCP251XFD_MATCH_SID_EID, .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES, .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES, .PointTo = MCP251XFD_FIFO1, },
};

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------