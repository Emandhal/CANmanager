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
//-----------------------------------------------------------------------------
#if !defined(__cplusplus)
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************

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
  .I2C =
  {
    .InterfaceDevice = TWIHS0,
    .UniqueID        = TWIHS_UNIQUE_ID,
    .fnI2C_Init      = TWIHS_MasterInit_Gen,
    .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
  },
  .I2CclockSpeed = 400000, // I2C speed at 400kHz
  //--- Device address --
  .AddrA2A1A0    = MCP230XX_ADDR(0, 0, 1),
  .DeviceName    = MCP23008,
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
  .PORTindex             = 0,
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
  .I2C =
  {
    .InterfaceDevice = TWIHS0,
    .UniqueID        = TWIHS_UNIQUE_ID,
    .fnI2C_Init      = TWIHS_MasterInit_Gen,
    .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
  },
  .I2CclockSpeed = 400000, // I2C speed at 400kHz
  //--- Device address --
  .AddrA2A1A0    = MCP230XX_ADDR(0, 0, 0),
  .DeviceName    = MCP23017,
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
  .PORTindex             = 0,
};

//! PORT interface of the MCP230XX as U12 PORT GPB on the CAN_Shield board
PORT_Interface PORTGPB_U8 =
{
  .InterfaceDevice       = &MCP23017_U8,
  .UniqueID              = MCP230XX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP230XX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP230XX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP230XX_SetPORToutputLevel_Gen,
  .PORTindex             = 1,
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
  .SPI =
  {
    .InterfaceDevice = SPI0,
    .UniqueID        = SPI_UNIQUE_ID,
    .fnSPI_Init      = SPI_MasterInit_Gen,
    .fnSPI_Transfer  = SPI_PacketTransfer_Gen,
  },
  .SPIclockSpeed = 10000000, // SPI speed at 10MHz
  //--- Device address --
  .AddrA2A1A0    = MCP23SXX_ADDR(0, 0, 0),
  .DeviceName    = MCP23S17,
};

//! Configuration structure of the MCP23SXX as U9
struct MCP23SXX_Config MCP23S17_U9_Conf =
{
  .GPA =
  {
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA0: SJA1000_AD0
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA1: SJA1000_AD1
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA2: SJA1000_AD2
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA3: SJA1000_AD3
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA4: SJA1000_AD4
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA5: SJA1000_AD5
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA6: SJA1000_AD6
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC |                                   MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPA7: SJA1000_AD7
  },
  .GPB =
  {
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB0: SJA1000_ALE/AS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB1: SJA1000_nCS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB2: SJA1000_nRD/E
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB3: SJA1000_nWR
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB4: MCP_Trans_CS
    MCP23SXX_PIN_AS_OUTPUT | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_HIGH | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB5: SJA_Trans_CS
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB6: SJA_Trans_INH
    MCP23SXX_PIN_AS_INPUT  | MCP23SXX_GPIO_STATE_SAME_LOGIC | MCP23SXX_GPIO_OUTPUT_STATE_LOW  | MCP23SXX_GPIO_PULLUP_DISABLE | MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE, // GPB7: SJA_Trans_WAKE
  },
};

//! PORT interface of the MCP230XX as U9 PORT GPA on the CAN_Shield board
PORT_Interface PORTGPA_U9 =
{
  .InterfaceDevice       = &MCP23S17_U9,
  .UniqueID              = MCP23SXX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP23SXX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP23SXX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP23SXX_SetPORToutputLevel_Gen,
  .PORTindex             = 0,
};

//! PORT interface of the MCP230XX as U9 PORT GPB on the CAN_Shield board
PORT_Interface PORTGPB_U9 =
{
  .InterfaceDevice       = &MCP23S17_U9,
  .UniqueID              = MCP23SXX_UNIQUE_ID,
  .fnPORT_SetDirection   = MCP23SXX_SetPORTdirection_Gen,
  .fnPORT_GetInputLevel  = MCP23SXX_GetPORTinputLevel_Gen,
  .fnPORT_SetOutputLevel = MCP23SXX_SetPORToutputLevel_Gen,
  .PORTindex             = 1,
};

//-----------------------------------------------------------------------------





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------