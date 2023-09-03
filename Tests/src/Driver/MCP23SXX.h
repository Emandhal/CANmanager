/*!*****************************************************************************
 * @file    MCP23SXX.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.1.0
 * @date    17/06/2023
 * @brief   MCP23SXX driver
 * @details
 * -> SPI-Compatible (3-wire) 8-bit I/O Expander
 *    Follow datasheet MCP23S08 Rev.F (March 2019)
 *                     MCP23S09 Rev.C (August 2014)
 * -> SPI-Compatible (3-wire) 16-bit I/O Expander
 *    Follow datasheet MCP23S17 Rev.C (July 2016)
 *                     MCP23S18 Rev.A (September 2008)
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
 * 1.1.0    SPI interface rework
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef MCP23SXX_H_INC
#define MCP23SXX_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include "ErrorsDef.h"
#include "SPI_Interface.h"
#ifdef USE_GENERICS_DEFINED
#  include "GPIO_Interface.h"
#endif
//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#  define __MCP23SXX_PACKED__
#  define MCP23SXX_PACKITEM    __pragma(pack(push, 1))
#  define MCP23SXX_UNPACKITEM  __pragma(pack(pop))
#else
#  define __MCP23SXX_PACKED__  __attribute__((packed))
#  define MCP23SXX_PACKITEM
#  define MCP23SXX_UNPACKITEM
#endif

//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define MCP23SXX_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP23SXX limits definitions
//********************************************************************************************************************

// Limits definitions
#define MCP23S08_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S08 Max SPI clock frequency
#define MCP23S09_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S09 Max SPI clock frequency
#define MCP23S0X_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S0X Max SPI clock frequency
#define MCP23S17_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S17 Max SPI clock frequency
#define MCP23S18_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S18 Max SPI clock frequency
#define MCP23S1X_SPICLOCK_MAX   ( 10000000u ) //!< MCP23S1X Max SPI clock frequency

#define MCP23SXX_SPICLOCK_SAFE  (  5000000u ) //!< MCP23SXX Safe SPI clock frequency
#define MCP23SXX_SPICLOCK_MAX   ( 10000000u ) //!< MCP23SXX Max SPI clock frequency

// Device definitions
#define MCP23SXX_CHIPADDRESS_BASE  ( 0x40 ) //!< MCP23SXX chip base address
#define MCP23SXX_CHIPADDRESS_MASK  ( 0xFE ) //!< MCP23SXX chip base address mask

//-----------------------------------------------------------------------------

/*! @brief Generate the MCP23SXX chip configurable address following the state of A0, A1, and A2
 * You shall set '1' (when corresponding pin is connected to +V) or '0' (when corresponding pin is connected to Ground) on each parameter
 */
#define MCP23SXX_ADDR(A2, A1, A0)  ( (uint8_t)((((A2) & 0x01) << 3) | (((A1) & 0x01) << 2) | (((A0) & 0x01) << 1)) )

/*! @brief Generate the MCP23SXX chip configurable address following the resistor divider and its power supply
 * You just have to put the values showed in the schematic and put them in this define and the formula will set the address for you
 * R1 is the resistor between Vdd and the ADDR pin and, R2 is the resistor between ADDR pin and the ground
 */
#define MCP23SXX_ADDR_RES(R1, R2)  ( (uint8_t)((((8u * (uint32_t)(R2)) / ((uint32_t)(R1) + (uint32_t)(R2))) << 1) & MCP23SXX_CHIPADDRESS_MASK) )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP23SXX Register list
//********************************************************************************************************************

//! MCP23SXX registers list
typedef enum
{
  // MCP23S08, and MCP23S09 registers
  RegMCP23SXX_IODIR     = 0x00, //!< I/O direction register
  RegMCP23SXX_IPOL      = 0x01, //!< Input polarity register
  RegMCP23SXX_GPINTEN   = 0x02, //!< Interrupt-On-Change control register
  RegMCP23SXX_DEFVAL    = 0x03, //!< Default compare register for Interrupt-On-Change
  RegMCP23SXX_INTCON    = 0x04, //!< Interrupt control register
  RegMCP23SXX_IOCON     = 0x05, //!< Configuration register
  RegMCP23SXX_GPPU      = 0x06, //!< Pull-Up resistor configuration register
  RegMCP23SXX_INTF      = 0x07, //!< Interrupt flag register
  RegMCP23SXX_INTCAP    = 0x08, //!< Interrupt capture register
  RegMCP23SXX_GPIO      = 0x09, //!< Port register
  RegMCP23SXX_OLAT      = 0x0A, //!< Output latch register

  // MCP23S17, and MCP23S18 registers (BANK = 1)
  RegMCP23SXX1_IODIRA   = 0x00, //!< I/O direction register A
  RegMCP23SXX1_IPOLA    = 0x01, //!< Input polarity register A
  RegMCP23SXX1_GPINTENA = 0x02, //!< Interrupt-On-Change pins register A
  RegMCP23SXX1_DEFVALA  = 0x03, //!< Default compare register for Interrupt-On-Change A
  RegMCP23SXX1_INTCONA  = 0x04, //!< Interrupt-On-Change control register A
  RegMCP23SXX1_IOCONA   = 0x05, //!< I/O expander control register
  RegMCP23SXX1_GPPUA    = 0x06, //!< Pull-Up resistor configuration register A
  RegMCP23SXX1_INTFA    = 0x07, //!< Interrupt flag register A
  RegMCP23SXX1_INTCAPA  = 0x08, //!< Interrupt captured value for port register A
  RegMCP23SXX1_GPIOA    = 0x09, //!< Port register A
  RegMCP23SXX1_OLATA    = 0x0A, //!< Output latch register A
  RegMCP23SXX1_IODIRB   = 0x10, //!< I/O direction register B
  RegMCP23SXX1_IPOLB    = 0x11, //!< Input polarity register B
  RegMCP23SXX1_GPINTENB = 0x12, //!< Interrupt-On-Change pins register B
  RegMCP23SXX1_DEFVALB  = 0x13, //!< Default compare register for Interrupt-On-Change B
  RegMCP23SXX1_INTCONB  = 0x14, //!< Interrupt-On-Change control register B
  RegMCP23SXX1_IOCONB   = 0x15, //!< I/O expander control register
  RegMCP23SXX1_GPPUB    = 0x16, //!< Pull-Up resistor configuration register B
  RegMCP23SXX1_INTFB    = 0x17, //!< Interrupt flag register B
  RegMCP23SXX1_INTCAPB  = 0x18, //!< Interrupt captured value for port register B
  RegMCP23SXX1_GPIOB    = 0x19, //!< Port register B
  RegMCP23SXX1_OLATB    = 0x1A, //!< Output latch register B

  // MCP23S17, and MCP23S18 registers (BANK = 0)
  RegMCP23SXX0_IODIRA   = 0x00, //!< I/O direction register A
  RegMCP23SXX0_IODIRB   = 0x01, //!< I/O direction register B
  RegMCP23SXX0_IPOLA    = 0x02, //!< Input polarity register A
  RegMCP23SXX0_IPOLB    = 0x03, //!< Input polarity register B
  RegMCP23SXX0_GPINTENA = 0x04, //!< Interrupt-On-Change pins register A
  RegMCP23SXX0_GPINTENB = 0x05, //!< Interrupt-On-Change pins register B
  RegMCP23SXX0_DEFVALA  = 0x06, //!< Default compare register for Interrupt-On-Change A
  RegMCP23SXX0_DEFVALB  = 0x07, //!< Default compare register for Interrupt-On-Change B
  RegMCP23SXX0_INTCONA  = 0x08, //!< Interrupt-On-Change control register A
  RegMCP23SXX0_INTCONB  = 0x09, //!< Interrupt-On-Change control register B
  RegMCP23SXX0_IOCONA   = 0x0A, //!< I/O expander control register
  RegMCP23SXX0_IOCONB   = 0x0B, //!< I/O expander control register
  RegMCP23SXX0_GPPUA    = 0x0C, //!< Pull-Up resistor configuration register A
  RegMCP23SXX0_GPPUB    = 0x0D, //!< Pull-Up resistor configuration register B
  RegMCP23SXX0_INTFA    = 0x0E, //!< Interrupt flag register A
  RegMCP23SXX0_INTFB    = 0x0F, //!< Interrupt flag register B
  RegMCP23SXX0_INTCAPA  = 0x10, //!< Interrupt captured value for port register A
  RegMCP23SXX0_INTCAPB  = 0x11, //!< Interrupt captured value for port register B
  RegMCP23SXX0_GPIOA    = 0x12, //!< Port register A
  RegMCP23SXX0_GPIOB    = 0x13, //!< Port register B
  RegMCP23SXX0_OLATA    = 0x14, //!< Output latch register A
  RegMCP23SXX0_OLATB    = 0x15, //!< Output latch register B
} eMCP23SXX_Registers;

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP23SXX Specific Controller Registers
//********************************************************************************************************************

//! MCP23S08 Configuration Register
MCP23SXX_PACKITEM
typedef union __MCP23SXX_PACKED__ MCP23S08_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t       : 1; //!< 0
    uint8_t INTPOL: 1; //!< 1   - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2   - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t HAEN  : 1; //!< 3   - Hardware Address Enable (MCP23S08 only), Address pins are always enabled on MCP23S08: 1 = Enables the MCP23S08 address pins ; 0 = Disables the MCP23S08 address pins
    uint8_t DISSLW: 1; //!< 4   - Slew Rate Control Bit for SDA Output. If enabled, the SDA slew rate will be controlled when driving from a high to a low: 1 = Slew rate disabled ; 0 = Slew rate enabled
    uint8_t SEQOP : 1; //!< 5   - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t       : 2; //!< 6-7
  } Bits;
} MCP23S08_IOCON_Register;
MCP23SXX_UNPACKITEM;
MCP23SXX_CONTROL_ITEM_SIZE(MCP23S08_IOCON_Register, 1);

#define MCP23S08_IOCON_INTPOL_ACTIVEHIGH  (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23S08_IOCON_INTPOL_ACTIVELOW   (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23S08_IOCON_ODR_OPENDRAIN      (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23S08_IOCON_ODR_ACTIVEDRIVER   (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23S08_IOCON_HAEN_DISABLE       (0x1u << 3) //!< Disables the MCP23S08 address pins
#define MCP23S08_IOCON_HAEN_ENABLE        (0x0u << 3) //!< Enables the MCP23S08 address pins
#define MCP23S08_IOCON_DISSLW_DISABLE     (0x1u << 4) //!< Disable Slew Rate Control Bit for SDA Output
#define MCP23S08_IOCON_DISSLW_ENABLE      (0x0u << 4) //!< Enable Slew Rate Control Bit for SDA Output
#define MCP23S08_IOCON_SEQOP_DISABLE      (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23S08_IOCON_SEQOP_ENABLE       (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments

//-----------------------------------------------------------------------------

//! MCP23S09 Configuration Register
MCP23SXX_PACKITEM
typedef union __MCP23SXX_PACKED__ MCP23S09_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t INTCC : 1; //!< 0   - Interrupt Clearing Control: 1 = Reading INTCAP register clears the Interrupt ; 0 = Reading GPIO register clears the Interrupt
    uint8_t INTPOL: 1; //!< 1   - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2   - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t       : 2; //!< 3-4
    uint8_t SEQOP : 1; //!< 5   - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t       : 2; //!< 6-7
  } Bits;
} MCP23S09_IOCON_Register;
MCP23SXX_UNPACKITEM;
MCP23SXX_CONTROL_ITEM_SIZE(MCP23S09_IOCON_Register, 1);

#define MCP23S09_IOCON_INTCC_INTCAPCLEARS  (0x1u << 0) //!< Reading INTCAP register clears the Interrupt
#define MCP23S09_IOCON_INTCC_GPIOCLEARS    (0x0u << 0) //!< Reading GPIO register clears the Interrupt
#define MCP23S09_IOCON_INTPOL_ACTIVEHIGH   (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23S09_IOCON_INTPOL_ACTIVELOW    (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23S09_IOCON_ODR_OPENDRAIN       (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23S09_IOCON_ODR_ACTIVEDRIVER    (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23S09_IOCON_SEQOP_DISABLE       (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23S09_IOCON_SEQOP_ENABLE        (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments

//-----------------------------------------------------------------------------

//! MCP23S17 Configuration Register
MCP23SXX_PACKITEM
typedef union __MCP23SXX_PACKED__ MCP23S17_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t       : 1; //!< 0
    uint8_t INTPOL: 1; //!< 1 - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2 - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t HAEN  : 1; //!< 3 - Hardware Address Enable (MCP23S17 only), Address pins are always enabled on MCP23S17: 1 = Enables the MCP23S17 address pins ; 0 = Disables the MCP23S17 address pins
    uint8_t DISSLW: 1; //!< 4 - Slew Rate Control Bit for SDA Output. If enabled, the SDA slew rate will be controlled when driving from a high to a low: 1 = Slew rate disabled ; 0 = Slew rate enabled
    uint8_t SEQOP : 1; //!< 5 - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t MIRROR: 1; //!< 6 - INT Pins Mirror bit: 1 = The INT pins are internally connected ; 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
    uint8_t BANK  : 1; //!< 7 - Controls how the registers are addressed: 1 = The registers associated with each port are separated into different banks ; 0 = The registers are in the same bank (addresses are sequential)
  } Bits;
} MCP23S17_IOCON_Register;
MCP23SXX_UNPACKITEM;
MCP23SXX_CONTROL_ITEM_SIZE(MCP23S17_IOCON_Register, 1);

#define MCP23S17_IOCON_INTPOL_ACTIVEHIGH  (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23S17_IOCON_INTPOL_ACTIVELOW   (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23S17_IOCON_ODR_OPENDRAIN      (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23S17_IOCON_ODR_ACTIVEDRIVER   (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23S17_IOCON_HAEN_DISABLE       (0x1u << 3) //!< Disables the MCP23S17 address pins
#define MCP23S17_IOCON_HAEN_ENABLE        (0x0u << 3) //!< Enables the MCP23S17 address pins
#define MCP23S17_IOCON_DISSLW_DISABLE     (0x1u << 4) //!< Disable Slew Rate Control Bit for SDA Output
#define MCP23S17_IOCON_DISSLW_ENABLE      (0x0u << 4) //!< Enable Slew Rate Control Bit for SDA Output
#define MCP23S17_IOCON_SEQOP_DISABLE      (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23S17_IOCON_SEQOP_ENABLE       (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments
#define MCP23S17_IOCON_MIRROR_DISABLE     (0x1u << 6) //!< The INT pins are internally connected
#define MCP23S17_IOCON_MIRROR_ENABLE      (0x0u << 6) //!< The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
#define MCP23S17_IOCON_BANK_DISABLE       (0x1u << 7) //!< The registers associated with each port are separated into different banks
#define MCP23S17_IOCON_BANK_ENABLE        (0x0u << 7) //!< The registers are in the same bank (addresses are sequential)

//-----------------------------------------------------------------------------

//! MCP23S18 Configuration Register
MCP23SXX_PACKITEM
typedef union __MCP23SXX_PACKED__ MCP23S18_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t INTCC : 1; //!< 0   - Interrupt Clearing Control: 1 = Reading INTCAP register clears the Interrupt ; 0 = Reading GPIO register clears the Interrupt
    uint8_t INTPOL: 1; //!< 1   - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2   - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t       : 2; //!< 3-4
    uint8_t SEQOP : 1; //!< 5   - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t MIRROR: 1; //!< 6   - INT Pins Mirror bit: 1 = The INT pins are internally connected ; 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
    uint8_t BANK  : 1; //!< 7   - Controls how the registers are addressed: 1 = The registers associated with each port are separated into different banks ; 0 = The registers are in the same bank (addresses are sequential)
  } Bits;
} MCP23S18_IOCON_Register;
MCP23SXX_UNPACKITEM;
MCP23SXX_CONTROL_ITEM_SIZE(MCP23S18_IOCON_Register, 1);

#define MCP23S18_IOCON_INTCC_INTCAPCLEARS  (0x1u << 0) //!< Reading INTCAP register clears the Interrupt
#define MCP23S18_IOCON_INTCC_GPIOCLEARS    (0x0u << 0) //!< Reading GPIO register clears the Interrupt
#define MCP23S18_IOCON_INTPOL_ACTIVEHIGH   (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23S18_IOCON_INTPOL_ACTIVELOW    (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23S18_IOCON_ODR_OPENDRAIN       (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23S18_IOCON_ODR_ACTIVEDRIVER    (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23S18_IOCON_SEQOP_DISABLE       (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23S18_IOCON_SEQOP_ENABLE        (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments
#define MCP23S18_IOCON_MIRROR_DISABLE      (0x1u << 6) //!< The INT pins are internally connected
#define MCP23S18_IOCON_MIRROR_ENABLE       (0x0u << 6) //!< The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
#define MCP23S18_IOCON_BANK_DISABLE        (0x1u << 7) //!< The registers associated with each port are separated into different banks
#define MCP23S18_IOCON_BANK_ENABLE         (0x0u << 7) //!< The registers are in the same bank (addresses are sequential)

//-----------------------------------------------------------------------------

//! List of supported devices
typedef enum
{
  MCP23S08                           , //!< MCP23S08 supported
  MCP23S09                           , //!< MCP23S09 supported
  MCP23S0X                           , //!< Autodetected a MCP23S08 or aMCP23S09
  MCP23SXX_DEV_1PORT_COUNT           , // 1 port device count
  MCP23S17 = MCP23SXX_DEV_1PORT_COUNT, //!< MCP23S17 supported
  MCP23S18                           , //!< MCP23S18 supported
  MCP23S1X                           , //!< Autodetected a MCP23S17 or aMCP23S18
  eMCP23SXX_DEVICE_COUNT             , // Device count of this enum, keep last of supported devices
  MCP23SXX_AUTODETECT = 0xFF         , //!< Autodetect the device. Can't differentiate the MCP23S08 over the MCP23S09 and the MCP23S17 over the MCP23S18
} eMCP23SXX_Devices;

static const char* const MCP23SXX_DevicesNames[eMCP23SXX_DEVICE_COUNT] =
{
  "MCP23S08",
  "MCP23S09",
  "MCP23S0X",
  "MCP23S17",
  "MCP23S18",
  "MCP23S1X",
};

//-----------------------------------------------------------------------------

//! Structure of device registers link
typedef struct MCP23SXX_RegLinks
{
  uint8_t DevPortCount;        //!< Device 8-pins port count
  // GPIO and Interrupt registers
  eMCP23SXX_Registers IODIR;   //!< Link to IODIR register
  eMCP23SXX_Registers IPOL;    //!< Link to IPOL register
  eMCP23SXX_Registers GPINTEN; //!< Link to GPINTEN register
  eMCP23SXX_Registers DEFVAL;  //!< Link to DEFVAL register
  eMCP23SXX_Registers INTCON;  //!< Link to INTCON register
  eMCP23SXX_Registers GPPU;    //!< Link to GPPU register
  eMCP23SXX_Registers INTF;    //!< Link to INTF register
  eMCP23SXX_Registers INTCAP;  //!< Link to INTCAP register
  eMCP23SXX_Registers GPIO;    //!< Link to GPIO register
  eMCP23SXX_Registers OLAT;    //!< Link to OLAT register
  // Configuration register
  eMCP23SXX_Registers IOCON;   //!< Link to IOCON register
} MCP23SXX_RegLinks;

//-----------------------------------------------------------------------------

#define MCP23SXX_PORT_GP   0 //!< PORT GP index (for MCP23S08 and MCP23S09)
#define MCP23SXX_PORT_GPA  0 //!< PORT GPA index (for MCP23S17 and MCP23S18)
#define MCP23SXX_PORT_GPB  1 //!< PORT GPB index (for MCP23S17 and MCP23S18)

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCP23SXX Driver API
//********************************************************************************************************************

typedef struct MCP23SXX MCP23SXX; //! Typedef of MCP23SXX device object structure

//-----------------------------------------------------------------------------

//! I/O expander configuration flags (can be OR'ed)
typedef enum
{
  MCP23SXX_DEFAULT_COMPONENT_BEHAVIOR           = 0x00, //!< [All devices] This mode alone says to the driver to not touch the default configuration of the component
  MCP23SXX_READING_INTCAP_CLEARS_INTERRUPT      = 0x01, //!< [MCP23S09 and MCP23S18] Reading INTCAP register clears the interrupt
  MCP23SXX_READING_GPIO_CLEARS_INTERRUPT        = 0x00, //!< [MCP23S09 and MCP23S18] Reading GPIO register clears the interrupt
  MCP23SXX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH  = 0x02, //!< The polarity of the INT output pin is Active-High
  MCP23SXX_INT_PIN_OUTPUT_POLARITY_ACTIVE_LOW   = 0x00, //!< The polarity of the INT output pin is Active-Low
  MCP23SXX_INT_PIN_OUTPUT_OPEN_DRAIN            = 0x04, //!< Configures the INT pin as open-drain output (overrides the INTPOL bit)
  MCP23SXX_INT_PIN_OUTPUT_PUSH_PULL             = 0x00, //!< Configures the INT pin as active driver output (INTPOL bit sets the polarity)
  MCP23SXX_ADDRESS_PIN_ENABLE                   = 0x08, //!< [MCP23S08 and MCP23S17] Enables the address pins
  MCP23SXX_ADDRESS_PIN_DISABLE                  = 0x00, //!< [MCP23S08 and MCP23S17] Disables the address pins
  MCP23SXX_SLEW_RATE_CONTROL_SDA_OUTPUT_DISABLE = 0x10, //!< [MCP23S08 and MCP23S17] Slew Rate control bit for SDA output disabled
  MCP23SXX_SLEW_RATE_CONTROL_SDA_OUTPUT_ENABLE  = 0x00, //!< [MCP23S08 and MCP23S17] Slew Rate control bit for SDA output enable
//MCP23SXX_SEQUENTIAL_OPERATION_DISABLE         = 0x20, //!< Sequential operation disabled, address pointer does not increment
//MCP23SXX_SEQUENTIAL_OPERATION_ENABLE          = 0x00, //!< Sequential operation enabled, address pointer increments
  MCP23SXX_INT_PIN_MIRRORED                     = 0x40, //!< [MCP23S17 and MCP23S18] The INT pins are internally connected
  MCP23SXX_INT_PIN_INDEPENDANT                  = 0x00, //!< [MCP23S17 and MCP23S18] The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
//MCP23SXX_PORT_REGISTERS_SEPARATED             = 0x80, //!< [MCP23S17 and MCP23S18] The registers associated with each port are separated into different banks
//MCP23SXX_PORT_REGISTERS_SAME_BANK             = 0x00, //!< [MCP23S17 and MCP23S18] The registers are in the same bank (addresses are sequential)
} eMCP23SXX_ControlFlags;

typedef eMCP23SXX_ControlFlags setMCP23SXX_ControlFlags; //!< Set of I/O expander configuration flags (can be OR'ed)

//-----------------------------------------------------------------------------

//! MCP23SXX device object structure
struct MCP23SXX
{
  void *UserDriverData;                  //!< Optional, can be used to store driver data or NULL
  
  //--- Device configuration ---
  setMCP23SXX_ControlFlags ControlFlags; //!< I/O expander configuration flags for general behavior (can be OR'ed)

  //--- PORT configuration ---
  uint8_t PORToutDir[2];                 //!< GPIOs pins direction (0 = set to output ; 1 = set to input). Used to speed up direction change
  uint8_t PORToutLevel[2];               //!< GPIOs pins output level (0 = set to '0' ; 1 = set to '1'). Used to speed up output change

  //--- Interface driver call functions ---
  uint8_t SPIchipSelect;                 //!< This is the Chip Select index that will be set at the call of a transfer
#ifdef USE_DYNAMIC_INTERFACE
  SPI_Interface* SPI;                  //!< This is the SPI_Interface descriptor pointer that will be used to communicate with the device
#else
  SPI_Interface SPI;                   //!< This is the SPI_Interface descriptor that will be used to communicate with the device
#endif
  uint32_t SPIclockSpeed;              //!< Clock frequency of the SPI interface in Hertz

  //--- Device address ---
  uint8_t AddrA2A1A0;                    //!< Device configurable address A2, A1, and A0. You can use the macro MCP23SXX_ADDR() to help filling this parameter. Only these 3 lower bits are used: ....210. where 2 is A2, 1 is A1, 0 is A0, and '.' are fixed by device
  eMCP23SXX_Devices DeviceName;          //!< This is the device name, set to MCP23SXX_AUTODETECT if you don't need specific behavior and the driver to work generically. The initialization of the driver will change this value
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define MCP23SXX_UNIQUE_ID  ( (((uint32_t)'M' << 0) ^ ((uint32_t)'C' << 4) ^ ((uint32_t)'P' << 8) ^ ((uint32_t)'2' << 12) ^ ((uint32_t)'3' << 16) ^ ((uint32_t)'S' << 20) ^ ((uint32_t)'X' << 24) ^ ((uint32_t)'X' << 28)) + __LINE__ + (sizeof(struct MCP23SXX) << 19) )

//-----------------------------------------------------------------------------

//! GPIO configuration (can be OR'ed)
typedef enum
{
  MCP23SXX_PIN_UNUSED                      = 0x00, //!< Unused pin will be set to output, low level, no pull-up, no interrupt
  // GPIO pin configuration
  MCP23SXX_PIN_AS_INPUT                    = 0x01, //!< The pin is configured as an input
  MCP23SXX_PIN_AS_OUTPUT                   = 0x00, //!< The pin is configured as an output
  MCP23SXX_GPIO_STATE_INVERTED_LOGIC       = 0x02, //!< GPIO register bit will reflect the opposite logic state of the input pin
  MCP23SXX_GPIO_STATE_SAME_LOGIC           = 0x00, //!< GPIO register bit will reflect the same logic state of the input pin
  MCP23SXX_GPIO_OUTPUT_STATE_HIGH          = 0x04, //!< GPIO pin output level set as high
  MCP23SXX_GPIO_OUTPUT_STATE_LOW           = 0x00, //!< GPIO pin output level set as low
  MCP23SXX_GPIO_PULLUP_ENABLE              = 0x08, //!< GPIO internal pull-up pin enable
  MCP23SXX_GPIO_PULLUP_DISABLE             = 0x00, //!< GPIO internal pull-up pin disable
  // GPIO interrupt configuration
  MCP23SXX_INTERRUPT_ON_CHANGE_ENABLE      = 0x10, //!< Enable GPIO input pin for interrupt-on-change event
  MCP23SXX_INTERRUPT_ON_CHANGE_DISABLE     = 0x00, //!< Disable GPIO input pin for interrupt-on-change event
  MCP23SXX_VALUE_COMPARED_DEFAULT_VALUE    = 0x20, //!< Pin value is compared against the associated bit is the default value (DEFVAL register)
  MCP23SXX_VALUE_COMPARED_PREVIOUS_VALUE   = 0x00, //!< Pin value is compared against the previous pin value
  MCP23SXX_DEFAULT_VALUE_NO_INT_LEVEL_HIGH = 0x40, //!< Sets the compare value for pins configured to level high for interrupt-on-change. If the associated pin level is low from the register bit, an interrupt occurs
  MCP23SXX_DEFAULT_VALUE_NO_INT_LEVEL_LOW  = 0x00, //!< Sets the compare value for pins configured to level low for interrupt-on-change. If the associated pin level is high from the register bit, an interrupt occurs
} eMCP23SXX_PinConf;

typedef eMCP23SXX_PinConf setMCP23SXX_PinConf; //!< Set of GPIO configuration (can be OR'ed)

//-----------------------------------------------------------------------------

//! MCP23SXX Configuration structure
typedef struct MCP23SXX_Config
{
  //--- GPIOs configuration ---
  union
  {
    setMCP23SXX_PinConf GPs[16];  //!< Configuration for all GPIOs at once (GPA (low) and GPB (high) for MCP23S17 and MCP23S18)
    setMCP23SXX_PinConf GP[8];    //!< Configuration for GP only (for MCP23S08 and MCP23S09)
    struct
    {
      setMCP23SXX_PinConf GPA[8]; //!< Configuration for GPA only (for MCP23S17 and MCP23S18)
      setMCP23SXX_PinConf GPB[8]; //!< Configuration for GPB only (for MCP23S17 and MCP23S18)
    };
  };
} MCP23SXX_Config;

//-----------------------------------------------------------------------------


/*! @brief MCP23SXX initialization
 *
 * This function initializes the MCP23SXX driver and call the initialization of the interface driver (SPI).
 * Next it checks parameters and configures the MCP23SXX
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @param[in] *pConf Is the pointed structure of the device configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_MCP23SXX(MCP23SXX *pComp, const MCP23SXX_Config *pConf);

//-----------------------------------------------------------------------------


/*! @brief Read data from register of the MCP23SXX device
 *
 * This function reads data from a register of a MCP23SXX device
 * @param[in] *pComp Is the pointed structure of the device to read
 * @param[in] reg Is the register to read
 * @param[in] *data Is where the data will be stored
 * @param[in] size Is the size of the data array to read
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_ReadRegister(MCP23SXX *pComp, eMCP23SXX_Registers reg, uint8_t* data, size_t size);

/*! @brief Write data to register of the MCP23SXX device
 *
 * This function writes data to a register of a MCP23SXX device
 * @param[in] *pComp Is the pointed structure of the device to modify
 * @param[in] reg Is the register where data will be written
 * @param[in] *data Is the data array to store
 * @param[in] size Is the size of the data array to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_WriteRegister(MCP23SXX *pComp, eMCP23SXX_Registers reg, uint8_t* data, size_t size);

//-----------------------------------------------------------------------------


/*! @brief Get all PORTs pins interrupt flags of the MCP23SXX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *intFlags Return of the interrupt flags of all ports
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_GetPinsInterruptFlags(MCP23SXX *pComp, uint16_t *intFlags);

/*! @brief Get all PORTs pins interrupt capture of the MCP23SXX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *portCapture Interrupt capture of all ports at the time the interrupt occurred
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_GetPinsInterruptCapture(MCP23SXX *pComp, uint16_t *portCapture);

//-----------------------------------------------------------------------------


/*! @brief Set pins direction on a PORT of the MCP23SXX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to change
 * @param[in] pinsDirection Set the GPIO pins direction, if bit is '1' then the corresponding GPIO is an input else it's an output
 * @param[in] pinsChangeMask If the bit is set to '1', then the corresponding GPIO must be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_SetPinsDirection(MCP23SXX *pComp, const uint8_t port, const uint8_t pinsDirection, const uint8_t pinsChangeMask);

/*! @brief Get pins input level on a PORT of the MCP23SXX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to use
 * @param[out] *pinsState Return the actual level of all I/O pins. If bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_GetPinsInputLevel(MCP23SXX *pComp, const uint8_t port, uint8_t *pinsState);

/*! @brief Set pins output level on a PORT of the MCP23SXX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to change
 * @param[in] pinsLevel Set the IO pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @param[in] pinsChangeMask If the bit is set to '1', then the corresponding GPIO must be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_SetPinsOutputLevel(MCP23SXX *pComp, const uint8_t port, const uint8_t pinsLevel, const uint8_t pinsChangeMask);

//-----------------------------------------------------------------------------


#ifdef USE_GENERICS_DEFINED
/*! @brief Set PORT direction of the MCP23SXX device
 *
 * @param[in] *pIntDev Is the pointed structure of the GPIO interface to be used
 * @param[in] pinsDirection Set the IO pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_SetPORTdirection_Gen(PORT_Interface *pIntDev, const uint32_t pinsDirection);

/*! @brief Get PORT pins input level of the MCP23SXX device
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to get input level of a whole PORT
 * @param[out] *pinsLevel Return the actual level of the PORT pins. If bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_GetPORTinputLevel_Gen(PORT_Interface *pIntDev, uint32_t *pinsLevel);

/*! @brief Set PORT pins output level of the MCP23SXX device
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to set output level of a whole PORT
 * @param[in] pinsLevel Set the PORT pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_SetPORToutputLevel_Gen(PORT_Interface *pIntDev, const uint32_t pinsLevel);

//-----------------------------------------------------------------------------


/*! @brief Set a pin on PORT direction of the MCP23SXX device
 *
 * This function will be called to change the direction of the GPIO
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO set direction
 * @param[in] pinState Set the GPIO state following #eGPIO_State enumerator
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_SetPinState_Gen(GPIO_Interface *pIntDev, const eGPIO_State pinState);

/*! @brief Get a pin on PORT input level of the MCP23SXX device
 *
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO get input level
 * @param[out] *pinLevel Return the actual level of the I/O pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP23SXX_GetPinInputLevel_Gen(GPIO_Interface *pIntDev, eGPIO_State *pinLevel);

#endif

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCP23SXX_H_INC */