/*!*****************************************************************************
 * @file    MCP230XX.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.1.0
 * @date    17/06/2023
 * @brief   MCP230XX driver
 * @details
 * -> I2C-Compatible (2-wire) 8-bit I/O Expander
 *    Follow datasheet MCP23008 Rev.F (March 2019)
 *                     MCP23009 Rev.C (August 2014)
 * -> I2C-Compatible (2-wire) 16-bit I/O Expander
 *    Follow datasheet MCP23016 Rev.C (January 2007)
 *                     MCP23017 Rev.C (July 2016)
 *                     MCP23018 Rev.A (September 2008)
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
 * 1.1.0    I2C interface rework
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef MCP230XX_H_INC
#define MCP230XX_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include "ErrorsDef.h"
#include "I2C_Interface.h"
#ifdef USE_GENERICS_DEFINED
#  include "GPIO_Interface.h"
#endif
//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#  define __MCP230XX_PACKED__
#  define MCP230XX_PACKITEM    __pragma(pack(push, 1))
#  define MCP230XX_UNPACKITEM  __pragma(pack(pop))
#else
#  define __MCP230XX_PACKED__  __attribute__((packed))
#  define MCP230XX_PACKITEM
#  define MCP230XX_UNPACKITEM
#endif

//-----------------------------------------------------------------------------

//! This macro is used to check the size of an object. If not, it will raise a "divide by 0" error at compile time
#define MCP230XX_CONTROL_ITEM_SIZE(item, size)  enum { item##_size_must_be_##size##_bytes = 1 / (int)(!!(sizeof(item) == size)) }

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP230XX limits definitions
//********************************************************************************************************************

// Limits definitions
#define MCP23008_I2CCLOCK_MAX   ( 1700000u ) //!< MCP23008 Max I2C clock frequency
#define MCP23009_I2CCLOCK_MAX   ( 3400000u ) //!< MCP23009 Max I2C clock frequency
#define MCP2300X_I2CCLOCK_MAX   ( 1700000u ) //!< MCP2300X Max I2C clock frequency
#define MCP23016_I2CCLOCK_MAX   (  400000u ) //!< MCP23016 Max I2C clock frequency
#define MCP23017_I2CCLOCK_MAX   ( 1700000u ) //!< MCP23017 Max I2C clock frequency
#define MCP23018_I2CCLOCK_MAX   ( 3400000u ) //!< MCP23018 Max I2C clock frequency
#define MCP2301X_I2CCLOCK_MAX   ( 1700000u ) //!< MCP2301X Max I2C clock frequency

#define MCP230XX_I2CCLOCK_SAFE  (  400000u ) //!< MCP230XX Safe I2C clock frequency
#define MCP230XX_I2CCLOCK_MAX   ( 3400000u ) //!< MCP230XX Max I2C clock frequency

// Device definitions
#define MCP230XX_CHIPADDRESS_BASE  ( 0x40 ) //!< MCP230XX chip base address
#define MCP230XX_CHIPADDRESS_MASK  ( 0xFE ) //!< MCP230XX chip base address mask

//-----------------------------------------------------------------------------

/*! @brief Generate the MCP230XX chip configurable address following the state of A0, A1, and A2
 * You shall set '1' (when corresponding pin is connected to +V) or '0' (when corresponding pin is connected to Ground) on each parameter
 */
#define MCP230XX_ADDR(A2, A1, A0)  ( (uint8_t)((((A2) & 0x01) << 3) | (((A1) & 0x01) << 2) | (((A0) & 0x01) << 1)) )

/*! @brief Generate the MCP230XX chip configurable address following the resistor divider and its power supply
 * You just have to put the values showed in the schematic and put them in this define and the formula will set the address for you
 * R1 is the resistor between Vdd and the ADDR pin and, R2 is the resistor between ADDR pin and the ground
 */
#define MCP230XX_ADDR_RES(R1, R2)  ( (uint8_t)((((8u * (uint32_t)(R2)) / ((uint32_t)(R1) + (uint32_t)(R2))) << 1) & MCP230XX_CHIPADDRESS_MASK) )

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP230XX Register list
//********************************************************************************************************************

//! MCP230XX registers list
typedef enum
{
  // MCP23008, and MCP23009 registers
  RegMCP230XX_IODIR     = 0x00, //!< I/O direction register
  RegMCP230XX_IPOL      = 0x01, //!< Input polarity register
  RegMCP230XX_GPINTEN   = 0x02, //!< Interrupt-On-Change control register
  RegMCP230XX_DEFVAL    = 0x03, //!< Default compare register for Interrupt-On-Change
  RegMCP230XX_INTCON    = 0x04, //!< Interrupt control register
  RegMCP230XX_IOCON     = 0x05, //!< Configuration register
  RegMCP230XX_GPPU      = 0x06, //!< Pull-Up resistor configuration register
  RegMCP230XX_INTF      = 0x07, //!< Interrupt flag register
  RegMCP230XX_INTCAP    = 0x08, //!< Interrupt capture register
  RegMCP230XX_GPIO      = 0x09, //!< Port register
  RegMCP230XX_OLAT      = 0x0A, //!< Output latch register

  // MCP23016 registers
  RegMCP230XX_GP0       = 0x00, //!< General purpose I/O port register 0
  RegMCP230XX_GP1       = 0x01, //!< General purpose I/O port register 1
  RegMCP230XX_OLAT0     = 0x02, //!< Output latch register 0
  RegMCP230XX_OLAT1     = 0x03, //!< Output latch register 1
  RegMCP230XX_IPOL0     = 0x04, //!< Input polarity register 0
  RegMCP230XX_IPOL1     = 0x05, //!< Input polarity register 1
  RegMCP230XX_IODIR0    = 0x06, //!< I/O direction register 0
  RegMCP230XX_IODIR1    = 0x07, //!< I/O direction register 1
  RegMCP230XX_INTCAP0   = 0x08, //!< Interrupt capture register 0
  RegMCP230XX_INTCAP1   = 0x09, //!< Interrupt capture register 1
  RegMCP230XX_IOCON0    = 0x0A, //!< I/O expander control register
  RegMCP230XX_IOCON1    = 0x0B, //!< I/O expander control register (shadow of RegMCP230XX_IOCON0)

  // MCP23017, and MCP23018 registers (BANK = 1)
  RegMCP230XX1_IODIRA   = 0x00, //!< I/O direction register A
  RegMCP230XX1_IPOLA    = 0x01, //!< Input polarity register A
  RegMCP230XX1_GPINTENA = 0x02, //!< Interrupt-On-Change pins register A
  RegMCP230XX1_DEFVALA  = 0x03, //!< Default compare register for Interrupt-On-Change A
  RegMCP230XX1_INTCONA  = 0x04, //!< Interrupt-On-Change control register A
  RegMCP230XX1_IOCONA   = 0x05, //!< I/O expander control register
  RegMCP230XX1_GPPUA    = 0x06, //!< Pull-Up resistor configuration register A
  RegMCP230XX1_INTFA    = 0x07, //!< Interrupt flag register A
  RegMCP230XX1_INTCAPA  = 0x08, //!< Interrupt captured value for port register A
  RegMCP230XX1_GPIOA    = 0x09, //!< Port register A
  RegMCP230XX1_OLATA    = 0x0A, //!< Output latch register A
  RegMCP230XX1_IODIRB   = 0x10, //!< I/O direction register B
  RegMCP230XX1_IPOLB    = 0x11, //!< Input polarity register B
  RegMCP230XX1_GPINTENB = 0x12, //!< Interrupt-On-Change pins register B
  RegMCP230XX1_DEFVALB  = 0x13, //!< Default compare register for Interrupt-On-Change B
  RegMCP230XX1_INTCONB  = 0x14, //!< Interrupt-On-Change control register B
  RegMCP230XX1_IOCONB   = 0x15, //!< I/O expander control register
  RegMCP230XX1_GPPUB    = 0x16, //!< Pull-Up resistor configuration register B
  RegMCP230XX1_INTFB    = 0x17, //!< Interrupt flag register B
  RegMCP230XX1_INTCAPB  = 0x18, //!< Interrupt captured value for port register B
  RegMCP230XX1_GPIOB    = 0x19, //!< Port register B
  RegMCP230XX1_OLATB    = 0x1A, //!< Output latch register B

  // MCP23017, and MCP23018 registers (BANK = 0)
  RegMCP230XX0_IODIRA   = 0x00, //!< I/O direction register A
  RegMCP230XX0_IODIRB   = 0x01, //!< I/O direction register B
  RegMCP230XX0_IPOLA    = 0x02, //!< Input polarity register A
  RegMCP230XX0_IPOLB    = 0x03, //!< Input polarity register B
  RegMCP230XX0_GPINTENA = 0x04, //!< Interrupt-On-Change pins register A
  RegMCP230XX0_GPINTENB = 0x05, //!< Interrupt-On-Change pins register B
  RegMCP230XX0_DEFVALA  = 0x06, //!< Default compare register for Interrupt-On-Change A
  RegMCP230XX0_DEFVALB  = 0x07, //!< Default compare register for Interrupt-On-Change B
  RegMCP230XX0_INTCONA  = 0x08, //!< Interrupt-On-Change control register A
  RegMCP230XX0_INTCONB  = 0x09, //!< Interrupt-On-Change control register B
  RegMCP230XX0_IOCONA   = 0x0A, //!< I/O expander control register
  RegMCP230XX0_IOCONB   = 0x0B, //!< I/O expander control register
  RegMCP230XX0_GPPUA    = 0x0C, //!< Pull-Up resistor configuration register A
  RegMCP230XX0_GPPUB    = 0x0D, //!< Pull-Up resistor configuration register B
  RegMCP230XX0_INTFA    = 0x0E, //!< Interrupt flag register A
  RegMCP230XX0_INTFB    = 0x0F, //!< Interrupt flag register B
  RegMCP230XX0_INTCAPA  = 0x10, //!< Interrupt captured value for port register A
  RegMCP230XX0_INTCAPB  = 0x11, //!< Interrupt captured value for port register B
  RegMCP230XX0_GPIOA    = 0x12, //!< Port register A
  RegMCP230XX0_GPIOB    = 0x13, //!< Port register B
  RegMCP230XX0_OLATA    = 0x14, //!< Output latch register A
  RegMCP230XX0_OLATB    = 0x15, //!< Output latch register B
} eMCP230XX_Registers;

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP230XX Specific Controller Registers
//********************************************************************************************************************

//! MCP23008 Configuration Register
MCP230XX_PACKITEM
typedef union __MCP230XX_PACKED__ MCP23008_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t       : 1; //!< 0
    uint8_t INTPOL: 1; //!< 1   - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2   - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t       : 1; //!< 3   - Not used: Hardware Address Enable, address pins are always enabled on MCP23008
//  uint8_t HAEN  : 1; //!< 3   - Hardware Address Enable (MCP23S08 only), Address pins are always enabled on MCP23008: 1 = Enables the MCP23S08 address pins ; 0 = Disables the MCP23S08 address pins
    uint8_t DISSLW: 1; //!< 4   - Slew Rate Control Bit for SDA Output. If enabled, the SDA slew rate will be controlled when driving from a high to a low: 1 = Slew rate disabled ; 0 = Slew rate enabled
    uint8_t SEQOP : 1; //!< 5   - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t       : 2; //!< 6-7
  } Bits;
} MCP23008_IOCON_Register;
MCP230XX_UNPACKITEM;
MCP230XX_CONTROL_ITEM_SIZE(MCP23008_IOCON_Register, 1);

#define MCP23008_IOCON_INTPOL_ACTIVEHIGH  (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23008_IOCON_INTPOL_ACTIVELOW   (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23008_IOCON_ODR_OPENDRAIN      (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23008_IOCON_ODR_ACTIVEDRIVER   (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23008_IOCON_DISSLW_DISABLE     (0x1u << 4) //!< Disable Slew Rate Control Bit for SDA Output
#define MCP23008_IOCON_DISSLW_ENABLE      (0x0u << 4) //!< Enable Slew Rate Control Bit for SDA Output
#define MCP23008_IOCON_SEQOP_DISABLE      (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23008_IOCON_SEQOP_ENABLE       (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments

//-----------------------------------------------------------------------------

//! MCP23009 Configuration Register
MCP230XX_PACKITEM
typedef union __MCP230XX_PACKED__ MCP23009_IOCON_Register
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
} MCP23009_IOCON_Register;
MCP230XX_UNPACKITEM;
MCP230XX_CONTROL_ITEM_SIZE(MCP23009_IOCON_Register, 1);

#define MCP23009_IOCON_INTCC_INTCAPCLEARS  (0x1u << 0) //!< Reading INTCAP register clears the Interrupt
#define MCP23009_IOCON_INTCC_GPIOCLEARS    (0x0u << 0) //!< Reading GPIO register clears the Interrupt
#define MCP23009_IOCON_INTPOL_ACTIVEHIGH   (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23009_IOCON_INTPOL_ACTIVELOW    (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23009_IOCON_ODR_OPENDRAIN       (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23009_IOCON_ODR_ACTIVEDRIVER    (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23009_IOCON_SEQOP_DISABLE       (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23009_IOCON_SEQOP_ENABLE        (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments

//-----------------------------------------------------------------------------

//! MCP23016 Configuration Register
MCP230XX_PACKITEM
typedef union __MCP230XX_PACKED__ MCP23016_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t IARES: 1; //!< 0   - Interrupt Activity Resolution: 1 = Fast sample rate (200µs max) ; 0 = Normal sample rate (32ms max)
    uint8_t      : 7; //!< 1-7
  } Bits;
} MCP23016_IOCON_Register;
MCP230XX_UNPACKITEM;
MCP230XX_CONTROL_ITEM_SIZE(MCP23016_IOCON_Register, 1);

#define MCP23016_IOCON_IARES_FASTSAMPLERATE    (0x1u << 0) //!< Fast sample rate (200µs max)
#define MCP23016_IOCON_IARES_NORMALSAMPLERATE  (0x0u << 0) //!< Normal sample rate (32ms max)

//-----------------------------------------------------------------------------

//! MCP23017 Configuration Register
MCP230XX_PACKITEM
typedef union __MCP230XX_PACKED__ MCP23017_IOCON_Register
{
  uint8_t IOCON;
  struct
  {
    uint8_t       : 1; //!< 0
    uint8_t INTPOL: 1; //!< 1 - This bit sets the polarity of the INT output pin. This bit is functional only when the ODR bit is cleared, configuring the INT pin as active push-pull: 1 = Active-high ; 0 = Active-low
    uint8_t ODR   : 1; //!< 2 - This bit configures the INT pin as an open-drain output: 1 = Open-drain output (overrides the INTPOL bit) ; 0 = Active driver output (INTPOL bit sets the polarity)
    uint8_t       : 1; //!< 3 - Not used: Hardware Address Enable, address pins are always enabled on MCP23017
//  uint8_t HAEN  : 1; //!< 3 - Hardware Address Enable (MCP23S17 only), Address pins are always enabled on MCP23017: 1 = Enables the MCP23S17 address pins ; 0 = Disables the MCP23S17 address pins
    uint8_t DISSLW: 1; //!< 4 - Slew Rate Control Bit for SDA Output. If enabled, the SDA slew rate will be controlled when driving from a high to a low: 1 = Slew rate disabled ; 0 = Slew rate enabled
    uint8_t SEQOP : 1; //!< 5 - Sequential Operation Mode: 1 = Sequential operation disabled, Address Pointer does not increment ; 0 = Sequential operation enabled, Address Pointer increments
    uint8_t MIRROR: 1; //!< 6 - INT Pins Mirror bit: 1 = The INT pins are internally connected ; 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
    uint8_t BANK  : 1; //!< 7 - Controls how the registers are addressed: 1 = The registers associated with each port are separated into different banks ; 0 = The registers are in the same bank (addresses are sequential)
  } Bits;
} MCP23017_IOCON_Register;
MCP230XX_UNPACKITEM;
MCP230XX_CONTROL_ITEM_SIZE(MCP23017_IOCON_Register, 1);

#define MCP23017_IOCON_INTPOL_ACTIVEHIGH   (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23017_IOCON_INTPOL_ACTIVELOW    (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23017_IOCON_ODR_OPENDRAIN       (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23017_IOCON_ODR_ACTIVEDRIVER    (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23017_IOCON_DISSLW_DISABLE      (0x1u << 4) //!< Disable Slew Rate Control Bit for SDA Output
#define MCP23017_IOCON_DISSLW_ENABLE       (0x0u << 4) //!< Enable Slew Rate Control Bit for SDA Output
#define MCP23017_IOCON_SEQOP_DISABLE       (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23017_IOCON_SEQOP_ENABLE        (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments
#define MCP23017_IOCON_MIRROR_DISABLE      (0x1u << 6) //!< The INT pins are internally connected
#define MCP23017_IOCON_MIRROR_ENABLE       (0x0u << 6) //!< The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
#define MCP23017_IOCON_BANK_DISABLE        (0x1u << 7) //!< The registers associated with each port are separated into different banks
#define MCP23017_IOCON_BANK_ENABLE         (0x0u << 7) //!< The registers are in the same bank (addresses are sequential)

//-----------------------------------------------------------------------------

//! MCP23018 Configuration Register
MCP230XX_PACKITEM
typedef union __MCP230XX_PACKED__ MCP23018_IOCON_Register
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
} MCP23018_IOCON_Register;
MCP230XX_UNPACKITEM;
MCP230XX_CONTROL_ITEM_SIZE(MCP23018_IOCON_Register, 1);

#define MCP23018_IOCON_INTCC_INTCAPCLEARS  (0x1u << 0) //!< Reading INTCAP register clears the Interrupt
#define MCP23018_IOCON_INTCC_GPIOCLEARS    (0x0u << 0) //!< Reading GPIO register clears the Interrupt
#define MCP23018_IOCON_INTPOL_ACTIVEHIGH   (0x1u << 1) //!< Sets the polarity of the INT output pin Active-high
#define MCP23018_IOCON_INTPOL_ACTIVELOW    (0x0u << 1) //!< Sets the polarity of the INT output pin Active-low
#define MCP23018_IOCON_ODR_OPENDRAIN       (0x1u << 2) //!< Configures the INT pin as an open-drain output
#define MCP23018_IOCON_ODR_ACTIVEDRIVER    (0x0u << 2) //!< Configures the INT pin as an active driver output
#define MCP23018_IOCON_SEQOP_DISABLE       (0x1u << 5) //!< Sequential operation disabled, Address Pointer does not increment
#define MCP23018_IOCON_SEQOP_ENABLE        (0x0u << 5) //!< Sequential operation enabled, Address Pointer increments
#define MCP23018_IOCON_MIRROR_DISABLE      (0x1u << 6) //!< The INT pins are internally connected
#define MCP23018_IOCON_MIRROR_ENABLE       (0x0u << 6) //!< The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
#define MCP23018_IOCON_BANK_DISABLE        (0x1u << 7) //!< The registers associated with each port are separated into different banks
#define MCP23018_IOCON_BANK_ENABLE         (0x0u << 7) //!< The registers are in the same bank (addresses are sequential)

//-----------------------------------------------------------------------------

//! List of supported devices
typedef enum
{
  MCP23008                           , //!< MCP23008 supported
  MCP23009                           , //!< MCP23009 supported
  MCP2300X                           , //!< Autodetected a MCP23008 or aMCP23009
  MCP230XX_DEV_1PORT_COUNT           , // 1 port device count
  MCP23016 = MCP230XX_DEV_1PORT_COUNT, //!< MCP23016 supported
  MCP23017                           , //!< MCP23017 supported
  MCP23018                           , //!< MCP23018 supported
  MCP2301X                           , //!< Autodetected a MCP23017 or aMCP23018
  eMCP230XX_DEVICE_COUNT             , // Device count of this enum, keep last of supported devices
  MCP230XX_AUTODETECT = 0xFF         , //!< Autodetect the device. Can't differentiate the MCP23008 over the MCP23009 and the MCP23017 over the MCP23018
} eMCP230XX_Devices;

static const char* const MCP230XX_DevicesNames[eMCP230XX_DEVICE_COUNT] =
{
  "MCP23008",
  "MCP23009",
  "MCP2300X",
  "MCP23016",
  "MCP23017",
  "MCP23018",
  "MCP2301X",
};

//-----------------------------------------------------------------------------

//! Structure of device registers link
typedef struct MCP230XX_RegLinks
{
  uint8_t DevPortCount;        //!< Device 8-pins port count
  // GPIO and Interrupt registers
  eMCP230XX_Registers IODIR;   //!< Link to IODIR register
  eMCP230XX_Registers IPOL;    //!< Link to IPOL register
  eMCP230XX_Registers GPINTEN; //!< Link to GPINTEN register
  eMCP230XX_Registers DEFVAL;  //!< Link to DEFVAL register
  eMCP230XX_Registers INTCON;  //!< Link to INTCON register
  eMCP230XX_Registers GPPU;    //!< Link to GPPU register
  eMCP230XX_Registers INTF;    //!< Link to INTF register
  eMCP230XX_Registers INTCAP;  //!< Link to INTCAP register
  eMCP230XX_Registers GPIO;    //!< Link to GPIO register
  eMCP230XX_Registers OLAT;    //!< Link to OLAT register
  // Configuration register
  eMCP230XX_Registers IOCON;   //!< Link to IOCON register
} MCP230XX_RegLinks;

//-----------------------------------------------------------------------------

#define MCP230XX_PORT_GP   0 //!< PORT GP index (for MCP23008 and MCP23009)
#define MCP230XX_PORT_GP0  0 //!< PORT GP0 index (for MCP23016)
#define MCP230XX_PORT_GP1  1 //!< PORT GP1 index (for MCP23016)
#define MCP230XX_PORT_GPA  0 //!< PORT GPA index (for MCP23017 and MCP23018)
#define MCP230XX_PORT_GPB  1 //!< PORT GPB index (for MCP23017 and MCP23018)

//-----------------------------------------------------------------------------



//********************************************************************************************************************
// MCP230XX Driver API
//********************************************************************************************************************

typedef struct MCP230XX MCP230XX; //! Typedef of MCP230XX device object structure

//-----------------------------------------------------------------------------

//! I/O expander configuration flags (can be OR'ed)
typedef enum
{
  MCP230XX_DEFAULT_COMPONENT_BEHAVIOR           = 0x00, //!< [All devices] This mode alone says to the driver to not touch the default configuration of the component
  MCP230XX_READING_INTCAP_CLEARS_INTERRUPT      = 0x01, //!< [MCP23009 and MCP23018] Reading INTCAP register clears the interrupt
  MCP230XX_READING_GPIO_CLEARS_INTERRUPT        = 0x00, //!< [MCP23009 and MCP23018] Reading GPIO register clears the interrupt
  MCP230XX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH  = 0x02, //!< [All except MCP23016] The polarity of the INT output pin is Active-High
  MCP230XX_INT_PIN_OUTPUT_POLARITY_ACTIVE_LOW   = 0x00, //!< [All except MCP23016] The polarity of the INT output pin is Active-Low
  MCP230XX_INT_PIN_OUTPUT_OPEN_DRAIN            = 0x04, //!< [All except MCP23016] Configures the INT pin as open-drain output (overrides the INTPOL bit)
  MCP230XX_INT_PIN_OUTPUT_PUSH_PULL             = 0x00, //!< [All except MCP23016] Configures the INT pin as active driver output (INTPOL bit sets the polarity)
  MCP230XX_GPIO_SAMPLING_RATE_FAST              = 0x08, //!< [MCP23016 only] GP ports sampling at fast rate (200µs max)
  MCP230XX_GPIO_SAMPLING_RATE_NORMAL            = 0x00, //!< [MCP23016 only] GP ports sampling at normal rate (32ms max)
  MCP230XX_SLEW_RATE_CONTROL_SDA_OUTPUT_DISABLE = 0x10, //!< [MCP23008 and MCP23017] Slew Rate control bit for SDA output disabled
  MCP230XX_SLEW_RATE_CONTROL_SDA_OUTPUT_ENABLE  = 0x00, //!< [MCP23008 and MCP23017] Slew Rate control bit for SDA output enable
//MCP230XX_SEQUENTIAL_OPERATION_DISABLE         = 0x20, //!< [All except MCP23016] Sequential operation disabled, address pointer does not increment
//MCP230XX_SEQUENTIAL_OPERATION_ENABLE          = 0x00, //!< [All except MCP23016] Sequential operation enabled, address pointer increments
  MCP230XX_INT_PIN_MIRRORED                     = 0x40, //!< [MCP23017 and MCP23018] The INT pins are internally connected
  MCP230XX_INT_PIN_INDEPENDANT                  = 0x00, //!< [MCP23017 and MCP23018] The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
//MCP230XX_PORT_REGISTERS_SEPARATED             = 0x80, //!< [MCP23017 and MCP23018] The registers associated with each port are separated into different banks
//MCP230XX_PORT_REGISTERS_SAME_BANK             = 0x00, //!< [MCP23017 and MCP23018] The registers are in the same bank (addresses are sequential)
} eMCP230XX_ControlFlags;

typedef eMCP230XX_ControlFlags setMCP230XX_ControlFlags; //!< Set of I/O expander configuration flags (can be OR'ed)

//-----------------------------------------------------------------------------

//! MCP230XX device object structure
struct MCP230XX
{
  void *UserDriverData;                  //!< Optional, can be used to store driver data or NULL
  
  //--- Device configuration ---
  setMCP230XX_ControlFlags ControlFlags; //!< I/O expander configuration flags for general behavior (can be OR'ed)

  //--- PORT configuration ---
  uint8_t PORTdirection[2];              //!< GPIOs pins direction (0 = set to output ; 1 = set to input). Used to speed up direction change
  uint8_t PORToutLevel[2];               //!< GPIOs pins output level (0 = set to low level ; 1 = set to high level). Used to speed up output change

  //--- Interface driver call functions ---
#ifdef USE_DYNAMIC_INTERFACE
  I2C_Interface* I2C;                    //!< This is the I2C_Interface descriptor pointer that will be used to communicate with the device
#else
  I2C_Interface I2C;                     //!< This is the I2C_Interface descriptor that will be used to communicate with the device
#endif
  uint32_t I2CclockSpeed;                //!< Clock frequency of the I2C interface in Hertz

  //--- Device address ---
  uint8_t AddrA2A1A0;                    //!< Device configurable address A2, A1, and A0. You can use the macro MCP230XX_ADDR() to help filling this parameter. Only these 3 lower bits are used: ....210. where 2 is A2, 1 is A1, 0 is A0, and '.' are fixed by device
  eMCP230XX_Devices DeviceName;          //!< This is the device name, set to MCP230XX_AUTODETECT if you don't need specific behavior and the driver to work generically. The initialization of the driver will change this value
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define MCP230XX_UNIQUE_ID  ( (((uint32_t)'M' << 0) ^ ((uint32_t)'C' << 4) ^ ((uint32_t)'P' << 8) ^ ((uint32_t)'2' << 12) ^ ((uint32_t)'3' << 16) ^ ((uint32_t)'0' << 20) ^ ((uint32_t)'X' << 24) ^ ((uint32_t)'X' << 28)) + (sizeof(struct MCP230XX) << 19) )

//-----------------------------------------------------------------------------

//! GPIO configuration (can be OR'ed)
typedef enum
{
  MCP230XX_PIN_UNUSED                      = 0x00, //!< Unused pin will be set to output, low level, no pull-up, no interrupt
  // GPIO pin configuration
  MCP230XX_PIN_AS_INPUT                    = 0x01, //!< The pin is configured as an input
  MCP230XX_PIN_AS_OUTPUT                   = 0x00, //!< The pin is configured as an output
  MCP230XX_GPIO_STATE_INVERTED_LOGIC       = 0x02, //!< GPIO register bit will reflect the opposite logic state of the input pin
  MCP230XX_GPIO_STATE_SAME_LOGIC           = 0x00, //!< GPIO register bit will reflect the same logic state of the input pin
  MCP230XX_GPIO_OUTPUT_STATE_HIGH          = 0x04, //!< GPIO pin output level set as high
  MCP230XX_GPIO_OUTPUT_STATE_LOW           = 0x00, //!< GPIO pin output level set as low
  MCP230XX_GPIO_PULLUP_ENABLE              = 0x08, //!< GPIO internal pull-up pin enable
  MCP230XX_GPIO_PULLUP_DISABLE             = 0x00, //!< GPIO internal pull-up pin disable
  // GPIO interrupt configuration
  MCP230XX_INTERRUPT_ON_CHANGE_ENABLE      = 0x10, //!< Enable GPIO input pin for interrupt-on-change event
  MCP230XX_INTERRUPT_ON_CHANGE_DISABLE     = 0x00, //!< Disable GPIO input pin for interrupt-on-change event
  MCP230XX_VALUE_COMPARED_DEFAULT_VALUE    = 0x20, //!< Pin value is compared against the associated bit is the default value (DEFVAL register)
  MCP230XX_VALUE_COMPARED_PREVIOUS_VALUE   = 0x00, //!< Pin value is compared against the previous pin value
  MCP230XX_DEFAULT_VALUE_NO_INT_LEVEL_HIGH = 0x40, //!< Sets the compare value for pins configured to level high for interrupt-on-change. If the associated pin level is low from the register bit, an interrupt occurs
  MCP230XX_DEFAULT_VALUE_NO_INT_LEVEL_LOW  = 0x00, //!< Sets the compare value for pins configured to level low for interrupt-on-change. If the associated pin level is high from the register bit, an interrupt occurs
} eMCP230XX_PinConf;

typedef eMCP230XX_PinConf setMCP230XX_PinConf; //!< Set of GPIO configuration (can be OR'ed)

//-----------------------------------------------------------------------------

//! MCP230XX Configuration structure
typedef struct MCP230XX_Config
{
  //--- GPIOs configuration ---
  union
  {
    setMCP230XX_PinConf GPs[16];  //!< Configuration for all GPIOs at once (GP0 (low) and GP1 (high) for MCP23016, GPA (low) and GPB (high) for MCP23017 and MCP23018)
    setMCP230XX_PinConf GP[8];    //!< Configuration for GP only (for MCP23008 and MCP23009)
    struct
    {
      setMCP230XX_PinConf GPA[8]; //!< Configuration for GPA only (for MCP23017 and MCP23018)
      setMCP230XX_PinConf GPB[8]; //!< Configuration for GPB only (for MCP23017 and MCP23018)
    };
    struct
    {
      setMCP230XX_PinConf GP0[8]; //!< Configuration for GP0 only (for MCP23016)
      setMCP230XX_PinConf GP1[8]; //!< Configuration for GP1 only (for MCP23016)
    };
  };
} MCP230XX_Config;

//-----------------------------------------------------------------------------


/*! @brief MCP230XX initialization
 *
 * This function initializes the MCP230XX driver and call the initialization of the interface driver (I2C).
 * Next it checks parameters and configures the MCP230XX
 * @param[in] *pComp Is the pointed structure of the device to be initialized
 * @param[in] *pConf Is the pointed structure of the device configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_MCP230XX(MCP230XX *pComp, const MCP230XX_Config *pConf);

/*! @brief Is the MCP230XX device ready
 *
 * Poll the acknowledge from the MCP230XX
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns 'true' if ready else 'false'
 */
bool MCP230XX_IsReady(MCP230XX *pComp);

//-----------------------------------------------------------------------------


/*! @brief Read data from register of the MCP230XX device
 *
 * This function reads data from a register of a MCP230XX device
 * @param[in] *pComp Is the pointed structure of the device to read
 * @param[in] reg Is the register to read
 * @param[in] *data Is where the data will be stored
 * @param[in] size Is the size of the data array to read
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_ReadRegister(MCP230XX *pComp, eMCP230XX_Registers reg, uint8_t* data, size_t size);

/*! @brief Write data to register of the MCP230XX device
 *
 * This function writes data to a register of a MCP230XX device
 * @param[in] *pComp Is the pointed structure of the device to modify
 * @param[in] reg Is the register where data will be written
 * @param[in] *data Is the data array to store
 * @param[in] size Is the size of the data array to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_WriteRegister(MCP230XX *pComp, eMCP230XX_Registers reg, uint8_t* data, size_t size);

//-----------------------------------------------------------------------------


/*! @brief Get all PORTs pins interrupt flags of the MCP230XX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *intFlags Return of the interrupt flags of all ports
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_GetPinsInterruptFlags(MCP230XX *pComp, uint16_t *intFlags);

/*! @brief Get all PORTs pins interrupt capture of the MCP230XX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *portCapture Interrupt capture of all ports at the time the interrupt occurred
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_GetPinsInterruptCapture(MCP230XX *pComp, uint16_t *portCapture);

//-----------------------------------------------------------------------------


/*! @brief Set pins direction on a PORT of the MCP230XX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to change
 * @param[in] pinsDirection Set the GPIO pins direction, if bit is '1' then the corresponding GPIO is an input else it's an output
 * @param[in] pinsChangeMask If the bit is set to '1', then the corresponding GPIO must be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_SetPinsDirection(MCP230XX *pComp, const uint8_t port, const uint8_t pinsDirection, const uint8_t pinsChangeMask);

/*! @brief Get pins input level on a PORT of the MCP230XX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to use
 * @param[out] *pinsState Return the actual level of all I/O pins. If bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_GetPinsInputLevel(MCP230XX *pComp, const uint8_t port, uint8_t *pinsState);

/*! @brief Set pins output level on a PORT of the MCP230XX device
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] port Is the port to change
 * @param[in] pinsLevel Set the IO pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @param[in] pinsChangeMask If the bit is set to '1', then the corresponding GPIO must be modified
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_SetPinsOutputLevel(MCP230XX *pComp, const uint8_t port, const uint8_t pinsLevel, const uint8_t pinsChangeMask);

//-----------------------------------------------------------------------------


#ifdef USE_GENERICS_DEFINED
/*! @brief Set PORT direction of the MCP230XX device
 *
 * @param[in] *pIntDev Is the pointed structure of the GPIO interface to be used
 * @param[in] pinsDirection Set the IO pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_SetPORTdirection_Gen(PORT_Interface *pIntDev, const uint32_t pinsDirection);

/*! @brief Get PORT pins input level of the MCP230XX device
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to get input level of a whole PORT
 * @param[out] *pinsLevel Return the actual level of the PORT pins. If bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_GetPORTinputLevel_Gen(PORT_Interface *pIntDev, uint32_t *pinsLevel);

/*! @brief Set PORT pins output level of the MCP230XX device
 *
 * @param[in] *pIntDev Is the PORT interface container structure used to set output level of a whole PORT
 * @param[in] pinsLevel Set the PORT pins output level, if bit is '1' then the corresponding GPIO is level high else it's level low
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_SetPORToutputLevel_Gen(PORT_Interface *pIntDev, const uint32_t pinsLevel);

//-----------------------------------------------------------------------------


/*! @brief Set a pin on PORT direction of the MCP230XX device
 *
 * This function will be called to change the direction of the GPIO
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO set direction
 * @param[in] pinState Set the GPIO state following #eGPIO_State enumerator
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_SetPinState_Gen(GPIO_Interface *pIntDev, const eGPIO_State pinState);

/*! @brief Get a pin on PORT input level of the MCP230XX device
 *
 * @param[in] *pIntDev Is the GPIO interface container structure used for the GPIO get input level
 * @param[out] *pinLevel Return the actual level of the I/O pin
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCP230XX_GetPinInputLevel_Gen(GPIO_Interface *pIntDev, eGPIO_State *pinLevel);

#endif

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCP230XX_H_INC */