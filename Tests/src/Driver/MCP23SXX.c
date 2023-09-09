/*!*****************************************************************************
 * @file    MCP23SXX.c
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

//-----------------------------------------------------------------------------
#include "MCP23SXX.h"
//-----------------------------------------------------------------------------
#ifdef _cplusplus
# include <cstdint>
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_DYNAMIC_INTERFACE
#  define GET_SPI_INTERFACE  pComp->SPI
#else
#  define GET_SPI_INTERFACE  &pComp->SPI
#endif

//-----------------------------------------------------------------------------





//=============================================================================
// Prototypes for private functions
//=============================================================================
/*! @brief Check SPI clock speed
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return 'true' if the clock speed is above limit, else 'false'
 */
static bool __MCP23SXX_CheckSCKspeed(MCP23SXX *pComp);
//-----------------------------------------------------------------------------





//! MCP23SXX devices Register list following device structure
static const MCP23SXX_RegLinks MCP23SXX_DevicesRegLinks[eMCP23SXX_DEVICE_COUNT] =
{
  // 1-port devices
  { .DevPortCount = 1, .IODIR = RegMCP23SXX_IODIR  , .IPOL = RegMCP23SXX_IPOL  , .GPINTEN = RegMCP23SXX_GPINTEN  , .DEFVAL = RegMCP23SXX_DEFVAL  , .INTCON = RegMCP23SXX_INTCON  , .GPPU = RegMCP23SXX_GPPU  , .INTF = RegMCP23SXX_INTF  , .INTCAP = RegMCP23SXX_INTCAP  , .GPIO = RegMCP23SXX_GPIO  , .OLAT = RegMCP23SXX_OLAT  , .IOCON = RegMCP23SXX_IOCON  , }, // MCP23S08
  { .DevPortCount = 1, .IODIR = RegMCP23SXX_IODIR  , .IPOL = RegMCP23SXX_IPOL  , .GPINTEN = RegMCP23SXX_GPINTEN  , .DEFVAL = RegMCP23SXX_DEFVAL  , .INTCON = RegMCP23SXX_INTCON  , .GPPU = RegMCP23SXX_GPPU  , .INTF = RegMCP23SXX_INTF  , .INTCAP = RegMCP23SXX_INTCAP  , .GPIO = RegMCP23SXX_GPIO  , .OLAT = RegMCP23SXX_OLAT  , .IOCON = RegMCP23SXX_IOCON  , }, // MCP23S09
  { .DevPortCount = 1, .IODIR = RegMCP23SXX0_IODIRA, .IPOL = RegMCP23SXX0_IPOLA, .GPINTEN = RegMCP23SXX0_GPINTENA, .DEFVAL = RegMCP23SXX0_DEFVALA, .INTCON = RegMCP23SXX0_INTCONA, .GPPU = RegMCP23SXX0_GPPUA, .INTF = RegMCP23SXX0_INTFA, .INTCAP = RegMCP23SXX0_INTCAPA, .GPIO = RegMCP23SXX0_GPIOA, .OLAT = RegMCP23SXX0_OLATA, .IOCON = RegMCP23SXX0_IOCONA, }, // MCP23S1X
  // 2-port devices
  { .DevPortCount = 2, .IODIR = RegMCP23SXX0_IODIRA, .IPOL = RegMCP23SXX0_IPOLA, .GPINTEN = RegMCP23SXX0_GPINTENA, .DEFVAL = RegMCP23SXX0_DEFVALA, .INTCON = RegMCP23SXX0_INTCONA, .GPPU = RegMCP23SXX0_GPPUA, .INTF = RegMCP23SXX0_INTFA, .INTCAP = RegMCP23SXX0_INTCAPA, .GPIO = RegMCP23SXX0_GPIOA, .OLAT = RegMCP23SXX0_OLATA, .IOCON = RegMCP23SXX0_IOCONA, }, // MCP23S17
  { .DevPortCount = 2, .IODIR = RegMCP23SXX0_IODIRA, .IPOL = RegMCP23SXX0_IPOLA, .GPINTEN = RegMCP23SXX0_GPINTENA, .DEFVAL = RegMCP23SXX0_DEFVALA, .INTCON = RegMCP23SXX0_INTCONA, .GPPU = RegMCP23SXX0_GPPUA, .INTF = RegMCP23SXX0_INTFA, .INTCAP = RegMCP23SXX0_INTCAPA, .GPIO = RegMCP23SXX0_GPIOA, .OLAT = RegMCP23SXX0_OLATA, .IOCON = RegMCP23SXX0_IOCONA, }, // MCP23S18
  { .DevPortCount = 2, .IODIR = RegMCP23SXX_IODIR  , .IPOL = RegMCP23SXX_IPOL  , .GPINTEN = RegMCP23SXX_GPINTEN  , .DEFVAL = RegMCP23SXX_DEFVAL  , .INTCON = RegMCP23SXX_INTCON  , .GPPU = RegMCP23SXX_GPPU  , .INTF = RegMCP23SXX_INTF  , .INTCAP = RegMCP23SXX_INTCAP  , .GPIO = RegMCP23SXX_GPIO  , .OLAT = RegMCP23SXX_OLAT  , .IOCON = RegMCP23SXX_IOCON  , }, // MCP23S0X
};

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// MCP23SXX initialization
//=============================================================================
eERRORRESULT Init_MCP23SXX(MCP23SXX *pComp, const MCP23SXX_Config *pConf)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (pConf == NULL)) return ERR__PARAMETER_ERROR;
#endif
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Init == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Data[2];
  bool InSafeSCKspeed = false;

  //--- Check and adjust SPI SCK speed ---
  uint32_t SCKfreq = pComp->SPIclockSpeed;
  if ((pComp->DeviceName == MCP23SXX_AUTODETECT) && (pComp->SPIclockSpeed > MCP23SXX_SPICLOCK_SAFE))
  { SCKfreq = MCP23SXX_SPICLOCK_SAFE; InSafeSCKspeed = true; }                // In case of Autodetect, we do not know yet the maximum speed of the device. We need to set a safe SCK clock speed

  //--- Initialize SPI interface ---
  Error = pSPI->fnSPI_Init(pSPI, pComp->SPIchipSelect, STD_SPI_MODE0, SCKfreq);
  if (Error != ERR_NONE) return Error;                                        // If there is an error while calling fnSPI_Init() then return the error

  //--- Check device ---
  if ((pComp->DeviceName == MCP23SXX_AUTODETECT) || (pComp->DeviceName >= eMCP23SXX_DEVICE_COUNT))
  {
    //--- Autodetect device ---
    // There is no register that can says which device is connected
    // After a POR/RESET, the device is at known state, only the registers 0x00 and 0x01 can says with a pretty good accuracy what kind of device it is
    Error = MCP23SXX_ReadRegister(pComp, 0x00, &Data[0], sizeof(Data));       // Read registers 0x00 and 0x01
    if (Error != ERR_NONE) return Error;                                      // If there is an error while calling MCP23SXX_ReadRegister() then return the error
    if ((Data[0] == 0xFF) && (Data[1] == 0x00)) pComp->DeviceName = MCP23S0X; // Can be a MCP23S08 or a MCP23S09
    if ((Data[0] == 0xFF) && (Data[1] == 0xFF)) pComp->DeviceName = MCP23S1X; // Can be a MCP23S17 or a MCP23S18
    if (pComp->DeviceName == MCP23SXX_AUTODETECT) return ERR__UNKNOWN_DEVICE; // Here the device have been used and is not at reset state, so there is no possibilities to be sure of which device it is
  }

  //--- Finally set the desired SPI SCK speed ---
  if (__MCP23SXX_CheckSCKspeed(pComp) == false) return ERR__SPI_FREQUENCY_ERROR;
  if (InSafeSCKspeed)
  {
    Error = pSPI->fnSPI_Init(pSPI, pComp->SPIchipSelect, STD_SPI_MODE0, pComp->SPIclockSpeed); // Reinit the SPI and set the desired speed
    if (Error != ERR_NONE) return Error;                                                       // If there is an error while calling fnSPI_Init() then return the error
  }

  //--- Configure pins ---
  const size_t GPcount = MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount;
  uint16_t IODIR = 0x0000, IPOL = 0x0000, OLAT = 0x0000, GPPU = 0x0000;                  // GPIOs registers
  uint16_t GPINTEN = 0x0000, INTCON = 0x0000, DEFVAL = 0x0000;                           // GPIOs interrupt registers
  for (uint8_t Pin = 0; Pin < 16; Pin++)                                                 // Prepare all the registers following parameters of each pins
  {
    if ((pConf->GPs[Pin] & MCP23SXX_PIN_AS_INPUT                   ) > 0) IODIR   |= (1 << Pin); // Set the pin direction for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_GPIO_STATE_INVERTED_LOGIC      ) > 0) IPOL    |= (1 << Pin); // Set the input polarity for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_GPIO_OUTPUT_STATE_HIGH         ) > 0) OLAT    |= (1 << Pin); // Set the output level for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_GPIO_PULLUP_ENABLE             ) > 0) GPPU    |= (1 << Pin); // Set the internal pull-up for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_INTERRUPT_ON_CHANGE_ENABLE     ) > 0) GPINTEN |= (1 << Pin); // Set the interrupt on change for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_VALUE_COMPARED_DEFAULT_VALUE   ) > 0) INTCON  |= (1 << Pin); // Set the interrupt control for the current pin
    if ((pConf->GPs[Pin] & MCP23SXX_DEFAULT_VALUE_NO_INT_LEVEL_HIGH) > 0) DEFVAL  |= (1 << Pin); // Set the default value of the pin
  }
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].OLAT , (uint8_t*)&OLAT, GPcount);      // Write OLAT register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].IODIR, (uint8_t*)&IODIR, GPcount);     // Write IODIR register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].IPOL , (uint8_t*)&IPOL , GPcount);     // Write IPOL register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].GPINTEN, (uint8_t*)&GPINTEN, GPcount); // Write GPINTEN register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].DEFVAL , (uint8_t*)&DEFVAL , GPcount); // Write DEFVAL register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].INTCON , (uint8_t*)&INTCON , GPcount); // Write INTCONDIR register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error
  Error = MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].GPPU   , (uint8_t*)&GPPU   , GPcount); // Write GPPU register
  if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP23SXX_WriteRegister() then return the error

  //--- Configure device ---
  uint16_t IOCON = 0x00;
  if ((pComp->ControlFlags & MCP23SXX_READING_INTCAP_CLEARS_INTERRUPT     ) > 0) IOCON |= MCP23S18_IOCON_INTCC_INTCAPCLEARS;
  if ((pComp->ControlFlags & MCP23SXX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH ) > 0) IOCON |= MCP23S18_IOCON_INTPOL_ACTIVEHIGH;
  if ((pComp->ControlFlags & MCP23SXX_INT_PIN_OUTPUT_OPEN_DRAIN           ) > 0) IOCON |= MCP23S18_IOCON_ODR_OPENDRAIN;
  if ((pComp->ControlFlags & MCP23SXX_SLEW_RATE_CONTROL_SDA_OUTPUT_DISABLE) > 0) IOCON |= MCP23S17_IOCON_DISSLW_DISABLE;
  if ((pComp->ControlFlags & MCP23SXX_INT_PIN_MIRRORED                    ) > 0) IOCON |= MCP23S18_IOCON_MIRROR_DISABLE;
  return MCP23SXX_WriteRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].IOCON, (uint8_t*)&IOCON, GPcount);      // Write IOCON register
}

//-----------------------------------------------------------------------------





//=============================================================================
// [STATIC] Check SPI clock speed
//=============================================================================
bool __MCP23SXX_CheckSCKspeed(MCP23SXX *pComp)
{
  if ((pComp->DeviceName == MCP23S08) && (pComp->SPIclockSpeed > MCP23S08_SPICLOCK_MAX)) return false; // Wrong MCP23S08 SCK clock speed
  if ((pComp->DeviceName == MCP23S09) && (pComp->SPIclockSpeed > MCP23S09_SPICLOCK_MAX)) return false; // Wrong MCP23S09 SCK clock speed
  if ((pComp->DeviceName == MCP23S0X) && (pComp->SPIclockSpeed > MCP23S0X_SPICLOCK_MAX)) return false; // Wrong MCP23S0X SCK clock speed
  if ((pComp->DeviceName == MCP23S17) && (pComp->SPIclockSpeed > MCP23S17_SPICLOCK_MAX)) return false; // Wrong MCP23S17 SCK clock speed
  if ((pComp->DeviceName == MCP23S18) && (pComp->SPIclockSpeed > MCP23S18_SPICLOCK_MAX)) return false; // Wrong MCP23S18 SCK clock speed
  if ((pComp->DeviceName == MCP23S1X) && (pComp->SPIclockSpeed > MCP23S1X_SPICLOCK_MAX)) return false; // Wrong MCP23S1X SCK clock speed
  return true;
}

//-----------------------------------------------------------------------------




//**********************************************************************************************************************************************************
//=============================================================================
// Read data from register of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_ReadRegister(MCP23SXX *pComp, eMCP23SXX_Registers reg, uint8_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t ChipAddr[2] = { ((MCP23SXX_CHIPADDRESS_BASE | pComp->AddrA2A1A0) & MCP23SXX_CHIPADDRESS_MASK), (uint8_t)reg };

  //--- Send the address ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipAddr[0], sizeof(ChipAddr), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the register's address
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Get the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_RX_DATA_DESC(data, size, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by reading the data, get the data and stop transfer at last byte
}


//=============================================================================
// Write data to register of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_WriteRegister(MCP23SXX *pComp, eMCP23SXX_Registers reg, uint8_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if ((uint8_t)reg == 0xFF) return ERR__NOT_SUPPORTED;
  eERRORRESULT Error;
  uint8_t ChipAddr[2] = { ((MCP23SXX_CHIPADDRESS_BASE | pComp->AddrA2A1A0) & MCP23SXX_CHIPADDRESS_MASK), (uint8_t)reg };

  //--- Send the address ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipAddr[0], sizeof(ChipAddr), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the register's address
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Send the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_TX_DATA_DESC(data, size, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by transferring the data, and stop transfer at last byte
}

//-----------------------------------------------------------------------------





//=============================================================================
// Get all PORTs pins interrupt flags of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_GetPinsInterruptFlags(MCP23SXX *pComp, uint16_t *intFlags)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Values[sizeof(uint16_t)] = { 0x00, 0x00 };
  eERRORRESULT Error = MCP23SXX_ReadRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].INTF, &Values[0], MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount);
  *intFlags = (uint16_t)Values[1] << 8 | (uint16_t)Values[0];
  return Error;
}


//=============================================================================
// Get all PORTs pins interrupt capture of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_GetPinsInterruptCapture(MCP23SXX *pComp, uint16_t *portCapture)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Values[sizeof(uint16_t)] = { 0x00, 0x00 };
  eERRORRESULT Error = MCP23SXX_ReadRegister(pComp, MCP23SXX_DevicesRegLinks[pComp->DeviceName].INTCAP, &Values[0], MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount);
  *portCapture = (uint16_t)Values[1] << 8 | (uint16_t)Values[0];
  return Error;
}

//-----------------------------------------------------------------------------





//=============================================================================
// Set pins direction on a PORT of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_SetPinsDirection(MCP23SXX *pComp, const uint8_t port, const uint8_t pinsDirection, const uint8_t pinsChangeMask)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pComp->PORToutDir[port] &= ~pinsChangeMask;                  // Force change bits to 0
  pComp->PORToutDir[port] |= (pinsDirection & pinsChangeMask); // Apply new output level only on changed pins
  const eMCP23SXX_Registers IODIRreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pComp->DeviceName].IODIR + port);
  return MCP23SXX_WriteRegister(pComp, IODIRreg, &pComp->PORToutDir[port], 1);
}


//=============================================================================
// Get PORT0 pins input level of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_GetPinsInputLevel(MCP23SXX *pComp, const uint8_t port, uint8_t *pinsState)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  const eMCP23SXX_Registers GPIOreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pComp->DeviceName].GPIO + port);
  return MCP23SXX_ReadRegister(pComp, GPIOreg, pinsState, 1);
}


//=============================================================================
// Set PORT0 pins output level of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_SetPinsOutputLevel(MCP23SXX *pComp, const uint8_t port, const uint8_t pinsLevel, const uint8_t pinsChangeMask)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP23SXX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pComp->PORToutLevel[port] &= ~pinsChangeMask;                 // Force change bits to 0
  pComp->PORToutLevel[port] |= (pinsLevel & pinsChangeMask);    // Apply new output level only on changed pins
  const eMCP23SXX_Registers OLATreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pComp->DeviceName].OLAT + port);
  return MCP23SXX_WriteRegister(pComp, OLATreg, &pComp->PORToutLevel[port], 1);
}

//-----------------------------------------------------------------------------





#ifdef USE_GENERICS_DEFINED

//=============================================================================
// Set PORT direction of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_SetPORTdirection_Gen(PORT_Interface *pIntDev, const uint32_t pinsDirection)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP23SXX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP23SXX* pDevice = (MCP23SXX*)(pIntDev->InterfaceDevice);         // Get the MCP23SXX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP23SXX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pDevice->PORToutDir[pIntDev->PORTindex] = (uint8_t)pinsDirection; // Apply new output direction on PORT
  const eMCP23SXX_Registers IODIRreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pDevice->DeviceName].IODIR + pIntDev->PORTindex);
  return MCP23SXX_WriteRegister(pDevice, IODIRreg, &pDevice->PORToutDir[pIntDev->PORTindex], 1);
}


//=============================================================================
// Get PORT pins input level of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_GetPORTinputLevel_Gen(PORT_Interface *pIntDev, uint32_t *pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP23SXX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP23SXX* pDevice = (MCP23SXX*)(pIntDev->InterfaceDevice); // Get the MCP23SXX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP23SXX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  const eMCP23SXX_Registers GPIOreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pDevice->DeviceName].GPIO + pIntDev->PORTindex);
  uint8_t Value = 0;
  eERRORRESULT Error = MCP23SXX_ReadRegister(pDevice, GPIOreg, &Value, 1);
  *pinsLevel = Value;
  return Error;
}


//=============================================================================
// Set PORT pins output level of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_SetPORToutputLevel_Gen(PORT_Interface *pIntDev, const uint32_t pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP23SXX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP23SXX* pDevice = (MCP23SXX*)(pIntDev->InterfaceDevice);     // Get the MCP23SXX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP23SXX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pDevice->PORToutDir[pIntDev->PORTindex] = (uint8_t)pinsLevel; // Apply new output level on PORT
  const eMCP23SXX_Registers OLATreg = (eMCP23SXX_Registers)((uint8_t)MCP23SXX_DevicesRegLinks[pDevice->DeviceName].OLAT + pIntDev->PORTindex);
  return MCP23SXX_WriteRegister(pDevice, OLATreg, &pDevice->PORToutDir[pIntDev->PORTindex], 1);
}

//-----------------------------------------------------------------------------



//=============================================================================
// Set a pin on PORT direction of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_SetPinState_Gen(GPIO_Interface *pIntDev, const eGPIO_State pinState)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP23SXX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP23SXX* pDevice = (MCP23SXX*)(pIntDev->InterfaceDevice); // Get the MCP23SXX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP23SXX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  switch (pinState)
  {
    default: break;
    case GPIO_STATE_OUTPUT: return MCP23SXX_SetPinsDirection(pDevice, pIntDev->PORTindex, 0x00, pIntDev->PinBitMask);
    case GPIO_STATE_INPUT : return MCP23SXX_SetPinsDirection(pDevice, pIntDev->PORTindex, pIntDev->PinBitMask, pIntDev->PinBitMask);
    case GPIO_STATE_RESET : return MCP23SXX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, 0x00, pIntDev->PinBitMask);
    case GPIO_STATE_SET   : return MCP23SXX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, pIntDev->PinBitMask, pIntDev->PinBitMask);
    case GPIO_STATE_TOGGLE: return MCP23SXX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, pDevice->PORToutLevel[pIntDev->PORTindex] ^ (uint8_t)pIntDev->PinBitMask, pIntDev->PinBitMask);
  }
  return ERR__PARAMETER_ERROR;
}


//=============================================================================
// Get a pin on PORT input level of the MCP23SXX device
//=============================================================================
eERRORRESULT MCP23SXX_GetPinInputLevel_Gen(GPIO_Interface *pIntDev, eGPIO_State *pinLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP23SXX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP23SXX* pDevice = (MCP23SXX*)(pIntDev->InterfaceDevice); // Get the MCP23SXX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP23SXX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  uint8_t PinState = 0;
  eERRORRESULT Error = MCP23SXX_GetPinsInputLevel(pDevice, pIntDev->PORTindex, &PinState);
  *pinLevel = (PinState > 0 ? GPIO_STATE_SET : GPIO_STATE_RESET);
  return Error;
}

#endif

//-----------------------------------------------------------------------------
#ifdef _cplusplus
}
#endif
//-----------------------------------------------------------------------------