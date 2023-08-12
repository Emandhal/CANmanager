/*!*****************************************************************************
 * @file    MCP230XX.c
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

//-----------------------------------------------------------------------------
#include "MCP230XX.h"
//-----------------------------------------------------------------------------
#ifdef _cplusplus
# include <cstdint>
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_DYNAMIC_INTERFACE
#  define GET_I2C_INTERFACE  pComp->I2C
#else
#  define GET_I2C_INTERFACE  &pComp->I2C
#endif

//-----------------------------------------------------------------------------





//=============================================================================
// Prototypes for private functions
//=============================================================================
/*! @brief Check I2C clock speed
 *
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return 'true' if the clock speed is above limit, else 'false'
 */
static bool __MCP230XX_CheckSCLspeed(MCP230XX *pComp);
//-----------------------------------------------------------------------------





//! MCP230XX devices Register list following device structure
static const MCP230XX_RegLinks MCP230XX_DevicesRegLinks[eMCP230XX_DEVICE_COUNT] =
{
  // 1-port devices
  { .DevPortCount = 1, .IODIR = RegMCP230XX_IODIR  , .IPOL = RegMCP230XX_IPOL  , .GPINTEN = RegMCP230XX_GPINTEN  , .DEFVAL = RegMCP230XX_DEFVAL  , .INTCON = RegMCP230XX_INTCON  , .GPPU = RegMCP230XX_GPPU  , .INTF = RegMCP230XX_INTF  , .INTCAP = RegMCP230XX_INTCAP  , .GPIO = RegMCP230XX_GPIO  , .OLAT = RegMCP230XX_OLAT  , .IOCON = RegMCP230XX_IOCON  , }, // MCP23008
  { .DevPortCount = 1, .IODIR = RegMCP230XX_IODIR  , .IPOL = RegMCP230XX_IPOL  , .GPINTEN = RegMCP230XX_GPINTEN  , .DEFVAL = RegMCP230XX_DEFVAL  , .INTCON = RegMCP230XX_INTCON  , .GPPU = RegMCP230XX_GPPU  , .INTF = RegMCP230XX_INTF  , .INTCAP = RegMCP230XX_INTCAP  , .GPIO = RegMCP230XX_GPIO  , .OLAT = RegMCP230XX_OLAT  , .IOCON = RegMCP230XX_IOCON  , }, // MCP23009
  { .DevPortCount = 1, .IODIR = RegMCP230XX0_IODIRA, .IPOL = RegMCP230XX0_IPOLA, .GPINTEN = RegMCP230XX0_GPINTENA, .DEFVAL = RegMCP230XX0_DEFVALA, .INTCON = RegMCP230XX0_INTCONA, .GPPU = RegMCP230XX0_GPPUA, .INTF = RegMCP230XX0_INTFA, .INTCAP = RegMCP230XX0_INTCAPA, .GPIO = RegMCP230XX0_GPIOA, .OLAT = RegMCP230XX0_OLATA, .IOCON = RegMCP230XX0_IOCONA, }, // MCP2301X
  // 2-port devices
  { .DevPortCount = 2, .IODIR = RegMCP230XX_IODIR0 , .IPOL = RegMCP230XX_IPOL0 , .GPINTEN = 0xFF                 , .DEFVAL = 0xFF                , .INTCON = 0xFF                , .GPPU = 0xFF              , .INTF = 0xFF              , .INTCAP = RegMCP230XX_INTCAP0 , .GPIO = RegMCP230XX_GP0   , .OLAT = RegMCP230XX_OLAT0 , .IOCON = RegMCP230XX_IOCON0 , }, // MCP23016
  { .DevPortCount = 2, .IODIR = RegMCP230XX0_IODIRA, .IPOL = RegMCP230XX0_IPOLA, .GPINTEN = RegMCP230XX0_GPINTENA, .DEFVAL = RegMCP230XX0_DEFVALA, .INTCON = RegMCP230XX0_INTCONA, .GPPU = RegMCP230XX0_GPPUA, .INTF = RegMCP230XX0_INTFA, .INTCAP = RegMCP230XX0_INTCAPA, .GPIO = RegMCP230XX0_GPIOA, .OLAT = RegMCP230XX0_OLATA, .IOCON = RegMCP230XX0_IOCONA, }, // MCP23017
  { .DevPortCount = 2, .IODIR = RegMCP230XX0_IODIRA, .IPOL = RegMCP230XX0_IPOLA, .GPINTEN = RegMCP230XX0_GPINTENA, .DEFVAL = RegMCP230XX0_DEFVALA, .INTCON = RegMCP230XX0_INTCONA, .GPPU = RegMCP230XX0_GPPUA, .INTF = RegMCP230XX0_INTFA, .INTCAP = RegMCP230XX0_INTCAPA, .GPIO = RegMCP230XX0_GPIOA, .OLAT = RegMCP230XX0_OLATA, .IOCON = RegMCP230XX0_IOCONA, }, // MCP23018
  { .DevPortCount = 2, .IODIR = RegMCP230XX_IODIR  , .IPOL = RegMCP230XX_IPOL  , .GPINTEN = RegMCP230XX_GPINTEN  , .DEFVAL = RegMCP230XX_DEFVAL  , .INTCON = RegMCP230XX_INTCON  , .GPPU = RegMCP230XX_GPPU  , .INTF = RegMCP230XX_INTF  , .INTCAP = RegMCP230XX_INTCAP  , .GPIO = RegMCP230XX_GPIO  , .OLAT = RegMCP230XX_OLAT  , .IOCON = RegMCP230XX_IOCON  , }, // MCP2300X
};

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// MCP230XX initialization
//=============================================================================
eERRORRESULT Init_MCP230XX(MCP230XX *pComp, const MCP230XX_Config *pConf)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (pConf == NULL)) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pI2C == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pI2C->fnI2C_Init == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Data[2];
  bool InSafeSCLspeed = false;

  //--- Check and adjust I2C SCL speed ---
  uint32_t SCLfreq = pComp->I2CclockSpeed;
  if ((pComp->DeviceName == MCP230XX_AUTODETECT) && (pComp->I2CclockSpeed > MCP230XX_I2CCLOCK_SAFE))
  { SCLfreq = MCP230XX_I2CCLOCK_SAFE; InSafeSCLspeed = true; }                // In case of Autodetect, we do not know yet the maximum speed of the device. We need to set a safe SCL clock speed

  //--- Initialize I2C interface ---
  Error = pI2C->fnI2C_Init(pI2C, SCLfreq);
  if (Error != ERR_NONE) return Error;                                        // If there is an error while calling fnI2C_Init() then return the error
  if (MCP230XX_IsReady(pComp)) return ERR__NO_DEVICE_DETECTED;

  //--- Check device ---
  if ((pComp->DeviceName == MCP230XX_AUTODETECT) || (pComp->DeviceName >= eMCP230XX_DEVICE_COUNT))
  {
    //--- Autodetect device ---
    // There is no register that can says which device is connected
    // After a POR/RESET, the device is at known state, only the registers 0x00 and 0x01 can says with a pretty good accuracy what kind of device it is
    Error = MCP230XX_ReadRegister(pComp, 0x00, &Data[0], sizeof(Data));       // Read registers 0x00 and 0x01
    if (Error == ERR__I2C_NACK) return ERR__NO_DEVICE_DETECTED;               // If there is no ACK, the device is not present
    if (Error != ERR_NONE) return Error;                                      // If there is an error while calling MCP230XX_ReadRegister() then return the error
    if ((Data[0] == 0xFF) && (Data[1] == 0x00)) pComp->DeviceName = MCP2300X; // Can be a MCP23008 or a MCP23009
    if ((Data[0] == 0x00) && (Data[1] == 0x00)) pComp->DeviceName = MCP23016; // It is a MCP23016
    if ((Data[0] == 0xFF) && (Data[1] == 0xFF)) pComp->DeviceName = MCP2301X; // Can be a MCP23017 or a MCP23018
    if (pComp->DeviceName == MCP230XX_AUTODETECT) return ERR__UNKNOWN_DEVICE; // Here the device have been used and is not at reset state, so there is no possibilities to be sure of which device it is
  }

  //--- Finally set the desired I2C SCL speed ---
  if (__MCP230XX_CheckSCLspeed(pComp) == false) return ERR__I2C_FREQUENCY_ERROR;
  if (InSafeSCLspeed)
  {
    Error = pI2C->fnI2C_Init(pI2C, pComp->I2CclockSpeed);                     // Reinit the I2C and set the desired speed
    if (Error != ERR_NONE) return Error;                                      // If there is an error while calling fnI2C_Init() then return the error
  }

  //--- Configure pins ---
  const size_t GPcount = MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount;
  uint16_t IODIR = 0x0000, IPOL = 0x0000, OLAT = 0x0000, GPPU = 0x0000;                  // GPIOs registers
  uint16_t GPINTEN = 0x0000, INTCON = 0x0000, DEFVAL = 0x0000;                           // GPIOs interrupt registers
  for (uint8_t Pin = 0; Pin < 16; Pin++)                                                 // Prepare all the registers following parameters of each pins
  {
    if ((pConf->GPs[Pin] & MCP230XX_PIN_AS_INPUT                   ) > 0) IODIR   |= (1 << Pin); // Set the pin direction for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_GPIO_STATE_INVERTED_LOGIC      ) > 0) IPOL    |= (1 << Pin); // Set the input polarity for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_GPIO_OUTPUT_STATE_HIGH         ) > 0) OLAT    |= (1 << Pin); // Set the output level for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_GPIO_PULLUP_ENABLE             ) > 0) GPPU    |= (1 << Pin); // Set the internal pull-up for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_INTERRUPT_ON_CHANGE_ENABLE     ) > 0) GPINTEN |= (1 << Pin); // Set the interrupt on change for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_VALUE_COMPARED_DEFAULT_VALUE   ) > 0) INTCON  |= (1 << Pin); // Set the interrupt control for the current pin
    if ((pConf->GPs[Pin] & MCP230XX_DEFAULT_VALUE_NO_INT_LEVEL_HIGH) > 0) DEFVAL  |= (1 << Pin); // Set the default value of the pin
  }
  Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].OLAT , (uint8_t*)&OLAT, GPcount);        // Write OLAT register
  if (Error != ERR_NONE) return Error;                                                                                       // If there is an error while calling MCP230XX_WriteRegister() then return the error
  Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].IODIR, (uint8_t*)&IODIR, GPcount);       // Write IODIR register
  if (Error != ERR_NONE) return Error;                                                                                       // If there is an error while calling MCP230XX_WriteRegister() then return the error
  Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].IPOL , (uint8_t*)&IPOL , GPcount);       // Write IPOL register
  if (Error != ERR_NONE) return Error;                                                                                       // If there is an error while calling MCP230XX_WriteRegister() then return the error
  if (pComp->DeviceName != MCP23016)
  {
    Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].GPINTEN, (uint8_t*)&GPINTEN, GPcount); // Write GPINTEN register
    if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP230XX_WriteRegister() then return the error
    Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].DEFVAL , (uint8_t*)&DEFVAL , GPcount); // Write DEFVAL register
    if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP230XX_WriteRegister() then return the error
    Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].INTCON , (uint8_t*)&INTCON , GPcount); // Write INTCONDIR register
    if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP230XX_WriteRegister() then return the error
    Error = MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].GPPU   , (uint8_t*)&GPPU   , GPcount); // Write GPPU register
    if (Error != ERR_NONE) return Error;                                                                                     // If there is an error while calling MCP230XX_WriteRegister() then return the error
  }

  //--- Configure device ---
  uint16_t IOCON = 0x00;
  if (pComp->DeviceName != MCP23016)
  {
    if ((pComp->ControlFlags & MCP230XX_READING_INTCAP_CLEARS_INTERRUPT     ) > 0) IOCON |= MCP23018_IOCON_INTCC_INTCAPCLEARS;
    if ((pComp->ControlFlags & MCP230XX_INT_PIN_OUTPUT_POLARITY_ACTIVE_HIGH ) > 0) IOCON |= MCP23018_IOCON_INTPOL_ACTIVEHIGH;
    if ((pComp->ControlFlags & MCP230XX_INT_PIN_OUTPUT_OPEN_DRAIN           ) > 0) IOCON |= MCP23018_IOCON_ODR_OPENDRAIN;
    if ((pComp->ControlFlags & MCP230XX_SLEW_RATE_CONTROL_SDA_OUTPUT_DISABLE) > 0) IOCON |= MCP23017_IOCON_DISSLW_DISABLE;
    if ((pComp->ControlFlags & MCP230XX_INT_PIN_MIRRORED                    ) > 0) IOCON |= MCP23018_IOCON_MIRROR_DISABLE;
  }
  else
  {
    if ((pComp->ControlFlags & MCP230XX_GPIO_SAMPLING_RATE_FAST             ) > 0) IOCON |= MCP23016_IOCON_IARES_FASTSAMPLERATE;
  }
  return MCP230XX_WriteRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].IOCON, (uint8_t*)&IOCON, GPcount);        // Write IOCON register
}


//=============================================================================
// Is the MCP230XX device ready
//=============================================================================
bool MCP230XX_IsReady(MCP230XX *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return false;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pI2C == NULL) return false;
# endif
  if (pI2C->fnI2C_Transfer == NULL) return false;
#endif
  I2CInterface_Packet PacketDesc = I2C_INTERFACE8_NO_DATA_DESC((MCP230XX_CHIPADDRESS_BASE | pComp->AddrA2A1A0) & I2C_WRITE_ANDMASK);
  return (pI2C->fnI2C_Transfer(pI2C, &PacketDesc) == ERR_NONE); // Send only the chip address and get the Ack flag
}

//-----------------------------------------------------------------------------





//=============================================================================
// [STATIC] Check I2C clock speed
//=============================================================================
bool __MCP230XX_CheckSCLspeed(MCP230XX *pComp)
{
  if ((pComp->DeviceName == MCP23008) && (pComp->I2CclockSpeed > MCP23008_I2CCLOCK_MAX)) return false; // Wrong MCP23008 SCL clock speed
  if ((pComp->DeviceName == MCP23009) && (pComp->I2CclockSpeed > MCP23009_I2CCLOCK_MAX)) return false; // Wrong MCP23009 SCL clock speed
  if ((pComp->DeviceName == MCP2300X) && (pComp->I2CclockSpeed > MCP2300X_I2CCLOCK_MAX)) return false; // Wrong MCP2300X SCL clock speed
  if ((pComp->DeviceName == MCP23016) && (pComp->I2CclockSpeed > MCP23016_I2CCLOCK_MAX)) return false; // Wrong MCP23016 SCL clock speed
  if ((pComp->DeviceName == MCP23017) && (pComp->I2CclockSpeed > MCP23017_I2CCLOCK_MAX)) return false; // Wrong MCP23017 SCL clock speed
  if ((pComp->DeviceName == MCP23018) && (pComp->I2CclockSpeed > MCP23018_I2CCLOCK_MAX)) return false; // Wrong MCP23018 SCL clock speed
  if ((pComp->DeviceName == MCP2301X) && (pComp->I2CclockSpeed > MCP2301X_I2CCLOCK_MAX)) return false; // Wrong MCP2301X SCL clock speed
  return true;
}

//-----------------------------------------------------------------------------




//**********************************************************************************************************************************************************
//=============================================================================
// Read data from register of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_ReadRegister(MCP230XX *pComp, eMCP230XX_Registers reg, uint8_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pI2C == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t ChipAddr = (MCP230XX_CHIPADDRESS_BASE | pComp->AddrA2A1A0);

  //--- Send the address ---
  I2CInterface_Packet RegPacketDesc = I2C_INTERFACE8_TX_DATA_DESC(ChipAddr, true, &reg, sizeof(uint8_t), false, I2C_WRITE_THEN_READ_FIRST_PART);
  Error = pI2C->fnI2C_Transfer(pI2C, &RegPacketDesc);               // Transfer the register's address
  if (Error == ERR__I2C_NACK) return ERR__NOT_READY;                // If the device receive a NAK, then the device is not ready
  if (Error == ERR__I2C_NACK_DATA) return ERR__I2C_INVALID_ADDRESS; // If the device receive a NAK while transferring data, then this is an invalid address
  if (Error != ERR_NONE) return Error;                              // If there is an error while calling fnI2C_Transfer() then return the Error
  //--- Get the data ---
  I2CInterface_Packet DataPacketDesc = I2C_INTERFACE8_RX_DATA_DESC(ChipAddr, true, data, size, true, I2C_WRITE_THEN_READ_SECOND_PART);
  return pI2C->fnI2C_Transfer(pI2C, &DataPacketDesc);               // Restart at first data read transfer, get the data and stop transfer at last byte
}


//=============================================================================
// Write data to register of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_WriteRegister(MCP230XX *pComp, eMCP230XX_Registers reg, uint8_t* data, size_t size)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  I2C_Interface* pI2C = GET_I2C_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pI2C == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pI2C->fnI2C_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if ((uint8_t)reg == 0xFF) return ERR__NOT_SUPPORTED;
  eERRORRESULT Error;
  uint8_t ChipAddr = (MCP230XX_CHIPADDRESS_BASE | pComp->AddrA2A1A0);

  //--- Send the address ---
  I2CInterface_Packet RegPacketDesc = I2C_INTERFACE8_TX_DATA_DESC(ChipAddr, true, &reg, sizeof(uint8_t), false, I2C_WRITE_THEN_WRITE_FIRST_PART);
  Error = pI2C->fnI2C_Transfer(pI2C, &RegPacketDesc);               // Transfer the register's address
  if (Error == ERR__I2C_NACK) return ERR__NOT_READY;                // If the device receive a NAK, then the device is not ready
  if (Error == ERR__I2C_NACK_DATA) return ERR__I2C_INVALID_ADDRESS; // If the device receive a NAK while transferring data, then this is an invalid address
  if (Error != ERR_NONE) return Error;                              // If there is an error while calling fnI2C_Transfer() then return the Error
  //--- Send the data ---
  I2CInterface_Packet DataPacketDesc = I2C_INTERFACE8_TX_DATA_DESC(ChipAddr, false, data, size, true, I2C_WRITE_THEN_WRITE_SECOND_PART);
  return pI2C->fnI2C_Transfer(pI2C, &DataPacketDesc);               // Continue by transferring the data, and stop transfer at last byte
}

//-----------------------------------------------------------------------------





//=============================================================================
// Get all PORTs pins interrupt flags of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_GetPinsInterruptFlags(MCP230XX *pComp, uint16_t *intFlags)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Values[sizeof(uint16_t)] = { 0x00, 0x00 };
  eERRORRESULT Error = MCP230XX_ReadRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].INTF, &Values[0], MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount);
  *intFlags = (uint16_t)Values[1] << 8 | (uint16_t)Values[0];
  return Error;
}


//=============================================================================
// Get all PORTs pins interrupt capture of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_GetPinsInterruptCapture(MCP230XX *pComp, uint16_t *portCapture)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Values[sizeof(uint16_t)] = { 0x00, 0x00 };
  eERRORRESULT Error = MCP230XX_ReadRegister(pComp, MCP230XX_DevicesRegLinks[pComp->DeviceName].INTCAP, &Values[0], MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount);
  *portCapture = (uint16_t)Values[1] << 8 | (uint16_t)Values[0];
  return Error;
}

//-----------------------------------------------------------------------------





//=============================================================================
// Set pins direction on a PORT of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_SetPinsDirection(MCP230XX *pComp, const uint8_t port, const uint8_t pinsDirection, const uint8_t pinsChangeMask)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pComp->PORToutDir[port] &= ~pinsChangeMask;                  // Force change bits to 0
  pComp->PORToutDir[port] |= (pinsDirection & pinsChangeMask); // Apply new output level only on changed pins
  const eMCP230XX_Registers IODIRreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pComp->DeviceName].IODIR + port);
  return MCP230XX_WriteRegister(pComp, IODIRreg, &pComp->PORToutDir[port], 1);
}


//=============================================================================
// Get PORT0 pins input level of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_GetPinsInputLevel(MCP230XX *pComp, const uint8_t port, uint8_t *pinsState)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  const eMCP230XX_Registers GPIOreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pComp->DeviceName].GPIO + port);
  return MCP230XX_ReadRegister(pComp, GPIOreg, pinsState, 1);
}


//=============================================================================
// Set PORT0 pins output level of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_SetPinsOutputLevel(MCP230XX *pComp, const uint8_t port, const uint8_t pinsLevel, const uint8_t pinsChangeMask)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (port >= MCP230XX_DevicesRegLinks[pComp->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pComp->PORToutLevel[port] &= ~pinsChangeMask;                 // Force change bits to 0
  pComp->PORToutLevel[port] |= (pinsLevel & pinsChangeMask);    // Apply new output level only on changed pins
  const eMCP230XX_Registers OLATreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pComp->DeviceName].OLAT + port);
  return MCP230XX_WriteRegister(pComp, OLATreg, &pComp->PORToutLevel[port], 1);
}

//-----------------------------------------------------------------------------





#ifdef USE_GENERICS_DEFINED

//=============================================================================
// Set PORT direction of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_SetPORTdirection_Gen(PORT_Interface *pIntDev, const uint32_t pinsDirection)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP230XX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP230XX* pDevice = (MCP230XX*)(pIntDev->InterfaceDevice);         // Get the MCP230XX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP230XX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pDevice->PORToutDir[pIntDev->PORTindex] = (uint8_t)pinsDirection; // Apply new output direction on PORT
  const eMCP230XX_Registers IODIRreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pDevice->DeviceName].IODIR + pIntDev->PORTindex);
  return MCP230XX_WriteRegister(pDevice, IODIRreg, &pDevice->PORToutDir[pIntDev->PORTindex], 1);
}


//=============================================================================
// Get PORT pins input level of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_GetPORTinputLevel_Gen(PORT_Interface *pIntDev, uint32_t *pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP230XX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP230XX* pDevice = (MCP230XX*)(pIntDev->InterfaceDevice); // Get the MCP230XX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP230XX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  const eMCP230XX_Registers GPIOreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pDevice->DeviceName].GPIO + pIntDev->PORTindex);
  uint8_t Value = 0;
  eERRORRESULT Error = MCP230XX_ReadRegister(pDevice, GPIOreg, &Value, 1);
  *pinsLevel = Value;
  return Error;
}


//=============================================================================
// Set PORT pins output level of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_SetPORToutputLevel_Gen(PORT_Interface *pIntDev, const uint32_t pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP230XX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP230XX* pDevice = (MCP230XX*)(pIntDev->InterfaceDevice);     // Get the MCP230XX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP230XX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  pDevice->PORToutDir[pIntDev->PORTindex] = (uint8_t)pinsLevel; // Apply new output level on PORT
  const eMCP230XX_Registers OLATreg = (eMCP230XX_Registers)((uint8_t)MCP230XX_DevicesRegLinks[pDevice->DeviceName].OLAT + pIntDev->PORTindex);
  return MCP230XX_WriteRegister(pDevice, OLATreg, &pDevice->PORToutDir[pIntDev->PORTindex], 1);
}

//-----------------------------------------------------------------------------



//=============================================================================
// Set a pin on PORT direction of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_SetPinState_Gen(GPIO_Interface *pIntDev, const eGPIO_State pinState)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP230XX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP230XX* pDevice = (MCP230XX*)(pIntDev->InterfaceDevice); // Get the MCP230XX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP230XX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  switch (pinState)
  {
    default: break;
    case GPIO_STATE_OUTPUT: return MCP230XX_SetPinsDirection(pDevice, pIntDev->PORTindex, 0x00, pIntDev->PinBitMask);
    case GPIO_STATE_INPUT : return MCP230XX_SetPinsDirection(pDevice, pIntDev->PORTindex, pIntDev->PinBitMask, pIntDev->PinBitMask);
    case GPIO_STATE_RESET : return MCP230XX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, 0x00, pIntDev->PinBitMask);
    case GPIO_STATE_SET   : return MCP230XX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, pIntDev->PinBitMask, pIntDev->PinBitMask);
    case GPIO_STATE_TOGGLE: return MCP230XX_SetPinsOutputLevel(pDevice, pIntDev->PORTindex, pDevice->PORToutLevel[pIntDev->PORTindex] ^ (uint8_t)pIntDev->PinBitMask, pIntDev->PinBitMask);
  }
  return ERR__PARAMETER_ERROR;
}


//=============================================================================
// Get a pin on PORT input level of the MCP230XX device
//=============================================================================
eERRORRESULT MCP230XX_GetPinInputLevel_Gen(GPIO_Interface *pIntDev, eGPIO_State *pinLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__NULL_POINTER;
#endif
  if (pIntDev->UniqueID != MCP230XX_UNIQUE_ID) return ERR__UNKNOWN_ELEMENT;
  MCP230XX* pDevice = (MCP230XX*)(pIntDev->InterfaceDevice); // Get the MCP230XX device of this GPIO port
#ifdef CHECK_NULL_PARAM
  if (pDevice == NULL) return ERR__UNKNOWN_DEVICE;
  if (pIntDev->PORTindex >= MCP230XX_DevicesRegLinks[pDevice->DeviceName].DevPortCount) return ERR__PERIPHERAL_NOT_VALID;
#endif
  uint8_t PinState = 0;
  eERRORRESULT Error = MCP230XX_GetPinsInputLevel(pDevice, pIntDev->PORTindex, &PinState);
  *pinLevel = (PinState > 0 ? GPIO_STATE_SET : GPIO_STATE_RESET);
  return Error;
}

#endif

//-----------------------------------------------------------------------------
#ifdef _cplusplus
}
#endif
//-----------------------------------------------------------------------------