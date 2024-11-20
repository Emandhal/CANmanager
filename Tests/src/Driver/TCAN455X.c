/*!*****************************************************************************
 * @file    TCAN455X.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.4
 * @date    15/07/2023
 * @brief   TCAN4550 and TCAN4551 driver
 * @details
 * The TCAN4550/TCAN4551 component is a CAN-bus controller supporting CAN2.0A, CAN2.0B
 * and CAN-FD with SPI interface
 * Follow datasheet TCAN4550    Rev.A (Jan  2020)
 *                  TCAN4550-Q1 Rev.D (June 2022)
 *                  TCAN4551-Q1 Rev.A (Nov  2019)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "TCAN455X.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
#  include <cstdint>
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
/*! @brief Check actual TCAN455X device attached
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
static eERRORRESULT __TCAN455X_CheckDeviceID(TCAN455X *pComp);
/*! @brief Test all RAM address of the TCAN455X for the driver flag DRIVER_INIT_CHECK_RAM
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @return Returns an #eERRORRESULT value enum
 */
static eERRORRESULT __TCAN455X_TestRAM(TCAN455X *pComp);
//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// MCAN on SAMV71 peripheral initialization
//=============================================================================
eERRORRESULT Init_TCAN455X(TCAN455X *pComp, const TCAN455X_Config* const pConf, const MCAN_FIFObuff* const listFIFO, const size_t listFIFOcount)
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
  const uint32_t StartAddress = TCAN455X_RAM_START_ADDRESS;
  eERRORRESULT Error;

  //--- Prepare MCAN Interface ------------------------------
  pComp->InternalConfig = MCAN_DEV_PS_SET(MCAN_DEVICE_SLEEP_NOT_CONFIGURED);                             // Device is in normal power state, sleep is not yet configured
  pComp->_MCAN.pMCANdriver    = (void*)pComp;
  pComp->_MCAN.MaxRAMaddress  = TCAN455X_MAX_RAM_ADDRESS;
  pComp->_MCAN.fnReadREG32    = TCAN455X_ReadMCANREG32;
  pComp->_MCAN.fnWriteREG32   = TCAN455X_WriteMCANREG32;
  pComp->_MCAN.fnReadRAM      = TCAN455X_ReadRAM;
  pComp->_MCAN.fnWriteRAM     = TCAN455X_WriteRAM;
  pComp->_MCAN.fnGetCurrentms = pComp->fnGetCurrentms;

  //--- Configure SPI Interface -----------------------------
  Error = pSPI->fnSPI_Init(pSPI, pComp->SPIchipSelect, STD_SPI_MODE0, pComp->SPIclockSpeed);             // Configure SPI
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling fnSPI_Init() then return the error

  //--- Reset -----------------------------------------------
  Error = TCAN455X_ResetDevice(pComp);                                                                   // Reset the device
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling TCAN455X_ResetDevice() then return the error

  //--- Check endianness and test SPI connection ------------
  Error = MCAN_CheckEndianness(&pComp->_MCAN);                                                           // Check endianness. If no proper communication, this function will return ERR__NO_DEVICE_DETECTED
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling MCAN_CheckEndianness() then return the error

  //--- Check device name -----------------------------------
  Error = __TCAN455X_CheckDeviceID(pComp);                                                               // Check TCAN455X device connected
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling __TCAN455X_CheckDeviceID() then return the error

  //--- Configure component clock ---------------------------
  pComp->_MCAN.PeripheralClock = TCAN455X_CRYSTAL_TO_FREQ(pConf->XtalFreq);
  if (pConf->MCANconf.SYSCLK_Result != NULL) *pConf->MCANconf.SYSCLK_Result = pComp->_MCAN.PeripheralClock; // Save the internal SYSCLK if needed
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_CLKIN_FREQ_SET(pConf->XtalFreq)
                       | (pConf->FailSafeEnable ? TCAN455X_MODE_FAIL_SAFE_ENABLE : TCAN455X_MODE_FAIL_SAFE_DISABLE)
                       | (pConf->SleepWakeDisable ? TCAN455X_MODE_SLEEP_WAKE_ERROR_DISABLE : TCAN455X_MODE_SLEEP_WAKE_ERROR_ENABLE);
  const uint32_t Mask = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_CLKIN_FREQ_Mask | TCAN455X_MODE_FAIL_SAFE_Mask | TCAN455X_MODE_SLEEP_WAKE_ERROR_Mask;
  Error = TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);                      // Configure Xtal, fail safe, and sleep wake
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling TCAN455X_ModifyREG32() then return the error

  //--- Configure Pins --------------------------------------
  Error = TCAN455X_ConfigurePins(pComp, pConf->GPIO1PinMode, pConf->GPIO2PinMode, pConf->WakePinConf, pConf->INHpinEnable, pConf->nWKRQconfig, pConf->nWKRQvoltageRef);
  if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling TCAN455X_ConfigurePins() then return the error

  //--- Test SPI connection and RAM Test --------------------
  if ((pComp->DriverConfig & TCAN455X_DRIVER_INIT_CHECK_RAM) > 0)                                        // If there is a DRIVER_INIT_CHECK_RAM flag then
  {
    Error = __TCAN455X_TestRAM(pComp);                                                                   // Check the all the RAM only of the device
    if (Error != ERR_NONE) return Error;                                                                 // If there is an error while calling __MCP251XFD_TestRAM() then return the error
  }
  else                                                                                                   // Else check only SPI interface
  {
    uint32_t Result;
    Error = TCAN455X_WriteRAM32(pComp, (TCAN455X_RAM_START_ADDRESS + TCAN455X_MAX_RAM - 4), 0xAA55AA55); // Write 0xAA55AA55 at address
    if (Error != ERR_NONE) return Error;                                                                 // If there is an error while writing the RAM address then return the error
    Error = TCAN455X_ReadRAM32(pComp, (TCAN455X_RAM_START_ADDRESS + TCAN455X_MAX_RAM - 4), &Result);     // Read again the data
    if (Error != ERR_NONE) return Error;                                                                 // If there is an error while reading the RAM address then return the error
    if (Result != 0xAA55AA55) return ERR__RAM_TEST_FAIL;                                                 // If data read is not 0xAA55AA55 then return an error
  }

  //--- Configure RAM ECC -----------------------------------
  if ((pComp->DriverConfig & TCAN455X_DRIVER_ENABLE_ECC) > 0)                                            // If there is a DRIVER_ENABLE_ECC flag then
  {
    Error = TCAN455X_ConfigureTestECC(pComp, true, 0x15);                                                // Configure the ECC
    if (Error != ERR_NONE) return Error;                                                                 // If there is an error while calling TCAN455X_ConfigureTestECC() then return the error
  }

  //--- Do general MCAN configuration -----------------------
  uint32_t CurrAddress = StartAddress;
  Error = __Init_MCAN(&pComp->_MCAN, (const MCAN_Config* const)&pConf->MCANconf, listFIFO, listFIFOcount, &CurrAddress); // Finish MCAN configuration
  if (CurrAddress > TCAN455X_MAX_RAM) return ERR__OUT_OF_MEMORY;                                         // Check RAM

  //--- Initialize RAM if configured ------------------------
  if ((pComp->DriverConfig & MCAN_DRIVER_INIT_SET_RAM_AT_0) > 0)                                         // If there is a DRIVER_INIT_SET_RAM_AT_0 flag then
  {
    for (uint32_t zAddr = StartAddress; zAddr < CurrAddress; zAddr += sizeof(uint32_t))
    {
      Error = TCAN455X_WriteRAM(pComp, zAddr, 0x00000000, 4);                                            // Write 0 on all used space
      if (Error != ERR_NONE) return Error;                                                               // If there is an error while calling TCAN455X_WriteRAM() then return the error
    }
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Read from a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT TCAN455X_ReadMCANREG32(void *pComp, const uint32_t address, uint32_t* data)
{
  if (MCAN_IS_USE_CACHE(address))
  {
    const size_t CacheIdx = MCAN_CACHE_IDX_GET(address);
    if (CacheIdx >= MCAN_CACHE_COUNT) return ERR__INVALID_DATA;
    *data = ((TCAN455X*)pComp)->__RegCache[CacheIdx];
    return ERR_NONE;
  }
  return TCAN455X_ReadRAM(pComp, TCAN455X_CONVERT_TO_MCAN_REG_ADDR(address), (uint8_t*)&data, 4);
}


//=============================================================================
// Write to a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT TCAN455X_WriteMCANREG32(void *pComp, const uint32_t address, const uint32_t data)
{
  if (MCAN_IS_USE_CACHE(address))
  {
    const size_t CacheIdx = MCAN_CACHE_IDX_GET(address);
    if (CacheIdx >= MCAN_CACHE_COUNT) return ERR__INVALID_DATA;
    ((TCAN455X*)pComp)->__RegCache[CacheIdx] = data;
  }
  return TCAN455X_WriteRAM(pComp, TCAN455X_CONVERT_TO_MCAN_REG_ADDR(address), (const uint8_t*)&data, 4);
}

//-----------------------------------------------------------------------------




//=============================================================================
// Read from a 32-bits register of TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_ReadREG32(TCAN455X *pComp, const uint32_t address, uint32_t* data)
{
  return TCAN455X_ReadRAM(pComp, address, (uint8_t*)&data, 4);
}


//=============================================================================
// Write to a 32-bits register of TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_WriteREG32(TCAN455X *pComp, const uint32_t address, const uint32_t data)
{
  return TCAN455X_WriteRAM(pComp, address, (const uint8_t*)&data, 4);
}


//=============================================================================
// Modify a register of the TCAN455X
//=============================================================================
eERRORRESULT TCAN455X_ModifyREG32(TCAN455X *pComp, const uint32_t registerAddr, uint32_t registerValue, uint32_t registerMask)
{
  eERRORRESULT Error;
  uint32_t RegValue;
  Error = TCAN455X_ReadREG32(pComp, registerAddr, &RegValue); // Read the register value
  if (Error != ERR_NONE) return Error;                        // If there is an error while calling TCAN455X_ReadREG32() then return the error
  RegValue &= ~registerMask;                                  // Clear bits to modify
  RegValue |= (registerValue & registerMask);                 // Set the new value
  return TCAN455X_WriteREG32(pComp, registerAddr, RegValue);  // Write the value to register
}

//-----------------------------------------------------------------------------



//=============================================================================
// Read from memory data of TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_ReadRAM(void *_pComp, const uint32_t address, uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((_pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  TCAN455X* const pComp = (TCAN455X*)_pComp;
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (address > RegTCAN455X_MRAM_END) return ERR__BAD_ADDRESS;
  if ((address & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  if ((count & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__BAD_DATA_SIZE;       // Only 4-bytes multiple allowed
  eERRORRESULT Error;
  const uint8_t ReadHeader[4] = { TCAN455X_READ_B_FL, (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF), (count >> 2) };

  //--- Send the header ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ReadHeader[0], sizeof(ReadHeader), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the register's address
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Get the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_RX_DATA_DESC(data, count, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by reading the data, get the data and stop transfer at last byte
}


//=============================================================================
// Write to memory data of TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_WriteRAM(void *_pComp, const uint32_t address, const uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((_pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  TCAN455X* const pComp = (TCAN455X*)_pComp;
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (address > RegTCAN455X_MRAM_END) return ERR__BAD_ADDRESS;
  if ((address & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  if ((count & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__BAD_DATA_SIZE;       // Only 4-bytes multiple allowed
  eERRORRESULT Error;
  const uint8_t WriteHeader[4] = { TCAN455X_WRITE_B_FL, (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF), (count >> 2) };

  //--- Send the header ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&WriteHeader[0], sizeof(WriteHeader), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the command, address, and length
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Send the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_TX_DATA_DESC(data, count, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by transferring the data, and stop transfer at last byte
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// [STATIC] Check actual TCAN455X device attached
//=============================================================================
eERRORRESULT __TCAN455X_CheckDeviceID(TCAN455X *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  pComp->InternalConfig &= ~TCAN455X_DEV_ID_Mask;                                              // Reset device found
  eERRORRESULT Error;

  TCAN455X_DEVID_Register RegDEVID;
  Error = TCAN455X_ReadRAM(pComp, RegTCAN455X_REVISION, &RegDEVID.Bytes[0], sizeof(RegDEVID)); // Get TCAN455X Device ID Register
  if (Error != ERR_NONE) return Error;                                                         // If there is an error while calling TCAN455X_ReadRAM() then return the error
  if (TCAN455X_DEVID1_GET(RegDEVID.DEVID1) == TCAN4550_DEVID1_VALUE)
  {
    if (TCAN455X_DEVID2_GET(RegDEVID.DEVID2) == TCAN4550_DEVID2_VALUE)
         pComp->InternalConfig |= TCAN455X_DEV_ID_SET(TCAN4550);                               // Set TCAN4550 device found
    else if (TCAN455X_DEVID2_GET(RegDEVID.DEVID2) == TCAN4551_DEVID2_VALUE)
         pComp->InternalConfig |= TCAN455X_DEV_ID_SET(TCAN4551);                               // Set TCAN4551 device found
    else return ERR__UNKNOWN_DEVICE;                                                           // Else return an UNKNOWN_DEVICE error
  }
  else return ERR__UNKNOWN_DEVICE;                                                             // Else return an UNKNOWN_DEVICE error
  return ERR_NONE;
}


//=============================================================================
// Get actual device of the TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_GetDeviceID(TCAN455X *pComp, eTCAN455X_Devices* const device, uint8_t* const revIdMajor, uint8_t* const revIdMinor, uint8_t* const revSPI)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (device == NULL)) return ERR__PARAMETER_ERROR;
#endif
  *device = TCAN455X_DEV_ID_GET(pComp->InternalConfig);                                // Get device found when initializing the module
  eERRORRESULT Error;

  if ((revIdMajor != NULL) || (revIdMinor != NULL) || (revSPI != NULL))
  {
    uint32_t Value;
    Error = TCAN455X_ReadREG32(pComp, RegTCAN455X_REVISION, &Value);                   // Read value of the REVISION register
    if (Error != ERR_NONE) return Error;                                               // If there is an error while calling TCAN455X_ReadREG32() then return the error
    if (revIdMajor != NULL) *revIdMajor = TCAN455X_REVISION_REV_ID_MINOR_GET(Value);   // Get Device ID Major revision
    if (revIdMinor != NULL) *revIdMinor = TCAN455X_REVISION_REV_ID_MAJOR_GET(Value);   // Get Device ID Minor revision
    if (revSPI     != NULL) *revSPI     = TCAN455X_REVISION_SPI_2_REVISION_GET(Value); // Get Device SPI Revision
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Configure pins of the TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_ConfigurePins(TCAN455X *pComp, eTCAN455X_GPIO1Mode gpio1PinMode, eTCAN455X_GPIO2Mode gpio2PinMode, eTCAN455X_WAKEpinConf wakePinConf, bool inhPinEnable, eTCAN455X_nWKRQconfig nWKRQconfig, eTCAN455X_nWKRQvoltRef nWKRQvoltageRef)
{
  const uint32_t Value = TCAN455X_MODE_RESERVED_BIT5 | (uint32_t)gpio1PinMode | TCAN455X_MODE_GPO2_SET(gpio2PinMode) | TCAN455X_MODE_WAKE_PIN_SET(wakePinConf)
                       | (inhPinEnable ? TCAN455X_MODE_INH_PIN_ENABLE : TCAN455X_MODE_INH_PIN_DISABLE)
                       | TCAN455X_MODE_nWKRQ_CONFIG_SET(nWKRQconfig) | TCAN455X_MODE_nWKRQ_VOLTAGE_SET(nWKRQvoltageRef);
  const uint32_t Mask  = TCAN455X_MODE_RESERVED_BIT5 | TCAN455X_MODE_GPIO1_Mask | TCAN455X_MODE_GPO1_Mask | TCAN455X_MODE_GPO2_Mask | TCAN455X_MODE_WAKE_PIN_Mask | TCAN455X_MODE_INH_PIN_Mask | TCAN455X_MODE_nWKRQ_CONFIG_Mask | TCAN455X_MODE_nWKRQ_VOLTAGE_Mask;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_MODES_AND_PINS, Value, Mask);
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Configure ECC tests of the TCAN455X device
//=============================================================================
eERRORRESULT TCAN455X_ConfigureTestECC(TCAN455X *pComp, bool forceSingleBitError, uint8_t forceBitSel)
{
  if (forceBitSel > TCAN455X_ECC_ERR_FORCE_BIT_SEL_MAX_VALUE) return ERR__PARAMETER_ERROR;
  const uint32_t Value = TCAN455X_ECC_ERR_FORCE_BIT_SEL_SET(forceBitSel) | (forceSingleBitError ? TCAN455X_ECC_TEST_FORCE_SINGLE_BIT_ERROR : 0);
  const uint32_t Mask  = TCAN455X_ECC_ERR_FORCE_BIT_SEL_Mask | TCAN455X_ECC_TEST_FORCE_SINGLE_BIT_ERROR;
  return TCAN455X_ModifyREG32(pComp, RegTCAN455X_DEV_ECC_TEST, Value, Mask);
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Test all RAM address of the TCAN455X for the driver flag DRIVER_INIT_CHECK_RAM
//=============================================================================
eERRORRESULT __TCAN455X_TestRAM(TCAN455X *pComp)
{
  eERRORRESULT Error;
  uint32_t Return = 0;
  for (uint16_t Address = TCAN455X_RAM_START_ADDRESS; Address < TCAN455X_MAX_RAM_ADDRESS; Address += 4)
  {
    Error = TCAN455X_WriteRAM32(pComp, Address, 0x55555555); // Write 0x55555555 at address
    if (Error != ERR_NONE) return Error;                     // If there is an error while writing the RAM address then return the error
    Error = TCAN455X_ReadRAM32(pComp, Address, &Return);     // Read again the data
    if (Error != ERR_NONE) return Error;                     // If there is an error while reading the RAM address then return the error
    if (Return != 0x55555555) return ERR__RAM_TEST_FAIL;     // If data read is not 0x55555555 then return an error

    Error = TCAN455X_WriteRAM32(pComp, Address, 0xAAAAAAAA); // Write 0xAAAAAAAA at address
    if (Error != ERR_NONE) return Error;                     // If there is an error while writing the RAM address then return the error
    Error = TCAN455X_ReadRAM32(pComp, Address, &Return);     // Read again the data
    if (Error != ERR_NONE) return Error;                     // If there is an error while reading the RAM address then return the error
    if (Return != 0xAAAAAAAA) return ERR__RAM_TEST_FAIL;     // If data read is not 0xAAAAAAAA then return an error
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------