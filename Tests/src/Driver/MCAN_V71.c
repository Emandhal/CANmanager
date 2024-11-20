/*!*****************************************************************************
 * @file    MCAN_V71.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    22/05/2021
 * @brief   MCAN driver for ATSAMV71 MCUs
 *
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "MCAN_V71.h"
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





//********************************************************************************************************************
// SAMV71 MCAN peripheral functions
//********************************************************************************************************************
//=============================================================================
// This function is called when the MCANV71 needs to configure the peripheral clock
//=============================================================================
eERRORRESULT MCANV71_ConfigurePeripheralClocks(MCANV71 *pComp, uint32_t* const peripheralClock)
{ // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
  #ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (peripheralClock == NULL)) return ERR__PARAMETER_ERROR;
  #endif
  volatile Mcan* pMCAN = (volatile Mcan*)pComp->Instance;
  //-- Get peripheral ID ---
  uint32_t PeriphID = 0;
  if (pMCAN == MCAN0) PeriphID = ID_MCAN0;
  else if (pMCAN == MCAN1) PeriphID = ID_MCAN1;
  else return ERR__PERIPHERAL_NOT_VALID;
  //--- Enable clock ---
  static bool Is_MCAN_PMC_PCK_Enabled = false;
  if (Is_MCAN_PMC_PCK_Enabled == false)
  {
    pmc_disable_pck(PMC_PCK_5);
    pmc_switch_pck_to_upllck(PMC_PCK_5, PMC_PCK_PRES(5)); // UPLL is 480MHz, so UPLL/(5+1) = 80MHz
    pmc_enable_pck(PMC_PCK_5);
    NVIC_EnableIRQ(PMC_PCK_5);
    Is_MCAN_PMC_PCK_Enabled = true;
  }
  pmc_enable_periph_clk(PeriphID);
  *peripheralClock = PLL_UPLL_HZ / (5+1);
  return ERR_NONE;
}


#define CCFG_CAN0_CAN0DMABA_SET(value)   ( (value) & CCFG_CAN0_CAN0DMABA_Msk  )
#define CCFG_SYSIO_CAN1DMABA_SET(value)  ( (value) & CCFG_SYSIO_CAN1DMABA_Msk )
//=============================================================================
// This function is called when the MCANV71 needs to configure the 16-bit MSB of
// the MCAN base address (replace weak function in "MCANV71")
//=============================================================================
eERRORRESULT MCANV71_ConfigureMCANbaseAddress(MCANV71 *pComp)
{
  if (pComp->Instance == MCAN0)
  MATRIX->CCFG_CAN0  = (MATRIX->CCFG_CAN0  & ~CCFG_CAN0_CAN0DMABA_Msk ) | CCFG_CAN0_CAN0DMABA_SET((uint32_t)pComp->RAMallocation);
  else MATRIX->CCFG_SYSIO = (MATRIX->CCFG_SYSIO & ~CCFG_SYSIO_CAN1DMABA_Msk) | CCFG_SYSIO_CAN1DMABA_SET((uint32_t)pComp->RAMallocation);
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// MCAN on SAMV71 peripheral initialization
//=============================================================================
eERRORRESULT Init_MCANV71(MCANV71 *pComp, const MCAN_Config* const pConf, const MCAN_FIFObuff* const listFIFO, const size_t listFIFOcount)
{
  const uint32_t StartAddress = MCANV71_RAM_START_ADDRESS;
  eERRORRESULT Error;
  
  //--- Checks ----------------------------------------------
  if ((StartAddress & 0x3) > 0) return ERR__ADDRESS_ALIGNMENT;                            // Shall be 4-bytes aligned

  //--- Configure peripheral clock --------------------------
  Error = MCANV71_ConfigurePeripheralClocks(pComp, &pComp->_MCAN.PeripheralClock);        //** This is microcontroller specific. It is called to enable peripheral clock and get the peripheral clock
  if (Error != ERR_NONE) return Error;                                                    // If there is an error while calling MCANV71_ConfigurePeripheralClocks() then return the error
  if (pComp->_MCAN.PeripheralClock < MCAN_PERIPHERAL_CLK_MIN) return ERR__FREQUENCY_ERROR;
  if (pComp->_MCAN.PeripheralClock > MCAN_PERIPHERAL_CLK_MAX) return ERR__FREQUENCY_ERROR;
  if (pConf->SYSCLK_Result != NULL) *pConf->SYSCLK_Result = pComp->_MCAN.PeripheralClock; // Save the internal SYSCLK if needed

  //--- Reset -----------------------------------------------
  pComp->_MCAN.pMCANdriver    = pComp;
  pComp->_MCAN.MaxRAMaddress  = MCANV71_MAX_RAM_ADDRESS;
  pComp->_MCAN.fnReadREG32    = MCANV71_ReadREG32;
  pComp->_MCAN.fnWriteREG32   = MCANV71_WriteREG32;
  pComp->_MCAN.fnReadRAM      = MCANV71_ReadRAM;
  pComp->_MCAN.fnWriteRAM     = MCANV71_WriteRAM;
  pComp->_MCAN.fnGetCurrentms = pComp->fnGetCurrentms;

  //--- Check endianness ------------------------------------
  Error = MCAN_CheckEndianness(&pComp->_MCAN);                                            // Check endianness
  if (Error != ERR_NONE) return Error;                                                    // If there is an error while calling MCAN_CheckEndianness() then return the error

  //--- Configure upper base address ------------------------
  Error = MCANV71_ConfigureMCANbaseAddress(pComp);                                        //** This is microcontroller specific. Some allows the configuration of the 16-bit MSB of the MCAN DMA base address
  if (Error != ERR_NONE) return Error;                                                    // If there is an error while calling MCANV71_ConfigureMCANbaseAddress() then return the error

  //--- Do general MCAN configuration -----------------------
  uint32_t CurrAddress = StartAddress;
  Error = __Init_MCAN(&pComp->_MCAN, pConf, listFIFO, listFIFOcount, &CurrAddress);       // Finish MCAN configuration
  if (CurrAddress >= MCANV71_MAX_RAM_ADDRESS) return ERR__OUT_OF_MEMORY;                  // Check RAM
  // Tricky verification: The RAM allocation can have its address at a position where the MCAN can misplace messages
  // Example: pComp->RAMallocation address is 0x1E000 and pComp->RAMsize is 15kB, the configuration is in a way that *startAddr have its 16-bit high address that is higher like 0x1E000+2F00 = 0x20F00,
  //          if the MCAN peripheral need to access the 0x20000 address, it will consider the address as 0x10000 because of the 16-bit high address that is stored fixed.
  if (((StartAddress & MCAN_BASE_ADDRESS_MASK) ^ ((CurrAddress - 1u) & MCAN_BASE_ADDRESS_MASK)) > 0) return ERR__BAD_ADDRESS;

  //--- Initialize RAM if configured ------------------------
  if ((pComp->DriverConfig & MCAN_DRIVER_INIT_SET_RAM_AT_0) > 0)                          // If there is a DRIVER_INIT_SET_RAM_AT_0 flag then
  {
    for (uint32_t zAddr = StartAddress; zAddr < CurrAddress; zAddr += sizeof(uint32_t))
    {
      Error = MCANV71_WriteRAM(pComp, zAddr, 0x00000000, 4);                              // Write 0 on all used space
      if (Error != ERR_NONE) return Error;                                                // If there is an error while calling MCANV71_WriteRAM() then return the error
    }
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
//=============================================================================
// Read from a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReadREG32(void *pComp, const uint32_t address, uint32_t* data)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if (address > (uint32_t)RegMCAN_TXEFA) return ERR__BAD_ADDRESS;
  *data = ((volatile MCAN_HardReg*)(((MCANV71*)pComp)->Instance))->Regs[MCAN_EXTRACT_REG_ADDR(address) >> 2];
  return ERR_NONE;
}


//=============================================================================
// Write to a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_WriteREG32(void *pComp, const uint32_t address, const uint32_t data)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (address > (uint32_t)RegMCAN_TXEFA) return ERR__BAD_ADDRESS;
  ((volatile MCAN_HardReg*)(((MCANV71*)pComp)->Instance))->Regs[MCAN_EXTRACT_REG_ADDR(address) >> 2] = data;
  return ERR_NONE;
}

//-----------------------------------------------------------------------------



//=============================================================================
// Read from allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReadRAM(void *_pComp, const uint32_t address, uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((_pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  MCANV71* pComp = (MCANV71*)_pComp;
  if ((address & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  uintptr_t CurrentAddr = ((uintptr_t)MCANV71_RAM_START_ADDRESS & MCAN_BASE_ADDRESS_MASK) + address;
  if ((CurrentAddr + count) >= MCANV71_MAX_RAM_ADDRESS) return ERR__OUT_OF_MEMORY;
  for (int_fast16_t z = 0; z < count; z += sizeof(uint32_t))
  {
    if (MCAN_IS_CHANGE_ENDIANNESS(&(pComp->_MCAN)))
    {
      data[0] = *((uint8_t*)(CurrentAddr + 3));
      data[1] = *((uint8_t*)(CurrentAddr + 2));
      data[2] = *((uint8_t*)(CurrentAddr + 1));
      data[3] = *((uint8_t*)(CurrentAddr + 0));
    }
    else
    {
      data[0] = *((uint8_t*)(CurrentAddr + 0));
      data[1] = *((uint8_t*)(CurrentAddr + 1));
      data[2] = *((uint8_t*)(CurrentAddr + 2));
      data[3] = *((uint8_t*)(CurrentAddr + 3));
    }
    CurrentAddr += sizeof(uint32_t);
    data        += sizeof(uint32_t);
  }
  return ERR_NONE;
}


//=============================================================================
// Write to allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_WriteRAM(void *_pComp, const uint32_t address, const uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((_pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  MCANV71* pComp = (MCANV71*)_pComp;
  if ((address & ~MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  uintptr_t CurrentAddr = ((uintptr_t)MCANV71_RAM_START_ADDRESS & MCAN_BASE_ADDRESS_MASK) + address;
  if ((CurrentAddr + count) >= MCANV71_MAX_RAM_ADDRESS) return ERR__OUT_OF_MEMORY;
  for (int_fast16_t z = 0; z < count; z += sizeof(uint32_t))
  {
    if (MCAN_IS_CHANGE_ENDIANNESS(&(pComp->_MCAN)))
    {
      *((uint8_t*)(CurrentAddr + 0)) = data[3];
      *((uint8_t*)(CurrentAddr + 1)) = data[2];
      *((uint8_t*)(CurrentAddr + 2)) = data[1];
      *((uint8_t*)(CurrentAddr + 3)) = data[0];
    }
    else
    {
      *((uint8_t*)(CurrentAddr + 0)) = data[0];
      *((uint8_t*)(CurrentAddr + 1)) = data[1];
      *((uint8_t*)(CurrentAddr + 2)) = data[2];
      *((uint8_t*)(CurrentAddr + 3)) = data[3];
    }
    CurrentAddr += sizeof(uint32_t);
    data        += sizeof(uint32_t);
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------