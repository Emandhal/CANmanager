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





//=============================================================================
// Prototypes for private functions
//=============================================================================
/*! @brief Configure a FIFO of the MCAN peripheral
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in/out] *startAddr Is the address inside the RAM allocation where the elements configuration will start (need 32-bits alignment). The address will be updated inside the RAM
 * @param[in] *confTxBuffers Is Tx buffer configuration to set
 * @param[in] *confTxFIFOTXQ Is Tx buffer configuration to set
 * @return Returns an #eERRORRESULT value enum
 */
static eERRORRESULT __MCANV71_ConfigureTxFIFObuffers(MCANV71 *pComp, uint32_t* const startAddr, MCAN_FIFObuff* const confTxBuffers, MCAN_FIFObuff* const confTxFIFOTXQ);
/*! @brief Configure a FIFO list of the MCAN peripheral
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in/out] *startAddr Is the address inside the RAM allocation where the elements configuration will start (need 32-bits alignment). The address will be updated inside the RAM
 * @param[in] *listFIFO Is Tx/Rx Buffers/FIFO/TXQ/TEF configuration list to set
 * @param[in] count Is the count of elements in the listFIFO array
 * @return Returns an #eERRORRESULT value enum
 */
static eERRORRESULT __MCANV71_ConfigureFIFOList(MCANV71 *pComp, uint32_t* const startAddr, const MCAN_FIFObuff* const listFIFO, size_t count);
//-----------------------------------------------------------------------------
/*! @brief Configure SID and EID filters MRAM allocation of the MCAN peripheral
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in/out] *startAddr Is the address inside the RAM allocation where the elements configuration will start (need 32-bits alignment). The address will be updated inside the RAM
 * @param[in] sidElementsCount Is the count of SID filters elements to allocate
 * @param[in] eidElementsCount Is the count of EID filters elements to allocate
 * @return Returns an #eERRORRESULT value enum
 */
static eERRORRESULT __MCANV71_ConfigureFiltersMRAM(MCANV71 *pComp, uint32_t* const startAddr, uint8_t sidElementsCount, uint8_t eidElementsCount);
//-----------------------------------------------------------------------------
/*! @brief Counting 1-bits in a uint32
 * @param[in] val Is the value to process
 * @return Returns the count of 1's in val
 */
static uint32_t __CalcBitCount(uint32_t val);
//-----------------------------------------------------------------------------
#define MCAN_TIME_DIFF(begin,end)  ( ((end) >= (begin)) ? ((end) - (begin)) : (UINT32_MAX - ((begin) - (end) - 1)) ) // Works only if time difference is strictly inferior to (UINT32_MAX/2) and call often
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
//=============================================================================
// [WEAK] Configure the MCAN peripheral clock
//=============================================================================
eERRORRESULT MCANV71_ConfigurePeripheralClocks(MCANV71 *pComp, uint32_t* const peripheralClock)
{ // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (peripheralClock == NULL)) return ERR__PARAMETER_ERROR;
#endif
  *peripheralClock = 0; // Set a wrong peripheral clock
  return ERR_NONE;
}


//=============================================================================
// [WEAK] Configure the MCAN DMA base address. Usually the 16-bit MSB of the RAM address
//=============================================================================
eERRORRESULT MCANV71_ConfigureMCANbaseAddress(MCANV71 *pComp)
{
  return ERR_NONE;
  // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
}
#endif
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// MCAN on SAMV71 peripheral initialization
//=============================================================================
eERRORRESULT Init_MCANV71(MCANV71 *pComp, const MCANV71_Config* const pConf, const MCAN_FIFObuff* const listFIFO, size_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (pConf == NULL)) return ERR__PARAMETER_ERROR;
//  if (pComp->fnSPI_Init == NULL) return ERR__PARAMETER_ERROR;
//  if (pComp->fnGetCurrentms  == NULL) return ERR__PARAMETER_ERROR;
#endif
  const uint32_t StartAddress = MCAN_RAM_START_ADDRESS;
  eERRORRESULT Error;
  pComp->InternalConfig = 0;

  //--- Configure peripheral clock ---------------------------
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  uint32_t PeripheralClock = pConf->MainFreq;
  if (PeripheralClock == 0)                                                                                  // If peripheral configuration is not configured (= 0)
  {
    Error = MCANV71_ConfigurePeripheralClocks(pComp, &PeripheralClock);                                      //** This is microcontroller specific. It is called to enable peripheral clock and get the peripheral clock
    if (Error != ERR_NONE) return Error;                                                                     // If there is an error while calling MCANV71_ConfigurePeripheralClocks() then return the error
    if (PeripheralClock < MCAN_PERIPHERAL_CLK_MIN) return ERR__FREQUENCY_ERROR;
    if (PeripheralClock > MCAN_PERIPHERAL_CLK_MAX) return ERR__FREQUENCY_ERROR;
  }
#endif

  //--- Configure SPI Interface -----------------------------
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)

#endif

  //--- Reset -----------------------------------------------
  pComp->InternalConfig = MCAN_DEV_PS_SET(MCAN_DEVICE_SLEEP_NOT_CONFIGURED);                                 // Device is in normal power state, sleep is not yet configured

  //--- Test SPI connection ---------------------------------

  //--- Check endianness ------------------------------------
  uint32_t RegENDNval = 0;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  RegENDNval = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_ENDN;                                             // Read value of the ENDN register
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_ENDN, &RegENDNval);                                               // Read value of the ENDN register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  if (MCAN_ENDN_IS_CORRECT_ENDIANNESS(RegENDNval)) return ERR__BAD_ENDIANNESS;                               // Check the endianness. If fail here, set the MCAN_SWAP_ENDIANNESS define

  //--- Configure component clock ---------------------------


  //--- Configure CRC Interrupts ----------------------------
  //--- Test SPI connection and RAM Test --------------------
  //--- Configure RAM ECC -----------------------------------

  //--- Watchdog RAM configuration --------------------------
  Error = MCANV71_ConfigureWatchdogRAM(pComp, pConf->MessageRAMwatchdogConf);
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureWatchdogRAM() then return the error

  //--- RAM configuration ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Error = MCANV71_ConfigureMCANbaseAddress(pComp);                                                           //** This is microcontroller specific. Some allows the configuration of the 16-bit MSB of the MCAN DMA base address
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureMCANbaseAddress() then return the error
#endif
  uint32_t CurrAddress = StartAddress;
  Error = __MCANV71_ConfigureFiltersMRAM(pComp, &CurrAddress, pConf->SIDelementsCount, pConf->EIDelementsCount);
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling __MCANV71_ConfigureFiltersMRAM() then return the error
  Error = __MCANV71_ConfigureFIFOList(pComp, &CurrAddress, listFIFO, count);
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling __MCANV71_ConfigureFIFOList() then return the error
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  // Tricky verification: The RAM allocation can have its address at a position where the MCAN can misplace messages
  // Example: pComp->RAMallocation address is 0x1E000 and pComp->RAMsize is 15kB, the configuration is in a way that *startAddr have its 16-bit high address that is higher like 0x1E000+2F00 = 0x20F00,
  //          if the MCAN peripheral need to access the 0x20000 address, it will consider the address as 0x10000 because of the 16-bit high address that is stored fixed.
  if (((StartAddress & MCAN_BASE_ADDRESS_MASK) ^ ((CurrAddress - 1u) & MCAN_BASE_ADDRESS_MASK)) > 0) return ERR__BAD_ADDRESS;
#endif

  //--- Initialize RAM if configured ------------------------
  if ((pComp->DriverConfig & MCAN_DRIVER_INIT_SET_RAM_AT_0) > 0)                                             // If there is a DRIVER_INIT_SET_RAM_AT_0 flag then
  {
    for (uint32_t zAddr = StartAddress; zAddr < CurrAddress; zAddr += sizeof(uint32_t))
    {
      Error = MCANV71_WriteRAM(pComp, zAddr, 0x00, 1);                                                       // Write 0 on all used space
      if (Error != ERR_NONE) return Error;                                                                   // If there is an error while calling MCANV71_ConfigureCANController() then return the error
    }
  }

  //--- CAN configuration -----------------------------------
  Error = MCANV71_ConfigureCANController(pComp, pConf->ControlFlags);                                        // Set the CAN configuration
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureCANController() then return the error
  Error = MCANV71_SetEIDrangeFilterMask(pComp, pConf->ExtendedIDrangeMask);                                  // Set the MCAN Extended ID AND Mask
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_SetEIDrangeFilterMask() then return the error

  //--- Initialize Int lines, pins, and/or GPIOs ------------
  Error = MCANV71_ConfigureINTlines(pComp, pConf->EnableLineINT0, pConf->EnableLineINT1);                    // Configure INT lines
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureINTlines() then return the error
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)

#endif

  //--- Set Nominal and Data bitrate ------------------------
  CAN_BitTimeConfig* ConfBitTime;
#if defined(SJA1000_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_BitTimeConfig BitTimeConfig;
  ConfBitTime = &BitTimeConfig;
  Error = MCANV71_CalculateBitTimeConfiguration(pConf->MainFreq, pConf->BusConfig, ConfBitTime);             // Calculate Bit Time
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_CalculateBitTimeConfiguration() then return the error
  ConfBitTime->Stats = pConf->BitTimeStats;
#else
  ConfBitTime = &pConf->BitTimeConfig;
#endif
  Error = MCANV71_SetBitTimeConfiguration(pComp, ConfBitTime);                                               // Set Bit Time configuration to registers
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_SetBitTimeConfiguration() then return the error

  //--- System interrupt enable -----------------------------
  uint32_t SysInterruptFlags;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  SysInterruptFlags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IE;                                        // Read FIFOs/Buffers/TEF interrupt flags already configured by __MCANV71_ConfigureFIFOList()
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_IE, &SysInterruptFlags);                                          // Read FIFOs/Buffers/TEF interrupt flags already configured by __MCANV71_ConfigureFIFOList() of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  SysInterruptFlags &= MCAN_INT_FIFO_BUFFER_TEF_FLAGS_MASK;                                                  // Clear all flags that are not FIFOs/Buffers/TEF flags
  SysInterruptFlags |= (uint32_t)pConf->SysInterruptFlags;                                                   // Merge all interrupts flags
  Error = MCANV71_ConfigureInterrupt(pComp, pConf->SysInterruptFlags, pConf->SysIntLineSelect);              // Configure interrupts and interrupts Line Select
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureInterrupt() then return the error
  return MCANV71_ConfigureTxBufferInterrupts(pComp, pConf->BufferTransmitInt, pConf->BufferCancelFinishInt); // Configure Tx buffers interrupts
}





//**********************************************************************************************************************************************************
//=============================================================================
// Get actual peripheral of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetMCANcoreID(MCANV71 *pComp, uint8_t* const mcanCoreId, uint8_t* const mcanStep, uint8_t* const mcanSubStep, uint8_t* const mcanYear, uint8_t* const mcanMonth, uint8_t* const mcanDay)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif

  MCAN_CREL_Register RegValue;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  RegValue.CREL = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CREL;             // Read value of the CREL register
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_CREL, &RegValue.CREL);  // Read value of the CREL register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  if (mcanCoreId  != NULL) *mcanCoreId  = MCAN_CREL_REL_GET(RegValue.CREL);     // Get Core Release
  if (mcanStep    != NULL) *mcanStep    = MCAN_CREL_STEP_GET(RegValue.CREL);    // Get Step of Core Release
  if (mcanSubStep != NULL) *mcanSubStep = MCAN_CREL_SUBSTEP_GET(RegValue.CREL); // Get Sub-step of Core Release
  if (mcanYear    != NULL) *mcanYear    = MCAN_CREL_YEAR_GET(RegValue.CREL);    // Get Timestamp Year
  if (mcanMonth   != NULL) *mcanMonth   = MCAN_CREL_MON_GET(RegValue.CREL);     // Get Timestamp Month
  if (mcanDay     != NULL) *mcanDay     = MCAN_CREL_DAY_GET(RegValue.CREL);     // Get Timestamp Day
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
//=============================================================================
// Read from a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReadREG32(MCANV71 *pComp, const uint32_t address, uint32_t* data)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif

  return ERR_NONE;
}


//=============================================================================
// Write to a 32-bits register of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_WriteREG32(MCANV71 *pComp, const uint32_t address, const uint32_t data)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif

  return ERR_NONE;
}
#endif

//-----------------------------------------------------------------------------



#ifdef MCAN_INTERNAL_CAN_CONTROLLER
//=============================================================================
// Read from allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReadRAM(MCANV71 *pComp, const uint32_t address, uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((address & MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  if (address >= pComp->RAMsize) return ERR__PARAMETER_ERROR;
  uintptr_t Address = ((uintptr_t)(pComp->RAMallocation) & MCAN_BASE_ADDRESS_MASK) + address;
  for (int_fast16_t z = count; --z >= 0;)
  {
#ifdef MCAN_SWAP_ENDIANNESS
    data[0] = *((uint8_t*)Address + 3);
    data[1] = *((uint8_t*)Address + 2);
    data[2] = *((uint8_t*)Address + 1);
    data[3] = *((uint8_t*)Address + 0);
#else
    data[0] = *((uint8_t*)Address + 0);
    data[1] = *((uint8_t*)Address + 1);
    data[2] = *((uint8_t*)Address + 2);
    data[3] = *((uint8_t*)Address + 3);
#endif
    Address += sizeof(uint32_t);
    data    += sizeof(uint32_t);
  }
  return ERR_NONE;
}


//=============================================================================
// Write to allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_WriteRAM(MCANV71 *pComp, const uint32_t address, const uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((address & MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses
  if (address >= pComp->RAMsize) return ERR__PARAMETER_ERROR;
  uintptr_t Address = ((uintptr_t)(pComp->RAMallocation) & MCAN_BASE_ADDRESS_MASK) + address;
  for (int_fast16_t z = count; --z >= 0;)
  {
#ifdef MCAN_SWAP_ENDIANNESS
    *((uint8_t*)Address + 0) = data[3];
    *((uint8_t*)Address + 1) = data[2];
    *((uint8_t*)Address + 2) = data[1];
    *((uint8_t*)Address + 3) = data[0];
#else
    *((uint8_t*)Address + 0) = data[0];
    *((uint8_t*)Address + 1) = data[1];
    *((uint8_t*)Address + 2) = data[2];
    *((uint8_t*)Address + 3) = data[3];
#endif
    Address += sizeof(uint32_t);
    data    += sizeof(uint32_t);
  }
  return ERR_NONE;
}

#else // MCAN_EXTERNAL_CAN_CONTROLLER

//=============================================================================
// Read from allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReadRAM(MCANV71 *pComp, const uint32_t address, uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((address & MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses

#ifdef MCAN_SWAP_ENDIANNESS
#endif


  return ERR_NONE;
}


//=============================================================================
// Write to allocated RAM data of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_WriteRAM(MCANV71 *pComp, const uint32_t address, const uint8_t* data, const uint16_t count)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (data == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((address & MCAN_ADDRESS32_ALIGN_MASK) > 0) return ERR__ADDRESS_ALIGNMENT; // Only 32-bits aligned addresses

#ifdef MCAN_SWAP_ENDIANNESS
    //MCAN_SWAP32();
#else

#endif

  return ERR_NONE;
}

#endif





//**********************************************************************************************************************************************************
//=============================================================================
// Transmit a message object (with data) to the FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_TransmitMessageObject(MCANV71 *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize, eMCAN_FIFObuffer toFIFObuff, uint8_t index)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((toFIFObuff != MCAN_TX_BUFFER) && (toFIFObuff != MCAN_TXQ_FIFO)) return ERR__PARAMETER_ERROR;
  eERRORRESULT Error;

  //--- Get address where to write the frame ---
  MCAN_TXBC_Register ConfReg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ConfReg.TXBC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBC;                    // Read Tx Buffer Configuration internal register
#elif defined(MCAN_USE_CACHE)
  ConfReg.TXBC = pComp->__RegCache[MCAN_CACHE_TXBC];                                  // Read Tx Buffer Configuration in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_TXBC, &ConfReg.TXBC);                      // Read Tx Buffer Configuration register of an external device
  if (Error != ERR_NONE) return Error;
#endif
  uint32_t NextAddress = MCAN_TXBC_TX_BUFFERS_SA_GET(ConfReg.TXBC);                   // Start address of the Tx Buffer/FIFO/TXQ
  MCAN_TXESC_Register SizeReg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  SizeReg.TXESC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXESC;                  // Read Tx Buffer Element Size Configuration internal register
#elif defined(MCAN_USE_CACHE)
  SizeReg.TXESC = pComp->__RegCache[MCAN_CACHE_TXESC];                                // Read Tx Buffer Element Size Configuration in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_TXESC, &SizeReg.TXESC);                    // Read Tx Buffer Element Size Configuration register of an external device
  if (Error != ERR_NONE) return Error;
#endif
  if (toFIFObuff == MCAN_TX_BUFFER)
  {
    if (index > MCAN_TXBC_TX_BUFFER_SIZE_GET(ConfReg.TXBC)) return ERR__OUT_OF_RANGE; // If Tx buffer and index > configuration, then return an error
  }
  else
  {
    MCAN_TXFQS_Register Status;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    Status.TXFQS = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXFQS;                 // Read FIFO/Buffer/TXQ status
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_TXFQS, Status.TXFQS);                    // Read FIFO/Buffer/TXQ status of an external controller
    if (Error != ERR_NONE) return Error;
#endif
    if ((Status.TXFQS & MCAN_TXFQS_TX_FIFO_QUEUE_FULL) > 0) return ERR__BUFFER_FULL;
    index = MCAN_TXFQS_TX_FIFO_PUT_INDEX_GET(Status.TXFQS);                           // Get put index
  }
  NextAddress += (MCANV71_PayloadToByte(MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_GET(SizeReg.TXESC)) * index);

  //--- Write data to RAM ---
  Error = MCANV71_WriteRAM(pComp, NextAddress, &messageObjectToSend[0], objectSize);  // Write data to RAM address. Size of uint32_t object
  if (Error != ERR_NONE) return Error;                                                // If there is an error while calling MCANV71_WriteRAM() then return the error

  //--- Add transmit request ---
  return MCANV71_SetTxBufferAddRequest(pComp, (1u << index));
}


//=============================================================================
// Transmit a message to a FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_TransmitMessage(MCANV71 *pComp, const CAN_CANMessage* const messageToSend, eMCAN_FIFObuffer toFIFObuff, uint8_t index)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((toFIFObuff != MCAN_TX_BUFFER) && (toFIFObuff != MCAN_TXQ_FIFO)) return ERR__PARAMETER_ERROR;
  uint8_t Buffer[MCAN_CAN_TX_MESSAGE_SIZE_MAX];
  MCAN_CAN_TxMessage* Message = (MCAN_CAN_TxMessage*)Buffer;                              // The first 8 bytes represent the MCAN_CAN_TxMessage struct

  //--- Fill message ID (T0) ---
  const bool CANFDframe = ((messageToSend->ControlFlags & CAN_CANFD_FRAME) > 0);
  Message->T0.T0 = 0;                                                                     // Initialize message Identifier to 0
  if ((messageToSend->ControlFlags & CAN_TRANSMIT_ERROR_PASSIVE     ) > 0) Message->T0.T0 |= MCAN_CAN_MSGT0_ERROR_STATUS_IND;  // ESI
  if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) > 0) Message->T0.T0 |= MCAN_CAN_MSGT0_REMOTE_FRAME;      // RTR
  if ((messageToSend->ControlFlags & CAN_EXTENDED_MESSAGE_ID        ) > 0)                                                     // XTD/IDE
  {
    Message->T0.T0 |= MCAN_CAN_MSGT0_EXTENDED_ID;
    Message->T0.T0 |= MCAN_CAN_MSGT0_ID_SET(messageToSend->MessageID);                    // Set EID in bits 28:0 of ID
  }
  else Message->T0.T0 |= MCAN_CAN_MSGT0_ID_SET(messageToSend->MessageID << CAN_EID_Size); // Set SID in bits 28:18 of ID

  //--- Fill message controls (T1) ---
  Message->T1.T1 = MCAN_CAN_MSGT1_DLC_SET(messageToSend->DLC);                            // Initialize message Controls to with DLC
  if (CANFDframe                                                 )      Message->T1.T1 |= MCAN_CAN_MSGT1_FD_FORMAT;             // FDF
  if ((messageToSend->ControlFlags & CAN_SWITCH_BITRATE          ) > 0) Message->T1.T1 |= MCAN_CAN_MSGT1_BITRATE_SWITCH;        // BRS
  if ((messageToSend->ControlFlags & CAN_STORE_TX_EVENT          ) > 0) Message->T1.T1 |= MCAN_CAN_MSGT1_STORE_TX_EVENT;        // EFC
  if ((pComp->InternalConfig & MCAN_16BIT_MM_ENABLED             ) > 0)
  {
    if ((messageToSend->ControlFlags & CAN_USE_EXTERNAL_TIMESTAMP) > 0) Message->T1.T1 |= MCAN_CAN_MSGT1_USE_TIMESTAMP_CAPTURE; // TSCE
    Message->T1.T1 |= MCAN_CAN_MSGT1_MM_SET(messageToSend->MessageMarker);
  }
  else Message->T1.T1 |= MCAN_CAN_MSGT1_MMl_SET(messageToSend->MessageMarker);

  //--- Fill payload data ---
  if (((eMCAN_DataLength)messageToSend->DLC != MCAN_DLC_0BYTE) && (messageToSend->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
  uint8_t BytesDLC = MCANV71_DLCToByte(messageToSend->DLC, CANFDframe);
  if (messageToSend->PayloadData != NULL)
  {
    uint8_t* pBuff = &Buffer[sizeof(MCAN_CAN_TxMessage)];                // Next bytes of the Buffer is for payload
    uint8_t* pData = &messageToSend->PayloadData[0];                     // Select the first byte of payload data
    uint8_t BytesToCopy = BytesDLC;                                      // Get how many byte in the payload data will be copied
    while (BytesToCopy-- > 0) *pBuff++ = *pData++;                       // Copy data
    if ((BytesDLC & 0x3) > 0)                                            // Not modulo 4?
      for (uint8_t z = 0; z < (4 - (BytesDLC & 0x3)); ++z) *pBuff++ = 0; // Fill with 0
  }

  //--- Send data ---
  uint8_t BytesToSend = (sizeof(MCAN_CAN_TxMessage) + BytesDLC);
  if ((BytesToSend & 0x3) != 0) BytesToSend = (BytesToSend & 0xFC) + 4;  // Adjust to the upper modulo 4 bytes (mandatory for RAM access)
  return MCANV71_TransmitMessageObject(pComp, &Buffer[0], BytesToSend, toFIFObuff, index);
}





//=============================================================================
// Receive a message object (with data) from a FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReceiveMessageObject(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize, eMCAN_FIFObuffer fromFIFObuff, uint8_t index)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((fromFIFObuff == MCAN_TX_BUFFER) || (fromFIFObuff == MCAN_TXQ_FIFO) || (fromFIFObuff == MCAN_TEF)) return ERR__PARAMETER_ERROR;
  uint32_t NextAddress = 0, FIFObuffSize = 0, Status = 0;
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  uint32_t RegAddr = 0;
#endif
  eERRORRESULT Error;

  //--- Get FIFO/Buffer paypload size ---
  MCAN_RXESC_Register SizeReg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  SizeReg.RXESC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXESC;             // Read internal register
#elif defined(MCAN_USE_CACHE)
  SizeReg.RXESC = pComp->__RegCache[MCAN_CACHE_RXESC];                           // Read in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_RXESC, &SizeReg.RXESC);               // Read register of an external device
  if (Error != ERR_NONE) return Error;
#endif

  //--- Get address where to read the frame ---
  uint32_t ConfReg;
  switch (fromFIFObuff)
  {
    default:
    case MCAN_RX_FIFO0 :
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
      ConfReg = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0C;               // Read FIFO0 configuration internal register
      Status  = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0S;               // Read FIFO0 status
#elif defined(MCAN_USE_CACHE)
      ConfReg = pComp->__RegCache[MCAN_CACHE_RXF0C];                             // Read FIFO0 configuration in cache
#else
      Error = MCANV71_ReadREG32(pComp, RegMCAN_RXF0C, &ConfReg);                 // Read FIFO0 configuration register of an external device
      if (Error != ERR_NONE) return Error;
      RegAddr = (uint32_t)RegMCAN_RXF0S;                                         // Prepare status address of Rx FIFO0
#endif
      NextAddress = MCAN_RXF0C_RX_FIFO0_SA_GET(ConfReg);                         // Get Rx FIFO0 Start Address
      FIFObuffSize = MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_GET(SizeReg.RXESC);     // Get Rx FIFO0 size
      break;

    case MCAN_RX_FIFO1 :
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
      ConfReg = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1C;               // Read FIFO1 configuration internal register
      Status  = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1S;               // Read FIFO1 status
#elif defined(MCAN_USE_CACHE)
      ConfReg = pComp->__RegCache[MCAN_CACHE_RXF1C];                             // Read FIFO1 configuration in cache
#else
      Error = MCANV71_ReadREG32(pComp, RegMCAN_RXF1C, &ConfReg);                 // Read FIFO1 configuration register of an external device
      if (Error != ERR_NONE) return Error;
      RegAddr = (uint32_t)RegMCAN_RXF1S;                                         // Prepare status address of Rx FIFO1
#endif
      NextAddress = MCAN_RXF1C_RX_FIFO1_SA_GET(ConfReg);                         // Get Rx FIFO1 Start Address
      FIFObuffSize = MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_GET(SizeReg.RXESC);     // Get Rx FIFO1 size
      break;

    case MCAN_RX_BUFFER:
      if (index > MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE; // If  index > configuration, then return an error
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
      ConfReg = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXBC;                // Read internal register
#elif defined(MCAN_USE_CACHE)
      ConfReg = pComp->__RegCache[MCAN_CACHE_RXBC];                              // Read in cache
#else
      Error = MCANV71_ReadREG32(pComp, RegMCAN_RXBC, &ConfReg);                  // Read register of an external device
      if (Error != ERR_NONE) return Error;
#endif
      NextAddress = MCAN_RXBC_RX_BUFFER_SA_GET(ConfReg);                         // Get Rx Buffer Start Address
      FIFObuffSize = MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_GET(SizeReg.RXESC);    // Get Rx Buffer size
      break;
  }

  //--- Read FIFO status ---
  if (fromFIFObuff != MCAN_RX_BUFFER)
  {
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
    Error = MCANV71_ReadREG32(pComp, RegAddr, Status);                           // Read FIFO status of an external controller
    if (Error != ERR_NONE) return Error;
#endif
    if (MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_GET(Status) == 0) return ERR__NO_DATA_AVAILABLE;
    index = MCAN_RXF0S_RX_FIFO0_GET_INDEX_GET(Status);                           // Get get index
  }
  NextAddress += (MCANV71_PayloadToByte(FIFObuffSize) * index);

  //--- Write data to RAM ---
  Error = MCANV71_ReadRAM(pComp, NextAddress, &messageObjectGet[0], objectSize); // Write data to RAM address
  if (Error != ERR_NONE) return Error;                                           // If there is an error while calling MCANV71_WriteRAM() then return the error

  //--- Acknowledge FIFO/Buffer ---
  if (fromFIFObuff != MCAN_RX_BUFFER)
       Error = MCANV71_AcknowledgeFIFO(pComp, fromFIFObuff, index);              // Acknowledge the frame
  else Error = MCANV71_ClearRxBufferNewDataFlag(pComp, (1u << index));           // Clear/update the frame
  return Error;
}


//=============================================================================
// Receive a message object (with data) from TEF of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReceiveMessageObjectFromTEF(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Get TEF configuration ---
  MCAN_TXEFC_Register ConfReg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ConfReg.TXEFC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFC;              // Read internal register
#elif defined(MCAN_USE_CACHE)
  ConfReg.TXEFC = pComp->__RegCache[MCAN_CACHE_TXEFC];                            // Read in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_TXEFC, &ConfReg.TXEFC);                // Read register of an external device
  if (Error != ERR_NONE) return Error;
#endif

  //--- Get address where to read the frame ---
  uint32_t NextAddress = MCAN_TXEFC_EVENT_FIFO_SA_GET(ConfReg.TXEFC);             // Start address of the TEF FIFO
  MCAN_TXEFS_Register Status;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Status.TXEFS = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFS;               // Read FIFO/Buffer/TXQ status
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_TXEFS, &Status.TXEFS);                 // Read FIFO/Buffer/TXQ status of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  if ((Status.TXEFS & MCAN_TXFQS_EVENT_FIFO_FULL) > 0) return ERR__BUFFER_FULL;
  uint32_t Index = MCAN_TXFQS_EVENT_FIFO_GET_INDEX_GET(Status.TXEFS);             // Get put index
  NextAddress += (MCAN_CAN_TX_EVENTOBJECT_SIZE * Index);                          // Get address following GetIndex

  //--- Write data to RAM ---
  Error = MCANV71_WriteRAM(pComp, NextAddress, &messageObjectGet[0], objectSize); // Write data to RAM address
  if (Error != ERR_NONE) return Error;                                            // If there is an error while calling MCANV71_WriteRAM() then return the error

  //--- Acknowledge FIFO ---
  return MCANV71_AcknowledgeTEF(pComp, (1u << Index));                            // Acknowledge the frame
}


//=============================================================================
// Receive a message from a FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ReceiveMessage(MCANV71 *pComp, CAN_CANMessage* const messageGet, eMCAN_PayloadSize payloadSize, uint32_t* const timeStamp, uint8_t* const filterHitIdx, eMCAN_FIFObuffer fromFIFObuff, uint8_t index)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((fromFIFObuff == MCAN_TX_BUFFER) || (fromFIFObuff == MCAN_TXQ_FIFO)) return ERR__PARAMETER_ERROR;
  uint8_t Buffer[MCAN_CAN_RX_MESSAGE_SIZE_MAX];
  MCAN_CAN_RxMessage* Message = (MCAN_CAN_RxMessage*)Buffer;                                // The first 8 bytes represent the MCAN_CAN_RxMessage struct
  eERRORRESULT Error;

  //--- Get data ---
  uint8_t BytesPayload = MCANV71_PayloadToByte(payloadSize);
  uint8_t BytesToGet = (sizeof(MCAN_CAN_RxMessage) + BytesPayload);
  if ((BytesToGet & 0x3) != 0) BytesToGet = (BytesToGet & 0xFC) + 4;                        // Adjust to the upper modulo 4 bytes (mandatory for RAM access)
  Error = MCANV71_ReceiveMessageObject(pComp, &Buffer[0], BytesToGet, fromFIFObuff, index); // Read bytes from RAM
  if (Error != ERR_NONE) return Error;                                                      // If there is an error while calling MCANV71_ReceiveMessageObject() then return the error

  //--- Extract message ID (R0) ---
  if ((Message->R0.R0 & MCAN_CAN_MSGR0_ERROR_STATUS_IND) > 0) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_TRANSMIT_ERROR_PASSIVE     ); // ESI
  if ((Message->R0.R0 & MCAN_CAN_MSGT0_REMOTE_FRAME    ) > 0) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_REMOTE_TRANSMISSION_REQUEST); // RTR
  if ((Message->R0.R0 & MCAN_CAN_MSGT0_EXTENDED_ID     ) > 0)                                                                                                                   // XTD/IDE
  {
    messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_EXTENDED_MESSAGE_ID);
    messageGet->MessageID = MCAN_CAN_MSGR0_ID_GET(Message->R0.R0);                    // Get EID in bits 28:0 of ID
  }
  else messageGet->MessageID = MCAN_CAN_MSGR0_ID_GET(Message->R0.R0) >> CAN_EID_Size; // Get SID in bits 28:18 of ID

  //--- Extract message controls (R1) ---
  messageGet->ControlFlags  = CAN_NO_MESSAGE_CTRL_FLAGS;
  messageGet->MessageMarker = 0u;
  const bool CANFDframe = (Message->R1B.R1B & MCAN_CAN_MSGR1A_FD_FORMAT) > 0;
  if ( CANFDframe                                             )      messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_CANFD_FRAME          ); // FDF
  if ((Message->R1B.R1B & MCAN_CAN_MSGR1A_BITRATE_SWITCH      ) > 0) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_SWITCH_BITRATE       ); // BRS
  messageGet->DLC = MCAN_CAN_MSGR1A_DLC_GET(Message->R1B.R1B);                                                                                                                   // DLC
  if (fromFIFObuff != MCAN_TEF)
  {
    if ((Message->R1B.R1B & MCAN_CAN_MSGR1A_NON_MATCHING_FRAME) > 0) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_RX_NON_MATCHING_FRAME); // ANMF
    if (filterHitIdx != NULL) *filterHitIdx = MCAN_CAN_MSGR1A_FIDX_GET(Message->R1B.R1B);                                                                                        // FIDX
  }
  else
  {
    if ((pComp->InternalConfig & MCAN_16BIT_MM_ENABLED) > 0)
         messageGet->MessageMarker = MCAN_CAN_MSGE1B_MM_GET(Message->R1B.R1B);
    else messageGet->MessageMarker = MCAN_CAN_MSGE1B_MMl_GET(Message->R1B.R1B);
    if (MCAN_CAN_MSGE1A_ET_GET(Message->R1B.R1B) == MCAN_TX_SPITE_CANCELLATION) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_TEF_CANCELLED_FRAME); // ET
  }

  //--- Extract TimeStamp ---
  if (timeStamp != NULL)
  {
    if ((pComp->InternalConfig & MCAN_16BIT_MM_ENABLED      ) > 0)
    {
      if ((Message->R1B.R1B & MCAN_CAN_MSGR1B_TIMESTAMP_CAPTURED) > 0) messageGet->ControlFlags = (setCAN_MessageCtrlFlags)(messageGet->ControlFlags + CAN_USE_EXTERNAL_TIMESTAMP); // TSC
      *timeStamp = MCAN_CAN_MSGR1B_RXTSP_GET(Message->R1B.R1B);                                                                                                                     // RXTSP
    }
    else *timeStamp = MCAN_CAN_MSGR1A_RXTS_GET(Message->R1B.R1B);                                                                                                                   // RXTS
  }

  //--- Extract payload data ---
  uint8_t* pBuff = &Buffer[sizeof(MCAN_CAN_RxMessage)];                  // Next bytes of the Buffer is for payload
  if (fromFIFObuff != MCAN_TEF)
  {
    if (((eMCAN_DataLength)messageGet->DLC != MCAN_DLC_0BYTE) && (messageGet->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
    if (messageGet->PayloadData != NULL)
    {
      uint8_t* pData = &messageGet->PayloadData[0];                      // Select the first byte of payload data
      uint8_t BytesDLC = MCANV71_DLCToByte(messageGet->DLC, CANFDframe); // Get how many byte need to be extract from the message to correspond to its DLC
      if (BytesPayload < BytesDLC) BytesDLC = BytesPayload;              // Get the least between BytesPayload and BytesDLC
      while (BytesDLC-- > 0) *pData++ = *pBuff++;                        // Copy data
    }
  }
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure INT Lines of MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureINTlines(MCANV71 *pComp, bool enableLineINT0, bool enableLineINT1)
{
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
  MCAN_ILE_Register RegConf;
  RegConf.ILE = MCAN_ILE_EINT0_DIS | MCAN_ILE_EINT1_DIS;
  if (enableLineINT0) RegConf.ILE |= MCAN_ILE_EINT0_EN;
  if (enableLineINT1) RegConf.ILE |= MCAN_ILE_EINT1_EN;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_ILE = RegConf.ILE;            // Write value of the ILE register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_ILE, RegConf.ILE); // Write value of the ILE register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Calculate Bit Time for CAN2.0 or CAN-FD configuration for the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CANFDbusConfig busConf, struct CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif
  //--- Check values ----------------------------------------
  if (busConf.DesiredNominalBitrate < MCAN_NOMBITRATE_MIN ) return ERR__BAUDRATE_ERROR;
  if (busConf.DesiredNominalBitrate > MCAN_NOMBITRATE_MAX ) return ERR__BAUDRATE_ERROR;
  if (busConf.DesiredDataBitrate != CAN_NO_CANFD)
    if (busConf.DesiredDataBitrate  < MCAN_DATABITRATE_MIN) return ERR__BAUDRATE_ERROR;
  if (busConf.DesiredDataBitrate    > MCAN_DATABITRATE_MAX) return ERR__BAUDRATE_ERROR;

  //--- Declaration -----------------------------------------
  uint32_t ErrorTQ, ErrorNTQ, ErrorDTQ, DTQbits = 0;
  uint32_t BestBRP = MCAN_NBRP_MAX, BestNTQbits = MCAN_NTQBIT_MAX, BestDTQbits = MCAN_DTQBIT_MAX;

  //--- Calculate Nominal & Data Bit Time parameter ---------
  uint32_t MinErrorBR = UINT32_MAX;
  uint32_t BRP = MCAN_NBRP_MAX;                                               // Select the worst BRP value. Here all value from max to min will be tested to get the best tuple of NBRP and DBRP, identical TQ in both phases prevents quantization errors during bit rate switching
  while (--BRP >= MCAN_NBRP_MIN)
  {
    uint32_t NTQbits = periphClk / busConf.DesiredNominalBitrate / BRP;       // Calculate the NTQbits according to BRP and the desired Nominal Bitrate
    if ((NTQbits < MCAN_NTQBIT_MIN) || (NTQbits > MCAN_NTQBIT_MAX)) continue; // This TQbits count is not possible with this BRP, then do the next BRP value
    DTQbits = periphClk / busConf.DesiredDataBitrate / BRP;                   // Calculate the DTQbits according to BRP and the desired Data Bitrate
    if ((DTQbits < MCAN_DTQBIT_MIN) || (DTQbits > MCAN_DTQBIT_MAX)) continue; // This TQbits count is not possible with this BRP, then do the next BRP value

    //--- NTQ & DTQ bits count ---
    ErrorNTQ = (periphClk - (busConf.DesiredNominalBitrate * NTQbits * BRP));                               // Calculate NTQ error
    if (ErrorNTQ == 0) ErrorNTQ = 1;                                                                        // Adjust NTQ error
    ErrorDTQ = (periphClk - (busConf.DesiredDataBitrate * DTQbits * BRP)); if (ErrorDTQ == 0) ErrorDTQ = 1; // Calculate DTQ error
    ErrorTQ = (ErrorNTQ * ErrorDTQ);
    if (ErrorTQ <= MinErrorBR)                                                                              // If better error then
    { MinErrorBR = ErrorTQ; BestBRP = BRP; BestNTQbits = NTQbits; BestDTQbits = DTQbits; }                  // Save best parameters

    //--- NTQ+1 & DTQ bits count ---
    if (NTQbits < MCAN_NTQBIT_MAX)
    {
      ErrorNTQ = ((busConf.DesiredNominalBitrate * (NTQbits+1) * BRP) - periphClk);                           // Calculate NTQ error with NTQbits+1
      if (ErrorNTQ == 0) ErrorNTQ = 1;                                                                        // Adjust NTQ error
      ErrorDTQ = (periphClk - (busConf.DesiredDataBitrate * DTQbits * BRP)); if (ErrorDTQ == 0) ErrorDTQ = 1; // Calculate DTQ error
      ErrorTQ = (ErrorNTQ * ErrorDTQ);
      if (ErrorTQ <= MinErrorBR)                                                                              // If better error then
      { MinErrorBR = ErrorTQ; BestBRP = BRP; BestNTQbits = NTQbits+1; BestDTQbits = DTQbits; }                // Save best parameters
    }

    //--- NTQ+1 & DTQ or DTQ+1 bits count ---
    if (DTQbits < MCAN_DTQBIT_MAX)
    {
      ErrorNTQ = (periphClk - (busConf.DesiredNominalBitrate * NTQbits * BRP));  if (ErrorNTQ == 0) ErrorNTQ = 1;    // Calculate NTQ error
      ErrorDTQ = ((busConf.DesiredDataBitrate * (DTQbits+1) * BRP) - periphClk); if (ErrorDTQ == 0) ErrorDTQ = 1;    // Calculate DTQ error with DTQbits+1
      ErrorTQ = (ErrorNTQ * ErrorDTQ);
      if (ErrorTQ <= MinErrorBR)                                                                                     // If better error then
      { MinErrorBR = ErrorTQ; BestBRP = BRP; BestNTQbits = NTQbits; BestDTQbits = DTQbits+1; }                       // Save best parameters
    }
    if ((NTQbits < MCAN_NTQBIT_MAX) && (DTQbits < MCAN_DTQBIT_MAX))
    {
      ErrorNTQ = ((busConf.DesiredNominalBitrate * (NTQbits+1) * BRP) - periphClk); if (ErrorNTQ == 0) ErrorNTQ = 1; // Calculate NTQ error with NTQbits+1
      ErrorDTQ = ((busConf.DesiredDataBitrate * (DTQbits+1) * BRP) - periphClk);    if (ErrorDTQ == 0) ErrorDTQ = 1; // Calculate DTQ error with DTQbits+1
      ErrorTQ = (ErrorNTQ * ErrorDTQ);
      if (ErrorTQ <= MinErrorBR)                                                                                     // If better error then
      { MinErrorBR = ErrorTQ; BestBRP = BRP; BestNTQbits = NTQbits+1; BestDTQbits = DTQbits+1; }                     // Save best parameters
    }
  }
  if (MinErrorBR == UINT32_MAX) return ERR__BITTIME_ERROR;               // Impossible to find a good BRP

  //--- Calculate Nominal segments --------------------------
  pConf->NBRP = BestBRP - 1;                                             // ** Save the best NBRP in the configuration **
  uint32_t NTSEG2 = (BestNTQbits * (100u - busConf.NominalSamplePoint)); // Calculate the Nominal Sample Point
  NTSEG2 = (NTSEG2 + 50) / 100;                                          // Round Nominal Sample Point
  if (NTSEG2 < MCAN_NTSEG2_MIN) NTSEG2 = MCAN_NTSEG2_MIN;                // Correct NTSEG2 if < 1
  if (NTSEG2 > MCAN_NTSEG2_MAX) NTSEG2 = MCAN_NTSEG2_MAX;                // Correct NTSEG2 if > 128
  pConf->NTSEG2 = NTSEG2 - 1;                                            // ** Save the NTSEG2 in the configuration **
  uint32_t NTSEG1 = BestNTQbits - NTSEG2 - MCAN_NSYNC;                   // NTSEG1  = NTQbits - NTSEG2 - 1 (NSYNC)
  if (NTSEG1 < MCAN_NTSEG1_MIN) NTSEG1 = MCAN_NTSEG1_MIN;                // Correct NTSEG1 if < 1
  if (NTSEG1 > MCAN_NTSEG1_MAX) NTSEG1 = MCAN_NTSEG1_MAX;                // Correct NTSEG1 if > 256
  pConf->NTSEG1 = NTSEG1 - 1;                                            // ** Save the NTSEG1 in the configuration **
  uint32_t NSJW = NTSEG2;                                                // Normally NSJW = NTSEG2, maximizing NSJW lessens the requirement for the oscillator tolerance
  if (NTSEG1 < NTSEG2) NSJW = NTSEG1;                                    // But NSJW = min(NPHSEG1, NPHSEG2)
  if (NSJW < MCAN_NSJW_MIN) NSJW = MCAN_NSJW_MIN;                        // Correct NSJW if < 1
  if (NSJW > MCAN_NSJW_MAX) NSJW = MCAN_NSJW_MAX;                        // Correct NSJW if > 128
  pConf->NSJW = NSJW - 1;                                                // ** Save the NSJW in the configuration **

  //--- Calculate Data segments -----------------------------
  pConf->DBRP = BestBRP - 1;                                          // ** Save the best DBRP in the configuration **
  uint32_t DTSEG2 = (BestDTQbits * (100u - busConf.DataSamplePoint)); // Calculate the Data Sample Point
  DTSEG2 = (DTSEG2 + 50) / 100;                                       // Round Data Sample Point
  if (DTSEG2 < MCAN_NTSEG2_MIN) DTSEG2 = MCAN_NTSEG2_MIN;             // Correct DTSEG2 if < 1
  if (DTSEG2 > MCAN_NTSEG2_MAX) DTSEG2 = MCAN_NTSEG2_MAX;             // Correct DTSEG2 if > 16
  pConf->DTSEG2 = DTSEG2 - 1;                                         // ** Save the DTSEG2 in the configuration **
  uint32_t DTSEG1 = BestDTQbits - DTSEG2 - MCAN_DSYNC;                // DTSEG1  = DTQbits - DTSEG2 - 1 (DSYNC)
  if (DTSEG1 < MCAN_NTSEG1_MIN) DTSEG1 = MCAN_NTSEG1_MIN;             // Correct DTSEG1 if < 1
  if (DTSEG1 > MCAN_NTSEG1_MAX) DTSEG1 = MCAN_NTSEG1_MAX;             // Correct DTSEG1 if > 32
  pConf->DTSEG1 = DTSEG1 - 1;                                         // ** Save the DTSEG1 in the configuration **
  uint32_t DSJW = DTSEG2;                                             // Normally DSJW = DTSEG2, maximizing DSJW lessens the requirement for the oscillator tolerance
  if (DTSEG1 < DTSEG2) DSJW = DTSEG1;                                 // But DSJW = min(DPHSEG1, DPHSEG2)
  if (DSJW < MCAN_DSJW_MIN) DSJW = MCAN_DSJW_MIN;                     // Correct DSJW if < 1
  if (DSJW > MCAN_DSJW_MAX) DSJW = MCAN_DSJW_MAX;                     // Correct DSJW if > 128
  pConf->DSJW = DSJW - 1;                                             // ** Save the DSJW in the configuration **

  //--- Calculate Transmitter Delay Compensation ----------
  if (busConf.DesiredDataBitrate >= 1000000)                      // Enable Automatic TDC for DBR of 1Mbps and Higher
       pConf->TDCmode = CAN_TDC_AUTO_MODE;                        // ** Set Automatic TDC measurement compensations for transmitter delay variations (Enable TCDC)
  else pConf->TDCmode = CAN_TDC_DISABLED;                         // ** Set Manual; Don’t measure, use TDCV + TDCO from register (Disable TCDC)
  const uint32_t SSP = BestBRP * DTSEG1;                          // In order to set the SSP, SSP = TDCO + TDCV. SSP is set to DBRP * (DPRSEG + DPHSEG1) = DBRP * DTSEG1
  uint32_t TDCO = SSP;
  if (TDCO > MCAN_TDCO_MAX) TDCO = MCAN_TDCO_MAX;                 // Correct TDCO if > 63
  pConf->TDCO = TDCO;                                             // ** Save the TDCO in the configuration **
  uint32_t TDCV = SSP - TDCO;                                     // TDCV is the remaining of SSP: TDCV = SSP - TDCO
#if defined(MCAN_TDCV_MIN) && defined(MCAN_TDCV_MAX)
  if (TDCV > MCAN_TDCV_MAX) TDCV = MCAN_TDCV_MAX;                 // Correct TDCV if > 63
#endif
  pConf->TDCV = TDCV;                                             // ** Save the TDCV in the configuration **

  pConf->Valid = true;                                            // ** Set configuration as valid **
  eERRORRESULT Error = ERR_NONE;
  if (pConf->Stats != NULL)
    Error = MCANV71_CalculateBitrateStatistics(periphClk, pConf); // If statistics are necessary, then calculate them
  return Error;                                                   // If there is an error while calling MCAN_CalculateBitrateStatistics() then return the error
}



//=============================================================================
// Calculate Bitrate Statistics of a Bit Time configuration
//=============================================================================
eERRORRESULT MCANV71_CalculateBitrateStatistics(const uint32_t periphClk, CAN_BitTimeConfig *pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
  if (pConf->Stats == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Declaration -----------------------------------------
  uint32_t DTQbits = 0;

  //--- Calculate bus length & Nominal Sample Point ---------
  const uint32_t NTQ = (((pConf->NBRP+1) * 1000000) / (periphClk / 1000));       // Nominal Time Quanta = 1/PERIPHCLK multiply by 1000000000 to get ns
  const uint32_t NPRSEG  = (pConf->NTSEG1+1) - (pConf->NTSEG2+1);                // Here PHSEG2 (NTSEG2) should be equal to PHSEG1 so NPRSEG = NTSEG1 - NTSEG2
  pConf->Stats->MaxBusLength = (uint32_t)(((NTQ * NPRSEG) - (2 * MCAN_tTXDtRXD_MAX)) / (2 * MCAN_tBUS_CONV)); // Formula is (2x(tTXD–RXD + (5*BusLen))/NTQ = NPRSEG => BusLen = ((NTQ*NPRESG)-(2*tTXD))/(2*5) in meter
  const uint32_t NTQbits = (MCAN_NSYNC + (pConf->NTSEG1+1) + (pConf->NTSEG2+1)); // NTQ per bits = NSYNC + NTSEG1 + NTSEG2
  uint32_t SamplePoint = ((MCAN_NSYNC + (pConf->NTSEG1+1)) * 100) / NTQbits;     // Calculate actual nominal sample point
  pConf->Stats->NSamplePoint = (uint32_t)(SamplePoint * 100);                    // ** Save actual Nominal sample point with 2 digits after the decimal point (divide by 100 to get percentage)
  pConf->Stats->NominalBitrate = (periphClk / (pConf->NBRP+1) / NTQbits);        // ** Save actual Nominal Bitrate

  //--- Calculate Data Sample Point -------------------------
  if (pConf->CAN20only == false)
  {
    DTQbits = (MCAN_DSYNC + (pConf->DTSEG1+1) + (pConf->DTSEG2+1));       // DTQ per bits = DSYNC + DTSEG1 + DTSEG2
    SamplePoint = ((MCAN_DSYNC + (pConf->DTSEG1+1)) * 100) / DTQbits;     // Calculate actual data sample point
    pConf->Stats->DSamplePoint = (uint32_t)(SamplePoint * 100.0f);        // ** Save actual Data sample point with 2 digits after the decimal point (divide by 100 to get percentage)
    pConf->Stats->DataBitrate  = (periphClk / (pConf->DBRP+1) / DTQbits); // ** Save actual Data Bitrate
  }
  else
  {
    pConf->Stats->DSamplePoint = 0;                                       // ** Set actual Data sample point
    pConf->Stats->DataBitrate  = 0;                                       // ** Set actual Data Bitrate
  }

  //--- Calculate oscillator tolerance ----------------------
  const uint16_t NPHSEG1     = (pConf->NTSEG1+1) - NPRSEG;                                                                                  // Get NPHSEG1
  const uint16_t MinNPHSEG   = (NPHSEG1 <= (pConf->NTSEG2+1) ? NPHSEG1 : (pConf->NTSEG2+1));                                                // Get min(NPHSEG1, NPHSEG2)
  pConf->Stats->OscTolC1     = (((pConf->NSJW+1) * 10000) / (2 * 10 * NTQbits));                                                            // Condition 1 for the maximum tolerance of the oscillator with 2 digits after the decimal point
  pConf->Stats->OscTolerance = pConf->Stats->OscTolC1;
  pConf->Stats->OscTolC2     = ((MinNPHSEG * 10000) / (2 * (13 * NTQbits - (pConf->NTSEG2+1))));                                            // Condition 2 for the maximum tolerance of the oscillator with 2 digits after the decimal point
  pConf->Stats->OscTolerance = (pConf->Stats->OscTolC2 < pConf->Stats->OscTolerance ? pConf->Stats->OscTolC2 : pConf->Stats->OscTolerance); // Oscillator Tolerance, minimum of conditions 1-5
  if (pConf->CAN20only)
  {
    pConf->Stats->OscTolC3   = 0;
    pConf->Stats->OscTolC4   = 0;
    pConf->Stats->OscTolC5   = 0;
  }
  else
  {
    pConf->Stats->OscTolC3   = (((pConf->DSJW+1) * 10000) / (2 * 10 * DTQbits));                                                                                     // Condition 3 for the maximum tolerance of the oscillator with 2 digits after the decimal point
    pConf->Stats->OscTolerance = (pConf->Stats->OscTolC3 < pConf->Stats->OscTolerance ? pConf->Stats->OscTolC3 : pConf->Stats->OscTolerance);                        // Oscillator Tolerance, minimum of conditions 1-5
    const uint32_t NBRP = (pConf->NBRP + 1), DBRP = (pConf->DBRP + 1);
    pConf->Stats->OscTolC4   = ((MinNPHSEG * 10000) / (2 * ((((6 * DTQbits - (pConf->DTSEG2+1)) * DBRP) / NBRP) + (7 * NTQbits))));                                  // Condition 4 for the maximum tolerance of the oscillator with 2 digits after the decimal point
    pConf->Stats->OscTolerance = (pConf->Stats->OscTolC4 < pConf->Stats->OscTolerance ? pConf->Stats->OscTolC4 : pConf->Stats->OscTolerance);                        // Oscillator Tolerance, minimum of conditions 1-5
    const int32_t NBRP_DBRP = ((NBRP * 10000) / DBRP), MaxBRP = ((NBRP_DBRP - 10000) > 0 ? (NBRP_DBRP - 10000) : 0);                                                 // NBRP/DBRP and max(0,(NBRP/DBRP-1)). The use of 10000 is to set 2 digits on the C5 result
    pConf->Stats->OscTolC5   = ((((pConf->DSJW+1) * 10000) - MaxBRP) / (2 * (((2 * NTQbits - (pConf->NTSEG2+1)) * NBRP) / DBRP + (pConf->DTSEG2+1) + 4 * DTQbits))); // Condition 5 for the maximum tolerance of the oscillator with 2 digits after the decimal point
    pConf->Stats->OscTolerance = (pConf->Stats->OscTolC5 < pConf->Stats->OscTolerance ? pConf->Stats->OscTolC5 : pConf->Stats->OscTolerance);                        // Oscillator Tolerance, minimum of conditions 1-5
  }
  return ERR_NONE;
}



//=============================================================================
// Set Bit Time Configuration to the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_SetBitTimeConfiguration(MCANV71 *pComp, const CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Write Nominal Bit Time configuration ----------------
  MCAN_NBTP_Register Nconfig;
  Nconfig.NBTP = MCAN_NBTP_NBRP_SET(pConf->NBRP) | MCAN_NBTP_NTSEG1_SET(pConf->NTSEG1)    // Set Nominal Bit Time configuration
               | MCAN_NBTP_NTSEG2_SET(pConf->NTSEG2) | MCAN_NBTP_NSJW_SET(pConf->NSJW);
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_NBTP = Nconfig.NBTP;                        // Write configuration to the NBTP register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_NBTP, Nconfig.NBTP);             // Write configuration to the NBTP register of an external controller
  if (Error != ERR_NONE) return Error;
#endif

  if (pConf->CAN20only == false)
  {
    //--- Write Data Bit Time configuration -----------------
    MCAN_DBTP_Register Dconfig;
    Dconfig.DBTP = MCAN_DBTP_DBRP_SET(pConf->DBRP) | MCAN_DBTP_DTSEG1_SET(pConf->DTSEG1)  // Set Data Bit Time configuration
                 | MCAN_DBTP_DTSEG2_SET(pConf->DTSEG2) | MCAN_DBTP_DSJW_SET(pConf->DSJW);
    if (pConf->TDCmode != CAN_TDC_DISABLED) Dconfig.DBTP |= MCAN_DBTP_TDC_EN;             // Enable TDC mode if not disabled
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_DBTP = Dconfig.DBTP;                      // Write configuration to the DBTP register
#else
    Error = MCANV71_WriteREG32(pComp, RegMCAN_DBTP, Dconfig.DBTP);                        // Write configuration to the DBTP register of an external controller
    if (Error != ERR_NONE) return Error;
#endif
    pComp->InternalConfig |= MCAN_CANFD_ENABLED;                                          // CAN-FD is enable if Data Bitrate is set
  }
  else pComp->InternalConfig &= ~MCAN_CANFD_ENABLED;                                      // Set no CAN-FD

  //--- Write Transmitter Delay Compensation configuration ---
  MCAN_TDCR_Register Tconfig;
  Tconfig.TDCR = MCAN_TDCO_SET((pConf->TDCO < 0 ? 0 : pConf->TDCO)) | MCAN_TDCF_SET(pConf->TDCV); // Set Transmitter Delay Compensation configuration
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TDCR = Tconfig.TDCR;                        // Write configuration to the TDCR register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_TDCR, Tconfig.TDCR);                          // Write configuration to the TDCR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure write protection of control register of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureWriteProtection(MCANV71 *pComp, bool enable, uint32_t* const lastRegRead)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnGetCurrentms == NULL) return ERR__PARAMETER_ERROR;
#endif
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
  MCAN_CCCR_Register Reg;

  //--- Set register protection ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Reg.CCCR = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR;             // Read CCCR register
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_CCCR, &Reg.CCCR);               // Read CCCR register of an external device
  if (Error != ERR_NONE) return Error;
#endif
  Reg.CCCR &= ~(MCAN_CCCR_INIT_STARTED | MCAN_CCCR_CONF_WRITE_PROTECT_EN | MCAN_CCCR_CLOCK_STOP_REQ); // Clear write protect bits
  Reg.CCCR |= (MCAN_CCCR_INIT_STARTED | MCAN_CCCR_CONF_WRITE_PROTECT_DIS); // Set default write protect bits
  if (enable) Reg.CCCR |= MCAN_CCCR_CONF_WRITE_PROTECT_EN;                 // Set write protection if ask
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR = Reg.CCCR;             // Write configuration to the CCCR register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_CCCR, Reg.CCCR);               // Write configuration to the CCCR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif

  //--- Wait until correct configuration ---
  uint32_t Value = Reg.CCCR;
  uint32_t StartTime = pComp->fnGetCurrentms();                            // Start the timeout
  while (true)
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    Reg.CCCR = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR;           // Read CCCR register
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_CCCR, &Reg.CCCR);             // Read CCCR register of an external device
    if (Error != ERR_NONE) return Error;
#endif
    if (Reg.CCCR == Value) break;                                          // Check if the write protection is enable
    if (MCAN_TIME_DIFF(StartTime, pComp->fnGetCurrentms()) > 2)            // Wait at least 2ms due to the synchronization mechanism between the two clock domains
      return ERR__DEVICE_TIMEOUT;                                          // Timeout? return the error
  }
  if (lastRegRead != NULL) *lastRegRead = Reg.CCCR;
  return ERR_NONE;
}



//=============================================================================
// Get actual operation mode of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetActualOperationMode(MCANV71 *pComp, eMCAN_OperationMode* const actualMode)
{
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
  uint8_t Config;

#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Config = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR; // Read actual configuration from the CCCR register
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_CCCR, &Config);   // Read actual configuration from the CCCR register of an external device
  if (Error != ERR_NONE) return Error;
#endif
  if ((Config & MCAN_CCCR_INIT_STARTED        ) > 0) *actualMode = MCAN_INITIALIZATION_MODE;
  if ((Config & MCAN_CCCR_NORMAL_CAN_OPERATION) > 0) *actualMode = MCAN_NORMAL_CAN20_MODE;
  if ((Config & MCAN_CCCR_CAN_FD_MODE_EN      ) > 0) *actualMode = MCAN_NORMAL_CANFD_MODE;
  if ((Config & MCAN_CCCR_RESTRICTED_OPERATION) > 0) *actualMode = MCAN_RESTRICTED_OPERATION_MODE;
  if ((Config & MCAN_CCCR_BUS_MONITOR_EN      ) > 0) *actualMode = MCAN_LISTEN_ONLY_MODE;
  if ((Config & MCAN_CCCR_TEST_MODE_EN        ) > 0) *actualMode = MCAN_TEST_MODE;
  if ((Config & MCAN_CCCR_CLOCK_STOP_REQ      ) > 0) *actualMode = MCAN_SLEEP_MODE;
  if (*actualMode == MCAN_TEST_MODE)
  {
    if ((Config & MCAN_CCCR_BUS_MONITOR_EN    ) > 0) *actualMode = MCAN_INTERNAL_LOOPBACK_MODE;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    Config = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TEST; // Read actual configuration from the TEST register
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_TEST, &Config);   // Read actual configuration from the TEST register of an external device
    if (Error != ERR_NONE) return Error;
#endif
    if ((Config & MCAN_TEST_LOOPBACK_MODE_EN  ) > 0) *actualMode = MCAN_EXTERNAL_LOOPBACK_MODE;
  }
  return ERR_NONE;
}


//=============================================================================
// Request operation mode change of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_RequestOperationMode(MCANV71 *pComp, eMCAN_OperationMode newMode)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
  if (pComp->fnGetCurrentms == NULL) return ERR__PARAMETER_ERROR;
#endif
  MCAN_CCCR_Register Reg;
  eERRORRESULT Error;

  //--- Remove write protection ---
  Error = MCANV71_ConfigureWriteProtection(pComp, false, &Reg.CCCR); // Remove write protection, will also put the peripheral in initialization mode
  if (Error != ERR_NONE) return Error;                               // If there is an error while calling MCANV71_ConfigureWriteProtection() then return the error
  Reg.CCCR &= MCAN_MODE_CLEAR_MASK;                                  // Clear operation modes but stay in initialization

  //--- Change mode ---
  if (((pComp->InternalConfig & MCAN_CANFD_ENABLED) == 0) && (newMode == MCAN_NORMAL_CANFD_MODE)) return ERR__CONFIGURATION; // Can't change to CAN-FD mode if the bitrate is not configured for
  //           | Normal | Init | FD | Restricted | Listen | Test | Internal | External |
  //           |        |      |    | Operation  |  Only  |      | LoopBack | LoopBack |
  // CCCR.INIT |    0   |   1  |  0 |      0     |    0   |   1  |     1    |     1    |
  // CCCR.TEST |    0   |   0  |  0 |      0     |    0   |   1  |     1    |     1    |
  // CCCR.MON  |    0   |   0  |  0 |      0     |    1   |   0  |     1    |     0    |
  // CCCR.ASM  |    0   |   0  |  0 |      1     |    0   |   0  |     0    |     0    |
  // CCCR.FDOE |    0   |   0  |  1 |      1     |    0   |   0  |     0    |     0    |
  // TEST.LBCK |    0   |   0  |  0 |      0     |    0   |   0  |     1    |     1    |
  Reg.CCCR |= (uint32_t)newMode & MCAN_CCCR_Mask;                // Apply the above table directly
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR = Reg.CCCR;   // Write configuration to the CCCR register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_CCCR, Reg.CCCR);     // Write configuration to the CCCR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif

  //--- Set Test mode special configuration ---
  if ((newMode == MCAN_INTERNAL_LOOPBACK_MODE) || (newMode == MCAN_EXTERNAL_LOOPBACK_MODE))
  {
    Error = MCANV71_ConfigureTest(pComp, true, MCAN_TX_RESET);   // Configure test mode for loopback
    if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCANV71_ConfigureTest() then return the error
  }
//  if (newMode == MCAN_INITIALIZATION_MODE) return ERR_NONE;      // For initialization mode, do no more...

  //--- Wait until correct configuration ---
  uint32_t Value = Reg.CCCR;
  uint32_t StartTime = pComp->fnGetCurrentms();                  // Start the timeout
  while (true)
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    Reg.CCCR = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR; // Read CCCR register
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_CCCR, &Reg.CCCR);   // Read CCCR register of an external device
    if (Error != ERR_NONE) return Error;
#endif
    if (Reg.CCCR == Value) break;                                // Check if the write protection is enable
    if (MCAN_TIME_DIFF(StartTime, pComp->fnGetCurrentms()) > 2)  // Wait at least 2ms due to the synchronization mechanism between the two clock domains
      return ERR__DEVICE_TIMEOUT;                                // Timeout? return the error
  }
  pComp->InternalConfig &= ~MCAN_DEV_PS_Mask;
  pComp->InternalConfig |= MCAN_DEV_PS_SET(MCAN_DEVICE_NORMAL_POWER_STATE); // Set normal power state
  return ERR_NONE;
}



//=============================================================================
// Configure the test register in the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureTest(MCANV71 *pComp, bool enableLoopback, eMCAN_TestTxPin txPinControl)
{
  MCAN_TEST_Register Reg;
  Reg.TEST = MCAN_TEST_TX_SET(txPinControl) | MCAN_TEST_LOOPBACK_MODE_DIS; // Configure register
  if (enableLoopback) Reg.TEST |= MCAN_TEST_LOOPBACK_MODE_EN;              // Enable loopback
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TEST = Reg.TEST;             // Write configuration to the TEST register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_TEST, Reg.TEST);  // Write configuration to the TEST register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// Read the test register in the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetTest(MCANV71 *pComp, eMCAN_TestRxPin* const rxPin, uint8_t* const txNumPrepared, bool* const preparedValid, uint8_t* const txNumStarted, bool* const startedValid)
{
  MCAN_TEST_Register Reg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Reg.TEST = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TEST;            // Read the value to the TSCC register
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_TEST, &Reg.TEST); // Read the value to the TSCC register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  if (rxPin         != NULL) *rxPin         = (eMCAN_TestRxPin)MCAN_TEST_RX_GET(Reg.TEST);
  if (txNumPrepared != NULL) *txNumPrepared = (uint8_t)MCAN_TEST_TXBNP_GET(Reg.TEST);
  if (preparedValid != NULL) *preparedValid = ((Reg.TEST & MCAN_TEST_PREPARE_VALID) > 0);
  if (txNumStarted  != NULL) *txNumStarted  = (uint8_t)MCAN_TEST_TXBNS_GET(Reg.TEST);
  if (startedValid  != NULL) *startedValid  = ((Reg.TEST & MCAN_TEST_STARTED_VALID) > 0);
  return ERR_NONE;
}



//=============================================================================
// Configure CAN Control of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureCANController(MCANV71 *pComp, setMCAN_CANCtrlFlags flags)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint32_t Config;

  Error = MCANV71_ConfigureWriteProtection(pComp, false, &Config);                                           // Disable write protection
  if (Error != ERR_NONE) return Error;                                                                       // If there is an error while calling MCANV71_ConfigureWriteProtection() then return the error
  if ((Config & MCAN_IN_INIT_AND_UNPROTECTED) != MCAN_IN_INIT_AND_UNPROTECTED) return ERR__NEED_CONFIG_MODE; // Device must be in initialization mode and unprotected to perform the configuration

  Config &= ~(MCAN_CCCR_NONISO_OPERATION_EN  | MCAN_CCCR_EXTERNAL_TIMESTAMP_TSU   | MCAN_CCCR_PROTOCOL_EXCEPTION_DIS // Clear by default all flags that can be changed
            | MCAN_CCCR_EDGE_FILTERING_EN    | MCAN_CCCR_8BIT_MESSAGE_MARKER_USED | MCAN_CCCR_AUTOMATIC_RETRANSMISSION_DIS
            | MCAN_CCCR_BITRATE_SWITCHING_EN | MCAN_CCCR_TRANSMIT_PAUSE_EN        | MCAN_MODE_CLEAR_MASK);
  if ((flags & MCAN_CAN_EDGE_FILTERING_DISABLE          ) == 0) Config |= MCAN_CCCR_EDGE_FILTERING_EN;            // Set two consecutive dominant tq required to detect an edge f or hard synchronization
  if ((flags & MCAN_CAN_TRANSMIT_PAUSE_DISABLE          ) == 0) Config |= MCAN_CCCR_TRANSMIT_PAUSE_EN;            // Set transmit pause enabled
  if ((flags & MCAN_CAN_AUTOMATIC_RETRANSMISSION_DISABLE) >  0) Config |= MCAN_CCCR_AUTOMATIC_RETRANSMISSION_DIS; // Set automatic retransmission disabled
  if ((flags & MCAN_CANFD_BITRATE_SWITCHING_DISABLE     ) == 0) Config |= MCAN_CCCR_BITRATE_SWITCHING_EN;         // Set bit rate switching for transmissions enabled
  if ((flags & MCAN_CAN_PROTOCOL_EXCEPT_HANDLING_DISABLE) >  0) Config |= MCAN_CCCR_PROTOCOL_EXCEPTION_DIS;       // Set protocol exception handling disabled
  if ((flags & MCAN_CANFD_USE_ISO_CRC                   ) == 0) Config |= MCAN_CCCR_NONISO_OPERATION_EN;          // Set CAN FD frame format according to ISO 11898-1:2015
  if ((flags & MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT       ) >  0) Config |= MCAN_CCCR_16BIT_MESSAGE_MARKER_USED;    // Set 16-bit Message Marker used, replacing 16-bit timestamps in Tx Event FIFO
  if ((flags & MCAN_CAN_EXTERNAL_TIMESTAMPING_BY_TSU    ) >  0) Config |= MCAN_CCCR_EXTERNAL_TIMESTAMP_TSU;       // Set external time stamping by TSU
  pComp->InternalConfig &= ~MCAN_16BIT_MM_ENABLED;                                                                // Clear the 16-bit Message Marker flag
  if ((flags & MCAN_WIDE_MESSAGE_MARKER) > 0) pComp->InternalConfig |= MCAN_16BIT_MM_ENABLED;                     // Set the 16-bit Message Marker flag if wide message marker configured
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR = Config;                                                      // Write configuration to the CCCR register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_CCCR, Config);                                                        // Write configuration to the CCCR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Enter the MCAN peripheral in sleep mode
//=============================================================================
eERRORRESULT MCANV71_EnterSleepMode(MCANV71 *pComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  eMCAN_PowerStates LastPS = MCAN_DEV_PS_GET(pComp->InternalConfig);            // Get last power state
  if (LastPS == MCAN_DEVICE_SLEEP_NOT_CONFIGURED) return ERR__CONFIGURATION;    // No configuration available to enter sleep mode
  if (LastPS != MCAN_DEVICE_NORMAL_POWER_STATE)   return ERR__ALREADY_IN_SLEEP; // Peripheral already in sleep mode
  Error = MCANV71_RequestOperationMode(pComp, MCAN_SLEEP_MODE);                 // Set Sleep mode
  if (Error != ERR_NONE) return Error;                                          // If there is an error while calling MCANV71_RequestOperationMode() then return the error
  pComp->InternalConfig &= ~MCAN_DEV_PS_Mask;
  pComp->InternalConfig |= MCAN_DEV_PS_SET(MCAN_DEVICE_SLEEP_STATE);            // Else the peripheral will be in sleep mode
  return ERR_NONE;
}



//=============================================================================
// Verify if the MCAN peripheral is in sleep mode
//=============================================================================
eERRORRESULT MCANV71_IsDeviceInSleepMode(MCANV71 *pComp, bool* const isInSleepMode)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif

  eMCAN_PowerStates LastPS = MCAN_DEV_PS_GET(pComp->InternalConfig);          // Get last power state
  if (LastPS == MCAN_DEVICE_SLEEP_NOT_CONFIGURED) return ERR__CONFIGURATION;  // No configuration available to enter sleep mode
  *isInSleepMode = true;
  uint32_t Config;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Config = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_CCCR;                  // Read the Oscillator Register configuration
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_CCCR, &Config);                    // Read the Oscillator Register configuration of an external device
  if (Error != ERR_NONE) return Error;
#endif
  *isInSleepMode = ((Config & MCAN_CCCR_CLOCK_STOP_ACK) > 0);                 // Return the actual state of the sleep mode
  if (*isInSleepMode == false)
  {
    pComp->InternalConfig &= ~MCAN_DEV_PS_Mask;
    pComp->InternalConfig |= MCAN_DEV_PS_SET(MCAN_DEVICE_NORMAL_POWER_STATE); // If the function return is not in sleep mode then refresh the internal state of the peripheral
  }
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure the Time Stamp of frames in the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureTimeStamp(MCANV71 *pComp, eMCAN_TimeStampSelect timestampSource, uint8_t prescaler)
{
  if (prescaler < (MCAN_TSCC_TCP_MINVALUE + 1)) return ERR__PARAMETER_ERROR;
  if (prescaler > (MCAN_TSCC_TCP_MAXVALUE + 1)) return ERR__PARAMETER_ERROR;
  MCAN_TSCC_Register Config;

  //--- Write Time Stamp configuration ----------------------
  Config.TSCC = MCAN_TSCC_TIMESTAMP_SELECT_SET(timestampSource) | MCAN_TSCC_TCP_SET(prescaler - 1); // Configure the register
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TSCC = Config.TSCC;                                   // Write configuration to the TSCC register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_TSCC, Config.TSCC);                        // Write configuration to the TSCC register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure the Rx timeout counter in the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureTimeoutCounter(MCANV71 *pComp, bool enableTC, eMCAN_TimeoutSelect timeoutSelect, uint8_t period)
{
  MCAN_TOCC_Register Config;

  //--- Write Time Stamp configuration ----------------------
  Config.TOCC = MCAN_TOCC_TIMEOUT_COUNTER_DIS | MCAN_TOCC_TIMEOUT_SELECT_SET(timeoutSelect) | MCAN_TOCC_TIMEOUT_PERIOD_SET(period); // Configure the register
  if (enableTC) Config.TOCC |= MCAN_TOCC_TIMEOUT_COUNTER_EN;                 // Set enable if ask
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TOCC = Config.TOCC;            // Write configuration to the TOCC register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_TOCC, Config.TOCC); // Write configuration to the TOCC register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// [STATIC] Configure a FIFO of the MCAN peripheral
//=============================================================================
eERRORRESULT __MCANV71_ConfigureTxFIFObuffers(MCANV71 *pComp, uint32_t* const startAddr, MCAN_FIFObuff* const confTxBuffers, MCAN_FIFObuff* const confTxFIFOTXQ)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  MCAN_TXBC_Register TxConf;

  //--- Check configuration ---
  uint32_t ElementsCount = 0;
  eMCAN_PayloadSize Payload = MCAN_8_BYTES;
  if (confTxBuffers != NULL)
  {
    ElementsCount += (confTxBuffers->Size > MCAN_TX_BUFFER_SIZE_MAX ? MCAN_TX_BUFFER_SIZE_MAX : confTxBuffers->Size);
    if (confTxBuffers->Payload > MCAN_PAYLOAD_COUNT) return ERR__CONFIGURATION;
    if (confTxBuffers->Payload > Payload) Payload = confTxBuffers->Payload;
  }
  if (confTxFIFOTXQ != NULL)
  {
    ElementsCount += (confTxFIFOTXQ->Size > MCAN_TX_FIFO_TXQ_SIZE_MAX ? MCAN_TX_FIFO_TXQ_SIZE_MAX : confTxFIFOTXQ->Size);
    if (confTxFIFOTXQ->Payload > MCAN_PAYLOAD_COUNT) return ERR__CONFIGURATION;
    if (confTxFIFOTXQ->Payload > Payload) Payload = confTxFIFOTXQ->Payload;
  }
  if (ElementsCount > MCAN_TX_ELEMENTS_SIZE_MAX) return ERR__CONFIGURATION; // Tx Buffers + Tx FIFO/TXQ shall be maximum 32
  *startAddr += (ElementsCount * (MCAN_CAN_TX_MESSAGE_HEADER_SIZE + MCANV71_PayloadToByte(Payload))); // Update the address
  if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;         // Check of configured memory

  //--- Configure MCAN Tx Buffer ---
  TxConf.TXBC = MCAN_TXBC_TX_BUFFERS_SA_SET(*startAddr) | MCAN_TXBC_TX_BUFFER_SIZE_SET(0) | MCAN_TXBC_TX_FIFO_QUEUE_SIZE_SET(0) | MCAN_TXBC_TX_FIFO_OPERATION; // Init TXBC register value
  if (confTxBuffers != NULL)
  {
    if (confTxBuffers->Name != MCAN_TX_BUFFER) return ERR__PARAMETER_ERROR;
    TxConf.TXBC |= MCAN_TXBC_TX_BUFFER_SIZE_SET(confTxBuffers->Size > MCAN_TX_BUFFER_SIZE_MAX ? MCAN_TX_BUFFER_SIZE_MAX : confTxBuffers->Size);
    if ((confTxBuffers->ControlFlags & MCAN_TXQ_MODE) > 0) TxConf.TXBC |= MCAN_TXBC_TX_QUEUE_OPERATION;
  }
  if (confTxFIFOTXQ != NULL)
  {
    if (confTxFIFOTXQ->Name != MCAN_TXQ_FIFO) return ERR__PARAMETER_ERROR;
    TxConf.TXBC |= MCAN_TXBC_TX_FIFO_QUEUE_SIZE_SET(confTxFIFOTXQ->Size > MCAN_TX_FIFO_TXQ_SIZE_MAX ? MCAN_TX_FIFO_TXQ_SIZE_MAX : confTxFIFOTXQ->Size);
  }
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBC = TxConf.TXBC;            // Write configuration to the TXBC register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_TXBC, TxConf.TXBC); // Write configuration to the TXBC register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_TXBC] = TxConf.TXBC;                          // Write in cache
#  endif
#endif

  //--- Configure MCAN Tx Buffer Element Size ---
  MCAN_TXESC_Register RegTXESC;
  RegTXESC.TXESC = MCAN_TXESC_TX_BUFFER_DATA_FIELD_SIZE_SET(Payload);
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXESC = RegTXESC.TXESC;       // Write configuration to the TXESC register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_TXESC, RegTXESC.TXESC);         // Write configuration to the TXESC register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_TXESC] = TxConf.TXESC;                       // Write in cache
#  endif
#endif

  return ERR_NONE;
}



//=============================================================================
// [STATIC] Configure a FIFO list of the MCAN peripheral
//=============================================================================
eERRORRESULT __MCANV71_ConfigureFIFOList(MCANV71 *pComp, uint32_t* const startAddr, const MCAN_FIFObuff* const listFIFO, size_t count)
{
#ifdef CHECK_NULL_PARAM
  if (listFIFO == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (count == 0) return ERR_NONE;
  if (count > MCAN_FIFO_CONF_MAX) return ERR__OUT_OF_RANGE;
  MCAN_RXESC_Register ESCconf;
  ESCconf.RXESC = MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_SET(MCAN_8_BYTES) | MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_SET(MCAN_8_BYTES) | MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_SET(MCAN_8_BYTES); // Init RXESC register value
  MCAN_IE_Register INTconf; INTconf.IE = 0;
  eERRORRESULT Error;

  //--- Device in Configuration Mode ---
  eMCAN_OperationMode OpMode;
  Error = MCANV71_GetActualOperationMode(pComp, &OpMode);                             // Get actual Operational Mode
  if (Error != ERR_NONE) return Error;                                                // If there is an error while calling MCAN_GetActualOperationMode() then return the error
  if (OpMode != MCAN_INITIALIZATION_MODE) return ERR__NEED_CONFIG_MODE;               // Device must be in Configuration Mode to perform the configuration

  //--- Find and configure Tx Buffer + Tx FIFO/TXQ ---
  MCAN_FIFObuff* pTxBuffer  = NULL;
  MCAN_FIFObuff* pTxFIFOTXQ = NULL;
  for (size_t z = 0; z < count; ++z)
  {
    if (listFIFO[z].Name == MCAN_TX_BUFFER) pTxBuffer = (MCAN_FIFObuff*)&listFIFO[z]; // Get the Tx Buffer
    if (listFIFO[z].Name == MCAN_TXQ_FIFO )
    {
      pTxFIFOTXQ = (MCAN_FIFObuff*)&listFIFO[z];                                      // Get the Tx FIFO/TXQ
      if (((listFIFO[z].InterruptFlags & MCAN_FIFO_TRANSMIT_FIFO_EMPTY_INT) > 0)
        /*&& ((listFIFO[z].ControlFlags & MCAN_TX_FIFO_MODE) > 0)*/) INTconf.IE |= MCAN_IE_TFE_EN; // Enable the Tx FIFO Empty Interrupt
    }
  }
  Error = __MCANV71_ConfigureTxFIFObuffers(pComp, startAddr, pTxBuffer, pTxFIFOTXQ);  // Configure the Tx Buffers/FIFO/TXQ
  if (Error != ERR_NONE) return Error;

  //--- Find and configure the TEF ---
  uint32_t ElementSize = MCAN_CAN_TX_EVENTOBJECT_SIZE;
  MCAN_TXEFC_Register TEFconf;
  TEFconf.TXEFC = MCAN_TXEFC_EVENT_FIFO_SA_SET(*startAddr) | MCAN_TXEFC_EVENT_FIFO_SIZE_SET(0) | MCAN_TXEFC_EVENT_FIFO_WATERMARK_SET(0); // Init TXEFC register value
  for (size_t z = 0; z < count; ++z)
    if (listFIFO[z].Name == MCAN_TEF)
    {
      TEFconf.TXEFC |= MCAN_TXEFC_EVENT_FIFO_SIZE_SET(listFIFO[z].Size) | MCAN_TXEFC_EVENT_FIFO_WATERMARK_SET(listFIFO[z].WatermarkLevel);
      *startAddr += (ElementSize * MCAN_CAN_TX_EVENTOBJECT_SIZE);                     // Update the address
      if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;               // Check of configured memory
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_EVENT_NEW_MESSAGE_INT      ) > 0) INTconf.IE |= MCAN_IE_TEFN_EN; // Enable the Tx Event FIFO New Entry Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_EVENT_WATERMARK_REACHED_INT) > 0) INTconf.IE |= MCAN_IE_TEFW_EN; // Enable the Tx Event FIFO Watermark Reached Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_EVENT_FULL_INT             ) > 0) INTconf.IE |= MCAN_IE_TEFF_EN; // Enable the Tx Event FIFO Full Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_EVENT_LOST_MESSAGE_INT     ) > 0) INTconf.IE |= MCAN_IE_TEFL_EN; // Enable the Tx Event FIFO Event Lost Interrupt
    }
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFC = TEFconf.TXEFC;                  // Write configuration to the TXEFC register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_TXEFC, TEFconf.TXEFC);                    // Write configuration to the TXEFC register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_TXEFC] = TEFconf.TXEFC;                                // Write in cache
#  endif
#endif

  //--- Configure Rx FIFO0 ---
  ElementSize = 0;
  MCAN_RXF0C_Register RxFIFO0conf;
  RxFIFO0conf.RXF0C = MCAN_RXF0C_RX_FIFO0_SA_SET(*startAddr) | MCAN_RXF0C_RX_FIFO0_SIZE_SET(0) | MCAN_RXF0C_RX_FIFO0_WATERMARK_SET(0) | MCAN_RXF0C_RX_FIFO0_BLOCKING_MODE; // Init RXF0C register value
  for (size_t z = 0; z < count; ++z)
    if (listFIFO[z].Name == MCAN_RX_FIFO0)
    {
      ElementSize = listFIFO[z].Size > MCAN_RX_FIFO0_SIZE_MAX ? MCAN_RX_FIFO0_SIZE_MAX : listFIFO[z].Size;
      if (listFIFO[z].Payload > MCAN_PAYLOAD_COUNT) return ERR__CONFIGURATION;
      ESCconf.RXESC |= MCAN_RXESC_RX_FIFO0_DATA_FIELD_SIZE_SET(listFIFO[z].Payload);      // Configure the payload configuration
      RxFIFO0conf.RXF0C |= MCAN_RXF0C_RX_FIFO0_SIZE_SET(ElementSize)                      // Configure the element size
                        |  MCAN_RXF0C_RX_FIFO0_WATERMARK_SET(listFIFO[z].WatermarkLevel); // Configure the watermark
      if ((listFIFO[z].ControlFlags & MCAN_RX_FIFO_OVERWRITE_MODE) > 0) RxFIFO0conf.RXF0C |= MCAN_RXF0C_RX_FIFO0_OVERWRITE_MODE;
      *startAddr += (ElementSize * (MCAN_CAN_RX_MESSAGE_HEADER_SIZE + MCANV71_PayloadToByte(listFIFO[z].Payload)));  // Update the address
      if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;                                              // Check of configured memory
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT      ) > 0) INTconf.IE |= MCAN_IE_RF0N_EN; // Enable the Receive FIFO 0 New Message Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_WATERMARK_REACHED_INT) > 0) INTconf.IE |= MCAN_IE_RF0W_EN; // Enable the Receive FIFO 0 Watermark Reached Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_FULL_INT             ) > 0) INTconf.IE |= MCAN_IE_RF0F_EN; // Enable the Receive FIFO 0 Full Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT     ) > 0) INTconf.IE |= MCAN_IE_RF0L_EN; // Enable the Receive FIFO 0 Message Lost Interrupt
    }
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0C = RxFIFO0conf.RXF0C;              // Write configuration to the RXF0C register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_RXF0C, RxFIFO0conf.RXF0C);                // Write configuration to the RXF0C register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_RXF0C] = RxFIFO0conf.RXF0C;                            // Write in cache
#  endif
#endif

  //--- Configure Rx FIFO1 ---
  ElementSize = 0;
  MCAN_RXF1C_Register RxFIFO1conf;
  RxFIFO1conf.RXF1C = MCAN_RXF1C_RX_FIFO1_SA_SET(*startAddr) | MCAN_RXF1C_RX_FIFO1_SIZE_SET(0) | MCAN_RXF1C_RX_FIFO1_WATERMARK_SET(0) | MCAN_RXF1C_RX_FIFO1_BLOCKING_MODE; // Init RXF1C register value
  for (size_t z = 0; z < count; ++z)
    if (listFIFO[z].Name == MCAN_RX_FIFO1)
    {
      ElementSize = listFIFO[z].Size > MCAN_RX_FIFO1_SIZE_MAX ? MCAN_RX_FIFO1_SIZE_MAX : listFIFO[z].Size;
      if (listFIFO[z].Payload > MCAN_PAYLOAD_COUNT) return ERR__CONFIGURATION;
      ESCconf.RXESC |= MCAN_RXESC_RX_FIFO1_DATA_FIELD_SIZE_SET(listFIFO[z].Payload);      // Configure the payload configuration
      RxFIFO1conf.RXF1C |= MCAN_RXF1C_RX_FIFO1_SIZE_SET(ElementSize)                      // Configure the element size
                        |  MCAN_RXF1C_RX_FIFO1_WATERMARK_SET(listFIFO[z].WatermarkLevel); // Configure the watermark
      if ((listFIFO[z].ControlFlags & MCAN_RX_FIFO_OVERWRITE_MODE) > 0) RxFIFO1conf.RXF1C |= MCAN_RXF1C_RX_FIFO1_OVERWRITE_MODE;
      *startAddr += (ElementSize * (MCAN_CAN_RX_MESSAGE_HEADER_SIZE + MCANV71_PayloadToByte(listFIFO[z].Payload)));  // Update the address
      if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;                                              // Check of configured memory
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_NEW_MESSAGE_INT      ) > 0) INTconf.IE |= MCAN_IE_RF1N_EN; // Enable the Receive FIFO 1 New Message Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_WATERMARK_REACHED_INT) > 0) INTconf.IE |= MCAN_IE_RF1W_EN; // Enable the Receive FIFO 1 Watermark Reached Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_FULL_INT             ) > 0) INTconf.IE |= MCAN_IE_RF1F_EN; // Enable the Receive FIFO 1 Full Interrupt
      if ((listFIFO[z].InterruptFlags & MCAN_FIFO_RECEIVE_LOST_MESSAGE_INT     ) > 0) INTconf.IE |= MCAN_IE_RF1L_EN; // Enable the Receive FIFO 1 Message Lost Interrupt
    }
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1C = RxFIFO1conf.RXF1C;              // Write configuration to the RXF1C register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_RXF1C, RxFIFO1conf.RXF1C);                // Write configuration to the RXF1C register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_RXF1C] = RxFIFO1conf.RXF1C;                            // Write in cache
#  endif
#endif

  //--- Configure Rx Buffer ---
  ElementSize = 0;
  MCAN_RXBC_Register RxBufferConf;
  RxBufferConf.RXBC = MCAN_RXBC_RX_BUFFER_SA_SET(*startAddr);                         // Init RXBC register value
  for (size_t z = 0; z < count; ++z)
    if (listFIFO[z].Name == MCAN_RX_BUFFER)
    {
      ElementSize = listFIFO[z].Size > MCAN_RX_BUFFER_SIZE_MAX ? MCAN_RX_BUFFER_SIZE_MAX : listFIFO[z].Size;
      if (listFIFO[z].Payload > MCAN_PAYLOAD_COUNT) return ERR__CONFIGURATION;
      ESCconf.RXESC |= MCAN_RXESC_RX_BUFFER_DATA_FIELD_SIZE_SET(listFIFO[z].Payload); // Configure the payload configuration
      MCAN_RX_BUFFERS_SIZE_CLEAR(pComp->InternalConfig);                              // Clear the previous configuration. The Rx Buffer is a little bid weird since the is no Element Size configuration
      pComp->InternalConfig |= MCAN_RX_BUFFERS_SIZE_SET(ElementSize);                 // Save size in the InternalConfig
      *startAddr += (ElementSize * (MCAN_CAN_RX_MESSAGE_HEADER_SIZE + MCANV71_PayloadToByte(listFIFO[z].Payload))); // Update the address
      if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;               // Check of configured memory
    }
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXBC = RxBufferConf.RXBC;               // Write configuration to the RXBC register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_RXBC, RxBufferConf.RXBC);                 // Write configuration to the RXBC register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_RXBC] = RxFIFO1conf.RXBC;                              // Write in cache
#  endif
#endif

  //--- Configure RXESC and IE registers ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXESC = ESCconf.RXESC;                  // Write configuration to the RXESC register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_RXESC, ESCconf.RXESC);                    // Write configuration to the RXESC register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_RXESC] = ESCconf.RXESC;                                // Write in cache
#  endif
#endif
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IE = INTconf.IE;                        // Write configuration to the IE register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_IE, INTconf.IE);                          // Write configuration to the IE register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  pComp->InternalConfig |= MCAN_FIFOS_BUFF_CONFIG_FLAG;
  return ERR_NONE;
}



//=============================================================================
// Get status of a FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetFIFOStatus(MCANV71 *pComp, eMCAN_FIFObuffer name, setMCAN_FIFObufferstatus* const statusFlags)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (statusFlags == NULL)) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Flags = 0 ,Level, ConfRegister, BufferSize, NewDataIdx0_31, NewDataIdx32_63, Count1;
  uint32_t* pNewDataIdx32_63 = NULL;
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  uint32_t Address = 0;
#endif
  eERRORRESULT Error;

  //--- Get FIFO/Buffer status ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  switch (name)
  {
    case MCAN_TEF      : Flags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFS; break; // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0 : Flags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0S; break; // RX FIFO 0
    case MCAN_RX_FIFO1 : Flags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1S; break; // RX FIFO 1
    case MCAN_RX_BUFFER:                                                            break; // RX buffer
    case MCAN_TX_BUFFER: Flags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXFQS; break; // TX buffer
    case MCAN_TXQ_FIFO : Flags = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXFQS; break; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }
#else
  switch (name)
  {
    case MCAN_TEF      : Address = RegMCAN_TXEFS; break; // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0 : Address = RegMCAN_RXF0S; break; // RX FIFO 0
    case MCAN_RX_FIFO1 : Address = RegMCAN_RXF1S; break; // RX FIFO 1
    case MCAN_RX_BUFFER: Address = RegMCAN_RXF1S; break; // RX buffer
    case MCAN_TX_BUFFER: Address = RegMCAN_TXFQS; break; // TX buffer
    case MCAN_TXQ_FIFO : Address = RegMCAN_TXFQS; break; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }
  if (name != MCAN_RX_BUFFER)
  {
    Error = MCANV71_ReadREG32(pComp, Address, &Flags);   // Read FIFO/Buffer/TXQ status but not for the Rx Buffer of an external controller
    if (Error != ERR_NONE) return Error;
  }
#endif

  //--- Generate status ---
  switch (name)
  {
    case MCAN_TEF      : // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0 : // RX FIFO 0
    case MCAN_RX_FIFO1 : // RX FIFO 1
      Level = MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_GET(Flags);
      if (Level > 0)                                      *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_NOT_EMPTY   ); // Same level position in TEF, Rx FIFO0 and Rx FIFO1
      if ((Flags & MCAN_RXF0S_RX_FIFO0_FULL        ) > 0) *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_FULL        ); // Same full flag position in TEF, Rx FIFO0 and Rx FIFO1
      if ((Flags & MCAN_RXF0S_RX_FIFO0_MESSAGE_LOST) > 0) *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_MESSAGE_LOST); // Same message lost position in TEF, Rx FIFO0 and Rx FIFO1
      if (name == MCAN_RX_FIFO1)
      {
        switch (MCAN_RXF1S_DEBUG_MESSAGE_STATUS_GET(Flags))
        {
          default:
          case MCAN_IDLE_STATE                : *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_DEBUG_MESSAGE_IDLE   ); break;
          case MCAN_DEBUG_MESSAGE_A_RECEIVED  : *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_DEBUG_MESSAGE_MSG_A  ); break;
          case MCAN_DEBUG_MESSAGE_AB_RECEIVED : *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_DEBUG_MESSAGE_MSG_AB ); break;
          case MCAN_DEBUG_MESSAGE_ABC_RECEIVED: *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_DEBUG_MESSAGE_MSG_ABC); break;
        }
      }        
      break;

    case MCAN_RX_BUFFER: // RX buffer
      BufferSize = MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig);
      if (BufferSize > 32) pNewDataIdx32_63 = &NewDataIdx32_63;                         // If the size of Rx Buffer is more than 32, get the flag for the last 32 indexes
      Error = MCANV71_GetRxBufferNewDataFlag(pComp, &NewDataIdx0_31, pNewDataIdx32_63);
      if (Error != ERR_NONE) return Error;                                              // If there is an error while calling MCANV71_GetRxBufferNewDataFlag() then return the error
      Count1 = __CalcBitCount(NewDataIdx0_31);                                          // Count messages available in Rx buffer (first 32 bits)
      if (BufferSize > 32) Count1 += __CalcBitCount(NewDataIdx32_63);                   // Count messages available in Rx buffer (next 32 bits)
      if (Count1 > 0)                  *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_NOT_EMPTY);
      if (Count1 >  (BufferSize >> 1)) *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_HALF_FULL);
      if (Count1 >= (BufferSize >> 0)) *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_RX_FIFOBUFF_FULL     );
      break;

    case MCAN_TX_BUFFER: // TX buffer
    case MCAN_TXQ_FIFO : // TXQ/FIFO - Transmit Queue or Tx FIFO
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
      ConfRegister = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBC;                  // Read Tx Buffer Configuration internal register
#elif defined(MCAN_USE_CACHE)
      ConfRegister = pComp->__RegCache[MCAN_CACHE_TXBC];                                // Read Tx Buffer Configuration in cache
#else
      Error = MCANV71_ReadREG32(pComp, RegMCAN_TXBC, &ConfRegister);                    // Read Tx Buffer Configuration of an external device
      if (Error != ERR_NONE) return Error;
#endif
      Level = MCAN_TXFQS_TX_FIFO_FREE_LEVEL_GET(Flags);
      if ((Flags & MCAN_TXFQS_TX_FIFO_QUEUE_FULL ) == 0)                *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_TX_FIFOBUFF_NOT_FULL);
      if ((Level == 0) && MCAN_TXBC_IS_TX_FIFO_OPERATION(ConfRegister)) *statusFlags = (setMCAN_FIFObufferstatus)(*statusFlags + MCAN_TX_FIFOBUFF_EMPTY   );
      break;

    default: return ERR__PARAMETER_ERROR;
  }
  return ERR_NONE;
}



//=============================================================================
// Get next message index of a FIFO/Buffer of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetNextMessageAddressFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  uint32_t Status = 0;


  //--- Get FIFO/Buffer status ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  switch (name)
  {
    case MCAN_TEF      : Status = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFS; break; // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0 : Status = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0S; break; // RX FIFO 0
    case MCAN_RX_FIFO1 : Status = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1S; break; // RX FIFO 1
    case MCAN_RX_BUFFER: return ERR__CONFIGURATION;                                         // RX buffer
    case MCAN_TX_BUFFER: return ERR__CONFIGURATION;                                         // TX buffer
    case MCAN_TXQ_FIFO : Status = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXFQS; break; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }
#else
  //--- Set address of the FIFO ---
  uint32_t Address = 0;
  switch (name)
  {
    case MCAN_TEF      : Address = RegMCAN_TXEFS; break; // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0 : Address = RegMCAN_RXF0S; break; // RX FIFO 0
    case MCAN_RX_FIFO1 : Address = RegMCAN_RXF1S; break; // RX FIFO 1
    case MCAN_RX_BUFFER: return ERR__CONFIGURATION;      // RX buffer
    case MCAN_TX_BUFFER: return ERR__CONFIGURATION;      // TX buffer
    case MCAN_TXQ_FIFO : Address = RegMCAN_TXFQS; break; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, Address, &Status); // Read FIFO/Buffer/TXQ status of an external controller
  if (Error != ERR_NONE) return Error;
#endif

  //--- Get indexes ---
  switch (name)
  {
    case MCAN_RX_FIFO0 : // RX FIFO 0
    case MCAN_RX_FIFO1 : // RX FIFO 1
      if (level    != NULL) *level    = MCAN_RXF0S_RX_FIFO0_FILL_LEVEL_GET(Status); // Same level position in Rx FIFO0 and Rx FIFO1
      if (getIndex != NULL) *getIndex = MCAN_RXF0S_RX_FIFO0_GET_INDEX_GET(Status);  // Same get index position in Rx FIFO0 and Rx FIFO1
      if (putIndex != NULL) *putIndex = MCAN_RXF0S_RX_FIFO0_PUT_INDEX_GET(Status);  // Same put index position in Rx FIFO0 and Rx FIFO1
      break;

    case MCAN_TEF      : // TEF - Transmit Event FIFO
    case MCAN_TXQ_FIFO : // TXQ/FIFO - Transmit Queue or Tx FIFO
      if (level    != NULL) *level    = MCAN_TXFQS_EVENT_FIFO_FILL_LEVEL_GET(Status); // Same level position in TEF and TXQ/FIFO
      if (getIndex != NULL) *getIndex = MCAN_TXFQS_EVENT_FIFO_GET_INDEX_GET(Status);  // Same get index position in TEF and TXQ/FIFO
      if (putIndex != NULL) *putIndex = MCAN_TXFQS_EVENT_FIFO_PUT_INDEX_GET(Status);  // Same put index position in TEF and TXQ/FIFO
      break;

    default: return ERR__PARAMETER_ERROR;
  }
  return ERR_NONE;
}



//=============================================================================
// Acknowledge FIFO/TEF of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_AcknowledgeFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t acknowledgeIndex)
{
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  switch (name)
  {
    case MCAN_TEF     : ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXEFA = MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_SET(acknowledgeIndex); break; // TEF - Transmit Event FIFO
    case MCAN_RX_FIFO0: ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF0A = MCAN_RXF0A_RX_FIFO0_ACK_INDEX_SET(acknowledgeIndex);   break; // RX FIFO 0
    case MCAN_RX_FIFO1: ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_RXF1A = MCAN_RXF1A_RX_FIFO1_ACK_INDEX_SET(acknowledgeIndex);   break; // RX FIFO 1
    case MCAN_RX_BUFFER: return ERR__CONFIGURATION; // RX buffer
    case MCAN_TX_BUFFER: return ERR__CONFIGURATION; // TX buffer
    case MCAN_TXQ_FIFO : return ERR__CONFIGURATION; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }
#else
  //--- Set address of the FIFO ---
  uint32_t Address = 0, RegData = 0;
  switch (name)
  {
    case MCAN_TEF      : // TEF - Transmit Event FIFO
      Address = RegMCAN_TXEFA;
      RegData = MCAN_TXEFA_EVENT_FIFO_ACK_INDEX_SET(acknowledgeIndex);
      break;

    case MCAN_RX_FIFO0 : // RX FIFO 0
      Address = RegMCAN_RXF0A;
      RegData = MCAN_RXF0A_RX_FIFO0_ACK_INDEX_SET(acknowledgeIndex);
      break;

    case MCAN_RX_FIFO1 : // RX FIFO 1
      Address = RegMCAN_RXF1A;
      RegData = MCAN_RXF1A_RX_FIFO1_ACK_INDEX_SET(acknowledgeIndex);
      break;

    case MCAN_RX_BUFFER: return ERR__CONFIGURATION; // RX buffer
    case MCAN_TX_BUFFER: return ERR__CONFIGURATION; // TX buffer
    case MCAN_TXQ_FIFO : return ERR__CONFIGURATION; // TXQ/FIFO - Transmit Queue or Tx FIFO
    default: return ERR__PARAMETER_ERROR;
  }

  //--- Set FIFO/TEF acknowledge ---
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, Address, RegData); // Write configuration to an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// Set multiple Tx Buffer cancellation request of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_SetMultipleTxBufferCancellationRequest(MCANV71 *pComp, uint32_t multipleRequest)
{
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBCR = multipleRequest;            // Write configuration to the TXBCR register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_TXBCR, multipleRequest); // Write configuration to the TXBCR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}






//**********************************************************************************************************************************************************
//=============================================================================
// Configure Global Filter Configuration of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureGlobalFilters(MCANV71 *pComp, bool rejectAllStandardIDs, bool rejectAllExtendedIDs, eMCAN_AcceptNonMatching nonMatchingStandardID, eMCAN_AcceptNonMatching nonMatchingExtendedID)
{
  MCAN_GFC_Register RegConf;
  RegConf.GFC = (rejectAllStandardIDs ? MCAN_GFC_REJECT_REMOTE_FRAMES_STANDARD_ID : 0) // Set Reject Remote Frames Standard if wanted
              | (rejectAllExtendedIDs ? MCAN_GFC_REJECT_REMOTE_FRAMES_EXTENDED_ID : 0) // Set Reject Remote Frames Extended if wanted
              | MCAN_GFC_ACCEPT_NON_MATCHING_STANDARD_SET(nonMatchingStandardID)       // Set Accept Non-matching Frames Standard
              | MCAN_GFC_ACCEPT_NON_MATCHING_EXTENDED_SET(nonMatchingExtendedID);      // Set Accept Non-matching Frames Extended
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_GFC = RegConf.GFC;                       // Write configuration to the GFC register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_GFC, RegConf.GFC);            // Write configuration to the GFC register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// [STATIC] Configure SID and EID filters MRAM allocation of the MCAN peripheral
//=============================================================================
eERRORRESULT __MCANV71_ConfigureFiltersMRAM(MCANV71 *pComp, uint32_t* const startAddr, uint8_t sidElementsCount, uint8_t eidElementsCount)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Device in Configuration Mode ---
  eMCAN_OperationMode OpMode;
  Error = MCANV71_GetActualOperationMode(pComp, &OpMode);               // Get actual Operational Mode
  if (Error != ERR_NONE) return Error;                                  // If there is an error while calling MCAN_GetActualOperationMode() then return the error
  if (OpMode != MCAN_INITIALIZATION_MODE) return ERR__NEED_CONFIG_MODE; // Device must be in Configuration Mode to perform the configuration

  //--- Configure SID filters MRAM ---
  MCAN_SIDFC_Register SIDconf;
  SIDconf.SIDFC = MCAN_SIDFC_FILTER_LIST_SA_SET(*startAddr) | MCAN_SIDFC_LIST_SIZE_SET(sidElementsCount > MCAN_SID_FILTERS_MAX ? MCAN_SID_FILTERS_MAX : sidElementsCount);
  *startAddr += (sidElementsCount * MCAN_CAN_STANDARD_FILTER_SIZE);     // Update the address
  if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;     // Check of configured memory
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_SIDFC = SIDconf.SIDFC;    // Write configuration to the XIDAM register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_SIDFC, SIDconf.SIDFC);      // Write configuration to the XIDAM register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_SIDFC] = SIDconf.SIDFC;                  // Write in cache
#  endif
#endif

  //--- Configure EID filters MRAM ---
  MCAN_XIDFC_Register EIDconf;
  EIDconf.XIDFC = MCAN_XIDFC_FILTER_LIST_SA_SET(*startAddr) | MCAN_XIDFC_LIST_SIZE_SET(eidElementsCount > MCAN_EID_FILTERS_MAX ? MCAN_EID_FILTERS_MAX : eidElementsCount);
  *startAddr += (eidElementsCount * MCAN_CAN_EXTENDED_FILTER_SIZE);     // Update the address
  if (*startAddr > MCAN_RAM_END_ADDRESS) return ERR__OUT_OF_MEMORY;     // Check of configured memory
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_XIDFC = EIDconf.XIDFC;    // Write configuration to the XIDAM register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_XIDFC, EIDconf.XIDFC);      // Write configuration to the XIDAM register of an external controller
  if (Error != ERR_NONE) return Error;
#  ifdef MCAN_USE_CACHE
  pComp->__RegCache[MCAN_CACHE_XIDFC] = EIDconf.XIDFC;                  // Write in cache
#  endif
#endif
  return ERR_NONE;
}



//=============================================================================
// Configure a SID filter of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureSIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter)
{
#ifdef CHECK_NULL_PARAM
  if (confFilter == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (MCAN_FILTERS_CONFIGURED(pComp->InternalConfig) == false) return ERR__NOT_INITIALIZED; // If filters elements count in RAM is not configured, return an error
  if (confFilter->Filter > MCAN_SID_FILTERS_MAX) return ERR__OUT_OF_RANGE;
  MCAN_SIDFC_Register RegSIDFC;
  eERRORRESULT Error;

  //--- Get 11-bit filters RAM configuration ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  RegSIDFC.SIDFC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_SIDFC; // Read internal register
#elif defined(MCAN_USE_CACHE)
  RegSIDFC.SIDFC = pComp->__RegCache[MCAN_CACHE_SIDFC];               // Read in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_SIDFC, &RegSIDFC.SIDFC);   // Read register of an external device
  if (Error != ERR_NONE) return Error;
#endif

  //--- Check if the filter is disabled ---
  MCAN_StandardFilterObject FilterConf;
  uint32_t AddrFilter = MCAN_SIDFC_FILTER_LIST_SA_GET(RegSIDFC.SIDFC) + (confFilter->Filter * MCAN_CAN_STANDARD_FILTER_SIZE); // Select the address of the Filter in the RAM allocation
  Error = MCANV71_ReadRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_STANDARD_FILTER_SIZE); // Read actual flags configuration in the SID reserved space in the RAM allocation
  if (Error != ERR_NONE) return Error;                                // If there is an error while calling MCANV71_ReadRAM() then return the error
  if (MCAN_CAN_FILTS0_SFEC_GET(FilterConf.S0) != MCAN_DISABLE_FILTER)
  {
    FilterConf.S0 &= ~MCAN_CAN_FILTS0_SFEC_Mask;                      // Disable the filter
    Error = MCANV71_WriteRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_STANDARD_FILTER_SIZE); // Write the new flags configuration in the SID reserved space in the RAM allocation
    if (Error != ERR_NONE) return Error;                              // If there is an error while calling MCANV71_WriteRAM() then return the error
  }

  if (confFilter->EnableFilter)
  {
    //--- Check values ---
    if (confFilter->Type == MCAN_FILTER_MATCH_ID_RANGE_MASK) return ERR__NOT_SUPPORTED; // Only supported with Extended Message ID Filter Elements
    if ((confFilter->DualID.AcceptanceID1 & ~MCAN_CAN_FILTER_SID_MASK) > 0) return ERR__FILTER_TOO_LARGE;
    if ((confFilter->DualID.AcceptanceID2 & ~MCAN_CAN_FILTER_SID_MASK) > 0) return ERR__FILTER_TOO_LARGE;

    //=== Fill Filter Object register ===
    FilterConf.S0 = MCAN_CAN_FILTS0_SFT_SET(confFilter->Type) | MCAN_CAN_FILTS0_SFID1_SET(confFilter->DualID.AcceptanceID1);
    switch (confFilter->Config)                                       // Set Filter configuration (SFEC)
    {
      default:
      case MCAN_FILTER_NO_CONFIG:
        switch (confFilter->PointTo)
        {
          default:
          case MCAN_NO_FIFO_BUFF: return ERR__NOT_SUPPORTED;
          case MCAN_RX_FIFO0    : FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_STORE_TO_RX_FIFO_0); break;
          case MCAN_RX_FIFO1    : FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_STORE_TO_RX_FIFO_1); break;
          case MCAN_RX_BUFFER   :
            if (confFilter->IDbuffer.BufferPosition >= MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE; // Position in Rx Buffer shall be in an available element
            FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG)
                          |  MCAN_CAN_FILTS0_DBMSG_SET(MCAN_STORE_IN_RX_BUFFER)
                          |  MCAN_CAN_FILTS0_RXID_SET(confFilter->IDbuffer.BufferPosition);
            break;
        }
        break;

      case MCAN_FILTER_REJECT_ID       : FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_REJECT_ID); break;
      case MCAN_FILTER_AS_DEBUG_MESSAGE:
        if (confFilter->DebugID.BufferPosition >= MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE;       // Position in Rx Buffer shall be in an available element
        if ((confFilter->DebugID.DebugMessage == 0) || ((confFilter->DebugID.DebugMessage & ~0x3) > 0)) return ERR__CONFIGURATION; // Shall be 1, 2, or 3 to reflect DEBUG_MESSAGE_A, DEBUG_MESSAGE_B, or DEBUG_MESSAGE_C
        FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG)
                      |  MCAN_CAN_FILTS0_DBMSG_SET(confFilter->DebugID.DebugMessage)
                      |  MCAN_CAN_FILTS0_RXID_SET(confFilter->IDbuffer.BufferPosition);
        break;

      case MCAN_FILTER_SET_PRIORITY:
        switch (confFilter->PointTo)
        {
          default:                return ERR__NOT_SUPPORTED;
          case MCAN_NO_FIFO_BUFF: FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_SET_PRIORITY);                    break;
          case MCAN_RX_FIFO0    : FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_0); break;
          case MCAN_RX_FIFO1    : FilterConf.S0 |= MCAN_CAN_FILTS0_SFEC_SET(MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_1); break;
        }
        break;
    }
    if (MCAN_CAN_FILTS0_SFEC_GET(FilterConf.S0) != MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG) FilterConf.S0 |= MCAN_CAN_FILTS0_SFID2_SET(confFilter->DualID.AcceptanceID2); // Set second SID

    //=== Configure Filter control ===
    Error = MCANV71_WriteRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_STANDARD_FILTER_SIZE); // Write the new flags configuration in the SID reserved space in the RAM allocation
    if (Error != ERR_NONE) return Error;                                                              // If there is an error while calling MCANV71_WriteRAM() then return the error
  }
  return ERR_NONE;
}



//=============================================================================
// Configure a EID filter of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureEIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter)
{
#ifdef CHECK_NULL_PARAM
  if (confFilter == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (MCAN_FILTERS_CONFIGURED(pComp->InternalConfig) == false) return ERR__NOT_INITIALIZED; // If filters elements count in RAM is not configured, return an error
  eERRORRESULT Error;

  //--- Get 29-bit filters RAM configuration ---
  MCAN_XIDFC_Register RegXIDFC;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  RegXIDFC.XIDFC = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_XIDFC;    // Read internal register
#elif defined(MCAN_USE_CACHE)
  RegXIDFC.XIDFC = pComp->__RegCache[MCAN_CACHE_XIDFC];                  // Read in cache
#else
  Error = MCANV71_ReadREG32(pComp, RegMCAN_XIDFC, &RegXIDFC.XIDFC);      // Read register of an external device
  if (Error != ERR_NONE) return Error;
#endif

  //--- Check if the filter is disabled ---
  MCAN_ExtendedFilterObject FilterConf;
  uint32_t AddrFilter = MCAN_XIDFC_FILTER_LIST_SA_GET(RegXIDFC.XIDFC) + (confFilter->Filter * MCAN_CAN_EXTENDED_FILTER_SIZE); // Select the address of the Filter in the RAM allocation
  Error = MCANV71_ReadRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_EXTENDED_FILTER_SIZE); // Read actual flags configuration in the EID reserved space in the RAM allocation
  if (Error != ERR_NONE) return Error;                                   // If there is an error while calling MCANV71_ReadRAM() then return the error
  if (MCAN_CAN_FILTF0_EFEC_GET(FilterConf.F0.F0) != MCAN_DISABLE_FILTER)
  {
    FilterConf.F0.F0 &= ~MCAN_CAN_FILTF0_EFEC_Mask;                      // Disable the filter
    Error = MCANV71_WriteRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_EXTENDED_FILTER_SIZE); // Write the new flags configuration in the EID reserved space in the RAM allocation
    if (Error != ERR_NONE) return Error;                                 // If there is an error while calling MCANV71_WriteRAM() then return the error
  }

  if (confFilter->EnableFilter)
  {
    //--- Check values ---
    if ((confFilter->DualID.AcceptanceID1 & ~MCAN_CAN_FILTER_EID_AND_SID_MASK) > 0) return ERR__FILTER_TOO_LARGE;
    if ((confFilter->DualID.AcceptanceID2 & ~MCAN_CAN_FILTER_EID_AND_SID_MASK) > 0) return ERR__FILTER_TOO_LARGE;

    //=== Fill Filter Object register ===
    FilterConf.F0.F0 = MCAN_CAN_FILTF0_EFID1_SET(confFilter->DualID.AcceptanceID1);
    FilterConf.F1.F1 = 0;
    switch (confFilter->Type)                                           // Set Extended Filter Type (EFT)
    {
      case MCAN_FILTER_MATCH_ID_RANGE     : FilterConf.F1.F1 = MCAN_CAN_FILTF1_EFT_SET(MCAN_RANGE_FROM_FID1_TO_FDI2_NO_MASK);
      case MCAN_FILTER_MATCH_DUAL_ID      :
      case MCAN_FILTER_MATCH_ID_MASK      : FilterConf.F1.F1 = MCAN_CAN_FILTF1_EFT_SET(confFilter->Type);
      case MCAN_FILTER_MATCH_ID_RANGE_MASK: FilterConf.F1.F1 = MCAN_CAN_FILTF1_EFT_SET(MCAN_RANGE_FROM_FID1_TO_FDI2);
    }
    switch (confFilter->Config)                                         // Set Filter configuration (EFEC)
    {
      default:
      case MCAN_FILTER_NO_CONFIG:
        switch (confFilter->PointTo)
        {
          default:
          case MCAN_NO_FIFO_BUFF: return ERR__NOT_SUPPORTED;
          case MCAN_RX_FIFO0    : FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_STORE_TO_RX_FIFO_0); break;
          case MCAN_RX_FIFO1    : FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_STORE_TO_RX_FIFO_1); break;
          case MCAN_RX_BUFFER   :
            if (confFilter->IDbuffer.BufferPosition >= MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE; // Position in Rx Buffer shall be in an available element
            FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG);
            FilterConf.F1.F1 |= MCAN_CAN_FILTF1_DBMSG_SET(MCAN_STORE_IN_RX_BUFFER)
                             |  MCAN_CAN_FILTF1_RXID_SET(confFilter->IDbuffer.BufferPosition);
            break;
        }
        break;

      case MCAN_FILTER_REJECT_ID       : FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_REJECT_ID); break;
      case MCAN_FILTER_AS_DEBUG_MESSAGE:
        if (confFilter->DebugID.BufferPosition >= MCAN_RX_BUFFERS_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE;       // Position in Rx Buffer shall be in an available element
        if ((confFilter->DebugID.DebugMessage == 0) || ((confFilter->DebugID.DebugMessage & ~0x3) > 0)) return ERR__CONFIGURATION; // Shall be 1, 2, or 3 to reflect DEBUG_MESSAGE_A, DEBUG_MESSAGE_B, or DEBUG_MESSAGE_C
        FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG);
        FilterConf.F1.F1 |= MCAN_CAN_FILTF1_DBMSG_SET(confFilter->DebugID.DebugMessage)
                         |  MCAN_CAN_FILTF1_RXID_SET(confFilter->IDbuffer.BufferPosition);
        break;

      case MCAN_FILTER_SET_PRIORITY:
        switch (confFilter->PointTo)
        {
          default:                return ERR__NOT_SUPPORTED;
          case MCAN_NO_FIFO_BUFF: FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_SET_PRIORITY);                    break;
          case MCAN_RX_FIFO0    : FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_0); break;
          case MCAN_RX_FIFO1    : FilterConf.F0.F0 |= MCAN_CAN_FILTF0_EFEC_SET(MCAN_SET_PRIORITY_STORE_TO_RX_FIFO_1); break;
        }
        break;
    }
    if (MCAN_CAN_FILTF0_EFEC_GET(FilterConf.F0.F0) != MCAN_STORE_RX_BUFFER_OR_AS_DEBUG_MSG) FilterConf.F1.F1 |= MCAN_CAN_FILTF1_EFID2_SET(confFilter->DualID.AcceptanceID2); // Set second EID

    //=== Configure Filter control ===
    Error = MCANV71_WriteRAM(pComp, AddrFilter, &FilterConf.Bytes[0], MCAN_CAN_EXTENDED_FILTER_SIZE); // Write the new flags configuration in the EID reserved space in the RAM allocation
    if (Error != ERR_NONE) return Error;                                                              // If there is an error while calling MCANV71_WriteRAM() then return the error
  }
  return ERR_NONE;
}


//=============================================================================
// Configure a filter list of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureFilterList(MCANV71 *pComp, MCAN_Filter* const listFilter, size_t count)
{
#ifdef CHECK_NULL_PARAM
  if (listFilter == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (count == 0) return ERR_NONE;
  if (count > MCAN_FILTERS_MAX) return ERR__OUT_OF_RANGE;
  eERRORRESULT Error;

  //--- Configure filters ---
  for (size_t zFilter = 0; zFilter < count; ++zFilter)
  {
    switch (listFilter[zFilter].Match)
    {
      case MCAN_FILTER_MATCH_ONLY_SID: Error = MCANV71_ConfigureSIDfilter(pComp, &listFilter[zFilter]); break;
      case MCAN_FILTER_MATCH_ONLY_EID: Error = MCANV71_ConfigureEIDfilter(pComp, &listFilter[zFilter]); break;
      case MCAN_FILTER_MATCH_SID_EID : return ERR__NOT_SUPPORTED;
    }
    if (Error != ERR_NONE) return Error; // If there is an error while calling MCANV71_ConfigureFilter() functions then return the error
  }

  return ERR_NONE;
}



//=============================================================================
// Disable a Filter of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_DisableFilter(MCANV71 *pComp, uint16_t name, bool extendedID)
{
#if !defined(__cplusplus)
  MCAN_Filter ClearFilter = { .Filter = name, .EnableFilter = false, .Match = MCAN_FILTER_MATCH_SID_EID, .Type = MCAN_FILTER_MATCH_ID_MASK, .Config = MCAN_FILTER_NO_CONFIG, .PointTo = 0, .ExtendedID = extendedID, .IDandMask = { .AcceptanceID = 0, .AcceptanceMask = 0, }, };
#else
  MCAN_Filter ClearFilter = { name, false, MCAN_FILTER_MATCH_SID_EID, MCAN_FILTER_MATCH_ID_MASK, MCAN_FILTER_NO_CONFIG, 0, extendedID, { 0, 0, }, };
#endif // !__cplusplus
  eERRORRESULT Error = ERR_NONE;
  if (extendedID)
  {
    if (name >= MCAN_FILTER_EID_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE;
    Error = MCANV71_ConfigureEIDfilter(pComp, &ClearFilter);
  }
  else
  {
    if (name >= MCAN_FILTER_SID_SIZE_GET(pComp->InternalConfig)) return ERR__OUT_OF_RANGE;
    Error = MCANV71_ConfigureSIDfilter(pComp, &ClearFilter);
  }
  return Error;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Configure interrupt of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureInterrupt(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags, setMCAN_IntLineSelect intLineSelect)
{
  //--- Configure IE register ---
  uint32_t Reg;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Reg = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IE;                                                              // Read configuration from the IE register
  Reg &= MCAN_IR_FIFO_BUFF_TEF_FLAGS;                                                                                // Keep FIFO/Buffers/TEF flags only
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IE = Reg | ((uint32_t)interruptsFlags & ~MCAN_IR_FIFO_BUFF_TEF_FLAGS); // Write configuration to the IE register          
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_IE, &Reg);                                                   // Read configuration from the IE register of an external controller
  Reg &= MCAN_IR_FIFO_BUFF_TEF_FLAGS;                                                                                // Keep FIFO/Buffers/TEF flags only
  Error = MCANV71_WriteREG32(pComp, RegMCAN_IE, Reg | ((uint32_t)interruptsFlags & ~MCAN_IR_FIFO_BUFF_TEF_FLAGS));   // Write configuration to the IE register of an external controller
  if (Error != ERR_NONE) return Error;
#endif

  //--- Configure ILS register ---
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_ILS = (uint32_t)intLineSelect; // Write configuration to the ILS register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_ILS, (uint32_t)intLineSelect);   // Write configuration to the ILS register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// Get interrupt events of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents* interruptsFlags)
{
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  *interruptsFlags = (setMCAN_InterruptEvents)(((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IR); // Read all interrupt flags status
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_IR, (uint32_t*)interruptsFlags);        // Read all interrupt flags status of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// Clear interrupt events of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ClearInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags)
{

#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_IR = (uint32_t)interruptsFlags;            // Write configuration to the IR register
#else
  eERRORRESULT Error = MCANV71_WriteREG32(pComp, RegMCAN_IR, (uint32_t)interruptsFlags); // Write configuration to the IR register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}



//=============================================================================
// Get high priority message status of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetHighPriorityMessageStatus(MCANV71 *pComp, eMCAN_MessageStorageIndicator* const messageIndicator, bool* const isExtended, uint8_t* const bufferIndex, uint8_t* const filterIndex)
{
  MCAN_HPMS_Register Status;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  Status.HPMS = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_HPMS;            // Read high priority message status
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_HPMS, &Status.HPMS); // Read high priority message status of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  if (messageIndicator != NULL) *messageIndicator = (eMCAN_MessageStorageIndicator)MCAN_HPMS_MESSAGE_STORAGE_INDICATOR_GET(Status.HPMS); // Get message storage indicator
  if (isExtended       != NULL) *isExtended       = ((Status.HPMS & MCAN_HPMS_EXTENDED_FILTER_LIST) > 0);                                // Is the filter list of the matching filter element extended?
  if (bufferIndex      != NULL) *bufferIndex      = (uint8_t)MCAN_HPMS_BUFFER_INDEX_GET(Status.HPMS);                                    // Get buffer index
  if (filterIndex      != NULL) *filterIndex      = (uint8_t)MCAN_HPMS_FILTER_INDEX_GET(Status.HPMS);                                    // Get filter index
  return ERR_NONE;
}



//=============================================================================
// Get Rx buffer New Data events flags of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetRxBufferNewDataFlag(MCANV71 *pComp, uint32_t* const newDataIdx0_31, uint32_t* const newDataIdx32_63)
{
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
  if (newDataIdx0_31  != NULL)
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    *newDataIdx0_31 = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_NDAT1; // Get New Data flags index 0 to 31
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_NDAT1, newDataIdx0_31);     // Get New Data flags index 0 to 31 of an external controller
    if (Error != ERR_NONE) return Error;
#endif
  }
  if (newDataIdx32_63 != NULL)
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    *newDataIdx32_63 = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_NDAT2; // Get New Data flags index 32 to 63
#else
    Error = MCANV71_ReadREG32(pComp, RegMCAN_NDAT2, newDataIdx32_63);     // Get New Data flags index 32 to 63 of an external controller
    if (Error != ERR_NONE) return Error;
#endif
  }
  return ERR_NONE;
}



//=============================================================================
// Clear a Rx buffer New Data events flags of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ClearRxBufferNewDataFlag(MCANV71 *pComp, uint8_t index)
{
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
  if (index < 32)
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_NDAT1 = (1 << (index -  0)); // Write configuration to the NDAT1 register
#else
    Error = MCANV71_WriteREG32(pComp, RegMCAN_NDAT1, (1 << (index -  0)));   // Write configuration to the NDAT1 register of an external controller
    if (Error != ERR_NONE) return Error;
#endif
  }       
  else
  {
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
    ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_NDAT2 = (1 << (index - 32)); // Write configuration to the NDAT2 register
#else
    Error = MCANV71_WriteREG32(pComp, RegMCAN_NDAT2, (1 << (index - 32)));   // Write configuration to the NDAT2 register of an external controller
    if (Error != ERR_NONE) return Error;
#endif
  }    
  return ERR_NONE;
}



//=============================================================================
// Configure Tx buffer interrupts of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_ConfigureTxBufferInterrupts(MCANV71 *pComp, uint32_t transmitEnable, uint32_t cancelFinishEnable)
{
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  eERRORRESULT Error;
#endif
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBTIE = transmitEnable; // Write configuration to the TXBTIE register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_TXBTIE, transmitEnable);   // Write configuration to the TXBTIE register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_TXBCIE = cancelFinishEnable; // Write configuration to the TXBCIE register
#else
  Error = MCANV71_WriteREG32(pComp, RegMCAN_TXBCIE, cancelFinishEnable);   // Write configuration to the TXBCIE register of an external controller
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}








//**********************************************************************************************************************************************************
//=============================================================================
// Get transmit/receive error count and status of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetTransmitReceiveErrorCountAndStatus(MCANV71 *pComp, uint8_t* transmitErrorCount, uint8_t* receiveErrorCount, bool* receiveErrorPassive, uint8_t* canErrorLogging)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif

  MCAN_ECR_Register RegValue;
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  RegValue.ECR = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_ECR;            // Read value of the ECR register
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_ECR, &RegValue.ECR); // Read value of the ECR register
  if (Error != ERR_NONE) return Error;
#endif
  if (transmitErrorCount  != NULL) *transmitErrorCount  = MCAN_ECR_TRANSMIT_ERROR_COUNTER_GET(RegValue.ECR);           // Get Transmit Error Counter
  if (receiveErrorCount   != NULL) *receiveErrorCount   = MCAN_ECR_RECEIVE_ERROR_COUNTER_GET(RegValue.ECR);            // Get Receive Error Counter
  if (receiveErrorPassive != NULL) *receiveErrorPassive = ((RegValue.ECR & MCAN_ECR_RECEIVE_ERROR_REACH_PASSIVE) > 0); // Get Sub-step of Core Release
  if (canErrorLogging     != NULL) *canErrorLogging     = MCAN_ECR_CAN_ERROR_LOGGING_GET(RegValue.ECR);                // Get CAN Error Logging (cleared by the reading of the ECR register)
  return ERR_NONE;
}



//=============================================================================
// Get Bus diagnostic of the MCAN peripheral
//=============================================================================
eERRORRESULT MCANV71_GetBusDiagnostic(MCANV71 *pComp, MCAN_PSR_Register* const busDiagnostic)
{
#ifdef CHECK_NULL_PARAM
  if (busDiagnostic == NULL) return ERR__PARAMETER_ERROR;
#endif
#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  busDiagnostic->PSR = ((MCAN_HardReg*)(pComp->Instance))->RegMCAN_PSR;                      // Read value of the PSR register
#else
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_PSR, (uint32_t*)busDiagnostic->PSR); // Read value of the PSR register
  if (Error != ERR_NONE) return Error;
#endif
  return ERR_NONE;
}





//**********************************************************************************************************************************************************
//=============================================================================
// Payload to Byte Count
//=============================================================================
uint8_t MCANV71_PayloadToByte(eMCAN_PayloadSize payload)
{
//  const uint8_t PAYLOAD_TO_VALUE[MCAN_PAYLOAD_COUNT] = {8, 12, 16, 20, 24, 32, 48, 64};
//  if (payload < MCAN_PAYLOAD_COUNT) return PAYLOAD_TO_VALUE[(size_t)payload];
//  return 0;
  switch (payload)
  {
    case MCAN_8_BYTES : return  8; // Payload  8 data bytes
    case MCAN_12_BYTES: return 12; // Payload 12 data bytes
    case MCAN_16_BYTES: return 16; // Payload 16 data bytes
    case MCAN_20_BYTES: return 20; // Payload 20 data bytes
    case MCAN_24_BYTES: return 24; // Payload 24 data bytes
    case MCAN_32_BYTES: return 32; // Payload 32 data bytes
    case MCAN_48_BYTES: return 48; // Payload 48 data bytes
    case MCAN_64_BYTES: return 64; // Payload 64 data bytes
    default: return 0;
  }
  return 0;
}



//=============================================================================
// Data Length Content to Byte Count
//=============================================================================
uint8_t MCANV71_DLCToByte(eMCAN_DataLength dlc, bool isCANFD)
{
  const uint8_t CAN20_DLC_TO_VALUE[MCAN_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8,  8,  8,  8,  8,  8,  8,  8};
  const uint8_t CANFD_DLC_TO_VALUE[MCAN_DLC_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  if (dlc >= MCAN_DLC_COUNT) return 0;
  if (isCANFD) return CANFD_DLC_TO_VALUE[(size_t)dlc];
  return CAN20_DLC_TO_VALUE[(size_t)dlc];
}





//**********************************************************************************************************************************************************
//=============================================================================
// [STATIC] Counting 1-bits in a uint32
//=============================================================================
uint32_t __CalcBitCount(uint32_t val)
{
  val = val - ((val >> 1) & 0x55555555);
  val = (val & 0x33333333) + ((val >> 2) & 0x33333333);
  val = (val + (val >> 4)) & 0x0F0F0F0F;
  val = val + (val >> 8);
  val = val + (val >> 16);
  return val & 0x0000003F;
}





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------