/*!*****************************************************************************
 * @file    MCAN_V71.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    22/05/2021
 * @brief   MCAN driver for ATSAMV71 MCUs
 *
 ******************************************************************************/
/* @page License
 *
 * Copyright (c) 2020-2022 Fabien MAILLY
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
 * 1.0.0    Release version
 *****************************************************************************/
#ifndef MCAN_V71_H_INC
#define MCAN_V71_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#include "CAN_common.h"
#include "MCAN_core.h"
#include "ErrorsDef.h"
#include "Main.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#define MCANV71_SYSTEM_CLOCK_DIVIDER  ( 2 ) //!< Sysclock divider before entering the MCANV71 peripheral
#define MCANV71_CLOCK_MAX             ( 150000000 ) //!< Max MCANV71 peripheral clock is 150MHz

#define MCANV71_RAM_START_ADDRESS     ( (uint32_t)pComp->RAMallocation )
#define MCANV71_MAX_RAM               ( 4352u * sizeof(uint32_t) ) //!< Max possible RAM use on ATSAMV71 is 4352 words of 32 bits
#define MCANV71_MAX_RAM_ADDRESS       ( MCANV71_RAM_START_ADDRESS + (uint32_t)pComp->RAMsize )

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// MCAN for ATSAMV71 Driver API
//********************************************************************************************************************

#define MCANV71_GetSIDfilterElementCount(pComp)   ( MCAN_FILTER_SID_SIZE_GET((pComp)->_MCAN.InternalConfig) ) //!< Get the SID filter elements count configured for the pComp
#define MCANV71_GetEIDfilterElementCount(pComp)   ( MCAN_FILTER_EID_SIZE_GET((pComp)->_MCAN.InternalConfig) ) //!< Get the EID filter elements count configured for the pComp

#define MCANV71_GetSIDfilterTotalByteSize(pComp)  ( MCANV71_GetSIDfilterElementCount(pComp) * MCAN_CAN_STANDARD_FILTER_SIZE ) //!< Get the SID filter total byte size
#define MCANV71_GetEIDfilterTotalByteSize(pComp)  ( MCANV71_GetEIDfilterElementCount(pComp) * MCAN_CAN_EXTENDED_FILTER_SIZE ) //!< Get the EID filter total byte size

//-----------------------------------------------------------------------------

typedef struct MCANV71 MCANV71; //!< Typedef of MCAN peripheral object structure

//-----------------------------------------------------------------------------

/*! @brief Function that gives the current millisecond of the system to the driver
 *
 * This function will be called when the driver needs to get current millisecond
 * @return Returns the current millisecond of the system
 */
typedef uint32_t (*GetCurrentms_Func)(void);

//-----------------------------------------------------------------------------

//! MCAN for ATSAMV71 peripheral object structure
struct MCANV71
{
  void *UserDriverData;              //!< Optional, can be used to store driver data or NULL

  //--- Driver configuration ---
  setMCAN_DriverConfig DriverConfig; //!< Driver configuration, by default it is MCAN_DRIVER_NORMAL_USE. Configuration can be OR'ed

  //--- MCAN interface ---
  MCAN_Interface _MCAN;              //!< DO NOT USE OR CHANGE THIS VALUE, WILL BE FILLED AT INIT WITH PROPER VALUES. This is the MCAN_Interface descriptor that will be used to communicate with the MCAN core

  //--- MCAN peripheral ---
  volatile Mcan* Instance;           //!< This is the pointer to the peripheral controller address
  uint8_t* RAMallocation;            //!< Message and Filters RAM allocation pointer
  size_t RAMsize;                    //!< RAM allocation size

  //--- Time call function ---
  GetCurrentms_Func fnGetCurrentms;  //!< This function will be called when the driver need to get current millisecond
};

//! This unique ID is a helper for pointer recognition when using USE_GENERICS_DEFINED for generic call of GPIO or PORT use (using GPIO_Interface.h)
#define MCANV71_UNIQUE_ID  ( (((uint32_t)'M' << 0) ^ ((uint32_t)'C' << 4) ^ ((uint32_t)'A' << 8) ^ ((uint32_t)'N' << 14) ^ ((uint32_t)'V' << 18) ^ ((uint32_t)'7' << 22) ^ ((uint32_t)'1' << 26)) + (sizeof(struct MCANV71) << 19) )

//-----------------------------------------------------------------------------



//********************************************************************************************************************


/*! Configure the MCAN peripheral clock
 * @param[in] *pComp Is the pointed structure of the peripheral to be use
 * @param[out] *peripheralClock Is the peripheral clock
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigurePeripheralClocks(MCANV71 *pComp, uint32_t* const peripheralClock);

/*! Configure the MCAN DMA base address. Usually the 16-bit MSB of the RAM address
 * @param[in] *pComp Is the pointed structure of the peripheral to be use
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureMCANbaseAddress(MCANV71 *pComp);

//********************************************************************************************************************


/*! @brief MCAN on SAMV71 peripheral initialization
 *
 * This function initializes the MCAN driver and call the initialization of the interface driver (SPI). It checks parameters and perform a RESET
 * Next this function configures the MCAN Controller and the CAN controller
 * @param[in] *pComp Is the pointed structure of the peripheral to be initialized
 * @param[in] *pConf Is the pointed structure of the peripheral configuration
 * @param[in] *listFIFO Is the FIFO/Buffers list to configure in RAM
 * @param[in] *count Is the element count of the listFIFO array
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT Init_MCANV71(MCANV71 *pComp, const MCAN_Config* const pConf, const MCAN_FIFObuff* const listFIFO, size_t listFIFOcount);

//********************************************************************************************************************


/*! @brief Read from a 32-bits register of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the register's address where data will be read (need 32-bits alignment)
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReadREG32(void *pComp, const uint32_t address, uint32_t* data);

/*! @brief Write to a 32-bits register of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the register's address where data will be written (need 32-bits alignment)
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_WriteREG32(void *pComp, const uint32_t address, const uint32_t data);

//-----------------------------------------------------------------------------


/*! @brief Read from allocated RAM data of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the address where data will be read in the RAM allocation (need 32-bits alignment)
 * @param[out] *data Is where the data will be stored
 * @param[in] count Is the count of 32-bits data to read
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReadRAM(void *_pComp, const uint32_t address, uint8_t* data, const uint16_t count);

/*! @brief Write to allocated RAM data of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the address where data will be written in the RAM allocation (need 32-bits alignment)
 * @param[in] data Is the data to write
 * @param[in] count Is the count of 32-bits data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_WriteRAM(void *_pComp, const uint32_t address, const uint8_t* data, const uint16_t count);



//********************************************************************************************************************
// MCAN links Driver API
//********************************************************************************************************************

inline eERRORRESULT MCANV71_CheckEndianness(MCANV71 *pComp) //!< @see #MCAN_CheckEndianness
{ return MCAN_CheckEndianness(&pComp->_MCAN); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_GetMCANcoreID(MCANV71 *pComp, uint8_t* const mcanCoreId, uint8_t* const mcanStep, uint8_t* const mcanSubStep, uint8_t* const mcanYear, uint8_t* const mcanMonth, uint8_t* const mcanDay) //!< @see #MCAN_GetMCANcoreID
{ return MCAN_GetMCANcoreID(&pComp->_MCAN, mcanCoreId, mcanStep, mcanSubStep, mcanYear, mcanMonth, mcanDay); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ReadCustomerRegister(MCANV71 *pComp, uint32_t* const data) //!< @see #MCAN_ReadCustomerRegister
{ return MCAN_ReadCustomerRegister(&pComp->_MCAN, data); };
inline eERRORRESULT MCANV71_WriteCustomerRegister(MCANV71 *pComp, const uint32_t data) //!< @see #MCAN_WriteCustomerRegister
{ return MCAN_WriteCustomerRegister(&pComp->_MCAN, data); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_TransmitMessageObject(MCANV71 *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize, eMCAN_FIFObuffer toFIFObuff, uint8_t index) //!< @see #MCAN_TransmitMessageObject
{ return MCAN_TransmitMessageObject(&pComp->_MCAN, messageObjectToSend, objectSize, toFIFObuff, index); };
inline eERRORRESULT MCANV71_TransmitMessageObjectToTXQ(MCANV71 *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize) //!< @see #MCAN_TransmitMessageObjectToTXQ
{ return MCAN_TransmitMessageObjectToTXQ(&pComp->_MCAN, messageObjectToSend, objectSize); };
inline eERRORRESULT MCANV71_TransmitMessage(MCANV71 *pComp, const CAN_CANMessage* const messageToSend, eMCAN_FIFObuffer toFIFObuff, uint8_t index) //!< @see #MCAN_TransmitMessage
{ return MCAN_TransmitMessage(&pComp->_MCAN, messageToSend, toFIFObuff, index); };
inline eERRORRESULT MCANV71_TransmitMessageToTXQ(MCANV71 *pComp, const CAN_CANMessage* const messageToSend) //!< @see #MCAN_TransmitMessageToTXQ
{ return MCAN_TransmitMessageToTXQ(&pComp->_MCAN, messageToSend); };

inline eERRORRESULT MCANV71_ReceiveMessageObject(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize, eMCAN_FIFObuffer fromFIFObuff, uint8_t index) //!< @see #MCAN_ReceiveMessageObject
{ return MCAN_ReceiveMessageObject(&pComp->_MCAN, messageObjectGet, objectSize, fromFIFObuff, index); };
inline eERRORRESULT MCANV71_ReceiveMessageObjectFromTEF(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize) //!< @see #MCAN_ReceiveMessageObjectFromTEF
{ return MCAN_ReceiveMessageObjectFromTEF(&pComp->_MCAN, messageObjectGet, objectSize); };
inline eERRORRESULT MCANV71_ReceiveMessage(MCANV71 *pComp, CAN_CANMessage* const messageGet, eMCAN_PayloadSize payloadSize, uint32_t* const timeStamp, uint8_t* const filterHitIdx, eMCAN_FIFObuffer fromFIFObuff, uint8_t index) //!< @see #MCAN_ReceiveMessage
{ return MCAN_ReceiveMessage(&pComp->_MCAN, messageGet, payloadSize, timeStamp, filterHitIdx, fromFIFObuff, index); };
inline eERRORRESULT MCANV71_ReceiveMessageFromTEF(MCANV71 *pComp, CAN_CANMessage* const messageGet, uint32_t* const timeStamp, uint8_t* const filterHitIdx) //!< @see #MCAN_ReceiveMessageFromTEF
{ return MCAN_ReceiveMessageFromTEF(&pComp->_MCAN, messageGet, timeStamp, filterHitIdx); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureWatchdogRAM(MCANV71 *pComp, uint8_t messageRAMwatchdogConf) //!< @see #MCAN_ConfigureWatchdogRAM
{ return MCAN_ConfigureWatchdogRAM(&pComp->_MCAN, messageRAMwatchdogConf); };
inline eERRORRESULT MCANV71_GetWatchdogRAMvalue(MCANV71 *pComp, uint8_t* const messageRAMwatchdogValue) //!< @see #MCAN_GetWatchdogRAMvalue
{ return MCAN_GetWatchdogRAMvalue(&pComp->_MCAN, messageRAMwatchdogValue); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureINTlines(MCANV71 *pComp, bool enableLineINT0, bool enableLineINT1) //!< @see #MCAN_ConfigureINTlines
{ return MCAN_ConfigureINTlines(&pComp->_MCAN, enableLineINT0, enableLineINT1); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_SetBitTimeConfiguration(MCANV71 *pComp, const CAN_BitTimeConfig* const pConf) //!< @see #MCAN_SetBitTimeConfiguration
{ return MCAN_SetBitTimeConfiguration(&pComp->_MCAN, pConf); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureWriteProtection(MCANV71 *pComp, bool enable, uint32_t* const lastRegRead) //!< @see #MCAN_ConfigureWriteProtection
{ return MCAN_ConfigureWriteProtection(&pComp->_MCAN, enable, lastRegRead); };
inline eERRORRESULT MCANV71_SetWriteProtection(MCANV71 *pComp) //!< @see #MCAN_SetWriteProtection
{ return MCAN_SetWriteProtection(&pComp->_MCAN); };
inline eERRORRESULT MCANV71_RemoveWriteProtection(MCANV71 *pComp) //!< @see #MCAN_RemoveWriteProtection
{ return MCAN_RemoveWriteProtection(&pComp->_MCAN); };

inline eERRORRESULT MCANV71_GetActualOperationMode(MCANV71 *pComp, eMCAN_OperationMode* const actualMode) //!< @see #MCAN_GetActualOperationMode
{ return MCAN_GetActualOperationMode(&pComp->_MCAN, actualMode); };
inline eERRORRESULT MCANV71_RequestOperationMode(MCANV71 *pComp, eMCAN_OperationMode newMode) //!< @see #MCAN_RequestOperationMode
{ return MCAN_RequestOperationMode(&pComp->_MCAN, newMode); };
inline eERRORRESULT MCANV71_StartCAN20(MCANV71 *pComp) //!< @see #MCAN_StartCAN20
{ return MCAN_StartCAN20(&pComp->_MCAN); };
inline eERRORRESULT MCANV71_StartCANFD(MCANV71 *pComp) //!< @see #MCAN_StartCANFD
{ return MCAN_StartCANFD(&pComp->_MCAN); };
inline eERRORRESULT MCANV71_StartCANListenOnly(MCANV71 *pComp) //!< @see #MCAN_StartCANListenOnly
{ return MCAN_StartCANListenOnly(&pComp->_MCAN); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureTest(MCANV71 *pComp, bool enableLoopback, eMCAN_TestTxPin txPinControl) //!< @see #MCAN_ConfigureTest
{ return MCAN_ConfigureTest(&pComp->_MCAN, enableLoopback, txPinControl); };
inline eERRORRESULT MCANV71_GetTest(MCANV71 *pComp, eMCAN_TestRxPin* const rxPin, uint8_t* const txNumPrepared, bool* const preparedValid, uint8_t* const txNumStarted, bool* const startedValid) //!< @see #MCAN_GetTest
{ return MCAN_GetTest(&pComp->_MCAN, rxPin, txNumPrepared, preparedValid, txNumStarted, startedValid); };
inline eERRORRESULT MCANV71_ConfigureCANController(MCANV71 *pComp, setMCAN_CANCtrlFlags flags) //!< @see #MCAN_ConfigureCANController
{ return MCAN_ConfigureCANController(&pComp->_MCAN, flags); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_EnterSleepMode(MCANV71 *pComp) //!< @see #MCAN_EnterSleepMode
{ return MCAN_EnterSleepMode(&pComp->_MCAN); };
inline eERRORRESULT MCANV71_IsDeviceInSleepMode(MCANV71 *pComp, bool* const isInSleepMode) //!< @see #MCAN_IsDeviceInSleepMode
{ return MCAN_IsDeviceInSleepMode(&pComp->_MCAN, isInSleepMode); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureTimeStamp(MCANV71 *pComp, eMCAN_TimeStampSelect timestampSource, uint8_t prescaler) //!< @see #MCAN_ConfigureTimeStamp
{ return MCAN_ConfigureTimeStamp(&pComp->_MCAN, timestampSource, prescaler); };
inline eERRORRESULT MCANV71_SetTimeStamp(MCANV71 *pComp, uint32_t value) //!< @see #MCAN_SetTimeStamp
{ return MCAN_SetTimeStamp(&pComp->_MCAN, value); };
inline eERRORRESULT MCANV71_GetTimeStamp(MCANV71 *pComp, uint32_t* value) //!< @see #MCAN_GetTimeStamp
{ return MCAN_GetTimeStamp(&pComp->_MCAN, value); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureTimeoutCounter(MCANV71 *pComp, bool enableTC, eMCAN_TimeoutSelect timeoutSelect, uint16_t period) //!< @see #MCAN_ConfigureTimeoutCounter
{ return MCAN_ConfigureTimeoutCounter(&pComp->_MCAN, enableTC, timeoutSelect, period); };
inline eERRORRESULT MCANV71_SetTimeoutCounter(MCANV71 *pComp, uint32_t value) //!< @see #MCAN_SetTimeoutCounter
{ return MCAN_SetTimeoutCounter(&pComp->_MCAN, value); };
inline eERRORRESULT MCANV71_GetTimeoutCounter(MCANV71 *pComp, uint32_t* value) //!< @see #MCAN_GetTimeoutCounter
{ return MCAN_GetTimeoutCounter(&pComp->_MCAN, value); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_GetFIFOStatus(MCANV71 *pComp, eMCAN_FIFObuffer name, setMCAN_FIFObufferstatus* const statusFlags) //!< @see #MCAN_GetFIFOStatus
{ return MCAN_GetFIFOStatus(&pComp->_MCAN, name, statusFlags); };
inline eERRORRESULT MCANV71_GetNextMessageAddressFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex) //!< @see #MCAN_GetNextMessageAddressFIFO
{ return MCAN_GetNextMessageAddressFIFO(&pComp->_MCAN, name, level, getIndex, putIndex); };
inline eERRORRESULT MCANV71_GetNextMessageAddressTEF(MCANV71 *pComp, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex) //!< @see #MCAN_GetNextMessageAddressTEF
{ return MCAN_GetNextMessageAddressTEF(&pComp->_MCAN, level, getIndex, putIndex); };
inline eERRORRESULT MCANV71_AcknowledgeFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t acknowledgeIndex) //!< @see #MCAN_AcknowledgeFIFO
{ return MCAN_AcknowledgeFIFO(&pComp->_MCAN, name, acknowledgeIndex); };
inline eERRORRESULT MCANV71_AcknowledgeTEF(MCANV71 *pComp, uint32_t acknowledgeIndex) //!< @see #MCAN_AcknowledgeTEF
{ return MCAN_AcknowledgeTEF(&pComp->_MCAN, acknowledgeIndex); };
inline eERRORRESULT MCANV71_SetTxBufferAddRequest(MCANV71 *pComp, uint32_t bufferIndex) //!< @see #MCAN_SetTxBufferAddRequest
{ return MCAN_SetTxBufferAddRequest(&pComp->_MCAN, bufferIndex); };
inline eERRORRESULT MCANV71_GetAllTxBufferRequestPending(MCANV71 *pComp, uint32_t* const requestPending) //!< @see #MCAN_GetAllTxBufferRequestPending
{ return MCAN_GetAllTxBufferRequestPending(&pComp->_MCAN, requestPending); };
inline eERRORRESULT MCANV71_SetMultipleTxBufferCancellationRequest(MCANV71 *pComp, uint32_t multipleRequest) //!< @see #MCAN_SetMultipleTxBufferCancellationRequest
{ return MCAN_SetMultipleTxBufferCancellationRequest(&pComp->_MCAN, multipleRequest); };
inline eERRORRESULT MCANV71_SetTxBufferCancellationRequest(MCANV71 *pComp, uint32_t bufferIndex) //!< @see #MCAN_SetTxBufferCancellationRequest
{ return MCAN_SetTxBufferCancellationRequest(&pComp->_MCAN, bufferIndex); };
inline eERRORRESULT MCANV71_GetAllTxBufferTransmitOccured(MCANV71 *pComp, uint32_t* const transmissionOccured) //!< @see #MCAN_GetAllTxBufferTransmitOccured
{ return MCAN_GetAllTxBufferTransmitOccured(&pComp->_MCAN, transmissionOccured); };
inline eERRORRESULT MCANV71_GetAllTxBufferCancellationFinished(MCANV71 *pComp, uint32_t* const cancellationFinished) //!< @see #MCAN_GetAllTxBufferCancellationFinished
{ return MCAN_GetAllTxBufferCancellationFinished(&pComp->_MCAN, cancellationFinished); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureGlobalFilters(MCANV71 *pComp, bool rejectAllStandardIDs, bool rejectAllExtendedIDs, eMCAN_AcceptNonMatching nonMatchingStandardID, eMCAN_AcceptNonMatching nonMatchingExtendedID) //!< @see #MCAN_ConfigureGlobalFilters
{ return MCAN_ConfigureGlobalFilters(&pComp->_MCAN, rejectAllStandardIDs, rejectAllExtendedIDs, nonMatchingStandardID, nonMatchingExtendedID); };
inline eERRORRESULT MCANV71_SetEIDrangeFilterMask(MCANV71 *pComp, uint32_t andMask) //!< @see #MCAN_SetEIDrangeFilterMask
{ return MCAN_SetEIDrangeFilterMask(&pComp->_MCAN, andMask); };
inline eERRORRESULT MCANV71_ConfigureSIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter) //!< @see #MCAN_ConfigureSIDfilter
{ return MCAN_ConfigureSIDfilter(&pComp->_MCAN, confFilter); };
inline eERRORRESULT MCANV71_ConfigureEIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter) //!< @see #MCAN_ConfigureEIDfilter
{ return MCAN_ConfigureEIDfilter(&pComp->_MCAN, confFilter); };
inline eERRORRESULT MCANV71_ConfigureFilterList(MCANV71 *pComp, MCAN_Filter* const listFilter, size_t count) //!< @see #MCAN_ConfigureFilterList
{ return MCAN_ConfigureFilterList(&pComp->_MCAN, listFilter, count); };
inline eERRORRESULT MCANV71_DisableFilter(MCANV71 *pComp, uint16_t name, bool extendedID) //!< @see #MCAN_DisableFilter
{ return MCAN_DisableFilter(&pComp->_MCAN, name, extendedID); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_ConfigureInterrupt(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags, setMCAN_IntLineSelect intLineSelect) //!< @see #MCAN_ConfigureInterrupt
{ return MCAN_ConfigureInterrupt(&pComp->_MCAN, interruptsFlags, intLineSelect); };
inline eERRORRESULT MCANV71_GetInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents* interruptsFlags) //!< @see #MCAN_GetInterruptEvents
{ return MCAN_GetInterruptEvents(&pComp->_MCAN, interruptsFlags); };
inline eERRORRESULT MCANV71_ClearInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags) //!< @see #MCAN_ClearInterruptEvents
{ return MCAN_ClearInterruptEvents(&pComp->_MCAN, interruptsFlags); };
inline eERRORRESULT MCANV71_GetHighPriorityMessageStatus(MCANV71 *pComp, eMCAN_MessageStorageIndicator* const messageIndicator, bool* const isExtended, uint8_t* const bufferIndex, uint8_t* const filterIndex) //!< @see #MCAN_GetHighPriorityMessageStatus
{ return MCAN_GetHighPriorityMessageStatus(&pComp->_MCAN, messageIndicator, isExtended, bufferIndex, filterIndex); };
inline eERRORRESULT MCANV71_GetRxBufferNewDataFlag(MCANV71 *pComp, uint32_t* const newDataIdx0_31, uint32_t* const newDataIdx32_63) //!< @see #MCAN_GetRxBufferNewDataFlag
{ return MCAN_GetRxBufferNewDataFlag(&pComp->_MCAN, newDataIdx0_31, newDataIdx32_63); };
inline eERRORRESULT MCANV71_ClearRxBufferNewDataFlag(MCANV71 *pComp, uint8_t index) //!< @see #MCAN_ClearRxBufferNewDataFlag
{ return MCAN_ClearRxBufferNewDataFlag(&pComp->_MCAN, index); };
inline eERRORRESULT MCANV71_ConfigureTxBufferInterrupts(MCANV71 *pComp, uint32_t transmitEnable, uint32_t cancelFinishEnable) //!< @see #MCAN_ConfigureTxBufferInterrupts
{ return MCAN_ConfigureTxBufferInterrupts(&pComp->_MCAN, transmitEnable, cancelFinishEnable); };

//********************************************************************************************************************

inline eERRORRESULT MCANV71_GetTransmitReceiveErrorCountAndStatus(MCANV71 *pComp, uint8_t* transmitErrorCount, uint8_t* receiveErrorCount, bool* receiveErrorPassive, uint8_t* canErrorLogging) //!< @see #MCAN_GetTransmitReceiveErrorCountAndStatus
{ return MCAN_GetTransmitReceiveErrorCountAndStatus(&pComp->_MCAN, transmitErrorCount, receiveErrorCount, receiveErrorPassive, canErrorLogging); };
inline eERRORRESULT MCANV71_GetBusDiagnostic(MCANV71 *pComp, MCAN_PSR_Register* const busDiagnostic) //!< @see #MCAN_GetBusDiagnostic
{ return MCAN_GetBusDiagnostic(&pComp->_MCAN, busDiagnostic); };

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCAN_V71_H_INC */