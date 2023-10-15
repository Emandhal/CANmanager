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

// DO NOT CHANGE THE FOLLOWING DEFINES
#define MCAN_INTERNAL_CAN_CONTROLLER // This is the configuration for internal MCU MCAN controllers. If commented, this is an external MCAN controller
#define MCAN_MAX_POSSIBLE_RAM      ( 4352u * sizeof(uint32_t) ) //! Max possible RAM use on ATSAMV71 is 4352 words of 32 bits
#define MCAN_SYSTEM_CLOCK_DIVIDER  ( 2 ) //! Sysclock divider before entering the MCAN peripheral
#define MCAN_CLOCK_MAX             ( 150000000 ) //! Max MCAN peripheral clock is 150MHz

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#include "CAN_common.h"
#include "MCAN_core.h"
#include "ErrorsDef.h"
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
#  include "SPI_Interface.h"
#endif
//-----------------------------------------------------------------------------
#ifdef __cplusplus
  extern "C" {
#endif
//-----------------------------------------------------------------------------

//! Driver configuration enum
typedef enum
{
  MCAN_DRIVER_NORMAL_USE        = 0x00, //!< Use the driver with no special verifications, just settings verifications (usually the fastest mode)
  MCAN_DRIVER_INIT_CHECK_RAM    = 0x04, //!< Check RAM at initialization by writing some data and checking them on all the RAM range (slower at initialization, take a long time)
  MCAN_DRIVER_INIT_SET_RAM_AT_0 = 0x08, //!< Set all bytes of the RAM to 0x00 (slower at initialization)
} eMCAN_DriverConfig;

typedef eMCAN_DriverConfig setMCAN_DriverConfig; //! Set of Driver configuration (can be OR'ed)

//-----------------------------------------------------------------------------




//********************************************************************************************************************
// MCAN for ATSAMV71 Driver API
//********************************************************************************************************************

#define MCAN_FILTERS_CONFIG_FLAG           ( 0x1u << 0 ) // Filter flag for Driver Internal data flags
#define MCAN_FILTERS_CONFIGURED(val)       ( ((val) & MCAN_FILTERS_CONFIG_FLAG) > 0 ) // Are filters configured?
#define MCAN_FIFOS_BUFF_CONFIG_FLAG        ( 0x1u << 1 ) // FIFOs/Buffers flag for Driver Internal data flags
#define MCAN_FIFOS_BUFF_CONFIGURED(val)    ( ((val) & MCAN_FIFOS_BUFF_CONFIG_FLAG) > 0 ) // Are FIFOs/Buffers configured?
#define MCAN_RX_BUFFERS_SIZE_Pos           2
#define MCAN_RX_BUFFERS_SIZE_Mask          (0x7Fu << MCAN_RX_BUFFERS_SIZE_Pos)
#define MCAN_RX_BUFFERS_SIZE_GET(value)    (((uint32_t)(value) & MCAN_RX_BUFFERS_SIZE_Mask) >> MCAN_RX_BUFFERS_SIZE_Pos) // Get Rx buffer size in the driver's InternalConfig
#define MCAN_RX_BUFFERS_SIZE_SET(value)    (((uint32_t)(value) << MCAN_RX_BUFFERS_SIZE_Pos) & MCAN_RX_BUFFERS_SIZE_Mask) // Set Rx buffer size in the driver's InternalConfig
#define MCAN_RX_BUFFERS_SIZE_CLEAR(value)  (value) &= ~MCAN_RX_BUFFERS_SIZE_Mask // Clear the Rx buffer position
#define MCAN_FILTER_SID_SIZE_Pos           9
#define MCAN_FILTER_SID_SIZE_Mask          (0xFFu << MCAN_FILTER_SID_SIZE_Pos)
#define MCAN_FILTER_SID_SIZE_GET(value)    (((uint32_t)(value) & MCAN_FILTER_SID_SIZE_Mask) >> MCAN_FILTER_SID_SIZE_Pos) // Get SID filters size in the driver's InternalConfig
#define MCAN_FILTER_SID_SIZE_SET(value)    (((uint32_t)(value) << MCAN_FILTER_SID_SIZE_Pos) & MCAN_FILTER_SID_SIZE_Mask) // Set SID filters size in the driver's InternalConfig
#define MCAN_FILTER_SID_SIZE_CLEAR(value)  (value) &= ~MCAN_FILTER_SID_SIZE_Mask // Clear the SID filters position
#define MCAN_FILTER_EID_SIZE_Pos           17
#define MCAN_FILTER_EID_SIZE_Mask          (0x7Fu << MCAN_FILTER_EID_SIZE_Pos)
#define MCAN_FILTER_EID_SIZE_GET(value)    (((uint32_t)(value) & MCAN_FILTER_EID_SIZE_Mask) >> MCAN_FILTER_EID_SIZE_Pos) // Get EID filters size in the driver's InternalConfig
#define MCAN_FILTER_EID_SIZE_SET(value)    (((uint32_t)(value) << MCAN_FILTER_EID_SIZE_Pos) & MCAN_FILTER_EID_SIZE_Mask) // Set EID filters size in the driver's InternalConfig
#define MCAN_FILTER_EID_SIZE_CLEAR(value)  (value) &= ~MCAN_FILTER_EID_SIZE_Mask // Clear the EID filters position
#define MCAN_DEV_PS_Pos                    28
#define MCAN_DEV_PS_Mask                   (0x3u << MCAN_DEV_PS_Pos)
#define MCAN_DEV_PS_SET(value)             (((uint32_t)(value) << MCAN_DEV_PS_Pos) & MCAN_DEV_PS_Mask)                    // Set Device Power State
#define MCAN_DEV_PS_GET(value)             (eMCAN_PowerStates)(((uint32_t)(value) & MCAN_DEV_PS_Mask) >> MCAN_DEV_PS_Pos) // Get Device Power State
#define MCAN_16BIT_MM_ENABLED              ( 1 << 30 ) // This value is used inside the driver (MCANV71.InternalConfig) to indicate if the Wide Message Marker is configured
#define MCAN_CANFD_ENABLED                 ( 1 << 31 ) // This value is used inside the driver (MCANV71.InternalConfig) to indicate if the CANFD is configured

#define MCANV71_GetSIDfilterElementCount(pComp)  ( MCAN_FILTER_SID_SIZE_GET((pComp)->InternalConfig) ) //!< Get the SID filter elements count configured for the pComp
#define MCANV71_GetEIDfilterElementCount(pComp)  ( MCAN_FILTER_EID_SIZE_GET((pComp)->InternalConfig) ) //!< Get the EID filter elements count configured for the pComp

#define MCANV71_GetSIDfilterTotalByteSize(pComp)  ( MCANV71_GetSIDfilterElementCount(pComp) * MCAN_CAN_STANDARD_FILTER_SIZE ) //!< Get the SID filter total byte size
#define MCANV71_GetEIDfilterTotalByteSize(pComp)  ( MCANV71_GetEIDfilterElementCount(pComp) * MCAN_CAN_EXTENDED_FILTER_SIZE ) //!< Get the EID filter total byte size

//-----------------------------------------------------------------------------

typedef struct MCANV71 MCANV71;          //! Typedef of MCAN peripheral object structure
typedef uint32_t TMCANV71DriverInternal; //! Alias for Driver Internal data flags

/*! @brief Function that gives the current millisecond of the system to the driver
 *
 * This function will be called when the driver needs to get current millisecond
 * @return Returns the current millisecond of the system
 */
typedef uint32_t (*GetCurrentms_Func)(void);

//! MCAN for ATSAMV71 peripheral object structure
struct MCANV71
{
  void *UserDriverData;                  //!< Optional, can be used to store driver data or NULL

  //--- Driver configuration ---
  setMCAN_DriverConfig DriverConfig;     //!< Driver configuration, by default it is MCAN_DRIVER_NORMAL_USE. Configuration can be OR'ed
  TMCANV71DriverInternal InternalConfig; //!< DO NOT USE OR CHANGE THIS VALUE, IT'S THE INTERNAL DRIVER CONFIGURATION

#ifdef MCAN_INTERNAL_CAN_CONTROLLER
  //--- MCAN peripheral ---
  volatile void* Instance;               //!< This is the pointer to the peripheral controller address
  void* RAMallocation;                   //!< Message and Filters RAM allocation pointer
  size_t RAMsize;                        //!< RAM allocation size
#else // MCAN_EXTERNAL_CAN_CONTROLLER
  //--- Interface driver call functions ---
  uint8_t SPI_ChipSelect;                //!< This is the Chip Select index that will be set at the call of a transfer
  void* InterfaceDevice;                 //!< This is the pointer that will be in the first parameter of all interface call functions
  uint32_t SPIClockSpeed;                //!< SPI nominal clock speed
# ifdef USE_DYNAMIC_INTERFACE
  SPI_Interface* SPI;                    //!< This is the SPI_Interface descriptor pointer that will be used to communicate with the device
# else
  SPI_Interface SPI;                     //!< This is the SPI_Interface descriptor that will be used to communicate with the device
# endif
# ifdef MCAN_USE_CACHE
  uint32_t __RegCache[MCAN_CACHE_COUNT]; //!< DO NOT USE OR CHANGE THESE VALUES. This is a cache for some register's configuration to avoid getting FIFOs/Buffers configuration each time the driver transmit/receive frames
# endif
#endif

  //--- GPIO configuration ---
#if !defined(MCAN_INTERNAL_CAN_CONTROLLER)
  uint8_t GPIOsOutState;                 //!< GPIOs pins output state (0 = set to '0' ; 1 = set to '1'). Used to speed up output change
#endif

  //--- Time call function ---
  GetCurrentms_Func fnGetCurrentms;      //!< This function will be called when the driver need to get current millisecond
};

//-----------------------------------------------------------------------------

//! MCANV71 Peripheral and CAN configuration structure
typedef struct MCANV71_Config
{
  //--- Controller clocks ---
  uint32_t MainFreq;                             //!< Main MCAN frequency. Set to 0 to use the Peripheral frequency with the #MCANV71_ConfigurePeripheralClocks() function call
  uint8_t MessageRAMwatchdogConf;                //!< Start value of the Message RAM Watchdog Counter. The counter is disabled when WDC is cleared
  uint32_t *SYSCLK_Result;                        //!< This is the SYSCLK of the component after configuration (can be NULL if the internal SYSCLK of the component do not have to be known)

  //--- RAM configuration ---
  uint8_t SIDelementsCount;                      //!< Standard ID Number of Filter Elements in RAM (0 to 128 elements)
  uint8_t EIDelementsCount;                      //!< Extended ID Number of Filter Elements in RAM (0 to 64 elements)

  //--- CAN configuration ---
  uint32_t ExtendedIDrangeMask;                  //!< MCAN Extended ID AND (range) Mask
  bool RejectAllStandardIDs;                     //!< Set to 'true' to reject remote frames with 11-bit standard IDs, else set to 'false' to filter remote frames with 11-bit standard IDs
  bool RejectAllExtendedIDs;                     //!< Set to 'true' to reject remote frames with 29-bit extended IDs, else set to 'false' to filter remote frames with 29-bit extended IDs
  eMCAN_AcceptNonMatching NonMatchingStandardID; //!< Accept Non-matching Frames Standard (11-bit standard IDs)
  eMCAN_AcceptNonMatching NonMatchingExtendedID; //!< Accept Non-matching Frames Extended (29-bit extended IDs)
#if defined(MCANV71_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_CANFDbusConfig BusConfig;                  //!< CAN Bus configuration
  CAN_BitTimeStats* BitTimeStats;                //!< Point to a Bit Time stat structure (set to NULL if no statistics are necessary)
#else
  CAN_BitTimeConfig BitTimeConfig;               //!< BitTime configuration
#endif
  setMCAN_CANCtrlFlags ControlFlags;             //!< Set of CAN control flags to configure the CAN controller. Configuration can be OR'ed

  //--- GPIOs and Interrupts pins ---
  bool EnableLineINT0;                           //!< Enable line MCAN_INT0 interrupt if set to 'true' else disable the line MCAN_INT0
  bool EnableLineINT1;                           //!< Enable line MCAN_INT1 interrupt if set to 'true' else disable the line MCAN_INT1

  //--- Interrupts ---
  setMCAN_InterruptEvents SysInterruptFlags;     //!< Set of system interrupt flags to enable. Configuration can be OR'ed
  setMCAN_IntLineSelect   SysIntLineSelect;      //!< Set of system interrupt line select. Configuration can be OR'ed
  uint32_t BufferTransmitInt;                    //!< Set of Tx buffer transmit interrupts. Configuration can be OR'ed
  uint32_t BufferCancelFinishInt;                //!< Set of Tx buffer cancel finish interrupts. Configuration can be OR'ed
} MCANV71_Config;

//-----------------------------------------------------------------------------



//********************************************************************************************************************


#ifdef MCAN_INTERNAL_CAN_CONTROLLER
/*! Configure the MCAN peripheral clock
 * @param[in] *pComp Is the pointed structure of the peripheral to be use
 * @param[out] *peripheralClock Is the peripheral clock
 * @return Returns an #eERRORRESULT value enum
 */
MCAN_EXTERN eERRORRESULT MCANV71_ConfigurePeripheralClocks(MCANV71 *pComp, uint32_t* const peripheralClock) MCAN_WEAK;

/*! Configure the MCAN DMA base address. Usually the 16-bit MSB of the RAM address
 * @param[in] *pComp Is the pointed structure of the peripheral to be use
 * @return Returns an #eERRORRESULT value enum
 */
MCAN_EXTERN eERRORRESULT MCANV71_ConfigureMCANbaseAddress(MCANV71 *pComp) MCAN_WEAK;
#endif

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
eERRORRESULT Init_MCANV71(MCANV71 *pComp, const MCANV71_Config* const pConf, const MCAN_FIFObuff* const listFIFO, size_t listFIFOcount);

//********************************************************************************************************************


/*! @brief Get actual peripheral of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral
 * @param[out] *mcanCoreId Is the Core Release. One digit, BCD-coded
 * @param[out] *mcanStep Is the Step of Core Release. One digit, BCD-coded
 * @param[out] *mcanSubStep Is the Sub-step of Core Release. One digit, BCD-coded
 * @param[out] *mcanYear Is the Timestamp Year. One digit, BCD-coded
 * @param[out] *mcanMonth Is the Timestamp Month. Two digits, BCD-coded
 * @param[out] *mcanDay Is the Timestamp Day. Two digits, BCD-coded
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetMCANcoreID(MCANV71 *pComp, uint8_t* const mcanCoreId, uint8_t* const mcanStep, uint8_t* const mcanSubStep, uint8_t* const mcanYear, uint8_t* const mcanMonth, uint8_t* const mcanDay);

//********************************************************************************************************************


/*! @brief Read from a 32-bits register of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the register's address where data will be read (need 32-bits alignment)
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReadREG32(MCANV71 *pComp, const uint32_t address, uint32_t* data);

/*! @brief Write to a 32-bits register of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the register's address where data will be written (need 32-bits alignment)
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_WriteREG32(MCANV71 *pComp, const uint32_t address, const uint32_t data);

//-----------------------------------------------------------------------------


/*! @brief Read from allocated RAM data of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the address where data will be read in the RAM allocation (need 32-bits alignment)
 * @param[out] *data Is where the data will be stored
 * @param[in] count Is the count of 32-bits data to read
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReadRAM(MCANV71 *pComp, const uint32_t address, uint8_t* data, const uint16_t count);


/*! @brief Write to allocated RAM data of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] address Is the address where data will be written in the RAM allocation (need 32-bits alignment)
 * @param[in] data Is the data to write
 * @param[in] count Is the count of 32-bits data to write
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_WriteRAM(MCANV71 *pComp, const uint32_t address, const uint8_t* data, const uint16_t count);

//********************************************************************************************************************


/*! @brief Read the customer register of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *data Is where the data will be stored
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_ReadCustomerRegister(MCANV71 *pComp, uint32_t* const data)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_CUST, data); // Read value of the CUST register of an external controller
}


/*! @brief Write the customer register of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] data Is the data to write
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_WriteCustomerRegister(MCANV71 *pComp, const uint32_t data)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_CUST, data); // Read value of the CUST register of an external controller
}

//********************************************************************************************************************


/*! @brief Transmit a message object (with data) to the FIFO/Buffer/TXQ of the MCAN peripheral
 *
 * Transmit the message to the specified FIFO/Buffer/TXQ. This function uses the specific format of the component (T0, T1, Ti).
 * This function gets the next address where to put the message object on, send it and update the head pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageObjectToSend Is the message object to send with all its data
 * @param[in] objectSize Is the size of the message object (with its data). This value needs to be modulo 4
 * @param[in] toFIFObuff Is the name of the FIFO/Buffer/TXQ to fill
 * @param[in] index Is the index where to put the message in case of Buffer. If not a buffer, this value will not be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_TransmitMessageObject(MCANV71 *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize, eMCAN_FIFObuffer toFIFObuff, uint8_t index);

/*! @brief Transmit a message object (with data) to the TXQ of the MCAN
 *
 * Transmit the message to the TXQ. This function uses the specific format of the component (T0, T1, Ti).
 * This function gets the next address where to put the message object on, send it and update the head pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageObjectToSend Is the message object to send with all its data
 * @param[in] objectSize Is the size of the message object (with its data). This value need to be modulo 4
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_TransmitMessageObjectToTXQ(MCANV71 *pComp, const uint8_t* const messageObjectToSend, uint8_t objectSize)
{
  return MCANV71_TransmitMessageObject(pComp, messageObjectToSend, objectSize, MCAN_TXQ_FIFO, 0);
}

/*! @brief Transmit a message to a FIFO/Buffer/TXQ of the MCAN peripheral
 *
 * Transmit the message to the specified FIFO/Buffer/TXQ
 * This function gets the next address where to put the message object on, send it and update the head pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageToSend Is the message to send with all the data attached with
 * @param[in] toFIFObuff Is the name of the FIFO/Buffer/TXQ to fill
 * @param[in] index Is the index where to put the message in case of Buffer. If not a buffer, this value will not be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_TransmitMessage(MCANV71 *pComp, const CAN_CANMessage* const messageToSend, eMCAN_FIFObuffer toFIFObuff, uint8_t index);

/*! @brief Transmit a message to the TXQ of the MCAN
 *
 * Transmit the message to the TXQ
 * This function gets the next address where to put the message object on, send it and update the head pointer
 * @warning This function does not check if the TXQ have a room for the message or the actual state of the TXQ
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[in] *messageToSend Is the message to send with all the data attached with
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_TransmitMessageToTXQ(MCANV71 *pComp, const CAN_CANMessage* const messageToSend)
{
  return MCANV71_TransmitMessage(pComp, messageToSend, MCAN_TXQ_FIFO, 0);
}


/*! @brief Receive a message object (with data) to the FIFO/Buffer of the MCAN
 *
 * Receive the message from the specified FIFO/Buffer. This function uses the specific format of the component (R0, R1/R1A/R1B, (R2,) Ri).
 * This function gets the next address where to get the message object from, get it and update the tail pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageObjectGet Is the message object retrieve with all its data
 * @param[in] objectSize Is the size of the message object (with its data). This value needs to be modulo 4
 * @param[in] fromFIFObuff Is the name of the FIFO/Buffer to extract
 * @param[in] index Is the index where to get the message in case of Buffer. If not a buffer, this value will not be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReceiveMessageObject(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize, eMCAN_FIFObuffer fromFIFObuff, uint8_t index);

/*! @brief Receive a message object (with data) from TEF of the MCAN peripheral
 *
 * Receive a message from the TEF. This function uses the specific format of the component (TE0, TE1/TE1A/TE1B (,TE2)).
 * This function gets the next address where to get the message object from, get it and update the tail pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageObjectGet Is the message object retrieve with all its data
 * @param[in] objectSize Is the size of the message object (with its data). This value needs to be modulo 4
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReceiveMessageObjectFromTEF(MCANV71 *pComp, uint8_t* const messageObjectGet, uint8_t objectSize);

/*! @brief Receive a message from a FIFO/Buffer/TEF of the MCAN
 *
 * Receive a message from the specified FIFO/Buffer/TEF
 * This function gets the next address where to get the message object from, get it and update the tail pointer
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageGet Is the message retrieve with all the data attached with
 * @param[in] payloadSize Indicate the payload of the FIFO (8, 12, 16, 20, 24, 32, 48 or 64)
 * @param[out] *timeStamp Is the returned TimeStamp of the message. Can be the Number of TSU Time Stamp register (TS0..15) where the related timestamp is stored (can be set to NULL if the TimeStamp is not wanted)
 * @param[out] *filterHitIdx Is the returned filter hit index of the message (can be set to NULL if the TimeStamp is not wanted)
 * @param[in] fromFIFObuff Is the name of the FIFO to extract
 * @param[in] index Is the index where to get the message in case of Buffer. If not a buffer, this value will not be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ReceiveMessage(MCANV71 *pComp, CAN_CANMessage* const messageGet, eMCAN_PayloadSize payloadSize, uint32_t* const timeStamp, uint8_t* const filterHitIdx, eMCAN_FIFObuffer fromFIFObuff, uint8_t index);

/*! @brief Receive a message from the TEF of the MCAN
 *
 * Receive a message from the TEF
 * This function gets the next address where to get the message object from, get it and update the tail pointer
 * @warning This function does not check if the TEF have a message pending or the actual state of the TEF or if the TimeStamp is set or not
 * @param[in] *pComp Is the pointed structure of the device to be used
 * @param[out] *messageGet Is the message retrieve with all the data attached with
 * @param[out] *timeStamp Is the returned TimeStamp of the message. Can be the Number of TSU Time Stamp register (TS0..15) where the related timestamp is stored (can be set to NULL if the TimeStamp is not wanted)
 * @param[out] *filterHitIdx Is the returned filter hit index of the message (can be set to NULL if the TimeStamp is not wanted)
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_ReceiveMessageFromTEF(MCANV71 *pComp, CAN_CANMessage* const messageGet, uint32_t* const timeStamp, uint8_t* const filterHitIdx)
{
  return MCANV71_ReceiveMessage(pComp, messageGet, MCAN_8_BYTES, timeStamp, filterHitIdx, MCAN_TEF, 0); // Here the payload will not be used inside the function
}

//********************************************************************************************************************


/*! @brief Configure message RAM watchdog of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] messageRAMwatchdogConf Is the start value of the Message RAM Watchdog Counter. The counter is disabled when WDC is cleared
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_ConfigureWatchdogRAM(MCANV71 *pComp, uint8_t messageRAMwatchdogConf)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_RWD, MCAN_RWD_WATCHDOG_CONFIG_SET(messageRAMwatchdogConf)); // Write value of the RWD register of an external controller
}

/*! @brief Get message RAM watchdog's value of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *messageRAMwatchdogValue Is the Watchdog Counter Value for the current message located in RAM
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetWatchdogRAMvalue(MCANV71 *pComp, uint8_t* const messageRAMwatchdogValue)
{
#ifdef CHECK_NULL_PARAM
   if ((pComp == NULL) || (messageRAMwatchdogValue == NULL)) return ERR__PARAMETER_ERROR;
#endif
  MCAN_RWD_Register Reg;
  eERRORRESULT Error = MCANV71_ReadREG32(pComp, RegMCAN_RWD, &Reg.RWD); // Read RWD register of an external controller
  if (Error != ERR_NONE) return Error;
  *messageRAMwatchdogValue = (uint8_t)MCAN_RWD_WATCHDOG_VALUE_GET(Reg.RWD);
  return ERR_NONE;
}

//********************************************************************************************************************


/*! @brief Configure INT Lines of MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] enableLineINT0 Set to 'true' to enable the MCAN_INT0 line, else set to 'false' to disable the line
 * @param[in] enableLineINT1 Set to 'true' to enable the MCAN_INT1 line, else set to 'false' to disable the line
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureINTlines(MCANV71 *pComp, bool enableLineINT0, bool enableLineINT1);

//********************************************************************************************************************


/*! @brief Calculate Bit Time for CAN2.0 or CAN-FD Configuration for the MCAN
 *
 * Calculate the best Bit Time configuration following desired bitrates for CAN-FD
 * This function call automatically the MCANV71_CalculateBitrateStatistics() function
 * @param[in] periphClk Is the clock of the peripheral
 * @param[in] *busConf Is the bus configuration of the CAN-bus
 * @param[out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CANFDbusConfig* const busConf, struct CAN_BitTimeConfig* const pConf);

/*! @brief Calculate Bitrate Statistics of a Bit Time configuration
 *
 * Calculate bus length, sample points, bitrates and oscillator tolerance following BitTime Configuration
 * @param[in] fsysclk Is the SYSCLK of the MCAN
 * @param[in,out] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_CalculateBitrateStatistics(const uint32_t periphClk, CAN_BitTimeConfig *pConf);

/*! @brief Set Bit Time Configuration to the MCAN
 *
 * Set the Nominal and Data Bit Time to registers
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] *pConf Is the pointed structure of the Bit Time configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_SetBitTimeConfiguration(MCANV71 *pComp, const CAN_BitTimeConfig* const pConf);

//********************************************************************************************************************


/*! @brief Configure write protection of control register of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] enable 'true' set the write protection, 'false' disable the write protection
 * @param[out] *lastRegRead If not NULL, it will be the last CCCR register's value read (Here to reduce communications)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureWriteProtection(MCANV71 *pComp, bool enable, uint32_t* const lastRegRead);

/*! @brief Set write protection of control register of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetWriteProtection(MCANV71 *pComp)
{
  return MCANV71_ConfigureWriteProtection(pComp, true, NULL);
}

/*! @brief Remove write protection of control register of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_RemoveWriteProtection(MCANV71 *pComp)
{
  return MCANV71_ConfigureWriteProtection(pComp, false, NULL);
}



/*! @brief Get actual operation mode of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[out] *actualMode Is where the result of the actual mode will be saved
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetActualOperationMode(MCANV71 *pComp, eMCAN_OperationMode* const actualMode);

/*! @brief Request operation mode change of the MCAN peripheral
 *
 * @note Configuration write protection will be set but not for MCAN_INITIALISATION_MODE mode
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] newMode Is the new operational mode to set
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_RequestOperationMode(MCANV71 *pComp, eMCAN_OperationMode newMode);

/*! @brief Start the MCAN peripheral in CAN2.0 mode
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_StartCAN20(MCANV71 *pComp)
{
  return MCANV71_RequestOperationMode(pComp, MCAN_NORMAL_CAN20_MODE);
}

/*! @brief Start the MCAN peripheral in CAN-FD mode
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_StartCANFD(MCANV71 *pComp)
{
  return MCANV71_RequestOperationMode(pComp, MCAN_NORMAL_CANFD_MODE);
}

/*! @brief Start the MCAN peripheral in CAN Listen-Only mode
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_StartCANListenOnly(MCANV71 *pComp)
{
  return MCANV71_RequestOperationMode(pComp, MCAN_LISTEN_ONLY_MODE);
}

//********************************************************************************************************************



/*! @brief Configure the test register in the MCAN peripheral
 *
 * @note Write access to the Test Register has to be enabled by setting bit MCAN_CCCR.TEST to '1'.
 * All MCAN Test Register functions are set to their reset values when bit MCAN_CCCR.TEST is cleared.
 * Loop Back mode and software control of pin CANTX are hardware test modes. Programming of TX != 0 disturbs the message transfer on the CAN bus
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] enableLoopback Set to 'true' to enable loopback mode, else 'false'
 * @param[in] txPinControl Is the control of transmit pin mode
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureTest(MCANV71 *pComp, bool enableLoopback, eMCAN_TestTxPin txPinControl);

/*! @brief Read the test register in the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[out] *rxPin Indicates the state of the Rx pin
 * @param[out] *txNumPrepared Is the number of Tx Buffer Prepared
 * @param[out] *preparedValid Set to 'true' if txNumPrepared is valid, else 'false'
 * @param[out] *txNumStarted Is the number of Tx Buffer Started
 * @param[out] *startedValid Set to 'true' if txNumStarted is valid, else 'false'
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetTest(MCANV71 *pComp, eMCAN_TestRxPin* const rxPin, uint8_t* const txNumPrepared, bool* const preparedValid, uint8_t* const txNumStarted, bool* const startedValid);

/*! @brief Configure CAN Control of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[in] flags Is all the flags for the configuration
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureCANController(MCANV71 *pComp, setMCAN_CANCtrlFlags flags);

//********************************************************************************************************************


/*! @brief Enter the MCAN peripheral in sleep mode
 *
 * This function puts the peripheral in sleep mode
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_EnterSleepMode(MCANV71 *pComp);

/*! @brief Verify if the MCAN peripheral is in sleep mode
 *
 * This function verifies if the peripheral is in sleep mode by checking the CCCR.CSA
 * @param[in] *pComp Is the pointed structure of the peripheral to be used
 * @param[out] *isInSleepMode Indicate if the peripheral is in sleep mode
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_IsDeviceInSleepMode(MCANV71 *pComp, bool* const isInSleepMode);

//********************************************************************************************************************


/*! @brief Configure the Time Stamp of frames in the MCAN peripheral
 *
 * This function configures the 32-bit free-running counter of the Time Stamp
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] timestampSource Is an enumerator that indicates the source of the timestamp counter
 * @param[in] prescaler Is the prescaler of the Time Stamp counter (1..16)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureTimeStamp(MCANV71 *pComp, eMCAN_TimeStampSelect timestampSource, uint8_t prescaler);

/*! @brief Set the Time Stamp counter the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] value Is the value to set into the counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetTimeStamp(MCANV71 *pComp, uint32_t value)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_TSCV, MCAN_TSCV_TIMESTAMP_SET(value)); // Write configuration to the TSCV register of an external controller
}

/*! @brief Get the Time Stamp counter the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *value Is the value to get from the counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetTimeStamp(MCANV71 *pComp, uint32_t* value)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_TSCV, value); // Read the value to the TSCV register of an external controller
}

//********************************************************************************************************************


/*! @brief Configure the Rx timeout counter in the MCAN peripheral
 *
 * This function configures the 32-bit free-running counter of the Time Stamp
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] enableTC Is at 'true' to enable the timeout counter or 'false' to disable the timeout counter
 * @param[in] timeoutSelect Is an enumerator that indicates the source of the timeout reset
 * @param[in] period Is the period of the timeout counter
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureTimeoutCounter(MCANV71 *pComp, bool enableTC, eMCAN_TimeoutSelect timeoutSelect, uint16_t period);

/*! @brief Set the Rx timeout counter the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] value Is the value to set into the counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetTimeoutCounter(MCANV71 *pComp, uint32_t value)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_TOCV, MCAN_TOCV_TIMEOUT_COUNTER_SET(value)); // Write configuration to the TOCV register of an external controller
}

/*! @brief Get the Rx timeout counter the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *value Is the value to get from the counter
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetTimeoutCounter(MCANV71 *pComp, uint32_t* value)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_TOCV, value); // Read the value to the TOCV register of an external controller
}

//********************************************************************************************************************

/*! @brief Get status of a FIFO/Buffer of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] name Is the name of the FIFO/TEF for which to retrieve the status
 * @param[out] *statusFlags Is where the status flag of the FIFO/Buffer will be stored
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetFIFOStatus(MCANV71 *pComp, eMCAN_FIFObuffer name, setMCAN_FIFObufferstatus* const statusFlags);

/*! @brief Get next message index of a FIFO of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] name Is the name of the FIFO/TEF for which to retrieve the information
 * @param[out] *level Is the number of elements stored in Receive FIFO
 * @param[out] *getIndex Is the receive FIFO/TEF read index pointer
 * @param[out] *putIndex Is the receive FIFO/TEF write index pointer
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetNextMessageAddressFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex);

/*! @brief Get next message index of a TEF of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *level Is the number of elements stored in Receive FIFO
 * @param[out] *getIndex Is the receive TEF read index pointer
 * @param[out] *putIndex Is the receive TEF write index pointer
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetNextMessageAddressTEF(MCANV71 *pComp, uint32_t* const level, uint8_t* const getIndex, uint8_t* const putIndex)
{
  return MCANV71_GetNextMessageAddressFIFO(pComp, MCAN_TEF, level, getIndex, putIndex);
}

/*! @brief Acknowledge FIFO/TEF of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] name Is the name of the FIFO/TEF for which to acknowledge
 * @param[in] acknowledgeIndex Is the index to acknowledge reception
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_AcknowledgeFIFO(MCANV71 *pComp, eMCAN_FIFObuffer name, uint32_t acknowledgeIndex);

/*! @brief Acknowledge TEF of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] acknowledgeIndex Is the index to acknowledge reception
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_AcknowledgeTEF(MCANV71 *pComp, uint32_t acknowledgeIndex)
{
  return MCANV71_AcknowledgeFIFO(pComp, MCAN_TEF, acknowledgeIndex);
}

/*! @brief Set Tx Buffer add request of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] bufferIndex Is the buffer index to set
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetTxBufferAddRequest(MCANV71 *pComp, uint32_t bufferIndex)
{
  if (bufferIndex > (MCAN_TX_BUFFER_SIZE_MAX - 1)) return ERR__PARAMETER_ERROR;
  bufferIndex = (1 << bufferIndex);
  return MCANV71_WriteREG32(pComp, RegMCAN_TXBAR, bufferIndex); // Write configuration to the TXBAR register of an external controller
}

/*! @brief Get all Tx Buffer request pending of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *requestPending Is the set of buffers with a request pending (bit set to '1' when corresponding buffer have a pending request). Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetAllTxBufferRequestPending(MCANV71 *pComp, uint32_t* const requestPending)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_TXBRP, requestPending); // Read register of an external device
}

/*! @brief Set multiple Tx Buffer cancellation request of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] multipleRequest Is the set of buffers to cancel (bit set to '1' when corresponding buffer have a cancellation request). Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetMultipleTxBufferCancellationRequest(MCANV71 *pComp, uint32_t multipleRequest)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_TXBCR, multipleRequest); // Write configuration to the TXBCR register
}

/*! @brief Set Tx Buffer cancellation request of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] bufferIndex Is the buffer index to cancel
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetTxBufferCancellationRequest(MCANV71 *pComp, uint32_t bufferIndex)
{
  if (bufferIndex > (MCAN_TX_BUFFER_SIZE_MAX - 1)) return ERR__PARAMETER_ERROR;
  bufferIndex = (1 << bufferIndex);
  return MCANV71_SetMultipleTxBufferCancellationRequest(pComp, bufferIndex);
}

/*! @brief Get all Tx Buffer transmission occured of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] transmissionOccured Is the set of buffers where the transmission occured (bit set to '1' when corresponding buffer have a transmission occured). Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetAllTxBufferTransmitOccured(MCANV71 *pComp, uint32_t* const transmissionOccured)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_TXBTO, transmissionOccured); // Read register of an external device
}

/*! @brief Get all Tx Buffer cancellation finished of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] cancellationFinished Is the set of buffers where the cancellation finished (bit set to '1' when corresponding buffer have a cancellation finished). Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetAllTxBufferCancellationFinished(MCANV71 *pComp, uint32_t* const cancellationFinished)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_TXBCF, cancellationFinished); // Read register of an external device
}

//********************************************************************************************************************


/*! @brief Configure Global Filter Configuration of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] rejectAllStandardIDs Set to 'true' to reject remote frames with 11-bit standard IDs, else set to 'false' to filter remote frames with 11-bit standard IDs
 * @param[in] rejectAllExtendedIDs Set to 'true' to reject remote frames with 29-bit extended IDs, else set to 'false' to filter remote frames with 29-bit extended IDs
 * @param[in] nonMatchingStandardID Indicate what to do with non matching standard ID frames (11-bit standard ID)
 * @param[in] nonMatchingExtendedID Indicate what to do with non matching extended ID frames (29-bit extended ID)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureGlobalFilters(MCANV71 *pComp, bool rejectAllStandardIDs, bool rejectAllExtendedIDs, eMCAN_AcceptNonMatching nonMatchingStandardID, eMCAN_AcceptNonMatching nonMatchingExtendedID);

/*! @brief Configure Extended ID AND Mask of the MCAN peripheral
 * @warning This register can only be written if the bits CCE and INIT are set in MCAN CC Control Register
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] andMask Is the Extended ID AND mask to configure
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_SetEIDrangeFilterMask(MCANV71 *pComp, uint32_t andMask)
{
  if ((andMask & ~MCAN_CAN_FILTER_EID_AND_SID_MASK) > 0) return ERR__FILTER_TOO_LARGE;
  return MCANV71_WriteREG32(pComp, RegMCAN_XIDAM, andMask); // Write configuration to the XIDAM register of an external controller
}

/*! @brief Configure a SID filter of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] *confFilter Is the filter configuration to apply
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureSIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter);

/*! @brief Configure a EID filter of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] *confFilter Is the filter configuration to apply
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureEIDfilter(MCANV71 *pComp, const MCAN_Filter* const confFilter);

/*! @brief Configure a filter list of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] *listFilter Is the filters list configuration to apply
 * @param[in] count Is the count of filters in the listFilter array
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureFilterList(MCANV71 *pComp, MCAN_Filter* const listFilter, size_t count);

/*! @brief Disable a Filter of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] name Is the filter index to disable
 * @param[in] extendedID 'true' if it is an extended filter else 'false' for a standard filter
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_DisableFilter(MCANV71 *pComp, uint16_t name, bool extendedID);

//********************************************************************************************************************


/*! @brief Configure interrupt of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be enabled. Flags can be OR'ed
 * @param[in] intLineSelect Is the set of assignment of each interrupt to INT0 or INT1. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureInterrupt(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags, setMCAN_IntLineSelect intLineSelect);

/*! @brief Get interrupt events of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *interruptsFlags Is the return value of interrupt events. Flags are OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents* interruptsFlags)
{
  return MCANV71_ReadREG32(pComp, RegMCAN_IR, (uint32_t*)interruptsFlags); // Read all interrupt flags status
}

/*! @brief Clear interrupt events of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] interruptsFlags Is the set of events where interrupts will be cleared. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_ClearInterruptEvents(MCANV71 *pComp, setMCAN_InterruptEvents interruptsFlags)
{
  return MCANV71_WriteREG32(pComp, RegMCAN_IR, (uint32_t)interruptsFlags); // Write configuration to the IR register
}

/*! @brief Get high priority message status of the MCAN peripheral
 * @detail This register is updated every time a Message ID filter element configured to generate a priority event matches.
 * This can be used to monitor the status of incoming high priority messages and to enable fast access to these messages
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *messageIndicator Is the message storage indicator of the high priority event
 * @param[out] *isExtended Is 'true' if the filter list of the matching filter element is extended, else 'false' for standard filter list
 * @param[out] *bufferIndex Is the index of Receive FIFO element to which the message was stored. Only valid when 'messageIndicator' is #MCAN_STORED_IN_FIFO_0 or #MCAN_STORED_IN_FIFO_1
 * @param[out] *filterIndex Is the index of matching filter element. Range is 0 to MCAN_SIDFC.LSS - 1 resp. MCAN_XIDFC.LSE - 1.
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetHighPriorityMessageStatus(MCANV71 *pComp, eMCAN_MessageStorageIndicator* const messageIndicator, bool* const isExtended, uint8_t* const bufferIndex, uint8_t* const filterIndex);

/*! @brief Get Rx buffer New Data events flags of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[out] *newDataIdx0_31 Is the New Data flags of Receive Buffers 0 to 31. Flags can be OR'ed
 * @param[out] *newDataIdx32_63 Is the New Data flags of Receive Buffers 32 to 63. Flags can be OR'ed
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetRxBufferNewDataFlag(MCANV71 *pComp, uint32_t* const newDataIdx0_31, uint32_t* const newDataIdx32_63);

/*! @brief Clear a Rx buffer New Data events flags of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] index Is the flag index to clear
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ClearRxBufferNewDataFlag(MCANV71 *pComp, uint8_t index);

/*! @brief Configure Tx buffer interrupts of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN to be used
 * @param[in] transmitEnable Is the flag index set to set transmit enable
 * @param[in] cancelFinishEnable Is the flag index set to set cancellation finish enable
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_ConfigureTxBufferInterrupts(MCANV71 *pComp, uint32_t transmitEnable, uint32_t cancelFinishEnable);

//********************************************************************************************************************


/*! @brief Get transmit/receive error count and status of the MCAN peripheral
 *
 * @warning Calling this function will clear the CAN Error Logging register even if all parameters are NULL
 * @param[in] *pComp Is the pointed structure of the MCAN use
 * @param[out] *transmitErrorCount Is the result of the transmit error count (this parameter can be NULL)
 * @param[out] *receiveErrorCount Is the result of the receive error count (this parameter can be NULL)
 * @param[out] *receiveErrorPassive Is set to 'true' if the Receive Error Counter has reached the error passive level of 128 else 'false' if below the error passive level of 128 (this parameter can be NULL)
 * @param[out] *canErrorLogging Is the CAN Error Logging (this parameter can be NULL)
 * @return Returns an #eERRORRESULT value enum
 */
eERRORRESULT MCANV71_GetTransmitReceiveErrorCountAndStatus(MCANV71 *pComp, uint8_t* transmitErrorCount, uint8_t* receiveErrorCount, bool* receiveErrorPassive, uint8_t* canErrorLogging);


/*! @brief Get Bus diagnostic of the MCAN peripheral
 *
 * @param[in] *pComp Is the pointed structure of the MCAN use
 * @param[out] *busDiagnostic Is the return value that contains separate errors of the CAN protocol
 * @return Returns an #eERRORRESULT value enum
 */
inline eERRORRESULT MCANV71_GetBusDiagnostic(MCANV71 *pComp, MCAN_PSR_Register* const busDiagnostic)
{
#ifdef CHECK_NULL_PARAM
  if (busDiagnostic == NULL) return ERR__PARAMETER_ERROR;
#endif
  return MCANV71_ReadREG32(pComp, RegMCAN_PSR, (uint32_t*)busDiagnostic->PSR); // Read value of the PSR register
}

//********************************************************************************************************************


/*! @brief Payload to Byte Count
 *
 * @param[in] payload Is the enum of Message Payload Size (8, 12, 16, 20, 24, 32, 48 or 64 bytes)
 * @return Returns the byte count
 */
uint8_t MCANV71_PayloadToByte(eMCAN_PayloadSize payload);


/*! @brief Data Length Content to Byte Count
 *
 * @param[in] dlc Is the enum of Message DLC Size (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48 or 64 bytes)
 * @param[in] isCANFD Indicate if the DLC is from a CAN-FD frame or not
 * @return Returns the byte count
 */
uint8_t MCANV71_DLCToByte(eMCAN_DataLength dlc, bool isCANFD);

//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* MCAN_V71_H_INC */