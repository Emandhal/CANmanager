/*!*****************************************************************************
 * @file    MCP251X.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    06/08/2023
 * @brief   MCP2510/MCP2515 driver
 * @details Stand-Alone CAN Controller with SPI Interface
 * Follow datasheet MCP2510 Rev.F (Jan 2007)
 *                  MCP2515 Rev.K (Apr 2021)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "MCP251X.h"
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
#define MCP251X_TIME_DIFF(begin,end)  ( ((end) >= (begin)) ? ((end) - (begin)) : (UINT32_MAX - ((begin) - (end) - 1)) ) // Works only if time difference is strictly inferior to (UINT32_MAX/2) and call often
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// MCP251X initialization
//=============================================================================
eERRORRESULT Init_MCP251X(MCP251X *pComp, const MCP251X_Config* pConf)
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

  //--- Check configuration ---
  if (pConf->XtalFreq > MCP2515_FOSCFREQ_4V5_5V5_MAX) return ERR__FREQUENCY_ERROR; // The device crystal should not be > 24MHz
  if (pComp->SPIclockSpeed > MCP2515_SPICLOCK_MAX)  return ERR__FREQUENCY_ERROR; // The SPI frequency should not be > 10MHz
  pComp->InternalConfig = MCP251X_DEV_PS_SET(MCP251X_DEVICE_SLEEP_NOT_CONFIGURED);

  //--- Initialize SPI interface ---
  Error = pSPI->fnSPI_Init(pSPI, pComp->SPIchipSelect, STD_SPI_MODE0, pComp->SPIclockSpeed);
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling fnSPI_Init() then return the error

  //--- Set configuration mode ---
  Error = MCP251X_RequestOperationMode(pComp, MCP251X_CONFIGURATION_MODE, true);
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_RequestOperationMode() then return the Error

  //--- Configure controller pins ---
  Error = MCP251X_ConfigurePins(pComp, pConf->tx0PinMode, pConf->tx1PinMode, pConf->tx2PinMode, pConf->rx0PinMode, pConf->rx1PinMode, pConf->clkoutMode);
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_ConfigurePins() then return the Error

  //--- Set CAN configuration ---
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CANCTRL0, (pConf->UseOneShotMode ? MCP251X_CANCTRL_OSM : 0), MCP251X_CANCTRL_OSM_Mask);
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_ModifyRegister() then return the Error

  //--- Set nominal bitrate ---
  CAN_BitTimeConfig* ConfBitTime;
#if defined(MCP251X_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_BitTimeConfig BitTimeConfig;
  ConfBitTime = &BitTimeConfig;
  ConfBitTime->Stats = pConf->BitTimeStats;
  Error = MCP251X_CalculateBitTimeConfiguration(pConf->XtalFreq, pConf->BusConfig, ConfBitTime); // Calculate Bit Time
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_CalculateBitTimeConfiguration() then return the Error
#else
  ConfBitTime = &pConf->BitTimeConfig;
#endif
  Error = MCP251X_SetBitTimeConfiguration(pComp, ConfBitTime); // Set Bit Time configuration to registers
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_SetBitTimeConfiguration() then return the Error

  //--- Configure buffers ---
  Error = MCP251X_ConfigureBuffers(pComp, pConf->tx0PinMode, pConf->tx1PinMode, pConf->tx2PinMode, pConf->rx0PinMode, pConf->rx1PinMode);
  if (Error != ERR_NONE) return Error;                         // If there is an error while calling MCP251X_ConfigureBuffers() then return the Error

  //--- Configure interrupts ---
  return MCP251X_ConfigureInterrupt(pComp, pConf->Interrupts); // Configure interrupts
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Send a SPI instruction to the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_SendInstruction(MCP251X *pComp, uint8_t instruction)
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

  //--- Send the packet ---
  SPIInterface_Packet PacketDesc = SPI_INTERFACE_TX_DATA_DESC(&instruction, sizeof(instruction), true);
  return pSPI->fnSPI_Transfer(pSPI, &PacketDesc); // Transfer the packet
}


//=============================================================================
// Read data from register of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ReadRegister(MCP251X *pComp, eMCP251X_Registers reg, uint8_t* data, size_t size)
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
  uint8_t ChipAddr[2] = { MCP251X_SPI_INSTRUCTION_READ, (uint8_t)reg };

  //--- Send the address ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipAddr[0], sizeof(ChipAddr), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the register's address
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Get the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_RX_DATA_DESC(data, size, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by reading the data, get the data and stop transfer at last byte
}


//=============================================================================
// Write data to register of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_WriteRegister(MCP251X *pComp, eMCP251X_Registers reg, const uint8_t* data, size_t size)
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
  if (((uint8_t)reg & 0x80) > 0) return ERR__NOT_SUPPORTED;
  eERRORRESULT Error;
  uint8_t ChipAddr[2] = { MCP251X_SPI_INSTRUCTION_WRITE, (uint8_t)reg };

  //--- Send the address ---
  SPIInterface_Packet RegPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipAddr[0], sizeof(ChipAddr), false);
  Error = pSPI->fnSPI_Transfer(pSPI, &RegPacketDesc); // Transfer the register's address
  if (Error != ERR_NONE) return Error;                // If there is an error while calling fnSPI_Transfer() then return the Error
  //--- Send the data ---
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_TX_DATA_DESC((uint8_t*)data, size, true);
  return pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by transferring the data, and stop transfer at last byte
}


//=============================================================================
// Modify a register of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ModifyRegister(MCP251X *pComp, eMCP251X_Registers reg, uint8_t data, uint8_t mask)
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
  if (((uint8_t)reg & 0x80) > 0) return ERR__NOT_SUPPORTED;
  uint8_t Packet[4] = { MCP251X_SPI_INSTRUCTION_BIT_MODIFY, (uint8_t)reg, mask, data };

  //--- Send the packet ---
  SPIInterface_Packet PacketDesc = SPI_INTERFACE_TX_DATA_DESC(&Packet[0], sizeof(Packet), true);
  return pSPI->fnSPI_Transfer(pSPI, &PacketDesc); // Transfer the packet
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Transmit a message object (with data) to the Buffer of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_TransmitMessageObject(MCP251X *pComp, const uint8_t* messageObjectToSend, uint8_t objectSize, uint8_t buffIdx, bool andFlush)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (buffIdx >= MCP251X_TX_BUFFER_MAX) return ERR__OUT_OF_RANGE;
  if (objectSize < 5) return ERR__BAD_DATA_SIZE;
  eERRORRESULT Error;

  if (MCP251X_DEV_ID_GET(pComp->InternalConfig) == MCP2515) // Is a MCP2515?
  {
    //--- Send the instruction ---
    uint8_t ChipInst = MCP251X_SPI_LOAD_TX_BUFF_TXBnSIDH(buffIdx);
    SPIInterface_Packet InstPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipInst, sizeof(ChipInst), false);
    Error = pSPI->fnSPI_Transfer(pSPI, &InstPacketDesc); // Transfer the register's address
    if (Error != ERR_NONE) return Error;                 // If there is an error while calling fnSPI_Transfer() then return the Error
    //--- Send the data ---
    SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_TX_DATA_DESC((uint8_t*)messageObjectToSend, objectSize, true);
    Error = pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by transferring the data, and stop transfer at last byte
  }
  else // Is a MCP2510
  {
    Error = MCP251X_WriteRegister(pComp, RegMCP251X_TXBnSIDH(buffIdx), messageObjectToSend, objectSize);
  }
  if (Error != ERR_OK) return Error;                     // If there is an error then return the error

  //--- Flush if asked ---
  if (andFlush) Error = MCP251X_FlushBuffer(pComp, buffIdx);
  return Error;
}


//=============================================================================
// Transmit a message to the Buffer of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_TransmitMessage(MCP251X *pComp, CAN_CANMessage* const messageToSend, uint8_t buffIdx, bool andFlush)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((messageToSend->ControlFlags & CAN_CANFD_FRAME) > 0) return ERR__NOT_SUPPORTED; // CAN-FD frames not supported by this controller
  uint8_t Buffer[MCP251X_CAN_TX_MESSAGE_SIZE_MAX];
  MCP251X_CAN_TxMessage* Message = (MCP251X_CAN_TxMessage*)Buffer;                    // The first 5 bytes represent the MCP251X_CAN_TxMessage struct

  //--- Fill message ID ---
  MCP251X_MessageIDtoTxObjectMessageIdentifier(Message, messageToSend->MessageID, ((messageToSend->ControlFlags & CAN_EXTENDED_MESSAGE_ID) > 0));
  Message->Bytes[MCP251X_CAN_TXMSG_DLCR] = MCP251X_CAN_DLC_SET(messageToSend->DLC) | MCP251X_CAN_RTR_DATA_FRAME;
  if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) > 0) Message->Bytes[MCP251X_CAN_TXMSG_DLCR] |= MCP251X_CAN_RTR_REMOTE_FRAME; // Flag the RTR
  size_t BytesToSend = MCP251X_CAN_TX_MESSAGE_HEADER_SIZE;

  //--- Fill data ---
  if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) == 0) // If not RTR frame
  {
    if (((eMCP251X_DataLength)messageToSend->DLC != MCP251X_DLC_0BYTE) && (messageToSend->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
    size_t BytesToCopy = MCP251X_DLC_TO_VALUE[messageToSend->DLC];
    uint8_t* pBuff = &Message->Bytes[BytesToSend];                          // Next bytes of the Buffer is for payload
    uint8_t* pData = &messageToSend->PayloadData[0];                        // Select the first byte of payload data
    BytesToSend += BytesToCopy;
    while (BytesToCopy-- > 0) *pBuff++ = *pData++;                          // Copy data
  }

  //--- Send data ---
  return MCP251X_TransmitMessageObject(pComp, &Buffer[0], BytesToSend, buffIdx, andFlush);
}





//=============================================================================
// Receive a message object (with data) from the Buffer of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ReceiveMessageObject(MCP251X *pComp, uint8_t* messageObjectGet, uint8_t objectSize, uint8_t buffIdx)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  SPI_Interface* pSPI = GET_SPI_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pSPI == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pSPI->fnSPI_Transfer == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (buffIdx >= MCP251X_RX_BUFFER_MAX) return ERR__OUT_OF_RANGE;
  if (objectSize < 5) return ERR__BAD_DATA_SIZE;
  eERRORRESULT Error;

  if (MCP251X_DEV_ID_GET(pComp->InternalConfig) == MCP2515) // Is a MCP2515?
  {
    //--- Send the instruction ---
    uint8_t ChipInst = MCP251X_SPI_READ_RX_BUFF_RXBnSIDH(buffIdx);
    SPIInterface_Packet InstPacketDesc = SPI_INTERFACE_TX_DATA_DESC(&ChipInst, sizeof(ChipInst), false);
    Error = pSPI->fnSPI_Transfer(pSPI, &InstPacketDesc); // Transfer the register's address
    if (Error != ERR_NONE) return Error;                 // If there is an error while calling fnSPI_Transfer() then return the Error
    //--- Get the data ---
    SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_RX_DATA_DESC((uint8_t*)&messageObjectGet, objectSize, true);
    Error = pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by transferring the data, and stop transfer at last byte
  }
  else // Is a MCP2510
  {
    Error = MCP251X_ReadRegister(pComp, RegMCP251X_RXBnSIDH(buffIdx), messageObjectGet, objectSize);
    if (Error != ERR_NONE) return Error;                 // If there is an error while calling fnSPI_Transfer() then return the Error
    //--- Free the buffer ---
    const uint8_t BufferEvent = (buffIdx == 0 ? MCP251X_RX_BUFFER0_FULL_EVENT : MCP251X_RX_BUFFER1_FULL_EVENT);
    Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CANINTF, BufferEvent, BufferEvent);
  }
  return Error;
}


//=============================================================================
// Receive a message from the Buffer of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ReceiveMessage(MCP251X *pComp, CAN_CANMessage* const messageGet, eMCP251X_PayloadSize payloadSize, uint8_t buffIdx)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Buffer[MCP251X_CAN_RX_MESSAGE_SIZE_MAX];
  MCP251X_CAN_RxMessage* Message = (MCP251X_CAN_RxMessage*)Buffer; // The first 5 bytes represent the MCP251X_CAN_RxMessage struct
  eERRORRESULT Error;

  //--- Get data ---
  Error = MCP251X_ReceiveMessageObject(pComp, &Buffer[0], sizeof(Buffer), buffIdx);
  if (Error != ERR_NONE) return Error;                             // If there is an error while calling MCP251X_ReceiveMessageObject() then return the error

  //--- Extract message informations ---
  messageGet->ControlFlags = CAN_NO_MESSAGE_CTRL_FLAGS;
  messageGet->MessageID    = MCP251X_RxObjectMessageIdentifierToMessageID(Message);
  messageGet->DLC          = MCP251X_CAN_DLC_GET(Message->Bytes[MCP251X_CAN_RXMSG_DLCR]);
  const bool IsExtended = ((Message->Bytes[MCP251X_CAN_RXMSG_STDL] & MCP251X_CAN_EXIDE_EXTENDED_ID) > 0);
  const bool IsRemote = (((Message->Bytes[MCP251X_CAN_RXMSG_DLCR] & MCP251X_CAN_RTR_REMOTE_FRAME) > 0) && IsExtended) // This RTR bit for extended frames
                      || ((Message->Bytes[MCP251X_CAN_RXMSG_STDL] & MCP251X_CAN_SRR_REMOTE_FRAME) > 0);               // This SSR bit for standard frames
  if (IsExtended                                                             ) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_EXTENDED_MESSAGE_ID        ); // IDE
  if ((Message->Bytes[MCP251X_CAN_RXMSG_DLCR] & MCP251X_CAN_R0_RECESSIVE) > 0) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_R0_RECESSIVE               ); // R0
  if ((Message->Bytes[MCP251X_CAN_RXMSG_DLCR] & MCP251X_CAN_R1_RECESSIVE) > 0) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_R1_RECESSIVE               ); // R1
  if (IsRemote                                                               ) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_REMOTE_TRANSMISSION_REQUEST); // RTR

  //--- Fill payload ---
  if (IsRemote == false)                                            // If not RTR frame
  {
    if (((eMCP251X_DataLength)messageGet->DLC != MCP251X_DLC_0BYTE) && (messageGet->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
    if (messageGet->PayloadData != NULL)
    {
      const size_t BytesToGet = MCP251X_CAN_RX_MESSAGE_HEADER_SIZE;
      uint8_t* pBuff = &Message->Bytes[BytesToGet];                 // Next bytes of the Buffer is for payload
      uint8_t* pData = &messageGet->PayloadData[0];                 // Select the first byte of payload data
      uint8_t BytesPayload = MCP251X_PAYLOAD_TO_VALUE[payloadSize]; // Get the payload sizein bytes
      uint8_t BytesDLC = MCP251X_DLC_TO_VALUE[messageGet->DLC];     // Get how many byte need to be extract from the message to correspond to its DLC
      if (BytesPayload < BytesDLC) BytesDLC = BytesPayload;         // Get the least between BytesPayload and BytesDLC
      while (BytesDLC-- > 0) *pData++ = *pBuff++;                   // Copy data
    }
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure pins of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ConfigurePins(MCP251X *pComp, eMCP251X_TxPinMode tx0PinMode, eMCP251X_TxPinMode tx1PinMode, eMCP251X_TxPinMode tx2PinMode, eMCP251X_RxPinMode rx0PinMode, eMCP251X_RxPinMode rx1PinMode, eMCP251X_CLKOUTpinMode clkoutMode)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;

  //--- Configure clock divider register ---
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CANCTRL1, MCP251X_CANCTRL_CLKOUT_SET(clkoutMode), MCP251X_CANCTRL_CLKOUT_Mask);
  if (Error != ERR_NONE) return Error;                                     // If there is an error while calling MCP251X_ModifyRegister() then return the Error
  if (MCP251X_DEV_ID_GET(pComp->InternalConfig) == MCP2515)                // Is a MCP2515?
  {
    Config = (clkoutMode & MCP251X_CNF3_CLKOUT_IS_SOF);                    // Set Start-of-Frame Signal
    Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CNF3, Config, MCP251X_CNF3_CLKOUT_IS_SOF);
  }
  else if (clkoutMode == MCP251X_CLKOUT_IS_SOF) return ERR__NOT_SUPPORTED; // If MCP2510 and MCP251X_CLKOUT_IS_SOF, then this is not supported

  //--- Configure Tx and Rx pins register ---
  Config = MCP251X_TX0RTS_MODE_SET(tx0PinMode) | MCP251X_TX1RTS_MODE_SET(tx1PinMode) | MCP251X_TX2RTS_MODE_SET(tx2PinMode);
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_TXRTSCTRL, Config, MCP251X_TX0RTS_MODE_Mask | MCP251X_TX1RTS_MODE_Mask | MCP251X_TX2RTS_MODE_Mask);
  if (Error != ERR_NONE) return Error;                            // If there is an error while calling MCP251X_ModifyRegister() then return the Error
  Config = MCP251X_Rx0BF_MODE_SET(rx0PinMode) | MCP251X_Rx1BF_MODE_SET(rx1PinMode);
  return MCP251X_ModifyRegister(pComp, RegMCP251X_BFPCTRL, Config, MCP251X_Rx0BF_MODE_Mask | MCP251X_Rx1BF_MODE_Mask);
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#if defined(MCP251X_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
//=============================================================================
// Calculate Bit Time for CAN2.0 configuration for the MCP251X peripheral
//=============================================================================
eERRORRESULT MCP251X_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CAN20busConfig pBusConf, struct CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
   if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif
  //--- Check values ---
  if (periphClk > MCP251X_PERIPHERAL_CLK_MAX) return ERR__PARAMETER_ERROR;
  if (pBusConf.DesiredBitrate <= MCP251X_NOMBITRATE_MIN) return ERR__BAUDRATE_ERROR;
  if (pBusConf.DesiredBitrate >  MCP251X_NOMBITRATE_MAX) return ERR__BAUDRATE_ERROR;

  //--- Declaration ---
  CAN_BitTimeConfig LastBTconf; LastBTconf.Valid = false;
  const uint32_t SysClkDivided = periphClk / MCP251X_SYSCLOCK_DIV;
  bool SearchExactBR = true;

  //--- Step 1 ---
  const uint32_t tPropSeg = 2 * ((MCP251X_tBUS_CONV * pBusConf.BusMeters) + pBusConf.TransceiverDelay); // Formula is tPROP_SEG(ns) = 2x((5ns * BusMeters) + tTXDtRXD)

  do
  {
    uint32_t BRP = (MCP251X_NBRP_MAX + 1);                                                       // Max of possible BRP
    while (--BRP > 0)
    {
      pConf->Valid = false;
      //--- Step 2 ---
      if (BRP > MCP251X_NBRP_MAX) break;                                                         // Adjust  and check maximum BRP
      const uint32_t NTQ = (SysClkDivided / (BRP * (pBusConf.DesiredBitrate / 100)) + 50) / 100; // Get NTQ rounded
      if (NTQ == 0) continue;                                                                    // Avoid divide by 0
      const uint32_t ActualBitrate = (SysClkDivided / (NTQ * BRP));                              // Actual bitrate
      int32_t ErrorBitrate = ((int32_t)(ActualBitrate * 100) / (int32_t)(pBusConf.DesiredBitrate / 100)) - 10000; // Bitrate deviation
      if (SearchExactBR && (ErrorBitrate != 0)) continue;                                        // Check the baudrate exactness if ask
      if (ErrorBitrate < 0) ErrorBitrate = -ErrorBitrate;                                        // Get absolute of bitrate deviation
      if (ErrorBitrate > MCP251X_UINT_MAX_OSC_TOL) continue;                                     // Check the maximum allowable tolerance

      //--- Step 3 ---
      const uint32_t TQ = 1000000 / (SysClkDivided / (BRP * 1000));                              // Get time quantum for this BRP
      uint32_t PropSeg = (((tPropSeg * 100) / TQ) + 99) / 100;                                   // Formula is PROP_SEG(bits) = ROUND_UP(tPROP_SEG/TimeQuantum)
      if (PropSeg < MCP251X_NPRSEG_MIN) PropSeg = MCP251X_NPRSEG_MIN;                            // Correct minimum NPRSEG
      if (NTQ < (MCP251X_NSYNC + PropSeg + MCP251X_NTSEG1_MIN + MCP251X_NTSEG2_MIN)) continue;   // Need a minimum TQbits for this bus length, then do the next NBRP value
      if (NTQ > (MCP251X_NSYNC + PropSeg + MCP251X_NTSEG1_MAX + MCP251X_NTSEG2_MAX)) continue;   // Higher than the maximum TQbits for this bus length, then do the next NBRP value
      pConf->NBRP = BRP;                                                                         // ** Save the NBRP in the configuration **
      if (pConf->NBRP < MCP251X_NBRP_MIN) pConf->NBRP = MCP251X_NBRP_MIN;                        // Correct NBRP min
      if (pConf->NBRP > MCP251X_NBRP_MAX) pConf->NBRP = MCP251X_NBRP_MAX;                        // Correct NBRP max

      //--- Step 4 ---
      uint32_t TQbits = NTQ;                                                                     // TQbits = NTQ
      TQbits -= (MCP251X_NSYNC + PropSeg);                                                       // Get remaining TQbits after removing NSYNC and PROP_SEG bits
      if (((TQbits & 1) > 0) && (TQbits != 3)) { ++PropSeg; --TQbits; }                          // If TQbits is odd and different of 3 (which have a special threatment), add one to PROP_SEG and remove it from TQbits
      pConf->NPRSEG = PropSeg;                                                                   // ** Save the NPRSEG in the configuration **
      if (pConf->NPRSEG > MCP251X_NPRSEG_MAX) continue;                                          // If NPRSEG > maximum, impossible solution
      pConf->NTSEG1 = TQbits >> 1;                                                               // ** Save the NSEG1PH = TQbits / 2 in the configuration **
      if (pConf->NTSEG1 < MCP251X_NTSEG1_MIN) pConf->NTSEG1 = MCP251X_NTSEG1_MIN;                // Correct NTSEG1 min
      if (pConf->NTSEG1 > MCP251X_NTSEG1_MAX) pConf->NTSEG1 = MCP251X_NTSEG1_MAX;                // Correct NTSEG1 max
      pConf->NTSEG2 = (TQbits >> 1) + (TQbits == 3 ? 1 : 0);                                     // ** Save the NSEG2PH = TQbits / 2 in the configuration **
      if (pConf->NTSEG2 < MCP251X_NTSEG2_MIN) pConf->NTSEG2 = MCP251X_NTSEG2_MIN;                // Correct NSEG2PH min
      if (pConf->NTSEG2 > MCP251X_NTSEG2_MAX) pConf->NTSEG2 = MCP251X_NTSEG2_MAX;                // Correct NSEG2PH max

      //--- Step 5 ---
      pConf->NSJW = ((pConf->NTSEG1 < 4) ? pConf->NTSEG1 : 4);                                   // ** Save the NSJW = min(4, SEG1PH) in the configuration **
      if (pConf->NSJW < MCP251X_NSJW_MIN) pConf->NSJW = MCP251X_NSJW_MIN;                        // Correct NSJW min
      if (pConf->NSJW > MCP251X_NSJW_MAX) pConf->NSJW = MCP251X_NSJW_MAX;                        // Correct NSJW max

      pConf->Valid = true;                                                                       // ** Mark as valid configuration **
      if (pConf->NTSEG1 > 4)
      {
        if (LastBTconf.Valid)
        {
          //--- Get minimum current oscillator tolerance ---
          uint32_t MinCurrOscTol, CurrOscTol;
          uint32_t MinNPHSEG = (pConf->NTSEG1 <= pConf->NTSEG2 ? pConf->NTSEG1 : pConf->NTSEG2); // Get min(NTSEG1, NTSEG2) of current conf
          MinCurrOscTol = ((pConf->NSJW * 10000) / (2 * 10 * NTQ));                              // Condition 1 for the tolerance of the oscillator with 2 digits after the decimal point [df <= NSJW/(2*10*NBT/NTQ)]
          CurrOscTol = ((MinNPHSEG * 10000) / (2 * (13 * NTQ - pConf->NTSEG2)));                 // Condition 2 for the tolerance of the oscillator with 2 digits after the decimal point [df <= min(NTSEG1,NTSEG2)/(2*(13*NBT/NTQ-NTSEG2))]
          MinCurrOscTol = (CurrOscTol < MinCurrOscTol ? CurrOscTol : MinCurrOscTol);             // Get Minimum Current Oscillator Tolerance

          //--- Get minimum last oscillator tolerance ---
          uint32_t MinLastOscTol, LastOscTol;
          uint32_t LastNTQ = (MCP251X_NSYNC + LastBTconf.NPRSEG + LastBTconf.NTSEG1 + LastBTconf.NTSEG2); // Get last NTQ
          MinNPHSEG = (LastBTconf.NTSEG1 <= LastBTconf.NTSEG2 ? LastBTconf.NTSEG1 : LastBTconf.NTSEG2); // Get min(NTSEG1, NTSEG2) of last conf
          MinLastOscTol = ((LastBTconf.NSJW * 10000) / (2 * 10 * LastNTQ));                      // Condition 1 for the tolerance of the oscillator with 2 digits after the decimal point [df <= NSJW/(2*10*NBT/NTQ)]
          LastOscTol = ((MinNPHSEG * 10000) / (2 * (13 * LastNTQ - LastBTconf.NTSEG2)));         // Condition 2 for the tolerance of the oscillator with 2 digits after the decimal point [df <= min(NTSEG1,NTSEG2)/(2*(13*NBT/NTQ-NTSEG2))]
          MinLastOscTol = (LastOscTol < MinLastOscTol ? LastOscTol : MinLastOscTol);             // Get Minimum Last Oscillator Tolerance

          //--- Check best oscillator tolerance ---
          if (MinLastOscTol > MinCurrOscTol) *pConf = LastBTconf;                                // Select the last configuration if the last have a better oscillator tolerance
        }
        break;                                                                                   // Cannot get a better configuration so finish
      }
      LastBTconf = *pConf;                                                                       // ** Save this valid configuration **
      if (pConf->NTSEG1 == 4) break;                                                             // Perfect configuration found
    }
    if (SearchExactBR && (BRP == 0)) { SearchExactBR = false; continue; }                        // No exact baudrate found, check again but with non-exact baudrate accepted
  } while (0);
  if (pConf->Valid == false) *pConf = LastBTconf;                                                // If the current is not valid, get the last good configuration
  pConf->DBRP       = 0;                                                                         // ** Set the DBRP in the configuration **
  pConf->DPRSEG     = 0;                                                                         // ** Set the DPRSEG in the configuration **
  pConf->DTSEG1     = 0;                                                                         // ** Set the DTSEG1 in the configuration **
  pConf->DTSEG2     = 0;                                                                         // ** Set the DTSEG2 in the configuration **
  pConf->DSJW       = 0;                                                                         // ** Set the DSJW in the configuration **
  pConf->TDCO       = 0;                                                                         // ** Set the TDCO in the configuration **
  pConf->TDCV       = 0;                                                                         // ** Set the TDCV in the configuration **
  pConf->SAMPL      = (pBusConf.DesiredBitrate > CAN_SAE_CLASS_B_SPEED_MAX ? 1 : 3);             // ** Set the SAMPL in the configuration ** (1 sample if SAE class C, else 3)
  pConf->PS2mode    = CAN_PS2_BLT_PHSEG2;                                                        // ** Set the PhaseSeg2 mode in the configuration **
  pConf->EdgeFilter = false;                                                                     // ** Edge Filtering enabled, according to ISO 11898-1:2015 **
  pConf->CAN20only  = true;                                                                      // ** Set the CAN2.0 only configuration **

  eERRORRESULT Error = ERR_OK;
  if (pConf->Stats != NULL)
      Error = MCP251X_CalculateBitrateStatistics(periphClk, pConf);                              // If statistics are necessary, then calculate them
  return Error;
}


//=============================================================================
// Calculate Bitrate Statistics of a Bit Time configuration
//=============================================================================
eERRORRESULT MCP251X_CalculateBitrateStatistics(const uint32_t periphClk, const CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
  if (pConf->Stats == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Calculate bus length & Nominal Sample Point ---------
  const uint32_t NTQ = (((pConf->NBRP+1) * 1000000) / (periphClk / 1000));          // Nominal Time Quanta = 1/PERIPHCLK multiply by 1000000000 to get ns
  const uint32_t NPRSEG  = (pConf->NTSEG1+1) - (pConf->NTSEG2+1);                   // Here PHSEG2 (NTSEG2) should be equal to PHSEG1 so NPRSEG = NTSEG1 - NTSEG2
  pConf->Stats->MaxBusLength = (uint32_t)(((NTQ * NPRSEG) - (2 * MCP251X_tTXDtRXD_MAX)) / (2 * MCP251X_tBUS_CONV)); // Formula is (2x(tTXD–RXD + (5*BusLen))/NTQ = NPRSEG => BusLen = ((NTQ*NPRESG)-(2*tTXD))/(2*5) in meter
  const uint32_t NTQbits = (MCP251X_NSYNC + (pConf->NTSEG1+1) + (pConf->NTSEG2+1)); // NTQ per bits = NSYNC + NTSEG1 + NTSEG2
  uint32_t SamplePoint = ((MCP251X_NSYNC + (pConf->NTSEG1+1)) * 100) / NTQbits;     // Calculate actual nominal sample point
  pConf->Stats->NSamplePoint = (uint32_t)(SamplePoint * 100);                       // ** Save actual Nominal sample point with 2 digits after the decimal point (divide by 100 to get percentage)
  pConf->Stats->NominalBitrate = (periphClk / (pConf->NBRP+1) / NTQbits);           // ** Save actual Nominal Bitrate
  pConf->Stats->DSamplePoint = 0;                                                   // ** Set actual Data sample point
  pConf->Stats->DataBitrate  = 0;                                                   // ** Set actual Data Bitrate

  //--- Calculate oscillator tolerance ----------------------
  const uint16_t NPHSEG1     = (pConf->NTSEG1+1) - NPRSEG;                                                                                  // Get NPHSEG1
  const uint16_t MinNPHSEG   = (NPHSEG1 <= (pConf->NTSEG2+1) ? NPHSEG1 : (pConf->NTSEG2+1));                                                // Get min(NPHSEG1, NPHSEG2)
  pConf->Stats->OscTolC1     = (((pConf->NSJW+1) * 10000) / (2 * 10 * NTQbits));                                                            // Condition 1 for the maximum tolerance of the oscillator with 2 digits after the decimal point
  pConf->Stats->OscTolerance = pConf->Stats->OscTolC1;
  pConf->Stats->OscTolC2     = ((MinNPHSEG * 10000) / (2 * (13 * NTQbits - (pConf->NTSEG2+1))));                                            // Condition 2 for the maximum tolerance of the oscillator with 2 digits after the decimal point
  pConf->Stats->OscTolerance = (pConf->Stats->OscTolC2 < pConf->Stats->OscTolerance ? pConf->Stats->OscTolC2 : pConf->Stats->OscTolerance); // Oscillator Tolerance, minimum of conditions 1-5
  pConf->Stats->OscTolC3   = 0;
  pConf->Stats->OscTolC4   = 0;
  pConf->Stats->OscTolC5   = 0;
  return ERR_OK;
}
#endif


//=============================================================================
// Set Bit Time Configuration to the MCP251X peripheral
//=============================================================================
eERRORRESULT MCP251X_SetBitTimeConfiguration(MCP251X *pComp, const CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (pConf->Valid == false) return ERR__INVALID_DATA;

  //--- Write Nominal Bit Time configuration ---
  uint8_t CfgCNF3 = MCP251X_CNF3_PHSEG2_SET(pConf->NTSEG2),                                        // Set Nominal Bit Time configuration (NTSEG1)
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CNF3, CfgCNF3, MCP251X_CNF3_PHSEG2_Mask);       // ** Write CNF3 configuration **
  if (Error != ERR_OK) return Error;                                                               // If there is an error while calling MCP251X_ModifyRegister() then return the error
  uint8_t CfgCNF2 = MCP251X_CNF2_PRSEG_SET(pConf->NPRSEG) | MCP251X_CNF2_PHSEG1_SET(pConf->NTSEG1) // Set Nominal Bit Time configuration (NPRSEG and NTSEG1)
                  | (pConf->PS2mode == CAN_PS2_BLT_PHSEG2 ? MCP251X_CNF2_BTLMODE_IS_PHSEG2 : MCP251X_CNF2_BTLMODE_IS_PS1_IPT) // Set PS2 Bit Time Length mode
                  | (pConf->SAMPL > 1 ?MCP251X_CNF2_3_SAMPLES : MCP251X_CNF2_1_SAMPLE);            // Set Sample Point Configuration
  Error = MCP251X_WriteRegister(pComp, RegMCP251X_CNF2, &CfgCNF2, sizeof(CfgCNF2));                // ** Write CNF2 configuration **
  if (Error != ERR_OK) return Error;                                                               // If there is an error while calling MCP251X_WriteRegister() then return the error
  uint8_t CfgCNF1 = MCP251X_CNF1_BRP_SET(pConf->NBRP) | MCP251X_CNF1_SJW_SET(pConf->NSJW);         // Set Nominal Bit Time configuration (BRP and SJW)
  return MCP251X_WriteRegister(pComp, RegMCP251X_CNF1, &CfgCNF1, sizeof(CfgCNF1));                 // ** Write CNF1 configuration **
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Get actual operation mode of the MCP251X peripheral
//=============================================================================
eERRORRESULT MCP251X_GetActualOperationMode(MCP251X *pComp, eMCP251X_OperationMode* const actualMode)
{
#ifdef CHECK_NULL_PARAM
  if (actualMode == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;
  Error = MCP251X_ReadRegister(pComp, RegMCP251X_CANSTAT1, &Config, sizeof(Config));
  *actualMode = MCP251X_CANSTAT_OPMOD_GET(Config);
  return Error;
}


//=============================================================================
// Request operation mode change of the MCP251X peripheral
//=============================================================================
eERRORRESULT MCP251X_RequestOperationMode(MCP251X *pComp, eMCP251X_OperationMode newMode, bool waitOperationChange)
{
  eERRORRESULT Error;
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CANCTRL1, MCP251X_CANCTRL_REQOP_SET(newMode) | MCP251X_CANCTRL_ABAT, MCP251X_CANCTRL_REQOP_Mask | MCP251X_CANCTRL_ABAT);
  if (waitOperationChange)
  {
    if (Error != ERR_OK) return Error;                       // If there is an error while calling MCP251X_ModifyRegister() then return the error
    Error = MCP251X_WaitOperationModeChange(pComp, newMode); // Wait for operation mode change
  }
  return Error;
}


//=============================================================================
// Wait for operation mode change of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_WaitOperationModeChange(MCP251X *pComp, eMCP251X_OperationMode askedMode)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eMCP251X_OperationMode ActualMode;
  eERRORRESULT Error;

  uint32_t StartTime = pComp->fnGetCurrentms();                    // Start the timeout
  while (true)
  {
    Error = MCP251X_GetActualOperationMode(pComp, &ActualMode);    // Read current configuration mode with the current driver configuration
    if (Error != ERR_OK) return Error;                             // If there is an error while calling MCP251X_GetActualOperationMode() then return the error
    if (ActualMode == askedMode) break;                            // Check if the controller is in configuration mode
    if (MCP251X_TIME_DIFF(StartTime, pComp->fnGetCurrentms()) > 7) // Wait at least 7ms because the longest message is 731 bit long at a bitrate of 125kbit/s that mean 5,8ms + 2x 6bytes @ 1Mbit/s over SPI that mean 96µs = ~6ms + 1ms because GetCurrentms can be 1 cycle before the new ms
      return ERR__DEVICE_TIMEOUT;                                  // Timeout? return the error
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure the filters of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ConfigureFilters(MCP251X *pComp, const MCP251X_Filter0* const confFilter0, const MCP251X_Filter1* const confFilter1)
{
#ifdef CHECK_NULL_PARAM
  if ((confFilter0 == NULL) || (confFilter1 == NULL)) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Buffer[MCP251X_CAN_FILTER_ID_SIZE_MAX * 3]; // Up to 3 programmed filters at once
  eERRORRESULT Error;

  //--- Set filter acceptance RXF0, RXF1, RXF2 ---
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[0], confFilter0->AcceptanceID0, (confFilter0->MatchID0 == MCP251X_MATCH_SID_EID), false);
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[4], confFilter0->AcceptanceID1, (confFilter0->MatchID1 == MCP251X_MATCH_SID_EID), false);
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[8], confFilter1->AcceptanceID2, (confFilter1->MatchID2 == MCP251X_MATCH_SID_EID), false);
  Error = MCP251X_WriteRegister(pComp, RegMCP251X_RXF0SIDH, &Buffer[0], sizeof(Buffer)); // Set filter acceptance RXF0, RXF1, RXF2
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_GetActualOperationMode() then return the error

  //--- Set filter acceptance RXF3, RXF4, RXF5 ---
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[0], confFilter1->AcceptanceID3, (confFilter1->MatchID3 == MCP251X_MATCH_SID_EID), false);
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[4], confFilter1->AcceptanceID4, (confFilter1->MatchID4 == MCP251X_MATCH_SID_EID), false);
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[8], confFilter1->AcceptanceID5, (confFilter1->MatchID5 == MCP251X_MATCH_SID_EID), false);
  Error = MCP251X_WriteRegister(pComp, RegMCP251X_RXF3SIDH, &Buffer[0], sizeof(Buffer)); // Set filter acceptance RXF3, RXF4, RXF5
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_GetActualOperationMode() then return the error

  //--- Set filter acceptance RXM0, RXM1 ---
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[0], confFilter0->AcceptanceMask0, true, true);
  MCP251X_MessageIDtoFilterObjectMessageIdentifier(&Buffer[4], confFilter1->AcceptanceMask1, true, true);
  return MCP251X_WriteRegister(pComp, RegMCP251X_RXM0SIDH, &Buffer[0], MCP251X_CAN_FILTER_ID_SIZE_MAX * 2); // Set filter mask RXM0, RXM1
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Sleep mode configuration of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ConfigureSleepMode(MCP251X *pComp, bool wakeUpFilter)
{
  eERRORRESULT Error;

  //--- Configure interrupt filter ---
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_CANINTE, MCP251X_WAKIE_EVENT, MCP251X_WAKIE_Mask);
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_ModifyRegister() then return the error
  pComp->InternalConfig &= ~MCP251X_DEV_PS_Mask;
  pComp->InternalConfig |= MCP251X_DEV_PS_SET(MCP251X_DEVICE_NORMAL_POWER_STATE);

  //--- Configure Wakeup filter ---
  return MCP251X_ModifyRegister(pComp, RegMCP251X_CNF3, (wakeUpFilter ? MCP251X_CNF3_WAKEUP_FILTER_EN : MCP251X_CNF3_WAKEUP_FILTER_DIS), MCP251X_CNF3_WAKEUP_FILTER_Mask);
}


//=============================================================================
// Verify if the MCP251X device is in sleep mode
//=============================================================================
eERRORRESULT MCP251X_IsDeviceInSleepMode(MCP251X *pComp, bool* const isInSleepMode)
{
#ifdef CHECK_NULL_PARAM
  if (isInSleepMode == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  eMCP251X_OperationMode ActualMode;
  Error = MCP251X_GetActualOperationMode(pComp, &ActualMode);
  *isInSleepMode = (ActualMode == MCP251X_SLEEP_MODE);
  pComp->InternalConfig &= ~MCP251X_DEV_PS_Mask;
  if (*isInSleepMode)
       pComp->InternalConfig |= MCP251X_DEV_PS_SET(MCP251X_DEVICE_SLEEP_STATE);
  else pComp->InternalConfig |= MCP251X_DEV_PS_SET(MCP251X_DEVICE_NORMAL_POWER_STATE);
  return Error;
}


//=============================================================================
// Manually wake up the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_WakeUp(MCP251X *pComp)
{
  pComp->InternalConfig &= ~MCP251X_DEV_PS_Mask;
  pComp->InternalConfig |= MCP251X_DEV_PS_SET(MCP251X_DEVICE_NORMAL_POWER_STATE);
  return MCP251X_ModifyRegister(pComp, RegMCP251X_CANINTF, MCP251X_WAKIE_EVENT, MCP251X_WAKIE_Mask);
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure interrupt of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ConfigureInterrupt(MCP251X *pComp, setMCP251X_InterruptEvents interruptsFlags)
{
  uint8_t Config = (uint8_t)(interruptsFlags & MCP251X_INT_EVENTS_FLAGS_MASK);
  return MCP251X_WriteRegister(pComp, RegMCP251X_CANINTE, &Config, sizeof(Config));
}


//=============================================================================
// Get interrupt events of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_GetInterruptEvents(MCP251X *pComp, setMCP251X_InterruptEvents* const interruptsFlags)
{
#ifdef CHECK_NULL_PARAM
  if (interruptsFlags == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;
  Error = MCP251X_ReadRegister(pComp, RegMCP251X_CANINTF, &Config, sizeof(Config));
  *interruptsFlags = (setMCP251X_InterruptEvents)Config;
  return Error;
}


//=============================================================================
// Get interrupt code of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_GetInterruptCode(MCP251X *pComp, eMCP251X_IntFlagCode* interruptsCode)
{
#ifdef CHECK_NULL_PARAM
  if (interruptsCode == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;
  Error = MCP251X_ReadRegister(pComp, RegMCP251X_CANSTAT0, &Config, sizeof(Config));
  *interruptsCode = MCP251X_CANSTAT_INT_FLAG_CODE_GET(Config);
  return Error;
}


//=============================================================================
// Clear interrupt events of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ClearInterruptEvents(MCP251X *pComp, setMCP251X_InterruptEvents interruptsFlags)
{
  return MCP251X_ModifyRegister(pComp, RegMCP251X_CANINTF, 0x00, (uint8_t)interruptsFlags);
}


//=============================================================================
// Get status events of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_GetStatusEvents(MCP251X *pComp, setMCP251X_StatusEvents* const statusFlags)
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

  //--- Send instruction packet ---
  uint8_t Buffer[2] = { MCP251X_SPI_INSTRUCTION_READ_STATUS, 0x00 };
  SPIInterface_Packet DataPacketDesc = SPI_INTERFACE_RX_DATA_DESC(&Buffer[0], sizeof(Buffer), true);
  Error =  pSPI->fnSPI_Transfer(pSPI, &DataPacketDesc); // Continue by reading the data, get the data and stop transfer at last byte
  *statusFlags = (setMCP251X_StatusEvents)Buffer[1];
  return Error;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure all Buffers of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_ConfigureBuffers(MCP251X *pComp, eMCP251X_Priority tx0Priority, eMCP251X_Priority tx1Priority, eMCP251X_Priority tx2Priority, setMCP251X_RxBufferConfig rx0Conf, setMCP251X_RxBufferConfig rx1Conf)
{
  eERRORRESULT Error;

  //--- Configure TXBnCTRL ---
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_TXB0CTRL, MCP251X_TX_MESSAGE_PRIORITY_SET(tx0Priority), MCP251X_TX_MESSAGE_PRIORITY_Mask);
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_ModifyRegister() then return the error
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_TXB1CTRL, MCP251X_TX_MESSAGE_PRIORITY_SET(tx1Priority), MCP251X_TX_MESSAGE_PRIORITY_Mask);
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_ModifyRegister() then return the error
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_TXB2CTRL, MCP251X_TX_MESSAGE_PRIORITY_SET(tx2Priority), MCP251X_TX_MESSAGE_PRIORITY_Mask);
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_ModifyRegister() then return the error

  //--- Configure RXBnCTRL ---
  Error = MCP251X_ModifyRegister(pComp, RegMCP251X_RXB0CTRL, rx0Conf, MCP251X_FILTER_MODE_Mask | MCP251X_ROLLOVER_MESSAGE_Mask);
  if (Error != ERR_OK) return Error; // If there is an error while calling MCP251X_ModifyRegister() then return the error
  return MCP251X_ModifyRegister(pComp, RegMCP251X_RXB1CTRL, rx1Conf, MCP251X_FILTER_MODE_Mask);
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Get error state of a Tx Buffer of the MCP251X device
//=============================================================================
eERRORRESULT MCP251X_GetTxBufferErrorStatus(MCP251X *pComp, uint8_t bufferIdx, setMCP251X_TxBufferErrorStatus *statusFlags)
{
#ifdef CHECK_NULL_PARAM
  if (statusFlags == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;
  Error = MCP251X_ReadRegister(pComp, RegMCP251X_TXBnCTRL(bufferIdx), &Config, sizeof(Config));
  *statusFlags = (setMCP251X_TxBufferErrorStatus)(Config & MCP251X_TX_BUFFER_ERR_STATUS_MASK);
  return Error;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Message ID to Object Message Identifier for Filter
//=============================================================================
void MCP251X_MessageIDtoFilterObjectMessageIdentifier(uint8_t* buffer, uint32_t messageID, bool extended, bool isMask)
{
  if (extended) // Extended ID?
  {
    buffer[MCP251X_CAN_TXMSG_STDH] = MCP251X_CAN_SID_10_3_SET(messageID >> (3 + CAN_EID_Size));
    buffer[MCP251X_CAN_TXMSG_STDL] = MCP251X_CAN_SID_2_0_SET(messageID >> CAN_EID_Size)
                                   | (isMask ? 0 : MCP251X_CAN_EXIDE_EXTENDED_ID)
                                   | MCP251X_CAN_EID_17_16_SET(messageID >> 16);
    buffer[MCP251X_CAN_TXMSG_EIDH] = MCP251X_CAN_EID_15_8_SET(messageID >> 8);
    buffer[MCP251X_CAN_TXMSG_EIDL] = MCP251X_CAN_EID_7_0_SET(messageID >> 0);
  }
  else // Standard ID
  {
    buffer[MCP251X_CAN_TXMSG_STDH] = MCP251X_CAN_SID_10_3_SET(messageID >> 3);
    buffer[MCP251X_CAN_TXMSG_STDL] = MCP251X_CAN_SID_2_0_SET(messageID);
    buffer[MCP251X_CAN_TXMSG_EIDH] = 0;
    buffer[MCP251X_CAN_TXMSG_EIDL] = 0;
  }
}


//=============================================================================
// Message ID to Object Message Identifier for Tx Messages
//=============================================================================
void MCP251X_MessageIDtoTxObjectMessageIdentifier(MCP251X_CAN_TxMessage* txMessage, uint32_t messageID, bool extended)
{
  if (extended) // Extended ID?
  {
    txMessage->Bytes[MCP251X_CAN_TXMSG_STDH] = MCP251X_CAN_SID_10_3_SET(messageID >> (3 + CAN_EID_Size));
    txMessage->Bytes[MCP251X_CAN_TXMSG_STDL] = MCP251X_CAN_SID_2_0_SET(messageID >> CAN_EID_Size)
                                             | MCP251X_CAN_EXIDE_EXTENDED_ID
                                             | MCP251X_CAN_EID_17_16_SET(messageID >> 16);
    txMessage->Bytes[MCP251X_CAN_TXMSG_EIDH] = MCP251X_CAN_EID_15_8_SET(messageID >> 8);
    txMessage->Bytes[MCP251X_CAN_TXMSG_EIDL] = MCP251X_CAN_EID_7_0_SET(messageID >> 0);
  }
  else // Standard ID
  {
    txMessage->Bytes[MCP251X_CAN_TXMSG_STDH] = MCP251X_CAN_SID_10_3_SET(messageID >> 3);
    txMessage->Bytes[MCP251X_CAN_TXMSG_STDL] = MCP251X_CAN_SID_2_0_SET(messageID);
    txMessage->Bytes[MCP251X_CAN_TXMSG_EIDH] = 0;
    txMessage->Bytes[MCP251X_CAN_TXMSG_EIDL] = 0;
  }
}


//=============================================================================
// Object Message Identifier for Rx Messages to Message ID
//=============================================================================
uint32_t MCP251X_RxObjectMessageIdentifierToMessageID(MCP251X_CAN_RxMessage* rxMessage)
{
  uint32_t ResultID = 0; // Initialize message ID to 0

  if ((rxMessage->Bytes[MCP251X_CAN_TXMSG_STDL] & MCP251X_CAN_EXIDE_EXTENDED_ID) > 0) // Extended ID?
  {
    ResultID |= MCP251X_CAN_SID_10_3_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_STDH]) << (3 + CAN_EID_Size);
    ResultID |= MCP251X_CAN_SID_2_0_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_STDL]) << CAN_EID_Size;
    ResultID |= MCP251X_CAN_EID_17_16_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_STDL]) << 16;
    ResultID |= MCP251X_CAN_EID_15_8_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_EIDH]) << 8;
    ResultID |= MCP251X_CAN_EID_7_0_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_EIDL]) << 0;
  }
  else // Standard ID
  {
    ResultID |= MCP251X_CAN_SID_10_3_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_STDH]) << 3;
    ResultID |= MCP251X_CAN_SID_2_0_GET(rxMessage->Bytes[MCP251X_CAN_TXMSG_STDL]) << 0;
  }
  return ResultID;
}

//-----------------------------------------------------------------------------
#ifdef _cplusplus
}
#endif
//-----------------------------------------------------------------------------