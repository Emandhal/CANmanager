/*!*****************************************************************************
 * @file    SJA1000.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    14/07/2023
 * @brief   SJA1000 driver
 * @details Stand-alone CAN controller
 * Follow datasheet SJA1000 Rev 3.0 (January 2004)
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "SJA1000.h"
//-----------------------------------------------------------------------------
#ifdef _cplusplus
# include <cstdint>
  extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_DYNAMIC_INTERFACE
#  define GET_CS_INTERFACE      pComp->CS
#  define GET_ALE_AS_INTERFACE  pComp->ALE_AS
#  define GET_RD_E_INTERFACE    pComp->RD_E
#  define GET_WR_INTERFACE      pComp->WR
#  define GET_DATA_INTERFACE    pComp->DATA
#else
#  define GET_CS_INTERFACE      &pComp->CS
#  define GET_ALE_AS_INTERFACE  &pComp->ALE_AS
#  define GET_RD_E_INTERFACE    &pComp->RD_E
#  define GET_WR_INTERFACE      &pComp->WR
#  define GET_DATA_INTERFACE    &pComp->DATA
#endif

//-----------------------------------------------------------------------------





//=============================================================================
// Prototypes for private functions
//=============================================================================

//-----------------------------------------------------------------------------
#define SJA1000_PIN_LOW(pin)           Error = p##pin->fnGPIO_SetState(p##pin, GPIO_STATE_RESET)
#define SJA1000_PIN_HIGH(pin)          Error = p##pin->fnGPIO_SetState(p##pin, GPIO_STATE_SET)
#define SJA1000_PIN_OUTPUT(pin)        Error = p##pin->fnGPIO_SetState(p##pin, GPIO_STATE_OUTPUT)
#define SJA1000_PORT_OUTPUT(port)      Error = p##port->fnPORT_SetDirection(p##port, PORT_AS_OUTPUT)
#define SJA1000_PORT_INPUT(port)       Error = p##port->fnPORT_SetDirection(p##port, PORT_AS_INPUT)
#define SJA1000_PORT_WRITE(port,data)  Error = p##port->fnPORT_SetOutputLevel(p##port, (uint8_t)(data))
#define SJA1000_PORT_READ(port)        ({ uint32_t Read##__LINE__; Error = p##port->fnPORT_GetInputLevel(p##port, &Read##__LINE__); (uint8_t)Read##__LINE__; }) // Statement expression
#define __SJA1000_CHECK_ERROR          if (Error != ERR_NONE) return Error
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// SJA1000 initialization
//=============================================================================
eERRORRESULT Init_SJA1000(SJA1000 *pComp, const SJA1000_Config* pConf)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (pConf == NULL)) return ERR__PARAMETER_ERROR;
#endif
  GPIO_Interface* pCS     = GET_CS_INTERFACE;
  GPIO_Interface* pALE_AS = GET_ALE_AS_INTERFACE;
  GPIO_Interface* pRD_E   = GET_RD_E_INTERFACE;
  GPIO_Interface* pWR     = GET_WR_INTERFACE;
  PORT_Interface* pDATA   = GET_DATA_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if ((pCS == NULL) || (pALE_AS == NULL) || (pRD_E == NULL) || (pWR == NULL) || (pDATA == NULL)) return ERR__PARAMETER_ERROR;
# endif
  if ((pCS->fnGPIO_SetState == NULL) || (pALE_AS->fnGPIO_SetState == NULL) || (pRD_E->fnGPIO_SetState == NULL) || (pWR->fnGPIO_SetState == NULL)) return ERR__PARAMETER_ERROR;
  if ((pDATA->fnPORT_SetDirection == NULL) || (pDATA->fnPORT_GetInputLevel == NULL) || (pDATA->fnPORT_SetOutputLevel == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Check configuration ---
  if (pConf->XtalFreq > SJA1000_XTALFREQ_MAX) return ERR__FREQUENCY_ERROR; // The device crystal should not be > 24MHz

  //--- Configure control pins and data port ---
  SJA1000_PIN_HIGH(CS);       __SJA1000_CHECK_ERROR; // Set CS pin high level
  SJA1000_PIN_OUTPUT(CS);     __SJA1000_CHECK_ERROR; // Set CS pin as output
  SJA1000_PIN_LOW(ALE_AS);    __SJA1000_CHECK_ERROR; // Set ALE/AS pin low level
  SJA1000_PIN_OUTPUT(ALE_AS); __SJA1000_CHECK_ERROR; // Set ALE/AS pin as output
#ifdef SJA1000_USE_INTELMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_INTEL_MODE)
# endif
  {
    SJA1000_PIN_HIGH(RD_E);                          // Set RD/E pin high level
  }
#endif
#ifdef SJA1000_USE_MOTOROLAMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_MOTOROLA_MODE)
# endif
  {
    SJA1000_PIN_LOW(RD_E);                           // Set RD/E pin low level
  }
#endif
  __SJA1000_CHECK_ERROR;
  SJA1000_PIN_HIGH(WR);       __SJA1000_CHECK_ERROR; // Set WR pin high level
  SJA1000_PIN_OUTPUT(RD_E);   __SJA1000_CHECK_ERROR; // Set RD/E pin as output
  SJA1000_PIN_OUTPUT(WR);     __SJA1000_CHECK_ERROR; // Set RD/E pin as output
  SJA1000_PORT_OUTPUT(DATA);  __SJA1000_CHECK_ERROR; // Set DATA port as output

  //--- Set configuration mode ---
  Error = SJA1000_RequestOperationMode(pComp, SJA1000_MODE_RESET);
  __SJA1000_CHECK_ERROR;

  //--- Configure controller pins ---
  Error = SJA1000_ConfigurePins(pComp, pConf->outMode, pConf->tx0PinMode, pConf->tx1PinMode, pConf->clkoutMode, pConf->tx1AsRxInt, pConf->bypassRxComp);
  __SJA1000_CHECK_ERROR;

  //--- Set nominal bitrate ---
  CAN_BitTimeConfig* ConfBitTime;
#if defined(SJA1000_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
  CAN_BitTimeConfig BitTimeConfig;
  ConfBitTime = &BitTimeConfig;
  Error = SJA1000_CalculateBitTimeConfiguration(pConf->XtalFreq, pConf->BusConfig, ConfBitTime); // Calculate Bit Time
  __SJA1000_CHECK_ERROR;
  ConfBitTime->Stats = pConf->BitTimeStats;
#else
  ConfBitTime = &pConf->BitTimeConfig;
#endif
  Error = SJA1000_SetBitTimeConfiguration(pComp, ConfBitTime); // Set Bit Time configuration to registers
  __SJA1000_CHECK_ERROR;

  //--- Configure interrupts ---
  return SJA1000_ConfigureInterrupt(pComp, pConf->Interrupts); // Configure interrupts
}

//-----------------------------------------------------------------------------




//**********************************************************************************************************************************************************
//=============================================================================
// Read data from register of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ReadRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t* const data)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  GPIO_Interface* pCS     = GET_CS_INTERFACE;
  GPIO_Interface* pALE_AS = GET_ALE_AS_INTERFACE;
  GPIO_Interface* pRD_E   = GET_RD_E_INTERFACE;
  GPIO_Interface* pWR     = GET_WR_INTERFACE;
  PORT_Interface* pDATA   = GET_DATA_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if ((pCS == NULL) || (pALE_AS == NULL) || (pRD_E == NULL) || (pWR == NULL) || (pDATA == NULL)) return ERR__PARAMETER_ERROR;
# endif
  if ((pCS->fnGPIO_SetState == NULL) || (pALE_AS->fnGPIO_SetState == NULL) || (pRD_E->fnGPIO_SetState == NULL) || (pWR->fnGPIO_SetState == NULL)) return ERR__PARAMETER_ERROR;
  if ((pDATA->fnPORT_SetDirection == NULL) || (pDATA->fnPORT_GetInputLevel == NULL) || (pDATA->fnPORT_SetOutputLevel == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

#ifdef SJA1000_USE_INTELMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_INTEL_MODE)
# endif
  {
    //--- Send the address ---
    SJA1000_PIN_HIGH(ALE_AS);        __SJA1000_CHECK_ERROR; // Set ALE/AS pin high level
    SJA1000_PORT_WRITE(DATA, reg);   __SJA1000_CHECK_ERROR; // Set DATA port output with register address
    SJA1000_PIN_LOW(ALE_AS);         __SJA1000_CHECK_ERROR; // Set ALE/AS pin low level

    //--- Read data ---
    SJA1000_PIN_LOW(CS);             __SJA1000_CHECK_ERROR; // Set CS pin low level
    SJA1000_PIN_LOW(RD_E);           __SJA1000_CHECK_ERROR; // Set RD/E pin low level
    SJA1000_PORT_INPUT(DATA);        __SJA1000_CHECK_ERROR; // Set DATA port as input
    *data = SJA1000_PORT_READ(DATA); __SJA1000_CHECK_ERROR; // Get DATA port input with data
    SJA1000_PIN_HIGH(RD_E);          __SJA1000_CHECK_ERROR; // Set RD/E pin high level
    SJA1000_PIN_HIGH(CS);            __SJA1000_CHECK_ERROR; // Set CS pin high level
    SJA1000_PORT_OUTPUT(DATA);                              // Set DATA port as output
  }
#endif
#ifdef SJA1000_USE_MOTOROLAMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_MOTOROLA_MODE)
# endif
  {
    //--- Send the address ---
    SJA1000_PIN_HIGH(ALE_AS);        __SJA1000_CHECK_ERROR; // Set ALE/AS pin high level
    SJA1000_PORT_WRITE(DATA, reg);   __SJA1000_CHECK_ERROR; // Set DATA port output with register address
    SJA1000_PIN_HIGH(WR);            __SJA1000_CHECK_ERROR; // Set WR pin high level
    SJA1000_PIN_LOW(ALE_AS);         __SJA1000_CHECK_ERROR; // Set ALE/AS pin low level

    //--- Read data ---
    SJA1000_PIN_LOW(CS);             __SJA1000_CHECK_ERROR; // Set CS pin low level
    SJA1000_PIN_HIGH(RD_E);          __SJA1000_CHECK_ERROR; // Set RD/E pin high level
    SJA1000_PORT_INPUT(DATA);        __SJA1000_CHECK_ERROR; // Set DATA port as input
    *data = SJA1000_PORT_READ(DATA); __SJA1000_CHECK_ERROR; // Get DATA port input with data
    SJA1000_PIN_LOW(RD_E);           __SJA1000_CHECK_ERROR; // Set RD/E pin low level
    SJA1000_PIN_HIGH(CS);            __SJA1000_CHECK_ERROR; // Set CS pin high level
    SJA1000_PORT_OUTPUT(DATA);                              // Set DATA port as output
  }
#endif
  return Error;
}


//=============================================================================
// Write data to register of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_WriteRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t data)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  GPIO_Interface* pCS     = GET_CS_INTERFACE;
  GPIO_Interface* pALE_AS = GET_ALE_AS_INTERFACE;
  GPIO_Interface* pRD_E   = GET_RD_E_INTERFACE;
  GPIO_Interface* pWR     = GET_WR_INTERFACE;
  PORT_Interface* pDATA   = GET_DATA_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if ((pCS == NULL) || (pALE_AS == NULL) || (pRD_E == NULL) || (pWR == NULL) || (pDATA == NULL)) return ERR__PARAMETER_ERROR;
# endif
  if ((pCS->fnGPIO_SetState == NULL) || (pALE_AS->fnGPIO_SetState == NULL) || (pRD_E->fnGPIO_SetState == NULL)
   || (pWR->fnGPIO_SetState == NULL) || (pDATA->fnPORT_SetOutputLevel == NULL)) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

#ifdef SJA1000_USE_INTELMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_INTEL_MODE)
# endif
  {
    //--- Send the address ---
    SJA1000_PIN_HIGH(ALE_AS);       __SJA1000_CHECK_ERROR; // Set ALE/AS pin high level
    SJA1000_PORT_WRITE(DATA, reg);  __SJA1000_CHECK_ERROR; // Set DATA port output with register address
    SJA1000_PIN_LOW(ALE_AS);        __SJA1000_CHECK_ERROR; // Set ALE/AS pin low level

    //--- Read data ---
    SJA1000_PIN_LOW(CS);            __SJA1000_CHECK_ERROR; // Set CS pin low level
    SJA1000_PIN_LOW(WR);            __SJA1000_CHECK_ERROR; // Set WR pin low level
    SJA1000_PORT_WRITE(DATA, data); __SJA1000_CHECK_ERROR; // Set DATA port output with data
    SJA1000_PIN_HIGH(WR);           __SJA1000_CHECK_ERROR; // Set WR pin high level
    SJA1000_PIN_HIGH(CS);                                  // Set CS pin high level
  }
#endif
#ifdef SJA1000_USE_MOTOROLAMODE
# ifdef SJA1000_BOTH_MODE_DEFINED
  if (pComp->CommMode == SJA1000_MOTOROLA_MODE)
# endif
  {
    //--- Send the address ---
    SJA1000_PIN_HIGH(ALE_AS);       __SJA1000_CHECK_ERROR; // Set ALE/AS pin high level
    SJA1000_PORT_WRITE(DATA, reg);  __SJA1000_CHECK_ERROR; // Set DATA port output with register address
    SJA1000_PIN_LOW(WR);            __SJA1000_CHECK_ERROR; // Set WR pin low level
    SJA1000_PIN_LOW(ALE_AS);        __SJA1000_CHECK_ERROR; // Set ALE/AS pin low level

    //--- Read data ---
    SJA1000_PIN_LOW(CS);            __SJA1000_CHECK_ERROR; // Set CS pin low level
    SJA1000_PIN_HIGH(RD_E);         __SJA1000_CHECK_ERROR; // Set RD/E pin high level
    SJA1000_PORT_WRITE(DATA, data); __SJA1000_CHECK_ERROR; // Set DATA port output with data
    SJA1000_PIN_LOW(RD_E);          __SJA1000_CHECK_ERROR; // Set RD/E pin low level
    SJA1000_PIN_HIGH(CS);                                  // Set CS pin high level
  }
#endif
  return Error;
}


//=============================================================================
// Modify a register of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ModifyRegister(SJA1000 *pComp, eSJA1000_Registers reg, uint8_t data, uint8_t mask)
{
  eERRORRESULT Error;
  uint8_t RegValue;
  Error = SJA1000_ReadRegister(pComp, reg, &RegValue); // Read the register value
  __SJA1000_CHECK_ERROR;
  RegValue &= ~mask;                                   // Clear bits to modify
  RegValue |= (data & mask);                           // Set the new value
  return SJA1000_WriteRegister(pComp, reg, RegValue);  // Write the value to register
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Transmit a message object (with data) to the Buffer of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_TransmitMessageObject(SJA1000 *pComp, const uint8_t* messageObjectToSend, uint8_t objectSize)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if (objectSize < 2) return ERR__BAD_DATA_SIZE;
  eERRORRESULT Error;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    //--- Write Transmit Message ---
    const uint8_t* const pMsgInf = messageObjectToSend;
    eSJA1000_Registers RegAddr = RegSJA1000_PCAN_TX_BUFFER_START;
    const size_t DataPayploadSize = SJA1000_DLC_TO_VALUE[SJA1000_PCAN_INF_DLC_GET(*pMsgInf)];
    const size_t SizeToWrite = ((*pMsgInf & SJA1000_PCAN_INF_EXTENDED_ID ) > 0 ? 5 : 3)
                             + ((*pMsgInf & SJA1000_PCAN_INF_REMOTE_FRAME) > 0 ? 0 : DataPayploadSize);
    if (SizeToWrite > objectSize) return ERR__BAD_DATA_SIZE; // Check size of the object
    for (size_t zIdx = 0; zIdx < SizeToWrite; ++zIdx)
    {
      Error = SJA1000_WriteRegister(pComp, RegAddr++, *messageObjectToSend++);
      __SJA1000_CHECK_ERROR;
    }

    //--- Transmit Request ---
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_PCAN_COMMAND, SJA1000_CMR_TRANSMISSION_REQUEST);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    //--- Write Transmit Message ---
    eSJA1000_Registers RegAddr = RegSJA1000_BCAN_TX_BUFFER_ID0;
    const uint8_t* const pMsgInf = (messageObjectToSend + 1);
    const size_t DataPayploadSize = SJA1000_DLC_TO_VALUE[SJA1000_BCAN_DES_DLC_GET(*pMsgInf)];
    const size_t SizeToWrite = ((*pMsgInf & SJA1000_BCAN_DES_REMOTE_FRAME) > 0 ? 2 : DataPayploadSize);
    if (SizeToWrite > objectSize) return ERR__BAD_DATA_SIZE; // Check size of the object
    for (size_t zIdx = 0; zIdx < SizeToWrite; ++zIdx)
    {
      Error = SJA1000_WriteRegister(pComp, RegAddr++, *messageObjectToSend++);
      __SJA1000_CHECK_ERROR;
    }

    //--- Transmit Request ---
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_COMMAND, SJA1000_COMMAND_TX_REQUEST);
  }
#endif
  return Error;
}


//=============================================================================
// Transmit a message to the Buffer of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_TransmitMessage(SJA1000 *pComp, CAN_CANMessage* const messageToSend)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageToSend == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if ((messageToSend->ControlFlags & CAN_CANFD_FRAME) > 0) return ERR__NOT_SUPPORTED; // CAN-FD frames not supported by this controller
  uint8_t Buffer[SJA1000_CAN_TX_MESSAGE_SIZE_MAX];
  SJA1000_CAN_TxMessage* Message = (SJA1000_CAN_TxMessage*)Buffer; // The first 5 bytes represent the SJA1000_CAN_TxMessage struct
  size_t BytesToSend = 0;

  //--- Fill message INF, DES, and ID ---
#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Message->Bytes[SJA1000_CAN_TXMSG_INF] = SJA1000_PCAN_INF_DLC_SET(messageToSend->DLC) | SJA1000_PCAN_INF_DATA_FRAME | SJA1000_PCAN_INF_STANDARD_ID;
    if ((messageToSend->ControlFlags & CAN_EXTENDED_MESSAGE_ID) > 0)       // Extended message
    {
      Message->Bytes[SJA1000_CAN_TXMSG_INF] |= SJA1000_PCAN_INF_EXTENDED_ID;
      Message->Bytes[SJA1000_CAN_TXMSG_ID1] = (uint8_t)((messageToSend->MessageID << 21) & 0xFF);
      Message->Bytes[SJA1000_CAN_TXMSG_ID2] = (uint8_t)((messageToSend->MessageID << 13) & 0xFF);
      Message->Bytes[SJA1000_CAN_TXMSG_ID3] = (uint8_t)((messageToSend->MessageID >>  5) & 0xFF);
      Message->Bytes[SJA1000_CAN_TXMSG_ID4] = (uint8_t)((messageToSend->MessageID <<  3) & 0xFF);
      BytesToSend = 5;
    }
    else // ((messageToSend->ControlFlags & CAN_EXTENDED_MESSAGE_ID) == 0) // Standard message
    {
      Message->Bytes[SJA1000_CAN_TXMSG_ID1] = (uint8_t)((messageToSend->MessageID >> 3) & 0xFF);
      Message->Bytes[SJA1000_CAN_TXMSG_ID2] = (uint8_t)((messageToSend->MessageID << 5) & 0xFF);
      BytesToSend = 3;
    }
    if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) > 0) Message->Bytes[SJA1000_CAN_TXMSG_INF] |= SJA1000_PCAN_INF_REMOTE_FRAME; // Flag the RTR
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    Message->Bytes[SJA1000_CAN_RXMSG_DES1] = (uint8_t)((messageToSend->MessageID >> 3) & 0xFF);
    Message->Bytes[SJA1000_CAN_RXMSG_DES2] = (uint8_t)((messageToSend->MessageID << 5) & 0xFF) | SJA1000_BCAN_DES_DLC_SET(messageToSend->DLC);
    if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) > 0) Message->Bytes[SJA1000_CAN_RXMSG_DES2] |= SJA1000_BCAN_DES_REMOTE_FRAME; // Flag the RTR
    BytesToSend = 2;
  }
#endif

  //--- Fill data ---
  if ((messageToSend->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) == 0) // If not RTR frame
  {
    if ((messageToSend->DLC != CAN_DLC_0BYTE) && (messageToSend->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
    size_t BytesToCopy = SJA1000_DLC_TO_VALUE[messageToSend->DLC];
    uint8_t* pBuff = &Message->Bytes[BytesToSend];   // Next bytes of the Buffer is for payload
    uint8_t* pData = &messageToSend->PayloadData[0]; // Select the first byte of payload data
    BytesToSend += BytesToCopy;
    while (BytesToCopy-- > 0) *pBuff++ = *pData++;   // Copy data
  }

  //--- Send data ---
  return SJA1000_TransmitMessageObject(pComp, &Buffer[0], BytesToSend);
}





//=============================================================================
// Receive a message object (with data) from the FIFO of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ReceiveMessageObject(SJA1000 *pComp, uint8_t* messageObjectGet, uint8_t objectSize)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageObjectGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  if (objectSize < 2) return ERR__BAD_DATA_SIZE;
  eERRORRESULT Error;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    //--- Read Received Message Information ---
    const uint8_t* const pMsgInf = messageObjectGet;
    eSJA1000_Registers RegAddr = RegSJA1000_PCAN_RX_BUFFER_START;
    Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++);
    __SJA1000_CHECK_ERROR;
    const size_t DataPayploadSize = SJA1000_DLC_TO_VALUE[SJA1000_PCAN_INF_DLC_GET(*pMsgInf)];
    const size_t SizeToRead = ((*pMsgInf & SJA1000_PCAN_INF_EXTENDED_ID ) > 0 ? 5 : 3)
                            + ((*pMsgInf & SJA1000_PCAN_INF_REMOTE_FRAME) > 0 ? 0 : DataPayploadSize);
    if (SizeToRead > objectSize) return ERR__BAD_DATA_SIZE; // Check size of the object

    //--- Read Received Message Identifier ---
    Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_ID1
    __SJA1000_CHECK_ERROR;
    Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_ID2
    __SJA1000_CHECK_ERROR;
    if ((*pMsgInf & SJA1000_PCAN_INF_EXTENDED_ID) > 0)
    {
      Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_ID3
      __SJA1000_CHECK_ERROR;
      Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_ID4
      __SJA1000_CHECK_ERROR;
    }

    //--- Read Received Message Data ---
    if ((*pMsgInf & SJA1000_PCAN_INF_REMOTE_FRAME) == 0) // Only if not a Remote Frame
    {
      for (size_t zIdx = 0; zIdx < DataPayploadSize; ++zIdx)
      {
        Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read payload bytes
        __SJA1000_CHECK_ERROR;
      }
    }

    //--- Release Receive Buffer ---
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_PCAN_COMMAND, SJA1000_CMR_RELEASE_RECEIVE_BUFFER);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    //--- Read Received Message Descriptor ---
    eSJA1000_Registers RegAddr = RegSJA1000_BCAN_RX_BUFFER_ID0;
    Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_DES1
    __SJA1000_CHECK_ERROR;
    const uint8_t* const pMsgInf = messageObjectGet; // Pre-declare pMsgInf pointer, at this point messageObjectGet points to the second byte (not yet filled) which is the one we want
    Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++); // Read SJA1000_CAN_RXMSG_DES2
    __SJA1000_CHECK_ERROR;
    const size_t DataPayploadSize = SJA1000_DLC_TO_VALUE[SJA1000_BCAN_DES_DLC_GET(*pMsgInf)];
    const size_t SizeToRead = ((*pMsgInf & SJA1000_BCAN_DES_REMOTE_FRAME) > 0 ? 2 : DataPayploadSize);
    if (SizeToRead > objectSize) return ERR__BAD_DATA_SIZE; // Check size of the object

    //--- Read Received Message Data ---
    if ((*pMsgInf & SJA1000_BCAN_DES_REMOTE_FRAME) == 0)    // Only if not a Remote Frame
    {
      for (size_t zIdx = 0; zIdx < DataPayploadSize; ++zIdx)
      {
        Error = SJA1000_ReadRegister(pComp, RegAddr++, messageObjectGet++);
        __SJA1000_CHECK_ERROR;
      }
    }

    //--- Release Receive Buffer ---
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_COMMAND, SJA1000_COMMAND_RELEASE_RX);
  }
#endif
  return Error;
}


//=============================================================================
// Receive a message from the FIFO of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ReceiveMessage(SJA1000 *pComp, CAN_CANMessage* const messageGet, eSJA1000_PayloadSize payloadSize)
{
#ifdef CHECK_NULL_PARAM
  if ((pComp == NULL) || (messageGet == NULL)) return ERR__PARAMETER_ERROR;
#endif
  uint8_t Buffer[SJA1000_CAN_RX_MESSAGE_SIZE_MAX];
  SJA1000_CAN_RxMessage* Message = (SJA1000_CAN_RxMessage*)Buffer; // The first 5 bytes represent the SJA1000_CAN_RxMessage struct
  eERRORRESULT Error;
  size_t BytesToGet = 0;

  //--- Get data ---
  Error = SJA1000_ReceiveMessageObject(pComp, &Buffer[0], sizeof(Buffer));
  __SJA1000_CHECK_ERROR;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    //--- Extract INF ---
    messageGet->ControlFlags = CAN_NO_MESSAGE_CTRL_FLAGS;
    messageGet->DLC = SJA1000_PCAN_INF_DLC_GET(Message->Bytes[SJA1000_CAN_RXMSG_INF]);
    if ((Message->Bytes[SJA1000_CAN_RXMSG_INF] & SJA1000_PCAN_INF_REMOTE_FRAME) > 0) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_REMOTE_TRANSMISSION_REQUEST); // RTR

    //--- Extract ID ---
    if ((Message->Bytes[SJA1000_CAN_RXMSG_INF] & CAN_EXTENDED_MESSAGE_ID) > 0) // Extended message
    {
      eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_EXTENDED_MESSAGE_ID); // IDE
      messageGet->MessageID  = (Message->Bytes[SJA1000_CAN_RXMSG_ID1] << 21);
      messageGet->MessageID |= (Message->Bytes[SJA1000_CAN_RXMSG_ID2] << 13);
      messageGet->MessageID |= (Message->Bytes[SJA1000_CAN_RXMSG_ID3] <<  5);
      messageGet->MessageID |= (Message->Bytes[SJA1000_CAN_RXMSG_ID4] >>  3);
      BytesToGet = 5;
    }
    else // ((Message->Bytes[SJA1000_CAN_RXMSG_INF] & CAN_EXTENDED_MESSAGE_ID) == 0) // Standard message
    {
      messageGet->MessageID  = (Message->Bytes[SJA1000_CAN_RXMSG_ID1] << 3);
      messageGet->MessageID |= (Message->Bytes[SJA1000_CAN_RXMSG_ID2] >> 5);
      BytesToGet = 3;
    }
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    //--- Extract DES ---
    messageGet->DLC = SJA1000_BCAN_DES_DLC_GET(Message->Bytes[SJA1000_CAN_RXMSG_DES2]);
    if ((Message->Bytes[SJA1000_CAN_RXMSG_DES2] & SJA1000_BCAN_DES_REMOTE_FRAME) > 0) eCAN_SET_CONTROL_FLAG(messageGet->ControlFlags, CAN_REMOTE_TRANSMISSION_REQUEST); // RTR
    messageGet->MessageID  = (Message->Bytes[SJA1000_CAN_RXMSG_DES1] << 3);
    messageGet->MessageID |= (Message->Bytes[SJA1000_CAN_RXMSG_DES2] >> 5);
    BytesToGet = 2;
  }
#endif

  //--- Fill data ---
  if ((messageGet->ControlFlags & CAN_REMOTE_TRANSMISSION_REQUEST) == 0) // If not RTR frame
  {
    if ((messageGet->DLC != CAN_DLC_0BYTE) && (messageGet->PayloadData == NULL)) return ERR__NO_DATA_AVAILABLE;
    if (messageGet->PayloadData != NULL)
    {
      uint8_t* pBuff = &Message->Bytes[BytesToGet];                         // Next bytes of the Buffer is for payload
      uint8_t* pData = &messageGet->PayloadData[0];                         // Select the first byte of payload data
      uint8_t BytesPayload = SJA1000_PAYLOAD_TO_VALUE[payloadSize];         // Get the payload sizein bytes
      uint8_t BytesDLC = SJA1000_DLC_TO_VALUE[messageGet->DLC];             // Get how many byte need to be extract from the message to correspond to its DLC
      if (BytesPayload < BytesDLC) BytesDLC = BytesPayload;                 // Get the least between BytesPayload and BytesDLC
      while (BytesDLC-- > 0) *pData++ = *pBuff++;                           // Copy data
    }
  }
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure pins of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ConfigurePins(SJA1000 *pComp, eSJA1000_OutMode outMode, eSJA1000_TXpinMode tx0PinMode, eSJA1000_TXpinMode tx1PinMode, eSJA1000_CLKOUTpinMode clkoutMode, bool tx1AsRxInt, bool bypassRxComp)
{
#ifdef CHECK_NULL_PARAM
  if (pComp == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;

  //--- Configure clock divider register ---
  uint8_t CDRconfig = SJA1000_CDR_CLKOUT_SET(clkoutMode) | SJA1000_CDR_TX1_AS_TX | SJA1000_CDR_BYPASS_CAN_INPUT_COMP_DIS | SJA1000_CDR_BASIC_CAN_MODE;
  if (tx1AsRxInt  ) CDRconfig |= SJA1000_CDR_TX1_AS_RX_INT;
  if (bypassRxComp) CDRconfig |= SJA1000_CDR_BYPASS_CAN_INPUT_COMP_EN;
#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
     CDRconfig |= SJA1000_CDR_PELI_CAN_MODE;
#endif
  Error = SJA1000_WriteRegister(pComp, RegSJA1000_CAN_CLOCK_DIVIDER, CDRconfig);
  __SJA1000_CHECK_ERROR;

  //--- Configure output control register ---
  uint8_t OCRconfig = SJA1000_OC_MODE_SET(outMode) | SJA1000_TX0_PIN_MODE_SET(tx0PinMode) | SJA1000_TX1_PIN_MODE_SET(tx1PinMode);
  return SJA1000_WriteRegister(pComp, RegSJA1000_CAN_OUTPUT_CONTROL, OCRconfig);
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#if defined(SJA1000_AUTOMATIC_BITRATE_CALCULUS) || defined(CAN_AUTOMATIC_BITRATE_CALCULUS)
//=============================================================================
// Calculate Bit Time for CAN2.0 configuration for the SJA1000 peripheral
//=============================================================================
eERRORRESULT SJA1000_CalculateBitTimeConfiguration(const uint32_t periphClk, const struct CAN_CAN20busConfig pBusConf, struct CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
   if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif
  //--- Check values ---
  if (periphClk > SJA1000_PERIPHERAL_CLK_MAX) return ERR__PARAMETER_ERROR;
  if (pBusConf.DesiredBitrate <= SJA1000_NOMBITRATE_MIN) return ERR__BAUDRATE_ERROR;
  if (pBusConf.DesiredBitrate >  SJA1000_NOMBITRATE_MAX) return ERR__BAUDRATE_ERROR;

  //--- Declaration ---
  CAN_BitTimeConfig LastBTconf; LastBTconf.Valid = false;
  const uint32_t SysClkDivided = periphClk / SJA1000_SYSCLOCK_DIV;
  bool SearchExactBR = true;

  //--- Step 1 ---
  const uint32_t tPropSeg = 2 * ((SJA1000_tBUS_CONV * pBusConf.BusMeters) + pBusConf.TransceiverDelay); // Formula is tPROP_SEG(ns) = 2x((5ns * BusMeters) + tTXDtRXD)

  do
  {
    uint32_t BRP = (SJA1000_NBRP_MAX + 1);                                                       // Max of possible BRP
    while (--BRP > 0)
    {
      pConf->Valid = false;
      //--- Step 2 ---
      if (BRP > SJA1000_NBRP_MAX) break;                                                         // Adjust  and check maximum BRP
      const uint32_t NTQ = (SysClkDivided / (BRP * (pBusConf.DesiredBitrate / 100)) + 50) / 100; // Get NTQ rounded
      if (NTQ == 0) continue;                                                                    // Avoid divide by 0
      const uint32_t ActualBitrate = (SysClkDivided / (NTQ * BRP));                              // Actual bitrate
      int32_t ErrorBitrate = ((int32_t)(ActualBitrate * 100) / (int32_t)(pBusConf.DesiredBitrate / 100)) - 10000; // Bitrate deviation
      if (SearchExactBR && (ErrorBitrate != 0)) continue;                                        // Check the baudrate exactness if ask
      if (ErrorBitrate < 0) ErrorBitrate = -ErrorBitrate;                                        // Get absolute of bitrate deviation
      if (ErrorBitrate > SJA1000_UINT_MAX_OSC_TOL) continue;                                     // Check the maximum allowable tolerance

      //--- Step 3 ---
      const uint32_t TQ = 1000000 / (SysClkDivided / (BRP * 1000));                              // Get time quantum for this BRP
      uint32_t PropSeg = (((tPropSeg * 100) / TQ) + 99) / 100;                                   // Formula is PROP_SEG(bits) = ROUND_UP(tPROP_SEG/TimeQuantum)
      if (PropSeg < SJA1000_NPRSEG_MIN) PropSeg = SJA1000_NPRSEG_MIN;                            // Correct minimum NPRSEG
      if (NTQ < (SJA1000_NSYNC + PropSeg + SJA1000_NTSEG1_MIN + SJA1000_NTSEG2_MIN)) continue;   // Need a minimum TQbits for this bus length, then do the next NBRP value
      if (NTQ > (SJA1000_NSYNC + PropSeg + SJA1000_NTSEG1_MAX + SJA1000_NTSEG2_MAX)) continue;   // Higher than the maximum TQbits for this bus length, then do the next NBRP value
      pConf->NBRP = BRP;                                                                         // ** Save the NBRP in the configuration **
      if (pConf->NBRP < SJA1000_NBRP_MIN) pConf->NBRP = SJA1000_NBRP_MIN;                        // Correct NBRP min
      if (pConf->NBRP > SJA1000_NBRP_MAX) pConf->NBRP = SJA1000_NBRP_MAX;                        // Correct NBRP max

      //--- Step 4 ---
      uint32_t TQbits = NTQ;                                                                     // TQbits = NTQ
      TQbits -= (SJA1000_NSYNC + PropSeg);                                                       // Get remaining TQbits after removing NSYNC and PROP_SEG bits
      if (((TQbits & 1) > 0) && (TQbits != 3)) { ++PropSeg; --TQbits; }                          // If TQbits is odd and different of 3 (which have a special threatment), add one to PROP_SEG and remove it from TQbits
      pConf->NPRSEG = PropSeg;                                                                   // ** Save the NPRSEG in the configuration **
      if (pConf->NPRSEG > SJA1000_NPRSEG_MAX) continue;                                          // If NPRSEG > maximum, impossible solution
      pConf->NTSEG1 = TQbits >> 1;                                                               // ** Save the NSEG1PH = TQbits / 2 in the configuration **
      if (pConf->NTSEG1 < SJA1000_NTSEG1_MIN) pConf->NTSEG1 = SJA1000_NTSEG1_MIN;                // Correct NTSEG1 min
      if (pConf->NTSEG1 > SJA1000_NTSEG1_MAX) pConf->NTSEG1 = SJA1000_NTSEG1_MAX;                // Correct NTSEG1 max
      pConf->NTSEG2 = (TQbits >> 1) + (TQbits == 3 ? 1 : 0);                                     // ** Save the NSEG2PH = TQbits / 2 in the configuration **
      if (pConf->NTSEG2 < SJA1000_NTSEG2_MIN) pConf->NTSEG2 = SJA1000_NTSEG2_MIN;                // Correct NSEG2PH min
      if (pConf->NTSEG2 > SJA1000_NTSEG2_MAX) pConf->NTSEG2 = SJA1000_NTSEG2_MAX;                // Correct NSEG2PH max

      //--- Step 5 ---
      pConf->NSJW = ((pConf->NTSEG1 < 4) ? pConf->NTSEG1 : 4);                                   // ** Save the NSJW = min(4, SEG1PH) in the configuration **
      if (pConf->NSJW < SJA1000_NSJW_MIN) pConf->NSJW = SJA1000_NSJW_MIN;                        // Correct NSJW min
      if (pConf->NSJW > SJA1000_NSJW_MAX) pConf->NSJW = SJA1000_NSJW_MAX;                        // Correct NSJW max

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
          uint32_t LastNTQ = (SJA1000_NSYNC + LastBTconf.NPRSEG + LastBTconf.NTSEG1 + LastBTconf.NTSEG2);  // Get last NTQ
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
  pConf->SAMPL      = (pBusConf.DesiredBitrate > CAN_SAE_CLASS_B_SPEED_MAX ? 1 : 3);             // ** Set the SAM in the configuration ** (1 sample if SAE class C, else 3)
  pConf->PS2mode    = CAN_PS2_BLT_PHSEG2;                                                        // ** Set the PhaseSeg2 mode in the configuration **
  pConf->EdgeFilter = false;                                                                     // ** Edge Filtering enabled, according to ISO 11898-1:2015 **
  pConf->CAN20only  = true;                                                                      // ** Set the CAN2.0 only configuration **

  eERRORRESULT Error = ERR_OK;
  if (pConf->Stats != NULL)
      Error = SJA1000_CalculateBitrateStatistics(periphClk, pConf);                              // If statistics are necessary, then calculate them
  return Error;
}


//=============================================================================
// Calculate Bitrate Statistics of a Bit Time configuration
//=============================================================================
eERRORRESULT SJA1000_CalculateBitrateStatistics(const uint32_t periphClk, CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
  if (pConf->Stats == NULL) return ERR__PARAMETER_ERROR;
#endif

  //--- Calculate bus length & Nominal Sample Point ---------
  const uint32_t NTQ = (((pConf->NBRP+1) * 1000000) / (periphClk / 1000));          // Nominal Time Quanta = 1/PERIPHCLK multiply by 1000000000 to get ns
  const uint32_t NPRSEG  = (pConf->NTSEG1+1) - (pConf->NTSEG2+1);                   // Here PHSEG2 (NTSEG2) should be equal to PHSEG1 so NPRSEG = NTSEG1 - NTSEG2
  pConf->Stats->MaxBusLength = (uint32_t)(((NTQ * NPRSEG) - (2 * SJA1000_tTXDtRXD_MAX)) / (2 * SJA1000_tBUS_CONV)); // Formula is (2x(tTXD–RXD + (5*BusLen))/NTQ = NPRSEG => BusLen = ((NTQ*NPRESG)-(2*tTXD))/(2*5) in meter
  const uint32_t NTQbits = (SJA1000_NSYNC + (pConf->NTSEG1+1) + (pConf->NTSEG2+1)); // NTQ per bits = NSYNC + NTSEG1 + NTSEG2
  uint32_t SamplePoint = ((SJA1000_NSYNC + (pConf->NTSEG1+1)) * 100) / NTQbits;     // Calculate actual nominal sample point
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
// Set Bit Time Configuration to the SJA1000 peripheral
//=============================================================================
eERRORRESULT SJA1000_SetBitTimeConfiguration(SJA1000 *pComp, const CAN_BitTimeConfig* const pConf)
{
#ifdef CHECK_NULL_PARAM
  if (pConf == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (pConf->Valid == false) return ERR__INVALID_DATA;
  eERRORRESULT Error;

  //--- Write Nominal Bit Time configuration ---
  SJA1000_BTR0_Register RegBRT0;
  RegBRT0.BTR0 = SJA1000_BUS_TIMING0_BRP_SET(pConf->NBRP) | SJA1000_BUS_TIMING0_SJW_SET(pConf->NSJW); // Set Nominal Bit Time configuration (BRP and SJW)
  Error = SJA1000_WriteRegister(pComp, RegSJA1000_CAN_BUS_TIMING_0, RegBRT0.BTR0);
  __SJA1000_CHECK_ERROR;
  SJA1000_BTR1_Register RegBRT1;
  RegBRT1.BTR1 = SJA1000_BUS_TIMING1_TSEG1_SET(pConf->NPRSEG + pConf->NTSEG1) | SJA1000_BUS_TIMING1_TSEG2_SET(pConf->NTSEG2); // Set Nominal Bit Time configuration (NTSEG1 and NTSEG2)
  if (pConf->SAMPL > 1) RegBRT1.BTR1 |= SJA1000_BUS_TIMING1_3_SAMPLES;                                // Set sampling
  return SJA1000_WriteRegister(pComp, RegSJA1000_CAN_BUS_TIMING_1, RegBRT1.BTR1);
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Get actual operation mode of the SJA1000 peripheral
//=============================================================================
eERRORRESULT SJA1000_GetActualOperationMode(SJA1000 *pComp, eSJA1000_OperationMode* const actualMode)
{
#ifdef CHECK_NULL_PARAM
  if (actualMode == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;
  *actualMode = SJA1000_MODE_NORMAL;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_MODE, &Config);
    *actualMode = SJA1000_MODE_OPERATION_GET(Config);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_BCAN_CONTROL, &Config);
    if ((Config & SJA1000_BCAN_CTRL_RESET_MODE) > 0) *actualMode = SJA1000_MODE_RESET;
  }
#endif
  return Error;
}


//=============================================================================
// Request operation mode change of the SJA1000 peripheral
//=============================================================================
eERRORRESULT SJA1000_RequestOperationMode(SJA1000 *pComp, eSJA1000_OperationMode newMode)
{
  eERRORRESULT Error;
  uint8_t Config;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Config = SJA1000_MODE_OPERATION_SET(newMode);
    Error = SJA1000_ModifyRegister(pComp, RegSJA1000_PCAN_MODE, Config, SJA1000_MODE_OPERATION_Mask);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    if ((newMode != SJA1000_MODE_NORMAL) && (newMode != SJA1000_MODE_RESET)) return ERR__CONFIGURATION;
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_BCAN_CONTROL, &Config);
    __SJA1000_CHECK_ERROR;
    Config &= ~SJA1000_BCAN_CTRL_RESET_MODE; // Clear configuration
    if (newMode == SJA1000_MODE_RESET) Config |= SJA1000_BCAN_CTRL_RESET_MODE;
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_CONTROL, Config);
  }
#endif
  return Error;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure the filters of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ConfigureFilters(SJA1000 *pComp, SJA1000_Filter* const confFilter1, SJA1000_Filter* const confFilter2)
{
#ifdef CHECK_NULL_PARAM
  if (confFilter1 == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    //--- Set acceptance filter mode ---
    Config = SJA1000_MODE_OPERATION_SET(SJA1000_MODE_RESET) | ((confFilter2 == NULL ? SJA1000_SINGLE_ACCEPTANCE_FILTER : SJA1000_DUAL_ACCEPTANCE_FILTER));
    Error = SJA1000_ModifyRegister(pComp, RegSJA1000_PCAN_MODE, Config, SJA1000_ACCEPTANCE_FILTER_Mask | SJA1000_MODE_OPERATION_Mask);
    __SJA1000_CHECK_ERROR;

    //--- Fill filter data ---
    uint8_t FilterBytes[4];
    uint8_t MaskBytes[4];
    eSJA1000_Registers RegAddr;
    if (confFilter2 == NULL) // Single filter mode (filter configuration one long filter (4-bytes))
    {
      if (confFilter1->ExtendedID) // Extended message ID
      {
        FilterBytes[0] = (uint8_t)((confFilter1->AcceptanceID >> 21) & 0xFF);
        FilterBytes[1] = (uint8_t)((confFilter1->AcceptanceID >> 13) & 0xFF);
        FilterBytes[2] = (uint8_t)((confFilter1->AcceptanceID >>  5) & 0xFF);
        FilterBytes[3] = (uint8_t)((confFilter1->AcceptanceID <<  3) & 0xFF) | 0b11;
        MaskBytes[0] = (uint8_t)((~confFilter1->AcceptanceIDmask >> 21) & 0xFF);
        MaskBytes[1] = (uint8_t)((~confFilter1->AcceptanceIDmask >> 13) & 0xFF);
        MaskBytes[2] = (uint8_t)((~confFilter1->AcceptanceIDmask >>  5) & 0xFF);
        MaskBytes[3] = (uint8_t)((~confFilter1->AcceptanceIDmask <<  3) & 0xFF) | 0b11;
        if (confFilter1->FilterRTR)
        {
          MaskBytes[3] |= 0b100;                                     // Set mask filter RTR bit
          if (confFilter1->RTRbitValue > 0) FilterBytes[3] |= 0b100; // Filter set RTR bit value
        }
      }
      else // Standard message ID
      {
        FilterBytes[0] = (uint8_t)((confFilter1->AcceptanceID   >> 3) & 0xFF);
        FilterBytes[1] = (uint8_t)((confFilter1->AcceptanceID   << 5) & 0xFF) | 0b1111;
        FilterBytes[2] = (uint8_t)((confFilter1->AcceptanceData >> 8) & 0xFF);
        FilterBytes[3] = (uint8_t)((confFilter1->AcceptanceData >> 0) & 0xFF);
        MaskBytes[0] = (uint8_t)((~confFilter1->AcceptanceIDmask   >> 3) & 0xFF);
        MaskBytes[1] = (uint8_t)((~confFilter1->AcceptanceIDmask   << 5) & 0xFF) | 0b1111;
        MaskBytes[2] = (uint8_t)((~confFilter1->AcceptanceDataMask >> 8) & 0xFF);
        MaskBytes[3] = (uint8_t)((~confFilter1->AcceptanceDataMask << 0) & 0xFF);
        if (confFilter1->FilterRTR)
        {
          MaskBytes[3] |= 0b10000;                                     // Set mask filter RTR bit
          if (confFilter1->RTRbitValue > 0) FilterBytes[3] |= 0b10000; // Filter set RTR bit value
        }
      }
    }
    else // Dual filter mode (filter configuration two short filters (2 x 2-bytes))
    {
      if (confFilter1->ExtendedID) // Extended message ID
      {
        //--- Filter 1 ---
        FilterBytes[0] = (uint8_t)((confFilter1->AcceptanceID >> 21) & 0xFF);
        FilterBytes[1] = (uint8_t)((confFilter1->AcceptanceID >> 13) & 0xFF);
        MaskBytes[0] = (uint8_t)((~confFilter1->AcceptanceIDmask >> 21) & 0xFF);
        MaskBytes[1] = (uint8_t)((~confFilter1->AcceptanceIDmask >> 13) & 0xFF);
        //--- Filter 2 ---
        FilterBytes[2] = (uint8_t)((confFilter2->AcceptanceID >> 21) & 0xFF);
        FilterBytes[3] = (uint8_t)((confFilter2->AcceptanceID >> 13) & 0xFF);
        MaskBytes[2] = (uint8_t)((~confFilter2->AcceptanceIDmask >> 21) & 0xFF);
        MaskBytes[3] = (uint8_t)((~confFilter2->AcceptanceIDmask >> 13) & 0xFF);
      }
      else
      {
        //--- Filter 1 ---
        FilterBytes[0] = (uint8_t)((confFilter1->AcceptanceID >> 3) & 0xFF);
        FilterBytes[1] = (uint8_t)((confFilter1->AcceptanceID << 5) & 0xFF) | ((confFilter1->AcceptanceData >> 4) & 0x0F);
        MaskBytes[0] = (uint8_t)((~confFilter1->AcceptanceIDmask >> 3) & 0xFF);
        MaskBytes[1] = (uint8_t)((~confFilter1->AcceptanceIDmask << 5) & 0xFF) | ((~confFilter1->AcceptanceDataMask >> 4) & 0x0F);
        if (confFilter1->FilterRTR)
        {
          MaskBytes[1] |= 0b10000;                                     // Set mask filter 1 RTR bit
          if (confFilter1->RTRbitValue > 0) FilterBytes[1] |= 0b10000; // Filter 1 set RTR bit value
        }
        //--- Filter 2 ---
        FilterBytes[2] = (uint8_t)((confFilter2->AcceptanceID >> 3) & 0xFF);
        FilterBytes[3] = (uint8_t)((confFilter2->AcceptanceID >> 5) & 0xFF) | ((confFilter1->AcceptanceData >> 0) & 0x0F);
        MaskBytes[2] = (uint8_t)((~confFilter2->AcceptanceIDmask >> 3) & 0xFF);
        MaskBytes[3] = (uint8_t)((~confFilter2->AcceptanceIDmask << 5) & 0xFF) | ((~confFilter1->AcceptanceDataMask >> 0) & 0x0F);
        if (confFilter2->FilterRTR)
        {
          MaskBytes[3] |= 0b10000;                                     // Set mask filter 2 RTR bit
          if (confFilter2->RTRbitValue > 0) FilterBytes[3] |= 0b10000; // Filter 2 set RTR bit value
        }
      }
    }

    //--- Send Acceptance ---
    RegAddr = RegSJA1000_PCAN_ACCEPTANCE_CODE0;
    for (size_t zIdx = 0; zIdx < sizeof(FilterBytes); ++RegAddr, ++zIdx)
    {
      Error = SJA1000_WriteRegister(pComp, RegAddr, FilterBytes[zIdx]);
      __SJA1000_CHECK_ERROR;
    }

    //--- Send Mask ---
    RegAddr = RegSJA1000_PCAN_ACCEPTANCE_MASK0;
    for (size_t zIdx = 0; zIdx < sizeof(MaskBytes); ++RegAddr, ++zIdx)
    {
      Error = SJA1000_WriteRegister(pComp, RegAddr, MaskBytes[zIdx]);
      __SJA1000_CHECK_ERROR;
    }
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
# if !defined(SJA1000_BOTH_CAN_DEFINED)
    if (confFilter2 != NULL) return ERR__NOT_SUPPORTED;
# endif
    Error = WriteRegister(pComp, RegSJA1000_BCAN_ACCEPTANCE_CODE, (uint8_t)(confFilter1->AcceptanceID     >> 3));
    __SJA1000_CHECK_ERROR;
    Error = WriteRegister(pComp, RegSJA1000_BCAN_ACCEPTANCE_MASK, (uint8_t)(confFilter1->AcceptanceIDmask >> 3));
  }
#endif
  return Error;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Enter the SJA1000 device in sleep mode
//=============================================================================
eERRORRESULT SJA1000_EnterSleepMode(SJA1000 *pComp)
{
  eERRORRESULT Error;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Error = SJA1000_RequestOperationMode(pComp, SJA1000_MODE_SLEEP);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_COMMAND, SJA1000_COMMAND_GO_TO_SLEEP);
  }
#endif
  return Error;
}


//=============================================================================
// Verify if the SJA1000 device is in sleep mode
//=============================================================================
eERRORRESULT SJA1000_IsDeviceInSleepMode(SJA1000 *pComp, bool* const isInSleepMode)
{
  eERRORRESULT Error;
  *isInSleepMode = false;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    eSJA1000_OperationMode ActualMode;
    Error = SJA1000_GetActualOperationMode(pComp, &ActualMode);
    *isInSleepMode = (ActualMode == SJA1000_MODE_SLEEP);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    uint8_t Config;
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_BCAN_COMMAND, &Config);
    *isInSleepMode = ((Config & SJA1000_COMMAND_GO_TO_SLEEP) > 0);
  }
#endif
  return Error;
}


//=============================================================================
// Manually wake up the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_WakeUp(SJA1000 *pComp)
{
  eERRORRESULT Error;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Error = SJA1000_RequestOperationMode(pComp, SJA1000_MODE_NORMAL);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_COMMAND, SJA1000_COMMAND_WAKE_UP);
  }
#endif
  return Error;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Configure interrupt of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ConfigureInterrupt(SJA1000 *pComp, setSJA1000_Interrupts interruptsFlags)
{
  eERRORRESULT Error;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_PCAN_INTERRUPT_ENABLE, (uint8_t)(interruptsFlags & SJA1000_PCAN_INTERRUPTS_FLAGS_MASK));
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    uint8_t Config;
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_BCAN_CONTROL, &Config);
    __SJA1000_CHECK_ERROR;
    interruptsFlags = ((interruptsFlags << 1) & SJA1000_BCAN_INTERRUPTS_FLAGS_MASK); // Interrupt flags on BasicCAN are shift by one on the control register
    Config &= ~SJA1000_BCAN_INTERRUPTS_FLAGS_MASK;
    Error = SJA1000_WriteRegister(pComp, RegSJA1000_BCAN_CONTROL, (Config | (uint8_t)interruptsFlags));
  }
#endif
  return Error;
}


//=============================================================================
// Get interrupt events of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_GetInterruptEvents(SJA1000 *pComp, setSJA1000_InterruptEvents* const interruptsFlags)
{
#ifdef CHECK_NULL_PARAM
  if (interruptsFlags == NULL) return ERR__PARAMETER_ERROR;
#endif
  eERRORRESULT Error;
  uint8_t Config;

#ifdef SJA1000_USE_PELICAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_PeliCAN)
# endif
  {
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_INTERRUPT_STATUS, &Config);
    *interruptsFlags = ((setSJA1000_InterruptEvents)Config & SJA1000_PCAN_INT_EVENTS_FLAGS_MASK);
  }
#endif
#ifdef SJA1000_USE_BASICCAN
# ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN)
# endif
  {
    Error = SJA1000_ReadRegister(pComp, RegSJA1000_BCAN_INTERRUPT, &Config);
    *interruptsFlags = ((setSJA1000_InterruptEvents)Config & SJA1000_BCAN_INT_EVENTS_FLAGS_MASK);
  }
#endif
  return Error;
}


//=============================================================================
// Clear interrupt events of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_ClearInterruptEvents(SJA1000 *pComp, setSJA1000_InterruptEvents interruptsFlags)
{ // Nothing to do here, events are cleared by reading interrupt events (ie. #SJA1000_GetInterruptEvents())
  (void)pComp;
  (void)interruptsFlags;
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//=============================================================================
// Get the last error capture of the SJA1000 device
//=============================================================================
eERRORRESULT SJA1000_GetLastErrorCapture(SJA1000 *pComp, eSJA1000_ErrorSegment* const errorSegment, eSJA1000_ErrorDirection* const errorDirection, eSJA1000_ErrorCode* const errorCode)
{
#ifdef CHECK_NULL_PARAM
  if ((errorSegment == NULL) || (errorDirection == NULL) || (errorCode == NULL)) return ERR__PARAMETER_ERROR;
#endif
#if defined(SJA1000_USE_BASICCAN) && !defined(SJA1000_BOTH_CAN_DEFINED)
  return ERR__NOT_SUPPORTED;
#endif
#ifdef SJA1000_BOTH_CAN_DEFINED
  if (pComp->CANtype == SJA1000_BasicCAN) return ERR__NOT_SUPPORTED;
#endif
  eERRORRESULT Error;
  uint8_t Config;

  Error = SJA1000_ReadRegister(pComp, RegSJA1000_PCAN_ERROR_CODE, &Config);
  *errorSegment   = SJA1000_ERROR_SEGMENT_GET(Config);
  *errorDirection = SJA1000_ERROR_DIRECTION_GET(Config);
  *errorCode      = SJA1000_ERROR_CODE_GET(Config);
  return Error;
}

//-----------------------------------------------------------------------------
#ifdef _cplusplus
}
#endif
//-----------------------------------------------------------------------------