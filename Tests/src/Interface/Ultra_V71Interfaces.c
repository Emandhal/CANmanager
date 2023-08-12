/*!*****************************************************************************
 * @file    Ultra_V71Interfaces.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    01/07/2023
 * @brief   V71 Xplained Ultra board interfaces
*******************************************************************************/

//-----------------------------------------------------------------------------
#include "Ultra_V71Interfaces.h"
#include "TWIHS_V71.h"
#include "SPI_V71.h"
#include "Main.h"
//-----------------------------------------------------------------------------
#if !defined(__cplusplus)
#  include <asf.h>
#else
#  include <cstdint>
extern "C" {
#endif
//-----------------------------------------------------------------------------



//**********************************************************************************************************************************************************

//! Peripheral structure of the hard I2C0 on the V71
struct I2C_Interface I2C0_V71 =
{
  .InterfaceDevice = TWIHS0,
  .UniqueID        = TWIHS_UNIQUE_ID,
  .fnI2C_Init      = TWIHS_MasterInit_Gen,
  .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
};


//! Peripheral structure of the hard SPI0 on the V71
struct SPI_Interface SPI0_V71 =
{
  .InterfaceDevice = SPI0,
  .UniqueID        = SPI_UNIQUE_ID,
  .fnSPI_Init      = SPI_MasterInit_Gen,
  .fnSPI_Transfer  = SPI_PacketTransfer_Gen,
};

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************

//! Component structure of the AT24MAC402 with hard I2C on the V71
struct AT24MAC402 AT24MAC402_V71 =
{
  .Eeprom =
  {
    .UserDriverData = NULL,
    //--- EEPROM configuration ---
    .Conf           = &AT24MAC402_Conf,
    //--- Interface driver call functions ---
    .I2C            = &I2C0_V71,
    .I2CclockSpeed  = 400000, // I2C speed at 400kHz
    //--- Time call function ---
    .fnGetCurrentms = GetCurrentms_V71,
    //--- Interface clocks ---
    .AddrA2A1A0     = AT24MAC402_ADDR(1, 1, 1),
  },
};

//-----------------------------------------------------------------------------

//! IOPORTA interface of the V71
PORT_Interface IOPORTA =
{
  .InterfaceDevice       = PIOA,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTB interface of the V71
PORT_Interface IOPORTB =
{
  .InterfaceDevice       = PIOB,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTC interface of the V71
PORT_Interface IOPORTC =
{
  .InterfaceDevice       = PIOC,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTD interface of the V71
PORT_Interface IOPORTD =
{
  .InterfaceDevice       = PIOD,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//! IOPORTE interface of the V71
PORT_Interface IOPORTE =
{
  .InterfaceDevice       = PIOE,
  .UniqueID              = 0,
  .fnPORT_SetDirection   = PORTSetDirection_V71,
  .fnPORT_GetInputLevel  = PORTGetInputLevel_V71,
  .fnPORT_SetOutputLevel = PORTSetOutputLevel_V71,
  .PORTindex             = 0,
};

//-----------------------------------------------------------------------------



//=============================================================================
// Get millisecond
//=============================================================================
uint32_t GetCurrentms_V71(void)
{
  return msCount;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// PORTs interfaces of V71
//********************************************************************************************************************

//=============================================================================
// PORT set pins direction for V71
//=============================================================================
eERRORRESULT PORTSetDirection_V71(PORT_Interface *pIntDev, const uint32_t pinsDirection)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  
  
  return ERR_NONE;
}


//=============================================================================
// PORT pins input level for V71
//=============================================================================
eERRORRESULT PORTGetInputLevel_V71(PORT_Interface *pIntDev, uint32_t *pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  
  
  return ERR_NONE;
}


//=============================================================================
// PORT pins output level for V71
//=============================================================================
eERRORRESULT PORTSetOutputLevel_V71(PORT_Interface *pIntDev, const uint32_t pinsLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  
  
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
//********************************************************************************************************************
// GPIO Interfaces of V71
//********************************************************************************************************************

//=============================================================================
// GPIO set direction for V71
//=============================================================================
eERRORRESULT GPIOSetState_V71(GPIO_Interface *pIntDev, const eGPIO_State pinState)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  
  
  return ERR_NONE;
}


//=============================================================================
// GPIO pin input level for V71
//=============================================================================
eERRORRESULT GPIOGetInputLevel_V71(GPIO_Interface *pIntDev, eGPIO_State *pinLevel)
{
#ifdef CHECK_NULL_PARAM
  if (pIntDev == NULL) return ERR__PARAMETER_ERROR;
#endif
  
  
  return ERR_NONE;
}


//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------