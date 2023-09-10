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
  Mcan* pMCAN = (Mcan*)pComp->Instance;
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
  MATRIX->CCFG_CAN0  = CCFG_CAN0_CAN0DMABA_SET((uint32_t)pComp->RAMallocation);
  else MATRIX->CCFG_SYSIO = CCFG_SYSIO_CAN1DMABA_SET((uint32_t)pComp->RAMallocation);
  return ERR_NONE;
}

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 MCAN0 peripheral
//********************************************************************************************************************

#define MCAN0_CAN_BUFFER_SIZE  ( sizeof(uint32_t) * 4352u ) //!< 32-bits word * 4352 words max for MCAN0 is 17408 bytes
uint8_t MCAN0_RAMbuffer[MCAN0_CAN_BUFFER_SIZE];             //!< RAM buffer for the MCAN0

//! Peripheral structure of the MCAN0 on SAMV71
struct MCANV71 MCAN0V71 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCAN_DRIVER_NORMAL_USE,
  .InternalConfig = 0,
  //--- MCAN peripheral ---
  .Instance       = MCAN0,
  .RAMallocation  = &MCAN0_RAMbuffer[0],
  .RAMsize        = MCAN0_CAN_BUFFER_SIZE,
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCAN0V71_BitTimeStats = { 0 }; //!< MCAN0V71 Bit Time stat
uint32_t MCAN0V71_SYSCLK; //!< SYSCLK frequency will be stored here after using #Init_MCANV71()

//! Configuration structure of the MCAN0 on SAMV71
struct MCANV71_Config MCAN0V71_Config =
{
  //--- Controller clocks ---
  .MainFreq               = 0, //!< Use peripheral frequency called by #MCANV71_ConfigurePeripheralClocks()
  .MessageRAMwatchdogConf = 0,
  //--- RAM configuration ---
  .SIDelementsCount       = 10,
  .EIDelementsCount       = 10,
  //--- CAN configuration ---
  .ExtendedIDrangeMask    = 0x00000000,
  .RejectAllStandardIDs   = false,
  .RejectAllExtendedIDs   = false,
  .NonMatchingStandardID  = MCAN_ACCEPT_IN_RX_FIFO_0,
  .NonMatchingExtendedID  = MCAN_ACCEPT_IN_RX_FIFO_1,
  .BusConfig =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,                    // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay      = 330,                  // The transceiver is a TLE9255WSKXUMA1 on the CAN_Shield_V71 board. The worst delay is from Normal mode, Propagation delay, increased load, TxD to RxD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats          = &MCAN0V71_BitTimeStats,
  .ControlFlags          = MCAN_CAN_AUTOMATIC_RETRANSMISSION_ENABLE
                         | MCAN_CANFD_BITRATE_SWITCHING_ENABLE
                         | MCAN_CANFD_USE_ISO_CRC
                         | MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT
                         | MCAN_CAN_INTERNAL_TIMESTAMPING,
  //--- GPIOs and Interrupts pins ---
  .EnableLineINT0        = true,
  .EnableLineINT1        = true,
  //--- Interrupts ---
  .SysInterruptFlags     = MCAN_INT_ENABLE_ALL_EVENTS,
  .SysIntLineSelect      = MCAN_INT_ALL_ON_INT0 // All but...
                         | MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT1
                         | MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT1
                         | MCAN_INT_TIMEOUT_OCCURED_ON_INT1
                         | MCAN_INT_ERROR_CORRECTED_ON_INT1
                         | MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT1
                         | MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT1
                         | MCAN_INT_ERROR_PASSIVE_ON_INT1
                         | MCAN_INT_WARNING_STATUS_ON_INT1
                         | MCAN_INT_BUS_OFF_STATUS_ON_INT1
                         | MCAN_INT_WATCHDOG_ON_INT1
                         | MCAN_INT_ARBITRATION_ERROR_ON_INT1
                         | MCAN_INT_DATA_PHASE_ERROR_ON_INT1
                         | MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT1,
  .BufferTransmitInt     = 0xFFFFFFFF,
  .BufferCancelFinishInt = 0xFFFFFFFF,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 MCAN1 peripheral
//********************************************************************************************************************

#define MCAN1_CAN_BUFFER_SIZE  ( sizeof(uint32_t) * 4352u ) //!< 32-bits word * 4352 words max for MCAN1 is 17408 bytes
uint8_t MCAN1_RAMbuffer[MCAN1_CAN_BUFFER_SIZE];             //!< RAM buffer for the MCAN1

//! Peripheral structure of the MCAN1 on SAMV71
struct MCANV71 MCAN1V71 =
{
  .UserDriverData = NULL,
  //--- Driver configuration ---
  .DriverConfig   = MCAN_DRIVER_NORMAL_USE,
  .InternalConfig = 0,
  //--- MCAN peripheral ---
  .Instance       = MCAN1,
  .RAMallocation  = &MCAN1_RAMbuffer[0],
  .RAMsize        = MCAN1_CAN_BUFFER_SIZE,
  //--- Time call function ---
  .fnGetCurrentms = GetCurrentms_V71,
};

//-----------------------------------------------------------------------------

CAN_BitTimeStats MCAN1V71_BitTimeStats = { 0 }; //!< MCAN1V71 Bit Time stat
uint32_t MCAN1V71_SYSCLK; //!< SYSCLK frequency will be stored here after using #Init_MCANV71()

//! Configuration structure of the MCAN1 on SAMV71
struct MCANV71_Config MCAN1V71_Config =
{
  //--- Controller clocks ---
  .MainFreq               = 0, //!< Use peripheral frequency called by #MCANV71_ConfigurePeripheralClocks()
  .MessageRAMwatchdogConf = 0,
  //--- RAM configuration ---
  .SIDelementsCount       = 10,
  .EIDelementsCount       = 10,
  //--- CAN configuration ---
  .ExtendedIDrangeMask    = 0x00000000,
  .RejectAllStandardIDs   = false,
  .RejectAllExtendedIDs   = false,
  .NonMatchingStandardID  = MCAN_ACCEPT_IN_RX_FIFO_0,
  .NonMatchingExtendedID  = MCAN_ACCEPT_IN_RX_FIFO_1,
  .BusConfig =
  {
    .DesiredNominalBitrate = CAN_SHIELD_BITRATE,   // Desired CAN2.0A/CAN2.0B bitrate in bit/s
    .DesiredDataBitrate    = CANFD_SHIELD_BITRATE, // Desired Data CANFD bitrate in bit/s
    .BusMeters             = 1,                    // Only 10cm on the V71_UltraXplained_CAN_Shield
    .TransceiverDelay      = 330,                  // The transceiver is a TLE9255WSKXUMA1 on the CAN_Shield_V71 board. The worst delay is from Normal mode, Propagation delay, increased load, TxD to RxD
    .NominalSamplePoint    = 75,                   // Nominal sample point in percent
    .DataSamplePoint       = 75,                   // Data sample point in percent
  },
  .BitTimeStats          = &MCAN1V71_BitTimeStats,
  .ControlFlags          = MCAN_CAN_AUTOMATIC_RETRANSMISSION_ENABLE
                         | MCAN_CANFD_BITRATE_SWITCHING_ENABLE
                         | MCAN_CANFD_USE_ISO_CRC
                         | MCAN_CAN_WIDE_MESSAGE_MARKER_16BIT
                         | MCAN_CAN_INTERNAL_TIMESTAMPING,
  //--- GPIOs and Interrupts pins ---
  .EnableLineINT0        = true,
  .EnableLineINT1        = true,
  //--- Interrupts ---
  .SysInterruptFlags     = MCAN_INT_ENABLE_ALL_EVENTS,
  .SysIntLineSelect      = MCAN_INT_ALL_ON_INT0 // All but...
                         | MCAN_INT_TIMESTAMP_WRAPAROUND_ON_INT1
                         | MCAN_INT_MESSAGE_RAM_ACCESS_ON_INT1
                         | MCAN_INT_TIMEOUT_OCCURED_ON_INT1
                         | MCAN_INT_ERROR_CORRECTED_ON_INT1
                         | MCAN_INT_ERROR_UNCORRECTED_INTERRUPT_ON_INT1
                         | MCAN_INT_ERROR_LOGGING_OVERFLOW_ON_INT1
                         | MCAN_INT_ERROR_PASSIVE_ON_INT1
                         | MCAN_INT_WARNING_STATUS_ON_INT1
                         | MCAN_INT_BUS_OFF_STATUS_ON_INT1
                         | MCAN_INT_WATCHDOG_ON_INT1
                         | MCAN_INT_ARBITRATION_ERROR_ON_INT1
                         | MCAN_INT_DATA_PHASE_ERROR_ON_INT1
                         | MCAN_INT_ACCESS_RESERVED_ADDRESS_ON_INT1,
  .BufferTransmitInt     = 0xFFFFFFFF,
  .BufferCancelFinishInt = 0xFFFFFFFF,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 I2C peripheral
//********************************************************************************************************************

//! Peripheral structure of the hard I2C0 on the V71
struct I2C_Interface I2C0_V71 =
{
  .InterfaceDevice = TWIHS0,
  .UniqueID        = TWIHS_UNIQUE_ID,
  .fnI2C_Init      = TWIHS_MasterInit_Gen,
  .fnI2C_Transfer  = TWIHS_PacketTransfer_Gen,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// SAMV71 SPI peripheral
//********************************************************************************************************************

//! Peripheral structure of the hard SPI0 on the V71
struct SPI_Interface SPI0_V71 =
{
  .InterfaceDevice = SPI0,
  .UniqueID        = SPI_UNIQUE_ID,
  .fnSPI_Init      = SPI_MasterInit_Gen,
  .fnSPI_Transfer  = SPI_PacketTransfer_Gen,
};

//-----------------------------------------------------------------------------





//********************************************************************************************************************
// AT24MAC402 Component
//********************************************************************************************************************

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





//********************************************************************************************************************
// SAN V71 GPIO Ports
//********************************************************************************************************************

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





//**********************************************************************************************************************************************************
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