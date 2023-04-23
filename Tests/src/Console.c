/*!*****************************************************************************
 * @file    Console.c
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    08/12/2017
 * @brief   Some functions for RS-232 console communication
 ******************************************************************************/

//-----------------------------------------------------------------------------
#include "Console.h"
#include "Main.h"
#include "string.h"
//-----------------------------------------------------------------------------
#ifdef __cplusplus
#  include "stdafx.h"
   extern "C" {
#endif
//-----------------------------------------------------------------------------

#ifdef USE_DYNAMIC_INTERFACE
#  define GET_UART_INTERFACE  pApi->UART
#else
#  define GET_UART_INTERFACE  &pApi->UART
#endif

//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#ifdef USE_CONSOLE_TX
//********************************************************************************************************************
// Console Transmit API
//********************************************************************************************************************
//=============================================================================
// Initialize the Console Transmit
//=============================================================================
eERRORRESULT InitConsoleTx(ConsoleTx* pApi)
{
  if (pApi == NULL) return ERR__PARAMETER_ERROR;

  // Initialize the print buffer
  if (pApi->Buffer == NULL) return ERR__NULL_BUFFER;
  if (pApi->BufferSize == 0) return ERR__BAD_DATA_SIZE;
  pApi->InPos  = 0;
  pApi->OutPos = 0;
  memset(pApi->Buffer, 0, (sizeof(pApi->Buffer[0]) * pApi->BufferSize));
  return ERR_OK;
}

//-----------------------------------------------------------------------------


//=============================================================================
// Set char to print buffer
//=============================================================================
void SetCharToConsoleBuffer(ConsoleTx* pApi, const char aChar)
{
  if (pApi == NULL) return;
  if (pApi->Buffer == NULL) return;
  if (aChar == '\0') return;

  bool BufferFull = false;
  // Check buffer full
  if (pApi->OutPos == 0) BufferFull = (pApi->InPos == (pApi->BufferSize - 1));
  else BufferFull = (pApi->InPos == (pApi->OutPos - 1));
  if (BufferFull)
    while (!TrySendingNextCharToConsole(pApi)) // If buffer full force sending next char
      if (pApi->InPos == pApi->OutPos) break;  // But both InPos and OutPos should not be at the same position
  // Store the new char
  pApi->Buffer[pApi->InPos++] = aChar;
  if (pApi->InPos >= pApi->BufferSize) pApi->InPos = 0;
}


//=============================================================================
// Set char array to print buffer
//=============================================================================
void SetStrToConsoleBuffer(ConsoleTx* pApi, const char* string)
{
  while (*string != '\0')
  {
    SetCharToConsoleBuffer(pApi, *string);
    string++;
  }
}


//=============================================================================
// Try to send next char in the console print buffer
//=============================================================================
bool TrySendingNextCharToConsole(ConsoleTx* pApi)
{
#ifdef CHECK_NULL_PARAM
  if (pApi == NULL) return false;
  if (pApi->Buffer == NULL) return false;
#endif
  UART_Interface* pUART = GET_UART_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pUART == NULL) return false;
# endif
  if (pUART->fnUART_Transmit == NULL) return false;
#endif
  bool Result = false;
  size_t DataSizeToSend;

  if (pApi->OutPos != pApi->InPos)
  {
    size_t ActuallySent = 0;
    if (pApi->OutPos >= pApi->InPos)
         DataSizeToSend = pApi->BufferSize - pApi->InPos; // Calculate space available to the end of buffer
    else DataSizeToSend = pApi->InPos - pApi->OutPos;     // Calculate space available to Out position
    if (pUART->fnUART_Transmit(pUART, (uint8_t*)&pApi->Buffer[pApi->OutPos], DataSizeToSend, &ActuallySent) == ERR_OK)
    {
      pApi->Buffer[pApi->OutPos] = 0;
      pApi->OutPos += ActuallySent;
      if (pApi->OutPos >= pApi->BufferSize) pApi->OutPos -= pApi->BufferSize;
      Result = true;
    }
  }
  return Result;
}

//-----------------------------------------------------------------------------


//=============================================================================
// Send a formated Logs to console (DO NOT USE DIRECTLY, use LOG*() instead)
//=============================================================================
void __LOG(ConsoleTx* pApi, const char* context, bool whiteText, const char* format, va_list args)
{
  const char* FormatLine = "%s [%u:%02u:%02u:%02u] ";

  // Fast div 1000 (+/- one unit error is not critical for logging purpose)
  uint64_t Val = msCount;
  uint32_t Time = (uint32_t)((Val * 0x00418937) >> 32); // Magic number : Here 0x418937 is 0xFFFFFFFF / 1000d. This is the overflow limit of an uint32
  uint32_t NewTime;
  // Extract fields
  NewTime = (Time / 60); uint32_t Sec = Time - (NewTime * 60); Time = NewTime;
  NewTime = (Time / 60); uint32_t Min = Time - (NewTime * 60); Time = NewTime;
  NewTime = (Time / 24); uint32_t Hor = Time - (NewTime * 24); Time = NewTime;
  uint32_t d = Time;


#ifndef __cplusplus
  char TmpBuff[200];
  siprintf(TmpBuff, FormatLine, context, (unsigned int)d, (unsigned int)Hor, (unsigned int)Min, (unsigned int)Sec);
  SetStrToConsoleBuffer(pApi, TmpBuff);
  if (whiteText) SetStrToConsoleBuffer(pApi, "\x001B[0m");
  vsiprintf(TmpBuff, format, args);
  SetStrToConsoleBuffer(pApi, TmpBuff);
  SetStrToConsoleBuffer(pApi, "\r\n");
  TrySendingNextCharToConsole(pApi);
#else
  printf(FormatLine, context, (unsigned int)d, (unsigned int)Hor, (unsigned int)Min, (unsigned int)Sec);
  vprintf(format, args);
  printf("\r\n");
#endif
}

//-----------------------------------------------------------------------------

//! Severity line color
#ifdef __cplusplus
const int SeverityColors[(size_t)lsLast_] =
{
  lsTitle   = wccLIME  , // lsTitle   -> Color: Text=green  ; Background=black
  lsFatal   = wccRED   , // lsFatal   -> Color: Text=red    ; Background=black
  lsError   = wccRED   , // lsError   -> Color: Text=red    ; Background=black
  lsWarning = wccYELLOW, // lsWarning -> Color: Text=yellow ; Background=black
  lsInfo    = wccAQUA  , // lsInfo    -> Color: Text=blue   ; Background=black
  lsTrace   = wccWHITE , // lsTrace   -> Color: Text=white  ; Background=black
  lsDebug   = wccGRAY  , // lsDebug   -> Color: Text=grey   ; Background=black
  lsSpecial = wccOLIVE , // lsSpecial -> Color: Text=kaki   ; Background=black
};
#else
const char* SeverityColors[(size_t)lsLast_] =
{
  "\x001B[1;32m", // lsTitle   -> Color: Text=green          ; Background=black ; Bold
  "\x001B[1;91m", // lsFatal   -> Color: Text=red bright     ; Background=black ; Bold
  "\x001B[0;91m", // lsError   -> Color: Text=red bright     ; Background=black
  "\x001B[0;93m", // lsWarning -> Color: Text=yellow bright  ; Background=black
  "\x001B[0;36m", // lsInfo    -> Color: Text=cyan           ; Background=black
  "\x001B[0;97m", // lsTrace   -> Color: Text=white          ; Background=black
  "\x001B[0;37m", // lsDebug   -> Color: Text=grey "white"   ; Background=black
  "\x001B[0;33m", // lsSpecial -> Color: Text=yellow         ; Background=black
};
#endif

//-----------------------------------------------------------------------------


//=============================================================================
// Send a formated Logs to console
//=============================================================================
void LOG(ConsoleTx* pApi, eSeverity severity, const char* format, ...)
{
  va_list args;
  va_start(args, format);

#ifdef __cplusplus
  SetConsoleColor(SeverityColors[(size_t)severity], wccBLACK);
#else
  SetStrToConsoleBuffer(pApi, SeverityColors[(size_t)severity]);
#endif

  bool KeepColorFor = (severity == lsFatal) || (severity == lsDebug);
  __LOG(pApi, "DEMO", !KeepColorFor, format, args);

  va_end(args);
}

//-----------------------------------------------------------------------------


#ifdef __cplusplus
//=============================================================================
// Set the Windows console color
//=============================================================================
void SetConsoleColor(eWinConsoleColor text, eWinConsoleColor background)
{
  HANDLE H = GetStdHandle(STD_OUTPUT_HANDLE);
  SetConsoleTextAttribute(H, ((int)background << 4) + (int)text);
}


//=============================================================================
// Send a formated Simulation Logs to console
//=============================================================================
void LOGSIM(ConsoleTx* pApi, const char* format, ...)
{
  va_list args;
  va_start(args, format);

#ifdef __cplusplus
  SetConsoleColor(wccTEAL, wccBLACK);         // Color: Text=blue-grey ; Background=black
#else
  SetStrToConsoleBuffer(pApi,"\x001B[0;96m"); // Color: Text=cyan bright ; Background=black
#endif

  __LOG(pApi, "SIMU", false, format, args);

  va_end(args);
}
#endif

//-----------------------------------------------------------------------------


#if (defined(DEBUG) || defined(_DEBUG))
//=============================================================================
// Show the hexadecimal dump of the memory to console
//=============================================================================
void __HexDump(ConsoleTx* pApi, const char* context, const void* src, unsigned int size)
{
  static const char* Hexa = "0123456789ABCDEF";

  #define ROW_LENGTH  16         // 16 bytes per row
  char HexaDump[ROW_LENGTH * 3]; // [2 digit hexa + space] - 1 space + 1 zero terminal
  char HexaChar[ROW_LENGTH + 1]; // [1 char] + 1 zero terminal

  LOGDEBUG_(pApi, "Dump %d bytes at 0x%08X - %s", size, (unsigned int)src, context);

  unsigned char* pSrc = (unsigned char*)src;
  HexaChar[ROW_LENGTH] = 0;
  for (int32_t i = ((size+ROW_LENGTH-1) / ROW_LENGTH); --i >= 0; pSrc += ROW_LENGTH, size -= ROW_LENGTH)
  {
    memset(HexaDump, ' ', sizeof(HexaDump));
    memset(HexaChar, '.', ROW_LENGTH);
    for (int j = (size >= ROW_LENGTH ? ROW_LENGTH : size); --j >= 0;)
    {
      HexaDump[j * 3 + 0] = Hexa[(pSrc[j] >> 4) & 0xF];
      HexaDump[j * 3 + 1] = Hexa[(pSrc[j] >> 0) & 0xF];
      //HexaDump[j * 3 + 2] = ' ';
      HexaChar[j] = (pSrc[j] < 0x20) ? '.' : pSrc[j];
    }
    HexaDump[ROW_LENGTH * 3 - 1] = 0;

    char TmpBuff[10 + 3 + (ROW_LENGTH * 3) + 2 + (ROW_LENGTH + 1) + 4];
    siprintf(TmpBuff, "  %08X : %s \"%s\"\r\n", (unsigned int)pSrc, HexaDump, HexaChar);
    SetStrToConsoleBuffer(pApi, TmpBuff);
  }
  #undef ROW_LENGTH
}


//=============================================================================
// Show the binary dump of the memory to console
//=============================================================================
void __BinDump(ConsoleTx* pApi, const char* context, const void* src, unsigned int size)
{
  static const char* Bin  = "01";
  static const char* Hexa = "0123456789ABCDEF";

  #define ROW_LENGTH  4          // 4 bytes per row
  char BinDump[ROW_LENGTH * 9]; // [8 digit bin  + space] - 1 space + 1 zero terminal
  char BinHexa[ROW_LENGTH * 3]; // [2 digit hexa + space] - 1 space + 1 zero terminal

  LOGDEBUG_(pApi, "Dump %d bytes at 0x%08X - %s", size, (unsigned int)src, context);

  unsigned char* pSrc = (unsigned char*)src;
  BinHexa[ROW_LENGTH] = 0;
  for (int32_t i = ((size+ROW_LENGTH-1) / ROW_LENGTH); --i >= 0; pSrc += ROW_LENGTH, size -= ROW_LENGTH)
  {
    memset(BinDump, ' ', sizeof(BinDump));
    memset(BinHexa, ' ', sizeof(BinHexa));
    for (int j = (size >= ROW_LENGTH ? ROW_LENGTH : size); --j >= 0;)
    {
      BinDump[j * 9 + 0] = Bin[(pSrc[j] >> 7) & 0x1];
      BinDump[j * 9 + 1] = Bin[(pSrc[j] >> 6) & 0x1];
      BinDump[j * 9 + 2] = Bin[(pSrc[j] >> 5) & 0x1];
      BinDump[j * 9 + 3] = Bin[(pSrc[j] >> 4) & 0x1];
      BinDump[j * 9 + 4] = Bin[(pSrc[j] >> 3) & 0x1];
      BinDump[j * 9 + 5] = Bin[(pSrc[j] >> 2) & 0x1];
      BinDump[j * 9 + 6] = Bin[(pSrc[j] >> 1) & 0x1];
      BinDump[j * 9 + 7] = Bin[(pSrc[j] >> 0) & 0x1];
      //BinDump[j * 3 + 8] = ' ';
      BinHexa[j * 3 + 0] = Hexa[(pSrc[j] >> 4) & 0xF];
      BinHexa[j * 3 + 1] = Hexa[(pSrc[j] >> 0) & 0xF];
      //BinHexa[j * 3 + 2] = ' ';

    }
    BinDump[ROW_LENGTH * 9 - 1] = 0;
    BinHexa[ROW_LENGTH * 3 - 1] = 0;

    char TmpBuff[10 + 3 + (ROW_LENGTH * 9) + 2 + (ROW_LENGTH * 3) + 4];
    siprintf(TmpBuff, "  %08X : %s - %s\r\n", (unsigned int)pSrc, BinDump, BinHexa);
    SetStrToConsoleBuffer(pApi, TmpBuff);
  }
  #undef ROW_LENGTH
}
#endif

//-----------------------------------------------------------------------------
#endif /* USE_CONSOLE_TX */
//-----------------------------------------------------------------------------





//**********************************************************************************************************************************************************
#ifdef USE_CONSOLE_RX
//********************************************************************************************************************
// Console Receive API
//********************************************************************************************************************
//=============================================================================
// Initialize the Console Reception
//=============================================================================
eERRORRESULT InitConsoleRx(ConsoleRx* pApi)
{
#ifdef CHECK_NULL_PARAM
  if (pApi == NULL) return ERR__PARAMETER_ERROR;
# ifdef CONSOLE_RX_USE_COMMAND_RECALL
  if (pApi->CommandBuffer == NULL) return ERR__NULL_BUFFER;
  if (pApi->BufferSize == 0) return ERR__BAD_DATA_SIZE;
# endif
#endif

  //--- Initialize the print buffer ---
  memset(pApi->CurrentBuff, 0, sizeof(pApi->CurrentBuff));
  pApi->RxIdx      = 0;
  pApi->ProcessIdx = 0;
  pApi->CursorIdx  = 0;
#ifdef CONSOLE_RX_USE_COMMAND_RECALL
  pApi->StartIdx   = 0;
  pApi->CurrentIdx = 1;
  memset(pApi->CommandBuffer, 0, (sizeof(pApi->CommandBuffer[0]) * pApi->BufferSize));
  pApi->CommandBuffer[0] = CONSOLE_STX; // Set the start of command char (Start of TeXt)
#endif
  return ERR_OK;
}

//-----------------------------------------------------------------------------


//=============================================================================
// [INTERRUPT] Receive char from console
//=============================================================================
eERRORRESULT ReceiveCharFromConsole(ConsoleRx* pApi)
{
#ifdef CHECK_NULL_PARAM
  if (pApi == NULL) return ERR__PARAMETER_ERROR;
#endif
  UART_Interface* pUART = GET_UART_INTERFACE;
#if defined(CHECK_NULL_PARAM)
# if defined(USE_DYNAMIC_INTERFACE)
  if (pUART == NULL) return ERR__PARAMETER_ERROR;
# endif
  if (pUART->fnUART_Receive == NULL) return ERR__PARAMETER_ERROR;
#endif
  if (pApi->RxIdx >= CONSOLE_RX_CURRENT_BUFFER_SIZE) return ERR__BUFFER_FULL;
  eERRORRESULT Error;

  //--- Receive char from UART ---
  const size_t DataSizeToGet = CONSOLE_RX_CURRENT_BUFFER_SIZE - pApi->RxIdx;  // Calculate data available to the end of buffer
  size_t ActuallyReceived; uint8_t LastCharError;
  Error = pUART->fnUART_Receive(pUART, &pApi->CurrentBuff[pApi->RxIdx], DataSizeToGet, &ActuallyReceived, &LastCharError);
  if (Error != ERR_OK) return Error;                                    // If there is an error while receiving data, return the error
  if (ActuallyReceived == 0) return ERR__NO_DATA_AVAILABLE;             // No data received? Then no data available
  pApi->RxIdx += ActuallyReceived;                                      // Set new RxIdx
  if (LastCharError > 0) return ERR__RECEIVE_ERROR;
  return ERR_OK;
}


//=============================================================================
// Process received char from console
//=============================================================================
eERRORRESULT ProcessReceivedCharFromConsole(ConsoleRx* pApi)
{
#ifdef CHECK_NULL_PARAM
  if (pApi == NULL) return ERR__PARAMETER_ERROR;
  if (pApi->Buffer == NULL) return ERR__PARAMETER_ERROR;
#endif
  size_t RxIdx      = pApi->RxIdx;
  size_t ProcessIdx = pApi->ProcessIdx;

  //--- Process command inputs ---
  while (ProcessIdx < RxIdx)
  {
    const uint8_t CurrentData = pApi->CurrentBuff[ProcessIdx];
    switch (CurrentData)
    {
      case CONSOLE_BS: // BackSpace
        memcpy(&pApi->CurrentBuff[ProcessIdx], &pApi->CurrentBuff[ProcessIdx + 1], (RxIdx - ProcessIdx - 1));              // Remove current backspace by shifting string by 1 char to the left
        --RxIdx;
        if (pApi->CursorIdx > 0)                                                                                           // Handle cursor index not in end of current command and only if cursor index is not at index 0
        {
          memcpy(&pApi->CurrentBuff[pApi->CursorIdx - 1], &pApi->CurrentBuff[pApi->CursorIdx], (RxIdx - pApi->CursorIdx)); // Shift string by 1 char to the left at cursor
          --pApi->CursorIdx;
          --ProcessIdx;
          --RxIdx;
        }
        break;

      case CONSOLE_DEL: // Delete
        memcpy(&pApi->CurrentBuff[ProcessIdx], &pApi->CurrentBuff[ProcessIdx + 1], (RxIdx - ProcessIdx - 1));                  // Remove current delete by shifting string by 1 char to the left
        --RxIdx;
        if (pApi->CursorIdx < ProcessIdx)                                                                                      // Handle cursor index not in end of current command
        {
          memcpy(&pApi->CurrentBuff[pApi->CursorIdx], &pApi->CurrentBuff[pApi->CursorIdx + 1], (RxIdx - pApi->CursorIdx - 1)); // Suppress char at cursor index
          --ProcessIdx;
          --RxIdx;
        }        
        break;

      case CONSOLE_CR: // #13
      case CONSOLE_LF: // #10
        //--- Process command ---
        if (ProcessIdx > 0)
        {
          pApi->CurrentBuff[ProcessIdx] = CONSOLE_NULL;                                // Force a NULL terminal string
          ProcessReceivedCommandCallBack(&pApi->CurrentBuff[0], ProcessIdx);           // Call to a weak function: Process received command Callback
        }
        //--- Prepare next command ---
        const size_t SizeToMove = (RxIdx - ProcessIdx - 1);
        memcpy(&pApi->CurrentBuff[0], &pApi->CurrentBuff[ProcessIdx + 1], SizeToMove); // Suppress CR/LF at process index and move the rest of non processed command to index 0
        pApi->CursorIdx = 0;
        ProcessIdx = 0;
        RxIdx = SizeToMove;                                                            // Set new Rx index
    	  break;

      default:
        if (pApi->CursorIdx < ProcessIdx)                                              // Handle cursor index not in end of current command
        {
          memcpy(&pApi->CurrentBuff[pApi->CursorIdx + 1], &pApi->CurrentBuff[pApi->CursorIdx], (ProcessIdx - pApi->CursorIdx)); // Shift string by 1 char to the right
        }
        pApi->CurrentBuff[pApi->CursorIdx] = CurrentData;                              // Cursor index takes the processed data
        ++pApi->CursorIdx;
        ++ProcessIdx;
        if ((CurrentData != '\r') && (CurrentData != '\n')) SetCharToConsoleBuffer(CONSOLE_TX, CurrentData); // Display received char
        break;
    }
  }
  pApi->RxIdx      = RxIdx;
  pApi->ProcessIdx = ProcessIdx;
  return ERR_OK;
}


//=============================================================================
// [WEAK] Process received command Callback
//=============================================================================
void ProcessReceivedCommandCallBack(const uint8_t* pCmd, size_t size)
{
  (void)pCmd;
  (void)size;
  // It's a weak function, the user need to create the same function in his project and implement things, thus this function will be discarded
}

//-----------------------------------------------------------------------------
#endif /* USE_CONSOLE_RX */
//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif