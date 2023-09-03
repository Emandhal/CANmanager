/*!*****************************************************************************
 * @file    CRC16_CMS.h
 * @author  Fabien 'Emandhal' MAILLY
 * @version 1.0.0
 * @date    14/05/2020
 * @brief   CRC16-CMS implementation
 * @details
 *  The CRC16-CMS polynomial is x^16 + x^15 + x^5 + 1 (0x8005)
 *   - Does not use RefIN and RefOUT, the initial value 0xFFFF
 *   - The result is XORed with 0x0000
 *   - http://reveng.sourceforge.net/crc-catalogue/16.htm
 ******************************************************************************/
#ifndef CRC16_CMS_H_INC
#define CRC16_CMS_H_INC
//=============================================================================

//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
//-----------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//-----------------------------------------------------------------------------





#if !defined(CRC16CMS_NOTABLE)
/*! @brief Compute a byte stream with CRC16-CMS with a table
 *
 * @param[in] *data Is the pointed byte stream
 * @param[in] size Is the size of the pointed byte stream
 * @return The CRC computed
 */
uint16_t ComputeCRC16CMS(const uint8_t* data, size_t size);

#else

/*! @brief Compute a byte stream with CRC16-CMS without a table
 *
 * @param[in] *data Is the pointed byte stream
 * @param[in] size Is the size of the pointed byte stream
 * @return The CRC computed
 */
uint16_t ComputeCRC16CMS(const uint8_t* data, size_t size);

#endif





//-----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
//-----------------------------------------------------------------------------
#endif /* CRC16_CMS_H_INC */