/*
 * crc32.c
 *
 * See crc32.h for the algorithm and why it is plain C.
 */

#include "crc32.h"

#include <stddef.h>

uint32_t CRC32_u32Update(uint32_t u32Crc, const uint8_t *pu8Data, uint32_t u32Len)
{
    if (pu8Data == NULL) return u32Crc;

    while (u32Len-- > 0U)
    {
        u32Crc ^= *pu8Data++;
        for (uint8_t u8Bit = 0U; u8Bit < 8U; u8Bit++)
            u32Crc = (u32Crc & 1U) ? ((u32Crc >> 1) ^ 0xEDB88320UL) : (u32Crc >> 1);
    }
    return u32Crc;
}

uint32_t CRC32_u32(const uint8_t *pu8Data, uint32_t u32Len)
{
    return CRC32_u32Final(CRC32_u32Update(CRC32_INIT, pu8Data, u32Len));
}
