/*
 * crc32.h
 *
 * CRC-32/ISO-HDLC (the zlib / Ethernet CRC-32): poly 0x04C11DB7 reflected
 * (0xEDB88320), init 0xFFFFFFFF, input and output reflected, final XOR
 * 0xFFFFFFFF.
 *
 * Used for the records this firmware keeps on flash: the settings records
 * (settings/SettingsStore.c) and the identity record (settings/Identity.c).
 * The identity record is also read by production tooling off the board, which
 * is why the algorithm is the most widely implemented CRC-32 there is -
 * Python's zlib.crc32() and binascii.crc32() compute exactly this.
 *
 * Plain C, bitwise, for the same reasons crc16.h gives: the STM32WL CRC
 * peripheral would need its reflection and init settings matched by hand to
 * agree with any off-board tool, and a 256-entry table is 1 KB of flash for
 * records of a few hundred bytes.
 *
 * Known answer: CRC32_u32("123456789", 9) == 0xCBF43926.
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include <stdint.h>

#define CRC32_INIT      UINT32_C(0xFFFFFFFF)

/* One-shot. */
uint32_t CRC32_u32(const uint8_t *pu8Data, uint32_t u32Len);

/* Incremental form, for data read in chunks: seed with CRC32_INIT, feed every
 * chunk through CRC32_u32Update, then CRC32_u32Final. */
uint32_t CRC32_u32Update(uint32_t u32Crc, const uint8_t *pu8Data, uint32_t u32Len);
#define  CRC32_u32Final(u32Crc)    (~(uint32_t)(u32Crc))

#endif /* INC_CRC32_H_ */
