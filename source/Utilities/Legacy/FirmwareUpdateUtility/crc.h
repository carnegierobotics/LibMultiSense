// 32 BIT ANSI X3.66 CRC, polynomial 0xedb88320
//
// Derived from work by Gary S. Brown, see crc.c for more details.

#ifndef _CRC_H_
#define _CRC_H_

#include <stdint.h>

#if __cplusplus
extern "C" {
#endif

uint32_t crc32(uint32_t crc, const void *buf, uint32_t len);

#if __cplusplus
}
#endif

#endif // _CRC_H_
