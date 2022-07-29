#ifndef __DATAFLASHAPI_H__
#define __DATAFLASHAPI_H__

#include "base_types.h"

#ifdef __cplusplus
extern "C"{
#endif

INT DataFlash_Write(BYTE *dest, BYTE *src, INT nSize);
INT DataFlash_Erase(BYTE *dest, INT nSize);

#ifdef __cplusplus
}
#endif

#endif // __DATAFLASHAPI_H__
