/***************************************************************************************************
*                    (c) Copyright 2008-2018  Syncretize technologies co.,ltd.
*										All Rights Reserved
*
*\File          crc32_calc.h
*\Description   
*\Note          
*\Log           2018.01.18    Ver 1.0    Job
*               创建文件。
***************************************************************************************************/
#ifndef _CRC32_CALC_H
#define _CRC32_CALC_H
#include "stm32l4xx.h"

typedef struct
{
    unsigned long crc;
} CRC32_CTX;

void CRC32_Init(CRC32_CTX *ctx);
void CRC32_Update(CRC32_CTX *ctx, uint8_t *data, uint32_t len);
uint32_t CRC32_Final(CRC32_CTX *ctx);
#endif /*_CRC32_CALC_H*/

