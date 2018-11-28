/***************************************************************************************************
*                    (c) Copyright 2008-2014 Syncretize technologies co.,ltd.
*                                       All Rights Reserved
*
*\File          udsl_buf.h
*\Description   
*\Note          
*\Log           2014.11.24    Ver 1.0    张波
*               创建文件。
***************************************************************************************************/
#ifndef _USDL_BUF_H
#define _USDL_BUF_H
#include "public_type.h"

/*接收缓冲区类型*/
typedef struct  RxBuf_Type_st
{
    u32 Head;
    u32 Tail;
    u8* RxBuf;
    u32 RxBufSize;
}RxBufType;


/*发送缓冲区类型*/
typedef struct TxBufType_st
{
    u32 len;
    u32 offset;
    u8* TxBuf;
}TxBufType;

void RxBufInit(RxBufType* buf, u8* buf_addr, u32 size);
bool AddByteToBuf(RxBufType* buf, u8 byte);
bool AddDataToBuf(RxBufType* buf, u8* data, u32 len);
u32 GetLenFromBuf(RxBufType* buf);
bool GetByteFromBuf(RxBufType* buf, u8* byte);
u32 DelDataFromBuf(RxBufType* buf, u32 len);
u32 ReadDataFromBuf(RxBufType* buf, u8* byte, u32 len, bool clear);
bool ClearDateFromBuf(RxBufType* buf);

#endif
