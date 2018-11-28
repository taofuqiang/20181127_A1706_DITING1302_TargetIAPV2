/***************************************************************************************************
*                    (c) Copyright 1992-2015 Syncretize technologies co.,ltd.
*                                       All Rights Reserved
*
*\File          watchdog.h
*\Description   
*\Note          
*\Log           2015.05.30    Ver 1.0    张波
*               创建文件。
***************************************************************************************************/
#ifndef _WATCHDOG_H
#define _WATCHDOG_H

#include "stm32l4xx.h"

void watchdog_init(void);
void Watchdog_Periodic_Handle(uint32_t localtime);
void AutoReset_Periodic_Handle(uint32_t localtime);
#endif /*_WATCHDOG_H*/

