/***************************************************************************************************
*                    (c) Copyright 2008-2018  Syncretize technologies co.,ltd.
*										All Rights Reserved
*
*\File          app_board.h
*\Description   
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               创建文件。
***************************************************************************************************/
#ifndef _APP_BOARD_H
#define _APP_BOARD_H
#include "public_type.h"
#include "stm32_uart.h"
#include "stm32_spi.h"
#include "stm32_Inrtc.h"
#include "stm32_lptim.h"
#include "drv_sx1278.h"
#include "drv_watchdog.h"
#include "stm32_gpio_irq.h"
#include "lora_pp.h"


//调试功能
//#define USING_USART_PRINTF

extern switch_type MSG_LED;
extern UartDevType UartTTL;
extern LoRa_Dev_type LoRa_Dev;

void app_board_init(void);
#endif /*_APP_BOARD_H*/

