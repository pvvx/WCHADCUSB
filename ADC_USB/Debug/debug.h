/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "config.h"

/* UART Printf Definition */
#define DEBUG_NONE     0
#define DEBUG_UART1    1
#define DEBUG_UART2    2
#define DEBUG_UART3    3

/* DEBUG UATR Definition */
#ifndef DEBUG
#define DEBUG   DEBUG_UART1
#endif

#ifndef DEBUG_PRINTF_ENA
#define DEBUG_PRINTF_ENA    DEBUG
#endif

/* SDI Printf Definition */
#define SDI_PR_CLOSE   0
#define SDI_PR_OPEN    1

#ifndef SDI_PRINT
#define SDI_PRINT   SDI_PR_CLOSE
#endif


void Delay_Init(void);
void Delay_Us (uint32_t n);
void Delay_Ms (uint32_t n);

#if DEBUG
void USART_Printf_Init(uint32_t baudrate);
void SDI_Printf_Enable(void);
void debug_write_srt(void *buf);
#if DEBUG_PRINTF_ENA
#define debug_printf(...) printf(__VA_ARGS__)
#else
#define debug_printf(...)
#endif
#define debug_write(a) debug_write_srt(a)
#else
#define USART_Printf_Init(a)
#define debug_printf(...)
#define debug_write(a)
#endif

uint8_t bin2hex(uint8_t x);

#ifdef __cplusplus
}
#endif

#endif 



