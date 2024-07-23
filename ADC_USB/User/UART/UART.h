/********************************** (C) COPYRIGHT *******************************
* File Name          : UART.H
* Author             : WCH
* Modified			 : pvvx
* Version            : V1.01
* Date               : 2022/12/13
* Description        : UART communication-related headers
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "string.h"
#include "debug.h"
#include "string.h"
#include "ch32v30x_usbhs_device.h"
#include "ch32v30x_conf.h"

/******************************************************************************/
/* Related macro definitions */
/* Serial buffer related definitions */
#define DEF_USB_PACK_LEN            DEF_USBD_HS_PACK_SIZE                       /* USB full speed mode packet size for serial x data */
#define DEF_UARTx_RX_BUF_LEN       ( 2 * DEF_USB_PACK_LEN )                     /* min 2, Serial x receive buffer size */
#define DEF_UARTx_TX_BUF_LEN       ( 5 * DEF_USB_PACK_LEN )                     /* min 2, Serial x transmit buffer size */
#define DEF_UARTx_RX_BUF_NUM_MAX   ( DEF_UARTx_RX_BUF_LEN / DEF_USB_PACK_LEN )  /* Serial x transmit buffer size */
#define DEF_UARTx_TX_BUF_NUM_MAX   ( DEF_UARTx_TX_BUF_LEN / DEF_USB_PACK_LEN )  /* Serial x transmit buffer size */

/* Serial port receive timeout related macro definition */
#define DEF_UARTx_BAUDRATE         115200                                       /* Default baud rate for serial port */
#define DEF_UARTx_STOPBIT          0                                            /* Default stop bit for serial port */
#define DEF_UARTx_PARITY           0                                            /* Default parity bit for serial port */
#define DEF_UARTx_DATABIT          8                                            /* Default data bit for serial port */
#define DEF_UARTx_RX_TIMEOUT       17    /* = 1.7 ms, Serial port receive timeout, in 100uS */
#define DEF_UARTx_USB_UP_TIMEOUT   30000 /* = 0.0001 sec, Serial port receive upload timeout, in 100uS */



/************************************************************/
/* Serial port USB config */
typedef struct __attribute__ ((packed)) {
    union {
        uint32_t    dBaudRate;
        uint8_t    bBaudRate[4];
    };
    uint8_t     bStopBit;
    uint8_t     bParity;
    uint8_t     bDataBit;
} USB_COM_CONFIG;

/* Serial port X related structure definition */
typedef struct _UART_CTL
{
	USART_TypeDef *USARTx;			/* USART2/USART3 */
	DMA_Channel_TypeDef *DMA_Channel_tx;	/* DMA UART tx */
	DMA_Channel_TypeDef *DMA_Channel_rx;	/* DMA UART rx */
	uint8_t  *UART_Tx_Buf;	 /* UART2_Tx_Buf/UART3_Tx_Buf */
	uint8_t  *UART_Rx_Buf;	 /* UART3_Tx_Buf/UART3_Tx_Buf */
#if	UART_DE_ENABLE
    GPIO_TypeDef *GPIOde;
    uint16_t pin_de;
#endif
#if DEBUG_PRINTF_ENA
    uint32_t RxCount;
#endif
    uint32_t TxCount;
#if DUAL_UART
    uint8_t  USB_tx_ep; /* DEF_UEP2/DEF_UEP4 */
    uint8_t  USB_rx_ep; /* DEF_UEP2/DEF_UEP4 */
#endif
    uint8_t  Rx_TimeOut;	/* §ã§é§Ö§ä§é§Ú§Ü §ß§Ñ §ä§Ñ§Û§Þ§Ö§â§Ö Serial x data receive timeout, timer2 tik 100 us (10 kHz) */
    uint8_t  Rx_TimeOutMax;	/* Serial x data receive timeout maximum, default (minimum) 1.7 ms */

    uint8_t  USB_Recv_StopFlag; /* §¶§Ý§Ñ§Ô: §á§â§Ú§Ö§Þ §Ó USB §à§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß */
    uint16_t Tx_LoadNum;	/* §å§Ü§Ñ§Ù§Ñ§ä§Ö§Ý§î §á§â§Ú§Ö§Þ§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã USB  */
    uint16_t Tx_DealNum;	/* §ß§à§Þ§Ö§â §Ò§Ý§à§Ü§Ñ §Õ§Ý§ñ §Ó§í§Ò§à§â§Ü§Ú §á§â§Ú§ß§ñ§ä§í§ç §á§à USB §Õ§Ñ§ß§ß§í§ç */
    uint16_t Tx_RemainNum;	/* §Ü§à§Ý-§Ó§à §ß§Ö§à§Ò§â§Ñ§Ò§à§ä§Ñ§ß§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã §á§â§Ú§ß§ñ§ä§í§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú §á§à USB */
    uint16_t Tx_PackLen[ DEF_UARTx_TX_BUF_NUM_MAX ];	/* The current packets length of the buffers */

    uint16_t Rx_LoadNum; /* §å§Ü§Ñ§Ù§Ñ§ä§Ö§Ý§î §ß§à§Þ§Ö§â§à§Ó §á§Ö§â§Ö§Õ§Ñ§Ó§Ñ§Ö§Þ§í§ç §Ò§Ý§à§Ü§à§Ó §Ó USB */
    uint16_t Rx_DealNum; /* §ß§à§Þ§Ö§â §Ò§Ý§à§Ü§Ñ §Õ§Ý§ñ §Ó§í§Ò§à§â§Ü§Ú §Õ§Ñ§ß§ß§í§ç §á§à USB */
    uint16_t Rx_RemainNum; /* §Ü§à§Ý-§Ó§à §ß§Ö§à§Ò§â§Ñ§Ò§à§ä§Ñ§ß§ß§í§ç §Ò§Ý§à§Ü§à§Ó §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Ó USB */
    uint16_t Rx_PackLen[ DEF_UARTx_RX_BUF_NUM_MAX ];   /* The current packets length of the buffers */

    uint16_t USB_Send_TimeOut;	/* packet upload timeout timer, timer2 tik 100 us (10 kHz) */
    uint8_t  USB_Send_Flag; // §Ú§Õ§Ö§ä §á§Ö§â§Ö§Õ§Ñ§é§Ñ §Ò§Ý§à§Ü§Ñ §Ó USB

    uint8_t	 reinit;    // §æ§Ý§Ñ§Ô §â§Ö§Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú
    USB_COM_CONFIG  UsbComCfg;	/* Serial x parameter configuration (default baud rate is 115200, 1 stop bit, no parity, 8 data bits) */

    uint16_t rts_dtr; // bit0 - DTR, bit1 - RST

} UART_CTL, *PUART_CTL;

typedef enum {
	UART_MODIFY_MODE = 0,	// = 0 : Used modify initialization
	UART_INIT_MODE			// = 1 : Used in default initializations
} UART_MODES_E;

typedef enum {
	UART_DMA_TX = 0,
	UART_DMA_RX
} UART_DMA_TYPE_E;

/***********************************************************************************************************************/
/* Constant, variable extents */
/* The following are serial port transmit and receive related variables and buffers */
//extern volatile UART_CTL Uart1, Uart2;                                                    /* Serial x control related structure */
extern UART_CTL Uart1;    /* Serial x control related structure */
extern __attribute__ ((aligned(4))) uint8_t UART1_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ]; /* Serial x transmit buffer */
extern __attribute__ ((aligned(4))) uint8_t UART1_Rx_Buf[ DEF_UARTx_RX_BUF_LEN ]; /* Serial x transmit buffer */
#if DUAL_UART
extern UART_CTL Uart2;    /* Serial x control related structure */
extern __attribute__ ((aligned(4))) uint8_t UART2_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ]; /* Serial x transmit buffer */
extern __attribute__ ((aligned(4))) uint8_t UART2_Rx_Buf[ DEF_UARTx_RX_BUF_LEN ]; /* Serial x transmit buffer */
#endif

/***********************************************************************************************************************/
/* Function extensibility */
extern void UARTs_SetConfiguration( void );
//extern void TIM2_Init( void );
extern void UARTs_CfgInit( UART_CTL * puart, uint32_t baudrate, uint8_t stopbits, uint8_t parity ); /* UART1 initialization */
extern void UARTs_ParaInit( UART_CTL * puart );			/* Serial port parameter initialization */
extern void UARTs_SetRxTimeout(UART_CTL * puart, uint32_t baudrate); /* Uart set rx char timeout (Rx_TimeOutMax) */
extern void UARTs_DMAInit( UART_CTL * puart, UART_DMA_TYPE_E type, uint8_t *pbuf, uint32_t len );           /* Serial port 1-related DMA initialization */
extern void UARTs_Init( void ); 						/* Serial ports initialization */
extern void Task_Read_from_USB( UART_CTL * puart );		/* Serial port 1 data sending processing  */
extern void Task_Send_to_USB( UART_CTL * puart );		/* Serial port 1 data reception processing */
extern void UARTs_USB_Init( UART_CTL * puart );			/* USB serial port initialization*/


#ifdef __cplusplus
}
#endif

#endif

/***********************************************************************************************************************/


