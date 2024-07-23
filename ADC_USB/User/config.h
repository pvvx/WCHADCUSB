/*
 * config.h
 *
 *  Created on: Sep 28, 2023
 *      Author: pvvx
 */

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#include "stdio.h"

#define HSE_VALUE    (8000000UL) /* Value of the External oscillator in Hz */

#include "ch32v30x.h"

/* Used Timers:
 * TIM2 - usb timeout, step 10 kHz
 */

#define DUAL_UART           0
#define USB_HID_ENABLE	    0
#define USB_SLEEP_ENABLE	1

#define GPIO_PIN_LED	    GPIO_Pin_8
#define GPIO_PORT_LED	    GPIOA

#define UART_DE_ENABLE		0
#define UART_INT_ENABLE		0

#define DEF_CDC_UART1       USART3
#define DEF_UART1_TX_DMA_CH        DMA1_Channel2                                /* Serial 3 transmit channel DMA channel */
#define DEF_UART1_RX_DMA_CH        DMA1_Channel3                                /* Serial 3 transmit channel DMA channel */

#define GPIO_PIN_URX3		GPIO_Pin_11
#define GPIO_PIN_UTX3		GPIO_Pin_10
#define GPIO_PORT_UART3		GPIOB
#define GPIO_PIN_UDE3		GPIO_Pin_12
#define GPIO_PORT_UDE3		GPIOB

#define DEF_CDC_UART2       USART2
#define DEF_UART2_TX_DMA_CH        DMA1_Channel7                                /* Serial 2 transmit channel DMA channel */
#define DEF_UART2_RX_DMA_CH        DMA1_Channel6                                /* Serial 2 transmit channel DMA channel */

#define GPIO_PIN_URX2       GPIO_Pin_3
#define GPIO_PIN_UTX2       GPIO_Pin_2
#define GPIO_PORT_UART2     GPIOA
#define GPIO_PORT_UDE2      GPIOA
#define GPIO_PIN_UDE2       GPIO_Pin_4

#define DEBUG               DEBUG_UART1
#define DEBUG_UART_PIN      GPIO_Pin_15
#define DEBUG_UART_PORT     GPIOB

#define DEBUG_PRINTF_ENA    DEBUG // =0 - §à§ä§Ü§Ý§ð§é§Ú§ä§î debug_printf()

/* USB Interfaces */
enum {
    usb_interface_cdc_0 = 0x00,
#if DUAL_UART
    usb_interface_cdc_1 = 0x02,
#endif
#if USB_HID_ENABLE
    usb_interface_hid = 2 + DUAL_UART*2
#endif
}; // USB_INTERFACE_E;

#if _BYTE_ORDER == _LITTLE_ENDIAN
#define th16(x) x
#else
//__htons() __builtin_bswap16()
#define th16(x) ((((uint16_t)x>>8)&0xff) | (((uint16_t)x & 0xff) << 8))
#endif


extern void GPIO_LED_Toggle(void);

//#include "debug.h"
//#include "UART.h"
//#include "ch32v30x.h"

#endif /* _USER_CONFIG_H_ */
