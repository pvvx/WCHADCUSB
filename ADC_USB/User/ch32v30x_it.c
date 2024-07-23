/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_it.h"
#include "UART.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler( void )__attribute__((interrupt("WCH-Interrupt-fast")));
void PVD_IRQHandler(void) __attribute__((interrupt( "WCH-Interrupt-fast" )));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      PVD_IRQHandler
 *
 * @brief   This function handles PVD.
 *
 * @return  none
 */
void PVD_IRQHandler(void)
{
    if (PWR_GetFlagStatus(PWR_FLAG_PVDO))
    {
        PWR_ClearFlag(PWR_FLAG_PVDO);
#if DEBUG
        debug_write("\r\nLowPower!");
#endif
        Delay_Ms(1000);
#if DEBUG
        debug_write(" -> Restart\r\n");
#endif
        /* Software reset */
        NVIC_SystemReset();
    }
    EXTI_ClearITPendingBit(EXTI_Line16); /* §°§é§Ú§ã§ä§Ú§ä§î §æ§Ý§Ñ§Ô */
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void TIM2_IRQHandler( void )
{
    /* uart timeout counts */
    Uart1.Rx_TimeOut++;
    Uart1.USB_Send_TimeOut++;
#if DUAL_UART
    Uart2.Rx_TimeOut++;
    Uart2.USB_Send_TimeOut++;
#endif
    /* clear status */
    TIM2->INTFR = (uint16_t)~TIM_IT_Update;
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    uint32_t i = 0;
#if DEBUG
    debug_write("Hardfault\r\n");
#endif
    while (--i);
    NVIC_SystemReset();
}


