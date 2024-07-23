/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *Example routine to emulate a simulate USB-CDC Device, USE USART2(PA2/PA3);
 *Please note: This code uses the default serial port 1 for debugging,
 *if you need to modify the debugging serial port, please do not use USART2
*/

#include "UART.h"
#include "debug.h"
#include "adc_dev.h"


/*********************************************************************
 * @fn GPIO_LED_Init
 *
 * @brief   Initializes GPIO LED.
 *
 * @return  none
 */
void GPIO_LED_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    //GPIO_SetBits(GPIO_PORT_LED, GPIO_PIN_LED);
    GPIO_ResetBits(GPIO_PORT_LED, GPIO_PIN_LED);

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT_LED, &GPIO_InitStructure);

}

/*********************************************************************
 * @fn GPIO_LED_Toggle
 *
 * @brief   LED Toggle.
 *
 * @return  none
 */
void GPIO_LED_Toggle(void)
{
    static uint8_t tog; // 孝扭把忘志抖快扶我快 LED, LDE toggle
    tog ? (GPIO_PORT_LED->BSHR = GPIO_PIN_LED):(GPIO_PORT_LED->BCR = GPIO_PIN_LED);
    tog ^= 1;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
	SystemCoreClockUpdate( );
	Delay_Init( );
#if DEBUG
	USART_Printf_Init( 115200 );
#endif
#ifdef GPIO_PIN_LED
	GPIO_LED_Init();
#endif
#if DEBUG_PRINTF_ENA
	debug_printf( "Simulate USB-CDC Device running on USBHS Controller\r\n" );
	debug_printf( "ChipID: %08x\r\n", DBGMCU_GetCHIPID() );
    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    debug_printf( "SYSCLK: %d\r\n", RCC_ClocksStatus.SYSCLK_Frequency);
    debug_printf( "HCLK:   %d\r\n", RCC_ClocksStatus.HCLK_Frequency);
    debug_printf( "PCLK1:  %d\r\n", RCC_ClocksStatus.PCLK1_Frequency);
    debug_printf( "PCLK2:  %d\r\n", RCC_ClocksStatus.PCLK2_Frequency);
    debug_printf( "ADCCLK: %d\r\n", RCC_ClocksStatus.ADCCLK_Frequency);
/*
ChipID: 30700528
SYSCLK: 96000000
HCLK:   96000000
PCLK1:  48000000
PCLK2:  96000000
ADCCLK: 48000000
 */
#else
    debug_write("\r\nSimulate USB-CDC Device running on USBHS Controller\r\n");
    //GPIO_LED_Toggle();
#endif

	/* Usarts init */
    UARTs_Init();


    /* USB20HS device init */
	USBHS_RCC_Init( );
	USBHS_Device_Init( ENABLE );

	Init_ADC();

	while(1)
	{
	    Task_Read_from_USB( &Uart1 );
        Task_Send_to_USB( &Uart1 );
	}
}
