/********************************** (C) COPYRIGHT *******************************
* File Name          : UART.C
* Author             : WCH
* Modified			 : pvvx
* Version            : V1.01
* Date               : 2022/12/13
* Description        : uart serial port related initialization and processing
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "UART.h"
#include "adc_dev.h"

/* Used Timers:
 * TIM2 - usb timeout, step 10 kHz
 */

#define USBHSD_UEP_TX_CTRL( N )        ( *((volatile uint8_t *)( USBHSD_UEP_TXCTL_BASE - 4 +  N * 4 ) ) )
#define USBHSD_UEP_RX_CTRL( N )        ( *((volatile uint8_t *)( USBHSD_UEP_RXCTL_BASE - 4 +  N * 4 ) ) )
#define USBHSD_UEP_TX_DMA( N )         ( *((volatile uint32_t *)( USBHSD_UEP_TXDMA_BASE - 4 + N * 4 ) ) )
#define USBHSD_UEP_RX_DMA( N )         ( *((volatile uint32_t *)( USBHSD_UEP_RXDMA_BASE - 4 + N * 4 ) ) )
#define USBHSD_UEP_TX_LEN( N )         ( *((volatile uint16_t *)( USBHSD_UEP_TXLEN_BASE - 4 + N * 4 ) ) )

/*******************************************************************************/
/* Variable Definition */
/* Global */

/* The following are serial port transmit and receive related variables and buffers */
UART_CTL Uart1;
#if DUAL_UART
UART_CTL Uart2;
#endif
__attribute__ ((aligned(4))) uint8_t  UART1_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ];  /* Serial port 2 transmit data buffer */
__attribute__ ((aligned(4))) uint8_t  UART1_Rx_Buf[ DEF_UARTx_RX_BUF_LEN ];  /* Serial port 2 receive data buffer */
#if DUAL_UART
__attribute__ ((aligned(4))) uint8_t  UART2_Tx_Buf[ DEF_UARTx_TX_BUF_LEN ];  /* Serial port 2 transmit data buffer */
__attribute__ ((aligned(4))) uint8_t  UART2_Rx_Buf[ DEF_UARTx_RX_BUF_LEN ];  /* Serial port 2 receive data buffer */
#endif

#if UART_DE_ENABLE && UART_INT_ENABLE
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn      USART2_IRQHandler
 *
 * @brief   This function handles USART2 global interrupt request.
 *
 * @return  none
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
    	GPIO_PORT_UDE2->BCR = GPIO_PIN_UDE2;  // UDE Low
//            USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    }
}
#endif

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   100us (10kHz) Timer
 *          144 * 100 * 13.8888 -----> 100uS
 *
 * @return  none
 */
void TIM2_Init( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};

    TIM_DeInit( TIM2 );

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;	// 10 kHz
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / 1000000 - 1; // 1 MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

    /* Clear TIM2 update pending flag */
    TIM_ClearFlag( TIM2, TIM_FLAG_Update );

    /* TIM IT enable */
    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );

    /* Enable Interrupt */
    NVIC_EnableIRQ( TIM2_IRQn );

    /* TIM2 enable counter */
    TIM_Cmd( TIM2, ENABLE );
}

/*********************************************************************
 * @fn      UARTs_set_default_cfg
 *
 * @brief   Set Com_Cfg the default.
 *
 * @return  none
 */

void UARTs_set_default_cfg(PUART_CTL puart)
{
    puart->UsbComCfg.bBaudRate[0] = (uint8_t)( DEF_UARTx_BAUDRATE );
    puart->UsbComCfg.bBaudRate[1] = (uint8_t)( DEF_UARTx_BAUDRATE >> 8 );
    puart->UsbComCfg.bBaudRate[2] = (uint8_t)( DEF_UARTx_BAUDRATE >> 16 );
    puart->UsbComCfg.bBaudRate[3] = (uint8_t)( DEF_UARTx_BAUDRATE >> 24 );
    puart->UsbComCfg.bStopBit = DEF_UARTx_STOPBIT;
    puart->UsbComCfg.bParity = DEF_UARTx_PARITY;
    puart->UsbComCfg.bDataBit = DEF_UARTx_DATABIT;
//    puart->UsbComCfg.bRxTimeOut = DEF_UARTx_RX_TIMEOUT;
}
/*********************************************************************
 * @fn      UARTs_Init
 *
 * @brief   Uart2 total initialization
 *
 * @return  none
 */
void UARTs_Init( void )
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE );
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE); //    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    Uart1.USARTx = DEF_CDC_UART1;
#if UART_DE_ENABLE
    Uart1.pin_de = GPIO_PIN_UDE3;
    Uart1.GPIOde = GPIO_PORT_UDE3;
#endif
#if DUAL_UART
    Uart1.USB_tx_ep = DEF_UEP2;
    Uart1.USB_rx_ep = DEF_UEP2;
#endif
    Uart1.DMA_Channel_tx = DEF_UART1_TX_DMA_CH;
    Uart1.DMA_Channel_rx = DEF_UART1_RX_DMA_CH;
    Uart1.UART_Tx_Buf = UART1_Tx_Buf;
    Uart1.UART_Rx_Buf = UART1_Rx_Buf;
    UARTs_set_default_cfg(&Uart1);
#if DUAL_UART
    Uart2.USARTx = DEF_CDC_UART2;
#if UART_DE_ENABLE
    Uart2.pin_de = GPIO_PIN_UDE2;
    Uart2.GPIOde = GPIO_PORT_UDE2;
#endif
    Uart2.DMA_Channel_tx = DEF_UART2_TX_DMA_CH;
    Uart2.DMA_Channel_rx = DEF_UART2_RX_DMA_CH;

    Uart2.USB_tx_ep = DEF_UEP4;
    Uart2.USB_rx_ep = DEF_UEP4;
    Uart2.UART_Tx_Buf = UART2_Tx_Buf;
    Uart2.UART_Rx_Buf = UART2_Rx_Buf;
    UARTs_set_default_cfg(&Uart2);
#endif

    TIM2_Init();
	UARTs_USB_Init(&Uart1);
#if DUAL_UART
	UARTs_USB_Init(&Uart2);
#endif
}

/*********************************************************************
 * @fn      UART_CfgInit
 *
 * @brief   Uart2/Uart3 configuration initialization
 *
 * @return  none
 */
void UARTs_CfgInit(PUART_CTL puart, uint32_t baudrate, uint8_t stopbits, uint8_t parity )
{
    USART_InitTypeDef USART_InitStructure = {0};
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_TypeDef *USARTx = puart->USARTx;
    uint16_t pin_tx;
    uint16_t pin_rx;
    GPIO_TypeDef *GPIOx;
    if(USARTx == USART2) {
    	pin_tx = GPIO_PIN_UTX2;
    	pin_rx = GPIO_PIN_URX2;
    	GPIOx = GPIO_PORT_UART2;
    } else if(USARTx == USART3) {
    	pin_tx = GPIO_PIN_UTX3;
    	pin_rx = GPIO_PIN_URX3;
    	GPIOx = GPIO_PORT_UART3;
    } else
    	return;

    /* delete contains in ( ... )  */
    /* First set the serial port introduction to output high then close the TE and RE of CTLR1 register (note that USARTx->CTLR1 register setting 9 bits has a limit) */
    /* Note: This operation must be performed, the TX pin otherwise the level will be pulled low */
    GPIOx->BSHR = pin_tx;
    GPIO_InitStructure.GPIO_Pin   = pin_tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init( GPIOx, &GPIO_InitStructure );
    /* clear te/re */
    USARTx->CTLR1 &= ~( USART_CTLR1_TE | USART_CTLR1_RE );
#if UART_DE_ENABLE
    /* UDE3 pin */
    puart->GPIOde->BCR = puart->pin_de;  // UDE Low
    GPIO_InitStructure.GPIO_Pin   = puart->pin_de;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init( puart->GPIOde, &GPIO_InitStructure );
#endif
    /* Configure USART3 Rx (PB11) as input floating */
    GPIO_InitStructure.GPIO_Pin   = pin_rx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init( GPIOx, &GPIO_InitStructure );
    /* Configure USART3 Tx (PB10) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin   = pin_tx;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init( GPIOx, &GPIO_InitStructure );
    /* USART configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
                         the SCLK pin
    */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    /* Number of stop bits (0: 1 stop bit; 1: 1.5 stop bits; 2: 2 stop bits). */
    if( stopbits == 1 )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    }
    else if( stopbits == 2 )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }
    else
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }

    /* Check digit (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space); */
    if( parity == 1 )
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
    else if( parity == 2 )
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
    else
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init( USARTx, &USART_InitStructure );
    USART_ClearFlag( USARTx, USART_FLAG_TC );

#if UART_DE_ENABLE & UART_INT_ENABLE
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    USART_ITConfig(USARTx, USART_IT_TC, ENABLE); // USART_IT_TC - Transmission complete interrupt
    if(USARTx == USART2)
    	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    else
    	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

    /* Enable USART */
    USART_Cmd( USARTx, ENABLE );
}

/*********************************************************************
 * @fn      UART2_ParaInit
 *
 * @brief   Uart2 parameters initialization
 * @return  none
 */
void UARTs_ParaInit(UART_CTL * puart)
{
#if UART_DE_ENABLE & UART_INT_ENABLE
    puart->GPIOde->BCR = puart->pin_de;  // pin UDE Low
#endif

//    NVIC_DisableIRQ( USBHS_IRQn );
    NVIC_DisableIRQ( USBHS_IRQn );

    puart->Rx_LoadNum = 0;
    puart->Rx_DealNum = 0;
    puart->Rx_RemainNum = 0;
    memset(puart->Rx_PackLen, 0, sizeof(puart->Rx_PackLen));

    puart->USB_Send_Flag = 0;
    puart->USB_Send_TimeOut = 0;

    puart->Tx_LoadNum = 0;
    puart->Tx_DealNum = 0;
    puart->Tx_RemainNum = 0;
    memset(puart->Tx_PackLen, 0, sizeof(puart->Rx_PackLen));
    puart->USB_Recv_StopFlag = 0;

    /* restart usb receive  */
#if DUAL_UART
    USBHSD_UEP_RX_DMA(puart->USB_rx_ep) = (uint32_t)puart->UART_Tx_Buf;
    USBHSD_UEP_TX_DMA(puart->USB_tx_ep) = (uint32_t)puart->UART_Rx_Buf;
    USBHSD_UEP_RX_CTRL(puart->USB_rx_ep) &= ~USBHS_UEP_R_RES_MASK;
    USBHSD_UEP_RX_CTRL(puart->USB_rx_ep) |= USBHS_UEP_R_RES_ACK;
#else
    USBHSD->UEP2_RX_DMA = (uint32_t)puart->UART_Tx_Buf;
    USBHSD->UEP2_TX_DMA = (uint32_t)puart->UART_Rx_Buf;
    USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
    USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
#endif

    NVIC_EnableIRQ( USBHS_IRQn );
}

/*********************************************************************
 * @fn      UART2_DMAInit
 *
 * @brief   Uart2 DMA configuration initialization
 *          type = 0 : UART_DMA_TX - USART2_TX
 *          type = 1 : UART_DMA_RX -  USART2_RX
 *          pbuf     : Tx/Rx Buffer, should be aligned(4)
 *          len      : buffer size of Tx/Rx Buffer
 *
 * @return  none
 */
void UARTs_DMAInit( UART_CTL * puart, UART_DMA_TYPE_E type, uint8_t *pbuf, uint32_t len )
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&puart->USARTx->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)pbuf;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;


    if( type == UART_DMA_TX )
    {
        /* UART2 Tx-DMA configuration */
        DMA_DeInit( puart->DMA_Channel_tx );

        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_Init( puart->DMA_Channel_tx, &DMA_InitStructure );

        DMA_Cmd( puart->DMA_Channel_tx, ENABLE );
    }
    else // UART_DMA_RX
    {
        /* UART2 Rx-DMA configuration */
        DMA_DeInit( puart->DMA_Channel_rx );

        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_Init( puart->DMA_Channel_rx, &DMA_InitStructure );

        DMA_Cmd( puart->DMA_Channel_rx, ENABLE );
    }
}

/*********************************************************************
 * @fn      UART2_USB_Init
 *
 * @brief   Uart2 initialization in usb interrupt
 *
 * @return  none
 */
void UARTs_USB_Init(UART_CTL * puart)
{
    uint32_t baudrate;

    puart->UsbComCfg.bBaudRate[0] = (uint8_t)( DEF_UARTx_BAUDRATE );
    puart->UsbComCfg.bBaudRate[1] = (uint8_t)( DEF_UARTx_BAUDRATE >> 8 );
    puart->UsbComCfg.bBaudRate[2] = (uint8_t)( DEF_UARTx_BAUDRATE >> 16 );
    puart->UsbComCfg.bBaudRate[3] = (uint8_t)( DEF_UARTx_BAUDRATE >> 24 );
    puart->UsbComCfg.bStopBit = DEF_UARTx_STOPBIT;
    puart->UsbComCfg.bParity = DEF_UARTx_PARITY;
    puart->UsbComCfg.bDataBit = DEF_UARTx_DATABIT;
//    puart->UsbComCfg.bRxTimeOut = DEF_UARTx_RX_TIMEOUT;

    baudrate = ( uint32_t )( puart->UsbComCfg.bBaudRate[ 3 ] << 24 ) + ( uint32_t )( puart->UsbComCfg.bBaudRate[ 2 ] << 16 );
    baudrate += ( uint32_t )( puart->UsbComCfg.bBaudRate[ 1 ] << 8 ) + ( uint32_t )( puart->UsbComCfg.bBaudRate[ 0 ] );

    if(baudrate < 300)
    	baudrate = 300;
    else if (baudrate > 6000000)
    	baudrate = 6000000;

    if (baudrate > 19200)
    {
        // T1_5 (us) = 750
        // T3_5 (us) = 1750
        puart->Rx_TimeOutMax = 1750/100;
    }
    else
    {
        // T1_5 (us) = 15000000/baud
        // T3_5 (us) = 35000000/baud
        puart->Rx_TimeOutMax = 350000/baudrate;
    }

	USART_DMACmd( puart->USARTx, USART_DMAReq_Rx, DISABLE );
    DMA_Cmd( puart->DMA_Channel_tx, DISABLE );
    DMA_Cmd( puart->DMA_Channel_rx, DISABLE );

    UARTs_CfgInit( puart, baudrate, puart->UsbComCfg.bStopBit, puart->UsbComCfg.bParity );
    UARTs_DMAInit( puart, UART_DMA_TX, puart->UART_Tx_Buf, 0 );
    UARTs_DMAInit( puart, UART_DMA_RX, puart->UART_Rx_Buf, DEF_UARTx_RX_BUF_LEN );

    USART_DMACmd( puart->USARTx, USART_DMAReq_Rx, ENABLE );

    UARTs_ParaInit(puart);

}

/*********************************************************************
 * @fn      UART2_DataTx_Deal
 *
 * @brief   Uart2 data transmission processing (USB read)
 *
 * @return  none
 */
void Task_Read_from_USB(UART_CTL * puart) {

    if( puart->reinit ) {
    	puart->reinit = 0;
    	UARTs_USB_Init(puart);
    	// debug_printf("UARTs_USB_reInit\n");
    	return;
    }
    if(puart->Rx_RemainNum < DEF_UARTx_RX_BUF_NUM_MAX - 1) { // §Þ§Ö§ã§ä§à §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§Ó§Ñ§Ö§Þ§í§ç §Õ§Ñ§ß§ß§í§ç §Ö§ã§ä§î

//        NVIC_DisableIRQ( USBHS_IRQn );
        NVIC_DisableIRQ( USBHS_IRQn );

        if(puart->USB_Recv_StopFlag) { // §ã§ä§Ñ§â§ä§à§Ó§Ñ§ä§î §á§â§Ú§Ö§Þ §Ó USB
#if DUAL_UART
            //USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
            USBHSD_UEP_RX_CTRL(puart->USB_rx_ep) &= ~USBHS_UEP_R_RES_MASK;
            //USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
            USBHSD_UEP_RX_CTRL(puart->USB_rx_ep) |= USBHS_UEP_R_RES_ACK;
#else
            USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
            USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_ACK;
#endif
            puart->USB_Recv_StopFlag = 0; // §ã§ß§ñ§ä§î §æ§Ý§Ñ§Ô §à§ã§ä§Ñ§ß§à§Ó§Ü§Ú §á§â§Ú§Ö§Þ§Ñ §Ó USB
            //debug_printf("Rx enable %d, %d, %d\n", puart->Rx_LoadNum, puart->Tx_DealNum, puart->Tx_RemainNum);
        }
        if(puart->Tx_RemainNum) { // §Ö§ã§ä§î §ß§Ö§à§Ò§â§Ñ§Ò§à§ä§Ñ§ß§ß§í§Ö §Ò§Ý§à§Ü§Ú §ã §á§â§Ú§ß§ñ§ä§í§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú §á§à USB
#ifdef GPIO_PIN_LED
            GPIO_LED_Toggle();
#endif
            puart->Tx_RemainNum = 0; // §à§Ò§â§Ñ§Ò§à§ä§Ñ§Ý§Ú §Ó§ã§Ö §á§â§Ú§ß§ñ§ä§í§Ö §Ò§Ý§à§Ü§Ú
            puart->Tx_DealNum = 0;
        }
            // §à§Ò§â§Ñ§Ò§Ñ§ä§í§Ó§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö (§Õ§Ý§ñ §ä§Ö§ã§ä§Ñ §á§â§à§ã§ä§à §Ü§à§á§Ú§â§å§Ö§Þ §Ó §Ò§å§æ§Ö§â §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §ß§Ñ USB)
        if(adc_count) {
            adc_count--;
                //debug_printf("pkt[%d] %d, %d, %d\n", packlen, puart->Rx_LoadNum, puart->Tx_DealNum, puart->Tx_RemainNum);
             memcpy(puart->UART_Rx_Buf + puart->Rx_LoadNum * DEF_USB_PACK_LEN,
                     &msg_adc, sizeof(msg_adc));
            puart->Rx_PackLen[puart->Rx_LoadNum] = sizeof(msg_adc);

            // §å§ã§ä§Ñ§ß§à§Ó§Ú§ä§î §ß§à§Þ§Ö§â §ã§Ý§Ö§Õ§å§ð§ë§Ö§Ô§à §Ò§å§æ§Ö§â§Ñ §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Ó USB
            if(puart->Rx_LoadNum >= DEF_UARTx_RX_BUF_NUM_MAX - 1) {
                puart->Rx_LoadNum = 0;
            } else {
                puart->Rx_LoadNum++;
            }
            puart->Rx_RemainNum++; // §Õ§à§Ò§Ñ§Ó§Ú§Ý§Ú §Ò§Ý§à§Ü §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú

        } // §ß§Ö§ä §á§â§Ú§ß§ñ§ä§í§ç §Õ§Ñ§ß§ß§í§ç

        NVIC_EnableIRQ( USBHS_IRQn );

    } // §Ò§å§æ§Ö§â§Ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Ù§Ñ§ß§ñ§ä§í
}

void Task_Send_to_USB(UART_CTL * puart) {
   uint32_t packptr;
   uint32_t packlen;

   // §á§Ö§â§Ö§Õ§Ñ§é§Ñ §Õ§Ñ§ß§ß§í§ç §Ó USB
   if(puart->USB_Send_Flag == 0) { // §Ò§Ý§à§Ü §á§Ö§â§Ö§Õ§Ñ§ß
       if(puart->Rx_RemainNum) { // §Ö§ã§ä§î §Õ§Ñ§ß§ß§í§Ö §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §á§à USB

           packptr = puart->Rx_DealNum * DEF_USB_PACK_LEN;
           packlen = puart->Rx_PackLen[puart->Rx_DealNum];

           // §å§ã§ä§Ñ§ß§à§Ó§Ú§ä§î §ß§à§Þ§Ö§â §ã§Ý§Ö§Õ§å§ð§ë§Ö§Ô§à §Ò§å§æ§Ö§â§Ñ §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ñ§ß§ß§í§ç §Ó USB
           if(puart->Rx_DealNum >= DEF_UARTx_RX_BUF_NUM_MAX - 1) {
               puart->Rx_DealNum = 0;
           } else {
               puart->Rx_DealNum++;
           }
           //debug_printf("send[%d] %d, %d, %d\n", packlen, puart->Rx_LoadNum, puart->Tx_DealNum, puart->Tx_RemainNum);
           puart->Rx_RemainNum--; // §Þ§Ú§ß§å§ã §Ò§Ý§à§Ü §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú
           // §ã§ä§Ñ§â§ä §á§Ö§â§Ö§Õ§Ñ§é§Ú
           //debug_printf("snd[%d] at %d, %d, %d, %d\n", packlen, puart->TxCount, puart->Rx_LoadNum, puart->Tx_DealNum, puart->Tx_RemainNum);

//           NVIC_DisableIRQ( USBHS_IRQn );
           NVIC_DisableIRQ( USBHS_IRQn );

           if(packlen) {
               puart->TxCount += packlen;
               if(packlen >= USBHS_Dev.MaxPackLen) {
                   if(puart->TxCount >= CDC_MAX_FARME_SIZE) {
                       puart->TxCount = 0;
                       puart->USB_Send_Flag = 2;
                   } else {
                       // §ß§Ö §ã§Ò§â§Ñ§ã§í§Ó§Ñ§ä§î §ã§é§Ö§ä§é§Ú§Ü
                       puart->USB_Send_Flag = 1;
                   }
               } else {
                   puart->TxCount = 0;
                   puart->USB_Send_Flag = 1;
               }
#if DUAL_UART
               USBHSD_UEP_TX_DMA(puart->USB_tx_ep) = (uint32_t)(uint8_t *)&puart->UART_Rx_Buf[packptr];
               USBHSD_UEP_TX_LEN(puart->USB_tx_ep) = packlen;
               USBHSD_UEP_TX_CTRL(puart->USB_tx_ep) &= ~USBHS_UEP_T_RES_MASK;
               USBHSD_UEP_TX_CTRL(puart->USB_tx_ep) |= USBHS_UEP_T_RES_ACK;
#else
               USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t *)&puart->UART_Rx_Buf[packptr];
               USBHSD->UEP2_TX_LEN = packlen;
               USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
               USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
#endif
           } else {
               puart->TxCount = 0;
           }

           NVIC_EnableIRQ( USBHS_IRQn );

           // §æ§Ý§Ñ§Ô§Ú §ã§ä§Ñ§â§ä§Ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú
           //puart->USB_Send_Flag = 1; // §Ò§Ý§à§Ü §á§Ö§â§Ö§Õ§Ñ§Ö§ä§ã§ñ
           puart->USB_Send_TimeOut = 0; // §â§Ö§ã§ä§Ñ§â§ä§à§Ó§Ñ§ä§î §ã§é§Ö§ä§é§Ú§Ü Tx TimeOut
       }
   } else {
       // §Ú§Õ§Ö§ä §á§Ö§â§Ö§Õ§Ñ§é§Ñ §Ò§Ý§à§Ü§Ñ §Ó USB...
       if( puart->USB_Send_TimeOut >= DEF_UARTx_USB_UP_TIMEOUT) {
           debug_printf("Tout at %d, wtxblk %d, wrxblk %d, nbuftx %d\n", puart->RxCount, puart->Rx_RemainNum, puart->Tx_RemainNum, puart->Rx_DealNum);
           puart->USB_Send_TimeOut = 0; // §â§Ö§ã§ä§Ñ§â§ä§à§Ó§Ñ§ä§î §ã§é§Ö§ä§é§Ú§Ü Tx TimeOut

//           NVIC_DisableIRQ( USBHS_IRQn );
           NVIC_DisableIRQ( USBHS_IRQn );

           // TimeOut §á§Ö§â§Ö§Õ§Ñ§é§Ú - §é§ä§à §Õ§Ö§Ý§Ñ§ä§î?
#if DUAL_UART
           USBHSD_UEP_TX_LEN(puart->USB_tx_ep) = 0;
           USBHSD_UEP_TX_CTRL(puart->USB_tx_ep) &= ~USBHS_UEP_T_RES_MASK;
           USBHSD_UEP_TX_CTRL(puart->USB_tx_ep) |= USBHS_UEP_T_RES_ACK; // USBHS_UEP_R_RES_NAK; // ?
#else
           USBOTG_FS->UEP1_TX_LEN = 0;
           USBOTG_FS->UEP1_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
           USBOTG_FS->UEP1_TX_CTRL |= USBHS_UEP_T_RES_ACK; // USBHS_UEP_R_RES_NAK; // ?
#endif
           NVIC_EnableIRQ( USBHS_IRQn );
           // §æ§Ý§Ñ§Ô§Ú §à§Ü§à§ß§é§Ñ§ß§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú
           puart->USB_Send_Flag = 0; // §Ò§Ý§à§Ü §á§Ö§â§Ö§Õ§Ñ§ß
       }
   }
}

