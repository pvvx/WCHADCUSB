/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbhs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2023/11/20
* Description        : This file provides all the USBHS firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_usbhs_device.h"

/*****************************************************************************
 * UART1 -> USB-RX: EP2_RX UART1_Tx_Buf, USB-TX: EP2_TX UART1_Rx_Buf, CNTRL: EP1
 * UART2 -> USB-RX: EP4_RX UART2_Tx_Buf, USB-TX: EP4_TX UART1_Rx_Buf, CNTRL: EP3
 ******************************************************************************/

/******************************************************************************/
/* Variable Definition */
/* test mode */
__attribute__ ((aligned(4))) uint8_t IFTest_Buf[ 53 ] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
    0xFE,//26
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,//37
    0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD,//44
    0xFC, 0x7E, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0x7E//53
};

const char *stop_name[] = { "1", "1.5", "2" };
const char *parity_name[] = { "N", "O", "E", "M", "S" };

/* Global */

/* Setup Request */
USBHS_Setup_t USBHS_SetupReq;

/* USB Device Status */
USBHS_Dev_t USBHS_Dev = {
        .MaxPackLen = DEF_USBD_HS_PACK_SIZE
};

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBHS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBHS_EP1_Rx_Buf[ DEF_USB_EP2_FS_SIZE ];

#if USE_USBHS_Endp_DataUp
/* Endpoint tx busy flag */
volatile uint8_t  USBHS_Endp_Busy[ DEF_UEP_NUM ];
#endif

/******************************************************************************/
/* Interrupt Service Routine Declaration*/
void USBHS_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************/
__attribute__((weak))
void usbd_cdc_acm_set_line_coding(uint8_t intf, USB_COM_CONFIG * pline_coding)
{
    PUART_CTL puart;
#if DUAL_UART
    if (intf == usb_interface_cdc_1)
            puart = &Uart2;
    else if(intf == usb_interface_cdc_0)
#endif
        puart = &Uart1;
    memcpy(&puart->UsbComCfg, pline_coding, sizeof(USB_COM_CONFIG));
/*
    debug_printf("Set uart%d linecoding <%d %d %s %s>\r\n",
                        (intf>>1) + 1,
                        pline_coding->bDataBit,
                        pline_coding->bStopBit,
                        parity_name[pline_coding->bParity],
                        stop_name[pline_coding->bStopBit]);
*/
}
__attribute__((weak))
void usbd_cdc_acm_set_dtr_rts(uint8_t intf, uint8_t dtr_rts)
{
    PUART_CTL puart;
#if DUAL_UART
    if (intf == usb_interface_cdc_1)
            puart = &Uart2;
    else if(intf == usb_interface_cdc_0)
#endif
        puart = &Uart1;
    puart->rts_dtr = dtr_rts;
/*
    debug_printf("Set uart%d DTR %d, RTS %d\r\n",
            (intf>>1) + 1,
            dtr_rts & 1,
            (dtr_rts >> 1) & 1); */
}
/*********************************************************************/
#if USB_SET_SERIAL
static void UsbSetSerialNumber(void)
{
    int i;
    uint32_t sn = *( uint32_t * )0x1FFFF7E8; // UID Register (ESIG_UNIID1)
    for(i = 20; i >= 6; i -= 2) {
        UsbSerNumInfo[i] = bin2hex(sn);
        sn >>= 4;
    }
}
#endif
/*********************************************************************
 * @fn      USB_TestMode_Deal
 *
 * @brief   Eye Diagram Test Function Processing.
 *
 * @return  none
 *
 */
void USB_TestMode_Deal( void )
{
    /* start test */
    USBHS_Dev.Test_Flag &= ~0x80;
    if( USBHS_SetupReq.wIndex == 0x0100 )
    {
        /* Test_J */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_J;
    }
    else if( USBHS_SetupReq.wIndex == 0x0200 )
    {
        /* Test_K */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_K;
    }
    else if( USBHS_SetupReq.wIndex == 0x0300 )
    {
        /* Test_SE0_NAK */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_SE0;
    }
    else if( USBHS_SetupReq.wIndex == 0x0400 )
    {
        /* Test_Packet */
        USBHSD->SUSPEND &= ~TEST_MASK;
        USBHSD->SUSPEND |= TEST_PACKET;

        USBHSD->CONTROL |= USBHS_UC_HOST_MODE;
        USBHSH->HOST_EP_CONFIG = USBHS_UH_EP_TX_EN | USBHS_UH_EP_RX_EN;
        USBHSH->HOST_EP_TYPE |= 0xff;

        USBHSH->HOST_TX_DMA = (uint32_t)(&IFTest_Buf[ 0 ]);
        USBHSH->HOST_TX_LEN = 53;
        USBHSH->HOST_EP_PID = ( USB_PID_SETUP << 4 );
        USBHSH->INT_FG = USBHS_UIF_TRANSFER;
    }
}

/*********************************************************************
 * @fn      USBHS_RCC_Init
 *
 * @brief   Initializes the clock for USB2.0 High speed device.
 *
 * @return  none
 */
void USBHS_RCC_Init( void )
{
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
#if (HSE_VALUE == 8000000UL)
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
#elif (HSE_VALUE == 12000000UL)
    RCC_USBHSConfig( RCC_USBPLL_Div3 );
#endif
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
}

/*********************************************************************
 * @fn      USBHS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 */
void USBHS_Device_Endp_Init ( void )
{

    USBHSD->ENDP_CONFIG =   USBHS_UEP1_T_EN | USBHS_UEP1_R_EN
                          | USBHS_UEP2_T_EN | USBHS_UEP2_R_EN
#if DUAL_UART
                          | USBHS_UEP3_T_EN | USBHS_UEP3_R_EN
                          | USBHS_UEP4_T_EN | USBHS_UEP4_R_EN
#endif
                          ;

    USBHSD->UEP0_MAX_LEN  = DEF_USBD_UEP0_SIZE;
    USBHSD->UEP1_MAX_LEN  = DEF_USB_EP1_FS_SIZE;
    USBHSD->UEP2_MAX_LEN  = DEF_USB_EP2_HS_SIZE;
#if DUAL_UART
    USBHSD->UEP3_MAX_LEN  = DEF_USB_EP5_FS_SIZE;
    USBHSD->UEP4_MAX_LEN  = DEF_USB_EP4_HS_SIZE;
#endif

    USBHSD->UEP0_DMA    = (uint32_t)(uint8_t *)USBHS_EP0_Buf;

    USBHSD->UEP1_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP1_Rx_Buf;
    USBHSD->UEP2_RX_DMA = (uint32_t)(uint8_t *)UART1_Tx_Buf;
    USBHSD->UEP2_TX_DMA = (uint32_t)(uint8_t *)UART1_Rx_Buf;

    USBHSD->UEP0_TX_LEN  = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP1_TX_LEN  = 0;
    USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP2_TX_LEN  = 0;
    USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;

#if DUAL_UART
//    USBHSD->UEP3_RX_DMA = (uint32_t)(uint8_t *)USBHS_EP3_Rx_Buf;
    USBHSD->UEP4_RX_DMA = (uint32_t)(uint8_t *)UART2_Tx_Buf;
    USBHSD->UEP4_TX_DMA = (uint32_t)(uint8_t *)UART2_Rx_Buf;

    USBHSD->UEP3_TX_LEN  = 0;
    USBHSD->UEP3_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK;

    USBHSD->UEP4_TX_LEN  = 0;
    USBHSD->UEP4_TX_CTRL = USBHS_UEP_T_RES_NAK;
    USBHSD->UEP4_RX_CTRL = USBHS_UEP_R_RES_ACK;
#endif

#if USE_USBHS_Endp_DataUp
    /* Clear End-points Busy Status */
    for(uint8_t i=0; i < DEF_UEP_NUM; i++ )
    {
        USBHS_Endp_Busy[ i ] = 0;
    }
#endif
}

/*********************************************************************
 * @fn      USBHS_Device_Init
 *
 * @brief   Initializes USB high-speed device.
 *
 * @return  none
 */
void USBHS_Device_Init ( FunctionalState sta )
{
    if( sta )
    {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us(10);
#if USB_SET_SERIAL
        UsbSetSerialNumber();
#endif
        USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
        USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
        USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY | USBHS_UC_SPEED_HIGH;
        USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER | USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;
        USBHS_Device_Endp_Init( );
        USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
        NVIC_EnableIRQ( USBHS_IRQn );
    }
    else
    {
        USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
        Delay_Us(10);
        USBHSD->CONTROL = 0;
        NVIC_DisableIRQ( USBHS_IRQn );
    }
}

#if USE_USBHS_Endp_DataUp
/*********************************************************************
 * @fn      USBHS_Endp_DataUp
 *
 * @brief   usbhs device data upload
 *          input: endp  - end-point numbers
 *                 *pubf - data buffer
 *                 len   - load data length
 *                 mod   - 0: DEF_UEP_DMA_LOAD 1: DEF_UEP_CPY_LOAD
 *
 * @return  none
 */
uint8_t USBHS_Endp_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod )
{
    uint8_t endp_buf_mode, endp_en, endp_tx_ctrl;

    /* DMA config, endp_ctrl config, endp_len config */
    if( (endp>=DEF_UEP1) && (endp<=DEF_UEP15) )
    {
        endp_en =  USBHSD->ENDP_CONFIG;
        if( endp_en & USBHSD_UEP_TX_EN( endp ) )
        {
            if( (USBHS_Endp_Busy[ endp ] & DEF_UEP_BUSY) == 0x00 )
            {
                endp_buf_mode = USBHSD->BUF_MODE;
                /* if end-point buffer mode is double buffer */
                if( endp_buf_mode & USBHSD_UEP_DOUBLE_BUF( endp ) )
                {
                    /* end-point buffer mode is double buffer */
                    /* only end-point tx enable  */
                    if( (endp_en & USBHSD_UEP_RX_EN( endp )) == 0x00 )
                    {
                        endp_tx_ctrl = USBHSD_UEP_TXCTRL( endp );
                        if( mod == DEF_UEP_DMA_LOAD )
                        {
                            if( (endp_tx_ctrl & USBHS_UEP_T_TOG_DATA1) ==  0 )
                            {
                                /* use UEPn_TX_DMA */
                                USBHSD_UEP_TXDMA( endp ) = (uint32_t)pbuf;
                            }
                            else
                            {
                                /* use UEPn_RX_DMA */
                                USBHSD_UEP_RXDMA( endp ) = (uint32_t)pbuf;
                            }
                        }
                        else if( mod == DEF_UEP_CPY_LOAD )
                        {
                            if( (endp_tx_ctrl & USBHS_UEP_T_TOG_DATA1) ==  0 )
                            {
                                /* use UEPn_TX_DMA */
                                memcpy( USBHSD_UEP_TXBUF(endp), pbuf, len );
                            }
                            else
                            {
                                /* use UEPn_RX_DMA */
                                memcpy( USBHSD_UEP_RXBUF(endp), pbuf, len );
                            }
                        }
                        else
                        {
                            return 1;
                        }
                    }
                    else
                    {
                        return 1;
                    }
                }
                else
                {
                    /* end-point buffer mode is single buffer */
                    if( mod == DEF_UEP_DMA_LOAD )
                    {

                        USBHSD_UEP_TXDMA( endp ) = (uint32_t)pbuf;
                    }
                    else if( mod == DEF_UEP_CPY_LOAD )
                    {
                        memcpy( USBHSD_UEP_TXBUF(endp), pbuf, len );
                    }
                    else
                    {
                        return 1;
                    }
                }

                /* end-point n response tx ack */
                USBHSD_UEP_TLEN( endp ) = len;
                USBHSD_UEP_TXCTRL( endp ) = (USBHSD_UEP_TXCTRL( endp ) &= ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                /* Set end-point busy */
                USBHS_Endp_Busy[ endp ] |= DEF_UEP_BUSY;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }

    return 0;
}
#endif

/* Set usb configuration descriptor by HS speed */
static USB_CFG_DESCRS * GetCfgDescr(uint8_t type, uint16_t ep_size) {
    uint8_t lo = (uint8_t)ep_size;
    uint8_t hi = (uint8_t)(ep_size >> 8);

    UsbCfgDescr.config.bDescriptorType = type;

    UsbCfgDescr.eprx0.wMaxPacketSizeL = lo;
    UsbCfgDescr.eprx0.wMaxPacketSizeH = hi;
    UsbCfgDescr.eptx0.wMaxPacketSizeL = lo;
    UsbCfgDescr.eptx0.wMaxPacketSizeH = hi;
#if DUAL_UART
    UsbCfgDescr.eprx1.wMaxPacketSizeL = lo;
    UsbCfgDescr.eprx1.wMaxPacketSizeH = hi;
    UsbCfgDescr.eptx1.wMaxPacketSizeL = lo;
    UsbCfgDescr.eptx1.wMaxPacketSizeH = hi;
#endif
    return &UsbCfgDescr;
}
/*********************************************************************
 * @fn      USBHS_IRQHandler
 *
 * @brief   This function handles USBHS exception.
 *
 * @return  none
 */
void USBHS_IRQHandler( void )
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBHSD->INT_FG;
    intst = USBHSD->INT_ST;

    if( intflag & USBHS_UIF_TRANSFER )
    {
        switch ( intst & USBHS_UIS_TOKEN_MASK )
        {
            /* data-in stage processing */
            case USBHS_UIS_TOKEN_IN:
                switch ( intst & ( USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK ) )
                {
                    /* end-point 0 data in interrupt */
                    case USBHS_UIS_TOKEN_IN | DEF_UEP0:
                        if( USBHS_SetupReq.wLength == 0 )
                        {
                            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                        }
                        if ( ( USBHS_SetupReq.bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */
                        }
                        else
                        {
                            /* Standard request endpoint 0 Data upload */
                            switch( USBHS_SetupReq.bRequest )
                            {
                                case USB_GET_DESCRIPTOR:
                                    len = USBHS_SetupReq.wLength >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBHS_SetupReq.wLength;
                                    memcpy(USBHS_EP0_Buf, USBHS_Dev.pDescr, len);
                                    USBHS_SetupReq.wLength -= len;
                                    USBHS_Dev.pDescr += len;
                                    USBHSD->UEP0_TX_LEN = len;
                                    USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                                    break;

                                case USB_SET_ADDRESS:
                                    USBHSD->DEV_AD = USBHS_Dev.Addr;
                                    break;

                                default:
                                    USBHSD->UEP0_TX_LEN = 0;
                                    break;
                            }
                        }

                        /* test mode */
                        if( USBHS_Dev.Test_Flag & 0x80 )
                        {
                            USB_TestMode_Deal( );
                        }
                        break;

                    /* end-point 2 data in interrupt */
                    case USBHS_UIS_TOKEN_IN | DEF_UEP2:
                        USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                        USBHSD->UEP2_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
#if USE_USBHS_Endp_DataUp
                        USBHS_Endp_Busy[ DEF_UEP2 ] &= ~DEF_UEP_BUSY;
#endif
                        if(Uart1.USB_Send_Flag == 2) {
                            //debug_printf("->\r\n");
                            USBHSD->UEP2_TX_LEN = 0;
                            USBHSD->UEP2_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                            USBHSD->UEP2_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                            Uart1.USB_Send_Flag = 1;
                        } else
                            Uart1.USB_Send_Flag = 0;
                        break;
#if DUAL_UART
                    /* end-point 4 data in interrupt */
                    case ( USBFS_UIS_TOKEN_IN | DEF_UEP4 ):
                        USBHSD->UEP4_TX_CTRL = (USBHSD->UEP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                        USBHSD->UEP4_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
#if USE_USBHS_Endp_DataUp
                        USBHS_Endp_Busy[ DEF_UEP4 ] &= ~DEF_UEP_BUSY;
#endif
                        if(Uart1.USB_Send_Flag == 2) {
                            //debug_printf("->\r\n");
                            USBHSD->UEP4_TX_LEN = 0;
                            USBHSD->UEP4_TX_CTRL &= ~USBHS_UEP_T_RES_MASK;
                            USBHSD->UEP4_TX_CTRL |= USBHS_UEP_T_RES_ACK;
                            Uart2.USB_Send_Flag = 1;
                        } else
                            Uart2.USB_Send_Flag = 0;
                        break;
#endif
                    /* end-point 1 data in interrupt */
                    case USBHS_UIS_TOKEN_IN | DEF_UEP1:
                        USBHSD->UEP1_TX_CTRL = (USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                        USBHSD->UEP1_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
#if USE_USBHS_Endp_DataUp
                        USBHS_Endp_Busy[ DEF_UEP1 ] &= ~DEF_UEP_BUSY;
#endif
                        break;
#if DUAL_UART
                    /* end-point 3 data in interrupt */
                    case USBHS_UIS_TOKEN_IN | DEF_UEP3:
                        USBHSD->UEP3_TX_CTRL = (USBHSD->UEP3_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
                        USBHSD->UEP3_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
#if USE_USBHS_Endp_DataUp
                        USBHS_Endp_Busy[ DEF_UEP3 ] &= ~DEF_UEP_BUSY;
#endif
                        break;
#endif

                    default :
                        break;
                }
                break;

            /* data-out stage processing */
            case USBHS_UIS_TOKEN_OUT:
                switch( intst & ( USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK ) )
                {
                    /* end-point 0 data out interrupt */
                    case USBHS_UIS_TOKEN_OUT | DEF_UEP0:
                         len = USBHSH->RX_LEN;
                         if ( intst & USBHS_UIS_TOG_OK )
                         {
                             /* if any processing about rx, set it here */
                             if ( ( USBHS_SetupReq.bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                             {
                                 USBHS_SetupReq.wLength = 0;
                                 /* Non-standard request end-point 0 Data download */
                                 if( USBHS_SetupReq.bRequest == CDC_SET_LINE_CODING )
                                 {
#if DUAL_UART
                                    if(USBHS_SetupReq.wIndex == usb_interface_cdc_1)
                                    {
                                        memcpy(&Uart2.UsbComCfg, USBHS_EP0_Buf, sizeof(USB_COM_CONFIG));
                                        Uart2.reinit = 1; // UARTs_USB_Init(&Uart2 );
                                    }
                                    else
                                        if(USBHS_SetupReq.wIndex == usb_interface_cdc_0)
#endif
                                    {
                                        memcpy(&Uart1.UsbComCfg, USBHS_EP0_Buf, sizeof(USB_COM_CONFIG));
                                        Uart1.reinit = 1; // UARTs_USB_Init(&Uart1 );

                                    }
                                 }
                             }
                             else
                             {
                                 /* Standard request end-point 0 Data download */
                             }

                             if( USBHS_SetupReq.wLength == 0 )
                             {
                                 USBHSD->UEP0_TX_LEN  = 0;
                                 USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                             }
                         }
                            break;

                    /* end-point 2 data out interrupt */
                    case USBHS_UIS_TOKEN_OUT | DEF_UEP2:
                        /* Endp download */
                        USBHSD->UEP2_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                        // §á§à USB §á§â§Ú§ß§ñ§Ý§Ú RX_LEN
                        /* DMA address */
                        Uart1.Tx_PackLen[ Uart1.Tx_LoadNum ] = USBHSD->RX_LEN;

                        if( Uart1.Tx_LoadNum >= DEF_UARTx_TX_BUF_NUM_MAX - 1 )  {
                            Uart1.Tx_LoadNum = 0; // §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§Ö§Þ §ã §ß§Ñ§é§Ñ§Ý§Ñ §Ò§å§æ§Ö§â§Ñ
                        } else
                            Uart1.Tx_LoadNum++; // §å§Ü§Ñ§Ù§Ñ§ä§Ö§Ý§î §á§â§Ú§Ö§Þ§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã USB

                        // §å§ã§Ñ§ß§à§Ó§Ú§ä§î §ß§à§Ó§í§Û §Ñ§Õ§â§Ö§ã §Ò§å§æ§Ö§â§Ñ §á§â§Ú§Ö§Þ§Ñ §ã USB
                        USBHSD->UEP2_RX_DMA = (uint32_t)Uart1.UART_Tx_Buf + ( Uart1.Tx_LoadNum * DEF_USB_PACK_LEN );

                        Uart1.Tx_RemainNum++; // §Ü§à§Ý-§Ó§à §ß§Ö§à§Ò§â§Ñ§Ò§à§ä§Ñ§ß§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã §á§â§Ú§ß§ñ§ä§í§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú §á§à USB

                        // §Ò§å§æ§Ö§â §Ù§Ñ§Ò§Ú§ä? §à§Ò§â§Ñ§Ò§à§ä§Ü§Ñ §Ó§ã§ä§Ñ§Ý§Ñ?
                        if( Uart1.Tx_RemainNum >= DEF_UARTx_TX_BUF_NUM_MAX) // ( DEF_UARTx_TX_BUF_NUM_MAX - 2 ) )
                        {
                            // §Ò§å§æ§Ö§â §Ù§Ñ§Ò§Ú§ä -> §àc§ä§Ñ§ß§à§Ó§Ú§Þ §á§â§Ú§Ö§Þ §Ó USB.EP2
                            USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
                            USBHSD->UEP2_RX_CTRL |= USBHS_UEP_R_RES_NAK;
                            Uart1.USB_Recv_StopFlag = 1; // §á§â§Ú§Ö§Þ §Ó USB §à§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß
                        }
                        break;
#if DUAL_UART
                    /* end-point 2 data out interrupt */
                    case USBHS_UIS_TOKEN_OUT | DEF_UEP4:
                        /* Endp download */
                        USBHSD->UEP4_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
                        // §á§à USB §á§â§Ú§ß§ñ§Ý§Ú RX_LEN
                        /* DMA address */
                        Uart2.Tx_PackLen[ Uart2.Tx_LoadNum ] = USBHSD->RX_LEN;

                        if( Uart2.Tx_LoadNum >= DEF_UARTx_TX_BUF_NUM_MAX - 1 )  {
                            Uart2.Tx_LoadNum = 0; // §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§Ö§Þ §ã §ß§Ñ§é§Ñ§Ý§Ñ §Ò§å§æ§Ö§â§Ñ
                        } else
                            Uart2.Tx_LoadNum++; // §å§Ü§Ñ§Ù§Ñ§ä§Ö§Ý§î §á§â§Ú§Ö§Þ§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã USB

                        // §å§ã§Ñ§ß§à§Ó§Ú§ä§î §ß§à§Ó§í§Û §Ñ§Õ§â§Ö§ã §Ò§å§æ§Ö§â§Ñ §á§â§Ú§Ö§Þ§Ñ §ã USB
                        USBHSD->UEP4_RX_DMA = (uint32_t)Uart2.UART_Tx_Buf + ( Uart2.Tx_LoadNum * DEF_USB_PACK_LEN );

                        Uart2.Tx_RemainNum++; // §Ü§à§Ý-§Ó§à §ß§Ö§à§Ò§â§Ñ§Ò§à§ä§Ñ§ß§ß§í§ç §Ò§Ý§à§Ü§à§Ó §ã §á§â§Ú§ß§ñ§ä§í§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú §á§à USB

                        // §Ò§å§æ§Ö§â §Ù§Ñ§Ò§Ú§ä? §à§Ò§â§Ñ§Ò§à§ä§Ü§Ñ §Ó§ã§ä§Ñ§Ý§Ñ?
                        if( Uart2.Tx_RemainNum >= DEF_UARTx_TX_BUF_NUM_MAX) // ( DEF_UARTx_TX_BUF_NUM_MAX - 2 ) )
                        {
                            // §Ò§å§æ§Ö§â §Ù§Ñ§Ò§Ú§ä -> §àc§ä§Ñ§ß§à§Ó§Ú§Þ §á§â§Ú§Ö§Þ §Ó USB.EP2
                            USBHSD->UEP4_RX_CTRL &= ~USBHS_UEP_R_RES_MASK;
                            USBHSD->UEP4_RX_CTRL |= USBHS_UEP_R_RES_NAK;
                            Uart2.USB_Recv_StopFlag = 1; // §á§â§Ú§Ö§Þ §Ó USB §à§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß
                        }
                        break;
#endif
                    default:
                        errflag = 0xFF;
                        break;
                }
                break;

            /* Sof pack processing */
            case USBHS_UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        USBHSD->INT_FG = USBHS_UIF_TRANSFER;
    }
    else if( intflag & USBHS_UIF_SETUP_ACT )
    {
        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

        /* Store All Setup Values */
        USBHS_SetupReq.bRequestType  = pUSBHS_SetupReqPak->bRequestType;
        USBHS_SetupReq.bRequest  = pUSBHS_SetupReqPak->bRequest;
        USBHS_SetupReq.wValue = pUSBHS_SetupReqPak->wValue;
        USBHS_SetupReq.wIndex = pUSBHS_SetupReqPak->wIndex;
        USBHS_SetupReq.wLength   = pUSBHS_SetupReqPak->wLength;
        //debug_printf("Setup: %02x, %02x, %02x, %02x, %u\n", USBHS_SetupReq.bRequestType, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue, USBHS_SetupReq.wIndex,  USBHS_SetupReq.wLength);

        len = 0;
        errflag = 0;
        if ( ( USBHS_SetupReq.bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
        {
            /* usb non-standard request processing */
            if( USBHS_SetupReq.bRequestType & USB_REQ_TYP_CLASS )
            {
                /* Class requests */
                switch( USBHS_SetupReq.bRequest )
                {
                    case CDC_GET_LINE_CODING:
                        // debug_printf("CDC_GET_LINE_CODING: %u, %u, %u\n", USBHS_SetupReq.wIndex, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue);
#if DUAL_UART
                        if(USBHS_SetupReq.wIndex == usb_interface_cdc_1) { // == 2
                            USBHS_Dev.pDescr = (uint8_t *)&Uart2.UsbComCfg;
                        } else if (USBHS_SetupReq.wIndex == usb_interface_cdc_0)
#endif
                        {
                            USBHS_Dev.pDescr = (uint8_t *)&Uart1.UsbComCfg;
/*
                            debug_printf("Uart1 send linecoding <%d %d %s %s>\r\n",
                                                Uart1.UsbComCfg.bDataBit,
                                                Uart1.UsbComCfg.bStopBit,
                                                parity_name[Uart1.UsbComCfg.bParity],
                                                stop_name[Uart1.UsbComCfg.bStopBit]); */
                        }
                        len = sizeof(USB_COM_CONFIG);
                        break;

                    case CDC_SET_LINE_CODING:
                        // debug_printf("SET_LINE_CODING: %u, %u, %u\n", USBHS_SetupReq.wIndex, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue);
                        usbd_cdc_acm_set_line_coding(USBHS_SetupReq.wIndex, (USB_COM_CONFIG *)USBHS_EP0_Buf);
                        break;

                    case CDC_SET_LINE_CTLSTE:
                        //debug_printf("SET_LINE_CTLSTE: %u, %u, %u\n", USBHS_SetupReq.wIndex, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue);
                        usbd_cdc_acm_set_dtr_rts(USBHS_SetupReq.wIndex, USBHS_SetupReq.wValue);
                        break;

                    case CDC_SEND_BREAK:
                        // debug_printf("SEND_BREAK: %u, %u, %u\n", USBHS_SetupReq.wIndex, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue);
                        break;

                    default:
                        errflag = 0xff;
                        break;
                }
            }
            else if( USBHS_SetupReq.bRequestType & USB_REQ_TYP_VENDOR )
            {
                /* Manufacturer request */
                // debug_printf("Manufacturer request: %u, %u, %u\n", USBHS_SetupReq.wIndex, USBHS_SetupReq.bRequest, USBHS_SetupReq.wValue);
            }
            else
            {
                errflag = 0xFF;
            }
            /* Copy Descriptors to Endp0 DMA buffer */
            len = (USBHS_SetupReq.wLength >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReq.wLength;
            memcpy( USBHS_EP0_Buf, USBHS_Dev.pDescr, len );
            USBHS_Dev.pDescr += len;
        }
        else
        {
            /* usb standard request processing */
            switch( USBHS_SetupReq.bRequest )
            {
                /* get device/configuration/string/report/... descriptors */
                case USB_GET_DESCRIPTOR:
                    switch( (uint8_t)(USBHS_SetupReq.wValue >> 8) )
                    {
                        /* get usb device descriptor */
                        case USB_DESCR_TYP_DEVICE:
                            USBHS_Dev.pDescr = (uint8_t  *)&UsbDevDescr;
                            len = sizeof(USB_DEV_DESCR); // UsbDevDescr.bLength;
                            break;

                        /* get usb configuration descriptor */
                        case USB_DESCR_TYP_CONFIG:
                            /* Query current usb speed */
                            if( ( USBHSD->SPEED_TYPE & USBHS_SPEED_TYPE_MASK ) == USBHS_SPEED_HIGH )
                            {
                                /* High speed mode */
                                USBHS_Dev.Speed = USBHS_SPEED_HIGH;
                                USBHS_Dev.MaxPackLen = DEF_USBD_HS_PACK_SIZE;
                            }
                            else
                            {
                                /* Full speed mode */
                                USBHS_Dev.Speed = USBHS_SPEED_FULL;
                                USBHS_Dev.MaxPackLen = DEF_USBD_FS_PACK_SIZE;
                            }
                            /* Load usb configuration descriptor by speed */
                            USBHS_Dev.pDescr = (uint8_t  *)GetCfgDescr(USB_DESCR_TYP_CONFIG, USBHS_Dev.MaxPackLen);
                            len = sizeof(USB_CFG_DESCRS);

                            break;

                        /* get usb string descriptor */
                        case USB_DESCR_TYP_STRING:
                            switch( (uint8_t)(USBHS_SetupReq.wValue & 0xFF) )
                            {
                                /* Descriptor 0, Language descriptor */
                                case DEF_STRING_DESC_LANG:
                                    USBHS_Dev.pDescr = UsbLangDescr;
                                    len = UsbLangDescr[0];
                                    break;

                                /* Descriptor 1, Manufacturers String descriptor */
                                case DEF_STRING_DESC_MANU:
                                    USBHS_Dev.pDescr = UsbManuInfo;
                                    len = UsbManuInfo[0];
                                    break;

                                /* Descriptor 2, Product String descriptor */
                                case DEF_STRING_DESC_PROD:
                                    USBHS_Dev.pDescr = UsbProdInfo;
                                    len = UsbProdInfo[0];
                                    break;

                                /* Descriptor 3, Serial-number String descriptor */
                                case DEF_STRING_DESC_SERN:
                                    USBHS_Dev.pDescr = UsbSerNumInfo;
                                    len = UsbSerNumInfo[0];
                                    break;

                                default:
                                    errflag = 0xFF;
                                    break;
                            }
                            break;

                        /* get usb device qualify descriptor */
                        case USB_DESCR_TYP_QUALIF:
                            USBHS_Dev.pDescr = UsbQuaDesc;
                            len = UsbQuaDesc[0];
                            break;

                        /* get usb BOS descriptor */
                        case USB_DESCR_TYP_BOS:
                            /* USB 2.00 DO NOT support BOS descriptor */
                            errflag = 0xFF;
                            break;

                        /* get usb other-speed descriptor */
                        case USB_DESCR_TYP_SPEED:
                            if (USBHS_Dev.Speed == USBHS_SPEED_HIGH) {
                                /* High speed mode */
                                USBHS_Dev.pDescr = (uint8_t *) GetCfgDescr(
                                        USB_DESCR_TYP_SPEED,
                                        DEF_USBD_FS_PACK_SIZE);
                            } else if (USBHS_Dev.Speed == USBHS_SPEED_FULL) {
                                /* Full speed mode */
                                USBHS_Dev.pDescr = (uint8_t *) GetCfgDescr(
                                        USB_DESCR_TYP_SPEED,
                                        DEF_USBD_HS_PACK_SIZE);
                            } else {
                                errflag = 0xFF;
                            }
                            len = sizeof(USB_CFG_DESCRS);
                            break;

                        default :
                            errflag = 0xFF;
                            break;
                    }

                    /* Copy Descriptors to Endp0 DMA buffer */
                    if( USBHS_SetupReq.wLength>len )
                    {
                        USBHS_SetupReq.wLength = len;
                    }
                    len = (USBHS_SetupReq.wLength >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReq.wLength;
                    memcpy( USBHS_EP0_Buf, USBHS_Dev.pDescr, len );
                    USBHS_Dev.pDescr += len;
                    break;

                /* Set usb address */
                case USB_SET_ADDRESS:
                    USBHS_Dev.Addr = (uint16_t)(USBHS_SetupReq.wValue & 0xFF);
                    break;

                /* Get usb configuration now set */
                case USB_GET_CONFIGURATION:
                    USBHS_EP0_Buf[0] = USBHS_Dev.Config;
                    if ( USBHS_SetupReq.wLength > 1 )
                    {
                        USBHS_SetupReq.wLength = 1;
                    }
                    break;

                /* Set usb configuration to use */
                case USB_SET_CONFIGURATION:
                    USBHS_Dev.Config = (uint8_t)(USBHS_SetupReq.wValue & 0xFF);
                    USBHS_Dev.EnumStatus = 0x01;
                    break;

                /* Clear or disable one usb feature */
                case USB_CLEAR_FEATURE:
                    if( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                    {
                        /* clear one device feature */
                        if((uint8_t)(USBHS_SetupReq.wValue & 0xFF) == 0x01)
                        {
                            /* clear usb sleep status, device not prepare to sleep */
                            USBHS_Dev.SleepStatus &= ~0x01;
                        }
                        else
                        {
                            errflag = 0xFF;
                        }
                    }
                    else if ( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                    {
                        /* Set End-point Feature */
                        if( (uint8_t)(USBHS_SetupReq.wValue&0xFF) == USB_REQ_FEAT_ENDP_HALT )
                        {
                            /* Clear End-point Feature */
                            switch( (uint8_t)(USBHS_SetupReq.wIndex & 0xFF) )
                            {
                                case (DEF_UEP2 | DEF_UEP_IN):
                                    /* Set End-point 2 IN NAK */
                                    USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK;
                                    //Uart1.usb_send_blk_ok = 1;
                                    break;

                                case (DEF_UEP2 | DEF_UEP_OUT):
                                    /* Set End-point 2 OUT ACK */
                                    USBHSD->UEP2_RX_CTRL = USBHS_UEP_R_RES_ACK;
                                    //Uart1.usb_send_blk_ok = 1;
                                    break;

                                case (DEF_UEP1 | DEF_UEP_IN):
                                    /* Set End-point 1 IN NAK */
                                    USBHSD->UEP1_TX_CTRL = USBHS_UEP_T_RES_NAK;
                                    break;

                                default:
                                    errflag = 0xFF;
                                    break;
                            }
                        }
                        else
                        {
                            errflag = 0xFF;
                        }

                    }
                    else
                    {
                        errflag = 0xFF;
                    }
                    break;

                /* set or enable one usb feature */
                case USB_SET_FEATURE:
                    if( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                    {
                        /* Set Device Feature */
                        if( (uint8_t)(USBHS_SetupReq.wValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP )
                        {
                            if( UsbCfgDescr.config.bmAttributes & 0x20 ) // USB_SLEEP_ENABLE ?
                            {
                                /* Set Wake-up flag, device prepare to sleep */
                                USBHS_Dev.SleepStatus |= 0x01;
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                        }
                        else if( (uint8_t)(USBHS_SetupReq.wValue & 0xFF) == 0x02 )
                        {
                            /* test mode deal */
                            if( ( USBHS_SetupReq.wIndex == 0x0100 ) ||
                                ( USBHS_SetupReq.wIndex == 0x0200 ) ||
                                ( USBHS_SetupReq.wIndex == 0x0300 ) ||
                                ( USBHS_SetupReq.wIndex == 0x0400 ) )
                            {
                                /* Set the flag and wait for the status to be uploaded before proceeding with the actual operation */
                                USBHS_Dev.Test_Flag |= 0x80;
                            }
                        }
                        else
                        {
                            errflag = 0xFF;
                        }
                    }
                    else if( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                    {
                        /* Set End-point Feature */
                        if( (uint8_t)(USBHS_SetupReq.wValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT )
                        {
                            /* Set end-points status stall */
                            switch((uint8_t)(USBHS_SetupReq.wIndex & 0xFF) )
                            {
                                case (DEF_UEP2 | DEF_UEP_IN):
                                    /* Set End-point 2 IN STALL */
                                    USBHSD->UEP2_TX_CTRL = ( USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                                    //Uart1.usb_send_blk_ok = 1;
                                    break;

                                case (DEF_UEP2 | DEF_UEP_OUT):
                                    /* Set End-point 2 OUT STALL */
                                    USBHSD->UEP2_RX_CTRL = ( USBHSD->UEP2_RX_CTRL & ~USBHS_UEP_R_RES_MASK ) | USBHS_UEP_R_RES_STALL;
                                    //Uart1.usb_send_blk_ok = 1;
                                    break;

                                case (DEF_UEP1 | DEF_UEP_IN):
                                    /* Set End-point 3 IN STALL */
                                    USBHSD->UEP1_TX_CTRL = ( USBHSD->UEP1_TX_CTRL & ~USBHS_UEP_T_RES_MASK ) | USBHS_UEP_T_RES_STALL;
                                    break;

                                default:
                                    errflag = 0xFF;
                                    break;
                            }
                        }
                    }
                    break;

                /* This request allows the host to select another setting for the specified interface  */
                case USB_GET_INTERFACE:
                    USBHS_EP0_Buf[0] = 0;
                    if ( USBHS_SetupReq.wLength > 1 ) {
                        USBHS_SetupReq.wLength = 1;
                    }
                    break;

                case USB_SET_INTERFACE:
                    break;

                /* host get status of specified device/interface/end-points */
                case USB_GET_STATUS:
                    USBHS_EP0_Buf[0] = 0;
                    USBHS_EP0_Buf[1] = 0;
                    if( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                    {
                        switch( (uint8_t)( USBHS_SetupReq.wIndex & 0xFF ) )
                        {
                            case (DEF_UEP2 | DEF_UEP_IN):
                                if( ( (USBHSD->UEP2_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;

                            case (DEF_UEP2 | DEF_UEP_OUT):
                                if( ( (USBHSD->UEP2_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;

                            case (DEF_UEP1 | DEF_UEP_IN):
                                if( ( (USBHSD->UEP1_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;
#if DUAL_UART
                            case (DEF_UEP4 | DEF_UEP_IN):
                                if( ( (USBHSD->UEP4_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;

                            case (DEF_UEP4 | DEF_UEP_OUT):
                                if( ( (USBHSD->UEP4_RX_CTRL) & USBHS_UEP_R_RES_MASK ) == USBHS_UEP_R_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;

                            case (DEF_UEP3 | DEF_UEP_IN):
                                if( ( (USBHSD->UEP3_TX_CTRL) & USBHS_UEP_T_RES_MASK ) == USBHS_UEP_T_RES_STALL )
                                {
                                    USBHS_EP0_Buf[ 0 ] = 1;
                                }
                                break;
#endif
                            default:
                                 errflag = 0xFF;
                                 break;
                        }
                    }
                    else if( ( USBHS_SetupReq.bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                    {
                          if( USBHS_Dev.SleepStatus & 0x01 )
                          {
                              USBHS_EP0_Buf[ 0 ] = 2;
                          }
                    }

                    if ( USBHS_SetupReq.wLength > 2 )
                    {
                        USBHS_SetupReq.wLength = 2;
                    }
                    break;

                default:
                    errflag = 0xFF;
                    break;
            }
        }

        /* errflag = 0xFF means a request not support or some errors occurred, else correct */
        if( errflag == 0xFF )
        {
            /* if one request not support, return stall */
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
        }
        else
        {
            /* end-point 0 data Tx/Rx */
            if( USBHS_SetupReq.bRequestType & DEF_UEP_IN )
            {
                /* tx */
                len = (USBHS_SetupReq.wLength > DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBHS_SetupReq.wLength;
                USBHS_SetupReq.wLength -= len;
                USBHSD->UEP0_TX_LEN = len;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
            }
            else
            {
                /* rx */
                if( USBHS_SetupReq.wLength == 0 )
                {
                    USBHSD->UEP0_TX_LEN = 0;
                    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                }
                else
                {
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }
            }
        }
        USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;
    }
    else if( intflag & USBHS_UIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBHS_Dev.Config = 0;
        USBHS_Dev.Addr = 0;
        USBHS_Dev.SleepStatus = 0;
        USBHS_Dev.EnumStatus = 0;

        USBHSD->DEV_AD = 0;
        USBHS_Device_Endp_Init( );
        Uart1.reinit = 1; // UARTs_USB_Init(&Uart1);
#if DUAL_UART
        Uart2.reinit = 1; // UARTs_USB_Init(&Uart2);
#endif
        //low_power = 0;
        USBHSD->INT_FG = USBHS_UIF_BUS_RST;
    }
    else if( intflag & USBHS_UIF_SUSPEND )
    {
        USBHSD->INT_FG = USBHS_UIF_SUSPEND;
        Delay_Us(10);
        /* usb suspend interrupt processing */
        if ( USBHSD->MIS_ST & USBHS_UMS_SUSPEND  )
        {
            USBHS_Dev.SleepStatus |= 0x02;
            if( USBHS_Dev.SleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBHS_Dev.SleepStatus &= ~0x02;
        }
    }
    else
    {
        /* other interrupts */
        USBHSD->INT_FG = intflag;
    }
}

