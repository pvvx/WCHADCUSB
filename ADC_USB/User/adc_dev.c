/*
 * adc_dev.c
 *
 *  Created on: Jul 16, 2024
 *      Author: pvvx
 */


#include "UART.h"
#include "debug.h"
#include "adc_dev.h"

#define ADC_DMA_IRQ_ENABLE  1


/* Global Variable */
u16 TxBuf[ADC_BUF_SIZE];

msg_adc_t msg_adc;
volatile unsigned int adc_count;
s16 Calibrattion_Val = 0;


#if ADC_DMA_IRQ_ENABLE
/*********************************************************************
 * @fn      DMA1_Channel1_IRQHandler
 *
 * @brief   DMA1_Channel1_IRQHandler Interrupt Service Function.
 *          §£§í§Ù§í§Ó§Ñ§Ö§ä§ã§ñ 16 §â§Ñ§Ù §Ó §ã§Ö§Ü (§ã§Þ. timer1)
 *
 * @return  none
 */
void DMA1_Channel1_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler()
{
    if(DMA_GetITStatus(DMA1_IT_TC1)==SET) {
       memcpy(msg_adc.data, TxBuf, sizeof(msg_adc.data));
       adc_count++;
    }
    DMA_ClearITPendingBit(DMA1_IT_GL1);
}
#endif

/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};

    msg_adc.id = ADC_BUF_SIZE + (0x0A <<8);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 72/6 = 12 MHz

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left; // ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    Calibrattion_Val = Get_CalibrationValue(ADC1);

}

/*********************************************************************
 * @fn      DMA_Tx_Init
 *
 * @brief   Initializes the DMAy Channelx configuration.
 *
 * @param   DMA_CHx - x can be 1 to 7.
 *          ppadr - Peripheral base address.
 *          memadr - Memory base address.
 *          bufsize - DMA channel buffer size.
 *
 * @return  none
 */
void DMA_Tx_Init( DMA_Channel_TypeDef* DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure={0};
#if ADC_DMA_IRQ_ENABLE
    NVIC_InitTypeDef NVIC_InitStructure={0};
#endif

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);

#if ADC_DMA_IRQ_ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // DMA_IT_TC | DMA_IT_HT | DMA_IT_TE
#endif
}

void Init_ADC(void) {
    ADC_Function_Init();

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_13Cycles5);

    ADC_Cmd(ADC1, DISABLE);

    debug_printf("CalibrattionValue:%d\r\n", Calibrattion_Val);

}

void usbd_cdc_acm_set_dtr_rts(uint8_t intf, uint8_t dtr_rts)
{
    PUART_CTL puart = &Uart1;
    if((dtr_rts ^ puart->rts_dtr) & 1) {
        if(dtr_rts & 1) {
            debug_printf("Start ADC\r\n");
            ADC_Cmd(ADC1, ENABLE);
            DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)TxBuf, sizeof(TxBuf)/sizeof(u16));
            DMA_Cmd(DMA1_Channel1, ENABLE);
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        } else {
            debug_printf("Stop ADC\r\n");
            DMA_Cmd(DMA1_Channel1, DISABLE);
            ADC_SoftwareStartConvCmd(ADC1, DISABLE);
            ADC_Cmd(ADC1, DISABLE);
        }
    }
    Uart1.Rx_RemainNum = 0;
    Uart1.Rx_LoadNum = 0;
    adc_count = 0;
    puart->rts_dtr = dtr_rts;
/*
    debug_printf("Set uart%d DTR %d, RTS %d\r\n",
            (intf>>1) + 1,
            dtr_rts & 1,
            (dtr_rts >> 1) & 1); */
}
