/*
 * adc_dev.h
 *
 *  Created on: Jul 16, 2024
 *      Author: pvvx
 */

#ifndef _USER_ADC_DEV_H_
#define _USER_ADC_DEV_H_

#define ADC_BUF_SIZE  255

typedef struct {
    uint16_t id;
    uint16_t data[ADC_BUF_SIZE];
} msg_adc_t;

extern msg_adc_t msg_adc;
extern volatile unsigned int adc_count;

void Init_ADC(void);



#endif /* _USER_ADC_DEV_H_ */
