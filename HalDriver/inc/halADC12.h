#ifndef __HALADC12_H
#define __HALADC12_H

#include "stm32f10x.h"

#define ADC_NUMBER_PER_CH      (64)
#define ADC_CH_NUM             (4)

#define ADC_BUF_SIZE           (ADC_NUMBER_PER_CH*ADC_CH_NUM)

extern void halADC12Init(void);
extern uint16_t halADC1GetData(uint8_t ch); 

#endif
