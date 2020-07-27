#ifndef __PERIPHERALS_H
#define __PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

void DMA_Init(void);
void DMA_DeInit(void);
void ADC_Init(ADC_HandleTypeDef*, uint32_t*, uint32_t);
void DAC_Init(DAC_HandleTypeDef*, uint32_t*, uint32_t);
void OPAMP_Init(OPAMP_HandleTypeDef*);
void Timer_Init(TIM_HandleTypeDef*);

#ifdef __cplusplus
}
#endif

#endif