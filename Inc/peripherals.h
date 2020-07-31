#ifndef __PERIPHERALS_H
#define __PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#define CONF_GET_ID(IDEnable) (IDEnable & 0x7f)
#define CONF_IS_ENABLED(IDEnable) ((IDEnable & 0x80) == 0x80)

typedef struct {
  uint8_t ChannelEnable;
  uint8_t SamplingTime;
} ADCChannelConfig;

typedef struct {
  uint8_t ChannelEnable;
} DACChannelConfig;

typedef struct {
  uint8_t IDEnable;
  uint8_t Oversampling;
  uint16_t Prescaler;
  uint16_t Period;
  ADCChannelConfig ChannelsConfig[1];
} ADCConfiguration;

typedef struct {
  uint8_t IDEnable;
  uint16_t Prescaler;
  uint16_t Period;
  DACChannelConfig ChannelsConfig[1];
} DACConfiguration;

typedef struct {
  uint8_t IDEnable;
  uint8_t Gain;
  uint8_t InvertingInput;
} OPAMPConfiguration;

void DMA_Init(void);
void DMA_DeInit(void);
void ADC_Init(ADC_HandleTypeDef*, ADCConfiguration*, uint32_t*, uint32_t);
void DAC_Init(DAC_HandleTypeDef*, DACConfiguration*, uint32_t*, uint32_t);
void OPAMP_Init(OPAMP_HandleTypeDef*, OPAMPConfiguration*);
void Timer_Init(TIM_HandleTypeDef*, uint16_t, uint16_t);

#ifdef __cplusplus
}
#endif

#endif