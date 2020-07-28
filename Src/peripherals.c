#include "peripherals.h"

void Error_Handler(void);

void ADC_Init(ADC_HandleTypeDef *hadc, uint32_t* Buf, uint32_t ElementCount) {
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.NbrOfDiscConversion = 1;
  hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc->Init.DMAContinuousRequests = ENABLE;
  hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc->Init.OversamplingMode = ENABLE;
  hadc->Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc->Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc->Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(hadc) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(hadc, &multimode) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(hadc, Buf, ElementCount)) Error_Handler();
}

void DAC_Init(DAC_HandleTypeDef* hdac, uint32_t* Buf, uint32_t ElementCount) {
  DAC_ChannelConfTypeDef sConfig = {0};

  if (HAL_DAC_Init(hdac) != HAL_OK) {
    Error_Handler();
  }

  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, Buf, ElementCount, DAC_ALIGN_12B_R) != HAL_OK) {
    Error_Handler();
  }
}

void OPAMP_Init(OPAMP_HandleTypeDef* hopamp) {
  hopamp->Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp->Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (HAL_OPAMP_Init(hopamp) != HAL_OK) {
    Error_Handler();
  }

  HAL_OPAMP_SelfCalibrate(hopamp);
  HAL_OPAMP_Start(hopamp);
}

void Timer_Init(TIM_HandleTypeDef *htim) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 799;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_Base_Start(htim);
}

void DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
}

void DMA_DeInit(void) {
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Channel4_IRQn);

  __HAL_RCC_DMA2_CLK_DISABLE();
  __HAL_RCC_DMA1_CLK_DISABLE();
}
