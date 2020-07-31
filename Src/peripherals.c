#include "peripherals.h"

void Error_Handler(void);

void ADC_Init(ADC_HandleTypeDef *hadc, ADCConfiguration* userConfig, uint32_t* Buf, uint32_t ElementCount) {
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  FunctionalState oversamplingEnabled = ENABLE;
  uint32_t oversampling = 0;
  uint32_t bitShift = ADC_RIGHTBITSHIFT_NONE;

  switch(userConfig->Oversampling) {
    case 2:
      oversampling = ADC_OVERSAMPLING_RATIO_2;
      break;
    case 3:
      oversampling = ADC_OVERSAMPLING_RATIO_4;
      break;
    case 4:
      oversampling = ADC_OVERSAMPLING_RATIO_8;
      break;
    case 5:
      oversampling = ADC_OVERSAMPLING_RATIO_16;
      break;
    case 6:
      oversampling = ADC_OVERSAMPLING_RATIO_32;
      bitShift = ADC_RIGHTBITSHIFT_1;
      break;
    case 7:
      oversampling = ADC_OVERSAMPLING_RATIO_64;
      bitShift = ADC_RIGHTBITSHIFT_2;
      break;
    case 8:
      oversampling = ADC_OVERSAMPLING_RATIO_128;
      bitShift = ADC_RIGHTBITSHIFT_3;
      break;
    case 9:
      oversampling = ADC_OVERSAMPLING_RATIO_256;
      bitShift = ADC_RIGHTBITSHIFT_4;
      break;
    default:
      oversamplingEnabled = DISABLE;
      break;
  }

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
  hadc->Init.OversamplingMode = oversamplingEnabled;
  hadc->Init.Oversampling.Ratio = oversampling;
  hadc->Init.Oversampling.RightBitShift = bitShift;
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

  if (CONF_IS_ENABLED(userConfig->ChannelsConfig[0].ChannelEnable)) {
    uint32_t sampleTime;

    switch(userConfig->ChannelsConfig[0].SamplingTime) {
      case 1:
        sampleTime = ADC_SAMPLETIME_6CYCLES_5;
        break;
      case 2:
        sampleTime = ADC_SAMPLETIME_12CYCLES_5;
        break;
      case 3:
        sampleTime = ADC_SAMPLETIME_24CYCLES_5;
        break;
      case 4:
        sampleTime = ADC_SAMPLETIME_47CYCLES_5;
        break;
      case 5:
        sampleTime = ADC_SAMPLETIME_92CYCLES_5;
        break;
      case 6:
        sampleTime = ADC_SAMPLETIME_247CYCLES_5;
        break;
      case 7:
        sampleTime = ADC_SAMPLETIME_640CYCLES_5;
        break;
      default:
        sampleTime = ADC_SAMPLETIME_2CYCLES_5;
        break;
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampleTime;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
      Error_Handler();
    }
  }

  if (HAL_ADC_Start_DMA(hadc, Buf, ElementCount)) Error_Handler();
}

void DAC_Init(DAC_HandleTypeDef* hdac, DACConfiguration* userConfig, uint32_t* Buf, uint32_t ElementCount) {
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

  if (CONF_IS_ENABLED(userConfig->ChannelsConfig[0].ChannelEnable)) {
    if (HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, Buf, ElementCount, DAC_ALIGN_12B_R) != HAL_OK) {
      Error_Handler();
    }
  }
}

void OPAMP_Init(OPAMP_HandleTypeDef* hopamp, OPAMPConfiguration* userConfig) {
  uint32_t mode;

  switch(userConfig->Gain) {
    case 0:
      mode = OPAMP_STANDALONE_MODE;
      break;
    case 1:
      mode = OPAMP_FOLLOWER_MODE;
      break;
    default:
      mode = OPAMP_PGA_MODE;
      break;
  }

  hopamp->Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
  hopamp->Init.Mode = mode;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp->Init.InvertingInput = ((mode == OPAMP_PGA_MODE) && !userConfig->InvertingInput) ? OPAMP_INVERTINGINPUT_CONNECT_NO : OPAMP_INVERTINGINPUT_IO0;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_NORMAL;

  if (mode == OPAMP_PGA_MODE) {
    switch(userConfig->Gain) {
      case 2:
        hopamp->Init.PgaGain = OPAMP_PGA_GAIN_2;
        break;
      case 4:
        hopamp->Init.PgaGain = OPAMP_PGA_GAIN_4;
        break;
      case 8:
        hopamp->Init.PgaGain = OPAMP_PGA_GAIN_8;
        break;
      case 16:
        hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16;
        break;
      default:
        hopamp->Init.Mode = OPAMP_FOLLOWER_MODE;
        break;
    }
  }

  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (HAL_OPAMP_Init(hopamp) != HAL_OK) {
    Error_Handler();
  }

  HAL_OPAMP_SelfCalibrate(hopamp);
  HAL_OPAMP_Start(hopamp);
}

void Timer_Init(TIM_HandleTypeDef *htim, uint16_t Prescaler, uint16_t Period) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Init.Prescaler = Prescaler;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = Period;
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
