#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

void Error_Handler(void);

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#define DEV_ID_TERMINATOR 0x00
#define DEV_ID_ADC 0x01
#define DEV_ID_DAC 0x02
#define DEV_ID_OPAMP 0x03

#define CMD_UNKNOWN 0x00
#define CMD_SET_CONFIG 0x01

#define ADC1_ID 0
#define DAC1_ID 0
#define OPAMP1_ID 0
#define OPAMP2_ID 1

#define CONF_ERR_OK 0x00
#define CONF_ERR_INVALID_ID 0x01

#ifdef __cplusplus
}
#endif

#endif