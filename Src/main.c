#include "main.h"
#include "peripherals.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define USB_TRANSFER_SIZE 64
#define ADC_SAMPLE_BUF_SIZE (USB_TRANSFER_SIZE * 2)
#define DAC_SAMPLE_BUF_SIZE (USB_TRANSFER_SIZE * 2)

#define RUN_MODE_SETUP 0
#define RUN_MODE_ACTIVE 1

extern USBD_HandleTypeDef hUsbDeviceFS;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

uint32_t ADCSampleBuffer[ADC_SAMPLE_BUF_SIZE/4];
uint32_t DACSampleBuffer[DAC_SAMPLE_BUF_SIZE/4];
uint8_t CommandBuffer[USB_TRANSFER_SIZE];

uint8_t* ToSendBuf;
uint8_t* ToRecvBuf;

uint8_t RunMode;
uint8_t NextMode;
uint8_t DACFreeBufCount;
uint8_t USBRxFIFOLen;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void EnterActiveMode();
static void ExitActiveMode();
static void Sleep(void);
static void InitBuffers();
static void ProcessCommand();

static void EnterActiveMode() {
  RunMode = RUN_MODE_ACTIVE;

  DMA_Init();

  __HAL_RCC_OPAMP_CLK_ENABLE();
  OPAMP_Init(&hopamp1);
  OPAMP_Init(&hopamp2);
  __HAL_RCC_OPAMP_CLK_DISABLE();

  ADC_Init(&hadc1, ADCSampleBuffer, (ADC_SAMPLE_BUF_SIZE/2));
  DAC_Init(&hdac1, DACSampleBuffer, (DAC_SAMPLE_BUF_SIZE/2));
  Timer_Init(&htim3);
  Timer_Init(&htim6);

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

static void ExitActiveMode() {
  RunMode = RUN_MODE_SETUP;

  HAL_TIM_Base_Stop(&htim6);
  HAL_TIM_Base_DeInit(&htim6);

  HAL_TIM_Base_Stop(&htim3);
  HAL_TIM_Base_DeInit(&htim3);

  HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_DeInit(&hdac1);

  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_DeInit(&hadc1);

  __HAL_RCC_OPAMP_CLK_ENABLE();
  HAL_OPAMP_Stop(&hopamp1);
  HAL_OPAMP_DeInit(&hopamp1);

  HAL_OPAMP_Stop(&hopamp2);
  HAL_OPAMP_DeInit(&hopamp2);
  __HAL_RCC_OPAMP_CLK_DISABLE();

  DMA_DeInit();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

static void ProcessCommand() {
  if (USBRxFIFOLen) {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, CommandBuffer);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    USBRxFIFOLen--;
  }

  //TODO: Do something with the command
}

int main(void) {
  RunMode = RUN_MODE_SETUP;
  NextMode = RUN_MODE_SETUP;

  InitBuffers();

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  htim3.Instance = TIM3;
  htim6.Instance = TIM6;
  hadc1.Instance = ADC1;
  hdac1.Instance = DAC1;
  hopamp2.Instance = OPAMP2;

  while (1) {
    if (NextMode != RunMode) {
      NextMode == RUN_MODE_ACTIVE ? EnterActiveMode() : ExitActiveMode();
    } else if (RunMode == RUN_MODE_SETUP) {
      ProcessCommand();
      Sleep();
    } else {
      if (DACFreeBufCount && USBRxFIFOLen) {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, ToRecvBuf);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        DACFreeBufCount--;
        USBRxFIFOLen--;
      }

      if (ToSendBuf) {
        CDC_Transmit_FS(ToSendBuf, USB_TRANSFER_SIZE);
        ToSendBuf = NULL;
      }
    }
  }
}

static void InitBuffers() {
  ToSendBuf = NULL;
  ToRecvBuf = (uint8_t*) DACSampleBuffer;
  DACFreeBufCount = 2;
  USBRxFIFOLen = 0;
}

static inline void Sleep() {
  HAL_SuspendTick();
  __WFI();
  HAL_ResumeTick();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  UNUSED(hadc);
  ToSendBuf = &((uint8_t *) ADCSampleBuffer)[USB_TRANSFER_SIZE];
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  UNUSED(hadc);
  ToSendBuf = (uint8_t *) ADCSampleBuffer;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  UNUSED(hdac);
  ToRecvBuf = &((uint8_t *) DACSampleBuffer)[USB_TRANSFER_SIZE];
  DACFreeBufCount++;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  UNUSED(hdac);
  ToRecvBuf = (uint8_t *) DACSampleBuffer;
  DACFreeBufCount++;
}

void CDC_Received() {
  USBRxFIFOLen++;
}

void CDC_StartStop_Signal(uint8_t on) {
  if(on && RunMode == RUN_MODE_SETUP) {
    NextMode = RUN_MODE_ACTIVE;
  } else if (!on && RunMode == RUN_MODE_ACTIVE) {
    NextMode = RUN_MODE_SETUP;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void) {
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif