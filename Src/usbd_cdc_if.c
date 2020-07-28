#include "usbd_cdc_if.h"

#define CDC_CONTROL_DTR 0x01
#define CDC_CONTROL_LINE_OFF 2
#define CDC_START_BAUDRATE 57600

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

void CDC_StartStop_Signal(uint8_t on);
void CDC_Received();

uint8_t CDCLineCoding[7];

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

static int8_t CDC_Init_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  uint32_t baudRate;

  switch(cmd)
  {
    case CDC_SET_CONTROL_LINE_STATE:
    if ((pbuf[CDC_CONTROL_LINE_OFF] & CDC_CONTROL_DTR) == 0) {
      CDC_StartStop_Signal(0);
    }
    break;
    case CDC_SEND_ENCAPSULATED_COMMAND:
    break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
    break;
    case CDC_SET_COMM_FEATURE:
    break;
    case CDC_GET_COMM_FEATURE:
    break;
    case CDC_CLEAR_COMM_FEATURE:
    break;
    case CDC_SET_LINE_CODING:
    for (int i = 0; i < 7; i++) CDCLineCoding[i] = pbuf[i];
    baudRate = (pbuf[3] << 24) | (pbuf[2] << 16) | (pbuf[1] << 8) | pbuf[0];
    CDC_StartStop_Signal(baudRate == CDC_START_BAUDRATE);
    break;
    case CDC_GET_LINE_CODING:
    for (int i = 0; i < 7; i++) pbuf[i] = CDCLineCoding[i];
    break;
    default:
    break;
  }

  return (USBD_OK);
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  CDC_Received();
  return (USBD_OK);
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  return result;
}

void CDC_USB_GlobalOUTNAK(uint32_t Flag) {
  PCD_HandleTypeDef *hpcd = hUsbDeviceFS.pData;
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USBx_DEVICE->DCTL |= Flag;
}
