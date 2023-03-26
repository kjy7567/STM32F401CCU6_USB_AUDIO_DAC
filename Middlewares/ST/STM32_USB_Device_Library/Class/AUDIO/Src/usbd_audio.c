/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "stm32f4xx_ll_dma.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq)         (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq)          (uint8_t)(((frq * 2U * 2U)/1000U) & 0xFFU), \
                                       (uint8_t)((((frq * 2U * 2U)/1000U) >> 8) & 0xFFU)

/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength  109 bytes*/
  HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                 /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                 /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER,                       /* bMaxPower = 100 mA */
  /* 09 byte*/

  /* USB Speaker Standard interface descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x27,                                 /* wTotalLength = 39*/
  0x00,
  0x01,                                 /* bInCollection */
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/

  /* USB Speaker Input Terminal Descriptor */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */
  0x01,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType AUDIO_TERMINAL_USB_STREAMING   0x0101 */
  0x01,
  0x00,                                 /* bAssocTerminal */
  0x01,                                 /* bNrChannels */
  0x00,                                 /* wChannelConfig 0x0000  Mono */
  0x00,
  0x00,                                 /* iChannelNames */
  0x00,                                 /* iTerminal */
  /* 12 byte*/

  /* USB Speaker Audio Feature Unit Descriptor */
  0x09,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID */
  0x01,                                 /* bSourceID */
  0x01,                                 /* bControlSize */
  AUDIO_CONTROL_MUTE,                   /* bmaControls(0) */
  0,                                    /* bmaControls(1) */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /*USB Speaker Output Terminal Descriptor */
  0x09,      /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */
  0x03,                                 /* bTerminalID */
  0x01,                                 /* wTerminalType  0x0301*/
  0x03,
  0x00,                                 /* bAssocTerminal */
  0x02,                                 /* bSourceID */
  0x00,                                 /* iTerminal */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
  /* Interface 1, Alternate Setting 0                                             */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x01,                                 /* bAlternateSetting */
  0x02,                                 /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB Speaker Audio Streaming Interface Descriptor */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */
  0x01,                                 /* bTerminalLink */
  0x01,                                 /* bDelay */
  0x01,                                 /* wFormatTag AUDIO_FORMAT_PCM  0x0001 */
  0x00,
  /* 07 byte*/

  /* USB Speaker Audio Type III Format Interface Descriptor */
  0x0B,                                 /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */
  0x02,                                 /* bNrChannels */
  0x02,                                 /* bSubFrameSize :  2 Bytes per frame (16bits) */
  16,                                   /* bBitResolution (16-bits per sample) */
  0x01,                                 /* bSamFreqType only one frequency supported */
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),   /* Audio sampling frequency coded on 3 bytes */
  /* 11 byte*/

  /* Endpoint 1 - Standard Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint */
  USBD_EP_TYPE_ISOC_ASYNC,              /* bmAttributes */
  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),    /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  AUDIO_FS_BINTERVAL,                   /* bInterval */
  0x00,                                 /* bRefresh */
  AUDIO_IN_EP,                          /* bSynchAddress */
  /* 09 byte*/

  /* Endpoint - Audio Streaming Descriptor*/
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */
  0x00,                                 /* bmAttributes */
  0x00,                                 /* bLockDelayUnits */
  0x00,                                 /* wLockDelay */
  0x00,
  /* 07 byte*/

  /* Endpoint 2 - Standard Descriptor - See UAC Spec 1.0 p.63 4.6.2.1 Standard AS Isochronous Synch Endpoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
  AUDIO_IN_EP,                       /* bEndpointAddress */
  0x11,                              /* bmAttributes */
  0x03, 0x00,                        /* wMaxPacketSize in Bytes */
  0x01,                              /* bInterval 1ms */
  0x00,		                         /* bRefresh 4ms = 2^2 */
  0x00,                              /* bSynchAddress */
  /* 09 byte*/
} ;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */
volatile uint32_t tx_flag = 1;
volatile uint32_t is_playing = 0;
volatile uint32_t all_ready = 0;
// FNSOF is critical for frequency changing to work
volatile uint32_t fnsof = 0;

/* Feature Unit Config */
#define AUDIO_CONTROL_FEATURES AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOL

/* Nomial feedback data for different frequencies */
#define AUDIO_FB_DEFAULT \
        (USBD_AUDIO_FREQ == 96000) ? (96 << 22) \
      : (USBD_AUDIO_FREQ == 48000) ? (48 << 22) \
      : (USBD_AUDIO_FREQ == 44100) ? ((44 << 22) + (1 << 22) / 10) \
      : (48 << 22)

/* Feedback is limited to +/- 1kHz */
#define AUDIO_FB_DELTA (uint32_t)(1 << 22)

volatile uint32_t fb_nom = AUDIO_FB_DEFAULT;
volatile uint32_t fb_value = AUDIO_FB_DEFAULT;
volatile uint32_t audio_buf_writable_size_last = AUDIO_TOTAL_BUF_SIZE / 2U;
volatile int32_t fb_raw = AUDIO_FB_DEFAULT;
volatile uint8_t fb_data[3] = {
    (uint8_t)((AUDIO_FB_DEFAULT & 0x0000FF00) >> 8),
    (uint8_t)((AUDIO_FB_DEFAULT & 0x00FF0000) >> 16),
    (uint8_t)((AUDIO_FB_DEFAULT & 0xFF000000) >> 24)};
/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Allocate Audio structure */
  haudio = USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (haudio == NULL)
  {
    pdev->pClassData = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassData = (void *)haudio;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_out[AUDIO_OUT_EP & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_out[AUDIO_OUT_EP & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
  }

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_OpenEP(pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  tx_flag = 1;

  haudio->alt_setting = 0U;
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->wr_ptr = 0U;
  haudio->rd_ptr = 0U;
  haudio->rd_enable = 0U;

  /* Initialize the Audio output Hardware layer */
  if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->Init(USBD_AUDIO_FREQ,
                                                       AUDIO_DEFAULT_VOLUME,
                                                       0U) != 0U)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Prepare Out endpoint to receive 1st packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, haudio->buffer,
                               AUDIO_OUT_PACKET);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  (void)USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
  (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].bInterval = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  tx_flag = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->DeInit(0U);
    (void)USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

            (void)USBD_CtlSendData(pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
            {
              haudio->alt_setting = (uint8_t)(req->wValue);
              if (haudio->alt_setting == 0)
              {
            	  memset(&haudio->buffer, 0, AUDIO_TOTAL_BUF_SIZE);
            	  all_ready = 0U;
            	  tx_flag = 1U;
            	  is_playing = 0U;

            	  haudio->offset = AUDIO_OFFSET_UNKNOWN;
            	  haudio->rd_enable = 0U;
            	  haudio->rd_ptr = 0U;
            	  haudio->wr_ptr = 0U;

            	  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            	  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

            	  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0);
              }
              else
              {
            	  all_ready = 0U;
            	  tx_flag = 1U;
            	  is_playing = 0U;

            	  haudio->offset = AUDIO_OFFSET_UNKNOWN;
            	  haudio->rd_enable = 0U;
            	  haudio->rd_ptr = 0U;
            	  haudio->wr_ptr = 0U;

            	  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            	  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

            	  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(USBD_AUDIO_FREQ,
																  AUDIO_DEFAULT_VOLUME,
																  0U);

            	  tx_flag = 0;
            	  all_ready = 1U;
              }
              (void)USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
            }
            else
            {
              /* Call the error management function (command will be NAKed */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_CfgDesc);

  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* epnum is the lowest 4 bits of bEndpointAddress. See UAC 1.0 spec, p.61 */
  if (epnum == (AUDIO_IN_EP & 0xf)) {
	tx_flag = 0U;
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->MuteCtl(haudio->control.data[0]);
      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  /* Only OUT control data are processed */
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  /**
   * 1. Must be static so that the values are kept when the function is
   *    again called.
   * 2. Must be volatile so that it will not be optimized out by the compiler.
   */
  static volatile uint32_t sof_count = 0;

  /* Do stuff only when playing */
  if (haudio->rd_enable >= 1U && all_ready == 1U)
  {
    /* Remaining writable buffer size */
    uint32_t audio_buf_writable_size;

    /* Update audio read pointer */
    haudio->rd_ptr = AUDIO_TOTAL_BUF_SIZE - (LL_DMA_ReadReg(DMA1_Stream4, NDTR) & 0xFFFF);

    /* Calculate remaining writable buffer size */
    audio_buf_writable_size = haudio->rd_ptr < haudio->wr_ptr
    						? (haudio->rd_ptr + AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr)/4
							: (haudio->rd_ptr - haudio->wr_ptr)/4;

    fb_value = AUDIO_FB_DEFAULT;

    fb_data[0] = (uint8_t)((fb_value >> 8) & 0x000000FF);
    fb_data[1] = (uint8_t)((fb_value >> 16) & 0x000000FF);
    fb_data[2] = (uint8_t)((fb_value >> 24) & 0x000000FF);

    sof_count += 1;

    if (sof_count == 1U)
    {
      sof_count = 0;
      // we start transmitting to I2S DAC when the audio buffer is half full, so the optimal
      // remaining writable size is (AUDIO_TOTAL_BUF_SIZE/2)/6 samples
      // Calculate feedback value based on the deviation from optimal
      int32_t audio_buf_writable_dev_from_nom_size = audio_buf_writable_size - AUDIO_TOTAL_BUF_SIZE/(2*6);
      // The feedback is ideally the true Fs generated by the I2S PLL clock and dividers. Unfortunately we have no means
      // to measure it internally. So we can only start with a nominal value calculated by assuming the HSE clock crystal
      // has 0ppm accuracy, and calculate the Fs frequency generated by the PLLI2S N, R, I2SDIV and ODD register values.
      // We then modify this nominal feedback frequency by the deviation from the ideal write pointer position wrt the read
      // pointer over time.
      // Need to multiply by at least a "PID k factor" of (1<<22) + 256 for a deviation of 1 sample to produce a change in feedback
      // as the internal fb value = (10.14) shifted 8bits in uint32_t.
      // We also should use the minimum "PID k factor" that keeps the write-pointer to read-pointer distance out of the
      // danger zone. This is to minimize the distortion caused by changes in host sampling frequency Fs.
      uint64_t tmp = (uint64_t)((int32_t)(1<<22) + (audio_buf_writable_dev_from_nom_size * 256));
      uint64_t pid_k = ((uint64_t)fb_nom) * tmp;
      fb_value = (uint32_t)(pid_k >> 22);

      /* Check feedback max / min */
      if (fb_value > fb_nom + AUDIO_FB_DELTA)
        fb_value = fb_raw = fb_nom + AUDIO_FB_DELTA;
      else if (fb_value < fb_nom - AUDIO_FB_DELTA)
    	fb_value = fb_raw = fb_nom - AUDIO_FB_DELTA;

      /* Set 10.14 format feedback data */
	  /**
	   * Order of 3 bytes in feedback packet: { LO byte, MID byte, HI byte }
	   *
	   * For example,
	   * 48.000(dec) => 300000(hex, 8.16) => 0C0000(hex, 10.14) => packet { 00, 00, 0C }
	   *
	   * Note that ALSA accepts 8.16 format.
	   */
      fb_data[0] = (uint8_t)((fb_value >> 8) & 0x000000FF);
	  fb_data[1] = (uint8_t)((fb_value >> 16) & 0x000000FF);
	  fb_data[2] = (uint8_t)((fb_value >> 24) & 0x000000FF);
	}
    /* Transmit feedback only when the last one is transmitted */
    if (tx_flag == 0U)
    {
      /* Get FNSOF. Use volatile for fnsof_new since its address is mapped to a hardware register. */
      USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
      uint32_t USBx_BASE = (uint32_t)USBx;
      uint32_t volatile fnsof_new = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

      if ((fnsof & 0x1) == (fnsof_new & 0x1))
//      if (fnsof_new & 0x1)
      {
        USBD_LL_Transmit(pdev, AUDIO_IN_EP, (uint8_t*)fb_data, 3U);
        /* Block transmission until it's finished. */
        tx_flag = 1U;
      }
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
//  USBD_AUDIO_HandleTypeDef *haudio;
//  uint32_t BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U;
//
//  if (pdev->pClassData == NULL)
//  {
//    return;
//  }
//
//  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;
//
//  haudio->offset = offset;
//
//  if (haudio->rd_enable == 1U)
//  {
//    haudio->rd_ptr += (uint16_t)BufferSize;
//
//    if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
//    {
//      /* roll back */
//      haudio->rd_ptr = 0U;
//    }
//  }
//
//  if (haudio->rd_ptr > haudio->wr_ptr)
//  {
//    if ((haudio->rd_ptr - haudio->wr_ptr) < AUDIO_OUT_PACKET)
//    {
//      BufferSize += 4U;
//    }
//    else
//    {
//      if ((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
//      {
//        BufferSize -= 4U;
//      }
//    }
//  }
//  else
//  {
//    if ((haudio->wr_ptr - haudio->rd_ptr) < AUDIO_OUT_PACKET)
//    {
//      BufferSize -= 4U;
//    }
//    else
//    {
//      if ((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
//      {
//        BufferSize += 4U;
//      }
//    }
//  }
//
//  if (haudio->offset == AUDIO_OFFSET_FULL)
//  {
//    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
//                                                         BufferSize, AUDIO_CMD_PLAY);
//    haudio->offset = AUDIO_OFFSET_NONE;
//  }
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USB_OTG_GlobalTypeDef* USBx = USB_OTG_FS;
  uint32_t USBx_BASE = (uint32_t)USBx;
  fnsof = (USBx_DEVICE->DSTS & USB_OTG_DSTS_FNSOF) >> 8;

  if (tx_flag == 1U) {
	tx_flag = 0U;
	USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);

  /* Prepare Out endpoint to receive next audio packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, (uint8_t *)&haudio->buffer[haudio->wr_ptr], AUDIO_OUT_PACKET);

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint16_t PacketSize;
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

  static uint8_t tmpbuf[1024];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == AUDIO_OUT_EP && all_ready == 1U)
  {
    /* Get received data packet length */
    PacketSize = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

	// Ignore strangely large packets
	if (PacketSize > AUDIO_OUT_PACKET)
	{
		PacketSize = 0U;
	}

	uint32_t tmpbuf_ptr = 0U;
	uint32_t num_samples = PacketSize / 4; // 2bytes per sample

	for (int i = 0; i < num_samples; i++)
	{
		haudio->buffer[haudio->wr_ptr++] = tmpbuf[tmpbuf_ptr]; // lsb
		haudio->buffer[haudio->wr_ptr++] = tmpbuf[tmpbuf_ptr+1];
		haudio->buffer[haudio->wr_ptr++] = tmpbuf[tmpbuf_ptr+2]; // msb
		haudio->buffer[haudio->wr_ptr++] = tmpbuf[tmpbuf_ptr+3];

		tmpbuf_ptr += 4;

		if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE) {
			haudio->wr_ptr = 0U;
		}
	}

	if (haudio->offset == AUDIO_OFFSET_UNKNOWN && is_playing == 0U)
	{
		if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE / 2U)
		{
			haudio->offset = AUDIO_OFFSET_NONE;
			is_playing = 1U;

			if (haudio->rd_enable == 0U) {
				haudio->rd_enable = 1U;
				// Set last writable buffer size to actual value. Note that rd_ptr is 0 now.
//				audio_buf_writable_samples_last = (AUDIO_TOTAL_BUF_SIZE - haudio->wr_ptr)/4;
				((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
																	AUDIO_TOTAL_BUF_SIZE / 2,
																	AUDIO_CMD_START);
			}
		}
	}


    /* Prepare Out endpoint to receive next audio packet */
    (void)USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, tmpbuf, AUDIO_OUT_PACKET);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, 64U);

  /* Send the current mute state */
  (void)USBD_CtlSendData(pdev, haudio->control.data, req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassData;

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    /* Prepare the reception of the buffer over EP0 */
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = (uint8_t)req->wLength; /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}


/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_DeviceQualifierDesc);

  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
  * @brief  USBD_AUDIO_RegisterInterface
  * @param  fops: Audio interface callback
  * @retval status
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData = fops;

  return (uint8_t)USBD_OK;
}
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
