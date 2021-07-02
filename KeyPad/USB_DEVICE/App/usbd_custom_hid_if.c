/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */

  0x05    ,//bSize: 0x01, bType: Global, bTag: Usage Page
  0x01    ,//Usage Page(Generic Desktop Controls )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x06    ,//Usage(Keyboard)
  0xA1    ,//bSize: 0x01, bType: Main, bTag: Collection
  0x01    ,//Collection(Application )
  0x85    ,//bSize: 0x01, bType: Global, bTag: Report ID
  0x01    ,//Report ID(0x1 )
  0x05    ,//bSize: 0x01, bType: Global, bTag: Usage Page
  0x07    ,//Usage Page(Keyboard/Keypad )
  0x19    ,//bSize: 0x01, bType: Local, bTag: Usage Minimum
  0xE0    ,//Usage Minimum(0xE0 )
  0x29    ,//bSize: 0x01, bType: Local, bTag: Usage Maximum
  0xE7    ,//Usage Maximum(0xE7 )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x00    ,//Logical Minimum(0x0 )
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x01    ,//Logical Maximum(0x1 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x01    ,//Report Size(0x1 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x08    ,//Report Count(0x8 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x02    ,//Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x08    ,//Report Size(0x8 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x01    ,//Report Count(0x1 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x01    ,//Input(Constant, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0x05    ,//bSize: 0x01, bType: Global, bTag: Usage Page
  0x07    ,//Usage Page(Keyboard/Keypad )
  0x19    ,//bSize: 0x01, bType: Local, bTag: Usage Minimum
  0x00    ,//Usage Minimum(0x0 )
  0x29    ,//bSize: 0x01, bType: Local, bTag: Usage Maximum
  0x65    ,//Usage Maximum(0x65 )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x00    ,//Logical Minimum(0x0 )
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x65    ,//Logical Maximum(0x65 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x08    ,//Report Size(0x8 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x05    ,//Report Count(0x5 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x00    ,//Input(Data, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0xC0    ,//bSize: 0x00, bType: Main, bTag: End Collection
  0x05    ,//bSize: 0x01, bType: Global, bTag: Usage Page
  0x0C    ,//Usage Page(Consumer )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x01    ,//Usage(Consumer Control)
  0xA1    ,//bSize: 0x01, bType: Main, bTag: Collection
  0x01    ,//Collection(Application )
  0x85    ,//bSize: 0x01, bType: Global, bTag: Report ID
  0x02    ,//Report ID(0x2 )
  0x19    ,//bSize: 0x01, bType: Local, bTag: Usage Minimum
  0x00    ,//Usage Minimum(0x0 )
  0x2A    ,//bSize: 0x02, bType: Local, bTag: Usage Maximum
  0x3C,
  0x02 ,//Usage Maximum(0x23C )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x00    ,//Logical Minimum(0x0 )
  0x26    ,//bSize: 0x02, bType: Global, bTag: Logical Maximum
  0x3C,
  0x02 ,//Logical Maximum(0x23C )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x01    ,//Report Count(0x1 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x10    ,//Report Size(0x10 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x00    ,//Input(Data, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0xC0    ,//bSize: 0x00, bType: Main, bTag: End Collection
  0x05    ,//bSize: 0x01, bType: Global, bTag: Usage Page
  0x01    ,//Usage Page(Generic Desktop Controls )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x80    ,//Usage(System Control)
  0xA1    ,//bSize: 0x01, bType: Main, bTag: Collection
  0x01    ,//Collection(Application )
  0x85    ,//bSize: 0x01, bType: Global, bTag: Report ID
  0x03    ,//Report ID(0x3 )
  0x19    ,//bSize: 0x01, bType: Local, bTag: Usage Minimum
  0x81    ,//Usage Minimum(0x81 )
  0x29    ,//bSize: 0x01, bType: Local, bTag: Usage Maximum
  0x83    ,//Usage Maximum(0x83 )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x00    ,//Logical Minimum(0x0 )
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x01    ,//Logical Maximum(0x1 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x01    ,//Report Size(0x1 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x03    ,//Report Count(0x3 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x02    ,//Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x05    ,//Report Count(0x5 )
  0x81    ,//bSize: 0x01, bType: Main, bTag: Input
  0x01    ,//Input(Constant, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
  0xC0    ,//bSize: 0x00, bType: Main, bTag: End Collection
  0x06    ,//bSize: 0x02, bType: Global, bTag: Usage Page
  0x01,
  0xFF ,//Usage Page(Undefined )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x01    ,//Usage(1)
  0xA1    ,//bSize: 0x01, bType: Main, bTag: Collection
  0x01    ,//Collection(Application )
  0x85    ,//bSize: 0x01, bType: Global, bTag: Report ID
  0x04    ,//Report ID(0x4 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x01    ,//Report Count(0x1 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x08    ,//Report Size(0x8 )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x01    ,//Logical Minimum(0x1 )
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x0A    ,//Logical Maximum(0xA )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x20    ,//Usage(32)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x23    ,//Usage(35)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x4F    ,//Logical Maximum(0x4F )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x21    ,//Usage(33)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x30    ,//Logical Maximum(0x30 )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x22    ,//Usage(34)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x03    ,//Report Count(0x3 )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x24    ,//Usage(36)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0xC0    ,//bSize: 0x00, bType: Main, bTag: End Collection
  0x06    ,//bSize: 0x02, bType: Global, bTag: Usage Page
  0x01,
  0xFF ,//Usage Page(Undefined )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x01    ,//Usage(1)
  0xA1    ,//bSize: 0x01, bType: Main, bTag: Collection
  0x01    ,//Collection(Application )
  0x85    ,//bSize: 0x01, bType: Global, bTag: Report ID
  0x05    ,//Report ID(0x5 )
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x01    ,//Report Count(0x1 )
  0x75    ,//bSize: 0x01, bType: Global, bTag: Report Size
  0x08    ,//Report Size(0x8 )
  0x15    ,//bSize: 0x01, bType: Global, bTag: Logical Minimum
  0x01    ,//Logical Minimum(0x1 )
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x0A    ,//Logical Maximum(0xA )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x20    ,//Usage(32)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x23    ,//Usage(35)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x4F    ,//Logical Maximum(0x4F )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x21    ,//Usage(33)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x25    ,//bSize: 0x01, bType: Global, bTag: Logical Maximum
  0x30    ,//Logical Maximum(0x30 )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x22    ,//Usage(34)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)
  0x95    ,//bSize: 0x01, bType: Global, bTag: Report Count
  0x03    ,//Report Count(0x3 )
  0x09    ,//bSize: 0x01, bType: Local, bTag: Usage
  0x24    ,//Usage(36)
  0xB1    ,//bSize: 0x01, bType: Main, bTag: Feature
  0x03    ,//Feature(Constant, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field)



  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  UNUSED(event_idx);
  UNUSED(state);

  /* Start next USB packet transfer once data processing is completed */
  USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS);

  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
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

