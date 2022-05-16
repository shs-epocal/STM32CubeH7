/**
  ******************************************************************************
  * @file    usbh_cdc_ecms.h
  * @author  MCD Application Team
  * @brief   This file contains all the prototypes for the usbh_cdc-ecm.c
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

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_CDC_ECM_H
#define __USBH_CDC_ECM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"
#include "usbh_cdc.h"

#define LTM_ENABLE				0x32
#define SET_ISOCH_DELAY         0x31
#define HEADER_FUNC_DESC_TYPE 0x00U
#define UNION_FUNC_DESC_TYPE 0x06U
#define ECM_FUNC_DESC_TYPE 0x0FU

/* States for CDC State Machine */
typedef enum
{
  CDC_ECM_IDLE = 0U,
  CDC_ECM_SEND_DATA,
  CDC_ECM_SEND_DATA_WAIT,
  CDC_ECM_RECEIVE_DATA,
  CDC_ECM_RECEIVE_DATA_WAIT,
  CDC_ECM_STOP,
  /*operations that should be performed:
   * listen to notifications on interrupt endpoint
   * respond to notifications appropriately
   * - NETWORK_CONNECTION 00h
   * - RESPONSE_AVAILABLE 01h
   * - CONNECTION_SPEED_CHANGE 2Ah*/
}
CDC_ECM_DataStateTypeDef;

typedef enum
{
  CDC_ECM_STATE_IDLE = 0U,
  CDC_ECM_STATE_SET_ALT_INTERFACE,
  CDC_ECM_STATE_SET_ETH_PACKET_FILTER,
  CDC_ECM_STATE_GET_STRING_DESCRIPTOR,
  CDC_ECM_STATE_TRANSFER_DATA,
  CDC_ECM_STATE_ERROR,
}
CDC_ECM_StateTypeDef;

typedef enum
{
  CDC_ECM_APP_STATE_IDLE = 0,
  CDC_ECM_APP_STATE_LINKED,
  CDC_ECM_APP_STATE_SETUP_STACK,
  CDC_ECM_APP_STATE_WAITING,
  CDC_ECM_APP_STATE_DISCONNECTED,
}CDC_ECM_APP_State;

typedef struct _EthernetNetworkingFunctionalDescriptor
{
  uint8_t    bLength;            /*Size of this functional descriptor, in bytes.*/
  uint8_t    bDescriptorType;    /*CS_INTERFACE (0x24)*/
  uint8_t    bDescriptorSubType; /* Call Management functional descriptor subtype*/
  uint8_t    iMACAddress;
  uint32_t   bmEthernetStatistics;
  uint16_t   wMaxSegmentSize;
  uint16_t   wNumberMCFilters;
  uint8_t    bNumberPowerFilters;
}
CDC_EthernetNetworkingFuncDesc_TypeDef;

typedef struct _USBH_CDC_ECM_InterfaceDesc
{
  CDC_HeaderFuncDesc_TypeDef             CDC_HeaderFuncDesc;
  CDC_UnionFuncDesc_TypeDef              CDC_UnionFuncDesc;
  CDC_EthernetNetworkingFuncDesc_TypeDef CDC_EthernetNetworkingFuncDesc;
}
CDC_ECM_InterfaceDesc_Typedef;

/* Structure for CDC process */
typedef struct _CDC_ECM_Process
{
  CDC_CommItfTypedef                 CommItf;
  CDC_DataItfTypedef                 DataItf;
  uint8_t                           *pTxData;
  uint8_t                           *pRxData;
  uint8_t                           *pNotificationData;
  uint32_t                           TxDataLength;
  uint32_t                           RxDataLength;
  uint32_t                           NotificationDataLength;
  uint8_t                            NotificationInterval;
  CDC_ECM_InterfaceDesc_Typedef      CDC_Desc;
  CDC_ECM_StateTypeDef               state;
  CDC_ECM_StateTypeDef               next_state;
  CDC_ECM_DataStateTypeDef           data_tx_state;
  CDC_ECM_DataStateTypeDef           data_rx_state;
  CDC_ECM_DataStateTypeDef           data_notification_state;
  uint8_t                            Rx_Poll;
  uint8_t                            string_desc_index;
  uint16_t                           string_desc_len;
  uint16_t                           eth_packet_filter;
  uint8_t                            *mac_address;
}
CDC_ECM_HandleTypeDef;

/** @defgroup USBH_CDC_CORE_Exported_Variables
* @{
*/
extern USBH_ClassTypeDef  CDC_ECM_Class;
#define USBH_CDC_ECM_CLASS    &CDC_ECM_Class

/**
* @}
*/

/** @defgroup USBH_CDC_CORE_Exported_FunctionsPrototype
* @{
*/

USBH_StatusTypeDef  USBH_CDC_ECM_Transmit(USBH_HandleTypeDef *phost,
                                      uint8_t *pbuff,
                                      uint32_t length);

USBH_StatusTypeDef  USBH_CDC_ECM_Receive(USBH_HandleTypeDef *phost,
                                     uint8_t *pbuff,
                                     uint32_t length);
USBH_StatusTypeDef USBH_CDC_ECM_NotificationReceive(USBH_HandleTypeDef *phost,
                                     uint8_t *pbuff,
                                     uint32_t length);


uint16_t            USBH_CDC_ECM_GetLastReceivedDataSize(USBH_HandleTypeDef *phost);
uint16_t            USBH_CDC_ECM_GetLastReceivedDataSizeNoti(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef  USBH_CDC_ECM_Stop(USBH_HandleTypeDef *phost);

void USBH_CDC_ECM_TransmitCallback(USBH_HandleTypeDef *phost);

void USBH_CDC_ECM_ReceiveCallback(USBH_HandleTypeDef *phost);

/**
* @}
*/

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CDC_H */

/**
* @}
*/

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

