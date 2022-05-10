/**
  ******************************************************************************
  * @file    usbh_cdc-ecm.c
  * @author  MCD Application Team
  * @brief   This file is the CDC Layer Handlers for USB Host CDC class.
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
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
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_sd.c"
- "stm32xxxxx_{eval}{discovery}{adafruit}_lcd.c"
- "stm32xxxxx_{eval}{discovery}_sdram.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbh_cdc.h"
#include "usbh_cdc-ecm.h"
#include <stdbool.h>

/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_CDC_CLASS
* @{
*/

/** @defgroup USBH_CDC_CORE
* @brief    This file includes CDC Layer Handlers for USB Host CDC class.
* @{
*/

/** @defgroup USBH_CDC_CORE_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBH_CDC_CORE_Private_Defines
* @{
*/
#define USBH_CDC_BUFFER_SIZE                 1024
/**
* @}
*/


/** @defgroup USBH_CDC_CORE_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup USBH_CDC_CORE_Private_Variables
* @{
*/
/**
* @}
*/


/** @defgroup USBH_CDC_CORE_Private_FunctionPrototypes
* @{
*/

extern bool updatePacketFilter;

//state queue
typedef struct node {
   int val;
   struct node *next;
} node_t;

static USBH_StatusTypeDef USBH_CDC_ECM_InterfaceInit(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_CDC_ECM_InterfaceDeInit(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_CDC_ECM_Process(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_CDC_ECM_SOFProcess(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_CDC_ECM_ClassRequest(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef SetEthPacketFilter(USBH_HandleTypeDef *phost,
		                                uint16_t value);

static void CDC_ProcessTransmission(USBH_HandleTypeDef *phost);

static void CDC_ProcessReception(USBH_HandleTypeDef *phost);

static void CDC_ProcessNotificationReception(USBH_HandleTypeDef *phost);

static void enqueue(node_t **head, int val);
static int dequeue(node_t **head);

USBH_ClassTypeDef  CDC_ECM_Class =
{
  "CDC-ECM",
  USB_CDC_CLASS,
//  VENDOR_SPECIFIC,
  USBH_CDC_ECM_InterfaceInit,
  USBH_CDC_ECM_InterfaceDeInit,
  USBH_CDC_ECM_ClassRequest,
  USBH_CDC_ECM_Process,
  USBH_CDC_ECM_SOFProcess,
  NULL,
};

static node_t *state_queue_head = NULL;
static node_t *string_descriptor_index_head = NULL;
static node_t *eth_packet_queue_head = NULL;

static bool firstPass = true;
static bool firstCasePass = true;
static bool firstEthPacketPass = true;
static bool continueForEver = false;
/**
* @}
*/


/** @defgroup USBH_CDC_CORE_Private_Functions
* @{
*/

/**
  * @brief  USBH_ParseECMDesc
  *         This function Parses the ECM functional descriptors
  * @param  int_descriptor : Interface descriptor destination
  * @param  cfg_desc: Configuration descriptor, expected to be populated
  * 		used for checking if number of bytes analyzed searching for
  * 		functional descriptors, is more than available
  * @param	buf : raw configuration data, to be processed
  * @retval None
  */
static void USBH_ParseECMDesc(CDC_ECM_InterfaceDesc_Typedef *int_desc, USBH_CfgDescTypeDef *cfg_desc, uint8_t *buf)
{
	USBH_DescHeader_t            *pdesc = (USBH_DescHeader_t *)(void *)buf;
	uint16_t                     ptr;
	uint8_t                      if_ix = 0U;
	uint8_t                      desc_sub_type;

	pdesc   = (USBH_DescHeader_t *)(void *)buf;

	ptr = USB_LEN_CFG_DESC; //start pointer after configuration descriptor at start

	while ((if_ix < 3) && (ptr < cfg_desc->wTotalLength))
	{
	  pdesc = USBH_GetNextDesc((uint8_t *)(void *)pdesc, &ptr);
	  if (pdesc->bDescriptorType   == (CS_INTERFACE))
	  {
//		USBH_ParseInterfaceDesc(pif, (uint8_t *)(void *)pdesc);
		desc_sub_type = *(uint8_t *)((uint8_t *)(void *)pdesc + 2);

		if (desc_sub_type == HEADER_FUNC_DESC_TYPE)
		{
			int_desc->CDC_HeaderFuncDesc.bLength = 			*(uint8_t *)((uint8_t *)(void *)pdesc + 0);
			int_desc->CDC_HeaderFuncDesc.bDescriptorType = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 1);
			int_desc->CDC_HeaderFuncDesc.bDescriptorSubType = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 2);
			int_desc->CDC_HeaderFuncDesc.bcdCDC = 				LE16((uint8_t *)(void *)pdesc + 3);
			if_ix++;
		} else if (desc_sub_type == UNION_FUNC_DESC_TYPE)
		{
			int_desc->CDC_UnionFuncDesc.bLength = 				*(uint8_t *)((uint8_t *)(void *)pdesc + 0);
			int_desc->CDC_UnionFuncDesc.bDescriptorType = 		*(uint8_t *)((uint8_t *)(void *)pdesc + 1);
			int_desc->CDC_UnionFuncDesc.bDescriptorSubType = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 2);
			int_desc->CDC_UnionFuncDesc.bMasterInterface = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 3);
			int_desc->CDC_UnionFuncDesc.bSlaveInterface0 = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 4);
			if_ix++;
		} else if (desc_sub_type == ECM_FUNC_DESC_TYPE)
		{
			int_desc->CDC_EthernetNetworkingFuncDesc.bLength = 				*(uint8_t *)((uint8_t *)(void *)pdesc + 0);
			int_desc->CDC_EthernetNetworkingFuncDesc.bDescriptorType = 		*(uint8_t *)((uint8_t *)(void *)pdesc + 1);
			int_desc->CDC_EthernetNetworkingFuncDesc.bDescriptorSubType = 		*(uint8_t *)((uint8_t *)(void *)pdesc + 2);
			int_desc->CDC_EthernetNetworkingFuncDesc.iMACAddress = 			*(uint8_t *)((uint8_t *)(void *)pdesc + 3);
			int_desc->CDC_EthernetNetworkingFuncDesc.bmEthernetStatistics = 	LE32((uint8_t *)(void *)pdesc + 4);
			int_desc->CDC_EthernetNetworkingFuncDesc.wMaxSegmentSize = 		LE16((uint8_t *)(void *)pdesc + 8);
			int_desc->CDC_EthernetNetworkingFuncDesc.wNumberMCFilters = 		LE16((uint8_t *)(void *)pdesc + 10);
			int_desc->CDC_EthernetNetworkingFuncDesc.bNumberPowerFilters = 	*(uint8_t *)((uint8_t *)(void *)pdesc + 12);
			if_ix++;
		}
	  }
	}
}

/**
  * @brief  USBH_CDC_InterfaceInit
  *         The function init the CDC class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CDC_ECM_InterfaceInit(USBH_HandleTypeDef *phost)
{

  USBH_StatusTypeDef status;
  uint8_t interface;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  interface = USBH_FindInterface(phost, USB_CDC_CLASS,
		  ETHERNET_NETWORKING_CONTROL_MODEL, NO_CLASS_SPECIFIC_PROTOCOL_CODE);

  if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) /* No Valid Interface */
  {
    USBH_DbgLog("Cannot Find the interface for Communication Interface Class.", phost->pActiveClass->Name);
    return USBH_FAIL;
  }

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData = (CDC_ECM_HandleTypeDef *)USBH_malloc(sizeof(CDC_ECM_HandleTypeDef));
  CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  //Populate functional descriptor
  USBH_ParseECMDesc(&CDC_Handle->CDC_Desc, &phost->device.CfgDesc, phost->device.CfgDesc_Raw);

  if (CDC_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for CDC Handle");
    return USBH_FAIL;
  }

  /* Initialize cdc handler */
  USBH_memset(CDC_Handle, 0, sizeof(CDC_ECM_HandleTypeDef));

  /*Collect the notification endpoint address and length*/
  if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress & 0x80U)
  {
    CDC_Handle->CommItf.NotifEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
    CDC_Handle->CommItf.NotifEpSize  = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
    CDC_Handle->NotificationInterval = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;
  }

  /*Allocate the length for host channel number in*/
  CDC_Handle->CommItf.NotifPipe = USBH_AllocPipe(phost, CDC_Handle->CommItf.NotifEp);

  /* Open pipe for Notification endpoint */
  USBH_OpenPipe(phost, CDC_Handle->CommItf.NotifPipe, CDC_Handle->CommItf.NotifEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_INTR,
                CDC_Handle->CommItf.NotifEpSize);

  USBH_LL_SetToggle(phost, CDC_Handle->CommItf.NotifPipe, 0U);

  interface = USBH_FindInterface(phost, DATA_INTERFACE_CLASS_CODE,
                                 RESERVED, NO_CLASS_SPECIFIC_PROTOCOL_CODE);

  if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) /* No Valid Interface */
  {
    USBH_DbgLog("Cannot Find the interface for Data Interface Class.", phost->pActiveClass->Name);
    return USBH_FAIL;
  }

  /*Collect the class specific endpoint address and length*/
  if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress & 0x80U)
  {
    CDC_Handle->DataItf.InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
    CDC_Handle->DataItf.InEpSize  = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  }
  else
  {
    CDC_Handle->DataItf.OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
    CDC_Handle->DataItf.OutEpSize  = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  }

  if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].bEndpointAddress & 0x80U)
  {
    CDC_Handle->DataItf.InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].bEndpointAddress;
    CDC_Handle->DataItf.InEpSize  = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].wMaxPacketSize;
  }
  else
  {
    CDC_Handle->DataItf.OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].bEndpointAddress;
    CDC_Handle->DataItf.OutEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].wMaxPacketSize;
  }

  /*Allocate the length for host channel number out*/
  CDC_Handle->DataItf.OutPipe = USBH_AllocPipe(phost, CDC_Handle->DataItf.OutEp);

  /*Allocate the length for host channel number in*/
  CDC_Handle->DataItf.InPipe = USBH_AllocPipe(phost, CDC_Handle->DataItf.InEp);

  /* Open channel for OUT endpoint */
  USBH_OpenPipe(phost, CDC_Handle->DataItf.OutPipe, CDC_Handle->DataItf.OutEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_BULK,
                CDC_Handle->DataItf.OutEpSize);

  /* Open channel for IN endpoint */
  USBH_OpenPipe(phost, CDC_Handle->DataItf.InPipe, CDC_Handle->DataItf.InEp,
                phost->device.address, phost->device.speed, USB_EP_TYPE_BULK,
                CDC_Handle->DataItf.InEpSize);

  CDC_Handle->state = CDC_ECM_IDLE_STATE;

  USBH_LL_SetToggle(phost, CDC_Handle->DataItf.OutPipe, 0U);
  USBH_LL_SetToggle(phost, CDC_Handle->DataItf.InPipe, 0U);

  return USBH_OK;
}



/**
  * @brief  USBH_CDC_InterfaceDeInit
  *         The function DeInit the Pipes used for the CDC class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CDC_ECM_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if (CDC_Handle->CommItf.NotifPipe)
  {
    USBH_ClosePipe(phost, CDC_Handle->CommItf.NotifPipe);
    USBH_FreePipe(phost, CDC_Handle->CommItf.NotifPipe);
    CDC_Handle->CommItf.NotifPipe = 0U;     /* Reset the Channel as Free */
  }

  if (CDC_Handle->DataItf.InPipe)
  {
    USBH_ClosePipe(phost, CDC_Handle->DataItf.InPipe);
    USBH_FreePipe(phost, CDC_Handle->DataItf.InPipe);
    CDC_Handle->DataItf.InPipe = 0U;     /* Reset the Channel as Free */
  }

  if (CDC_Handle->DataItf.OutPipe)
  {
    USBH_ClosePipe(phost, CDC_Handle->DataItf.OutPipe);
    USBH_FreePipe(phost, CDC_Handle->DataItf.OutPipe);
    CDC_Handle->DataItf.OutPipe = 0U;    /* Reset the Channel as Free */
  }

  if (phost->pActiveClass->pData)
  {
    USBH_free(phost->pActiveClass->pData);
    phost->pActiveClass->pData = 0U;
  }

  return USBH_OK;
}

/**
  * @brief  USBH_CDC_ECM_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for CDC ECM class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CDC_ECM_ClassRequest(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  USBH_ParseECMDesc(&CDC_Handle->CDC_Desc, &phost->device.CfgDesc, phost->device.CfgDesc_Raw);

  //setting up the queue
//  enqueue(&state_queue_head, (int)CDC_ECM_SET_ALT_INTERFACE);
//  enqueue(&eth_packet_queue_head, 12U);
//  enqueue(&state_queue_head, (int)CDC_ECM_SET_ETH_PACKET_FILTER);
//  enqueue(&string_descriptor_index_head, CDC_Handle->CDC_Desc.CDC_EthernetNetworkingFuncDesc.iMACAddress);
//  enqueue(&state_queue_head, (int)CDC_ECM_GET_STRING_DESCRIPTOR);
//
//  enqueue(&state_queue_head, (int)CDC_ECM_CONTINUE_FOREVER);
//  enqueue(&state_queue_head, (int)CDC_ECM_LISTEN_NOTIFICATIONS);
  CDC_Handle->state = CDC_ECM_SET_ALT_INTERFACE;
  status = USBH_OK;
  return status;
}

void enqueue(node_t **head, int val) {
   node_t *new_node = malloc(sizeof(node_t));
   if (!new_node) return;

   new_node->val = val;
   new_node->next = *head;

   *head = new_node;
}

int dequeue(node_t **head) {
   node_t *current, *prev = NULL;
   int retval = -1;

   if (*head == NULL) return -1;

   current = *head;
   while (current->next != NULL) {
      prev = current;
      current = current->next;
   }

   retval = current->val;
   free(current);

   if (prev)
      prev->next = NULL;
   else
      *head = NULL;

   return retval;
}

uint32_t NotificationInterruptTimer;
uint32_t RxTimer;

uint8_t charToHexConverter (char c)
{
	return (uint8_t) ((c <= '9') ? (c - '0') : (c - 'A' + 10)) ;
}

void parseMACAddress (char *mac_string, uint8_t *mac_array, int size)
{
	//0 0,1
	//1 2,3
	//2 4,5
	//3 6,7
	//4 8,9
	//5 10,11
	//MAC address comes in as characters, convert to integer
	for (int i = 0; i < size/2; i++)
	{
		uint8_t a = charToHexConverter(mac_string[i*2]);
		uint8_t b = charToHexConverter(mac_string[i*2 +1]);
		mac_array[i] = (uint8_t)(a << 4 | b);
	}
}

/**
  * @brief  USBH_CDC_ECM_Process
  *         The function is for managing state machine for CDC data transfers
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CDC_ECM_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_BUSY;
  USBH_StatusTypeDef req_status = USBH_OK;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if ((int)CDC_Handle->state <0)
  {
	  CDC_Handle->state = CDC_ECM_ERROR_STATE;
  }

  switch (CDC_Handle->state)
  {
    case CDC_ECM_IDLE_STATE:
      status = USBH_OK;
      break;

    case CDC_ECM_GET_STRING_DESCRIPTOR:
    	CDC_Handle->string_desc_index = CDC_Handle->CDC_Desc.CDC_EthernetNetworkingFuncDesc.iMACAddress;
    	CDC_Handle->string_desc_len = 255;
    	req_status = USBH_Get_StringDesc(phost,
                CDC_Handle->string_desc_index, CDC_Handle->pRxData,
                CDC_Handle->string_desc_len);
    	if (req_status == USBH_OK)
    	{
    		status = USBH_OK;

    		if (CDC_Handle->string_desc_index == CDC_Handle->CDC_Desc.CDC_EthernetNetworkingFuncDesc.iMACAddress)
    		{
    			parseMACAddress((char*)CDC_Handle->pRxData, CDC_Handle->mac_address, strlen((char*)CDC_Handle->pRxData));
    			CDC_Handle->state = CDC_ECM_CONTINUE_FOREVER;
    		}

    	}

    	else
    	{
    		if (req_status != USBH_BUSY)
    		{
    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
    		}
    	}
    	break;

    case CDC_ECM_SET_ALT_INTERFACE:
    	req_status = USBH_SetInterface(phost, 1U, 1U);

    	if (req_status == USBH_OK)
    	{
    		CDC_Handle->state = CDC_ECM_STATE_SET_ETH_PACKET_FILTER;
    	}

    	else
    	{
    		if (req_status != USBH_BUSY)
    		{
    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
    		}
    	}
    	break;

    case CDC_ECM_STATE_SET_ETH_PACKET_FILTER:

    	CDC_Handle->eth_packet_filter = 12U;

    	req_status = SetEthPacketFilter(phost, CDC_Handle->eth_packet_filter);
    	if (req_status == USBH_OK)
    	{
    		CDC_Handle->state = CDC_ECM_GET_STRING_DESCRIPTOR;
    	}
    	else
    	{
    		if (req_status != USBH_BUSY)
    		{
    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
    		}
    	}
    	break;

	case CDC_ECM_LISTEN_NOTIFICATIONS:
		//Process notification at interval time
		//Interval is number of frames, in FS frame is 1ms
		if (HAL_GetTick() - NotificationInterruptTimer >= CDC_Handle->NotificationInterval * 1)
		{
			NotificationInterruptTimer = HAL_GetTick();
			CDC_ProcessNotificationReception(phost);
		}

		CDC_ProcessReception(phost);
		CDC_ProcessTransmission(phost);
		break;

    case CDC_ECM_TRANSFER_DATA:
		CDC_ProcessTransmission(phost);
		CDC_ProcessReception(phost);

		firstCasePass = true;
    	status = USBH_OK;
    	break;

    case CDC_ECM_CONTINUE_FOREVER:
    	phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
    	CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
    	break;

    case CDC_ECM_ERROR_STATE:
      req_status = USBH_ClrFeature(phost, 0x00U);

      if (req_status == USBH_OK)
      {
        /*Change the state to waiting*/
        CDC_Handle->state = CDC_ECM_IDLE_STATE;
      }
      break;

    default:
      break;

  }

  return status;
}
//static USBH_StatusTypeDef USBH_CDC_ECM_Process(USBH_HandleTypeDef *phost)
//{
//  USBH_StatusTypeDef status = USBH_BUSY;
//  USBH_StatusTypeDef req_status = USBH_OK;
//  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
//
//  if (firstCasePass)
//  {
//	  CDC_Handle->state = (CDC_ECM_StateTypeDef)dequeue(&state_queue_head);
//	  firstCasePass = false;
//  }
//
//  if ((int)CDC_Handle->state <0)
//  {
//	  CDC_Handle->state = CDC_ECM_ERROR_STATE;
//  }
//
//  switch (CDC_Handle->state)
//  {
//    case CDC_ECM_IDLE_STATE:
//      status = USBH_OK;
//      break;
//
//    case CDC_ECM_GET_STRING_DESCRIPTOR:
//    	if (firstPass)
//    	{
//        	CDC_Handle->string_desc_index = (uint8_t)dequeue(&string_descriptor_index_head);
//        	firstPass = false;
//        	CDC_Handle->string_desc_len = 255;
//    	}
//    	req_status = USBH_Get_StringDesc(phost,
//                CDC_Handle->string_desc_index, CDC_Handle->pRxData,
//                CDC_Handle->string_desc_len);
//    	if (req_status == USBH_OK)
//    	{
//    		status = USBH_OK;
//    		firstPass = true;
//
//    		if (CDC_Handle->string_desc_index == CDC_Handle->CDC_Desc.CDC_EthernetNetworkingFuncDesc.iMACAddress)
//    		{
//    			parseMACAddress((char*)CDC_Handle->pRxData, CDC_Handle->mac_address, strlen((char*)CDC_Handle->pRxData));
//    		}
//
//			firstCasePass = true;
//    	}
//
//    	else
//    	{
//    		if (req_status != USBH_BUSY)
//    		{
//    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
//    		}
//    	}
//    	break;
//
//    case CDC_ECM_SET_ALT_INTERFACE:
//    	req_status = USBH_SetInterface(phost, 1U, 1U);
//
//    	if (req_status == USBH_OK)
//    	{
//			firstCasePass = true;
//    	}
//
//    	else
//    	{
//    		if (req_status != USBH_BUSY)
//    		{
//    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
//    		}
//    	}
//    	break;
//
//    case CDC_ECM_SET_ETH_PACKET_FILTER:
//    	if (firstEthPacketPass)
//    	{
//    		CDC_Handle->eth_packet_filter = (uint8_t)dequeue(&eth_packet_queue_head);
//        	firstEthPacketPass = false;
//    	}
//
//    	req_status = SetEthPacketFilter(phost, CDC_Handle->eth_packet_filter);
//    	if (req_status == USBH_OK)
//    	{
//			firstCasePass = true;
//			firstEthPacketPass = true;
//    	}
//    	else
//    	{
//    		if (req_status != USBH_BUSY)
//    		{
//    			CDC_Handle->state = CDC_ECM_ERROR_STATE;
//    		}
//    	}
//    	break;
//
//	case CDC_ECM_LISTEN_NOTIFICATIONS:
//		//Process notification at interval time
//		//Interval is number of frames, in FS frame is 1ms
//		if (HAL_GetTick() - NotificationInterruptTimer >= CDC_Handle->NotificationInterval * 1)
//		{
//			NotificationInterruptTimer = HAL_GetTick();
//			CDC_ProcessNotificationReception(phost);
//		}
//
////		if (HAL_GetTick() - RxTimer >= 50)
////		{
////			RxTimer = HAL_GetTick();
//			CDC_ProcessReception(phost);
////		}
//
//		CDC_ProcessTransmission(phost);
//
//		if (updatePacketFilter)
//		{
//			enqueue(&eth_packet_queue_head, 12U);
//			enqueue(&state_queue_head, (int)CDC_ECM_SET_ETH_PACKET_FILTER);
//			updatePacketFilter = false;
//		}
//		if (continueForEver){
//			CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
//			enqueue(&state_queue_head, (int)CDC_Handle->state);
//		}
//		firstCasePass = true;
//		break;
//
//    case CDC_ECM_TRANSFER_DATA:
//		CDC_ProcessTransmission(phost);
//		CDC_ProcessReception(phost);
//
//		firstCasePass = true;
//    	status = USBH_OK;
//    	break;
//
//    case CDC_ECM_CONTINUE_FOREVER:
//    	phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
//    	continueForEver = true;
//		firstCasePass = true;
//    	break;
//
//    case CDC_ECM_ERROR_STATE:
//      req_status = USBH_ClrFeature(phost, 0x00U);
//
//      if (req_status == USBH_OK)
//      {
//        /*Change the state to waiting*/
//        CDC_Handle->state = CDC_ECM_IDLE_STATE;
//      }
//      break;
//
//    default:
//      break;
//
//  }
//
//  return status;
//}

/**
  * @brief  USBH_CDC_SOFProcess
  *         The function is for managing SOF callback
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_CDC_ECM_SOFProcess(USBH_HandleTypeDef *phost)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(phost);

  return USBH_OK;
}

/**
  * @brief  USBH_CDC_Stop
  *         Stop current CDC Transmission
  * @param  phost: Host handle
  * @retval USBH Status
  */
USBH_StatusTypeDef  USBH_CDC_ECM_Stop(USBH_HandleTypeDef *phost)
{
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if (phost->gState == HOST_CLASS)
  {
    CDC_Handle->state = CDC_IDLE_STATE;

    USBH_ClosePipe(phost, CDC_Handle->CommItf.NotifPipe);
    USBH_ClosePipe(phost, CDC_Handle->DataItf.InPipe);
    USBH_ClosePipe(phost, CDC_Handle->DataItf.OutPipe);
  }
  return USBH_OK;
}

/**
  * @brief  SetEthPacketFilter
  * 		Sets Ethernet packet filter on adapter
  * 		Adapter controls what packets it forwards to host, this is where that is setup
  * 		Value of 1 in bit indicates that type of packet is forwarded
  * 		D15..5 Reserved
  * 		D4 PACKET_TYPE_MULTICAST
  * 		D3 PACKET_TYPE_BROADCAST
  * 		D2 PACKET_TYPE_DIRECTED
  * 		D1 PACKET_TYPE_ALL_MULTICAST
  * 		D0 PACKET_TYPE_PROMISCUOUS
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef SetEthPacketFilter(USBH_HandleTypeDef *phost, uint16_t value)
{

  phost->Control.setup.b.bmRequestType = USB_REQ_TYPE_CLASS | \
                                         USB_REQ_RECIPIENT_INTERFACE;

  phost->Control.setup.b.bRequest = CDC_SET_ETHERNET_PACKET_FILTER;
  phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = 0U;
  phost->Control.setup.b.wLength.w = 0U;

  return USBH_CtlReq(phost, 0U, 0U);
}

/**
  * @brief  This function return last received data size
  * @param  None
  * @retval None
  */
uint16_t USBH_CDC_ECM_GetLastReceivedDataSize(USBH_HandleTypeDef *phost)
{
  uint32_t dataSize;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if (phost->gState == HOST_CLASS)
  {
    dataSize = USBH_LL_GetLastXferSize(phost, CDC_Handle->DataItf.InPipe);
  }
  else
  {
    dataSize =  0U;
  }

  return (uint16_t)dataSize;
}

/**
  * @brief  This function return last received notification data size
  * @param  None
  * @retval None
  */
uint16_t USBH_CDC_ECM_GetLastReceivedDataSizeNoti(USBH_HandleTypeDef *phost)
{
  uint32_t dataSize;
  CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *) phost->pActiveClass->pData;

  if (phost->gState == HOST_CLASS)
  {
	  dataSize = USBH_LL_GetLastXferSize(phost, CDC_Handle->CommItf.NotifPipe);
  }
  else
  {
    dataSize =  0U;
  }

  return (uint16_t)dataSize;
}

/**
  * @brief  This function prepares the state before issuing the class specific commands
  * @param  None
  * @retval None
  */
USBH_StatusTypeDef  USBH_CDC_ECM_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint32_t length)
{
  USBH_StatusTypeDef Status = USBH_BUSY;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if ((CDC_Handle->state == CDC_ECM_IDLE_STATE)
		  || (CDC_Handle->state == CDC_ECM_TRANSFER_DATA)
		  || (CDC_Handle->state == CDC_ECM_LISTEN_NOTIFICATIONS)
		  || (CDC_Handle->state == CDC_ECM_CONTINUE_FOREVER))
  {
    CDC_Handle->pTxData = pbuff;
    CDC_Handle->TxDataLength = length;
    CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
    CDC_Handle->data_tx_state = CDC_ECM_SEND_DATA;
    Status = USBH_OK;
  }
  return Status;
}

USBH_StatusTypeDef  USBH_CDC_ECM_StopTransmit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef Status = USBH_OK;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
  CDC_Handle->data_tx_state = CDC_ECM_STOP;
  Status = USBH_OK;
  return Status;
}


/**
* @brief  This function prepares the state before issuing the class specific commands
* @param  None
* @retval None
*/
USBH_StatusTypeDef  USBH_CDC_ECM_Receive(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint32_t length)
{
  USBH_StatusTypeDef Status = USBH_BUSY;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if ((CDC_Handle->state == CDC_ECM_LISTEN_NOTIFICATIONS) || (CDC_Handle->state == CDC_ECM_TRANSFER_DATA))
  {
    CDC_Handle->pRxData = pbuff;
    CDC_Handle->RxDataLength = length;
    CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
    CDC_Handle->data_rx_state = CDC_ECM_RECEIVE_DATA;
    Status = USBH_OK;
  }
  return Status;
}

USBH_StatusTypeDef  USBH_CDC_ECM_StopReceive(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef Status = USBH_OK;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
  CDC_Handle->data_rx_state = CDC_ECM_STOP;
  Status = USBH_OK;
  return Status;
}

USBH_StatusTypeDef  USBH_CDC_ECM_NotificationReceive(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint32_t length)
{
  USBH_StatusTypeDef Status = USBH_BUSY;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

  if ((CDC_Handle->state == CDC_ECM_LISTEN_NOTIFICATIONS) || (CDC_Handle->state == CDC_ECM_TRANSFER_DATA))
  {
    CDC_Handle->pNotificationData = pbuff;
    CDC_Handle->NotificationDataLength = length;
    CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
    CDC_Handle->data_notification_state = CDC_ECM_RECEIVE_DATA;
    Status = USBH_OK;
  }
  return Status;
}

USBH_StatusTypeDef  USBH_CDC_ECM_NotificationStopReceive(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef Status = USBH_OK;
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  CDC_Handle->state = CDC_ECM_LISTEN_NOTIFICATIONS;
  CDC_Handle->data_notification_state = CDC_ECM_STOP;
  Status = USBH_OK;
  return Status;
}

/**
* @brief  The function is responsible for sending data to the device
*  @param  pdev: Selected device
* @retval None
*/
static void CDC_ProcessTransmission(USBH_HandleTypeDef *phost)
{
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;

  switch (CDC_Handle->data_tx_state)
  {
    case CDC_ECM_SEND_DATA:
      if (CDC_Handle->TxDataLength > CDC_Handle->DataItf.OutEpSize)
      {
        USBH_BulkSendData(phost,
                          CDC_Handle->pTxData,
                          CDC_Handle->DataItf.OutEpSize,
                          CDC_Handle->DataItf.OutPipe,
                          1U);
      }
      else
      {
        USBH_BulkSendData(phost,
                          CDC_Handle->pTxData,
                          (uint16_t)CDC_Handle->TxDataLength,
                          CDC_Handle->DataItf.OutPipe,
                          1U);
      }

      CDC_Handle->data_tx_state = CDC_ECM_SEND_DATA_WAIT;
      break;

    case CDC_ECM_SEND_DATA_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, CDC_Handle->DataItf.OutPipe);

      /* Check the status done for transmission */
      if (URB_Status == USBH_URB_DONE)
      {
        if (CDC_Handle->TxDataLength > CDC_Handle->DataItf.OutEpSize)
        {
          CDC_Handle->TxDataLength -= CDC_Handle->DataItf.OutEpSize;
          CDC_Handle->pTxData += CDC_Handle->DataItf.OutEpSize;
        }
        else
        {
          CDC_Handle->TxDataLength = 0U;
        }

        if (CDC_Handle->TxDataLength > 0U)
        {
          CDC_Handle->data_tx_state = CDC_ECM_SEND_DATA;
        }
        else
        {
          CDC_Handle->data_tx_state = CDC_ECM_IDLE;
          USBH_CDC_TransmitCallback(phost);
        }
      }
      else
      {
        if (URB_Status == USBH_URB_NOTREADY)
        {
          CDC_Handle->data_tx_state = CDC_ECM_SEND_DATA;
        }
      }
      break;

    default:
      break;
  }
}
/**
* @brief  This function responsible for reception of data from the device
*  @param  pdev: Selected device
* @retval None
*/

static void CDC_ProcessReception(USBH_HandleTypeDef *phost)
{
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;
  uint32_t length;

  switch (CDC_Handle->data_rx_state)
  {

    case CDC_ECM_RECEIVE_DATA:

      USBH_BulkReceiveData(phost,
                           CDC_Handle->pRxData,
                           CDC_Handle->DataItf.InEpSize,
                           CDC_Handle->DataItf.InPipe);

      CDC_Handle->data_rx_state = CDC_ECM_RECEIVE_DATA_WAIT;

      break;

    case CDC_ECM_RECEIVE_DATA_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, CDC_Handle->DataItf.InPipe);

      /*Check the status done for reception*/
      if (URB_Status == USBH_URB_DONE )//|| URB_Status == USBH_URB_IDLE)
      {
        length = USBH_LL_GetLastXferSize(phost, CDC_Handle->DataItf.InPipe);

        if (((CDC_Handle->RxDataLength - length) > 0U) && (length > CDC_Handle->DataItf.InEpSize))
        {
          CDC_Handle->RxDataLength -= length ;
          CDC_Handle->pRxData += length;
          CDC_Handle->data_rx_state = CDC_ECM_RECEIVE_DATA;
        }
        else
        {
          USBH_CDC_ReceiveCallback(phost);
          CDC_Handle->data_rx_state = CDC_ECM_IDLE;
        }
      }
      break;

    default:
      break;
  }
}

static void CDC_ProcessNotificationReception(USBH_HandleTypeDef *phost)
{
  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
  USBH_URBStateTypeDef URB_Status = USBH_URB_IDLE;
  uint32_t length = 0;

  switch (CDC_Handle->data_notification_state)
  {

    case CDC_ECM_RECEIVE_DATA:

    	USBH_InterruptReceiveData(phost,
                           CDC_Handle->pNotificationData,
                           CDC_Handle->CommItf.NotifEpSize,
                           CDC_Handle->CommItf.NotifPipe);

      CDC_Handle->data_notification_state = CDC_ECM_RECEIVE_DATA_WAIT;

      break;

    case CDC_ECM_RECEIVE_DATA_WAIT:

      URB_Status = USBH_LL_GetURBState(phost, CDC_Handle->CommItf.NotifPipe);
      /*Check the status done for reception*/
      if (URB_Status == USBH_URB_DONE || URB_Status == USBH_URB_IDLE)
      {
        length = USBH_LL_GetLastXferSize(phost, CDC_Handle->CommItf.NotifPipe);

        if (((CDC_Handle->NotificationDataLength - length) > 0U) && (length > CDC_Handle->CommItf.NotifEpSize))
        {
          CDC_Handle->NotificationDataLength -= length ;
          CDC_Handle->pNotificationData += length;
          CDC_Handle->data_notification_state = CDC_ECM_RECEIVE_DATA;
        }
        else
        {
          CDC_Handle->data_notification_state = CDC_ECM_IDLE;
          USBH_CDC_ECM_ReceiveCallback(phost);
        }
      }
      break;

    default:
      break;
  }
}

/**
* @brief  The function informs user that data have been received
*  @param  pdev: Selected device
* @retval None
*/
__weak void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(phost);
}

/**
* @brief  The function informs user that data have been sent
*  @param  pdev: Selected device
* @retval None
*/
__weak void USBH_CDC_ECM_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(phost);
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


/**
* @}
*/


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
