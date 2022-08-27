/**
  ******************************************************************************
  * @file    cdc_ecm_tunnel.c
  * @author  Hashem Alnader
  * @brief   CDC ECM HS port to FS port Tunnel
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "cdc_ecm_tunnel.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NOTIFICATION_BUFFER_SIZE 16
#define RX_USB_BUFF 64
#define MAC_ADDRESS_STRING_SIZE 12

#define USBH_SPEED_IS_FS
//#define USBH_SPEED_IS_HS

#ifdef USBH_SPEED_IS_FS
#define USB_MAX_PACKET_SIZE 64 //Full Speed max packet size
#else
#define USB_MAX_PACKET_SIZE 512 //High Speed max packet size
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t usbh_cdc_ecm_mac_addr_string[MAC_ADDRESS_STRING_SIZE];
CDC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
CDC_ECM_APP_State cdc_ecm_app_state;

uint8_t usbh_rx_notification_buffer[NOTIFICATION_BUFFER_SIZE];
uint8_t usbh_rx_data_buffer[ETH_MAX_PACKET_SIZE];

uint8_t usbd_to_usbh_eth_frame_holding_buffer[ETH_MAX_PACKET_SIZE];
volatile bool flag_usbd_rx_data_ready = false;
uint32_t usbd_rx_data_size_received = 0;

uint32_t usbh_to_usbd_eth_frame_holding_buffer_size = 0;
uint8_t usbh_to_usbd_eth_frame_holding_buffer[ETH_MAX_PACKET_SIZE];
static volatile uint8_t usbh_in_data_prev_packet_size = 0;

static uint8_t usbh_rx_data_circular_buffer_storage_[ETH_MAX_PACKET_SIZE] = {0};
static cbuf_handle_t usbh_rx_data_circular_buffer = NULL;

uint8_t usbd_rx_data_circular_buffer_storage_[ETH_MAX_PACKET_SIZE] = {0};
cbuf_handle_t usbd_rx_data_circular_buffer = NULL;

USBH_StatusTypeDef ReqStatus = USBH_BUSY;
ENUM_StateTypeDef enumState = ENUM_GET_CFG_DESC;
int config_index = 1; //start at one since config at index zero already gathered

bool usbd_cdc_ecm_update_mac_address = false;

node_t *usbh_to_usbd_eth_frame_sizes_queue = NULL;
node_t *usbd_to_usbh_eth_frame_sizes_queue = NULL;
/* Private function prototypes -----------------------------------------------*/
static void USBH_UserProcess(USBH_HandleTypeDef * phost, uint8_t id);
static void USBH_CDC_ECM_UserProcess(USBH_HandleTypeDef *phost, USBD_HandleTypeDef *pdev);
/* Private functions ---------------------------------------------------------*/

void CDC_ECM_Tunnel_Init(USBH_HandleTypeDef *USBH_Host, USBD_HandleTypeDef *USBD_Device)
{
	/* Init Device Library */
	USBD_Init(USBD_Device, &VCP_Desc, 0);

	/* Add Supported Class */
	USBD_RegisterClass(USBD_Device, USBD_CDC_ECM_CLASS);

	/* Add CDC Interface Class */
	USBD_CDC_ECM_RegisterInterface(USBD_Device, &USBD_CDC_ECM_fops);

	/* Init Host Library */
	USBH_Init(USBH_Host, USBH_UserProcess, 0);

	/* Add Supported Class */
	USBH_RegisterClass(USBH_Host, USBH_CDC_ECM_CLASS);

	/* Start Device Process */
	//Device Process is started in CDC_ECM_Tunnel_Process after MAC address is collected from USB Host stack

	/* Start Host Process */
	USBH_Start(USBH_Host);
}

void CDC_ECM_Tunnel_Buffer_Init(void)
{
	usbh_rx_data_circular_buffer = circular_buf_init(usbh_rx_data_circular_buffer_storage_, ETH_MAX_PACKET_SIZE);

	usbd_rx_data_circular_buffer = circular_buf_init(usbd_rx_data_circular_buffer_storage_, ETH_MAX_PACKET_SIZE);
}


void CDC_ECM_Tunnel_Process(USBH_HandleTypeDef *USBH_Host, USBD_HandleTypeDef *USBD_Device)
{
	if (cdc_ecm_app_state == CDC_ECM_APP_STATE_LINKED && !usbd_cdc_ecm_update_mac_address)
	{
		usbd_cdc_ecm_update_mac_address = true;

		CDC_ECM_HandleTypeDef *CDC_Host_Handle = (CDC_ECM_HandleTypeDef *) USBH_Host->pActiveClass->pData;

		//pStrDesc is the MAC address string of the Device stack
		for (int i = 0; i < MAC_ADDRESS_STRING_SIZE; i ++)
		{
			usbh_cdc_ecm_mac_addr_string[i] = CDC_Host_Handle->mac_address[i];
		}
		((USBD_CDC_ECM_ItfTypeDef *)USBD_Device->pUserData[USBD_Device->classId])->pStrDesc = usbh_cdc_ecm_mac_addr_string;
		USBD_Start(USBD_Device);
	}
	/* USB Host Background task */
	USBH_Process(USBH_Host);

	USBH_CDC_ECM_UserProcess(USBH_Host, USBD_Device);

	int16_t eth_packet_len = dequeue(&usbd_to_usbh_eth_frame_sizes_queue);
	if (eth_packet_len > 0)
	{
		for (int i = 0; i < usbd_rx_data_size_received; i++)
		  {
			  uint8_t data;
			  circular_buf_get(usbd_rx_data_circular_buffer, &data);
			  usbd_to_usbh_eth_frame_holding_buffer[i] = data;
		  }
		  if(Appli_state == APPLICATION_READY)
		  {
			  USBH_CDC_ECM_Transmit(USBH_Host, usbd_to_usbh_eth_frame_holding_buffer, usbd_rx_data_size_received);
		  }
		  flag_usbd_rx_data_ready = false;
	}

	int32_t eth_packet_len_host_buf = dequeue(&usbh_to_usbd_eth_frame_sizes_queue);
	if (eth_packet_len_host_buf > 0)
	{
		  int i = 0;

		  while (i < eth_packet_len_host_buf)
		  {
			  uint8_t data;
			  circular_buf_get(usbh_rx_data_circular_buffer, &data);
			  usbh_to_usbd_eth_frame_holding_buffer[i] = data;
			  i++;
		  }
		  //when ethernet frame received send to ecm device to transmit
		  if (USBD_CDC_ECM_SetTxBuffer(USBD_Device, usbh_to_usbd_eth_frame_holding_buffer, eth_packet_len_host_buf) == USBD_OK)
		  {
			  USBD_CDC_ECM_TransmitPacket(USBD_Device);
		  }
	}
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
	HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
	uint16_t size = USBH_CDC_ECM_GetLastReceivedDataSize(&hUSBHost);

	if (size > 0){
		int i = 0;
		while (i < size)
		{
			circular_buf_put(usbh_rx_data_circular_buffer, usbh_rx_data_buffer[i]);
			usbh_to_usbd_eth_frame_holding_buffer_size++;
			i++;
		}

		if ((usbh_in_data_prev_packet_size == USB_MAX_PACKET_SIZE && size < USB_MAX_PACKET_SIZE)
				|| (usbh_in_data_prev_packet_size == USB_MAX_PACKET_SIZE && size < 1)
				|| (usbh_in_data_prev_packet_size < USB_MAX_PACKET_SIZE && size < USB_MAX_PACKET_SIZE))
		{
			enqueue(&usbh_to_usbd_eth_frame_sizes_queue, usbh_to_usbd_eth_frame_holding_buffer_size);
			usbh_to_usbd_eth_frame_holding_buffer_size = 0;
			usbh_in_data_prev_packet_size = 0;
		}
		usbh_in_data_prev_packet_size = size;
	}
	HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
	USBH_CDC_ECM_StopTransmit(phost);
}

/**
* @brief  User Process
* @param  phost: Host Handle
* @param  id: Host Library user message ID
* @retval None
*/
static void USBH_UserProcess(USBH_HandleTypeDef * phost, uint8_t id)
{
	  switch (id)
	  {
	  case HOST_USER_SELECT_CONFIGURATION:
		  //since multiple configurations exist, must run back through enum steps
		  //and get other configurations
		  switch (enumState)
		  {
		  case ENUM_GET_CFG_DESC:
			  ReqStatus = USBH_Get_CfgDescAtIndex(phost, USB_CONFIGURATION_DESC_SIZE, config_index);
			  if (ReqStatus == USBH_OK)
			  {
				  enumState = ENUM_GET_FULL_CFG_DESC;
			  }
			  else if (ReqStatus == USBH_NOT_SUPPORTED)
			  {
				  USBH_ErrLog("Control error: Get Device configuration descriptor request failed");
				  phost->device.EnumCnt++;
				  if (phost->device.EnumCnt > 3U)
				  {
					  /* Buggy Device can't complete get device desc request */
					  USBH_UsrLog("Control error, Device not Responding Please unplug the Device.");
					  phost->gState = HOST_ABORT_STATE;
					  enumState = ENUM_IDLE;
				  }
				  else
				  {
					  /* Free control pipes */
					  USBH_FreePipe(phost, phost->Control.pipe_out);
					  USBH_FreePipe(phost, phost->Control.pipe_in);

					  /* Reset the USB Device */
					  phost->gState = HOST_IDLE;
					  enumState = ENUM_IDLE;
				  }
			  }
			  else
			  {
				  /* .. */
			  }
			  break;
		  case ENUM_GET_FULL_CFG_DESC:
		      /* get FULL config descriptor (config, interface, endpoints) */
		      ReqStatus = USBH_Get_CfgDescAtIndex(phost, phost->device.CfgDesc.wTotalLength, config_index);
		      if (ReqStatus == USBH_OK)
		      {
		    	  phost->gState = HOST_SET_CONFIGURATION;
		      }
		      else if (ReqStatus == USBH_NOT_SUPPORTED)
		      {
		        USBH_ErrLog("Control error: Get Device configuration descriptor request failed");
		        phost->device.EnumCnt++;
		        if (phost->device.EnumCnt > 3U)
		        {
		          /* Buggy Device can't complete get device desc request */
		          USBH_UsrLog("Control error, Device not Responding Please unplug the Device.");
		          phost->gState = HOST_ABORT_STATE;
		          enumState = ENUM_IDLE;
		        }
		        else
		        {
		          /* Free control pipes */
		          USBH_FreePipe(phost, phost->Control.pipe_out);
		          USBH_FreePipe(phost, phost->Control.pipe_in);

		          /* Reset the USB Device */
		          phost->gState = HOST_IDLE;
		          enumState = ENUM_IDLE;
		        }
		      }
		      else
		      {
		        /* .. */
		      }
			  break;
		  default:
			  break;
		  }
	    break;

	  case HOST_USER_DISCONNECTION:
	    Appli_state = APPLICATION_DISCONNECT;
	    cdc_ecm_app_state = CDC_ECM_APP_STATE_DISCONNECTED;
	    USBH_ErrLog("USB device disconnected !!! \n");
	    break;

	  case HOST_USER_CLASS_ACTIVE:
	    Appli_state = APPLICATION_READY;
	    cdc_ecm_app_state = CDC_ECM_APP_STATE_SETUP_STACK;
	    break;

	  case HOST_USER_CONNECTION:
	    Appli_state = APPLICATION_START;
	    break;

	  default:
	    break;
	  }
}

void USBH_CDC_ECM_ReceiveCallback(USBH_HandleTypeDef *phost)
{
	CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
	/* First byte is request type */
	if (CDC_Handle->pNotificationData[0] == CDC_ECM_BMREQUEST_TYPE_ECM){
		switch (CDC_Handle->pNotificationData[1])
		{
		case NETWORK_CONNECTION:
			if (CDC_Handle->pNotificationData[2] == CDC_ECM_NET_CONNECTED){
				cdc_ecm_app_state = CDC_ECM_APP_STATE_LINKED;
			}
			break;
		case RESPONSE_AVAILABLE:
			//for encapsulated responses
			break;
		case CONNECTION_SPEED_CHANGE:
			break;
		default:
			break;
		}
	}
}


static void USBH_CDC_ECM_UserProcess(USBH_HandleTypeDef *phost, USBD_HandleTypeDef *pdev)
{
	switch(cdc_ecm_app_state)
	{
	case CDC_ECM_APP_STATE_LINKED:
		if(Appli_state == APPLICATION_READY)
		{
			CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
			if (CDC_Handle->data_rx_state == CDC_ECM_IDLE)
			{
				USBH_CDC_ECM_Receive(&hUSBHost, usbh_rx_data_buffer, RX_USB_BUFF);
			}
		}
		break;
	case CDC_ECM_APP_STATE_SETUP_STACK:
		if(Appli_state == APPLICATION_READY)
		{
			CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
			if (CDC_Handle->data_notification_state == CDC_ECM_IDLE)
			{
				USBH_CDC_ECM_NotificationReceive(&hUSBHost, usbh_rx_notification_buffer, NOTIFICATION_BUFFER_SIZE);
			}
		}
		break;
	case CDC_ECM_APP_STATE_WAITING:
		break;
	case CDC_ECM_APP_STATE_DISCONNECTED:
		USBD_Stop(pdev);
		USBD_LL_Reset(pdev);
		usbd_cdc_ecm_update_mac_address = false;
		flag_usbd_rx_data_ready = false;
		cdc_ecm_app_state = CDC_ECM_APP_STATE_IDLE;
		break;
	default:
		break;
	}
}
