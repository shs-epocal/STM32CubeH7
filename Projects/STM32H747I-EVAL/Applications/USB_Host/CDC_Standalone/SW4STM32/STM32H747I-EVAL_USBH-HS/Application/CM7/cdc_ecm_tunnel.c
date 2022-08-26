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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t cdc_ecm_mac_string_host_stack[MAC_ADDRESS_STRING_SIZE];
CDC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
CDC_ECM_APP_State cdc_ecm_app_state;

uint8_t notification_buffer[NOTIFICATION_BUFFER_SIZE];
uint8_t rx_buffer[ETH_MAX_PACKET_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];

volatile bool flag_tx_data_ready = false;
uint32_t tx_size_received = 0;

uint32_t source_array_size = 0;
uint8_t source_array[ETH_MAX_PACKET_SIZE];
//uint8_t source_array[RX_USB_BUFF];
static volatile uint8_t prev_packet_size = 0;

bool full_multipacket = false;

static uint8_t circular_buffer_storage_[RX_BUFFER_SIZE] = {0};
static cbuf_handle_t host_in_buf = NULL;

uint8_t tx_circ_buf_storage_[TX_BUFFER_SIZE] = {0};
cbuf_handle_t tx_circ_buf = NULL;

USBH_StatusTypeDef ReqStatus = USBH_BUSY;
ENUM_StateTypeDef enumState = ENUM_GET_CFG_DESC;
int config_index = 1; //start at one since config at index zero already gathered

bool update_mac_address = false;

bool transmit_usbd_cmplt = true;

node_t *head = NULL;
node_t *tx_buf_queue = NULL;
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
	host_in_buf = circular_buf_init(circular_buffer_storage_, RX_BUFFER_SIZE);

	tx_circ_buf = circular_buf_init(tx_circ_buf_storage_, TX_BUFFER_SIZE);
}

static uint16_t counting_dropped = 0;
uint8_t temp_main_global_buf[1528];
void CDC_ECM_Tunnel_Process(USBH_HandleTypeDef *USBH_Host, USBD_HandleTypeDef *USBD_Device)
{
	if (cdc_ecm_app_state == CDC_ECM_APP_STATE_LINKED && !update_mac_address)
	{
		update_mac_address = true;

		CDC_ECM_HandleTypeDef *CDC_Host_Handle = (CDC_ECM_HandleTypeDef *) USBH_Host->pActiveClass->pData;

//		pStrDesc is the MAC address string of the Device stack
		for (int i = 0; i < MAC_ADDRESS_STRING_SIZE; i ++)
		{
			cdc_ecm_mac_string_host_stack[i] = CDC_Host_Handle->mac_address[i];
		}
		((USBD_CDC_ECM_ItfTypeDef *)USBD_Device->pUserData[USBD_Device->classId])->pStrDesc = cdc_ecm_mac_string_host_stack;
		transmit_usbd_cmplt = true;
		USBD_Start(USBD_Device);
	}
	/* USB Host Background task */
	USBH_Process(USBH_Host);

	USBH_CDC_ECM_UserProcess(USBH_Host, USBD_Device);

	int16_t eth_packet_len = dequeue(&tx_buf_queue);
	if (eth_packet_len > 0)
//	if (flag_tx_data_ready)
	{
//	  int16_t eth_packet_len = dequeue(&tx_buf_queue);
//	  if (eth_packet_len > 0)
//	  {
		  for (int i = 0; i < tx_size_received; i++)
		  {
			  uint8_t data;
			  circular_buf_get(tx_circ_buf, &data);
			  temp_main_global_buf[i] = data;
		  }
		  if(Appli_state == APPLICATION_READY)
		  {
			  USBH_CDC_ECM_Transmit(USBH_Host, temp_main_global_buf, tx_size_received);
		  } else {
			  counting_dropped++;
		  }
		  flag_tx_data_ready = false;
//	  }
	}

	int32_t eth_packet_len_host_buf = dequeue(&head);
	if (eth_packet_len_host_buf > 0)
//	if (full_multipacket)
//	if (transmit_usbd_cmplt)
	{
//		int16_t usb_packet_len = dequeue(&head);
//		if (usb_packet_len > 0)
//		{
		  int i = 0;

//		  USBH_UsrLog("%d", usb_packet_len < 64 ? 0 : 1);
//		  HAL_Delay(1);

		  while (i < eth_packet_len_host_buf)
//		  while (i < source_array_size)
		  {
			  uint8_t data;
			  circular_buf_get(host_in_buf, &data);
			  source_array[i] = data;
			  i++;
		  }
		  //when ethernet frame received send to ecm device to transmit
		  if (USBD_CDC_ECM_SetTxBuffer(USBD_Device, source_array, eth_packet_len_host_buf) == USBD_OK)
//		  if (USBD_CDC_ECM_SetTxBuffer(USBD_Device, source_array, source_array_size) == USBD_OK)
		  {
//			  transmit_usbd_cmplt = false;
			  USBD_CDC_ECM_TransmitPacket(USBD_Device);
		  }
//		  source_array_size = 0;
//		  prev_packet_size = 0;
//		  full_multipacket = false;
//		  circular_buf_reset(host_in_buf);
//		}
	}
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
	__disable_irq();
	uint16_t size = USBH_CDC_ECM_GetLastReceivedDataSize(&hUSBHost);

	if (size > 0){
		int i = 0;
		while (i < size)
		{
			circular_buf_put(host_in_buf, rx_buffer[i]);
			source_array_size++;
			i++;
		}

		if ((prev_packet_size == 64 && size < 64)
				|| (prev_packet_size == 64 && size < 1)
				|| (prev_packet_size < 64 && size < 64))
		{
//			full_multipacket = true;
			enqueue(&head, source_array_size);
			source_array_size = 0;
			prev_packet_size = 0;
		}
		prev_packet_size = size;
	}
	__enable_irq();
}

void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
	__disable_irq();
	USBH_CDC_ECM_StopTransmit(phost);
	__enable_irq();
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
				USBH_CDC_ECM_Receive(&hUSBHost, rx_buffer, RX_USB_BUFF);
			}
		}
		break;
	case CDC_ECM_APP_STATE_SETUP_STACK:
		if(Appli_state == APPLICATION_READY)
		{
			CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
			if (CDC_Handle->data_notification_state == CDC_ECM_IDLE)
			{
				USBH_CDC_ECM_NotificationReceive(&hUSBHost, notification_buffer, NOTIFICATION_BUFFER_SIZE);
			}
		}
		break;
	case CDC_ECM_APP_STATE_WAITING:
		break;
	case CDC_ECM_APP_STATE_DISCONNECTED:
		break;
	default:
		break;
	}

	if(Appli_state == APPLICATION_DISCONNECT)
	{
		Appli_state = APPLICATION_IDLE;
		USBH_ErrLog("USB device disconnected !!! \n");
		cdc_ecm_app_state = CDC_ECM_APP_STATE_IDLE;
	}
}
