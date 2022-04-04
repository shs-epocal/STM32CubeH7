/**
  ******************************************************************************
  * @file    USB_Host/CDC_Standalone/Src/main.c
  * @author  MCD Application Team
  * @brief   USB host CDC demo main file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "main.h"
#include <stdbool.h>

#include "lwip/opt.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif
#include "app_ethernet.h"
#include "tcp_echoclient.h"
#include "stm32h7xx_hal_eth.h"
#include "circular_buffer.h"

//extern void initialise_monitor_handles(void);

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
#define NOTIFICATION_BUFFER_SIZE 16
#define RX_BUFFER_SIZE ETH_MAX_PACKET_SIZE
#define TX_BUFFER_SIZE ETH_MAX_PACKET_SIZE
#define IFNAME0 's'
#define IFNAME1 't'

#define RX_USB_BUFF 64

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
USBH_HandleTypeDef hUSBHost;
CDC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
__IO CDC_ECM_APP_State cdc_ecm_app_state;
#define HSEM_ID_0 (0U) /* HW semaphore 0 */
uint8_t notification_buffer[NOTIFICATION_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];

volatile bool link_state_changed_status = false;

uint8_t rx_buffer_store[RX_BUFFER_SIZE];
uint8_t notification_buffer_store[NOTIFICATION_BUFFER_SIZE];
uint8_t size_rx_last;
uint8_t size_noti_last;

bool connect_tcp = false;

static uint32_t source_array_size = 0;
static uint8_t source_array[RX_BUFFER_SIZE];
static volatile uint8_t prev_packet_size = 0;

bool full_multipacket = false;

static uint8_t circular_buffer_storage_[RX_BUFFER_SIZE*2] = {0};
static cbuf_handle_t handle_ = NULL;

static uint8_t packet_size_order_in[4096];
static uint32_t packet_order_index = 0;
static uint32_t packet_order_index_read = 0;

/* Private function prototypes ----------------------------------------------- */
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void USBH_UserProcess(USBH_HandleTypeDef * phost, uint8_t id);
static void CDC_InitApplication(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void Error_Handler(void);
void USBH_CDC_ECM_Process(USBH_HandleTypeDef *phost);
static void DumpReceivedData(uint8_t *buf, bool notification);
static void Netif_Config(void);

typedef struct node {
   struct pbuf* val;
   struct node *next;
} node_t;

static node_t *pbuf_queue_head = NULL;

bool enqueue(node_t **head, struct pbuf* val) {
   node_t *new_node = malloc(sizeof(node_t));
   if (!new_node) return false;

   new_node->val = val;
   new_node->next = *head;

   *head = new_node;
   return true;
}

struct pbuf* dequeue(node_t **head) {
   node_t *current, *prev = NULL;
   struct pbuf* retval = NULL;

   if (*head == NULL) return NULL;

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

/* Private functions --------------------------------------------------------- */
static err_t netif_output(struct netif *netif, struct pbuf *p);
static void netif_status_callback(struct netif *netif);
static err_t netif_init(struct netif *netif);

static err_t
netif_output(struct netif *netif, struct pbuf *p)
{
  __disable_irq();
  uint16_t copied_total = pbuf_copy_partial(p, tx_buffer, p->tot_len, 0);
  USBH_CDC_ECM_Transmit(&hUSBHost, tx_buffer, copied_total);
  __enable_irq();
  return ERR_OK;
}
static void
netif_status_callback(struct netif *netif)
{
  printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}
static err_t
netif_init(struct netif *netif)
{
	netif->name[0] = 's';
	netif->name[1] = 't';
	netif->output     = etharp_output;
	netif->linkoutput = netif_output;
	netif->hwaddr_len = ETH_HWADDR_LEN;
	/* maximum transfer unit */
	netif->mtu = ETH_MAX_PAYLOAD;
	/* device capabilities */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
	return ERR_OK;
}

static struct netif netif;

bool updatePacketFilter;

/**
* @brief  Main program
* @param  None
* @retval None
*/
int main(void)
{
  int32_t timeout;
//  initialise_monitor_handles();
  /* This project calls firstly two functions in order to configure MPU feature
  and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable()*/

  /* Configure the MPU attributes as Write Through for SDRAM*/
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if (timeout < 0)
  {
    Error_Handler();
  }
  /* STM32H7xx HAL library initialization:
  - Systick timer is configured by default as source of time base, but user
  can eventually implement his proper time base source (a general purpose
  timer for application or other time source), keeping in mind that Time base
  duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
  handled in milliseconds basis.
  - Set NVIC Group Priority to 4
  - Low Level Initialization: global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the System clock to have a frequency of 400 Mhz */
  SystemClock_Config();
  /* When system initialization is finished, Cortex-M7 will release (wakeup) Cortex-M4  by means of
  HSEM notification. Cortex-M4 release could be also ensured by any Domain D2 wakeup source (SEV,EXTI..).
  */

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();

  /*Take HSEM */
  HAL_HSEM_FastTake(HSEM_ID_0);
  /*Release HSEM in order to notify the CPU2(CM4)*/
  HAL_HSEM_Release(HSEM_ID_0,0);

  /* wait until CPU2 wakes up from stop mode */
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
    Error_Handler();
  }

  updatePacketFilter = false;

  /* Init CDC Application */
  CDC_InitApplication();

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_ORANGE);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_BLUE);

  /* Enable the USB voltage level detector */
  HAL_PWREx_EnableUSBVoltageDetector();

  /* Init Host Library */
  USBH_Init(&hUSBHost, USBH_UserProcess, 0);

  /* Add Supported Class */
  USBH_RegisterClass(&hUSBHost, USBH_CDC_ECM_CLASS);

  lwip_init();

  /* Start Host Process */
  USBH_Start(&hUSBHost);

  handle_ = circular_buf_init(circular_buffer_storage_, RX_BUFFER_SIZE*2);

  /* Run Application (Blocking mode) */
  while (1)
  {
	  BSP_LED_Toggle(LED_GREEN);
    /* USB Host Background task */
    USBH_Process(&hUSBHost);

    USBH_CDC_ECM_Process(&hUSBHost);

    /* Read a received packet from the Ethernet buffers and send it
       to the lwIP for handling */

    if (full_multipacket)
    {
    	__disable_irq();
        int i = 0;

        while (i < source_array_size)
        {
        	uint8_t data;
        	circular_buf_get(handle_, &data);
        	source_array[i] = data;
        	i++;
        }

        struct pbuf* p = pbuf_alloc(PBUF_RAW, source_array_size, PBUF_POOL);
        if(p != NULL) {
        	/* Copy ethernet frame into pbuf */
        	pbuf_take(p, source_array, source_array_size);
        	/* Put in a queue which is processed in main loop */
    		if(!enqueue(&pbuf_queue_head, p)) {
    		  /* queue is full -> packet loss */
    		  pbuf_free(p);
    		}
        	source_array_size = 0;
        	prev_packet_size = 0;
        	full_multipacket = false;
        	circular_buf_reset(handle_);
        }
        __enable_irq();
    }

    struct pbuf* p = (struct pbuf*)dequeue(&pbuf_queue_head);
    if(p != NULL) {
      if(netif.input(p, &netif) != ERR_OK) {
        pbuf_free(p);
      }
    }

    /* Cyclic lwIP timers check */
    sys_check_timeouts();

#if LWIP_NETIF_LINK_CALLBACK
    Ethernet_Link_Periodic_Handle(&netif,notification_buffer_store, size_noti_last);
#endif

#if LWIP_DHCP
    DHCP_Periodic_Handle(&gnetif);
#endif
    /* CDC Menu Process */
//    CDC_MenuProcess();
  }
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  /* This function should be implemented by the user application.
  It is called into this driver when an event on Button is triggered.   */
  static uint32_t debounce_time = 0;
  if(Button == BUTTON_TAMPER)
  {
	  /* Prevent debounce effect for Tamper button */
	  if((HAL_GetTick() - debounce_time) > 50)
	  {
		  debounce_time = HAL_GetTick();
	  }
	  else
	  {
		  return;
	  }
      tcp_echoclient_connect();
  }
}

/**
* @brief  CDC application Init.
* @param  None
* @retval None
*/
static void CDC_InitApplication(void)
{
  /* Configure Tamper Button */
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);

  /* Configure Joystick in EXTI mode */
 BSP_JOY_Init(JOY1, JOY_MODE_EXTI, JOY_ALL);

  /* Configure LED1 and LED3 */
//  BSP_LED_Init(LED1);
//  BSP_LED_Init(LED3);

  /* Initialize the LCD */
  BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
  UTIL_LCD_SetFuncDriver(&LCD_Driver);

  UTIL_LCD_TRACE_Init();
#ifdef USE_USB_HS
  UTIL_LCD_TRACE_SetHeader((uint8_t *) " USB OTG HS CDC Host");
#else
  UTIL_LCD_TRACE_SetHeader((uint8_t *) " USB OTG FS CDC Host");
#endif

  LCD_UsrTrace("USB Host library started.\n");

  /* Start CDC Interface */
  USBH_UsrLog("Starting CDC Demo");

//  Menu_Init();
  /* Initialize microSD */
//  if (SD_StorageInit() == 0)
//  {
//    SD_StorageParse();
//  }
}

USBH_StatusTypeDef ReqStatus = USBH_BUSY;
ENUM_StateTypeDef enumState = ENUM_GET_CFG_DESC;
int config_index = 1; //start at one since config at index zero already gathered

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
	    cdc_ecm_app_state = CDC_ECM_APP_DISCONNECTED;
	    break;

	  case HOST_USER_CLASS_ACTIVE:
	    Appli_state = APPLICATION_READY;
	    cdc_ecm_app_state = CDC_ECM_APP_SETUP_STACK;
	    updatePacketFilter = true;
	    break;

	  case HOST_USER_CONNECTION:
	    Appli_state = APPLICATION_START;
	    break;

	  default:
	    break;
	  }
}

void lwip_network_interface_setup(void)
{
	  ip_addr_t ipaddr;
	  ip_addr_t netmask;
	  ip_addr_t gw;

	  /* IP address default setting */
	  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
	  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
	  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

	  /* add the network interface */
	  netif_add(&netif, &ipaddr, &netmask, &gw, NULL, netif_init, netif_input);

	  /*  Registers the default network interface */
	  netif_set_default(&netif);

	  netif_set_status_callback(&netif, &netif_status_callback);
}

void USBH_CDC_ECM_Process(USBH_HandleTypeDef *phost)
{
	  switch(cdc_ecm_app_state)
	  {
	  case CDC_ECM_APP_LINKED:
	    if(Appli_state == APPLICATION_READY)
	    {
	    	CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;
			if (CDC_Handle->data_notification_state == CDC_ECM_IDLE)
			{
				USBH_CDC_ECM_NotificationReceive(&hUSBHost, notification_buffer, NOTIFICATION_BUFFER_SIZE);
			}
			if (CDC_Handle->data_rx_state == CDC_ECM_IDLE)
			{
				USBH_CDC_ECM_Receive(&hUSBHost, rx_buffer, RX_USB_BUFF);
			}
	    }
	    break;
	  case CDC_ECM_APP_SETUP_STACK:
		  if(Appli_state == APPLICATION_READY)
		  {
			  CDC_ECM_HandleTypeDef *CDC_Handle = (CDC_ECM_HandleTypeDef *) phost->pActiveClass->pData;

			  /* set MAC hardware address */
			  netif.hwaddr[0] =  CDC_Handle->mac_address[0];
			  netif.hwaddr[1] =  CDC_Handle->mac_address[1];
			  netif.hwaddr[2] =  CDC_Handle->mac_address[2];
			  netif.hwaddr[3] =  CDC_Handle->mac_address[3];
			  netif.hwaddr[4] =  CDC_Handle->mac_address[4];
			  netif.hwaddr[5] =  CDC_Handle->mac_address[5];

			  lwip_network_interface_setup();
			  cdc_ecm_app_state = CDC_ECM_APP_LINKED;
		  }
		  break;
	  case CDC_ECM_APP_WAITING:
		  break;
	  case CDC_ECM_APP_DISCONNECTED:
		  netif_set_down(&netif);
		  netif_remove(&netif);
		  break;
	  default:
	    break;
	  }

	  if(Appli_state == APPLICATION_DISCONNECT)
	  {
	    Appli_state = APPLICATION_IDLE;
	    USBH_ErrLog("USB device disconnected !!! \n");
	    cdc_ecm_app_state = CDC_ECM_APP_IDLE;
	  }
}
void USBH_CDC_ECM_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  /* Prevent unused argument(s) compilation warning */
	  uint16_t size;
	  size = USBH_CDC_ECM_GetLastReceivedDataSizeNoti(&hUSBHost);
	  if (size > 0)
	  {
		  size_noti_last = size;
		  memcpy(notification_buffer_store, notification_buffer, size_noti_last);
	  }
	  else
		  size_noti_last = 0;
}

void mergeArrays(uint8_t *arr1, uint8_t *arr2, uint8_t size1, uint8_t size2)
{
	int i = size1;
	int j = 0;
	while (i < RX_BUFFER_SIZE && j < size2)
	{
		arr1[i] = arr2[j];
		i++;
		j++;
	}
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
	BSP_LED_Off(LED_ORANGE);
	BSP_LED_Toggle(LED_GREEN);
	uint16_t size = USBH_CDC_ECM_GetLastReceivedDataSize(&hUSBHost);

	if (size >= 0){
		packet_size_order_in[packet_order_index++] = size;
		int i = 0;
		while (i < size)
		{
			circular_buf_put(handle_, rx_buffer[i]);
			source_array_size++;
			i++;
		}

		if ((prev_packet_size == 64 && size < 64)
				|| (prev_packet_size == 64 && size < 1)
				|| (prev_packet_size < 64 && size < 64))
		{
			full_multipacket = true;
		}
		prev_packet_size = size;
	}
}

void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
	BSP_LED_On(LED_ORANGE);
	USBH_CDC_ECM_Receive(&hUSBHost, rx_buffer, RX_BUFFER_SIZE);
}

/**
  * @brief  Setup the network interface
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
//  ip_addr_t ipaddr;
//  ip_addr_t netmask;
//  ip_addr_t gw;
//
//#if LWIP_DHCP
//  ip_addr_set_zero_ip4(&ipaddr);
//  ip_addr_set_zero_ip4(&netmask);
//  ip_addr_set_zero_ip4(&gw);
//#else
//
//  /* IP address default setting */
//  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
//  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
//
//#endif
//
//  /* add the network interface */
//  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
//
//  /*  Registers the default network interface */
//  netif_set_default(&gnetif);
//
////  ethernet_link_status_updated(&gnetif);
//
//#if LWIP_NETIF_LINK_CALLBACK
////  netif_set_link_callback(&gnetif, ethernet_link_status_updated);
//#endif
}

/**
* @brief  System Clock Configuration
*         The system Clock is configured as follow :
*            System Clock source            = PLL (HSE)
*            SYSCLK(Hz)                     = 400000000 (CPU Clock)
*            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
*            AHB Prescaler                  = 2
*            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
*            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
*            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
*            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
*            HSE Frequency(Hz)              = 25000000
*            PLL_M                          = 5
*            PLL_N                          = 160
*            PLL_P                          = 2
*            PLL_Q                          = 4
*            PLL_R                          = 2
*            PLL3_M                         = 5
*            PLL3_N                         = 96
*            PLL3_P                         = 2
*            PLL3_Q                         = 10
*            PLL3_R                         = 18
*            VDD(V)                         = 3.3
*            Flash Latency(WS)              = 4
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
  clocked below the maximum system frequency, to update the voltage scaling value
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  /* PLL1 for System Clock */
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* PLL3 for USB Clock */

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 96;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 10;
  PeriphClkInitStruct.PLL3.PLL3R = 18;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
    RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  /*activate CSI clock mondatory for I/O Compensation Cell*/
  __HAL_RCC_CSI_ENABLE() ;

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;

  /* Enables the I/O Compensation Cell */
  HAL_EnableCompensationCell();
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM6_Init(void)
//{
//
//  /* USER CODE BEGIN TIM6_Init 0 */
//
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
//
//  /* USER CODE END TIM6_Init 1 */
//  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 20000;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 20;
//  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM6_Init 2 */
//
//  /* USER CODE END TIM6_Init 2 */
//
//}

/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as WT for SDRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Normal Non Cacheable for SRAM1 */

  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  LTDC Clock Config for LCD DSI display.
  * @param  hltdc  LTDC Handle
  * @retval HAL_status
  */
 HAL_StatusTypeDef MX_LTDC_ClockConfig(LTDC_HandleTypeDef *hltdc)
{
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  /* LCD clock configuration */
  /* PLL3_VCO Input = HSE_VALUE/PLL3M = 5 Mhz */
  /* PLL3_VCO Output = PLL3_VCO Input * PLL3N = 480 Mhz */
  /* PLLLCDCLK = PLL3_VCO Output/PLL3R = 480/18 = 26.666 Mhz */
  /* LTDC clock frequency = PLLLCDCLK = 26.666 Mhz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 96;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 10;
  PeriphClkInitStruct.PLL3.PLL3R = 18;
  return HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}
/**
* @brief  CPU L1-Cache enable.
* @param  None
* @retval None
*/
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
/**
  * @brief Error Handler
  * @retval None
  */
static void Error_Handler(void)
{
  while(1) { ; } /* Blocking on error */
}

/**
  * @brief Error Handler
  * @retval None
  */
#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t * file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line
  * number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
  * line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

