/**
  ******************************************************************************
  * @file    cdc_ecm_tunnel.h
  * @author  Hashem Alnader
  * @brief   Header for cdc_ecm_tunnel.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CDC_ECM_TUNNEL_H
#define __CDC_ECM_TUNNEL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbh_cdc.h"
#include "usbh_cdc_ecm.h"
#include "usbd_cdc_ecm_if_template.h"
#include "usbd_cdc_ecm.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void CDC_ECM_Tunnel_Init(USBH_HandleTypeDef *USBH_Host, USBD_HandleTypeDef *USBD_Device);
void CDC_ECM_Tunnel_Buffer_Init(void);
void CDC_ECM_Tunnel_Process(USBH_HandleTypeDef *USBH_Host, USBD_HandleTypeDef *USBD_Device);

#ifdef __cplusplus
}
#endif

#endif /* __CDC_ECM_TUNNEL_H */
