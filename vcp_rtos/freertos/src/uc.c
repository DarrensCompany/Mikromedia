/*
 * uc.c
 *
 *  Created on: Dec 25, 2012
 *      Author: davhak
 */

#include "uc.h"
//#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "misc.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

void uc_init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

}

void vUsbTask( void * pvParameters ) {

   usbd_init(&USB_OTG_dev,
	#ifdef USE_USB_OTG_HS
	  USB_OTG_HS_CORE_ID,
	#else
	  USB_OTG_FS_CORE_ID,
	#endif
	  &USR_desc,
	  &USBD_CDC_cb,
	  &USR_cb);

	  for( ;; ) {
		  vTaskSuspend(NULL);
	  }
}

void usbd_init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBD_DEVICE *pDevice,
               USBD_Class_cb_TypeDef *class_cb,
               USBD_Usr_cb_TypeDef *usr_cb)
{
	  USBD_Init(pdev, coreID, pDevice, class_cb, usr_cb);
}

