/*
 * uc.h
 *
 *  Created on: Dec 25, 2012
 *      Author: davhak
 */

#ifndef UC_H_
#define UC_H_

#include "usb_dcd.h"
#include "usbd_def.h"
#include "usbd_conf.h"
#include "FreeRTOS.h"
#include "task.h"
extern xTaskHandle hUsbTask;

void uc_init();

void usbd_init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBD_DEVICE *pDevice,
               USBD_Class_cb_TypeDef *class_cb,
               USBD_Usr_cb_TypeDef *usr_cb);

void vUsbTask( void * pvParameters );

#endif /* UC_H_ */
