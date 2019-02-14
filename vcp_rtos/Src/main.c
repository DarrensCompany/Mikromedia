/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**
*****************************************************************************
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "uc.h"
#include "defs.h"

#include <stdio.h>
#include "usbd_cdc_vcp.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

#define _USB_TASK_PRIORITY     	       (tskIDLE_PRIORITY + 1)

const uint16_t LEDS = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
const uint16_t LED[4] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};

xTaskHandle hUsbTask;
xSemaphoreHandle _COM_TransmitMutex;
void vUsbTask( void * pvParameters );
void usb_io_demo();
void start_rtos();

int main() {

    uc_init();

    /*
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = LEDS;
    GPIO_Init(GPIOD, &gpio);
    GPIO_SetBits(GPIOD, LEDS);
    */

    _COM_TransmitMutex = xSemaphoreCreateMutex();
	start_rtos();
    return 0;
}

void start_rtos()
{
  xTaskCreate(vUsbTask, (signed char *)"_USB_Task", configMINIMAL_STACK_SIZE,NULL, _USB_TASK_PRIORITY, &hUsbTask);
  xTaskCreate(usb_io_demo, NULL, configMINIMAL_STACK_SIZE,NULL, _USB_TASK_PRIORITY, &hUsbTask);
  vTaskStartScheduler(); // This should never return.

  for(;;);
}

void usb_io_demo()
{
	while(1)
	{
		if(usb_cdc_kbhit()){
			char c, buffer_out[32];
			c = usb_cdc_getc();
            sprintf(buffer_out,"%c is pressed/FreeRTOS\r\n",c);
			usb_cdc_printf(buffer_out);
            /*
			switch(c){
				case '3':
					STM_EVAL_LEDToggle(LED3);
					sprintf(buffer_out,"LED%c = %u\r\n",c,GPIO_ReadInputDataBit(GPIOD,LED3_PIN));
					usb_cdc_printf(buffer_out);
					break;
				case '4':
					STM_EVAL_LEDToggle(LED4);
					sprintf(buffer_out,"LED%c = %u\r\n",c,GPIO_ReadInputDataBit(GPIOD,LED4_PIN));
					usb_cdc_printf(buffer_out);
					break;
				case '5':
					STM_EVAL_LEDToggle(LED5);
					sprintf(buffer_out,"LED%c = %u\r\n",c,GPIO_ReadInputDataBit(GPIOD,LED5_PIN));
					usb_cdc_printf(buffer_out);
					break;
				case '6':
					STM_EVAL_LEDToggle(LED6);
					sprintf(buffer_out,"LED%c = %u\r\n",c,GPIO_ReadInputDataBit(GPIOD,LED6_PIN));
					usb_cdc_printf(buffer_out);
					break;
			}
			*/
		}
	}
}

void vApplicationMallocFailedHook( void )
{
	volatile unsigned long ul;

	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	//for( ;; );

	for (;;)
	{
		//STM_EVAL_LEDToggle(LED5);
		//for(ul = 0; ul < 100000; ++ul);
	}
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	volatile unsigned long ul;

	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */

	for (;;)
	{
		//STM_EVAL_LEDToggle(LED5);
		//for(ul = 0; ul < 100000; ++ul);
	}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	volatile unsigned long ul;

	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	// for( ;; );

	for (;;)
	{
		//STM_EVAL_LEDToggle(LED6);
		//for(ul = 0; ul < 100000; ++ul);
	}
}
