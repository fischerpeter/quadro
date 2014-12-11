#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "SendMetric.h"
#include "minimu9v2.h"
#include "uart_et.h"
#include <stdio.h>



void vSendMetric(void *pvParameters)
{
	char str[32];

	const portTickType xDelay = 50 / portTICK_RATE_MS; //zahl ist die periode in ms

	while(1)
    {
		sprintf(str, "%d,%d,%d\r", (int) att.roll, (int)att.pitch, (int) att.yaw);
		USART_puts(str);
		vTaskDelay(xDelay);
    }
}
