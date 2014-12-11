

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "led.h"
#include "PWM.h"
#include "PID.h"
#include "regler.h"
#include "uart_et.h"

#include "minimu9v2.h"
//#include "uart_dma.h"
#include "L3GD20.h"
#include "SendMetric.h"

void vApplicationTickHook( void ) { }

int main(void)
{
	SystemInit();
	//Board_InitLEDs();

	USARTinit();
	InitLEDs();
	//StartUART();

	//L3GD20_Init_Default();

	//*
	xTaskCreate(vLEDFlashTask ,
			(signed char * ) "vTaskLed1",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 1UL),
			NULL);
	//*/
	//*
	xTaskCreate(vMinIMU9v2Task,
			(signed char * ) "vIMUTask",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 1UL),
			NULL);
	//*/
	/*
	xTaskCreate(vPID,
			(signed char * ) "vPIDTask",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 1UL),
			NULL);
	//*/
	xTaskCreate(vRegler,
			(signed char * ) "vRegler",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 1UL),
			NULL);
	//*/
	//*
	xTaskCreate(vPWMTask,
			(signed char * ) "vPWMTask",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 1UL),
			NULL);
	//*/
	/*
	xTaskCreate(vSendMetric,
			(signed char * ) "toJava",
			configMINIMAL_STACK_SIZE,
			NULL,
			(tskIDLE_PRIORITY + 2UL),
			NULL);
	//*/



	vTaskStartScheduler();
	//L3GD20_Read1Byte(WHO_AM_I);
	//vLEDFlashTask("dfsf");

	while(1)
	{
		//L3GD20_Read1Byte(WHO_AM_I);
		//vTaskDelay(200);
		//LED_resetPin();
	}

}


