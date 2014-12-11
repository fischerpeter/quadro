
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include "uart_et.h"



//*--------------Declarations--------------
//void vLEDFLashTask (void *pvParameters);

void LED_setPin ();
void LED_resetPin ();

//*--------------Init Function--------------
void InitLEDs()
{

	//Clock Leitung enablen
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//PE.8 bis PE.15 als ausgang definieren
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

//*--------------FreeRTOS Task, periodic blinks--------------
void vLEDFlashTask (void *pvParameters)
{


	const portTickType xDelay = 250 / portTICK_RATE_MS;

	InitLEDs();

	while(1)
    {
		//GPIOE->BSRR = (1<<(8+led)); // geht mit M4 nicht...
		LED_setPin();
		//Delay(1000000);
		vTaskDelay(xDelay);

		//GPIOE->BRR = (1<<(8+led));  // geht mir M4 nicht...
		LED_resetPin();
		vTaskDelay(xDelay);


    }
}





//*--------------Software Delay--------------
void Delay (long i)
{
	volatile long iZ = i;
	while (iZ--);
}

void LED_setPin ()
{
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}
void LED_resetPin ()
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}
