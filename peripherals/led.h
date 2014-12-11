#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

void InitLEDs();
void vLEDFlashTask (void *pvParameters);
void LED_setPin ();
void LED_resetPin ();
