#include "minimu9v2.h"

#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "L3GD20.h"
#include "LSM303DLHC.h"
#include "led.h"
#include "uart_et.h"
#include "regler.h"
#include <stdio.h>

#include <stdio.h>

#define DT	 0.01

void IMU_Init(void )
{
	
	L3GD20_Init_Default();
	LSM303DLHC_Init_Default();
	regler_start = true;
}

void vMinIMU9v2Task(void *pvParameters)
{

	const portTickType xDelay = 10 / portTICK_RATE_MS; //zahl ist die periode in ms
	portTickType xLastWakeTime;

	float fv = 0.97; // filter value, //-----Current angle = 98% x (current angle + gyro rotation rate) + (2% * Accelerometer angle)

	volatile float CFangleX = 0;
	volatile float CFangleY = 0;

	IMU_Init();

	while(1)
    {
		xLastWakeTime = xTaskGetTickCount();

		L3GD20_updateAngle();
		LSM303DLHC_updateAngle();

		//-----Complementary filter;
		CFangleX = fv*(CFangleX+gyr.Xrate*DT)+ (1-fv)*acc.Xangle;
		CFangleY = fv*(CFangleY+gyr.Yrate*DT)+ (1-fv)*acc.Yangle;

		att.roll = CFangleX;
		att.pitch = CFangleY;
		att.Xrate = gyr.Xrate;
		att.Yrate = gyr.Yrate;
		att.Zrate = gyr.Zrate;

		//USART_send3values(att.roll, att.pitch, att.yaw);
		//USART_send3values(att.Xrate, att.Yrate, att.Zrate);

		vTaskDelayUntil( &xLastWakeTime, xDelay);
    }
}

