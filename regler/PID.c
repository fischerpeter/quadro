#include "PID.h"

#include "FreeRTOS.h"
#include "task.h"

#include "minimu9v2.h"

#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

#include "uart_et.h"

#include "PWM.h"
#include "dac.h"

#include <stdio.h>



void vPID(void *pvParameters)
{
	const portTickType xDelay = 10 / portTICK_RATE_MS; //zahl ist die periode in ms
	portTickType xLastWakeTime;


	float soll = 0;

	float Kp = 0.01;	//Kp ... irgendwas
	float Kd = 0.01;	//Kd ... irgendwas
	float Ki = 0.01;

	float Ta = 0.01; 	//aufweckrate

	float p_anteil, i_anteil, d_anteil;
	float error, error_integral, error_old;

	error_old = 0;

	float pwm_x = 72;

	//int pwm_ch1_offset = 72;
	//pwm_ch1 = 0;

	//int p = 5;   // änderungsrate für P-Teil , in % vom gesamt auschlag


	DAC_Initialisation();

	while(1)
    {
		xLastWakeTime = xTaskGetTickCount();



		//----- Abweichung
		error = att.pitch-soll;

		//-----P Anteil
		p_anteil = error*Kp;

		//-----I-Anteil
		error_integral = error_integral + error;
		i_anteil = error_integral * Ki;

		//-----D-Anteil
		d_anteil = (error_old-error) * Kd;
		error_old = error;


		//---- Zusammensetzen
		pwm_x += p_anteil;
		pwm_x += i_anteil;
		//pwm_x += d_anteil;

		if (pwm_x > 100){
			pwm_x=100;
			error_integral -= error;
		}
		if (pwm_x < 0	 ) {
			pwm_x=0;
			error_integral -= error;
		}

		USART_send3values (att.pitch, error, pwm_ch1);
		pwm_ch1=(int) pwm_x;



		/*/-----P regelung
		if (error==0) pwm_ch1 = pwm_ch1_offset;
		if (error>0) {
			pwm_ch1 = pwm_ch1_offset-p;

			//DAC_set(pwm_ch1);
		}
		if (error<0) {
			pwm_ch1 = pwm_ch1_offset+p;

			//DAC_set(pwm_ch1);
		}

		//*/
		DAC_set(att.pitch);


		vTaskDelayUntil( &xLastWakeTime, xDelay);
    }
}
